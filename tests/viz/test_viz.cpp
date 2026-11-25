// 显示配置（M4-08d，viz.config）：
//   - .terviz 的解析：各窗口的参数、相对场景路径、默认值；
//   - 往返：写出再读入一致；不认识的参数、窗口字段、窗口类型与顶层字段给出警告并原样保留（设计稿 4.1 节）；
//   - 错误：格式、版本过高、缺少话题、参数类型或取值不对、窗口标识重复；
//   - 导出器注册表：内置 terviz 与 rviz；可以注册新的导出器；参数清单的默认值与校验；
//   - .rviz：固定坐标系、网格、TF、LaserScan（best effort）、Image、压缩图像的基础话题；没有导出的部分列在注释中；
//     Python 的 yaml 能读入，结构与 RViz2 的格式一致（没有 python3-yaml 时跳过这一项）。
// 用法：test_viz <临时目录>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include "terra/viz/exporter.hpp"
#include "terra/viz/viz_config.hpp"

namespace {

namespace fs = std::filesystem;
namespace vz = terra::viz;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

bool anyHas(const std::vector<std::string>& list, const std::string& part) {
    for (const auto& s : list) {
        if (has(s, part)) return true;
    }
    return false;
}

const char* kFull = R"({
  "format": "terra.viz", "version": 1, "name": "导航调试",
  "scene": "../scenes/tb4.terra-scene.json",
  "windows": [
    {"type": "topics", "params": {"filter": "oakd"}},
    {"type": "message", "topic": "/odom", "params": {"pageSize": 50}},
    {"type": "plot", "title": "角速度", "params": {"seconds": 20, "timeBase": "header", "yRange": [-2, 2],
      "series": [{"source": "ros2", "topic": "/imu", "field": "angular_velocity.z", "label": "陀螺 z", "color": [1, 0, 0, 1], "scale": 2},
                 {"source": "sim", "sensor": "tb4/imu", "field": "gyro.z", "futureKnob": 7}]},
     "placement": {"position": [0.6, 0.1], "size": [0.3, 0.25], "docked": false}},
    {"type": "image", "topic": "/oakd/stereo/image_raw", "params": {"depthRange": [0.2, 6], "colormap": "gray", "fit": "actual", "gamma": 2.2}},
    {"type": "image", "id": "rgb", "topic": "/cam/image/compressed"},
    {"type": "tf", "params": {"root": "odom"}, "pinned": true},
    {"type": "scan3d", "topic": "/points", "params": {"decay": 1}},
    {"type": "message", "topic": "/scan"}
  ],
  "overlays": [{"type": "axes"}],
  "extensions": {"vendor.x": {"a": 1}},
  "createdBy": "terra 9.0"
})";

}  // namespace

int main(int argc, char** argv) {
    const fs::path dir = fs::absolute(argc > 1 ? argv[1] : "viz_tmp").lexically_normal();
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir / "displays", ec);
    const std::string file = (dir / "displays" / "nav.terviz").string();

    // 解析
    auto r = vz::parseViz(kFull, file);
    for (const auto& e : r.errors) std::fprintf(stderr, "  错误 %s\n", e.c_str());
    check(r.config.has_value(), "完整的显示配置可以解析");
    if (!r.config) return 1;
    const auto& c = *r.config;
    check(c.name == "导航调试" && c.scene == (dir / "scenes" / "tb4.terra-scene.json").string(), "名字与场景（相对路径按文件所在目录展开）");
    check(c.windows.size() == 8, "8 个窗口（含 1 个不认识的类型）");
    if (c.windows.size() == 8) {
        check(c.windows[0].type == vz::WindowType::Topics && c.windows[0].filter == "oakd", "话题表：筛选文字");
        check(c.windows[1].pageSize == 50, "消息：每页条数");
        const auto& p = c.windows[2];
        check(p.type == vz::WindowType::Plot && p.seconds == 20 && p.timeBase == "header" && p.yRange && (*p.yRange)[1] == 2 && p.series.size() == 2,
              "曲线：时长、时间轴、纵轴范围、两条曲线");
        if (p.series.size() == 2) {
            check(p.series[0].topic == "/imu" && p.series[0].label == "陀螺 z" && p.series[0].color && p.series[0].scale == 2,
                  "曲线 1：话题、标签、颜色、比例");
            check(p.series[1].source == "sim" && p.series[1].sensor == "tb4/imu" && has(p.series[1].extraJson, "futureKnob"),
                  "曲线 2：仿真来源；不认识的参数保留");
        }
        check(p.placement && p.placement->position[0] == 0.6 && p.placement->size[1] == 0.25 && !p.placement->docked, "窗口位置与大小");
        check(c.windows[3].colormap == "gray" && c.windows[3].fit == "actual" && c.windows[3].depthRange[1] == 6 && has(c.windows[3].extraParams, "gamma"),
              "图像：深度范围、色表、缩放；不认识的参数保留");
        check(c.windows[4].colormap == "turbo" && c.windows[4].depthRange[0] == 0.0, "没写的参数取默认值");
        check(c.windows[5].tfRoot == "odom" && has(c.windows[5].extraFields, "pinned"), "TF：根坐标系；窗口一级不认识的字段保留");
        check(c.windows[6].type == vz::WindowType::Unknown && c.windows[6].typeName == "scan3d", "不认识的窗口类型：保留");
    }
    check(anyHas(r.warnings, "futureKnob") && anyHas(r.warnings, "scan3d") && anyHas(r.warnings, "createdBy") && anyHas(r.warnings, "gamma"),
          "不认识的内容都有警告");

    // 往返
    check(!vz::saveViz(c, file), "保存");
    std::string written;
    {
        std::ifstream in(file);
        written.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    check(has(written, "\"scene\": \"../scenes/tb4.terra-scene.json\""), "场景写为相对路径");
    check(has(written, "futureKnob") && has(written, "\"gamma\"") && has(written, "\"pinned\"") && has(written, "\"scan3d\"") &&
              has(written, "createdBy") && has(written, "vendor.x") && has(written, "\"axes\""),
          "不认识的内容与预留字段原样写回");
    const auto back = vz::readViz(file);
    check(back.config && back.config->windows.size() == 8 && back.config->scene == c.scene && vz::vizToJson(*back.config, file) == written,
          "读回后再写出逐字节相同");

    // 错误
    const auto expectError = [&](const std::string& text, const std::string& part, const std::string& what) {
        const auto e = vz::parseViz(text, file);
        check(!e.config && anyHas(e.errors, part), what + "（应报“" + part + "”）");
    };
    expectError(R"({"format": "x", "version": 1})", "terra.viz", "格式不对");
    expectError(R"({"format": "terra.viz", "version": 9})", "高于本程序支持的版本", "版本过高");
    expectError(R"({"format": "terra.viz", "version": 1, "windows": [{"type": "message"}]})", "需要 topic", "消息窗口缺少话题");
    expectError(R"({"format": "terra.viz", "version": 1, "windows": [{"type": "image", "topic": "/a", "params": {"colormap": "jet"}}]})", "不是可用的值",
                "色表不可用");
    expectError(R"({"format": "terra.viz", "version": 1, "windows": [{"type": "plot", "params": {"series": [{"topic": "/a"}]}}]})", "缺少 field",
                "曲线缺少字段");
    expectError(R"({"format": "terra.viz", "version": 1, "windows": [{"type": "plot", "params": {"yRange": [3, 1]}}]})", "下限应小于上限",
                "纵轴范围颠倒");
    expectError(R"({"format": "terra.viz", "version": 1, "windows": [{"type": "message", "topic": "/a"}, {"type": "message", "topic": "/a"}]})", "重复",
                "窗口标识重复");
    expectError(R"({"format": "terra.viz", "version": 1, "windows": [{"type": "message", "topic": "/a", "params": {"pageSize": "x"}}]})", "整数",
                "参数类型不对");

    // 导出器注册表
    check(vz::findExporter("terviz") && vz::findExporter("rviz") && vz::exporters().size() >= 2, "内置导出器：terviz 与 rviz");
    class Dummy final : public vz::Exporter {
    public:
        std::string id() const override { return "dummy"; }
        vz::Text name() const override { return {"测试", "Test"}; }
        std::string suffix() const override { return ".dummy"; }
        std::vector<vz::ParameterSpec> parameters(const vz::VizConfig&, const vz::ExportContext&) const override { return {}; }
        std::vector<std::string> notExported(const vz::VizConfig&, const vz::ExportContext&) const override { return {}; }
        vz::ExportResult convert(const vz::VizConfig& config, const vz::ParameterValues&, const vz::ExportContext&, const std::string&) const override {
            vz::ExportResult x;
            x.ok = true;
            x.text = config.name;
            return x;
        }
    };
    vz::registerExporter(std::make_unique<Dummy>());
    check(vz::findExporter("dummy") != nullptr && vz::exporters().back()->id() == "dummy", "可以注册新的导出器（以后的格式）");

    // .rviz
    const vz::Exporter* rviz = vz::findExporter("rviz");
    vz::ExportContext ctx;
    ctx.frames = {"odom", "base_link", "rplidar_link"};
    ctx.topicTypes = {{"/odom", "nav_msgs/msg/Odometry"},
                      {"/imu", "sensor_msgs/msg/Imu"},
                      {"/scan", "sensor_msgs/msg/LaserScan"},
                      {"/oakd/stereo/image_raw", "sensor_msgs/msg/Image"},
                      {"/cam/image/compressed", "sensor_msgs/msg/CompressedImage"}};
    ctx.bestEffort = {{"/scan", true}};
    const auto specs = rviz->parameters(c, ctx);
    const auto defaults = vz::withDefaults(specs, {});
    check(specs.size() == 4 && defaults.at("fixedFrame") == "odom" && defaults.at("grid") == "true" && defaults.at("frameRate") == "30",
          "rviz 的参数清单与默认值（固定坐标系取 TF 树的根）");
    check(vz::validateParameters(specs, {{"frameRate", "500"}}).size() == 1 && vz::validateParameters(specs, {{"grid", "maybe"}}).size() == 1 &&
              vz::validateParameters(specs, {{"nope", "1"}}).size() == 1 && vz::validateParameters(specs, {{"viewDistance", "5"}}).empty(),
          "参数校验：范围、类型、不认识的参数");
    const std::string rvizFile = (dir / "nav.rviz").string();
    const auto out = vz::exportToFile(*rviz, c, {{"viewDistance", "5"}}, ctx, rvizFile);
    check(out.ok, "导出 .rviz：" + out.error);
    const std::string& y = out.text;
    check(has(y, "Fixed Frame: \"odom\"") && has(y, "rviz_default_plugins/Grid") && has(y, "rviz_default_plugins/TF") && has(y, "Distance: 5"),
          "固定坐标系、网格、TF、视距");
    check(has(y, "rviz_default_plugins/LaserScan") && has(y, "Value: \"/scan\"") && has(y, "Reliability Policy: Best Effort"),
          "/scan 的消息窗口导出为 LaserScan（best effort）");
    check(has(y, "rviz_default_plugins/Odometry") && has(y, "Value: \"/oakd/stereo/image_raw\"") && has(y, "Value: \"/cam/image\"") &&
              has(y, "Transport Hint: compressed"),
          "Odometry、图像；压缩图像写基础话题与传输方式");
    check(anyHas(out.notExported, "角速度") && anyHas(out.notExported, "tb4/imu") && anyHas(out.notExported, "话题表") &&
              anyHas(out.notExported, "scan3d") && anyHas(out.notExported, "overlays") && !anyHas(out.notExported, "/odom"),
          "没有导出的部分：曲线、仿真来源、话题表、不认识的窗口、视口叠加（/odom 有 Odometry 显示，不列入）");
    check(has(y, "# 没有导出的部分") && has(y, "#   - 曲线“角速度”"), "没有导出的部分写在文件开头的注释中");
    const std::string script = "python3 -I -c \"import sys, yaml\nd = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))\n"
                               "vm = d['Visualization Manager']\nassert vm['Global Options']['Fixed Frame'] == 'odom'\n"
                               "classes = [x['Class'] for x in vm['Displays']]\nassert 'rviz_default_plugins/LaserScan' in classes, classes\n"
                               "assert vm['Views']['Current']['Class'] == 'rviz_default_plugins/Orbit'\nprint('yaml ok', len(classes))\" ";
    if (std::system("python3 -I -c \"import yaml\" 2>/dev/null") == 0) {
        check(std::system((script + "'" + rvizFile + "'").c_str()) == 0, "Python 的 yaml 能读入导出的 .rviz，结构正确");
    } else {
        std::printf("INFO 没有 python3-yaml，跳过 .rviz 的 YAML 检查\n");
    }

    // .terviz 导出再导入
    const std::string tervizFile = (dir / "export" / "nav.terviz").string();
    const auto t = vz::exportToFile(*vz::findExporter("terviz"), c, {}, {}, tervizFile);
    const auto again = vz::readViz(tervizFile);
    check(t.ok && t.notExported.empty() && again.config && again.config->windows.size() == 8 && again.config->scene == c.scene,
          ".terviz 导出再导入（场景路径按新位置重写）");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
