// 设置模块测试（M1-14，D-026）。不需要窗口或图形环境，渲染关闭时也运行。
// 覆盖：默认值；JSON 往返（全部字段）；文件缺失、语法错误、顶层非对象、无法读取；逐字段类型与范围校验（坏字段用默认值，
// 其他字段照常读取）；未知字段与较新版本的警告；按键名称、保留键、重复按键；按键分配（抢占其他操作的键）；
// 原子保存、自动建目录、保存失败不破坏原文件；坏文件备份；XDG 默认路径。
//
// 用法：test_settings <临时目录>（测试开始时清空该目录）

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "terra/settings/settings.hpp"

namespace {

namespace fs = std::filesystem;
namespace in = terra::input;
namespace st = terra::settings;
using st::Command;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string joined(const std::vector<std::string>& lines) {
    std::string out;
    for (const auto& l : lines) out += "  " + l + "\n";
    return out;
}

bool contains(const std::vector<std::string>& lines, const std::string& part) {
    for (const auto& l : lines) {
        if (l.find(part) != std::string::npos) return true;
    }
    return false;
}

void writeFile(const fs::path& p, const std::string& text) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out << text;
}

std::string readFile(const fs::path& p) {
    std::ifstream file(p, std::ios::binary);
    std::ostringstream s;
    s << file.rdbuf();
    return s.str();
}

// 全部字段都不等于默认值的设置。
st::Settings everythingChanged() {
    st::Settings s;
    s.ui.language = "en";
    s.ui.uiScale = 1.75;
    s.ui.showOutliner = false;
    s.ui.showDetails = false;
    s.ui.showLog = false;
    s.ui.showStats = false;
    s.ui.layout = "[Window][大纲###Outliner]\nPos=0,19\nSize=272,700\n\n[Docking][Data]\nDockSpace ID=0x1 Window=0x2 Pos=0,19\n";
    s.display.mode = st::WindowMode::ExclusiveFullscreen;
    s.display.monitor = 1;
    s.display.videoMode = st::VideoModeSetting{2560, 1440, 144};
    s.display.vsync = false;
    s.display.windowWidth = 1280;
    s.display.windowHeight = 720;
    s.camera.lookDegreesPerPixel = 0.2;
    s.camera.speed = 7.5;
    s.camera.boost = 4.0;
    s.camera.wheelStep = 1.25;
    s.camera.panMetersPerPixel = 0.02;
    s.camera.fovDegrees = 75.0;
    s.render.sunAzimuthDegrees = 200.0;
    s.render.sunElevationDegrees = 30.0;
    s.render.sunIntensity = 5.0;
    s.render.ambientIntensity = 1.2;
    s.render.exposureEv = -1.5;
    s.render.toneMapper = st::ToneMapperSetting::Aces;
    s.render.msaaSamples = 8;
    s.render.shadows = false;
    s.render.grid = false;
    s.render.environment = st::EnvironmentSetting::Studio;
    s.render.environmentIntensity = 1.5;
    s.render.environmentRotationDegrees = 90.0;
    s.render.environmentBackground = false;
    s.render.backgroundBlur = 0.6;
    s.render.sunFromEnvironment = false;
    s.render.assetArmOcclusion = false;
    s.keys[Command::CameraForward] = {in::Key::I, in::Key::Unknown};
    s.keys[Command::CameraBack] = {in::Key::K, in::Key::Down};
    s.keys[Command::CameraBoost] = {in::Key::LeftAlt, in::Key::Unknown};
    s.keys[Command::AppFullscreen] = {in::Key::F10, in::Key::F11};
    s.keys[Command::SimPlayPause] = {in::Key::P, in::Key::Unknown};
    s.keys[Command::SimStep] = {in::Key::N, in::Key::Unknown};
    s.keys[Command::SimReset] = {in::Key::Backspace, in::Key::Unknown};
    s.ui.recentScenes = {"/data/scenes/b.terra-scene.json", "/data/scenes/a.terra-scene.json"};
    return s;
}

void testDefaults() {
    const st::Settings s;
    check(s.ui.language == "zh" && !s.ui.uiScale && s.ui.showOutliner && s.ui.layout.empty(), "界面默认：中文、自动缩放、默认布局");
    check(s.display.mode == st::WindowMode::Windowed && s.display.monitor == 0 && !s.display.videoMode && s.display.vsync &&
              s.display.windowWidth == 1600 && s.display.windowHeight == 900,
          "显示默认：窗口 1600×900、垂直同步");
    check(s.camera.lookDegreesPerPixel == 0.15 && s.camera.speed == 3.0 && s.camera.boost == 3.0 && s.camera.wheelStep == 0.5 &&
              s.camera.panMetersPerPixel == 0.01 && s.camera.fovDegrees == 60.0,
          "相机默认值与 M1-07 相同");
    check(s.render.sunAzimuthDegrees == 150.0 && s.render.sunElevationDegrees == 50.0 && s.render.sunIntensity == 3.0 &&
              s.render.ambientIntensity == 0.6 && s.render.exposureEv == 0.0 &&
              s.render.toneMapper == st::ToneMapperSetting::AgX && s.render.msaaSamples == 4 && s.render.shadows &&
              s.render.grid,
          "渲染默认：太阳 150°/50°、强度 3、环境 0.6、0 EV、AgX、4× MSAA、阴影与网格开启（M1-08 设计稿）");
    check(s.render.environment == st::EnvironmentSetting::Hemisphere && s.render.environmentIntensity == 0.6 &&
              s.render.environmentRotationDegrees == 0.0 && s.render.environmentBackground && s.render.backgroundBlur == 0.25 &&
              s.render.sunFromEnvironment && s.render.assetArmOcclusion,
          "环境默认：半球光（2026-11-24 起不用环境贴图）、强度 0.6、不旋转、显示背景、模糊 0.25（M1-08b）");
    // 版本 2 → 3：旧默认的室外贴图改为半球光；版本 3 文件中的室外贴图是用户的选择，保持
    {
        const auto v2 = st::fromJson(R"({"version": 2, "render": {"environment": "outdoor"}})");
        check(v2.settings.render.environment == st::EnvironmentSetting::Hemisphere && contains(v2.warnings, "默认环境改为半球光"),
              "版本 2 的室外贴图迁移为半球光\n" + joined(v2.warnings));
        const auto studio = st::fromJson(R"({"version": 2, "render": {"environment": "studio"}})");
        check(studio.settings.render.environment == st::EnvironmentSetting::Studio && studio.warnings.empty(), "版本 2 的摄影棚贴图保持");
        const auto v3 = st::fromJson(R"({"version": 3, "render": {"environment": "outdoor"}})");
        check(v3.settings.render.environment == st::EnvironmentSetting::Outdoor && v3.warnings.empty(), "版本 3 的室外贴图保持");
    }
    const st::KeySlots forward{in::Key::W, in::Key::Up};
    const st::KeySlots down{in::Key::Q, in::Key::Unknown};
    const st::KeySlots full{in::Key::F11, in::Key::Unknown};
    check(s.keys[Command::CameraForward] == forward && s.keys[Command::CameraDown] == down &&
              s.keys[Command::AppFullscreen] == full,
          "默认按键：前进 W/↑、下降 Q、全屏 F11");
    for (std::size_t c = 0; c < st::kCommandCount; ++c) {
        const auto command = static_cast<Command>(c);
        const auto parsed = st::commandFromName(st::commandName(command));
        check(parsed && *parsed == command, std::string{"操作名称往返："} + st::commandName(command));
    }
    check(!st::commandFromName("camera.nope"), "未知操作名称");
}

void testRoundTrip() {
    const st::Settings s = everythingChanged();
    check(s != st::Settings{}, "测试数据与默认值不同");
    const std::string text = st::toJson(s);
    const st::ParseResult back = st::fromJson(text);
    check(!back.error, "往返：解析成功");
    check(back.warnings.empty(), "往返：没有警告\n" + joined(back.warnings));
    check(back.settings == s, "往返：全部字段相同\n" + text);
    check(st::toJson(back.settings) == text, "往返：再次序列化逐字节相同");

    const st::ParseResult defaults = st::fromJson(st::toJson(st::Settings{}));
    check(!defaults.error && defaults.warnings.empty() && defaults.settings == st::Settings{}, "默认设置往返");
    check(text.find("\"uiScale\": 1.75") != std::string::npos && st::toJson(st::Settings{}).find("\"uiScale\": \"auto\"") != std::string::npos,
          "界面缩放：数字或 \"auto\"");
    check(text.find("\"camera.forward\": [\n      \"I\",\n      \"\"\n    ]") != std::string::npos,
          "按键写成键名数组，空槽位为空字符串");
    check(text.rfind("{\n  \"version\": 3,", 0) == 0 && text.back() == '\n', "以 version 开头、以换行结尾（2026-11-24 起为版本 3）");
}

void testPartialAndInvalidFields() {
    // 只给出部分字段：其余保持默认，没有警告
    {
        const auto r = st::fromJson(R"({"version": 1, "camera": {"speed": 10}})");
        st::Settings expected;
        expected.camera.speed = 10;
        check(!r.error && r.warnings.empty() && r.settings == expected, "部分字段：其余为默认值\n" + joined(r.warnings));
    }
    // 坏字段用默认值，同组的好字段照常读取
    {
        const auto r = st::fromJson(R"({
          "version": 1,
          "interface": {"language": "fr", "uiScale": 9, "panels": {"log": "yes", "stats": false}, "layout": 5},
          "display": {"mode": "fullscreen", "monitor": -1, "videoMode": {"width": 1920, "height": 0, "refreshRate": 60},
                      "vsync": 1, "windowSize": {"width": 100, "height": 800}},
          "camera": {"speed": 1000, "boost": "fast", "wheelStep": 2, "fovDegrees": 10, "lookDegreesPerPixel": 0.3}
        })");
        check(!r.error, "坏字段不影响整体解析");
        const auto& s = r.settings;
        check(s.ui.language == "zh" && contains(r.warnings, "interface.language"), "language 非法 → 默认 zh");
        check(!s.ui.uiScale && contains(r.warnings, "interface.uiScale = 9 超出范围"), "uiScale 超出范围 → 自动");
        check(s.ui.showLog && contains(r.warnings, "interface.panels.log"), "panels.log 类型错误 → 默认 true");
        check(!s.ui.showStats, "panels.stats 正常读取");
        check(s.ui.layout.empty() && contains(r.warnings, "interface.layout"), "layout 类型错误 → 默认布局");
        check(s.display.mode == st::WindowMode::Windowed && contains(r.warnings, "display.mode"), "mode 非法 → 窗口");
        check(s.display.monitor == 0 && contains(r.warnings, "display.monitor = -1 超出范围"), "monitor 越界 → 0");
        check(!s.display.videoMode && contains(r.warnings, "display.videoMode"), "videoMode 高度为 0 → 显示器当前模式");
        check(s.display.vsync && contains(r.warnings, "display.vsync"), "vsync 不是布尔值 → 默认 true");
        check(s.display.windowWidth == 1600 && s.display.windowHeight == 800 && contains(r.warnings, "display.windowSize.width"),
              "窗口宽度过小 → 默认 1600；高度 800 照常读取");
        check(s.camera.speed == 3.0 && contains(r.warnings, "camera.speed = 1000 超出范围 [0.1, 100]，已使用默认值 3"),
              "speed 超出范围 → 默认 3，警告含范围与默认值");
        check(s.camera.boost == 3.0 && contains(r.warnings, "camera.boost：应为数字"), "boost 类型错误 → 默认 3");
        check(s.camera.wheelStep == 2.0 && s.camera.lookDegreesPerPixel == 0.3, "合法的相机字段照常读取");
        check(s.camera.fovDegrees == 60.0 && contains(r.warnings, "camera.fovDegrees"), "fov 过小 → 默认 60");
        check(r.warnings.size() == 12, "共 12 条警告（实际 " + std::to_string(r.warnings.size()) + "）\n" + joined(r.warnings));
    }
    // 渲染组（M1-08）
    {
        const auto r = st::fromJson(R"({"version": 1, "render": {"toneMapper": "filmic", "msaa": 3, "sunElevationDegrees": 0,
                                         "exposureEv": 2.5, "toneMapper2": 1}})");
        const auto& rs = r.settings.render;
        check(rs.toneMapper == st::ToneMapperSetting::AgX && contains(r.warnings, "render.toneMapper"), "未知色调映射 → AgX");
        check(rs.msaaSamples == 4 && contains(r.warnings, "render.msaa"), "MSAA 3 → 默认 4");
        check(rs.sunElevationDegrees == 50.0 && contains(r.warnings, "render.sunElevationDegrees = 0 超出范围"), "高度角 0 → 默认 50");
        check(rs.exposureEv == 2.5, "合法的曝光照常读取");
        check(contains(r.warnings, "未知字段 render.toneMapper2") && r.warnings.size() == 4,
              "渲染组共 4 条警告（实际 " + std::to_string(r.warnings.size()) + "）\n" + joined(r.warnings));
        const auto e = st::fromJson(R"({"version": 1, "render": {"environment": "forest", "environmentIntensity": 9,
                                         "backgroundBlur": 0.5}})");
        check(e.settings.render.environment == st::EnvironmentSetting::Hemisphere && contains(e.warnings, "render.environment") &&
                  e.settings.render.environmentIntensity == 0.6 && contains(e.warnings, "render.environmentIntensity = 9") &&
                  e.settings.render.backgroundBlur == 0.5 && e.warnings.size() == 2,
              "环境：未知贴图名与超出范围的强度回退默认值，模糊照常读取\n" + joined(e.warnings));
        for (const auto env : {st::EnvironmentSetting::Hemisphere, st::EnvironmentSetting::Outdoor, st::EnvironmentSetting::Studio}) {
            st::Settings one;
            one.render.environment = env;
            check(st::fromJson(st::toJson(one)).settings == one, std::string{"环境往返："} + st::environmentName(env));
        }
        for (const auto m : {st::ToneMapperSetting::AgX, st::ToneMapperSetting::PbrNeutral, st::ToneMapperSetting::Aces,
                             st::ToneMapperSetting::None}) {
            st::Settings one;
            one.render.toneMapper = m;
            check(st::fromJson(st::toJson(one)).settings == one, std::string{"色调映射往返："} + st::toneMapperName(m));
        }
    }
    // 整组类型错误、未知字段、版本
    {
        const auto r = st::fromJson(R"({"camera": [1, 2], "display": {"colour": "blue"}, "extra": true})");
        check(!r.error && r.settings == st::Settings{}, "整组类型错误与未知字段：其余为默认");
        check(contains(r.warnings, "camera：应为对象") && contains(r.warnings, "未知字段 display.colour") &&
                  contains(r.warnings, "未知字段 extra") && contains(r.warnings, "缺少 version"),
              "警告：整组类型、未知字段、缺少版本\n" + joined(r.warnings));
        const auto newer = st::fromJson(R"({"version": 7, "camera": {"speed": 5}})");
        check(!newer.error && newer.settings.camera.speed == 5.0 && contains(newer.warnings, "文件版本 7"),
              "较新的版本：警告并读取认识的字段");
    }
    // 整体无法使用
    {
        const auto syntax = st::fromJson("{\n  \"camera\": {\"speed\": 5,}\n}");
        check(syntax.error && syntax.error->find("JSON 语法错误") != std::string::npos && syntax.settings == st::Settings{},
              "语法错误：整体失败，默认设置");
        check(syntax.error && syntax.error->find("line 2") != std::string::npos, "语法错误信息含行号：" + syntax.error.value_or(""));
        check(st::fromJson("[1, 2]").error.has_value(), "顶层是数组：失败");
        check(st::fromJson("").error.has_value(), "空文本：失败");
        check(st::fromJson(R"({"camera": {"speed": 1e999}})").error.has_value(), "数值溢出：失败而不是崩溃");
    }
}

void testKeys() {
    // 文件中的按键
    {
        const auto r = st::fromJson(R"({"version": 1, "keys": {
            "camera.forward": ["I"],
            "camera.back": ["K", null],
            "camera.reset": ["Nope"],
            "camera.left": ["Escape", "A"],
            "camera.up": "E",
            "camera.dance": ["Z"],
            "app.fullscreen": ["F10", ""]}})");
        const auto& k = r.settings.keys;
        check(k[Command::CameraForward] == st::KeySlots{in::Key::I, in::Key::Unknown}, "只给一个键：备用槽位为空");
        check(k[Command::CameraBack] == st::KeySlots{in::Key::K, in::Key::Unknown}, "null 表示空槽位");
        check(k[Command::AppFullscreen] == st::KeySlots{in::Key::F10, in::Key::Unknown}, "空字符串表示空槽位");
        check(k[Command::CameraReset] == st::KeySlots{in::Key::F, in::Key::Unknown} && contains(r.warnings, "未知键名 \"Nope\""),
              "未知键名 → 该操作用默认按键");
        check(k[Command::CameraLeft] == st::KeySlots{in::Key::A, in::Key::Left} && contains(r.warnings, "保留键"),
              "保留键 Esc → 该操作用默认按键");
        check(k[Command::CameraUp] == st::KeySlots{in::Key::E, in::Key::Unknown} && contains(r.warnings, "keys.camera.up：应为"),
              "不是数组 → 默认按键");
        check(contains(r.warnings, "未知字段 keys.camera.dance"), "未知操作：警告");
        check(r.warnings.size() == 4, "共 4 条警告（实际 " + std::to_string(r.warnings.size()) + "）\n" + joined(r.warnings));
    }
    // 重复按键：按操作顺序保留第一次出现
    {
        const auto r = st::fromJson(R"({"keys": {"camera.reset": ["W"], "camera.right": ["D", "D"]}})");
        const auto& k = r.settings.keys;
        check(k[Command::CameraForward][0] == in::Key::W && k[Command::CameraReset][0] == in::Key::Unknown,
              "W 同时用于前进（默认）与复位：保留前进，复位清空");
        check(k[Command::CameraRight] == st::KeySlots{in::Key::D, in::Key::Unknown}, "同一操作两个槽位相同：清空第二个");
        check(contains(r.warnings, "按键 W 同时用于 camera.forward 与 camera.reset"), "重复按键的警告\n" + joined(r.warnings));
    }
    // 保留键
    check(st::isReservedKey(in::Key::Escape) && st::isReservedKey(in::Key::LeftControl) &&
              st::isReservedKey(in::Key::RightControl) && st::isReservedKey(in::Key::Unknown),
          "保留键：Esc、左右 Ctrl、Unknown");
    check(!st::isReservedKey(in::Key::LeftShift) && !st::isReservedKey(in::Key::F11), "Shift、F11 不是保留键");

    // 分配：抢占其他操作的键
    st::KeyMap keys;
    auto from = keys.assign(Command::CameraReset, 0, in::Key::W);
    check(from && *from == Command::CameraForward, "把 W 分配给复位：从前进移走");
    check(keys[Command::CameraReset][0] == in::Key::W && keys[Command::CameraForward] == st::KeySlots{in::Key::Unknown, in::Key::Up},
          "分配后：复位 = W，前进只剩 ↑");
    from = keys.assign(Command::CameraReset, 1, in::Key::W);
    check(from && *from == Command::CameraReset && keys[Command::CameraReset] == st::KeySlots{in::Key::Unknown, in::Key::W},
          "同一操作换槽位：从原槽位移走");
    from = keys.assign(Command::CameraUp, 0, in::Key::Home);
    check(!from && keys[Command::CameraUp][0] == in::Key::Home, "未被占用的键：直接分配");
    const st::KeyMap before = keys;
    check(!keys.assign(Command::CameraUp, 0, in::Key::Escape) && keys == before, "保留键：不做修改");
    check(!keys.assign(Command::CameraUp, 2, in::Key::Z) && keys == before, "非法槽位：不做修改");
    (void)keys.assign(Command::CameraUp, 0, in::Key::Unknown);
    check(keys[Command::CameraUp][0] == in::Key::Unknown, "分配 Unknown 等同于清除");
    check(keys.find(in::Key::W) && *keys.find(in::Key::W) == Command::CameraReset && !keys.find(in::Key::Z), "find 查找使用者");
}

void testFiles(const fs::path& dir) {
    const fs::path file = dir / "nested" / "eai-terra" / "settings.json";
    // 缺失
    const auto missing = st::load(file.string());
    check(missing.status == st::LoadStatus::Missing && missing.settings == st::Settings{} && missing.warnings.empty() &&
              missing.error.empty(),
          "文件不存在：Missing、默认设置、无错误");
    // 保存（自动建目录）并读回
    const st::Settings s = everythingChanged();
    const auto saved = st::save(s, file.string());
    check(!saved, "保存成功（自动创建目录）" + (saved ? "：" + *saved : std::string{}));
    check(!fs::exists(file.string() + ".tmp"), "保存后不留临时文件");
    const auto loaded = st::load(file.string());
    check(loaded.status == st::LoadStatus::Loaded && loaded.settings == s && loaded.warnings.empty(), "读回与保存的设置相同");
    check(readFile(file) == st::toJson(s), "文件内容即 toJson 的结果");

    // 覆盖保存
    st::Settings changed = s;
    changed.camera.speed = 12.0;
    check(!st::save(changed, file.string()) && st::load(file.string()).settings == changed, "覆盖保存");

    // 保存失败不破坏原文件：目标位置是目录 → 改名失败
    const fs::path dirTarget = dir / "is_a_directory";
    fs::create_directories(dirTarget / "child");
    const auto failed = st::save(s, dirTarget.string());
    check(failed.has_value() && fs::is_directory(dirTarget) && !fs::exists(dirTarget.string() + ".tmp"),
          "目标是目录：保存失败、不留临时文件" + (failed ? "（" + *failed + "）" : std::string{}));
    // 父路径是普通文件 → 无法建目录
    const auto noDir = st::save(s, (file / "x.json").string());
    check(noDir.has_value() && st::load(file.string()).settings == changed, "无法创建目录：保存失败，原文件不变");

    // 无法读取：是目录
    const auto unreadable = st::load(dirTarget.string());
    check(unreadable.status == st::LoadStatus::Unreadable && unreadable.settings == st::Settings{} && !unreadable.error.empty(),
          "路径是目录：Unreadable、默认设置");

    // 损坏 → 备份
    writeFile(file, "{ \"camera\": ");
    const auto corrupt = st::load(file.string());
    check(corrupt.status == st::LoadStatus::Corrupt && corrupt.settings == st::Settings{} &&
              corrupt.error.find("JSON 语法错误") != std::string::npos,
          "损坏的文件：Corrupt、默认设置、给出原因");
    std::string error;
    const auto backup = st::backupCorrupt(file.string(), &error);
    check(backup && *backup == file.string() + ".bak" && !fs::exists(file) && readFile(*backup) == "{ \"camera\": ",
          "坏文件改名为 .bak，内容不变" + (error.empty() ? std::string{} : "：" + error));
    check(st::load(file.string()).status == st::LoadStatus::Missing, "备份后原路径不存在");
    check(!st::backupCorrupt((dir / "no_such.json").string(), &error) && !error.empty(), "备份不存在的文件：失败并给出原因");

    // 字段级警告随 load 返回
    writeFile(file, R"({"version": 1, "camera": {"speed": -1}})");
    const auto warned = st::load(file.string());
    check(warned.status == st::LoadStatus::Loaded && warned.warnings.size() == 1 && warned.settings.camera.speed == 3.0,
          "load 返回字段级警告");
}

// M2-08：仿真操作、版本 1 → 2 的按键迁移、最近打开的场景
void testSimulationSettings() {
    const st::Settings d;
    check(d.keys[Command::SimPlayPause] == st::KeySlots{in::Key::Space, in::Key::Unknown} &&
              d.keys[Command::SimStep] == st::KeySlots{in::Key::Period, in::Key::Unknown} &&
              d.keys[Command::SimReset] == st::KeySlots{in::Key::Unknown, in::Key::Unknown},
          "默认：空格播放 / 暂停，句号单步，重置不绑定单键");
    check(d.keys[Command::CameraUp] == st::KeySlots{in::Key::E, in::Key::Unknown}, "版本 2 默认：camera.up 只有 E");
    check(st::kVersion >= 2 && st::commandName(Command::SimPlayPause) == std::string{"sim.playPause"}, "版本 ≥ 2（2026-11-24 起为 3）；操作名 sim.playPause");

    // 版本 1 的文件：camera.up 仍为旧默认（E、空格）→ 迁移，空格归 sim.playPause
    {
        const auto r = st::fromJson(R"({"version": 1, "keys": {"camera.up": ["E", "Space"]}})");
        check(r.settings.keys[Command::CameraUp] == st::KeySlots{in::Key::E, in::Key::Unknown} &&
                  r.settings.keys[Command::SimPlayPause][0] == in::Key::Space && contains(r.warnings, "设置文件版本 1"),
              "版本 1 迁移：空格改给播放 / 暂停\n" + joined(r.warnings));
    }
    // 版本 1 但用户改过 camera.up：不迁移；空格冲突按操作顺序处理（camera.up 在前，播放 / 暂停的空格被清空）
    {
        const auto r = st::fromJson(R"({"version": 1, "keys": {"camera.up": ["Space", "E"]}})");
        check(r.settings.keys[Command::CameraUp] == st::KeySlots{in::Key::Space, in::Key::E} &&
                  r.settings.keys[Command::SimPlayPause][0] == in::Key::Unknown && !contains(r.warnings, "设置文件版本 1") &&
                  contains(r.warnings, "同时用于 camera.up 与 sim.playPause"),
              "用户自定义的按键不迁移\n" + joined(r.warnings));
    }
    // 版本 2 的文件写了 camera.up [E, Space]：用户的选择，不迁移
    {
        const auto r = st::fromJson(R"({"version": 2, "keys": {"camera.up": ["E", "Space"]}})");
        check(r.settings.keys[Command::CameraUp] == st::KeySlots{in::Key::E, in::Key::Space} && !contains(r.warnings, "设置文件版本 1"),
              "版本 2 不迁移");
    }

    // M3-07c：记录采样率（默认 50 Hz，范围 1–1000）
    {
        check(st::Settings{}.ui.recordSampleHz == 50.0, "记录采样率默认 50 Hz");
        st::Settings custom;
        custom.ui.recordSampleHz = 100.0;
        const auto back = st::fromJson(st::toJson(custom));
        check(back.settings.ui.recordSampleHz == 100.0 && back.warnings.empty(), "记录采样率往返");
        const auto bad = st::fromJson(R"({"version": 3, "interface": {"recordSampleHz": 0}})");
        check(bad.settings.ui.recordSampleHz == 50.0 && contains(bad.warnings, "interface.recordSampleHz"), "采样率 0 → 默认 50，并警告");
    }

    // M4-08c：资源库位置（默认空 = 默认位置）往返
    {
        check(st::Settings{}.ui.libraryLocation.empty(), "资源库位置默认为空");
        st::Settings custom;
        custom.ui.libraryLocation = "/data/terra-library";
        const auto back = st::fromJson(st::toJson(custom));
        check(back.settings.ui.libraryLocation == "/data/terra-library" && back.warnings.empty() && back.settings == custom, "资源库位置往返");
        const auto bad = st::fromJson(R"({"version": 3, "interface": {"libraryLocation": 5}})");
        check(bad.settings.ui.libraryLocation.empty() && contains(bad.warnings, "interface.libraryLocation"), "资源库位置不是字符串：警告");
        // M4-08d：每个场景上次使用的显示配置
        st::Settings withDisplays;
        withDisplays.ui.sceneDisplays["/s/a.terra-scene.json"] = "/lib/displays/a/默认.terviz";
        const auto displaysBack = st::fromJson(st::toJson(withDisplays));
        check(displaysBack.settings == withDisplays && displaysBack.settings.ui.sceneDisplays.size() == 1 && displaysBack.warnings.empty(),
              "场景的显示配置往返");
    }

    // 最近打开的场景
    st::InterfaceSettings ui;
    for (int i = 0; i < 10; ++i) st::rememberScene(ui, "/s/" + std::to_string(i) + ".json");
    check(ui.recentScenes.size() == st::InterfaceSettings::kMaxRecentScenes && ui.recentScenes.front() == "/s/9.json" &&
              ui.recentScenes.back() == "/s/2.json",
          "最多 8 个，最新的在前");
    st::rememberScene(ui, "/s/5.json");
    check(ui.recentScenes.front() == "/s/5.json" && ui.recentScenes.size() == 8 &&
              std::count(ui.recentScenes.begin(), ui.recentScenes.end(), "/s/5.json") == 1,
          "再次打开：移到最前，不重复");
    st::rememberScene(ui, "");
    check(ui.recentScenes.front() == "/s/5.json", "空路径不记录");
    const auto bad = st::fromJson(R"({"version": 2, "interface": {"recentScenes": ["/a.json", 3, "", "/b.json"]}})");
    check(bad.settings.ui.recentScenes == std::vector<std::string>{"/a.json", "/b.json"} && contains(bad.warnings, "recentScenes"),
          "recentScenes 中不是路径的项被忽略并警告");
    const auto notArray = st::fromJson(R"({"version": 2, "interface": {"recentScenes": "/a.json"}})");
    check(notArray.settings.ui.recentScenes.empty() && contains(notArray.warnings, "recentScenes"), "recentScenes 不是数组");
}

void testDefaultPath() {
    check(st::defaultPath("/xdg", "/home/u") == "/xdg/eai-terra/settings.json", "XDG_CONFIG_HOME 优先");
    check(st::defaultPath("/xdg/", "/home/u") == "/xdg/eai-terra/settings.json", "XDG_CONFIG_HOME 末尾斜杠");
    check(st::defaultPath(nullptr, "/home/u") == "/home/u/.config/eai-terra/settings.json", "无 XDG：~/.config");
    check(st::defaultPath("", "/home/u") == "/home/u/.config/eai-terra/settings.json", "XDG 为空：~/.config");
    check(st::defaultPath("relative/dir", "/home/u") == "/home/u/.config/eai-terra/settings.json", "XDG 为相对路径：忽略");
    check(st::defaultPath(nullptr, nullptr).empty() && st::defaultPath("x", "y").empty(), "都不可用：空路径");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法：test_settings <临时目录>\n");
        return 2;
    }
    const fs::path dir{argv[1]};
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);

    testDefaults();
    testRoundTrip();
    testPartialAndInvalidFields();
    testKeys();
    testFiles(dir);
    testDefaultPath();
    testSimulationSettings();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
