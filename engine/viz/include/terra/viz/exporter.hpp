#pragma once

// 显示配置的导出器（M4-08d；设计稿 4.1 节：导出格式可以增加，不改界面）。
//
// 每个导出器给出：标识、显示名、文件后缀、参数清单（键、显示名、类型、默认值、可选值）与转换函数。
// 导出器注册在一张表中；查看器的导出弹窗按表列出所有格式，按参数清单自动生成控件。
// 本步注册两个：
//   - "terviz"：Terra 显示配置，没有参数；
//   - "rviz"：RViz2 配置，参数为固定坐标系、网格、视距、帧率。只导出 RViz2 默认插件中有对应显示的窗口，
//     其余（曲线、消息窗口、仿真来源）列为“未导出的部分”，并写在文件开头的注释中。
// 以后增加 Foxglove、PlotJuggler 等格式只需实现 Exporter 并 registerExporter。

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "terra/viz/viz_config.hpp"

namespace terra::viz {

// 中英文两种显示文字（engine/viz 不依赖界面模块的文本表）
struct Text {
    std::string chinese;
    std::string english;
};

struct ParameterSpec {
    enum class Kind { Text, Number, Bool, Choice };
    std::string key;
    Text label;
    Kind kind = Kind::Text;
    std::string defaultValue;          // 数值与开关也以文本表示："1.5"、"true"
    std::vector<std::string> choices;  // Choice：可选值（Text 也可以给建议值，界面以下拉提示）
    double min = 0.0;                  // Number 的范围
    double max = 0.0;
    Text help;
};

using ParameterValues = std::map<std::string, std::string>;

// 导出时可用的环境信息（由查看器从 ROS2 监视器填写；没有时为空）
struct ExportContext {
    std::vector<std::string> frames;                     // TF 树中的坐标系（第一个为根）
    std::map<std::string, std::string> topicTypes;       // 话题 → 类型，如 "sensor_msgs/msg/LaserScan"
    std::map<std::string, bool> bestEffort;              // 话题 → 发布者是否为 best effort
};

struct ExportResult {
    bool ok = false;
    std::string text;                     // 文件内容
    std::vector<std::string> notExported;  // 没有导出的部分（每项一句说明）
    std::string error;
};

class Exporter {
public:
    virtual ~Exporter() = default;
    virtual std::string id() const = 0;
    virtual Text name() const = 0;
    virtual std::string suffix() const = 0;  // 含点，如 ".rviz"
    virtual std::vector<ParameterSpec> parameters(const VizConfig& config, const ExportContext& context) const = 0;
    // 预览：不写文件，只列出会导出与不会导出的部分（导出弹窗中显示）
    virtual std::vector<std::string> notExported(const VizConfig& config, const ExportContext& context) const = 0;
    // outputPath：导出文件的路径（用于写相对路径）
    virtual ExportResult convert(const VizConfig& config, const ParameterValues& values, const ExportContext& context,
                                 const std::string& outputPath) const = 0;
};

// 注册表（按注册顺序）；内置的两个在首次调用时注册
const std::vector<const Exporter*>& exporters();
const Exporter* findExporter(const std::string& id);
void registerExporter(std::unique_ptr<Exporter> exporter);

// 参数值：缺少的取默认值；校验类型与范围，不合法的返回错误说明（空表示全部合法）
ParameterValues withDefaults(const std::vector<ParameterSpec>& specs, const ParameterValues& values);
std::vector<std::string> validateParameters(const std::vector<ParameterSpec>& specs, const ParameterValues& values);

// 导出到文件：转换后写入 outputPath（先写临时文件再改名）
ExportResult exportToFile(const Exporter& exporter, const VizConfig& config, const ParameterValues& values, const ExportContext& context,
                          const std::string& outputPath);

}  // namespace terra::viz
