#pragma once

// 显示配置（M4-08d；开发计划 11.7 节：以场景为核心、存入资源库的用户资产）。文件为 .terviz（JSON）：
//
//   {"format": "terra.viz", "version": 1, "name": "…", "scene": "<相对本文件的场景路径>",
//    "windows": [{"type": "plot", "id": "…", "topic": "…", "title": "…", "params": {…}, "placement": {…}}, …],
//    "overlays": [], "extensions": {}}
//
// 扩展的余地（M4-08d 评审时用户的要求，设计稿 4.1 节）：
//   - 已知的参数严格校验，类型错误为错误；
//   - 不认识的参数、窗口类型与顶层字段（来自更新的版本）给出警告并原样保留，写回时不丢失；
//   - overlays（视口叠加，B 层）与 extensions（第三方扩展）本步只读写、不使用；
//   - version 只在不兼容时增加。
// 公共头不含 JSON 库的类型：保留的内容以 JSON 文本保存。

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace terra::viz {

inline constexpr const char* kVizSuffix = ".terviz";
inline constexpr const char* kVizFormat = "terra.viz";
constexpr int kVizVersion = 1;

enum class WindowType { Topics, Message, Plot, Image, Tf, Unknown };

const char* windowTypeName(WindowType type);  // "topics"、"message"、"plot"、"image"、"tf"；Unknown 为 ""
std::optional<WindowType> windowTypeFromName(const std::string& name);

struct PlotSeries {
    std::string source = "ros2";  // "ros2"（话题）或 "sim"（仿真自己的传感器）
    std::string topic;            // source = ros2
    std::string sensor;           // source = sim：传感器全名（如 "turtlebot4/imu"）
    std::string field;            // 字段路径，如 "angular_velocity.z"、"motor_state[2].q"；sim 为 "gyro.z" 等
    std::string label;            // 空时显示 话题 + 字段
    std::optional<std::array<float, 4>> color;  // RGBA 0–1；空时自动配色
    double scale = 1.0;
    double offset = 0.0;
    std::string extraJson;  // 不认识的参数（JSON 对象文本；空为没有）
};

// 窗口在主窗口中的相对位置与大小（0–1），以及停靠或浮动；具体停靠位置跟随全局布局（设计稿问题 4）
struct Placement {
    std::array<double, 2> position{0.0, 0.0};
    std::array<double, 2> size{0.3, 0.3};
    bool docked = false;
};

struct VizWindow {
    WindowType type = WindowType::Topics;
    std::string typeName;  // 文件中的 type（Unknown 时保留原名）
    std::string id;        // 稳定的窗口标识；空时按类型与话题生成
    std::string topic;     // message、image
    std::string title;
    // 曲线
    double seconds = 10.0;
    std::string timeBase = "receive";  // "receive"（接收时间）或 "header"（消息头时间）
    std::optional<std::array<double, 2>> yRange;  // 空为自动
    std::vector<PlotSeries> series;
    // 图像
    std::array<double, 2> depthRange{0.0, 5.0};  // 米
    std::string colormap = "turbo";               // "turbo"、"gray"
    std::string fit = "fit";                      // "fit"（等比缩放）、"actual"（1:1）
    // 消息
    int pageSize = 100;
    // TF 树
    std::string tfRoot;  // 只看该坐标系之下；空为全部
    // 话题表
    std::string filter;

    std::optional<Placement> placement;
    std::string extraParams;  // params 中不认识的参数（JSON 对象文本）
    std::string extraFields;  // 窗口一级不认识的字段（JSON 对象文本）
    std::string rawJson;      // Unknown：整个窗口的 JSON 文本，原样写回
};

struct VizConfig {
    std::string name;
    std::string scene;  // 关联的场景（绝对路径；文件中写相对路径）
    std::vector<VizWindow> windows;
    std::string overlaysJson = "[]";    // 预留：视口叠加
    std::string extensionsJson = "{}";  // 预留：第三方扩展
    std::string extraJson;              // 不认识的顶层字段（JSON 对象文本）
};

struct VizReadResult {
    std::optional<VizConfig> config;  // 有错误时为空
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

// 解析文本；path 用于展开相对的场景路径
VizReadResult parseViz(const std::string& text, const std::string& path);
VizReadResult readViz(const std::string& path);
// 序列化；场景路径写为相对 path 所在目录（不同盘符等无法相对时写绝对路径）
std::string vizToJson(const VizConfig& config, const std::string& path);
// 写文件（先写临时文件再改名）；失败时返回原因
std::optional<std::string> saveViz(const VizConfig& config, const std::string& path);

// 窗口的稳定标识：id 非空时用它，否则按 类型:话题（曲线按标题）生成
std::string windowId(const VizWindow& window);

}  // namespace terra::viz
