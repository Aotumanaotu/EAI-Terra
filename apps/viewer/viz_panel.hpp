#pragma once

// 显示配置的窗口与 ROS2 面板（M4-08d；设计稿第 3、4 节）。
//
// 当前的显示配置（VizConfig）就是“打开了哪些窗口”：话题表（ROS2 面板）、消息、曲线、图像、TF 树。界面上打开或关闭窗口
// 直接改配置（标“未保存”），“保存”由查看器写入资源库。窗口的位置与大小在保存时取当前值，恢复后第一次绘制时应用。
// ROS2 的数据来自 terra::ros2::Monitor（只在 TERRA_WITH_ROS2 时编译）：每帧按配置同步订阅（打开的消息与图像窗口、
// 曲线中的 ROS2 字段、TF 树）；仿真来源的曲线由查看器提供数据（SimPlotSource）。
// 没有 ROS2 的构建：ROS2 来源的窗口显示安装说明，仿真来源的曲线照常可用。

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "terra/render/resources.hpp"
#include "terra/viz/exporter.hpp"
#include "terra/viz/viz_config.hpp"
#ifdef TERRA_WITH_ROS2
#include "terra/ros2/monitor.hpp"
#endif

namespace terra::viewer {

// 仿真来源的曲线数据：sensor 为 IMU 全名，field 为 "acc.x" … "gyro.z"
struct SimPlotSource {
    std::function<std::vector<std::string>()> sensors;
    std::function<std::vector<std::pair<double, double>>(const std::string& sensor, const std::string& field)> samples;  // 仿真时间、值
};

// 界面上要查看器处理的操作
struct VizAction {
    enum class Type { Save, SaveAs, Switch, Import, CloseAll };
    Type type = Type::Save;
    std::string text;  // SaveAs：名字；Switch：显示配置文件
};

struct VizContext {
    std::string currentScene;
    std::vector<std::pair<std::string, std::string>> displays;  // 当前场景的显示配置：名字、文件
    std::string defaultExportDir;
    const SimPlotSource* sim = nullptr;
};

struct VizPanelState {
    terra::viz::VizConfig config;
    std::string path;   // 当前显示配置的文件；空表示还没有保存
    bool dirty = false;
    std::set<std::string> pendingPlacement;  // 恢复后第一次绘制时应用位置与大小的窗口
    struct ImageView {
        terra::render::Texture2D texture;
        std::uint64_t sequence = 0;
        std::string info;
        std::string error;
    };
    std::map<std::string, ImageView> images;  // 窗口标识 → 纹理
    std::map<std::string, bool> paused;
    std::map<std::string, int> pages;        // 消息窗口：数组路径 → 页码
    std::array<char, 128> topicFilter{};
    // 另存为、导出
    bool saveAsOpen = false;
    std::array<char, 128> saveName{};
    bool exportOpen = false;
    std::array<char, 128> exportName{};
    std::array<char, 512> exportDir{};
    std::map<std::string, bool> exportChosen;
    std::map<std::string, terra::viz::ParameterValues> exportValues;
    std::vector<std::string> exportReport;
#ifdef TERRA_WITH_ROS2
    std::unique_ptr<terra::ros2::Monitor> monitor;
    std::string monitorError;
    bool monitorTried = false;
    std::set<std::string> watched;                             // 已 watch 的话题（消息、图像窗口）
    std::set<std::pair<std::string, std::string>> fields;      // 已加入的曲线字段
    bool tfWatched = false;
#endif
    // 自检
    std::size_t windowsDrawn = 0;
    std::size_t imageUploads = 0;
    std::size_t messageTrees = 0;
    std::size_t plotPoints = 0;
    bool exportSelfTest = false;  // 自检：本帧按导出弹窗的默认值导出全部格式
};

bool ros2PanelAvailable();

// 打开或切到某类窗口（已有同一标识的窗口时不重复添加）；返回窗口标识
std::string openWindow(VizPanelState& state, terra::viz::VizWindow window);
// 曲线：加入一条（没有曲线窗口时新建一个）
void addSeries(VizPanelState& state, const terra::viz::PlotSeries& series, const std::string& plotTitle = {});
// 换成另一份配置（打开场景时恢复、导入）：关闭现有窗口，恢复位置
void applyConfig(VizPanelState& state, terra::viz::VizConfig config, std::string path);
// 保存前取各窗口当前的位置与大小（相对主窗口），写入 config
void capturePlacement(VizPanelState& state);
// ROS2 的导出环境（话题类型、坐标系）；没有 ROS2 时为空
terra::viz::ExportContext exportContext(const VizPanelState& state);

// 每帧：同步订阅并绘制配置中的全部窗口与弹窗；返回需要查看器处理的操作
std::vector<VizAction> drawVizWindows(VizPanelState& state, const VizContext& context);

}  // namespace terra::viewer
