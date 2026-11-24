#pragma once

// 用户设置（M1-14，D-026）：设置模型、校验、JSON 读写与默认路径。
//
// 文件：默认 $XDG_CONFIG_HOME/eai-terra/settings.json（XDG_CONFIG_HOME 须为绝对路径），否则 ~/.config/eai-terra/settings.json。
// 读取：文件不存在 → 默认设置；JSON 语法错误或顶层不是对象 → 默认设置并给出错误（调用方可把坏文件备份为 .bak）；
//       单个字段类型不对或超出范围 → 该字段用默认值并给出警告，其他字段照常读取；未知字段 → 警告并忽略。
// 写入：先写同目录的临时文件再改名（原子替换），自动创建目录。
// 公共接口只用 Terra 类型与标准库；JSON 库只在实现中使用。本模块不依赖平台、渲染与界面，由应用换算成各模块参数。

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "terra/input/types.hpp"

namespace terra::settings {

constexpr int kVersion = 3;  // 文件格式版本。2（M2-08）：新增仿真操作与最近打开的场景；空格键默认改为仿真播放/暂停。
                             // 3（2026-11-24，用户要求）：默认环境改为半球光（不用环境贴图）

// ---- 界面 ----
struct InterfaceSettings {
    static constexpr double kMinScale = 0.5;
    static constexpr double kMaxScale = 4.0;

    std::string language = "zh";    // "zh" | "en"
    std::optional<double> uiScale;  // 空 = 自动（按显示器推算）；否则 [kMinScale, kMaxScale]
    bool showOutliner = true;
    bool showDetails = true;
    bool showLog = true;
    bool showStats = true;
    std::string layout;             // 停靠布局（界面库的布局文本，原样保存）；空 = 默认布局
    // M2-08：最近打开的场景文件（绝对路径），最新的在前，最多 kMaxRecentScenes 个
    static constexpr std::size_t kMaxRecentScenes = 8;
    std::vector<std::string> recentScenes;
};

// 把 path 放到最近打开列表的最前面（去重，超出上限的丢弃）
void rememberScene(InterfaceSettings& ui, const std::string& path);

// ---- 显示 ----
enum class WindowMode { Windowed, BorderlessFullscreen, ExclusiveFullscreen };
// 文件中的名称："windowed" | "borderless" | "exclusive"。
const char* windowModeName(WindowMode mode) noexcept;

struct VideoModeSetting {
    int width = 0;
    int height = 0;
    int refreshRate = 0;  // Hz
};

struct DisplaySettings {
    static constexpr int kMinWindowSize = 320;
    static constexpr int kMaxWindowSize = 16384;
    static constexpr int kMaxMonitor = 63;

    WindowMode mode = WindowMode::Windowed;
    int monitor = 0;                            // 全屏所用显示器；0 = 主显示器；不存在时应用退回主显示器
    std::optional<VideoModeSetting> videoMode;  // 独占全屏的分辨率与刷新率；空 = 显示器当前模式
    bool vsync = true;
    int windowWidth = 1600;                     // 窗口模式下的大小（屏幕坐标）
    int windowHeight = 900;
};

// ---- 相机（与 terra::view::CameraControlSettings 对应，另加视场角）----
struct CameraSettings {
    double lookDegreesPerPixel = 0.15;  // [0.01, 1]
    double speed = 3.0;                 // m/s，[0.1, 100]
    double boost = 3.0;                 // 倍率，[1, 20]
    double wheelStep = 0.5;             // m/格，[0.01, 10]
    double panMetersPerPixel = 0.01;    // m/像素，[0.0005, 0.5]
    double fovDegrees = 60.0;           // 竖直视场角，[20, 120]
};

// ---- 渲染（M1-08，与 terra::render 的光照与 RenderSettings 对应）----
enum class ToneMapperSetting { AgX, PbrNeutral, Aces, None };
// 文件中的名称："agx" | "pbrNeutral" | "aces" | "none"。
const char* toneMapperName(ToneMapperSetting mapper) noexcept;

// 环境光来源（M1-08b）：半球光，或 assets/hdri/ 中的环境贴图。文件中的名称："hemisphere" | "outdoor" | "studio"。
enum class EnvironmentSetting { Hemisphere, Outdoor, Studio };
const char* environmentName(EnvironmentSetting environment) noexcept;

struct RenderSettings {
    double sunAzimuthDegrees = 150.0;   // 太阳方位角：从 +X 起逆时针（从上往下看），[0, 360]
    double sunElevationDegrees = 50.0;  // 太阳高度角，[1, 90]
    double sunIntensity = 3.0;          // [0, 20]
    double ambientIntensity = 0.6;      // 半球环境光强度，[0, 5]
    double exposureEv = 0.0;            // [-10, 10]
    ToneMapperSetting toneMapper = ToneMapperSetting::AgX;
    int msaaSamples = 4;                // 1、2、4、8
    bool shadows = true;
    bool grid = true;
    // M1-08b：环境贴图光照
    EnvironmentSetting environment = EnvironmentSetting::Hemisphere;  // 2026-11-24 起默认不用环境贴图（用户要求）
    double environmentIntensity = 0.6;          // [0, 5]
    double environmentRotationDegrees = 0.0;    // [0, 360]
    bool environmentBackground = true;          // 视口背景显示环境贴图
    double backgroundBlur = 0.25;               // [0, 1]
    // M1-09：方向光跟随环境贴图中的太阳（方向与亮度，贴图中的太阳同时从环境光照中去掉）；贴图没有明显太阳时不起作用
    bool sunFromEnvironment = true;
    // M1-10：资产按 ARM 打包约定（R = 环境遮蔽）推断的遮蔽贴图是否使用
    bool assetArmOcclusion = true;
};

struct Range {
    double min;
    double max;
};
namespace ranges {
constexpr Range kLook{0.01, 1.0};
constexpr Range kSpeed{0.1, 100.0};
constexpr Range kBoost{1.0, 20.0};
constexpr Range kWheelStep{0.01, 10.0};
constexpr Range kPan{0.0005, 0.5};
constexpr Range kFov{20.0, 120.0};
constexpr Range kAzimuth{0.0, 360.0};
constexpr Range kElevation{1.0, 90.0};
constexpr Range kSunIntensity{0.0, 20.0};
constexpr Range kAmbient{0.0, 5.0};
constexpr Range kExposure{-10.0, 10.0};
constexpr Range kEnvironmentIntensity{0.0, 5.0};
constexpr Range kEnvironmentRotation{0.0, 360.0};
constexpr Range kBackgroundBlur{0.0, 1.0};
}  // namespace ranges

// ---- 按键 ----
// 可在设置中修改按键的操作。文件中的名称见 commandName（如 "camera.forward"）。鼠标操作暂不可改。
enum class Command : int {
    CameraForward,
    CameraBack,
    CameraLeft,
    CameraRight,
    CameraUp,
    CameraDown,
    CameraBoost,  // 也是 Shift+中键平移的修饰键
    CameraReset,
    AppFullscreen,
    SimPlayPause,  // M2-08：仿真播放 / 暂停（默认空格）
    SimStep,       // M2-08：单步（默认句号键）
    SimReset,      // M2-08：重置（默认不绑定单键；界面另有固定的 Ctrl+R）
    Count,
};
constexpr std::size_t kCommandCount = static_cast<std::size_t>(Command::Count);
const char* commandName(Command command) noexcept;
std::optional<Command> commandFromName(std::string_view name) noexcept;

using KeySlots = std::array<terra::input::Key, 2>;  // 主键与备用键；Key::Unknown 表示空

// 不能分配给操作的键：Escape（取消按键录制）、左右 Ctrl（界面快捷键，如 Ctrl+加减号）、Key::Unknown。
bool isReservedKey(terra::input::Key key) noexcept;

class KeyMap {
public:
    KeyMap();  // 默认按键

    const KeySlots& operator[](Command command) const noexcept;
    KeySlots& operator[](Command command) noexcept;

    // 把 key 放到 command 的第 slot 个槽位（slot 为 0 或 1）。key 已被其他操作或同一操作的另一槽位占用时，
    // 先从那里移除，返回被移除的操作；保留键或非法槽位不做修改，返回空。Key::Unknown 等同于 clear。
    std::optional<Command> assign(Command command, std::size_t slot, terra::input::Key key);
    void clear(Command command, std::size_t slot);
    // 使用 key 的操作；未使用时返回空。
    std::optional<Command> find(terra::input::Key key) const noexcept;

    friend bool operator==(const KeyMap& a, const KeyMap& b) noexcept { return a.slots_ == b.slots_; }
    friend bool operator!=(const KeyMap& a, const KeyMap& b) noexcept { return !(a == b); }

private:
    std::array<KeySlots, kCommandCount> slots_{};
};

// ---- 全部设置 ----
struct Settings {
    InterfaceSettings ui;
    DisplaySettings display;
    CameraSettings camera;
    RenderSettings render;
    KeyMap keys;
};

bool operator==(const VideoModeSetting& a, const VideoModeSetting& b) noexcept;
bool operator==(const InterfaceSettings& a, const InterfaceSettings& b) noexcept;
bool operator==(const DisplaySettings& a, const DisplaySettings& b) noexcept;
bool operator==(const CameraSettings& a, const CameraSettings& b) noexcept;
bool operator==(const RenderSettings& a, const RenderSettings& b) noexcept;
bool operator==(const Settings& a, const Settings& b) noexcept;
inline bool operator!=(const VideoModeSetting& a, const VideoModeSetting& b) noexcept { return !(a == b); }
inline bool operator!=(const InterfaceSettings& a, const InterfaceSettings& b) noexcept { return !(a == b); }
inline bool operator!=(const DisplaySettings& a, const DisplaySettings& b) noexcept { return !(a == b); }
inline bool operator!=(const CameraSettings& a, const CameraSettings& b) noexcept { return !(a == b); }
inline bool operator!=(const RenderSettings& a, const RenderSettings& b) noexcept { return !(a == b); }
inline bool operator!=(const Settings& a, const Settings& b) noexcept { return !(a == b); }

// ---- 序列化 ----
// 带缩进的 JSON 文本，字段顺序固定，以换行结尾。
std::string toJson(const Settings& settings);

struct ParseResult {
    Settings settings;                  // 失败时为默认设置
    std::vector<std::string> warnings;  // 字段级问题（已用默认值替代）
    std::optional<std::string> error;   // 整体无法使用（语法错误、顶层不是对象）
};
ParseResult fromJson(std::string_view text);

enum class LoadStatus {
    Loaded,      // 已读取（可能有字段级警告）
    Missing,     // 文件不存在：默认设置，不算错误
    Corrupt,     // 无法解析：默认设置
    Unreadable,  // 存在但无法读取（权限、是目录等）：默认设置
};
const char* loadStatusName(LoadStatus status) noexcept;

struct LoadResult {
    LoadStatus status = LoadStatus::Missing;
    Settings settings;
    std::vector<std::string> warnings;
    std::string error;  // Corrupt / Unreadable 时的原因
};
LoadResult load(const std::string& path);

// 原子写入；失败时返回原因，原文件保持不变。
std::optional<std::string> save(const Settings& settings, const std::string& path);

// 把无法解析的设置文件改名为 path + ".bak"（覆盖旧备份），避免下一次保存时覆盖用户内容。成功时返回备份路径。
std::optional<std::string> backupCorrupt(const std::string& path, std::string* error = nullptr);

// 默认路径（见文件开头）；两个环境变量都不可用时返回空字符串。
std::string defaultPath(const char* xdgConfigHome, const char* home);
std::string defaultPath();  // 读取当前进程的 XDG_CONFIG_HOME 与 HOME

}  // namespace terra::settings
