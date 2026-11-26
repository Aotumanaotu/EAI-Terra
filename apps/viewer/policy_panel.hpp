#pragma once

// 策略的运行界面与组件管理（M4-10a；用户 2026-11-25：只需选择策略与机器人即可自动配置；需要的库文件由客户端提示并代为
// 下载安装，带进度条，没有网络等问题时给出警告）。
//
// - “仿真 → 运行策略…”：选择策略文件（.terra-policy.json，或 .onnx 由当前场景自动配置），窗口列出自动完成的项、警告与
//   无法运行的原因；“运行”后策略接管对应的执行器。需要的组件没有安装时弹出“需要下载组件”。
// - 组件窗口（设置 → 组件…）：列出组件、版本、大小、许可与状态；下载并安装（进度条、可取消）、从文件安装、重新校验、删除。
//   下载失败（没有网络、地址不可达）时警告，并给出下载地址与“从文件安装”的方法。
// - 策略运行时视口上方的速度指令面板：三个滑块、归零、键盘驾驶（I/K、J/L、U/O）、推理次数与耗时。
// 这里只绘制并返回操作（PolicyAction），由查看器把控制器交给当前场景。

#include <array>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "imgui.h"
#include "terra/library/components.hpp"
#include "terra/policy/autoconfig.hpp"
#include "terra/policy/controller.hpp"
#include "terra/scene/scene.hpp"
#include "terra/ui/file_browser.hpp"

namespace terra::viewer {

struct PolicyContext {
    const terra::scene::SceneInstance* scene = nullptr;  // 当前场景（没有打开时为空）
    std::string scenePath;
    std::string componentsDirectory;
    std::string catalogPath;
    bool noise = true;                                    // 传感器噪声的运行时开关
    terra::policy::PolicyController* running = nullptr;  // 正在运行的策略
    ImVec2 viewportMin{0, 0}, viewportMax{0, 0};          // 视口图像的位置（速度指令面板贴在其上方）
    bool viewportHovered = false;
    bool keyboardFree = true;  // 没有输入框占用键盘
};

struct PolicyAction {
    enum class Type { None, Start, Stop };
    Type type = Type::None;
    std::unique_ptr<terra::policy::PolicyController> controller;  // Start
    std::string name;                                              // Start、Stop：日志用
};

struct PolicyPanelState {
    // 运行策略的窗口
    bool dialogOpen = false;
    std::string file;
    std::string preparedKey;  // 文件 + 场景（变化时重新自动配置）
    std::optional<terra::policy::PolicyDescription> prepared;
    std::string format;
    std::vector<std::string> notes, warnings, errors;
    terra::ui::FileBrowser browser;
    bool startRequested = false;  // 自检或命令行：下一帧按“运行”
    // 组件
    bool componentsOpen = false;
    std::string neededComponent;  // “需要下载组件”弹窗（非空时打开）
    bool startAfterInstall = false;
    struct Install {
        std::string id;
        bool fromFile = false;
        terra::library::InstallProgress progress;
        terra::library::InstallResult result;
        std::atomic<bool> finished{false};
        std::thread worker;
        ~Install() {
            progress.cancel = true;
            if (worker.joinable()) worker.join();
        }
    };
    std::unique_ptr<Install> install;
    std::string installMessage;  // 最近一次安装的结果
    bool installNetworkError = false;
    std::string manualHint;
    std::string verifyMessage;
    terra::ui::FileBrowser componentBrowser;
    std::string componentForFile;  // 从文件安装的组件
    // 速度指令
    std::array<float, 3> command{0.0f, 0.0f, 0.0f};
    bool keyboard = false;
    std::array<float, 3> keyboardSpeed{0.5f, 0.3f, 1.0f};  // 键盘驾驶时按住的指令大小
    std::vector<std::string> log;                           // 本帧要写到日志面板的行
    // 自检
    int commandPanelFrames = 0;
};

// 设置要运行的策略文件（菜单、拖放、命令行）；打开窗口
void requestPolicy(PolicyPanelState& state, const std::string& file);

// 每帧调用：运行策略的窗口、需要组件的弹窗、组件窗口、速度指令面板。返回用户的操作。
PolicyAction drawPolicyUi(PolicyPanelState& state, const PolicyContext& context);

// 按当前场景准备（自动配置或读取说明文件）；结果写入 state（窗口打开时每帧检查，文件或场景变化时重新准备）
void preparePolicy(PolicyPanelState& state, const PolicyContext& context);

// 按准备好的说明文件加载模型并创建控制器；需要的组件不在时设置 neededComponent 并返回空
std::unique_ptr<terra::policy::PolicyController> createController(PolicyPanelState& state, const PolicyContext& context, std::string& error);

}  // namespace terra::viewer
