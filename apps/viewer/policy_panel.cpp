// 策略的运行界面与组件管理（M4-10a），说明见 policy_panel.hpp。

#include "policy_panel.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>

#include "terra/policy/backend.hpp"
#include "terra/ui/text.hpp"

namespace terra::viewer {

namespace {

namespace lib = terra::library;
namespace pol = terra::policy;
using terra::ui::TextId;
using terra::ui::tr;

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string megabytes(std::uint64_t bytes) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.0f MB", static_cast<double>(bytes) / 1e6);
    return buf;
}

const char* stateText(lib::ComponentState s) {
    switch (s) {
        case lib::ComponentState::Installed: return tr(TextId::ComponentsInstalled);
        case lib::ComponentState::Missing: return tr(TextId::ComponentsMissing);
        case lib::ComponentState::Corrupt: return tr(TextId::ComponentsCorrupt);
        case lib::ComponentState::Outdated: return tr(TextId::ComponentsOutdated);
    }
    return "";
}

ImVec4 stateColor(lib::ComponentState s) {
    return s == lib::ComponentState::Installed ? ImVec4(0.45f, 0.85f, 0.45f, 1.0f) : s == lib::ComponentState::Missing ? ImVec4(0.75f, 0.75f, 0.75f, 1.0f)
                                                                                                                      : ImVec4(0.95f, 0.6f, 0.3f, 1.0f);
}

void startInstall(PolicyPanelState& state, const lib::ComponentInfo& info, const PolicyContext& context, const std::string& localFile) {
    if (state.install) return;
    state.install = std::make_unique<PolicyPanelState::Install>();
    auto* job = state.install.get();
    job->id = info.id;
    job->fromFile = !localFile.empty();
    state.installMessage.clear();
    state.installNetworkError = false;
    state.manualHint.clear();
    const lib::ComponentInfo copy = info;
    const std::string dir = context.componentsDirectory;
    job->worker = std::thread([job, copy, dir, localFile] {
        job->result = lib::installComponent(copy, dir, job->progress, localFile);
        job->finished = true;
    });
}

// 安装进度（窗口与弹窗共用）；返回是否正在安装
bool drawInstallProgress(PolicyPanelState& state) {
    if (!state.install) return false;
    auto& job = *state.install;
    const int stage = job.progress.stage.load();
    const auto total = job.progress.total.load();
    const auto done = job.progress.done.load();
    float fraction = total > 0 ? static_cast<float>(static_cast<double>(done) / static_cast<double>(total)) : 0.0f;
    std::string overlay;
    if (stage == lib::InstallProgress::Download) {
        overlay = std::string{tr(TextId::ComponentsDownloading)} + "  " + megabytes(done) + " / " + megabytes(total);
    } else if (stage == lib::InstallProgress::Verify) {
        overlay = tr(TextId::ComponentsVerifying);
        fraction = 1.0f;
    } else {
        overlay = tr(TextId::ComponentsExtracting);
        fraction = 1.0f;
    }
    ImGui::ProgressBar(std::clamp(fraction, 0.0f, 1.0f), ImVec2(-1.0f, 0.0f), overlay.c_str());
    if (ImGui::Button(tr(TextId::PolicyCancel))) job.progress.cancel = true;
    return true;
}

void drawInstallResult(const PolicyPanelState& state) {
    if (state.installMessage.empty()) return;
    if (state.installNetworkError) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.65f, 0.3f, 1.0f));
        ImGui::TextWrapped("%s", tr(TextId::ComponentsOffline));
        ImGui::PopStyleColor();
        ImGui::TextWrapped("%s", state.manualHint.c_str());
        if (ImGui::SmallButton(tr(TextId::ComponentsCopyHint))) ImGui::SetClipboardText(state.manualHint.c_str());
    } else {
        ImGui::TextWrapped("%s", state.installMessage.c_str());
    }
}

}  // namespace

void requestPolicy(PolicyPanelState& state, const std::string& file) {
    std::error_code ec;
    state.file = std::filesystem::absolute(file, ec).lexically_normal().string();
    state.preparedKey.clear();
    state.dialogOpen = true;
}

void preparePolicy(PolicyPanelState& state, const PolicyContext& context) {
    const std::string key = state.file + "|" + context.scenePath;
    if (key == state.preparedKey) return;
    state.preparedKey = key;
    state.prepared.reset();
    state.format.clear();
    state.notes.clear();
    state.warnings.clear();
    state.errors.clear();
    if (state.file.empty()) return;
    if (endsWith(state.file, pol::kPolicySuffix)) {
        auto r = pol::readPolicyFile(state.file);
        state.warnings = r.warnings;
        state.errors = r.errors;
        if (r.policy) {
            if (context.scene && !r.policy->scene.empty() && r.policy->scene != context.scenePath) {
                state.warnings.push_back("说明文件推荐的场景是 " + r.policy->scene + "，当前场景不同：按关节名对应到当前场景");
            }
            state.notes.push_back(r.policy->name.empty() ? state.file : r.policy->name);
            state.prepared = std::move(r.policy);
        }
        if (!context.scene) state.errors.push_back(tr(TextId::PolicyNeedScene));
        return;
    }
    if (!context.scene) {
        state.errors.push_back(tr(TextId::PolicyNeedScene));
        return;
    }
    pol::AutoconfigInput input;
    input.model = state.file;
    input.scene = context.scenePath;
    auto r = pol::autoconfigure(input, context.scene->world->description());
    state.format = r.format;
    state.notes = r.notes;
    state.warnings = r.warnings;
    state.errors = r.errors;
    state.prepared = std::move(r.policy);
}

std::unique_ptr<pol::PolicyController> createController(PolicyPanelState& state, const PolicyContext& context, std::string& error) {
    if (!state.prepared || !context.scene) {
        error = state.errors.empty() ? tr(TextId::PolicyNeedScene) : state.errors.front();
        return nullptr;
    }
    pol::BackendContext backend;
    backend.componentsDirectory = context.componentsDirectory;
    backend.catalogPath = context.catalogPath;
    auto loaded = pol::loadModel(*state.prepared, backend);
    for (const auto& w : loaded.warnings) state.log.push_back(w);
    if (!loaded.model) {
        error = loaded.error;
        if (!loaded.missingComponent.empty()) state.neededComponent = loaded.missingComponent;
        return nullptr;
    }
    auto controller = pol::PolicyController::create(*state.prepared, std::move(loaded.model), *context.scene, context.noise, error);
    if (controller) {
        controller->setCommand({static_cast<double>(state.command[0]), static_cast<double>(state.command[1]), static_cast<double>(state.command[2])});
    }
    return controller;
}

PolicyAction drawPolicyUi(PolicyPanelState& state, const PolicyContext& context) {
    PolicyAction action;
    const auto catalog = lib::readComponentCatalog(context.catalogPath);

    // 安装完成
    if (state.install && state.install->finished) {
        state.install->worker.join();
        const auto& r = state.install->result;
        const lib::ComponentInfo* info = catalog.find(state.install->id);
        const std::string name = info ? info->nameChinese + " " + info->version : state.install->id;
        if (r.ok) {
            state.installMessage = std::string{tr(TextId::ComponentsDone)} + name;
            state.log.push_back(state.installMessage);
        } else if (r.cancelled) {
            state.installMessage = r.error;
        } else {
            state.installNetworkError = r.networkError;
            state.installMessage = std::string{tr(TextId::ComponentsFailed)} + r.error;
            if (info) state.manualHint = lib::manualInstallHint(*info, terra::ui::language() == terra::ui::Language::English);
            state.log.push_back(state.installMessage);
        }
        const bool ok = r.ok;
        state.install.reset();
        if (ok && state.startAfterInstall) {
            state.startAfterInstall = false;
            state.neededComponent.clear();
            state.startRequested = true;
        }
    }

    // 运行策略的窗口
    if (state.dialogOpen) {
        preparePolicy(state, context);
        ImGui::SetNextWindowSize(ImVec2(560, 0), ImGuiCond_Appearing);
        if (ImGui::Begin((std::string{tr(TextId::PolicyTitle)} + "###PolicyDialog").c_str(), &state.dialogOpen,
                         ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(tr(TextId::PolicyFile));
            ImGui::SameLine();
            ImGui::TextDisabled("%s", state.file.empty() ? "—" : state.file.c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton(tr(TextId::PolicyChoose))) {
                state.browser.open(state.file.empty() ? std::string{} : state.file, ".terra-policy.json|.onnx|.pt|.pth", "");
            }
            ImGui::TextUnformatted(tr(TextId::PolicyRobot));
            ImGui::SameLine();
            ImGui::TextDisabled("%s", context.scenePath.empty() ? "—" : context.scenePath.c_str());
            ImGui::TextWrapped("%s", tr(TextId::PolicyRobotHint));
            ImGui::Separator();
            if (!state.notes.empty()) {
                ImGui::TextUnformatted((std::string{tr(TextId::PolicyAutoconfig)} + (state.format.empty() ? "" : "（" + state.format + "）")).c_str());
                for (const auto& n : state.notes) ImGui::BulletText("%s", n.c_str());
            }
            if (!state.warnings.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.8f, 0.35f, 1.0f));
                ImGui::TextUnformatted(tr(TextId::PolicyWarnings));
                for (const auto& w : state.warnings) ImGui::BulletText("%s", w.c_str());
                ImGui::PopStyleColor();
            }
            if (!state.errors.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.45f, 0.4f, 1.0f));
                ImGui::TextUnformatted(tr(TextId::PolicyProblems));
                for (const auto& e : state.errors) ImGui::BulletText("%s", e.c_str());
                ImGui::PopStyleColor();
            }
            ImGui::Separator();
            ImGui::BeginDisabled(!state.prepared || !context.scene);
            if (ImGui::Button(tr(TextId::PolicyRun))) state.startRequested = true;
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button(tr(TextId::PolicyCancel))) state.dialogOpen = false;
        }
        ImGui::End();
    }
    if (auto chosen = state.browser.draw((std::string{tr(TextId::PolicyBrowserTitle)} + "###PolicyBrowser").c_str())) {
        requestPolicy(state, *chosen);
    }

    // 运行（窗口中的按钮、安装完成后、自检）
    if (state.startRequested) {
        state.startRequested = false;
        preparePolicy(state, context);
        std::string error;
        auto controller = createController(state, context, error);
        if (controller) {
            action.type = PolicyAction::Type::Start;
            action.name = state.prepared->name;
            action.controller = std::move(controller);
            state.dialogOpen = false;
        } else if (state.neededComponent.empty()) {
            state.errors.push_back(error);
            state.dialogOpen = true;
            state.log.push_back(std::string{tr(TextId::PolicyProblems)} + error);
        } else {
            state.log.push_back(error);
        }
    }

    // 需要下载组件
    if (!state.neededComponent.empty()) {
        const lib::ComponentInfo* info = catalog.find(state.neededComponent);
        bool open = true;
        ImGui::SetNextWindowSize(ImVec2(520, 0), ImGuiCond_Appearing);
        if (ImGui::Begin((std::string{tr(TextId::ComponentsNeeded)} + "###NeedComponent").c_str(), &open,
                         ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
            ImGui::TextWrapped("%s", tr(TextId::ComponentsNeededText));
            if (info) {
                ImGui::Separator();
                ImGui::Text("%s %s", info->nameChinese.c_str(), info->version.c_str());
                ImGui::TextDisabled("%s %s · %s %s", tr(TextId::ComponentsSize), megabytes(info->size).c_str(), tr(TextId::ComponentsLicense), info->license.c_str());
                if (!info->purposeChinese.empty()) ImGui::TextWrapped("%s", info->purposeChinese.c_str());
                ImGui::Separator();
                if (!drawInstallProgress(state)) {
                    if (ImGui::Button(tr(TextId::ComponentsInstall))) {
                        state.startAfterInstall = true;
                        startInstall(state, *info, context, {});
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(tr(TextId::ComponentsInstallFile))) {
                        state.startAfterInstall = true;
                        state.componentForFile = info->id;
                        state.componentBrowser.open({}, ".nupkg|.zip", "");
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(tr(TextId::ComponentsLater))) open = false;
                }
                drawInstallResult(state);
            }
        }
        ImGui::End();
        if (!open) {
            state.neededComponent.clear();
            state.startAfterInstall = false;
        }
    }

    // 组件窗口
    if (state.componentsOpen) {
        ImGui::SetNextWindowSize(ImVec2(640, 0), ImGuiCond_Appearing);
        if (ImGui::Begin((std::string{tr(TextId::ComponentsTitle)} + "###Components").c_str(), &state.componentsOpen, ImGuiWindowFlags_NoDocking)) {
            ImGui::TextWrapped("%s", tr(TextId::ComponentsHint));
            ImGui::TextDisabled("%s", context.componentsDirectory.c_str());
            for (const auto& e : catalog.errors) ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.4f, 1.0f), "%s", e.c_str());
            ImGui::Separator();
            for (const auto& c : catalog.components) {
                ImGui::PushID(c.id.c_str());
                const auto st = lib::componentStatus(c, context.componentsDirectory);
                ImGui::Text("%s", c.nameChinese.c_str());
                ImGui::SameLine();
                ImGui::TextColored(stateColor(st.state), "[%s]", stateText(st.state));
                ImGui::TextDisabled("%s %s · %s %s · %s %s", tr(TextId::ComponentsVersion), c.version.c_str(), tr(TextId::ComponentsSize),
                                    megabytes(c.size).c_str(), tr(TextId::ComponentsLicense), c.license.c_str());
                if (!c.purposeChinese.empty()) ImGui::TextWrapped("%s", c.purposeChinese.c_str());
                if (!st.detail.empty()) ImGui::TextDisabled("%s", st.detail.c_str());
                const bool busy = state.install != nullptr;
                if (busy && state.install->id == c.id) {
                    drawInstallProgress(state);
                } else {
                    ImGui::BeginDisabled(busy);
                    if (st.state != lib::ComponentState::Installed && ImGui::Button(tr(TextId::ComponentsInstall))) startInstall(state, c, context, {});
                    if (st.state != lib::ComponentState::Installed) ImGui::SameLine();
                    if (ImGui::Button(tr(TextId::ComponentsInstallFile))) {
                        state.componentForFile = c.id;
                        state.componentBrowser.open({}, ".nupkg|.zip", "");
                    }
                    if (st.state != lib::ComponentState::Missing) {
                        ImGui::SameLine();
                        if (ImGui::Button(tr(TextId::ComponentsVerify))) {
                            const auto v = lib::componentStatus(c, context.componentsDirectory, true);
                            state.verifyMessage = c.nameChinese + "：" + stateText(v.state) + (v.detail.empty() ? "" : "（" + v.detail + "）");
                        }
                        ImGui::SameLine();
                        if (ImGui::Button(tr(TextId::ComponentsRemove)) && !context.running) {
                            const auto failed = lib::removeComponent(c, context.componentsDirectory);
                            state.verifyMessage = failed ? *failed : c.nameChinese + "：" + tr(TextId::ComponentsMissing);
                        }
                    }
                    ImGui::EndDisabled();
                }
                ImGui::Separator();
                ImGui::PopID();
            }
            if (!state.verifyMessage.empty()) ImGui::TextWrapped("%s", state.verifyMessage.c_str());
            drawInstallResult(state);
        }
        ImGui::End();
    }
    if (auto chosen = state.componentBrowser.draw((std::string{tr(TextId::ComponentsBrowserTitle)} + "###ComponentBrowser").c_str())) {
        if (const auto* info = catalog.find(state.componentForFile)) startInstall(state, *info, context, *chosen);
    }

    // 速度指令面板
    if (context.running) {
        auto& c = *context.running;
        const auto& range = c.policy().commandRange;
        if (state.keyboard && context.keyboardFree) {
            const auto axis = [](ImGuiKey plus, ImGuiKey minus) { return (ImGui::IsKeyDown(plus) ? 1.0f : 0.0f) - (ImGui::IsKeyDown(minus) ? 1.0f : 0.0f); };
            state.command[0] = axis(ImGuiKey_I, ImGuiKey_K) * state.keyboardSpeed[0];
            state.command[1] = axis(ImGuiKey_J, ImGuiKey_L) * state.keyboardSpeed[1];
            state.command[2] = axis(ImGuiKey_U, ImGuiKey_O) * state.keyboardSpeed[2];
        }
        ImGui::SetNextWindowPos(ImVec2(context.viewportMin.x + 12.0f, context.viewportMin.y + 12.0f), ImGuiCond_Appearing);
        ImGui::SetNextWindowBgAlpha(0.88f);
        if (ImGui::Begin((std::string{tr(TextId::PolicyCommand)} + "###PolicyCommand").c_str(), nullptr,
                         ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                             ImGuiWindowFlags_NoFocusOnAppearing)) {
            ++state.commandPanelFrames;
            ImGui::TextDisabled("%s", c.policy().name.c_str());
            const TextId labels[3] = {TextId::PolicyForward, TextId::PolicyLateral, TextId::PolicyYaw};
            ImGui::PushItemWidth(220.0f);
            for (int i = 0; i < 3; ++i) {
                const auto k = static_cast<std::size_t>(i);
                ImGui::BeginDisabled(state.keyboard);
                ImGui::SliderFloat(tr(labels[i]), &state.command[k], static_cast<float>(range[k][0]), static_cast<float>(range[k][1]), "%.2f");
                ImGui::EndDisabled();
            }
            ImGui::PopItemWidth();
            if (ImGui::Button(tr(TextId::PolicyZero))) state.command = {0.0f, 0.0f, 0.0f};
            ImGui::SameLine();
            ImGui::Checkbox(tr(TextId::PolicyKeyboard), &state.keyboard);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr(TextId::PolicyKeyboardHint));
            ImGui::TextDisabled("%s %llu · %s %.2f ms · %s", tr(TextId::PolicyInferences), static_cast<unsigned long long>(c.inferences()),
                                tr(TextId::PolicyLastInference), c.lastInferenceSeconds() * 1e3, c.model().describe().c_str());
            if (c.failed()) ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.4f, 1.0f), "%s", c.error().c_str());
        }
        ImGui::End();
        c.setCommand({static_cast<double>(state.command[0]), static_cast<double>(state.command[1]), static_cast<double>(state.command[2])});
        // setCommand 截断到范围；面板显示截断后的值
        for (std::size_t i = 0; i < 3; ++i) state.command[i] = static_cast<float>(c.command()[i]);
    }
    return action;
}

}  // namespace terra::viewer
