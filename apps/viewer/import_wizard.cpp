#include "import_wizard.hpp"

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <filesystem>

#include "imgui.h"
#include "terra/ui/text.hpp"

namespace terra::viewer {

namespace {

namespace im = terra::importer;
using terra::ui::TextId;
using terra::ui::tr;

void reanalyze(ImportWizardState& s) {
    s.options.meshScale = s.meshScale > 0.0f ? static_cast<double>(s.meshScale) : 1.0;
    s.options.name = s.name;
    s.plan = im::analyze(s.options);
    s.result.reset();
    if (s.plan->name != s.name) std::snprintf(s.name, sizeof s.name, "%s", s.plan->name.c_str());
}

ImVec4 severityColor(im::ImportIssue::Severity severity) {
    switch (severity) {
        case im::ImportIssue::Severity::Error: return ImVec4(1.0f, 0.4f, 0.35f, 1.0f);
        case im::ImportIssue::Severity::Warning: return ImVec4(1.0f, 0.8f, 0.3f, 1.0f);
        case im::ImportIssue::Severity::Info: return ImVec4(0.55f, 0.75f, 1.0f, 1.0f);
    }
    return ImVec4(1, 1, 1, 1);
}

const char* severityText(im::ImportIssue::Severity severity) {
    switch (severity) {
        case im::ImportIssue::Severity::Error: return tr(TextId::ImportError);
        case im::ImportIssue::Severity::Warning: return tr(TextId::ImportWarning);
        case im::ImportIssue::Severity::Info: return tr(TextId::ImportInfo);
    }
    return "";
}

// 问题列表：级别、说明、处理办法（自动换行）
void issuesTable(const std::vector<im::ImportIssue>& issues) {
    if (!ImGui::BeginTable("issues", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) return;
    ImGui::TableSetupColumn("##severity", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("##text", ImGuiTableColumnFlags_WidthStretch);
    for (const auto& i : issues) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextColored(severityColor(i.severity), "%s", severityText(i.severity));
        ImGui::TableSetColumnIndex(1);
        ImGui::TextWrapped("%s", i.message.c_str());
        if (!i.hint.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            ImGui::TextWrapped("→ %s", i.hint.c_str());
            ImGui::PopStyleColor();
        }
    }
    ImGui::EndTable();
}

}  // namespace

void beginImport(ImportWizardState& s, const std::string& input, const std::string& outputRoot) {
    s.options = im::defaultOptions();
    s.options.input = input;
    s.options.outputRoot = outputRoot;
    s.meshScale = 1.0f;
    s.name[0] = '\0';
    s.plan = im::analyze(s.options);
    s.result.reset();
    std::snprintf(s.name, sizeof s.name, "%s", s.plan->name.c_str());
    s.open = true;
    s.focus = true;
    s.log.push_back(std::string{tr(TextId::LogImportAnalyzed)} + input);
}

std::optional<std::string> runImport(ImportWizardState& s, const terra::scene::WorldFactory& factory) {
    if (!s.plan) return std::nullopt;
    s.options.name = s.name;
    s.plan->name = im::sanitizeName(s.name);
    s.result = im::execute(*s.plan, s.options, factory);
    if (!s.result->ok) {
        s.log.push_back(std::string{tr(TextId::LogImportFailed)} + s.options.input);
        return std::nullopt;
    }
    s.log.push_back(std::string{tr(TextId::LogImported)} + s.result->directory);
    std::printf("INFO 导入完成：%s → %s\n", s.options.input.c_str(), s.result->scene.c_str());
    s.open = false;
    return s.result->scene;
}

std::optional<std::string> drawImportWizard(ImportWizardState& s, const terra::scene::WorldFactory& factory) {
    std::optional<std::string> opened;
    if (!s.open || !s.plan) return opened;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x * 0.6f, vp->WorkSize.y * 0.75f), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (s.focus) {
        ImGui::SetNextWindowFocus();
        s.focus = false;
    }
    bool visible = true;
    const std::string title = std::string{tr(TextId::ImportTitle)} + "###ImportWizard";
    if (ImGui::Begin(title.c_str(), &visible, ImGuiWindowFlags_NoDocking)) {
        auto& plan = *s.plan;
        const char* kind = plan.kind == im::SourceKind::Mjcf ? "MJCF" : plan.kind == im::SourceKind::Xacro ? "xacro → URDF" : "URDF";
        ImGui::TextWrapped("%s  %s", tr(TextId::ImportSource), plan.input.c_str());
        ImGui::TextDisabled("%s  ·  %zu %s  ·  %zu %s  ·  %zu %s", kind, plan.links, tr(TextId::ImportLinks), plan.joints, tr(TextId::ImportJoints),
                            plan.meshes.size(), tr(TextId::ImportMeshes));

        // 设置
        ImGui::SeparatorText(tr(TextId::ImportSettings));
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16.0f);
        ImGui::InputText(tr(TextId::ImportName), s.name, sizeof s.name);
        ImGui::SameLine();
        ImGui::TextDisabled("→ %s/%s", s.options.outputRoot.c_str(), im::sanitizeName(s.name).c_str());
        if (plan.kind != im::SourceKind::Mjcf) {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8.0f);
            ImGui::InputFloat(tr(TextId::ImportMeshScale), &s.meshScale, 0.0f, 0.0f, "%.4g");
            ImGui::SameLine();
            bool fixed = plan.fixedBase;
            if (ImGui::Checkbox(tr(TextId::ImportFixedBase), &fixed)) {
                s.options.fixedBase = fixed;
                plan.fixedBase = fixed;
            }
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.0f);
            ImGui::InputText("##pkgname", s.packageName, sizeof s.packageName);
            ImGui::SameLine();
            ImGui::TextUnformatted("=");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - ImGui::GetFontSize() * 8.0f);
            ImGui::InputText("##pkgdir", s.packageDir, sizeof s.packageDir);
            ImGui::SameLine();
            if (ImGui::Button(tr(TextId::ImportAddPackage)) && s.packageName[0] != '\0' && s.packageDir[0] != '\0') {
                s.options.packages[s.packageName] = s.packageDir;
                s.packageName[0] = '\0';
                s.packageDir[0] = '\0';
                reanalyze(s);
            }
            if (ImGui::Button(tr(TextId::ImportReanalyze))) reanalyze(s);
        }

        // 问题
        const auto& issues = s.result ? s.result->issues : s.plan->issues;
        ImGui::SeparatorText(tr(TextId::ImportIssues));
        issuesTable(issues);

        auto& p = *s.plan;
        if (!p.meshes.empty() && ImGui::CollapsingHeader(tr(TextId::ImportMeshes))) {
            if (ImGui::BeginTable("meshes", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
                for (const auto& m : p.meshes) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextWrapped("%s", m.uri.c_str());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(m.convert ? tr(TextId::ImportConvert) : m.format.c_str());
                    ImGui::TableSetColumnIndex(2);
                    if (m.resolved.empty()) ImGui::TextColored(severityColor(im::ImportIssue::Severity::Error), "%s", tr(TextId::ImportNotFound));
                    else ImGui::TextWrapped("%s", m.resolved.c_str());
                }
                ImGui::EndTable();
            }
        }
        if (!p.actuators.empty() && ImGui::CollapsingHeader(tr(TextId::ImportActuators), ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::BeginTable("actuators", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn(tr(TextId::ImportEnabled), ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn(tr(TextId::ImportJoint));
                ImGui::TableSetupColumn(tr(TextId::ImportType));
                ImGui::TableSetupColumn(tr(TextId::ImportGain));
                ImGui::TableSetupColumn(tr(TextId::ImportForce));
                ImGui::TableHeadersRow();
                for (std::size_t k = 0; k < p.actuators.size(); ++k) {
                    auto& a = p.actuators[k];
                    ImGui::PushID(static_cast<int>(k));
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Checkbox("##on", &a.enabled);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(a.joint.c_str());
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", a.reason.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    int type = a.type == "velocity" ? 0 : 1;
                    const char* types[] = {tr(TextId::ImportVelocity), tr(TextId::ImportPosition)};
                    if (ImGui::Combo("##type", &type, types, 2)) a.type = type == 0 ? "velocity" : "position";
                    ImGui::TableSetColumnIndex(3);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputDouble("##gain", &a.gain, 0.0, 0.0, "%.4g");
                    ImGui::TableSetColumnIndex(4);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputDouble("##force", &a.forceLimit, 0.0, 0.0, "%.4g");
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
        }
        if (!p.sensors.empty() && ImGui::CollapsingHeader(tr(TextId::ImportSensors), ImGuiTreeNodeFlags_DefaultOpen)) {
            for (std::size_t k = 0; k < p.sensors.size(); ++k) {
                auto& sensor = p.sensors[k];
                ImGui::PushID(static_cast<int>(k) + 1000);
                ImGui::Checkbox("##on", &sensor.enabled);
                ImGui::SameLine();
                ImGui::Text("%s  ·  %s", sensor.type.c_str(), sensor.link.c_str());
                ImGui::SameLine();
                ImGui::TextDisabled("%s", sensor.reason.c_str());
                ImGui::PopID();
            }
        }

        ImGui::Separator();
        ImGui::BeginDisabled(!p.ok());
        if (ImGui::Button(tr(TextId::ImportRun))) opened = runImport(s, factory);
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button(tr(TextId::ImportCancel))) visible = false;
        if (!p.ok()) {
            ImGui::SameLine();
            ImGui::TextColored(severityColor(im::ImportIssue::Severity::Error), "%s", tr(TextId::ImportFixErrors));
        }
    }
    ImGui::End();
    if (!visible) s.open = false;
    return opened;
}

}  // namespace terra::viewer
