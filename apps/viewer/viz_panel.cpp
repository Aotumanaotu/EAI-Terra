// 显示配置的窗口与 ROS2 面板（M4-08d）。

#include "viz_panel.hpp"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cstddef>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>

#include "imgui.h"
#include "imgui_internal.h"
#include "camera_sensor.hpp"
#include "terra/render/image.hpp"
#include "terra/ui/text.hpp"

namespace terra::viewer {

namespace {

namespace vz = terra::viz;
namespace fs = std::filesystem;
using terra::ui::TextId;
using terra::ui::tr;

constexpr ImU32 rgba(unsigned r, unsigned g, unsigned b, unsigned a = 255) {
    return static_cast<ImU32>((a << 24) | (b << 16) | (g << 8) | r);
}

constexpr ImU32 kPalette[] = {rgba(230, 80, 70), rgba(90, 190, 90), rgba(80, 140, 240), rgba(240, 180, 50), rgba(190, 100, 220), rgba(60, 200, 200)};

std::string windowName(const vz::VizWindow& w) {
    std::string title = w.title;
    if (title.empty()) {
        switch (w.type) {
            case vz::WindowType::Topics: title = tr(TextId::PanelRos2); break;
            case vz::WindowType::Message: title = std::string{tr(TextId::Ros2Message)} + "  " + w.topic; break;
            case vz::WindowType::Plot: title = tr(TextId::VizPlotTitle); break;
            case vz::WindowType::Image: title = std::string{tr(TextId::Ros2Image)} + "  " + w.topic; break;
            case vz::WindowType::Tf: title = tr(TextId::Ros2Tf); break;
            case vz::WindowType::Unknown: break;
        }
    }
    return title + "###viz:" + vz::windowId(w);
}

std::string formatNumber(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.6g", v);
    return buf;
}

#ifdef TERRA_WITH_ROS2

std::string formatBytes(double bytesPerSecond) {
    char buf[64];
    if (bytesPerSecond >= 1e6) std::snprintf(buf, sizeof buf, "%.1f MB/s", bytesPerSecond / 1e6);
    else if (bytesPerSecond >= 1e3) std::snprintf(buf, sizeof buf, "%.1f KB/s", bytesPerSecond / 1e3);
    else std::snprintf(buf, sizeof buf, "%.0f B/s", bytesPerSecond);
    return buf;
}

bool isImageType(const std::string& type) { return type == "sensor_msgs/msg/Image" || type == "sensor_msgs/msg/CompressedImage"; }

std::vector<std::uint8_t> decodeCompressed(const std::uint8_t* data, std::size_t size, int& width, int& height) {
    auto r = terra::render::decodeImage(data, size, "CompressedImage");
    if (!r.ok()) return {};
    width = r.image.width;
    height = r.image.height;
    return std::move(r.image.pixels);
}

terra::ros2::Monitor* monitorOf(VizPanelState& s) {
    if (!s.monitorTried) {
        s.monitorTried = true;
        s.monitor = terra::ros2::Monitor::create(s.monitorError, decodeCompressed);
        if (s.monitor) std::printf("INFO ROS2 监视器已启动\n");
        else std::printf("INFO %s%s\n", tr(TextId::Ros2MonitorFailed), s.monitorError.c_str());
    }
    return s.monitor.get();
}

// 按配置同步订阅：打开的消息与图像窗口、曲线中的 ROS2 字段、TF 树
void syncSubscriptions(VizPanelState& s) {
    std::set<std::string> topics;
    std::set<std::pair<std::string, std::string>> fields;
    bool tf = false;
    bool any = false;
    for (const auto& w : s.config.windows) {
        if (w.type == vz::WindowType::Message || w.type == vz::WindowType::Image) topics.insert(w.topic);
        if (w.type == vz::WindowType::Tf) tf = true;
        if (w.type == vz::WindowType::Topics) any = true;
        if (w.type == vz::WindowType::Plot) {
            for (const auto& series : w.series) {
                if (series.source == "ros2") fields.emplace(series.topic, series.field);
            }
        }
    }
    any = any || tf || !topics.empty() || !fields.empty();
    if (!any && !s.monitor) return;  // 没有 ROS2 窗口时不启动监视器
    auto* m = monitorOf(s);
    if (m == nullptr) return;
    for (auto it = s.watched.begin(); it != s.watched.end();) {
        if (topics.count(*it) == 0) {
            m->unwatch(*it);
            it = s.watched.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto& t : topics) {
        if (s.watched.insert(t).second) m->watch(t);
    }
    for (auto it = s.fields.begin(); it != s.fields.end();) {
        if (fields.count(*it) == 0) {
            m->removeField(it->first, it->second);
            it = s.fields.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto& f : fields) {
        if (s.fields.insert(f).second) m->addField(f.first, f.second);
    }
    if (tf != s.tfWatched) {
        m->watchTf(tf);
        s.tfWatched = tf;
    }
}

// ---- 消息树 ----

void drawValue(VizPanelState& s, const std::string& topic, const terra::ros2::MessageValue& v, const std::string& path, int pageSize, int depth) {
    using Kind = terra::ros2::MessageValue::Kind;
    ImGui::PushID(path.c_str());
    if (v.kind == Kind::Message) {
        const ImGuiTreeNodeFlags flags = depth == 0 ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None;
        if (ImGui::TreeNodeEx("##m", flags, "%s", v.name.c_str())) {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", v.type.c_str());
            for (const auto& c : v.children) drawValue(s, topic, c, path.empty() ? c.name : path + "." + c.name, pageSize, depth + 1);
            ImGui::TreePop();
        } else {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", v.type.c_str());
        }
    } else if (v.kind == Kind::Array) {
        if (ImGui::TreeNodeEx("##a", ImGuiTreeNodeFlags_None, "%s", v.name.c_str())) {
            ImGui::SameLine();
            ImGui::TextDisabled("%s  [%zu]", v.type.c_str(), v.arraySize);
            const int pages = std::max(1, static_cast<int>((v.children.size() + static_cast<std::size_t>(pageSize) - 1) / static_cast<std::size_t>(pageSize)));
            int& page = s.pages[topic + path];
            page = std::clamp(page, 0, pages - 1);
            if (pages > 1) {
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.0f);
                int shown = page + 1;
                if (ImGui::InputInt(tr(TextId::Ros2Page), &shown)) page = std::clamp(shown - 1, 0, pages - 1);
                ImGui::SameLine();
                ImGui::TextDisabled("/ %d", pages);
            }
            const std::size_t begin = static_cast<std::size_t>(page) * static_cast<std::size_t>(pageSize);
            const std::size_t end = std::min(v.children.size(), begin + static_cast<std::size_t>(pageSize));
            for (std::size_t i = begin; i < end; ++i) drawValue(s, topic, v.children[i], path + v.children[i].name, pageSize, depth + 1);
            if (v.children.size() < v.arraySize) ImGui::TextDisabled("… %zu / %zu", v.children.size(), v.arraySize);
            ImGui::TreePop();
        } else {
            ImGui::SameLine();
            ImGui::TextDisabled("%s  [%zu]", v.type.c_str(), v.arraySize);
        }
    } else {
        ImGui::Bullet();
        ImGui::SameLine();
        if (v.kind == Kind::String) {
            ImGui::Text("%s = \"%s\"", v.name.c_str(), v.text.c_str());
        } else if (v.kind == Kind::Bool) {
            ImGui::Text("%s = %s", v.name.c_str(), v.boolean ? "true" : "false");
        } else {
            ImGui::Text("%s = %s", v.name.c_str(), formatNumber(v.number).c_str());
        }
        if (v.kind != Kind::String) {
            ImGui::SameLine();
            if (ImGui::SmallButton("+")) {
                vz::PlotSeries series;
                series.topic = topic;
                series.field = path;
                addSeries(s, series);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr(TextId::Ros2AddToPlot));
        }
    }
    ImGui::PopID();
}

void messageWindow(VizPanelState& s, vz::VizWindow& w) {
    auto* m = monitorOf(s);
    if (m == nullptr) return;
    const auto st = m->stats(w.topic);
    const auto info = m->topic(w.topic);
    ImGui::TextDisabled("%s  ·  %.1f Hz  ·  %s  ·  %llu", info ? info->type.c_str() : "?", st.hz, formatBytes(st.bytesPerSecond).c_str(),
                        static_cast<unsigned long long>(st.messages));
    if (const std::string e = m->typeError(w.topic); !e.empty()) {
        ImGui::TextWrapped("%s", e.c_str());
        return;
    }
    const auto v = m->latestMessage(w.topic, 1000);
    if (!v) {
        ImGui::TextDisabled("%s", tr(TextId::Ros2Waiting));
        return;
    }
    ++s.messageTrees;
    for (const auto& c : v->children) drawValue(s, w.topic, c, c.name, w.pageSize, 1);
}

void imageWindow(VizPanelState& s, vz::VizWindow& w) {
    auto* m = monitorOf(s);
    if (m == nullptr) return;
    auto& view = s.images[vz::windowId(w)];
    if (auto frame = m->latestImage(w.topic, view.sequence)) {
        view.sequence = frame->sequence;
        view.error = frame->error;
        if (frame->error.empty()) {
            terra::render::ImageRgba8 image;
            if (!frame->depth.empty()) {
                const float lo = static_cast<float>(w.depthRange[0]), hi = static_cast<float>(w.depthRange[1]);
                if (w.colormap == "gray") {
                    image.width = frame->width;
                    image.height = frame->height;
                    image.pixels.assign(frame->depth.size() * 4, 255);
                    for (std::size_t i = 0; i < frame->depth.size(); ++i) {
                        const float d = frame->depth[i];
                        const float t = std::isfinite(d) ? std::clamp((d - lo) / std::max(hi - lo, 1e-6f), 0.0f, 1.0f) : 0.0f;
                        const auto g = static_cast<std::uint8_t>(std::lround(255.0f * (1.0f - t)));
                        image.pixels[i * 4] = image.pixels[i * 4 + 1] = image.pixels[i * 4 + 2] = std::isfinite(d) ? g : 0;
                    }
                } else {
                    image = depthPreview(frame->depth, frame->width, frame->height, lo, hi);
                }
            } else {
                image.width = frame->width;
                image.height = frame->height;
                image.pixels = std::move(frame->rgba);
            }
            if (!image.empty() && image.pixels.size() == static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4) {
                view.texture = terra::render::Texture2D::create(image.width, image.height, image.pixels.data(), false);
                ++s.imageUploads;
            }
            view.info = std::to_string(frame->width) + " × " + std::to_string(frame->height) + "  ·  " + frame->encoding;
        }
    }
    const auto st = m->stats(w.topic);
    ImGui::TextDisabled("%s  ·  %.1f Hz%s", view.info.c_str(), st.hz,
                        st.lastReceive >= 0 ? ("  ·  " + std::string{tr(TextId::Ros2Latency)} + " " +
                                               formatNumber(std::round((m->now() - st.lastReceive) * 1000.0)) + " ms")
                                                  .c_str()
                                            : "");
    if (!view.error.empty()) ImGui::TextWrapped("%s", view.error.c_str());
    if (view.texture.name() == 0) {
        ImGui::TextDisabled("%s", tr(TextId::Ros2Waiting));
        return;
    }
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float iw = static_cast<float>(view.texture.width()), ih = static_cast<float>(view.texture.height());
    ImVec2 size(iw, ih);
    if (w.fit == "fit" && iw > 0 && ih > 0) {
        const float k = std::max(0.01f, std::min(avail.x / iw, avail.y / ih));
        size = ImVec2(iw * k, ih * k);
    }
    ImGui::Image(static_cast<ImTextureID>(view.texture.name()), size);
}

void tfNode(const std::map<std::string, std::vector<const terra::ros2::TransformInfo*>>& children, const std::string& frame,
            const terra::ros2::TransformInfo* t, double now, int depth) {
    if (depth > 64) return;
    const auto it = children.find(frame);
    const bool leaf = it == children.end();
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_DefaultOpen | (leaf ? ImGuiTreeNodeFlags_Leaf : ImGuiTreeNodeFlags_None);
    const bool stale = t != nullptr && !t->isStatic && now - t->receive > 1.0;
    if (stale) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.8f, 0.3f, 1.0f));
    const bool open = ImGui::TreeNodeEx(frame.c_str(), flags, "%s", frame.c_str());
    if (stale) ImGui::PopStyleColor();
    if (t != nullptr) {
        // 四元数 → 欧拉角（ZYX，度）
        const double x = t->rotation[0], y = t->rotation[1], z = t->rotation[2], qw = t->rotation[3];
        const double roll = std::atan2(2 * (qw * x + y * z), 1 - 2 * (x * x + y * y)) * 57.29577951308232;
        const double pitch = std::asin(std::clamp(2 * (qw * y - z * x), -1.0, 1.0)) * 57.29577951308232;
        const double yaw = std::atan2(2 * (qw * z + x * y), 1 - 2 * (y * y + z * z)) * 57.29577951308232;
        ImGui::SameLine();
        ImGui::TextDisabled("(%.3f, %.3f, %.3f)  rpy (%.1f°, %.1f°, %.1f°)  %s", t->translation[0], t->translation[1], t->translation[2], roll, pitch,
                            yaw, t->isStatic ? tr(TextId::Ros2Static) : (formatNumber(std::round((now - t->receive) * 1000.0)) + " ms").c_str());
        if (stale && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr(TextId::Ros2Stale));
    }
    if (!open) return;
    if (!leaf) {
        for (const auto* c : it->second) tfNode(children, c->child, c, now, depth + 1);
    }
    ImGui::TreePop();
}

void tfWindow(VizPanelState& s, vz::VizWindow& w) {
    auto* m = monitorOf(s);
    if (m == nullptr) return;
    const auto all = m->transforms();
    std::map<std::string, std::vector<const terra::ros2::TransformInfo*>> children;
    std::set<std::string> childFrames;
    for (const auto& t : all) {
        children[t.parent].push_back(&t);
        childFrames.insert(t.child);
    }
    for (auto& [parent, list] : children) {
        std::sort(list.begin(), list.end(), [](const auto* a, const auto* b) { return a->child < b->child; });
    }
    if (all.empty()) {
        ImGui::TextDisabled("%s", tr(TextId::Ros2Waiting));
        return;
    }
    const double now = m->now();
    if (!w.tfRoot.empty()) {
        tfNode(children, w.tfRoot, nullptr, now, 0);
        return;
    }
    for (const auto& [parent, list] : children) {
        if (childFrames.count(parent) == 0) tfNode(children, parent, nullptr, now, 0);
    }
}

// 按类型给出的一键曲线
std::vector<std::string> presetFields(const std::string& type) {
    if (type == "sensor_msgs/msg/Imu") return {"angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};
    if (type == "nav_msgs/msg/Odometry") return {"twist.twist.linear.x", "twist.twist.angular.z"};
    if (type == "geometry_msgs/msg/Twist") return {"linear.x", "angular.z"};
    if (type.rfind("std_msgs/msg/Float", 0) == 0 || type.rfind("std_msgs/msg/Int", 0) == 0 || type.rfind("std_msgs/msg/UInt", 0) == 0) return {"data"};
    return {};
}

#endif  // TERRA_WITH_ROS2

void displayToolbar(VizPanelState& s, const VizContext& ctx, std::vector<VizAction>& actions) {
    const std::string current = s.config.name.empty() || s.path.empty() ? std::string{tr(TextId::VizDisplayNone)} : s.config.name;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(tr(TextId::VizDisplay));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.0f);
    if (ImGui::BeginCombo("##display", (current + (s.dirty ? " *" : "")).c_str())) {
        for (const auto& [name, file] : ctx.displays) {
            if (ImGui::Selectable(name.c_str(), file == s.path)) actions.push_back({VizAction::Type::Switch, file});
        }
        ImGui::EndCombo();
    }
    if (s.dirty && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr(TextId::VizUnsaved));
    ImGui::SameLine();
    if (ImGui::Button(tr(TextId::VizSave))) actions.push_back({VizAction::Type::Save, {}});
    ImGui::SameLine();
    if (ImGui::Button(tr(TextId::VizSaveAs))) {
        s.saveAsOpen = true;
        std::snprintf(s.saveName.data(), s.saveName.size(), "%s", s.config.name.c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button(tr(TextId::VizExport))) s.exportOpen = true;
    ImGui::SameLine();
    if (ImGui::Button(tr(TextId::VizImport))) actions.push_back({VizAction::Type::Import, {}});
}

void topicsWindow(VizPanelState& s, vz::VizWindow& w, const VizContext& ctx, std::vector<VizAction>& actions) {
    displayToolbar(s, ctx, actions);
    ImGui::Separator();
#ifdef TERRA_WITH_ROS2
    auto* m = monitorOf(s);
    if (m == nullptr) {
        ImGui::TextWrapped("%s%s", tr(TextId::Ros2MonitorFailed), s.monitorError.c_str());
        return;
    }
    if (w.filter.size() < s.topicFilter.size() && std::string{s.topicFilter.data()} != w.filter && s.topicFilter[0] == '\0') {
        std::snprintf(s.topicFilter.data(), s.topicFilter.size(), "%s", w.filter.c_str());
    }
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::InputTextWithHint("##topicFilter", tr(TextId::Ros2Filter), s.topicFilter.data(), s.topicFilter.size())) {
        w.filter = s.topicFilter.data();
        s.dirty = true;
    }
    std::string filter = s.topicFilter.data();
    std::transform(filter.begin(), filter.end(), filter.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const auto matches = [&](const std::string& text) {
        std::string lower = text;
        std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return filter.empty() || lower.find(filter) != std::string::npos;
    };
    if (!ImGui::BeginTabBar("##ros2Tabs")) return;
    if (ImGui::BeginTabItem(tr(TextId::Ros2Topics))) {
        auto topics = m->topics();
        if (topics.empty()) ImGui::TextDisabled("%s", tr(TextId::Ros2NoTopics));
        const ImGuiTableFlags flags = ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Sortable |
                                      ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp;
        if (!topics.empty() && ImGui::BeginTable("##topics", 7, flags)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn(tr(TextId::Ros2Topics), ImGuiTableColumnFlags_DefaultSort, 3.0f);
            ImGui::TableSetupColumn(tr(TextId::Ros2Type), ImGuiTableColumnFlags_None, 2.6f);
            ImGui::TableSetupColumn(tr(TextId::Ros2Publishers), ImGuiTableColumnFlags_None, 0.6f);
            ImGui::TableSetupColumn(tr(TextId::Ros2Subscribers), ImGuiTableColumnFlags_None, 0.6f);
            ImGui::TableSetupColumn(tr(TextId::Ros2Rate), ImGuiTableColumnFlags_None, 0.8f);
            ImGui::TableSetupColumn(tr(TextId::Ros2Bandwidth), ImGuiTableColumnFlags_None, 1.0f);
            ImGui::TableSetupColumn("##actions", ImGuiTableColumnFlags_NoSort, 2.4f);
            ImGui::TableHeadersRow();
            std::map<std::string, terra::ros2::TopicStats> stats;
            for (const auto& t : topics) stats[t.name] = m->stats(t.name);
            if (ImGuiTableSortSpecs* sort = ImGui::TableGetSortSpecs(); sort != nullptr && sort->SpecsCount > 0) {
                const ImGuiTableColumnSortSpecs spec = sort->Specs[0];
                const bool up = spec.SortDirection == ImGuiSortDirection_Ascending;
                const auto before = [&](const terra::ros2::TopicInfo& a, const terra::ros2::TopicInfo& b) {
                    switch (spec.ColumnIndex) {
                        case 1: return a.type < b.type;
                        case 2: return a.publishers < b.publishers;
                        case 3: return a.subscribers < b.subscribers;
                        case 4: return stats[a.name].hz < stats[b.name].hz;
                        case 5: return stats[a.name].bytesPerSecond < stats[b.name].bytesPerSecond;
                        default: return a.name < b.name;
                    }
                };
                std::stable_sort(topics.begin(), topics.end(), [&](const terra::ros2::TopicInfo& a, const terra::ros2::TopicInfo& b) {
                    return up ? before(a, b) : before(b, a);
                });
            }
            for (const auto& t : topics) {
                if (!matches(t.name) && !matches(t.type)) continue;
                ImGui::PushID(t.name.c_str());
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(t.name.c_str());
                if (ImGui::IsItemHovered() && !t.qos.empty()) ImGui::SetTooltip("%s", t.qos.c_str());
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", t.type.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%zu", t.publishers);
                ImGui::TableNextColumn();
                ImGui::Text("%zu", t.subscribers);
                const auto& st = stats[t.name];
                const bool watched = m->watching(t.name);
                ImGui::TableNextColumn();
                if (watched) ImGui::Text("%.1f", st.hz);
                else ImGui::TextDisabled("—");
                ImGui::TableNextColumn();
                if (watched) ImGui::TextUnformatted(formatBytes(st.bytesPerSecond).c_str());
                else ImGui::TextDisabled("—");
                ImGui::TableNextColumn();
                // 一键打开（M4-08c 起的原则：常用的事情在行上点一下就能完成）
                if (ImGui::SmallButton(tr(TextId::Ros2Message))) {
                    vz::VizWindow nw;
                    nw.type = vz::WindowType::Message;
                    nw.topic = t.name;
                    openWindow(s, nw);
                }
                if (isImageType(t.type)) {
                    ImGui::SameLine();
                    if (ImGui::SmallButton(tr(TextId::Ros2Image))) {
                        vz::VizWindow nw;
                        nw.type = vz::WindowType::Image;
                        nw.topic = t.name;
                        openWindow(s, nw);
                    }
                }
                if (t.type == "tf2_msgs/msg/TFMessage") {
                    ImGui::SameLine();
                    if (ImGui::SmallButton(tr(TextId::Ros2Tf))) {
                        vz::VizWindow nw;
                        nw.type = vz::WindowType::Tf;
                        openWindow(s, nw);
                    }
                }
                if (const auto preset = presetFields(t.type); !preset.empty()) {
                    ImGui::SameLine();
                    if (ImGui::SmallButton(tr(TextId::Ros2Plot))) {
                        for (const auto& field : preset) {
                            vz::PlotSeries series;
                            series.topic = t.name;
                            series.field = field;
                            addSeries(s, series, t.name);
                        }
                    }
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(tr(TextId::Ros2Nodes))) {
        for (const auto& n : m->nodes()) {
            if (!matches(n.name)) continue;
            if (ImGui::TreeNode(n.name.c_str())) {
                for (const auto& p : n.publishes) ImGui::BulletText("%s%s", tr(TextId::Ros2Publishes), p.c_str());
                for (const auto& p : n.subscribes) ImGui::BulletText("%s%s", tr(TextId::Ros2Subscribes), p.c_str());
                ImGui::TreePop();
            }
        }
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
#else
    (void)w;
    ImGui::TextWrapped("%s", tr(TextId::Ros2Unavailable));
#endif
}

// ---- 曲线 ----

void plotWindow(VizPanelState& s, vz::VizWindow& w, const VizContext& ctx) {
    const std::string id = vz::windowId(w);
    bool& paused = s.paused[id];
    ImGui::Checkbox(tr(TextId::VizPause), &paused);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.0f);
    static constexpr double kSpans[] = {5.0, 10.0, 30.0, 60.0};
    if (ImGui::BeginCombo(tr(TextId::VizSeconds), (formatNumber(w.seconds) + " s").c_str())) {
        for (const double span : kSpans) {
            if (ImGui::Selectable((formatNumber(span) + " s").c_str(), span == w.seconds)) {
                w.seconds = span;
                s.dirty = true;
            }
        }
        ImGui::EndCombo();
    }
    const bool hasRos2 = std::any_of(w.series.begin(), w.series.end(), [](const vz::PlotSeries& x) { return x.source == "ros2"; });
    if (hasRos2) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.0f);
        const bool header = w.timeBase == "header";
        if (ImGui::BeginCombo("##timeBase", tr(header ? TextId::VizTimeHeader : TextId::VizTimeReceive))) {
            if (ImGui::Selectable(tr(TextId::VizTimeReceive), !header)) {
                w.timeBase = "receive";
                s.dirty = true;
            }
            if (ImGui::Selectable(tr(TextId::VizTimeHeader), header)) {
                w.timeBase = "header";
                s.dirty = true;
            }
            ImGui::EndCombo();
        }
    }
    // 取数据：每条曲线相对各自来源的“现在”（ROS2 接收时间、消息头时间、仿真时间不能混在一个时间轴上）
    struct Line {
        std::vector<std::pair<double, double>> points;  // 相对时间（≤ 0）、值
        std::string label;
        std::string error;
        ImU32 color = 0;
        double latest = std::numeric_limits<double>::quiet_NaN();
    };
    static std::map<std::string, std::vector<Line>> frozen;  // 暂停时保留的画面
    std::vector<Line> lines;
    if (paused && frozen.count(id)) {
        lines = frozen[id];
    } else {
        std::vector<std::vector<std::pair<double, double>>> raw(w.series.size());
        double rosRef = -std::numeric_limits<double>::infinity(), simRef = -std::numeric_limits<double>::infinity();
        std::vector<std::string> errors(w.series.size());
        for (std::size_t k = 0; k < w.series.size(); ++k) {
            const auto& x = w.series[k];
            if (x.source == "sim") {
                if (ctx.sim && ctx.sim->samples) raw[k] = ctx.sim->samples(x.sensor, x.field);
                for (const auto& p : raw[k]) simRef = std::max(simRef, p.first);
                continue;
            }
#ifdef TERRA_WITH_ROS2
            if (auto* m = monitorOf(s)) {
                const bool header = w.timeBase == "header";
                for (const auto& sample : m->samples(x.topic, x.field, m->now() - w.seconds - 1.0)) {
                    const double t = header ? sample.stamp : sample.receive;
                    if (std::isfinite(t)) raw[k].emplace_back(t, sample.value);
                }
                errors[k] = m->fieldError(x.topic, x.field);
                rosRef = header ? std::max(rosRef, raw[k].empty() ? rosRef : raw[k].back().first) : m->now();
            }
#else
            errors[k] = tr(TextId::Ros2Unavailable);
#endif
        }
        for (std::size_t k = 0; k < w.series.size(); ++k) {
            const auto& x = w.series[k];
            Line line;
            line.label = !x.label.empty() ? x.label
                         : x.source == "sim" ? std::string{tr(TextId::VizSimSource)} + " " + x.sensor + " " + x.field
                                             : x.topic + " " + x.field;
            line.color = x.color ? ImGui::ColorConvertFloat4ToU32(ImVec4((*x.color)[0], (*x.color)[1], (*x.color)[2], (*x.color)[3]))
                                 : kPalette[k % (sizeof(kPalette) / sizeof(kPalette[0]))];
            line.error = errors[k];
            const double ref = x.source == "sim" ? simRef : rosRef;
            for (const auto& [t, v] : raw[k]) {
                const double rel = t - ref;
                if (rel < -w.seconds || rel > 0.5) continue;
                line.points.emplace_back(rel, v * x.scale + x.offset);
            }
            if (!line.points.empty()) line.latest = line.points.back().second;
            lines.push_back(std::move(line));
        }
        frozen[id] = lines;
    }
    // 绘制
    const float font = ImGui::GetFontSize();
    const float legend = static_cast<float>(w.series.size()) * ImGui::GetFrameHeightWithSpacing();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 size(std::max(avail.x, font * 10.0f), std::max(avail.y - legend - font * 0.5f, font * 6.0f));
    const ImVec2 a = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##plot", size);
    const ImVec2 b(a.x + size.x, a.y + size.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(a, b, ImGui::GetColorU32(ImGuiCol_FrameBg), font * 0.25f);
    double lo = std::numeric_limits<double>::infinity(), hi = -std::numeric_limits<double>::infinity();
    for (const auto& l : lines) {
        for (const auto& p : l.points) {
            lo = std::min(lo, p.second);
            hi = std::max(hi, p.second);
        }
    }
    if (w.yRange) {
        lo = (*w.yRange)[0];
        hi = (*w.yRange)[1];
    } else if (!std::isfinite(lo)) {
        lo = -1.0;
        hi = 1.0;
    } else {
        const double pad = std::max((hi - lo) * 0.08, 1e-6);
        lo -= pad;
        hi += pad;
    }
    const float left = a.x + font * 3.6f, right = b.x - font * 0.4f, top = a.y + font * 0.4f, bottom = b.y - font * 1.2f;
    for (int g = 0; g <= 4; ++g) {
        const float y = top + (bottom - top) * static_cast<float>(g) / 4.0f;
        dl->AddLine(ImVec2(left, y), ImVec2(right, y), ImGui::GetColorU32(ImGuiCol_Border), 1.0f);
        const double value = hi - (hi - lo) * g / 4.0;
        dl->AddText(ImGui::GetFont(), font * 0.75f, ImVec2(a.x + font * 0.2f, y - font * 0.4f), ImGui::GetColorU32(ImGuiCol_TextDisabled),
                    formatNumber(value).c_str());
    }
    dl->AddText(ImGui::GetFont(), font * 0.75f, ImVec2(left, bottom + font * 0.15f), ImGui::GetColorU32(ImGuiCol_TextDisabled),
                ("-" + formatNumber(w.seconds) + " s").c_str());
    dl->AddText(ImGui::GetFont(), font * 0.75f, ImVec2(right - font * 1.0f, bottom + font * 0.15f), ImGui::GetColorU32(ImGuiCol_TextDisabled), "0");
    dl->PushClipRect(ImVec2(left, top), ImVec2(right, bottom), true);
    for (const auto& l : lines) {
        std::vector<ImVec2> pts;
        pts.reserve(l.points.size());
        for (const auto& [t, v] : l.points) {
            const float x = left + (right - left) * static_cast<float>((t + w.seconds) / w.seconds);
            const float y = bottom - (bottom - top) * static_cast<float>((v - lo) / (hi - lo));
            pts.emplace_back(x, y);
        }
        s.plotPoints += pts.size();
        if (pts.size() >= 2) dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), l.color, std::max(1.0f, font * 0.1f), ImDrawFlags_None);
    }
    dl->PopClipRect();
    // 图例：颜色、名字、最新值、问题；可移除
    std::optional<std::size_t> remove;
    for (std::size_t k = 0; k < lines.size(); ++k) {
        ImGui::PushID(static_cast<int>(k));
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const float box = ImGui::GetTextLineHeight();
        ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(p.x, p.y + box * 0.2f), ImVec2(p.x + box * 0.6f, p.y + box * 0.8f), lines[k].color);
        ImGui::Dummy(ImVec2(box * 0.7f, box));
        ImGui::SameLine();
        ImGui::TextUnformatted(lines[k].label.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("%s", std::isfinite(lines[k].latest) ? formatNumber(lines[k].latest).c_str() : "—");
        if (!lines[k].error.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.95f, 0.6f, 0.3f, 1.0f), "%s", lines[k].error.c_str());
        }
        ImGui::SameLine();
        if (ImGui::SmallButton(tr(TextId::VizRemove))) remove = k;
        ImGui::PopID();
    }
    if (remove && *remove < w.series.size()) {
        w.series.erase(w.series.begin() + static_cast<std::ptrdiff_t>(*remove));
        s.dirty = true;
    }
}

// ---- 弹窗：另存为、导出 ----

void saveAsPopup(VizPanelState& s, std::vector<VizAction>& actions) {
    const std::string title = std::string{tr(TextId::VizSaveAs)} + "###VizSaveAs";
    if (s.saveAsOpen) {
        ImGui::OpenPopup(title.c_str());
        s.saveAsOpen = false;
    }
    if (!ImGui::BeginPopupModal(title.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 18.0f);
    ImGui::InputText(tr(TextId::VizName), s.saveName.data(), s.saveName.size());
    const std::string name = s.saveName.data();
    ImGui::BeginDisabled(name.empty());
    if (ImGui::Button(tr(TextId::VizSave))) {
        actions.push_back({VizAction::Type::SaveAs, name});
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button(tr(TextId::ImportCancel)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void runExport(VizPanelState& s) {
    s.exportReport.clear();
    capturePlacement(s);
    const auto ctx = exportContext(s);
    const std::string dir = s.exportDir.data();
    const std::string name = s.exportName.data();
    for (const vz::Exporter* e : vz::exporters()) {
        if (!s.exportChosen[e->id()]) continue;
        const std::string out = (fs::path{dir} / (name + e->suffix())).string();
        const auto r = vz::exportToFile(*e, s.config, s.exportValues[e->id()], ctx, out);
        if (r.ok) {
            s.exportReport.push_back(std::string{tr(TextId::LogVizExported)} + out);
            std::printf("INFO 显示配置已导出：%s（没有导出的部分 %zu 项）\n", out.c_str(), r.notExported.size());
        } else {
            s.exportReport.push_back(std::string{tr(TextId::LogVizFailed)} + r.error);
            std::printf("INFO 显示配置导出失败：%s\n", r.error.c_str());
        }
    }
}

void exportPopup(VizPanelState& s, const VizContext& ctx) {
    const std::string title = std::string{tr(TextId::VizExportTitle)} + "###VizExport";
    const auto prepare = [&] {
        std::snprintf(s.exportName.data(), s.exportName.size(), "%s", s.config.name.empty() ? "display" : s.config.name.c_str());
        std::snprintf(s.exportDir.data(), s.exportDir.size(), "%s", ctx.defaultExportDir.c_str());
        for (const vz::Exporter* e : vz::exporters()) {
            if (!s.exportChosen.count(e->id())) s.exportChosen[e->id()] = e->id() == "terviz" || e->id() == "rviz";
        }
        s.exportReport.clear();
    };
    if (s.exportSelfTest) {  // 自检：按默认值导出全部内置格式
        prepare();
        runExport(s);
        s.exportSelfTest = false;
    }
    if (s.exportOpen) {
        prepare();
        ImGui::OpenPopup(title.c_str());
        s.exportOpen = false;
    }
    if (!ImGui::BeginPopupModal(title.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
    const float font = ImGui::GetFontSize();
    const bool english = terra::ui::language() == terra::ui::Language::English;
    ImGui::SetNextItemWidth(font * 20.0f);
    ImGui::InputText(tr(TextId::VizName), s.exportName.data(), s.exportName.size());
    ImGui::SetNextItemWidth(font * 30.0f);
    ImGui::InputText(tr(TextId::VizExportDir), s.exportDir.data(), s.exportDir.size());
    const auto ctxExport = exportContext(s);
    // 导出格式来自注册表；各格式的参数按清单生成控件（设计稿 4.1 节：以后增加格式不改这里）
    for (const vz::Exporter* e : vz::exporters()) {
        ImGui::PushID(e->id().c_str());
        ImGui::Separator();
        bool chosen = s.exportChosen[e->id()];
        const std::string label = (english ? e->name().english : e->name().chinese) + "（" + e->suffix() + "）";
        if (ImGui::Checkbox(label.c_str(), &chosen)) s.exportChosen[e->id()] = chosen;
        if (chosen) {
            ImGui::Indent();
            const auto specs = e->parameters(s.config, ctxExport);
            auto& values = s.exportValues[e->id()];
            values = vz::withDefaults(specs, values);
            for (const auto& spec : specs) {
                ImGui::PushID(spec.key.c_str());
                const std::string specLabel = english ? spec.label.english : spec.label.chinese;
                std::string& value = values[spec.key];
                switch (spec.kind) {
                    case vz::ParameterSpec::Kind::Bool: {
                        bool on = value == "true";
                        if (ImGui::Checkbox(specLabel.c_str(), &on)) value = on ? "true" : "false";
                        break;
                    }
                    case vz::ParameterSpec::Kind::Number: {
                        double d = std::strtod(value.c_str(), nullptr);
                        ImGui::SetNextItemWidth(font * 8.0f);
                        if (ImGui::InputDouble(specLabel.c_str(), &d, 0.0, 0.0, "%g")) value = formatNumber(d);
                        break;
                    }
                    case vz::ParameterSpec::Kind::Choice:
                    case vz::ParameterSpec::Kind::Text: {
                        std::array<char, 256> buf{};
                        std::snprintf(buf.data(), buf.size(), "%s", value.c_str());
                        ImGui::SetNextItemWidth(font * 12.0f);
                        if (ImGui::InputText(specLabel.c_str(), buf.data(), buf.size())) value = buf.data();
                        if (!spec.choices.empty()) {
                            ImGui::SameLine();
                            if (ImGui::BeginCombo("##choices", "", ImGuiComboFlags_NoPreview)) {
                                for (const auto& c : spec.choices) {
                                    if (ImGui::Selectable(c.c_str(), c == value)) value = c;
                                }
                                ImGui::EndCombo();
                            }
                        }
                        break;
                    }
                }
                const std::string help = english ? spec.help.english : spec.help.chinese;
                if (!help.empty() && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", help.c_str());
                ImGui::PopID();
            }
            for (const auto& error : vz::validateParameters(specs, values)) ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", error.c_str());
            const auto skipped = e->notExported(s.config, ctxExport);
            if (!skipped.empty()) {
                ImGui::TextDisabled("%s%zu", tr(TextId::VizNotExported), skipped.size());
                for (const auto& n : skipped) ImGui::BulletText("%s", n.c_str());
            }
            ImGui::Unindent();
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    for (const auto& line : s.exportReport) ImGui::TextUnformatted(line.c_str());
    if (ImGui::Button(tr(TextId::VizExportRun))) runExport(s);
    ImGui::SameLine();
    if (ImGui::Button(tr(TextId::Close)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

}  // namespace

bool ros2PanelAvailable() {
#ifdef TERRA_WITH_ROS2
    return true;
#else
    return false;
#endif
}

std::string openWindow(VizPanelState& s, vz::VizWindow window) {
    const std::string id = vz::windowId(window);
    for (const auto& w : s.config.windows) {
        if (w.type != vz::WindowType::Unknown && vz::windowId(w) == id) {
            ImGui::SetWindowFocus(windowName(w).c_str());
            return id;
        }
    }
    s.config.windows.push_back(std::move(window));
    s.dirty = true;
    return id;
}

void addSeries(VizPanelState& s, const vz::PlotSeries& series, const std::string& plotTitle) {
    vz::VizWindow* plot = nullptr;
    for (auto& w : s.config.windows) {
        if (w.type != vz::WindowType::Plot) continue;
        if (plotTitle.empty() || w.title == plotTitle) {
            plot = &w;
            break;
        }
    }
    if (plot == nullptr) {
        vz::VizWindow w;
        w.type = vz::WindowType::Plot;
        w.title = plotTitle.empty() ? std::string{tr(TextId::VizPlotTitle)} + " 1" : plotTitle;
        s.config.windows.push_back(std::move(w));
        plot = &s.config.windows.back();
    }
    for (const auto& x : plot->series) {
        if (x.source == series.source && x.topic == series.topic && x.sensor == series.sensor && x.field == series.field) return;
    }
    plot->series.push_back(series);
    s.dirty = true;
}

void applyConfig(VizPanelState& s, vz::VizConfig config, std::string path) {
    s.config = std::move(config);
    s.path = std::move(path);
    s.dirty = false;
    s.images.clear();
    s.paused.clear();
    s.pages.clear();
    s.topicFilter.fill('\0');
    s.pendingPlacement.clear();
    for (const auto& w : s.config.windows) {
        if (w.placement) s.pendingPlacement.insert(vz::windowId(w));
    }
}

void capturePlacement(VizPanelState& s) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    if (vp == nullptr || vp->Size.x <= 0 || vp->Size.y <= 0) return;
    for (auto& w : s.config.windows) {
        if (w.type == vz::WindowType::Unknown) continue;
        const ImGuiWindow* win = ImGui::FindWindowByName(("###viz:" + vz::windowId(w)).c_str());
        if (win == nullptr) continue;
        vz::Placement p;
        p.position = {static_cast<double>((win->Pos.x - vp->Pos.x) / vp->Size.x), static_cast<double>((win->Pos.y - vp->Pos.y) / vp->Size.y)};
        p.size = {static_cast<double>(win->Size.x / vp->Size.x), static_cast<double>(win->Size.y / vp->Size.y)};
        p.docked = win->DockIsActive;
        w.placement = p;
    }
}

vz::ExportContext exportContext(const VizPanelState& s) {
    vz::ExportContext ctx;
#ifdef TERRA_WITH_ROS2
    if (s.monitor) {
        for (const auto& t : s.monitor->topics()) {
            ctx.topicTypes[t.name] = t.type;
            ctx.bestEffort[t.name] = t.bestEffort;
        }
        // 坐标系：根在前
        std::set<std::string> children, parents;
        for (const auto& t : s.monitor->transforms()) {
            children.insert(t.child);
            parents.insert(t.parent);
        }
        for (const auto& p : parents) {
            if (!children.count(p)) ctx.frames.push_back(p);
        }
        for (const auto& c : children) ctx.frames.push_back(c);
    }
#else
    (void)s;
#endif
    return ctx;
}

std::vector<VizAction> drawVizWindows(VizPanelState& s, const VizContext& ctx) {
    std::vector<VizAction> actions;
    s.windowsDrawn = 0;
#ifdef TERRA_WITH_ROS2
    syncSubscriptions(s);
#endif
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float font = ImGui::GetFontSize();
    std::vector<std::string> closed;
    for (auto& w : s.config.windows) {
        if (w.type == vz::WindowType::Unknown) continue;
        const std::string id = vz::windowId(w);
        if (s.pendingPlacement.count(id) && w.placement && !w.placement->docked) {
            ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + static_cast<float>(w.placement->position[0]) * vp->Size.x,
                                           vp->Pos.y + static_cast<float>(w.placement->position[1]) * vp->Size.y),
                                    ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(static_cast<float>(w.placement->size[0]) * vp->Size.x, static_cast<float>(w.placement->size[1]) * vp->Size.y),
                                     ImGuiCond_Always);
        } else {
            ImGui::SetNextWindowSize(ImVec2(font * (w.type == vz::WindowType::Topics ? 46.0f : 28.0f), font * 18.0f), ImGuiCond_FirstUseEver);
        }
        s.pendingPlacement.erase(id);
        bool open = true;
        if (ImGui::Begin(windowName(w).c_str(), &open)) {
            ++s.windowsDrawn;
            switch (w.type) {
                case vz::WindowType::Topics: topicsWindow(s, w, ctx, actions); break;
                case vz::WindowType::Plot: plotWindow(s, w, ctx); break;
#ifdef TERRA_WITH_ROS2
                case vz::WindowType::Message: messageWindow(s, w); break;
                case vz::WindowType::Image: imageWindow(s, w); break;
                case vz::WindowType::Tf: tfWindow(s, w); break;
#else
                case vz::WindowType::Message:
                case vz::WindowType::Image:
                case vz::WindowType::Tf: ImGui::TextWrapped("%s", tr(TextId::Ros2Unavailable)); break;
#endif
                case vz::WindowType::Unknown: break;
            }
        }
        ImGui::End();
        if (!open) closed.push_back(id);
    }
    if (!closed.empty()) {
        auto& ws = s.config.windows;
        ws.erase(std::remove_if(ws.begin(), ws.end(),
                                [&closed](const vz::VizWindow& w) {
                                    return w.type != vz::WindowType::Unknown && std::find(closed.begin(), closed.end(), vz::windowId(w)) != closed.end();
                                }),
                 ws.end());
        for (const auto& id : closed) s.images.erase(id);
        s.dirty = true;
    }
    saveAsPopup(s, actions);
    exportPopup(s, ctx);
    return actions;
}

}  // namespace terra::viewer
