#include "terra/settings/settings.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <sstream>
#include <system_error>
#include <utility>

#include "nlohmann/json.hpp"

namespace terra::settings {

namespace {

namespace fs = std::filesystem;
namespace in = terra::input;
using Json = nlohmann::ordered_json;  // 保持字段顺序，文件便于阅读

constexpr std::uintmax_t kMaxFileBytes = 16u * 1024u * 1024u;
constexpr std::size_t kMaxLayoutBytes = 1024u * 1024u;

constexpr const char* kCommandNames[] = {
    "camera.forward", "camera.back", "camera.left", "camera.right", "camera.up",
    "camera.down",    "camera.boost", "camera.reset", "app.fullscreen",
};
static_assert(sizeof(kCommandNames) / sizeof(kCommandNames[0]) == kCommandCount, "kCommandNames 必须与 Command 一一对应");

std::string number(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

std::size_t ci(Command c) noexcept { return static_cast<std::size_t>(c); }

// 逐字段读取：缺失的字段保持默认值；类型不对或超出范围时保持默认值并记录警告。
class Reader {
public:
    explicit Reader(std::vector<std::string>& warnings) : warnings_(warnings) {}

    void warn(std::string message) { warnings_.push_back(std::move(message)); }

    // 不认识的字段：警告并忽略。
    void unknownFields(const Json& obj, const std::string& path, std::initializer_list<const char*> known) {
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            bool ok = false;
            for (const char* k : known) ok = ok || it.key() == k;
            if (!ok) warn("未知字段 " + join(path, it.key()) + "，已忽略");
        }
    }

    // 子对象；存在但不是对象时警告，返回空。
    const Json* object(const Json& parent, const char* key, const std::string& path) {
        const auto it = parent.find(key);
        if (it == parent.end()) return nullptr;
        if (!it->is_object()) {
            warn(join(path, key) + "：应为对象，整组使用默认值");
            return nullptr;
        }
        return &*it;
    }

    void real(const Json& obj, const char* key, const std::string& path, Range range, double& target) {
        const auto it = obj.find(key);
        if (it == obj.end()) return;
        const std::string where = join(path, key);
        if (!it->is_number()) {
            warn(where + "：应为数字，已使用默认值 " + number(target));
            return;
        }
        const double v = it->get<double>();
        if (!std::isfinite(v) || v < range.min || v > range.max) {
            warn(where + " = " + number(v) + " 超出范围 [" + number(range.min) + ", " + number(range.max) +
                 "]，已使用默认值 " + number(target));
            return;
        }
        target = v;
    }

    void integer(const Json& obj, const char* key, const std::string& path, int lo, int hi, int& target) {
        const auto it = obj.find(key);
        if (it == obj.end()) return;
        const std::string where = join(path, key);
        if (!it->is_number_integer()) {
            warn(where + "：应为整数，已使用默认值 " + std::to_string(target));
            return;
        }
        const std::int64_t v = it->get<std::int64_t>();
        if (v < lo || v > hi) {
            warn(where + " = " + std::to_string(v) + " 超出范围 [" + std::to_string(lo) + ", " + std::to_string(hi) +
                 "]，已使用默认值 " + std::to_string(target));
            return;
        }
        target = static_cast<int>(v);
    }

    void boolean(const Json& obj, const char* key, const std::string& path, bool& target) {
        const auto it = obj.find(key);
        if (it == obj.end()) return;
        if (!it->is_boolean()) {
            warn(join(path, key) + "：应为 true 或 false，已使用默认值 " + (target ? "true" : "false"));
            return;
        }
        target = it->get<bool>();
    }

    static std::string join(const std::string& path, const std::string& key) {
        return path.empty() ? key : path + "." + key;
    }

private:
    std::vector<std::string>& warnings_;
};

void readInterface(Reader& r, const Json& j, InterfaceSettings& s) {
    const std::string path = "interface";
    r.unknownFields(j, path, {"language", "uiScale", "panels", "layout"});
    if (const auto it = j.find("language"); it != j.end()) {
        const std::string value = it->is_string() ? it->get<std::string>() : std::string{};
        if (value == "zh" || value == "en") {
            s.language = value;
        } else {
            r.warn("interface.language：应为 \"zh\" 或 \"en\"，已使用默认值 \"" + s.language + "\"");
        }
    }
    if (const auto it = j.find("uiScale"); it != j.end()) {
        if (it->is_string() && it->get<std::string>() == "auto") {
            s.uiScale.reset();
        } else if (it->is_number()) {
            const double v = it->get<double>();
            if (std::isfinite(v) && v >= InterfaceSettings::kMinScale && v <= InterfaceSettings::kMaxScale) {
                s.uiScale = v;
            } else {
                r.warn("interface.uiScale = " + number(v) + " 超出范围 [" + number(InterfaceSettings::kMinScale) + ", " +
                       number(InterfaceSettings::kMaxScale) + "]，已使用默认值 \"auto\"");
            }
        } else {
            r.warn("interface.uiScale：应为 \"auto\" 或数字，已使用默认值 \"auto\"");
        }
    }
    if (const Json* panels = r.object(j, "panels", path)) {
        const std::string p = "interface.panels";
        r.unknownFields(*panels, p, {"outliner", "details", "log", "stats"});
        r.boolean(*panels, "outliner", p, s.showOutliner);
        r.boolean(*panels, "details", p, s.showDetails);
        r.boolean(*panels, "log", p, s.showLog);
        r.boolean(*panels, "stats", p, s.showStats);
    }
    if (const auto it = j.find("layout"); it != j.end()) {
        if (!it->is_string()) {
            r.warn("interface.layout：应为字符串，已使用默认布局");
        } else if (it->get_ref<const std::string&>().size() > kMaxLayoutBytes) {
            r.warn("interface.layout：超过 1 MB，已使用默认布局");
        } else {
            s.layout = it->get<std::string>();
        }
    }
}

void readDisplay(Reader& r, const Json& j, DisplaySettings& s) {
    const std::string path = "display";
    r.unknownFields(j, path, {"mode", "monitor", "videoMode", "vsync", "windowSize"});
    if (const auto it = j.find("mode"); it != j.end()) {
        const std::string value = it->is_string() ? it->get<std::string>() : std::string{};
        bool found = false;
        for (const WindowMode m : {WindowMode::Windowed, WindowMode::BorderlessFullscreen, WindowMode::ExclusiveFullscreen}) {
            if (value == windowModeName(m)) {
                s.mode = m;
                found = true;
            }
        }
        if (!found) {
            r.warn("display.mode：应为 \"windowed\"、\"borderless\" 或 \"exclusive\"，已使用默认值 \"" +
                   std::string{windowModeName(s.mode)} + "\"");
        }
    }
    r.integer(j, "monitor", path, 0, DisplaySettings::kMaxMonitor, s.monitor);
    if (const auto it = j.find("videoMode"); it != j.end()) {
        if (it->is_null()) {
            s.videoMode.reset();
        } else if (it->is_object()) {
            const std::string p = "display.videoMode";
            r.unknownFields(*it, p, {"width", "height", "refreshRate"});
            int w = 0, h = 0, hz = 0;
            const bool complete = it->contains("width") && it->contains("height") && it->contains("refreshRate");
            std::vector<std::string> local;  // 逐项问题合并为一条警告
            Reader strict(local);
            strict.integer(*it, "width", p, 1, DisplaySettings::kMaxWindowSize, w);
            strict.integer(*it, "height", p, 1, DisplaySettings::kMaxWindowSize, h);
            strict.integer(*it, "refreshRate", p, 1, 1000, hz);
            if (!complete || !local.empty() || w == 0 || h == 0 || hz == 0) {
                r.warn("display.videoMode：需要 width、height、refreshRate 三个正整数，已使用显示器当前模式");
                s.videoMode.reset();
            } else {
                s.videoMode = VideoModeSetting{w, h, hz};
            }
        } else {
            r.warn("display.videoMode：应为对象或 null，已使用显示器当前模式");
        }
    }
    r.boolean(j, "vsync", path, s.vsync);
    if (const Json* size = r.object(j, "windowSize", path)) {
        const std::string p = "display.windowSize";
        r.unknownFields(*size, p, {"width", "height"});
        r.integer(*size, "width", p, DisplaySettings::kMinWindowSize, DisplaySettings::kMaxWindowSize, s.windowWidth);
        r.integer(*size, "height", p, DisplaySettings::kMinWindowSize, DisplaySettings::kMaxWindowSize, s.windowHeight);
    }
}

void readCamera(Reader& r, const Json& j, CameraSettings& s) {
    const std::string path = "camera";
    r.unknownFields(j, path, {"lookDegreesPerPixel", "speed", "boost", "wheelStep", "panMetersPerPixel", "fovDegrees"});
    r.real(j, "lookDegreesPerPixel", path, ranges::kLook, s.lookDegreesPerPixel);
    r.real(j, "speed", path, ranges::kSpeed, s.speed);
    r.real(j, "boost", path, ranges::kBoost, s.boost);
    r.real(j, "wheelStep", path, ranges::kWheelStep, s.wheelStep);
    r.real(j, "panMetersPerPixel", path, ranges::kPan, s.panMetersPerPixel);
    r.real(j, "fovDegrees", path, ranges::kFov, s.fovDegrees);
}

void readRender(Reader& r, const Json& j, RenderSettings& s) {
    const std::string path = "render";
    r.unknownFields(j, path, {"sunAzimuthDegrees", "sunElevationDegrees", "sunIntensity", "ambientIntensity", "exposureEv",
                              "toneMapper", "msaa", "shadows", "grid", "environment", "environmentIntensity",
                              "environmentRotationDegrees", "environmentBackground", "backgroundBlur", "sunFromEnvironment",
                              "assetArmOcclusion"});
    r.real(j, "sunAzimuthDegrees", path, ranges::kAzimuth, s.sunAzimuthDegrees);
    r.real(j, "sunElevationDegrees", path, ranges::kElevation, s.sunElevationDegrees);
    r.real(j, "sunIntensity", path, ranges::kSunIntensity, s.sunIntensity);
    r.real(j, "ambientIntensity", path, ranges::kAmbient, s.ambientIntensity);
    r.real(j, "exposureEv", path, ranges::kExposure, s.exposureEv);
    if (const auto it = j.find("toneMapper"); it != j.end()) {
        const std::string value = it->is_string() ? it->get<std::string>() : std::string{};
        bool found = false;
        for (const ToneMapperSetting m :
             {ToneMapperSetting::AgX, ToneMapperSetting::PbrNeutral, ToneMapperSetting::Aces, ToneMapperSetting::None}) {
            if (value == toneMapperName(m)) {
                s.toneMapper = m;
                found = true;
            }
        }
        if (!found) {
            r.warn("render.toneMapper：应为 \"agx\"、\"pbrNeutral\"、\"aces\" 或 \"none\"，已使用默认值 \"" +
                   std::string{toneMapperName(s.toneMapper)} + "\"");
        }
    }
    if (const auto it = j.find("msaa"); it != j.end()) {
        const bool valid = it->is_number_integer() && (it->get<std::int64_t>() == 1 || it->get<std::int64_t>() == 2 ||
                                                       it->get<std::int64_t>() == 4 || it->get<std::int64_t>() == 8);
        if (valid) {
            s.msaaSamples = static_cast<int>(it->get<std::int64_t>());
        } else {
            r.warn("render.msaa：应为 1、2、4 或 8，已使用默认值 " + std::to_string(s.msaaSamples));
        }
    }
    r.boolean(j, "shadows", path, s.shadows);
    r.boolean(j, "grid", path, s.grid);
    if (const auto it = j.find("environment"); it != j.end()) {
        const std::string value = it->is_string() ? it->get<std::string>() : std::string{};
        bool found = false;
        for (const EnvironmentSetting e : {EnvironmentSetting::Hemisphere, EnvironmentSetting::Outdoor, EnvironmentSetting::Studio}) {
            if (value == environmentName(e)) {
                s.environment = e;
                found = true;
            }
        }
        if (!found) {
            r.warn("render.environment：应为 \"hemisphere\"、\"outdoor\" 或 \"studio\"，已使用默认值 \"" +
                   std::string{environmentName(s.environment)} + "\"");
        }
    }
    r.real(j, "environmentIntensity", path, ranges::kEnvironmentIntensity, s.environmentIntensity);
    r.real(j, "environmentRotationDegrees", path, ranges::kEnvironmentRotation, s.environmentRotationDegrees);
    r.boolean(j, "environmentBackground", path, s.environmentBackground);
    r.real(j, "backgroundBlur", path, ranges::kBackgroundBlur, s.backgroundBlur);
    r.boolean(j, "sunFromEnvironment", path, s.sunFromEnvironment);
    r.boolean(j, "assetArmOcclusion", path, s.assetArmOcclusion);
}

void readKeys(Reader& r, const Json& j, KeyMap& keys) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string where = "keys." + it.key();
        const auto command = commandFromName(it.key());
        if (!command) {
            r.warn("未知字段 " + where + "，已忽略");
            continue;
        }
        const std::string fallback = "，已使用默认按键";
        if (!it->is_array() || it->size() > 2) {
            r.warn(where + "：应为最多两个键名的数组" + fallback);
            continue;
        }
        KeySlots slots{in::Key::Unknown, in::Key::Unknown};
        bool ok = true;
        for (std::size_t i = 0; i < it->size() && ok; ++i) {
            const Json& item = (*it)[i];
            if (item.is_null()) continue;
            if (!item.is_string()) {
                r.warn(where + "：键名应为字符串" + fallback);
                ok = false;
                break;
            }
            const std::string name = item.get<std::string>();
            if (name.empty()) continue;
            const in::Key key = in::keyFromName(name);
            if (key == in::Key::Unknown) {
                r.warn(where + "：未知键名 \"" + name + "\"" + fallback);
                ok = false;
            } else if (isReservedKey(key)) {
                r.warn(where + "：\"" + name + "\" 是保留键（Esc、Ctrl）" + fallback);
                ok = false;
            } else {
                slots[i] = key;
            }
        }
        if (ok) keys[*command] = slots;
    }
    // 同一个键只能属于一个操作：按操作顺序保留第一次出现，后面的清空。
    std::array<std::optional<Command>, static_cast<std::size_t>(in::Key::Count)> owner{};
    for (std::size_t c = 0; c < kCommandCount; ++c) {
        const auto command = static_cast<Command>(c);
        for (std::size_t slot = 0; slot < 2; ++slot) {
            const in::Key key = keys[command][slot];
            if (key == in::Key::Unknown) continue;
            auto& first = owner[static_cast<std::size_t>(key)];
            if (first) {
                r.warn(std::string{"按键 "} + in::keyName(key) + " 同时用于 " + commandName(*first) + " 与 " +
                       commandName(command) + "，已从后者移除");
                keys.clear(command, slot);
            } else {
                first = command;
            }
        }
    }
}

}  // namespace

// ---- 名称 ----

const char* windowModeName(WindowMode mode) noexcept {
    switch (mode) {
        case WindowMode::Windowed: return "windowed";
        case WindowMode::BorderlessFullscreen: return "borderless";
        case WindowMode::ExclusiveFullscreen: return "exclusive";
    }
    return "";
}

const char* toneMapperName(ToneMapperSetting mapper) noexcept {
    switch (mapper) {
        case ToneMapperSetting::AgX: return "agx";
        case ToneMapperSetting::PbrNeutral: return "pbrNeutral";
        case ToneMapperSetting::Aces: return "aces";
        case ToneMapperSetting::None: return "none";
    }
    return "";
}

const char* environmentName(EnvironmentSetting environment) noexcept {
    switch (environment) {
        case EnvironmentSetting::Hemisphere: return "hemisphere";
        case EnvironmentSetting::Outdoor: return "outdoor";
        case EnvironmentSetting::Studio: return "studio";
    }
    return "";
}

const char* commandName(Command command) noexcept {
    const std::size_t i = ci(command);
    return i < kCommandCount ? kCommandNames[i] : "";
}

std::optional<Command> commandFromName(std::string_view name) noexcept {
    for (std::size_t i = 0; i < kCommandCount; ++i) {
        if (name == kCommandNames[i]) return static_cast<Command>(i);
    }
    return std::nullopt;
}

const char* loadStatusName(LoadStatus status) noexcept {
    switch (status) {
        case LoadStatus::Loaded: return "已读取";
        case LoadStatus::Missing: return "文件不存在";
        case LoadStatus::Corrupt: return "无法解析";
        case LoadStatus::Unreadable: return "无法读取";
    }
    return "";
}

bool isReservedKey(in::Key key) noexcept {
    return key == in::Key::Unknown || key == in::Key::Escape || key == in::Key::LeftControl ||
           key == in::Key::RightControl || static_cast<int>(key) >= static_cast<int>(in::Key::Count);
}

// ---- 按键表 ----

KeyMap::KeyMap() {
    using K = in::Key;
    slots_[ci(Command::CameraForward)] = {K::W, K::Up};
    slots_[ci(Command::CameraBack)] = {K::S, K::Down};
    slots_[ci(Command::CameraLeft)] = {K::A, K::Left};
    slots_[ci(Command::CameraRight)] = {K::D, K::Right};
    slots_[ci(Command::CameraUp)] = {K::E, K::Space};
    slots_[ci(Command::CameraDown)] = {K::Q, K::Unknown};
    slots_[ci(Command::CameraBoost)] = {K::LeftShift, K::RightShift};
    slots_[ci(Command::CameraReset)] = {K::F, K::Unknown};
    slots_[ci(Command::AppFullscreen)] = {K::F11, K::Unknown};
}

const KeySlots& KeyMap::operator[](Command command) const noexcept { return slots_[ci(command) % kCommandCount]; }

KeySlots& KeyMap::operator[](Command command) noexcept { return slots_[ci(command) % kCommandCount]; }

std::optional<Command> KeyMap::assign(Command command, std::size_t slot, in::Key key) {
    if (ci(command) >= kCommandCount || slot > 1) return std::nullopt;
    if (key == in::Key::Unknown) {
        clear(command, slot);
        return std::nullopt;
    }
    if (isReservedKey(key)) return std::nullopt;
    std::optional<Command> removedFrom;
    for (std::size_t c = 0; c < kCommandCount; ++c) {
        for (std::size_t s = 0; s < 2; ++s) {
            if (slots_[c][s] == key && !(c == ci(command) && s == slot)) {
                slots_[c][s] = in::Key::Unknown;
                removedFrom = static_cast<Command>(c);
            }
        }
    }
    slots_[ci(command)][slot] = key;
    return removedFrom;
}

void KeyMap::clear(Command command, std::size_t slot) {
    if (ci(command) < kCommandCount && slot < 2) slots_[ci(command)][slot] = in::Key::Unknown;
}

std::optional<Command> KeyMap::find(in::Key key) const noexcept {
    if (key == in::Key::Unknown) return std::nullopt;
    for (std::size_t c = 0; c < kCommandCount; ++c) {
        if (slots_[c][0] == key || slots_[c][1] == key) return static_cast<Command>(c);
    }
    return std::nullopt;
}

// ---- 比较 ----

bool operator==(const VideoModeSetting& a, const VideoModeSetting& b) noexcept {
    return a.width == b.width && a.height == b.height && a.refreshRate == b.refreshRate;
}

bool operator==(const InterfaceSettings& a, const InterfaceSettings& b) noexcept {
    return a.language == b.language && a.uiScale == b.uiScale && a.showOutliner == b.showOutliner &&
           a.showDetails == b.showDetails && a.showLog == b.showLog && a.showStats == b.showStats && a.layout == b.layout;
}

bool operator==(const DisplaySettings& a, const DisplaySettings& b) noexcept {
    return a.mode == b.mode && a.monitor == b.monitor && a.videoMode == b.videoMode && a.vsync == b.vsync &&
           a.windowWidth == b.windowWidth && a.windowHeight == b.windowHeight;
}

bool operator==(const CameraSettings& a, const CameraSettings& b) noexcept {
    return a.lookDegreesPerPixel == b.lookDegreesPerPixel && a.speed == b.speed && a.boost == b.boost &&
           a.wheelStep == b.wheelStep && a.panMetersPerPixel == b.panMetersPerPixel && a.fovDegrees == b.fovDegrees;
}

bool operator==(const RenderSettings& a, const RenderSettings& b) noexcept {
    return a.sunAzimuthDegrees == b.sunAzimuthDegrees && a.sunElevationDegrees == b.sunElevationDegrees &&
           a.sunIntensity == b.sunIntensity && a.ambientIntensity == b.ambientIntensity && a.exposureEv == b.exposureEv &&
           a.toneMapper == b.toneMapper && a.msaaSamples == b.msaaSamples && a.shadows == b.shadows && a.grid == b.grid &&
           a.environment == b.environment && a.environmentIntensity == b.environmentIntensity &&
           a.environmentRotationDegrees == b.environmentRotationDegrees &&
           a.environmentBackground == b.environmentBackground && a.backgroundBlur == b.backgroundBlur &&
           a.sunFromEnvironment == b.sunFromEnvironment && a.assetArmOcclusion == b.assetArmOcclusion;
}

bool operator==(const Settings& a, const Settings& b) noexcept {
    return a.ui == b.ui && a.display == b.display && a.camera == b.camera && a.render == b.render && a.keys == b.keys;
}

// ---- 序列化 ----

std::string toJson(const Settings& s) {
    Json root;
    root["version"] = kVersion;

    Json ui;
    ui["language"] = s.ui.language;
    if (s.ui.uiScale) {
        ui["uiScale"] = *s.ui.uiScale;
    } else {
        ui["uiScale"] = "auto";
    }
    Json panels;
    panels["outliner"] = s.ui.showOutliner;
    panels["details"] = s.ui.showDetails;
    panels["log"] = s.ui.showLog;
    panels["stats"] = s.ui.showStats;
    ui["panels"] = panels;
    ui["layout"] = s.ui.layout;
    root["interface"] = ui;

    Json display;
    display["mode"] = windowModeName(s.display.mode);
    display["monitor"] = s.display.monitor;
    if (s.display.videoMode) {
        Json mode;
        mode["width"] = s.display.videoMode->width;
        mode["height"] = s.display.videoMode->height;
        mode["refreshRate"] = s.display.videoMode->refreshRate;
        display["videoMode"] = mode;
    } else {
        display["videoMode"] = nullptr;
    }
    display["vsync"] = s.display.vsync;
    Json size;
    size["width"] = s.display.windowWidth;
    size["height"] = s.display.windowHeight;
    display["windowSize"] = size;
    root["display"] = display;

    Json camera;
    camera["lookDegreesPerPixel"] = s.camera.lookDegreesPerPixel;
    camera["speed"] = s.camera.speed;
    camera["boost"] = s.camera.boost;
    camera["wheelStep"] = s.camera.wheelStep;
    camera["panMetersPerPixel"] = s.camera.panMetersPerPixel;
    camera["fovDegrees"] = s.camera.fovDegrees;
    root["camera"] = camera;

    Json render;
    render["sunAzimuthDegrees"] = s.render.sunAzimuthDegrees;
    render["sunElevationDegrees"] = s.render.sunElevationDegrees;
    render["sunIntensity"] = s.render.sunIntensity;
    render["ambientIntensity"] = s.render.ambientIntensity;
    render["exposureEv"] = s.render.exposureEv;
    render["toneMapper"] = toneMapperName(s.render.toneMapper);
    render["msaa"] = s.render.msaaSamples;
    render["shadows"] = s.render.shadows;
    render["grid"] = s.render.grid;
    render["environment"] = environmentName(s.render.environment);
    render["environmentIntensity"] = s.render.environmentIntensity;
    render["environmentRotationDegrees"] = s.render.environmentRotationDegrees;
    render["environmentBackground"] = s.render.environmentBackground;
    render["backgroundBlur"] = s.render.backgroundBlur;
    render["sunFromEnvironment"] = s.render.sunFromEnvironment;
    render["assetArmOcclusion"] = s.render.assetArmOcclusion;
    root["render"] = render;

    Json keys = Json::object();
    for (std::size_t c = 0; c < kCommandCount; ++c) {
        const KeySlots& slots = s.keys[static_cast<Command>(c)];
        keys[kCommandNames[c]] = Json::array({in::keyName(slots[0]), in::keyName(slots[1])});
    }
    root["keys"] = keys;
    // 非法 UTF-8（理论上不会出现）替换为 U+FFFD，而不是抛出异常。
    return root.dump(2, ' ', false, Json::error_handler_t::replace) + "\n";
}

ParseResult fromJson(std::string_view text) {
    ParseResult result;
    Json root;
    try {
        root = Json::parse(text.begin(), text.end());
    } catch (const Json::exception& e) {  // 语法错误（parse_error）与数值溢出（out_of_range）
        result.error = std::string{"JSON 语法错误："} + e.what();
        return result;
    }
    if (!root.is_object()) {
        result.error = "顶层应为 JSON 对象";
        return result;
    }

    Settings s;
    Reader r(result.warnings);
    r.unknownFields(root, "", {"version", "interface", "display", "camera", "render", "keys"});
    if (const auto it = root.find("version"); it == root.end()) {
        r.warn("缺少 version 字段，按版本 " + std::to_string(kVersion) + " 读取");
    } else if (!it->is_number_integer()) {
        r.warn("version：应为整数，按版本 " + std::to_string(kVersion) + " 读取");
    } else if (const std::int64_t v = it->get<std::int64_t>(); v > kVersion) {
        r.warn("文件版本 " + std::to_string(v) + " 比本程序支持的 " + std::to_string(kVersion) + " 新，只读取认识的字段");
    } else if (v < 1) {
        r.warn("version = " + std::to_string(v) + " 无效，按版本 " + std::to_string(kVersion) + " 读取");
    }
    if (const Json* j = r.object(root, "interface", "")) readInterface(r, *j, s.ui);
    if (const Json* j = r.object(root, "display", "")) readDisplay(r, *j, s.display);
    if (const Json* j = r.object(root, "camera", "")) readCamera(r, *j, s.camera);
    if (const Json* j = r.object(root, "render", "")) readRender(r, *j, s.render);
    if (const Json* j = r.object(root, "keys", "")) readKeys(r, *j, s.keys);
    result.settings = std::move(s);
    return result;
}

// ---- 文件 ----

LoadResult load(const std::string& path) {
    LoadResult result;
    std::error_code ec;
    const fs::path p{path};
    const fs::file_status st = fs::status(p, ec);
    if (st.type() == fs::file_type::not_found) {
        result.status = LoadStatus::Missing;
        return result;
    }
    if (ec) {
        result.status = LoadStatus::Unreadable;
        result.error = "无法访问 " + path + "：" + ec.message();
        return result;
    }
    if (!fs::is_regular_file(st)) {
        result.status = LoadStatus::Unreadable;
        result.error = path + " 不是常规文件";
        return result;
    }
    const std::uintmax_t bytes = fs::file_size(p, ec);
    if (ec || bytes > kMaxFileBytes) {
        result.status = LoadStatus::Unreadable;
        result.error = ec ? "无法读取 " + path + "：" + ec.message() : path + " 超过 16 MB，不像设置文件";
        return result;
    }
    std::ifstream file(p, std::ios::binary);
    if (!file.is_open()) {
        result.status = LoadStatus::Unreadable;
        result.error = "无法打开 " + path;
        return result;
    }
    std::ostringstream content;
    content << file.rdbuf();
    if (file.bad()) {
        result.status = LoadStatus::Unreadable;
        result.error = "无法读取 " + path;
        return result;
    }
    ParseResult parsed = fromJson(content.str());
    if (parsed.error) {
        result.status = LoadStatus::Corrupt;
        result.error = *parsed.error;
        return result;
    }
    result.status = LoadStatus::Loaded;
    result.settings = std::move(parsed.settings);
    result.warnings = std::move(parsed.warnings);
    return result;
}

std::optional<std::string> save(const Settings& settings, const std::string& path) {
    if (path.empty()) return std::string{"设置文件路径为空"};
    std::error_code ec;
    const fs::path p{path};
    if (p.has_parent_path()) {
        fs::create_directories(p.parent_path(), ec);
        if (ec) return "无法创建目录 " + p.parent_path().string() + "：" + ec.message();
    }
    const fs::path tmp{path + ".tmp"};
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) return "无法写入 " + tmp.string();
        out << toJson(settings);
        out.flush();
        if (!out) {
            out.close();
            fs::remove(tmp, ec);
            return "写入 " + tmp.string() + " 失败";
        }
    }
    fs::rename(tmp, p, ec);
    if (ec) {
        const std::string message = "无法替换 " + path + "：" + ec.message();
        std::error_code ignored;
        fs::remove(tmp, ignored);
        return message;
    }
    return std::nullopt;
}

std::optional<std::string> backupCorrupt(const std::string& path, std::string* error) {
    const std::string backup = path + ".bak";
    std::error_code ec;
    fs::rename(fs::path{path}, fs::path{backup}, ec);
    if (ec) {
        if (error != nullptr) *error = "无法把 " + path + " 改名为 " + backup + "：" + ec.message();
        return std::nullopt;
    }
    return backup;
}

std::string defaultPath(const char* xdgConfigHome, const char* home) {
    // XDG 规范：XDG_CONFIG_HOME 必须是绝对路径，否则忽略。
    if (xdgConfigHome != nullptr && xdgConfigHome[0] == '/') {
        return (fs::path{xdgConfigHome} / "eai-terra" / "settings.json").string();
    }
    if (home != nullptr && home[0] == '/') {
        return (fs::path{home} / ".config" / "eai-terra" / "settings.json").string();
    }
    return {};
}

std::string defaultPath() { return defaultPath(std::getenv("XDG_CONFIG_HOME"), std::getenv("HOME")); }

}  // namespace terra::settings
