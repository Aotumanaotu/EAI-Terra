#include "terra/input/input_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <unordered_map>
#include <utility>

namespace terra::input {

const char* deviceKindName(DeviceKind kind) noexcept {
    switch (kind) {
        case DeviceKind::Keyboard: return "keyboard";
        case DeviceKind::Mouse: return "mouse";
        case DeviceKind::Gamepad: return "gamepad";
        case DeviceKind::XrController: return "xr_controller";
        case DeviceKind::External: return "external";
    }
    return "unknown";
}

namespace {

constexpr float kButtonThreshold = 0.5f;

struct ControlState {
    float value = 0.0f;   // 绝对值（setControl）
    float delta = 0.0f;   // 本帧增量（addDelta），update 后清零
    bool tapped = false;  // 本帧发生过 “< 0.5 → ≥ 0.5” 的上升沿，update 后清零
    double changedAt = -std::numeric_limits<double>::infinity();
};

std::uint32_t controlKey(Control c) noexcept {
    return (static_cast<std::uint32_t>(c.kind) << 16) | static_cast<std::uint32_t>(c.code);
}

std::uint64_t consumedKey(DeviceId device, Control c) noexcept {
    return (static_cast<std::uint64_t>(device.value) << 32) | controlKey(c);
}

struct Device {
    DeviceInfo info;
    std::unordered_map<std::uint32_t, ControlState> controls;

    const ControlState* find(Control c) const {
        const auto it = controls.find(controlKey(c));
        return it == controls.end() ? nullptr : &it->second;
    }
};

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

float axialDeadzone(float v, float inner, float outer) {
    const float m = std::fabs(v);
    if (m < inner) return 0.0f;
    const float t = std::min(1.0f, (m - inner) / (outer - inner));
    return std::copysign(t, v);
}

Vec2 applyProcessor(const Processor& p, Vec2 v) {
    switch (p.kind) {
        case Processor::Kind::AxialDeadzone:
            return {axialDeadzone(v.x, p.a, p.b), axialDeadzone(v.y, p.a, p.b)};
        case Processor::Kind::RadialDeadzone: {
            const float length = std::sqrt(v.x * v.x + v.y * v.y);
            if (length < p.a || length <= 0.0f) return {};
            const float t = std::min(1.0f, (length - p.a) / (p.b - p.a));
            return {v.x * t / length, v.y * t / length};
        }
        case Processor::Kind::Scale:
            return {v.x * p.a, v.y * p.b};
        case Processor::Kind::Clamp:
            return {std::clamp(v.x, p.a, p.b), std::clamp(v.y, p.a, p.b)};
    }
    return v;
}

bool twoDimensional(ActionType type) noexcept {
    return type == ActionType::Axis2D || type == ActionType::Delta2D;
}

// 绑定在某台设备上的一次求值结果（候选来源）。
struct Candidate {
    Vec2 value;
    float magnitude = 0.0f;
    bool tapped = false;
    double changedAt = -std::numeric_limits<double>::infinity();
    int priority = 0;
    DeviceId device;
    const Binding* binding = nullptr;
};

struct ActionRuntime {
    ActionDefinition definition;
    ActionState state;
};

struct MapRuntime {
    std::string name;
    int priority = 0;
    bool enabled = true;
    std::vector<std::size_t> actions;  // ActionRuntime 下标
};

std::optional<std::string> validateProcessor(const Processor& p) {
    if (!std::isfinite(p.a) || !std::isfinite(p.b)) return std::string{"处理器参数不是有限值"};
    switch (p.kind) {
        case Processor::Kind::AxialDeadzone:
        case Processor::Kind::RadialDeadzone:
            if (!(p.a >= 0.0f && p.a < p.b && p.b <= 1.0f)) {
                return "死区参数须满足 0 ≤ inner < outer ≤ 1（inner=" + std::to_string(p.a) +
                       "，outer=" + std::to_string(p.b) + "）";
            }
            break;
        case Processor::Kind::Clamp:
            if (p.a > p.b) return std::string{"裁剪范围须满足 lo ≤ hi"};
            break;
        case Processor::Kind::Scale:
            break;
    }
    return std::nullopt;
}

bool bindingKindAllowed(ActionType type, Binding::Kind kind) noexcept {
    switch (type) {
        case ActionType::Button: return kind == Binding::Kind::Single;
        case ActionType::Axis1D: return kind == Binding::Kind::Single || kind == Binding::Kind::Composite1D;
        case ActionType::Axis2D: return kind == Binding::Kind::Pair || kind == Binding::Kind::Composite2D;
        case ActionType::Delta2D: return kind == Binding::Kind::Pair;
    }
    return false;
}

std::optional<std::string> validateBindings(const std::string& where, ActionType type,
                                            const std::vector<Binding>& bindings) {
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        const Binding& b = bindings[i];
        const std::string bwhere = where + " 的第 " + std::to_string(i) + " 个绑定";
        if (!bindingKindAllowed(type, b.kind)) return bwhere + "的种类与动作类型不符";
        for (int k = 1; k < b.controlCount(); ++k) {
            if (b.controls[static_cast<std::size_t>(k)].kind != b.controls[0].kind) {
                return bwhere + "的控件来自不同种类的设备";
            }
        }
        for (const Processor& p : b.processors) {
            if (auto error = validateProcessor(p)) return bwhere + "：" + *error;
        }
    }
    return std::nullopt;
}

}  // namespace

struct InputSystem::Impl {
    std::vector<Device> devices;  // 按 ID 递增排列
    std::uint32_t nextDeviceId = 1;
    std::uint64_t revision = 0;
    std::vector<MapRuntime> maps;
    std::vector<ActionRuntime> actions;

    Device* findDevice(DeviceId id) {
        for (auto& d : devices) {
            if (d.info.id == id) return &d;
        }
        return nullptr;
    }
    const Device* findDevice(DeviceId id) const {
        for (const auto& d : devices) {
            if (d.info.id == id) return &d;
        }
        return nullptr;
    }

    bool modifiersSatisfied(const Binding& binding) const {
        for (const Control m : binding.modifiers) {
            bool held = false;
            for (const auto& d : devices) {
                if (d.info.description.kind != m.kind) continue;
                const ControlState* s = d.find(m);
                if (s && s->value >= kButtonThreshold) {
                    held = true;
                    break;
                }
            }
            if (!held) return false;
        }
        return true;
    }

    // 绑定在设备上的求值；设备种类不符时返回 false。
    bool evaluate(const ActionDefinition& action, const Binding& binding, const Device& device, Candidate& out) const {
        if (device.info.description.kind != binding.controls[0].kind) return false;
        std::array<float, 4> v{};
        out = Candidate{};
        for (int i = 0; i < binding.controlCount(); ++i) {
            const auto idx = static_cast<std::size_t>(i);
            if (const ControlState* s = device.find(binding.controls[idx])) {
                v[idx] = s->value + s->delta;
                out.tapped = out.tapped || s->tapped;
                out.changedAt = std::max(out.changedAt, s->changedAt);
            }
        }
        const auto on = [](float x) { return x >= kButtonThreshold ? 1.0f : 0.0f; };
        Vec2 raw;
        switch (binding.kind) {
            case Binding::Kind::Single: raw = {v[0], 0.0f}; break;
            case Binding::Kind::Pair: raw = {v[0], v[1]}; break;
            case Binding::Kind::Composite1D: raw = {on(v[1]) - on(v[0]), 0.0f}; break;
            case Binding::Kind::Composite2D: raw = {on(v[3]) - on(v[2]), on(v[0]) - on(v[1])}; break;
        }
        if (!modifiersSatisfied(binding)) {
            raw = {};
            out.tapped = false;
        }
        for (const Processor& p : binding.processors) raw = applyProcessor(p, raw);
        if (!twoDimensional(action.type)) raw.y = 0.0f;
        out.value = raw;
        out.magnitude = twoDimensional(action.type) ? std::sqrt(raw.x * raw.x + raw.y * raw.y) : std::fabs(raw.x);
        out.priority = binding.sourcePriority;
        out.device = device.info.id;
        out.binding = &binding;
        return true;
    }

    static bool active(const ActionDefinition& action, const Candidate& c) {
        if (action.type == ActionType::Button) return c.magnitude >= action.pressThreshold || c.tapped;
        return c.magnitude > 0.0f;
    }

    static const Candidate* arbitrate(Arbitration rule, const std::vector<Candidate>& candidates) {
        const Candidate* best = nullptr;
        for (const Candidate& c : candidates) {
            if (best == nullptr) {
                best = &c;
                continue;
            }
            bool better = false;
            switch (rule) {
                case Arbitration::LargestMagnitude:
                    better = c.magnitude > best->magnitude;
                    break;
                case Arbitration::MostRecent:
                    better = c.changedAt > best->changedAt ||
                             (c.changedAt == best->changedAt && c.magnitude > best->magnitude);
                    break;
                case Arbitration::SourcePriority:
                    better = c.priority > best->priority ||
                             (c.priority == best->priority && c.magnitude > best->magnitude);
                    break;
            }
            if (better) best = &c;
        }
        return best;
    }

    void updateAction(ActionRuntime& runtime, bool enabled, const std::set<std::uint64_t>& consumed,
                      std::vector<std::uint64_t>& consumes) {
        const ActionDefinition& action = runtime.definition;
        const bool wasDown = runtime.state.down;
        std::vector<Candidate> candidates;
        if (enabled) {
            for (const Binding& binding : action.bindings) {
                for (const Device& device : devices) {
                    Candidate c;
                    if (!evaluate(action, binding, device, c) || !active(action, c)) continue;
                    bool blocked = false;
                    for (int i = 0; i < binding.controlCount(); ++i) {
                        const Control ctl = binding.controls[static_cast<std::size_t>(i)];
                        blocked = blocked || consumed.count(consumedKey(device.info.id, ctl)) != 0;
                    }
                    if (blocked) continue;
                    if (binding.consume) {
                        for (int i = 0; i < binding.controlCount(); ++i) {
                            consumes.push_back(consumedKey(device.info.id, binding.controls[static_cast<std::size_t>(i)]));
                        }
                    }
                    candidates.push_back(c);
                }
            }
        }

        ActionState next;
        const Candidate* winner = arbitrate(action.arbitration, candidates);
        bool downNow = false;
        if (winner != nullptr) {
            next.source = winner->device;
            if (action.type == ActionType::Button) {
                downNow = winner->magnitude >= action.pressThreshold;
                next.x = downNow ? 1.0f : 0.0f;
            } else {
                next.x = winner->value.x;
                next.y = winner->value.y;
                downNow = winner->magnitude > 0.0f;
            }
        }
        bool tap = false;
        for (const Candidate& c : candidates) tap = tap || c.tapped;
        // 帧内按下又抬起：既报告 pressed 也报告 released，down 保持 false，不丢失快速点按。
        const bool tapOnly = tap && !downNow && !wasDown;
        next.down = downNow;
        next.pressed = !wasDown && (downNow || tap);
        next.released = (wasDown && !downNow) || tapOnly;
        if (!downNow && !tapOnly) next.source = DeviceId{};
        runtime.state = next;
    }
};

InputSystem::InputSystem() : impl_(std::make_unique<Impl>()) {}
InputSystem::~InputSystem() = default;

DeviceId InputSystem::connectDevice(const DeviceDescription& description) {
    Impl& s = *impl_;
    Device d;
    d.info.id = DeviceId{s.nextDeviceId++};
    d.info.description = description;
    s.devices.push_back(std::move(d));
    ++s.revision;
    return s.devices.back().info.id;
}

void InputSystem::disconnectDevice(DeviceId id) {
    Impl& s = *impl_;
    const auto it = std::find_if(s.devices.begin(), s.devices.end(), [id](const Device& d) { return d.info.id == id; });
    if (it == s.devices.end()) return;
    // 移除设备即释放其全部控件：下一次 update 中，依赖它的动作归零并产生“抬起”。
    s.devices.erase(it);
    ++s.revision;
}

void InputSystem::resetDevice(DeviceId id) {
    if (Device* d = impl_->findDevice(id)) {
        for (auto& entry : d->controls) {
            entry.second.value = 0.0f;
            entry.second.delta = 0.0f;
        }
    }
}

void InputSystem::setControl(DeviceId id, Control control, float value, double timestamp) {
    if (!std::isfinite(value)) return;  // 非有限值视为无效输入，忽略
    Device* d = impl_->findDevice(id);
    if (d == nullptr) return;
    ControlState& cs = d->controls[controlKey(control)];
    if (cs.value < kButtonThreshold && value >= kButtonThreshold) cs.tapped = true;
    if (cs.value != value) cs.changedAt = timestamp;
    cs.value = value;
}

void InputSystem::addDelta(DeviceId id, Control control, float delta, double timestamp) {
    if (!std::isfinite(delta)) return;
    Device* d = impl_->findDevice(id);
    if (d == nullptr) return;
    ControlState& cs = d->controls[controlKey(control)];
    cs.delta += delta;
    if (delta != 0.0f) cs.changedAt = timestamp;
}

std::optional<std::string> InputSystem::addActionMap(ActionMapDefinition definition) {
    Impl& s = *impl_;
    if (definition.name.empty()) return std::string{"动作表名称为空"};
    for (const MapRuntime& m : s.maps) {
        if (m.name == definition.name) return "动作表 '" + definition.name + "' 已存在";
    }
    std::set<std::string> names;
    for (const ActionDefinition& a : definition.actions) {
        const std::string where = "动作 '" + definition.name + "." + a.name + "'";
        if (a.name.empty()) return "动作表 '" + definition.name + "' 中有空的动作名称";
        if (!names.insert(a.name).second) return where + " 重复";
        if (!(a.pressThreshold > 0.0f && a.pressThreshold <= 1.0f)) return where + " 的按下阈值须在 (0, 1] 内";
        if (auto error = validateBindings(where, a.type, a.bindings)) return error;
    }

    MapRuntime map;
    map.name = definition.name;
    map.priority = definition.priority;
    map.enabled = definition.enabled;
    for (ActionDefinition& a : definition.actions) {
        map.actions.push_back(s.actions.size());
        s.actions.push_back(ActionRuntime{std::move(a), ActionState{}});
    }
    s.maps.push_back(std::move(map));
    return std::nullopt;
}

std::optional<std::string> InputSystem::setBindings(std::string_view map, std::string_view action,
                                                    std::vector<Binding> bindings) {
    Impl& s = *impl_;
    for (const MapRuntime& m : s.maps) {
        if (m.name != map) continue;
        for (const std::size_t index : m.actions) {
            ActionDefinition& definition = s.actions[index].definition;
            if (definition.name != action) continue;
            const std::string where = "动作 '" + m.name + "." + definition.name + "'";
            if (auto error = validateBindings(where, definition.type, bindings)) return error;
            // 状态保留：下一次 update 按新绑定计算，按下中的按钮若不再被绑定，产生“抬起”。
            definition.bindings = std::move(bindings);
            return std::nullopt;
        }
        return "动作表 '" + m.name + "' 中没有动作 '" + std::string{action} + "'";
    }
    return "动作表 '" + std::string{map} + "' 不存在";
}

bool InputSystem::setActionMapEnabled(std::string_view map, bool enabled) {
    for (MapRuntime& m : impl_->maps) {
        if (m.name == map) {
            m.enabled = enabled;
            return true;
        }
    }
    return false;
}

ActionHandle InputSystem::findAction(std::string_view map, std::string_view action) const {
    for (const MapRuntime& m : impl_->maps) {
        if (m.name != map) continue;
        for (const std::size_t index : m.actions) {
            if (impl_->actions[index].definition.name == action) {
                return ActionHandle{static_cast<std::uint32_t>(index)};
            }
        }
    }
    return ActionHandle{};
}

void InputSystem::update() {
    Impl& s = *impl_;
    // 动作表按优先级从高到低处理；优先级相同时按注册顺序。
    std::vector<std::size_t> order(s.maps.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t a, std::size_t b) { return s.maps[a].priority > s.maps[b].priority; });

    std::set<std::uint64_t> consumed;
    for (const std::size_t mi : order) {
        const MapRuntime& map = s.maps[mi];
        // 同一动作表内的绑定互不屏蔽；本表的消费在处理完整张表后才生效。
        std::vector<std::uint64_t> consumes;
        for (const std::size_t ai : map.actions) {
            s.updateAction(s.actions[ai], map.enabled, consumed, consumes);
        }
        consumed.insert(consumes.begin(), consumes.end());
    }

    for (Device& d : s.devices) {
        for (auto& entry : d.controls) {
            entry.second.delta = 0.0f;
            entry.second.tapped = false;
        }
    }
}

const ActionState& InputSystem::state(ActionHandle handle) const {
    static const ActionState kEmpty{};
    if (!handle.valid() || handle.index >= impl_->actions.size()) return kEmpty;
    return impl_->actions[handle.index].state;
}

std::vector<DeviceInfo> InputSystem::devices() const {
    std::vector<DeviceInfo> out;
    out.reserve(impl_->devices.size());
    for (const Device& d : impl_->devices) out.push_back(d.info);
    return out;
}

std::uint64_t InputSystem::deviceListRevision() const noexcept {
    return impl_->revision;
}

float InputSystem::controlValue(DeviceId id, Control control) const {
    const Impl& system = *impl_;
    if (const Device* d = system.findDevice(id)) {
        if (const ControlState* cs = d->find(control)) return cs->value + cs->delta;
    }
    return 0.0f;
}

}  // namespace terra::input
