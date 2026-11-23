#include "terra/scene/composition.hpp"

#include <cstdio>
#include <map>
#include <utility>

namespace terra::scene {

namespace {

namespace tp = terra::physics;
using Severity = SceneIssue::Severity;

std::string num(double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.10g", v);
    return buf;
}

std::string vec(const terra::data::Vec3& v) { return "(" + num(v.x) + ", " + num(v.y) + ", " + num(v.z) + ")"; }

bool sameVec(const terra::data::Vec3& a, const terra::data::Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

SceneIssue issue(Severity severity, const SceneModel* instance, std::string message) {
    SceneIssue i;
    i.severity = severity;
    if (instance != nullptr) {
        i.instance = instance->name;
        i.where = instance->where;
    }
    i.message = std::move(message);
    return i;
}

}  // namespace

bool isValidInstanceName(std::string_view name) {
    if (name.empty()) return false;
    const auto letter = [](char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_'; };
    const auto digit = [](char c) { return c >= '0' && c <= '9'; };
    if (!letter(name.front())) return false;
    for (const char c : name) {
        if (!letter(c) && !digit(c) && c != '-') return false;
    }
    return true;
}

std::string instancePrefix(const std::string& name) { return name + "/"; }

tp::PhysicsOptions resolvePhysicsOptions(const PhysicsOverrides& overrides, const std::vector<tp::PhysicsOptions>& models,
                                         const std::vector<SceneModel>& instances, std::vector<SceneIssue>& warnings) {
    tp::PhysicsOptions chosen = models.empty() ? tp::PhysicsOptions{} : models.front();
    if (overrides.timestep) chosen.timestep = *overrides.timestep;
    if (overrides.gravity) chosen.gravity = *overrides.gravity;
    if (overrides.integrator) chosen.integrator = *overrides.integrator;
    if (overrides.solver) chosen.solver = *overrides.solver;
    if (overrides.iterations) chosen.iterations = *overrides.iterations;
    if (overrides.tolerance) chosen.tolerance = *overrides.tolerance;

    const std::string first = instances.empty() ? std::string{} : instances.front().name;
    for (std::size_t i = 1; i < models.size(); ++i) {
        const tp::PhysicsOptions& m = models[i];
        const SceneModel* inst = i < instances.size() ? &instances[i] : nullptr;
        const auto differ = [&](const char* field, const std::string& value, const std::string& used) {
            warnings.push_back(issue(Severity::Warning, inst,
                                     std::string{"全局物理配置 "} + field + " 为 " + value + "，与第一个实例“" + first +
                                         "”不同；采用 " + used + "（可在场景文件的 physics 中统一指定）"));
        };
        if (!overrides.timestep && m.timestep != chosen.timestep) differ("timestep", num(m.timestep), num(chosen.timestep));
        if (!overrides.gravity && !sameVec(m.gravity, chosen.gravity)) differ("gravity", vec(m.gravity), vec(chosen.gravity));
        if (!overrides.integrator && m.integrator != chosen.integrator) differ("integrator", m.integrator, chosen.integrator);
        if (!overrides.solver && m.solver != chosen.solver) differ("solver", m.solver, chosen.solver);
        if (!overrides.iterations && m.iterations != chosen.iterations) {
            differ("iterations", std::to_string(m.iterations), std::to_string(chosen.iterations));
        }
        if (!overrides.tolerance && m.tolerance != chosen.tolerance) differ("tolerance", num(m.tolerance), num(chosen.tolerance));
    }
    return chosen;
}

int attributeByPrefix(const std::string& message, const std::vector<SceneModel>& instances) {
    int best = -1;
    std::size_t bestPos = std::string::npos;
    for (std::size_t i = 0; i < instances.size(); ++i) {
        const std::size_t pos = message.find(instancePrefix(instances[i].name));
        // 取最先出现的前缀。一个前缀可能出现在另一个名字内部（如 "a/" 出现在 "ba/x" 中），但完整的 "ba/" 出现得更早
        if (pos != std::string::npos && (bestPos == std::string::npos || pos < bestPos)) {
            best = static_cast<int>(i);
            bestPos = pos;
        }
    }
    return best;
}

ComposedWorld composeWorld(const std::vector<SceneModel>& instances, const PhysicsOverrides& overrides,
                           const WorldFactory& factory, const std::string& label) {
    ComposedWorld out;
    if (instances.empty()) {
        out.issues.push_back(issue(Severity::Error, nullptr, "场景中没有模型实例"));
        return out;
    }
    std::map<std::string, const SceneModel*> seen;
    for (const SceneModel& m : instances) {
        if (!isValidInstanceName(m.name)) {
            out.issues.push_back(issue(Severity::Error, &m,
                                       "实例名“" + m.name + "”不合法：只能用字母、数字、下划线与连字符，且以字母或下划线开头"));
        } else if (const auto it = seen.find(m.name); it != seen.end()) {
            std::string message = "实例名“" + m.name + "”重复";
            if (!it->second->where.empty()) message += "（与 " + it->second->where + " 相同）";
            out.issues.push_back(issue(Severity::Error, &m, std::move(message)));
        } else {
            seen.emplace(m.name, &m);
        }
        if (m.mjcf.empty()) out.issues.push_back(issue(Severity::Error, &m, "没有指定 MJCF 文件"));
    }
    if (!factory) out.issues.push_back(issue(Severity::Error, nullptr, "没有提供物理世界工厂"));
    if (!out.issues.empty()) return out;

    std::vector<SceneIssue> optionWarnings;
    tp::WorldComposition request;
    request.sourceLabel = label;
    for (const SceneModel& m : instances) request.models.push_back({m.name, instancePrefix(m.name), m.mjcf, m.placement});
    request.resolveOptions = [&](const std::vector<tp::PhysicsOptions>& models) {
        return resolvePhysicsOptions(overrides, models, instances, optionWarnings);
    };

    tp::ComposeResult result = factory(request);
    out.issues = std::move(optionWarnings);
    for (const tp::ComposeIssue& i : result.issues) {
        int model = i.model;
        if (model < 0 && i.severity == tp::ComposeIssue::Severity::Error) model = attributeByPrefix(i.message, instances);
        const SceneModel* inst =
            model >= 0 && static_cast<std::size_t>(model) < instances.size() ? &instances[static_cast<std::size_t>(model)] : nullptr;
        out.issues.push_back(issue(i.severity == tp::ComposeIssue::Severity::Error ? Severity::Error : Severity::Warning,
                                   inst, i.message));
    }
    out.world = std::move(result.world);
    return out;
}

}  // namespace terra::scene
