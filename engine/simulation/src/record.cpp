// 实验记录的写入、读取与复现（M3-07a）。nlohmann/json 只在本文件中使用，不进入公共头。

#include "terra/simulation/record.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "terra/core/sha256.hpp"
#include "terra/scene/scene.hpp"

namespace terra::simulation {

namespace {

namespace fs = std::filesystem;
namespace td = terra::data;
namespace tp = terra::physics;
using Json = nlohmann::ordered_json;

constexpr int kRecordVersion = 1;

Json vec3(const td::Vec3& v) { return Json::array({v.x, v.y, v.z}); }

// JSON 中的数值；null（非有限值写出时为 null）读作 NaN
double number(const Json& v) {
    return v.is_number() ? v.get<double>() : std::numeric_limits<double>::quiet_NaN();
}

td::Vec3 readVec3(const Json& v) {
    if (!v.is_array() || v.size() != 3) throw std::runtime_error("应为 3 个数值的数组");
    return {number(v[0]), number(v[1]), number(v[2])};
}

Json stateJson(const td::PhysicsSnapshot& s) {
    Json j;
    j["type"] = "state";
    j["step"] = s.stepIndex;
    j["time"] = s.simulationTime;
    j["run_epoch"] = s.runEpoch;
    Json entities = Json::array();
    for (const auto& e : s.entities) {
        const auto& q = e.pose.orientation;
        entities.push_back({{"id", e.id.value},
                            {"position", vec3(e.pose.position)},
                            {"orientation", Json::array({q.w, q.x, q.y, q.z})},
                            {"linear_velocity", vec3(e.linearVelocity)},
                            {"angular_velocity", vec3(e.angularVelocity)}});
    }
    j["entities"] = std::move(entities);
    Json joints = Json::array();
    for (const auto& js : s.joints) {
        Json position = Json::array();
        Json velocity = Json::array();
        for (int k = 0; k < js.positionSize; ++k) position.push_back(js.position[static_cast<std::size_t>(k)]);
        for (int k = 0; k < js.velocitySize; ++k) velocity.push_back(js.velocity[static_cast<std::size_t>(k)]);
        joints.push_back({{"id", js.id.value}, {"position", std::move(position)}, {"velocity", std::move(velocity)}});
    }
    j["joints"] = std::move(joints);
    Json pairs = Json::array();
    for (const auto& p : s.contacts.pairs) {
        pairs.push_back({{"entities", Json::array({p.entityA.value, p.entityB.value})},
                         {"points", p.pointCount},
                         {"normal_force", p.normalForce}});
    }
    j["contacts"] = {{"candidates", s.contacts.candidateCount}, {"active", s.contacts.activeCount}, {"pairs", std::move(pairs)}};
    j["diagnostics"] = {{"finite", s.diagnostics.finite}, {"solver_iterations", s.diagnostics.solverIterations}};
    return j;
}

td::PhysicsSnapshot readState(const Json& j) {
    td::PhysicsSnapshot s;
    s.stepIndex = j.at("step").get<std::uint64_t>();
    s.simulationTime = number(j.at("time"));
    s.runEpoch = j.value("run_epoch", std::uint64_t{0});
    for (const auto& e : j.at("entities")) {
        td::EntityState es;
        es.id = td::EntityId{e.at("id").get<std::uint32_t>()};
        es.pose.position = readVec3(e.at("position"));
        const auto& q = e.at("orientation");
        if (!q.is_array() || q.size() != 4) throw std::runtime_error("orientation 应为 4 个数值");
        es.pose.orientation = {number(q[0]), number(q[1]), number(q[2]), number(q[3])};
        es.linearVelocity = readVec3(e.at("linear_velocity"));
        es.angularVelocity = readVec3(e.at("angular_velocity"));
        s.entities.push_back(es);
    }
    for (const auto& jj : j.at("joints")) {
        td::JointState js;
        js.id = td::JointId{jj.at("id").get<std::uint32_t>()};
        const auto& p = jj.at("position");
        const auto& v = jj.at("velocity");
        if (p.size() > js.position.size() || v.size() > js.velocity.size()) throw std::runtime_error("关节状态的维度过大");
        js.positionSize = static_cast<int>(p.size());
        js.velocitySize = static_cast<int>(v.size());
        for (std::size_t k = 0; k < p.size(); ++k) js.position[k] = number(p[k]);
        for (std::size_t k = 0; k < v.size(); ++k) js.velocity[k] = number(v[k]);
        s.joints.push_back(js);
    }
    const auto& c = j.at("contacts");
    s.contacts.candidateCount = c.at("candidates").get<std::uint32_t>();
    s.contacts.activeCount = c.at("active").get<std::uint32_t>();
    for (const auto& p : c.at("pairs")) {
        td::ContactPair pair;
        pair.entityA = td::EntityId{p.at("entities").at(0).get<std::uint32_t>()};
        pair.entityB = td::EntityId{p.at("entities").at(1).get<std::uint32_t>()};
        pair.pointCount = p.at("points").get<std::uint32_t>();
        pair.normalForce = number(p.at("normal_force"));
        s.contacts.pairs.push_back(pair);
    }
    s.diagnostics.finite = j.at("diagnostics").at("finite").get<bool>();
    s.diagnostics.solverIterations = j.at("diagnostics").at("solver_iterations").get<int>();
    return s;
}

Json headerJson(const RecordHeader& h) {
    Json j;
    j["type"] = "header";
    j["format"] = "terra.record";
    j["version"] = kRecordVersion;
    j["environment"] = {{"terra", h.terraVersion},
                        {"compiler", h.compiler},
                        {"physics_backend", h.physicsBackend},
                        {"platform", h.platform},
                        {"created", h.created}};
    j["inputs"] = {{"scene", h.scenePath},
                   {"scene_sha256", h.sceneSha256},
                   {"motion", h.motionPath},
                   {"motion_sha256", h.motionSha256},
                   {"start_keyframe", h.startKeyframe}};
    j["physics"] = {{"timestep", h.timestep},     {"integrator", h.integrator}, {"solver", h.solver},
                    {"iterations", h.iterations}, {"tolerance", h.tolerance},   {"gravity", vec3(h.gravity)}};
    j["sample_every"] = h.sampleEvery;
    j["model"] = {{"entities", h.entities}, {"joints", h.joints}, {"actuators", h.actuators}};
    return j;
}

RecordHeader readHeader(const Json& j) {
    RecordHeader h;
    const auto& env = j.at("environment");
    h.terraVersion = env.at("terra").get<std::string>();
    h.compiler = env.at("compiler").get<std::string>();
    h.physicsBackend = env.at("physics_backend").get<std::string>();
    h.platform = env.at("platform").get<std::string>();
    h.created = env.at("created").get<std::string>();
    const auto& in = j.at("inputs");
    h.scenePath = in.at("scene").get<std::string>();
    h.sceneSha256 = in.at("scene_sha256").get<std::string>();
    h.motionPath = in.at("motion").get<std::string>();
    h.motionSha256 = in.at("motion_sha256").get<std::string>();
    h.startKeyframe = in.at("start_keyframe").get<std::string>();
    const auto& ph = j.at("physics");
    h.timestep = ph.at("timestep").get<double>();
    h.integrator = ph.at("integrator").get<std::string>();
    h.solver = ph.at("solver").get<std::string>();
    h.iterations = ph.at("iterations").get<int>();
    h.tolerance = ph.at("tolerance").get<double>();
    h.gravity = readVec3(ph.at("gravity"));
    h.sampleEvery = j.at("sample_every").get<std::uint64_t>();
    const auto& model = j.at("model");
    h.entities = model.at("entities").get<std::vector<std::string>>();
    h.joints = model.at("joints").get<std::vector<std::string>>();
    h.actuators = model.at("actuators").get<std::vector<std::string>>();
    return h;
}

// 两个姿态之间的转角（rad）：相对四元数 r = conj(a) ⊗ b，角度 = 2·atan2(|r 的向量部分|, |r.w|)。
// 两个输入逐位相同时 r 的向量部分恰为 0，结果恰为 0（2026-11-24：原先用 acos(|a·b|)，四元数的模偏离 1 约 1e-15
// 时会得到 1e-7 量级的假误差）。
double rotationAngle(const td::Quaternion& a, const td::Quaternion& b) {
    const double rw = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    const double rx = a.w * b.x - a.x * b.w - a.y * b.z + a.z * b.y;
    const double ry = a.w * b.y + a.x * b.z - a.y * b.w - a.z * b.x;
    const double rz = a.w * b.z - a.x * b.y + a.y * b.x - a.z * b.w;
    return 2.0 * std::atan2(std::sqrt(rx * rx + ry * ry + rz * rz), std::fabs(rw));
}

}  // namespace

RecordHeader makeRecordHeader(const td::ModelDescription& model, std::uint64_t sampleEvery) {
    RecordHeader h;
    h.timestep = model.timestep;
    h.integrator = model.settings.integrator;
    h.solver = model.settings.solver;
    h.iterations = model.settings.iterations;
    h.tolerance = model.settings.tolerance;
    h.gravity = model.settings.gravity;
    h.sampleEvery = std::max<std::uint64_t>(1, sampleEvery);
    for (const auto& e : model.entities) h.entities.push_back(e.name);
    for (const auto& j : model.joints) h.joints.push_back(j.name);
    for (const auto& a : model.actuators) h.actuators.push_back(a.name);
    return h;
}

// ---- 写入 ----

struct RecordWriter::Impl {
    std::string path;
    std::ofstream out;
    RecordHeader header;
    std::map<std::uint32_t, std::vector<double>> last;  // 每个执行器最近一次记录的控制量
    // 同一步内可能多次下发（查看器：排队的命令、再是每步控制器的输出），合并后写一行；后下发的覆盖先下发的，
    // 与 MuJoCo 中最终的控制量一致（M3-07c）
    std::optional<std::uint64_t> pendingStep;
    std::map<std::uint32_t, std::vector<double>> pending;
    std::uint64_t bytes = 0;
    bool finished = false;

    void flush() {
        if (!pendingStep) return;
        Json actuators = Json::array();
        for (const auto& [id, values] : pending) {
            auto it = last.find(id);
            if (it != last.end() && it->second == values) continue;
            last[id] = values;
            actuators.push_back(Json::array({id, values}));
        }
        if (!actuators.empty()) {
            Json j;
            j["type"] = "control";
            j["step"] = *pendingStep;
            j["actuators"] = std::move(actuators);
            line(j);
        }
        pendingStep.reset();
        pending.clear();
    }

    void line(const Json& j) {
        const std::string text = j.dump();
        out << text << '\n';
        bytes += text.size() + 1;
    }
};

RecordWriter::RecordWriter(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

RecordWriter::~RecordWriter() = default;

std::unique_ptr<RecordWriter> RecordWriter::create(const std::string& path, const RecordHeader& header, std::string& error) {
    auto impl = std::make_unique<Impl>();
    impl->path = path;
    impl->header = header;
    std::error_code ec;
    const fs::path parent = fs::path{path}.parent_path();
    if (!parent.empty()) fs::create_directories(parent, ec);
    impl->out.open(path, std::ios::binary | std::ios::trunc);
    if (!impl->out) {
        error = "无法创建记录文件：" + path;
        return nullptr;
    }
    impl->line(headerJson(header));
    if (!impl->out) {
        error = "写入记录文件失败：" + path;
        return nullptr;
    }
    return std::unique_ptr<RecordWriter>(new RecordWriter(std::move(impl)));
}

void RecordWriter::control(std::uint64_t step, const std::vector<tp::ActuatorCommand>& commands) {
    if (impl_->pendingStep && *impl_->pendingStep != step) impl_->flush();
    impl_->pendingStep = step;
    for (const auto& c : commands) impl_->pending[c.actuator.value] = c.values;
}

void RecordWriter::state(const td::PhysicsSnapshot& snapshot) {
    impl_->flush();
    impl_->line(stateJson(snapshot));
}

void RecordWriter::event(std::uint64_t step, const std::string& kind, const std::string& detail) {
    impl_->flush();
    Json j;
    j["type"] = "event";
    j["step"] = step;
    j["kind"] = kind;
    if (!detail.empty()) j["detail"] = detail;
    impl_->line(j);
}

bool RecordWriter::finish(std::uint64_t steps, double time, const std::string& reason) {
    if (impl_->finished) return good();
    impl_->flush();
    Json j;
    j["type"] = "summary";
    j["steps"] = steps;
    j["time"] = time;
    j["reason"] = reason;
    impl_->line(j);
    impl_->out.flush();
    impl_->finished = true;
    const bool ok = static_cast<bool>(impl_->out);
    impl_->out.close();
    return ok;
}

bool RecordWriter::good() const noexcept { return static_cast<bool>(impl_->out) || impl_->finished; }
std::uint64_t RecordWriter::bytes() const noexcept { return impl_->bytes; }
const std::string& RecordWriter::path() const noexcept { return impl_->path; }

// ---- 读取 ----

RecordReadResult readRecord(const std::string& path) {
    RecordReadResult result;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::error_code ec;
        result.errors.push_back(fs::exists(path, ec) ? "无法读取记录文件：" + path : "记录文件不存在：" + path);
        return result;
    }
    Record record;
    bool haveHeader = false;
    std::string text;
    std::size_t lineNo = 0;
    std::uint64_t lastControl = 0;
    bool anyControl = false;
    while (std::getline(in, text)) {
        ++lineNo;
        if (text.empty()) continue;
        const std::string where = "第 " + std::to_string(lineNo) + " 行：";
        try {
            const Json j = Json::parse(text);
            const std::string type = j.at("type").get<std::string>();
            if (!haveHeader) {
                if (type != "header") {
                    result.errors.push_back(where + "第一行应为记录头（type 为 header），实际为 " + type);
                    break;
                }
                if (j.value("format", std::string{}) != "terra.record") {
                    result.errors.push_back(where + "不是 Terra 实验记录（format 应为 terra.record）");
                    break;
                }
                const int version = j.value("version", 0);
                if (version > kRecordVersion) {
                    result.errors.push_back(where + "记录版本 " + std::to_string(version) + " 高于本程序支持的版本 " +
                                            std::to_string(kRecordVersion) + "，请更新程序");
                    break;
                }
                if (version < 1) {
                    result.errors.push_back(where + "记录版本无效");
                    break;
                }
                record.header = readHeader(j);
                haveHeader = true;
                continue;
            }
            if (record.summary) {
                result.errors.push_back(where + "summary 之后不应再有内容");
                break;
            }
            if (type == "control") {
                RecordControl c;
                c.step = j.at("step").get<std::uint64_t>();
                if (anyControl && c.step <= lastControl) throw std::runtime_error("control 的步序号应严格递增");
                anyControl = true;
                lastControl = c.step;
                for (const auto& a : j.at("actuators")) {
                    tp::ActuatorCommand command;
                    command.actuator = td::ActuatorId{a.at(0).get<std::uint32_t>()};
                    for (const auto& v : a.at(1)) command.values.push_back(number(v));
                    c.commands.push_back(std::move(command));
                }
                record.controls.push_back(std::move(c));
            } else if (type == "state") {
                record.states.push_back(readState(j));
                if (record.states.size() > 1 && record.states.back().stepIndex <= record.states[record.states.size() - 2].stepIndex) {
                    throw std::runtime_error("state 的步序号应严格递增");
                }
            } else if (type == "event") {
                record.events.push_back({j.at("step").get<std::uint64_t>(), j.at("kind").get<std::string>(), j.value("detail", std::string{})});
            } else if (type == "summary") {
                record.summary = RecordSummary{j.at("steps").get<std::uint64_t>(), number(j.at("time")), j.at("reason").get<std::string>()};
            } else {
                result.errors.push_back(where + "未知的记录类型“" + type + "”");
            }
        } catch (const std::exception& e) {
            result.errors.push_back(where + e.what());
        }
    }
    if (!haveHeader && result.errors.empty()) result.errors.push_back("记录文件为空：" + path);
    if (!result.errors.empty()) return result;
    if (!record.summary) result.warnings.push_back("记录没有正常结束（缺少 summary），可能是运行被中断；已读到的部分仍可使用");
    if (record.states.empty()) result.warnings.push_back("记录中没有状态采样");
    result.record = std::move(record);
    return result;
}

std::optional<RecordInfo> readRecordInfo(const std::string& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::error_code ec;
        error = fs::exists(path, ec) ? "无法读取记录文件：" + path : "记录文件不存在：" + path;
        return std::nullopt;
    }
    std::string first;
    if (!std::getline(in, first) || first.empty()) {
        error = "记录文件为空：" + path;
        return std::nullopt;
    }
    RecordInfo info;
    try {
        const Json j = Json::parse(first);
        if (j.value("type", std::string{}) != "header" || j.value("format", std::string{}) != "terra.record") {
            error = "不是 Terra 实验记录（第一行不是记录头）：" + path;
            return std::nullopt;
        }
        if (j.value("version", 0) > kRecordVersion) {
            error = "记录版本高于本程序支持的版本，请更新程序：" + path;
            return std::nullopt;
        }
        info.header = readHeader(j);
    } catch (const std::exception& e) {
        error = "第 1 行：" + std::string{e.what()};
        return std::nullopt;
    }
    // 最后一个非空行：从文件末尾向前读一段
    in.clear();
    in.seekg(0, std::ios::end);
    const std::streamoff size = in.tellg();
    const std::streamoff chunk = std::min<std::streamoff>(size, 65536);
    in.seekg(size - chunk);
    std::string tail(static_cast<std::size_t>(chunk), '\0');
    in.read(tail.data(), chunk);
    while (!tail.empty() && (tail.back() == '\n' || tail.back() == '\r')) tail.pop_back();
    const std::size_t start = tail.rfind('\n');
    const std::string last = start == std::string::npos ? tail : tail.substr(start + 1);
    try {
        const Json j = Json::parse(last);
        if (j.value("type", std::string{}) == "summary") {
            info.summary = RecordSummary{j.at("steps").get<std::uint64_t>(), number(j.at("time")), j.at("reason").get<std::string>()};
        }
    } catch (const std::exception&) {
        // 最后一行不完整（如写入中断）：当作没有摘要
    }
    return info;
}

std::map<std::uint32_t, std::vector<double>> controlsAt(const Record& record, std::uint64_t step) {
    std::map<std::uint32_t, std::vector<double>> out;
    for (const auto& c : record.controls) {
        if (c.step > step) break;
        for (const auto& command : c.commands) out[command.actuator.value] = command.values;
    }
    return out;
}

// ---- 复现 ----

RerunResult rerunRecord(const Record& record, const terra::scene::WorldFactory& factory, const RerunOptions& options) {
    RerunResult out;
    const RecordHeader& h = record.header;
    if (const auto digest = terra::sha256File(h.scenePath); !digest) {
        out.error = "场景文件不存在或读不出：" + h.scenePath;
        return out;
    } else if (*digest != h.sceneSha256) {
        out.warnings.push_back("场景文件的内容与记录时不同（sha256 不一致）：" + h.scenePath);
    }
    if (!h.motionPath.empty()) {
        const auto digest = terra::sha256File(h.motionPath);
        if (!digest || *digest != h.motionSha256) {
            out.warnings.push_back("动作文件的内容与记录时不同或已不存在（复现只用记录中的控制量，不读动作文件）：" + h.motionPath);
        }
    }
    auto loaded = terra::scene::loadScene(h.scenePath, factory);
    if (!loaded.ok()) {
        out.error = "场景无法加载：" + h.scenePath;
        for (const auto& i : loaded.issues) out.error += "；" + i.where + " " + i.message;
        return out;
    }
    tp::PhysicsWorld& world = *loaded.scene->world;
    const td::ModelDescription& d = world.description();
    // 模型必须与记录时相同（名字与顺序）
    std::vector<std::string> entities, joints, actuators;
    for (const auto& e : d.entities) entities.push_back(e.name);
    for (const auto& j : d.joints) joints.push_back(j.name);
    for (const auto& a : d.actuators) actuators.push_back(a.name);
    if (entities != h.entities || joints != h.joints || actuators != h.actuators) {
        out.error = "场景中的实体、关节或执行器与记录时不同，无法复现";
        return out;
    }
    if (d.timestep != h.timestep || d.settings.integrator != h.integrator || d.settings.solver != h.solver ||
        d.settings.iterations != h.iterations) {
        out.warnings.push_back("物理设置与记录时不同（步长、积分器、求解器或迭代次数）");
    }
    if (!h.startKeyframe.empty()) {
        const int k = d.findKeyframe(h.startKeyframe);
        if (k < 0) {
            out.error = "场景中找不到记录的起始关键帧：" + h.startKeyframe;
            return out;
        }
        world.resetToKeyframe(static_cast<std::size_t>(k));
    }
    const std::uint64_t steps = record.summary ? record.summary->steps
                                : !record.states.empty() ? record.states.back().stepIndex
                                                         : 0;
    out.steps = steps;
    std::size_t nextControl = 0;
    std::size_t nextState = 0;
    const auto compare = [&](const td::PhysicsSnapshot& expected) {
        const auto actual = world.snapshot();
        ++out.compared;
        double position = 0.0, rotation = 0.0;
        for (std::size_t i = 0; i < expected.entities.size() && i < actual->entities.size(); ++i) {
            const auto& p = actual->entities[i].pose;
            const auto& q = expected.entities[i].pose;
            const bool same = p.position.x == q.position.x && p.position.y == q.position.y && p.position.z == q.position.z &&
                              p.orientation.w == q.orientation.w && p.orientation.x == q.orientation.x &&
                              p.orientation.y == q.orientation.y && p.orientation.z == q.orientation.z;
            if (!same && !out.firstDifferenceStep) {
                out.firstDifferenceStep = expected.stepIndex;
                char buf[160];
                std::snprintf(buf, sizeof buf, "（位置差 %.3g, %.3g, %.3g；姿态差 w %.3g）", p.position.x - q.position.x,
                              p.position.y - q.position.y, p.position.z - q.position.z, p.orientation.w - q.orientation.w);
                out.firstDifference = (i < h.entities.size() ? h.entities[i] : std::string{"?"}) + buf;
            }
            out.bitwise = out.bitwise && same;
            if (same) continue;  // 逐位相同时误差恰为 0（不经过三角函数与可能的乘加融合带来的 1e-17 量级舍入）
            position = std::max(position, std::hypot(p.position.x - q.position.x, p.position.y - q.position.y,
                                                     p.position.z - q.position.z));
            rotation = std::max(rotation, rotationAngle(p.orientation, q.orientation));
        }
        if (expected.entities.size() != actual->entities.size()) out.bitwise = false;
        out.maxPositionError = std::max(out.maxPositionError, position);
        out.maxRotationError = std::max(out.maxRotationError, rotation);
        if (!out.firstExceedStep && (position > options.positionTolerance || rotation > options.rotationTolerance)) {
            out.firstExceedStep = expected.stepIndex;
        }
    };
    if (nextState < record.states.size() && record.states[nextState].stepIndex == 0) compare(record.states[nextState++]);
    for (std::uint64_t k = 0; k < steps; ++k) {
        while (nextControl < record.controls.size() && record.controls[nextControl].step < k) ++nextControl;
        if (nextControl < record.controls.size() && record.controls[nextControl].step == k) {
            if (const auto error = world.setControl(record.controls[nextControl].commands)) {
                out.error = "第 " + std::to_string(k) + " 步的控制量被拒绝：" + error->message;
                return out;
            }
            ++nextControl;
        }
        if (const auto error = world.step()) {
            out.error = "第 " + std::to_string(error->step) + " 步出错：" + error->message;
            return out;
        }
        while (nextState < record.states.size() && record.states[nextState].stepIndex < world.stepIndex()) ++nextState;
        if (nextState < record.states.size() && record.states[nextState].stepIndex == world.stepIndex()) compare(record.states[nextState++]);
    }
    out.ran = true;
    return out;
}

}  // namespace terra::simulation
