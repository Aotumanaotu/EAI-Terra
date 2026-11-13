#include "terra/physics/mujoco/mujoco_world.hpp"

#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <system_error>
#include <utility>

#include <mujoco/mujoco.h>

namespace terra::physics::mujoco {

namespace {

using terra::data::EntityId;
using terra::data::JointId;
using terra::data::JointType;

struct ModelDeleter {
    void operator()(mjModel* m) const noexcept { mj_deleteModel(m); }
};
struct DataDeleter {
    void operator()(mjData* d) const noexcept { mj_deleteData(d); }
};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

// MuJoCo 默认日志会在当前工作目录追加写入 MUJOCO_LOG.TXT。Headless 运行不应在用户目录留下文件，
// 因此只保留控制台输出（警告与错误写 stderr，不影响 stdout 上的 JSON Lines）。进程级设置，只做一次。
void configureMujocoLogging() {
    static std::once_flag once;
    std::call_once(once, [] {
        mjLogConfig config = mju_getLogConfig();
        config.logto_file = 0;
        mju_setLogConfig(config);
    });
}

std::string nameOf(const mjModel* m, int type, int id) {
    const char* name = mj_id2name(m, type, id);
    return name != nullptr ? std::string{name} : std::string{};
}

bool allFinite(const mjtNum* values, mjtSize count) {
    for (mjtSize i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) {
            return false;
        }
    }
    return true;
}

// 返回第一个“坏值”的下标，没有则返回 -1。坏值按 MuJoCo 的定义：NaN 或 |x| > mjMAXVAL（1e10）。
mjtSize firstBad(const mjtNum* values, mjtSize count) {
    for (mjtSize i = 0; i < count; ++i) {
        if (mju_isBad(values[i])) {
            return i;
        }
    }
    return -1;
}

const char* warningName(int warning) {
    switch (warning) {
        case mjWARN_INERTIA: return "惯量矩阵（接近）奇异";
        case mjWARN_CONTACTFULL: return "接触数量超出缓冲区";
        case mjWARN_CNSTRFULL: return "约束数量超出缓冲区";
        case mjWARN_BADQPOS: return "qpos 出现坏值";
        case mjWARN_BADQVEL: return "qvel 出现坏值";
        case mjWARN_BADQACC: return "qacc 出现坏值";
        case mjWARN_BADCTRL: return "ctrl 出现坏值";
        default: return "未知警告";
    }
}

std::string integratorName(int integrator) {
    switch (integrator) {
        case mjINT_EULER: return "Euler";
        case mjINT_RK4: return "RK4";
        case mjINT_IMPLICIT: return "implicit";
        case mjINT_IMPLICITFAST: return "implicitfast";
        case mjINT_DISCRETE: return "discrete";
        default: return "unknown(" + std::to_string(integrator) + ")";
    }
}

std::string solverName(int solver) {
    switch (solver) {
        case mjSOL_PGS: return "PGS";
        case mjSOL_CG: return "CG";
        case mjSOL_NEWTON: return "Newton";
        default: return "unknown(" + std::to_string(solver) + ")";
    }
}

LoadResult failure(LoadErrorKind kind, std::string message) {
    LoadResult result;
    result.error = LoadError{kind, std::move(message)};
    return result;
}

std::string checkFile(const std::filesystem::path& path) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::file_status status = fs::status(path, ec);
    if (ec || !fs::exists(status)) {
        return "模型文件不存在：" + path.string();
    }
    if (!fs::is_regular_file(status)) {
        return "模型路径不是常规文件：" + path.string();
    }
    if (!std::ifstream{path, std::ios::binary}) {
        return "无法读取模型文件：" + path.string();
    }
    return {};
}

}  // namespace

struct MujocoWorld::Impl {
    // 声明顺序决定析构顺序：data 先于 model 释放（成员按声明的逆序析构）。
    ModelPtr model;
    DataPtr data;

    terra::data::ModelDescription description;
    // Terra 实体 ID（value - 1 为下标）到 MuJoCo body 索引的私有映射。
    std::vector<int> entityToBody;

    std::uint64_t stepIndex = 0;
    std::uint64_t runEpoch = 0;
    bool faulted = false;
    // 上次检查时各类 MuJoCo 警告的累计次数；计数增加即视为出现严重诊断。
    std::array<int, mjNWARNING> warningBaseline{};

    void captureWarningBaseline() {
        for (int w = 0; w < mjNWARNING; ++w) {
            warningBaseline[static_cast<std::size_t>(w)] = data->warning[w].number;
        }
    }

    // 检查刚完成的一步。返回错误时不修改 stepIndex。
    std::optional<SimulationError> checkAfterStep(std::uint64_t step, double previousTime) const;
};

std::optional<SimulationError> MujocoWorld::Impl::checkAfterStep(std::uint64_t step,
                                                                 double previousTime) const {
    const mjModel* m = model.get();
    const mjData* d = data.get();

    for (int w = 0; w < mjNWARNING; ++w) {
        if (d->warning[w].number == warningBaseline[static_cast<std::size_t>(w)]) {
            continue;
        }
        const bool autoReset = (w == mjWARN_BADQPOS || w == mjWARN_BADQVEL || w == mjWARN_BADQACC) &&
                               !(m->opt.disableflags & mjDSBL_AUTORESET);
        std::string message = std::string{"MuJoCo 报告："} + warningName(w) + "（附加信息 " +
                              std::to_string(d->warning[w].lastinfo) + "）";
        if (autoReset) {
            message += "，MuJoCo 已自动重置状态";
        }
        return SimulationError{autoReset ? SimulationErrorKind::AutoReset : SimulationErrorKind::SolverWarning,
                               step, std::move(message)};
    }

    // 时间应单调增加；倒退说明发生了未报告的重置。
    if (!(d->time > previousTime)) {
        return SimulationError{SimulationErrorKind::AutoReset, step,
                               "仿真时间没有前进（" + std::to_string(previousTime) + " -> " +
                                   std::to_string(d->time) + "），疑似发生重置"};
    }

    if (const mjtSize i = firstBad(d->qpos, m->nq); i >= 0) {
        return SimulationError{SimulationErrorKind::NonFinite, step,
                               "qpos[" + std::to_string(i) + "] 为坏值：" + std::to_string(d->qpos[i])};
    }
    if (const mjtSize i = firstBad(d->qvel, m->nv); i >= 0) {
        return SimulationError{SimulationErrorKind::NonFinite, step,
                               "qvel[" + std::to_string(i) + "] 为坏值：" + std::to_string(d->qvel[i])};
    }
    if (!std::isfinite(d->time) || !allFinite(d->xpos, 3 * m->nbody) || !allFinite(d->xquat, 4 * m->nbody)) {
        return SimulationError{SimulationErrorKind::NonFinite, step, "时间或实体位姿含非有限值"};
    }
    return std::nullopt;
}

MujocoWorld::MujocoWorld(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

MujocoWorld::~MujocoWorld() = default;

LoadResult MujocoWorld::load(const terra::data::ModelSource& source) {
    configureMujocoLogging();

    namespace fs = std::filesystem;
    if (source.mjcfPath.empty()) {
        return failure(LoadErrorKind::FileUnavailable, "模型路径为空");
    }
    std::error_code ec;
    fs::path path = fs::absolute(source.mjcfPath, ec);
    if (ec) {
        path = source.mjcfPath;
    }
    path = path.lexically_normal();
    if (std::string problem = checkFile(path); !problem.empty()) {
        return failure(LoadErrorKind::FileUnavailable, std::move(problem));
    }

    // MuJoCo 以模型文件所在目录为基准解析 MJCF 中的相对资源路径。
    std::array<char, 1024> message{};
    ModelPtr model{mj_loadXML(path.c_str(), nullptr, message.data(), static_cast<int>(message.size()))};
    if (!model) {
        std::string detail = message[0] != '\0' ? message.data() : "未知错误";
        while (!detail.empty() && std::isspace(static_cast<unsigned char>(detail.back()))) {
            detail.pop_back();
        }
        return failure(LoadErrorKind::InvalidModel, "MJCF 加载失败（" + path.string() + "）：" + detail);
    }

    LoadResult result;
    if (message[0] != '\0') {
        result.warnings.emplace_back(message.data());
    }

    const mjtNum timestep = model->opt.timestep;
    if (!std::isfinite(timestep) || timestep <= 0) {
        return failure(LoadErrorKind::InvalidModel, "模型步长无效：" + std::to_string(timestep));
    }
    if (model->nbody > std::numeric_limits<std::uint32_t>::max() ||
        model->njnt > std::numeric_limits<std::uint32_t>::max()) {
        return failure(LoadErrorKind::InvalidModel, "模型规模超出 Terra ID 范围");
    }

    DataPtr data{mj_makeData(model.get())};
    if (!data) {
        return failure(LoadErrorKind::Internal, "MuJoCo 运行数据分配失败");
    }

    // mj_makeData 已将状态置为模型初值（qpos0、time=0）；mj_forward 计算同一时刻的派生量（xpos/xquat 等）。
    mj_forward(model.get(), data.get());
    if (!allFinite(data->qpos, model->nq) || !allFinite(data->qvel, model->nv) ||
        !allFinite(data->xpos, 3 * model->nbody) || !allFinite(data->xquat, 4 * model->nbody)) {
        return failure(LoadErrorKind::InvalidModel, "模型初始状态含非有限值");
    }

    auto impl = std::make_unique<Impl>();
    terra::data::ModelDescription& desc = impl->description;
    desc.sourcePath = path.string();
    desc.timestep = timestep;
    desc.settings.gravity = {model->opt.gravity[0], model->opt.gravity[1], model->opt.gravity[2]};
    desc.settings.integrator = integratorName(model->opt.integrator);
    desc.settings.solver = solverName(model->opt.solver);
    desc.settings.iterations = model->opt.iterations;
    desc.settings.tolerance = model->opt.tolerance;

    // 实体：MuJoCo 的每个非 world body 对应一个 Terra 实体，按 body 顺序分配 ID。
    // world body（索引 0）上的静态 geom（如地面）不是实体，将来作为 collider 描述。
    const int nbody = static_cast<int>(model->nbody);
    std::vector<EntityId> bodyToEntity(static_cast<std::size_t>(nbody));  // world 保持无效 ID
    desc.entities.reserve(static_cast<std::size_t>(nbody > 0 ? nbody - 1 : 0));
    impl->entityToBody.reserve(desc.entities.capacity());
    for (int b = 1; b < nbody; ++b) {
        const EntityId id{static_cast<std::uint32_t>(desc.entities.size() + 1)};
        bodyToEntity[static_cast<std::size_t>(b)] = id;
        const int parentBody = model->body_parentid[b];
        desc.entities.push_back({id, nameOf(model.get(), mjOBJ_BODY, b),
                                 bodyToEntity[static_cast<std::size_t>(parentBody)]});
        impl->entityToBody.push_back(b);
    }

    const int njnt = static_cast<int>(model->njnt);
    desc.joints.reserve(static_cast<std::size_t>(njnt));
    for (int j = 0; j < njnt; ++j) {
        terra::data::JointDescription joint;
        joint.id = JointId{static_cast<std::uint32_t>(j + 1)};
        joint.name = nameOf(model.get(), mjOBJ_JOINT, j);
        joint.entity = bodyToEntity[static_cast<std::size_t>(model->jnt_bodyid[j])];
        switch (model->jnt_type[j]) {
            case mjJNT_FREE:
                joint.type = JointType::Free;
                joint.configurationSize = 7;
                joint.velocitySize = 6;
                break;
            case mjJNT_BALL:
                joint.type = JointType::Ball;
                joint.configurationSize = 4;
                joint.velocitySize = 3;
                break;
            case mjJNT_SLIDE:
                joint.type = JointType::Slide;
                joint.configurationSize = 1;
                joint.velocitySize = 1;
                break;
            case mjJNT_HINGE:
                joint.type = JointType::Hinge;
                joint.configurationSize = 1;
                joint.velocitySize = 1;
                break;
            default:
                return failure(LoadErrorKind::InvalidModel,
                               "不支持的关节类型 " + std::to_string(model->jnt_type[j]) + "（关节 '" +
                                   joint.name + "'）");
        }
        desc.joints.push_back(std::move(joint));
    }

    impl->model = std::move(model);
    impl->data = std::move(data);
    impl->captureWarningBaseline();
    result.world.reset(new MujocoWorld(std::move(impl)));
    return result;
}

const terra::data::ModelDescription& MujocoWorld::description() const noexcept {
    return impl_->description;
}

std::optional<SimulationError> MujocoWorld::step() {
    Impl& w = *impl_;
    const std::uint64_t target = w.stepIndex + 1;
    if (w.faulted) {
        return SimulationError{SimulationErrorKind::Faulted, target, "世界处于故障状态，需要 reset 后才能继续"};
    }

    const double previousTime = w.data->time;
    // mj_step 先计算动力学再积分，积分后 xpos/xquat 等派生量仍对应积分前的状态；
    // 再调用一次 mj_forward，使发布的位姿与新的时间一致（技术设计 2.3 节）。
    // 已验证：额外的 mj_forward 不改变后续轨迹（见 M0-05 报告）。
    mj_step(w.model.get(), w.data.get());
    mj_forward(w.model.get(), w.data.get());

    if (auto error = w.checkAfterStep(target, previousTime)) {
        w.faulted = true;
        w.captureWarningBaseline();
        return error;
    }
    w.stepIndex = target;
    return std::nullopt;
}

void MujocoWorld::reset() {
    Impl& w = *impl_;
    // mj_resetData 恢复 qpos0、零速度、time=0，并将 ctrl 等输入清零。
    mj_resetData(w.model.get(), w.data.get());
    mj_forward(w.model.get(), w.data.get());
    w.stepIndex = 0;
    ++w.runEpoch;
    w.faulted = false;
    w.captureWarningBaseline();
}

double MujocoWorld::simulationTime() const noexcept {
    return impl_->data->time;
}

std::uint64_t MujocoWorld::stepIndex() const noexcept {
    return impl_->stepIndex;
}

std::uint64_t MujocoWorld::runEpoch() const noexcept {
    return impl_->runEpoch;
}

bool MujocoWorld::faulted() const noexcept {
    return impl_->faulted;
}

std::optional<terra::data::Pose> MujocoWorld::entityPose(terra::data::EntityId id) const {
    if (impl_->description.entity(id) == nullptr) {
        return std::nullopt;
    }
    const auto body = static_cast<std::size_t>(impl_->entityToBody[id.value - 1]);
    const mjtNum* p = impl_->data->xpos + 3 * body;
    const mjtNum* q = impl_->data->xquat + 4 * body;  // MuJoCo 四元数顺序同为 w, x, y, z
    terra::data::Pose pose;
    pose.position = {p[0], p[1], p[2]};
    pose.orientation = {q[0], q[1], q[2], q[3]};
    return pose;
}

}  // namespace terra::physics::mujoco
