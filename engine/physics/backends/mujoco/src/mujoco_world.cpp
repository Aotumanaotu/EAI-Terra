#include "mujoco_world.hpp"

#include "terra/physics/mujoco/mujoco_backend.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <system_error>
#include <utility>

#include <mujoco/mujoco.h>

#include "terra/core/fatal.hpp"

namespace terra::physics::mujoco {

namespace {

using terra::data::ActuatorId;
using terra::data::EntityId;
using terra::data::GeomId;
using terra::data::GeomShape;
using terra::data::ModelInstanceId;
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

// 出错时正在运行的世界（M2-02）：致命错误报告中写出世界实例、模型文件、步序号与仿真时间。
// 每个线程各自记录；MuJoCo 的致命错误在出错的线程上回调。
struct ActiveWorld {
    const terra::data::ModelDescription* description = nullptr;
    const std::uint64_t* stepIndex = nullptr;
    const mjData* data = nullptr;
    const char* phase = nullptr;  // "step"、"reset"
};
thread_local ActiveWorld activeWorld;

class ActiveWorldGuard {
public:
    ActiveWorldGuard(const terra::data::ModelDescription* description, const std::uint64_t* stepIndex, const mjData* data,
                     const char* phase)
        : previous_(activeWorld) {
        activeWorld = {description, stepIndex, data, phase};
    }
    ~ActiveWorldGuard() { activeWorld = previous_; }
    ActiveWorldGuard(const ActiveWorldGuard&) = delete;
    ActiveWorldGuard& operator=(const ActiveWorldGuard&) = delete;

private:
    ActiveWorld previous_;
};

mjfLogHandler previousLogHandler = nullptr;

// Terra 的 MuJoCo 日志处理（M2-02）：
//   - 错误（mju_error）：不可恢复，MuJoCo 要求处理函数不返回。交给 terra::core::fatalExit——写诊断、
//     调用紧急保存、以退出码 70 结束。模型编译期间的错误由 MuJoCo 编译器自己的线程内处理函数截获并作为
//     编译错误返回，不会到达这里（M0-08 已验证：fault_arena_too_small 报告为模型错误）。
//   - 其他级别：交给 MuJoCo 的默认处理（只打印到 stderr，不写日志文件）。
void terraLogHandler(const mjLogMessage* msg) {
    if (msg != nullptr && msg->level == mjLOG_ERROR) {
        terra::core::FatalReport report;
        report.source = "MuJoCo";
        report.message = msg->func != nullptr ? std::string{msg->func} + ": " + msg->subject : std::string{msg->subject};
        while (!report.message.empty() && std::isspace(static_cast<unsigned char>(report.message.back()))) {
            report.message.pop_back();
        }
        if (msg->body != nullptr) report.message += "\n" + std::string{msg->body};
        const ActiveWorld& w = activeWorld;
        if (w.description != nullptr) {
            report.details.emplace_back("世界实例", std::to_string(w.description->worldInstance));
            report.details.emplace_back("模型文件", w.description->sourcePath);
        }
        if (w.phase != nullptr) report.details.emplace_back("调用", w.phase);
        if (w.stepIndex != nullptr) report.details.emplace_back("步序号（出错前已完成）", std::to_string(*w.stepIndex));
        if (w.data != nullptr) report.details.emplace_back("仿真时间（秒）", std::to_string(w.data->time));
        terra::core::fatalExit(report);
    }
    if (previousLogHandler != nullptr && previousLogHandler != terraLogHandler) previousLogHandler(msg);
}

// MuJoCo 默认日志会在当前工作目录追加写入 MUJOCO_LOG.TXT。Headless 运行不应在用户目录留下文件，
// 因此只保留控制台输出（警告写 stderr，不影响 stdout 上的 JSON Lines）；错误交给 Terra 处理（M2-02）。
// 进程级设置，只做一次。
void configureMujocoLogging() {
    static std::once_flag once;
    std::call_once(once, [] {
        mjLogConfig config = mju_getLogConfig();
        config.logto_file = 0;
        mju_setLogConfig(config);
        previousLogHandler = mju_setLogHandler(terraLogHandler);
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

GeomShape shapeOf(int type) {
    switch (type) {
        case mjGEOM_PLANE: return GeomShape::Plane;
        case mjGEOM_SPHERE: return GeomShape::Sphere;
        case mjGEOM_CAPSULE: return GeomShape::Capsule;
        case mjGEOM_ELLIPSOID: return GeomShape::Ellipsoid;
        case mjGEOM_CYLINDER: return GeomShape::Cylinder;
        case mjGEOM_BOX: return GeomShape::Box;
        case mjGEOM_MESH: return GeomShape::Mesh;
        default: return GeomShape::Other;
    }
}

// 几何体与网格描述（M2-03）。颜色规则与 MuJoCo 自带查看器相同（engine_vis_visualize.c 的 setMaterial）：
// 用了材质时取材质颜色，但几何体颜色不是默认值（0.5, 0.5, 0.5, 1）时以几何体颜色为准。
void describeGeometry(const mjModel* m, const std::vector<EntityId>& bodyToEntity, ModelInstanceId instance,
                      terra::data::ModelDescription& desc) {
    const int nmesh = static_cast<int>(m->nmesh);
    desc.meshes.reserve(static_cast<std::size_t>(nmesh));
    for (int k = 0; k < nmesh; ++k) {
        terra::data::MeshDescription mesh;
        mesh.name = nameOf(m, mjOBJ_MESH, k);
        const int vertadr = m->mesh_vertadr[k];
        const int vertnum = m->mesh_vertnum[k];
        mesh.vertices.reserve(static_cast<std::size_t>(vertnum));
        for (int v = 0; v < vertnum; ++v) {
            const float* p = m->mesh_vert + 3 * static_cast<std::size_t>(vertadr + v);
            mesh.vertices.push_back({p[0], p[1], p[2]});
        }
        const int faceadr = m->mesh_faceadr[k];
        const int facenum = m->mesh_facenum[k];
        mesh.triangles.reserve(static_cast<std::size_t>(facenum));
        for (int f = 0; f < facenum; ++f) {
            const int* t = m->mesh_face + 3 * static_cast<std::size_t>(faceadr + f);
            mesh.triangles.push_back({static_cast<std::uint32_t>(t[0]), static_cast<std::uint32_t>(t[1]),
                                      static_cast<std::uint32_t>(t[2])});
        }
        desc.meshes.push_back(std::move(mesh));
    }

    const int ngeom = static_cast<int>(m->ngeom);
    desc.geoms.reserve(static_cast<std::size_t>(ngeom));
    for (int g = 0; g < ngeom; ++g) {
        const auto gi = static_cast<std::size_t>(g);
        terra::data::GeomDescription geom;
        geom.id = GeomId{static_cast<std::uint32_t>(g + 1)};
        geom.name = nameOf(m, mjOBJ_GEOM, g);
        geom.entity = bodyToEntity[static_cast<std::size_t>(m->geom_bodyid[g])];
        geom.modelInstance = instance;
        geom.shape = shapeOf(m->geom_type[g]);
        const mjtNum* size = m->geom_size + 3 * gi;
        geom.size = {size[0], size[1], size[2]};
        const mjtNum* pos = m->geom_pos + 3 * gi;
        const mjtNum* quat = m->geom_quat + 4 * gi;
        geom.local.position = {pos[0], pos[1], pos[2]};
        geom.local.orientation = {quat[0], quat[1], quat[2], quat[3]};
        const float* rgba = m->geom_rgba + 4 * gi;
        const int matid = m->geom_matid[g];
        const bool defaultColor = rgba[0] == 0.5f && rgba[1] == 0.5f && rgba[2] == 0.5f && rgba[3] == 1.0f;
        const float* color = (matid >= 0 && defaultColor) ? m->mat_rgba + 4 * static_cast<std::size_t>(matid) : rgba;
        geom.rgba = {color[0], color[1], color[2], color[3]};
        geom.group = m->geom_group[g];
        geom.collides = m->geom_contype[g] != 0 || m->geom_conaffinity[g] != 0;
        if (geom.shape == GeomShape::Mesh && m->geom_dataid[g] >= 0) geom.mesh = m->geom_dataid[g];
        desc.geoms.push_back(std::move(geom));
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

// 用于错误信息的执行器标识，如 "执行器 2（'shoulder'）"。
std::string actuatorLabel(ActuatorId id, const std::string& name) {
    std::string label = "执行器 " + std::to_string(id.value);
    if (!name.empty()) {
        label += "（'" + name + "'）";
    }
    return label;
}

ControlError controlError(ControlErrorKind kind, ActuatorId actuator, int input, std::string message) {
    return ControlError{kind, actuator, input, std::move(message)};
}

}  // namespace

struct MujocoWorld::Impl {
    // 声明顺序决定析构顺序：data 先于 model 释放（成员按声明的逆序析构）。
    ModelPtr model;
    DataPtr data;

    terra::data::ModelDescription description;
    // Terra 实体 ID（value - 1 为下标）到 MuJoCo body 索引的私有映射，以及反向映射（world 为无效 ID）。
    std::vector<int> entityToBody;
    std::vector<EntityId> bodyToEntity;

    // 当前观测时刻的快照缓存；step/reset 后清空，下次请求时重新构建。
    std::shared_ptr<const terra::data::PhysicsSnapshot> cachedSnapshot;

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

    std::shared_ptr<const terra::data::PhysicsSnapshot> buildSnapshot() const;
};

std::shared_ptr<const terra::data::PhysicsSnapshot> MujocoWorld::Impl::buildSnapshot() const {
    const mjModel* m = model.get();
    const mjData* d = data.get();
    auto snap = std::make_shared<terra::data::PhysicsSnapshot>();
    snap->worldInstance = description.worldInstance;
    snap->runEpoch = runEpoch;
    snap->stepIndex = stepIndex;
    snap->simulationTime = d->time;
    snap->fixedTimestep = m->opt.timestep;
    bool finite = std::isfinite(d->time);

    // 实体：位姿取 body 坐标系原点（xpos/xquat）。速度用 mjOBJ_XBODY：参考点为 body 原点；
    // 注意 mjOBJ_BODY 的参考点是质心（xipos），不符合 Terra 的约定。flg_local=0 表示世界坐标朝向。
    // 返回值顺序为 [角速度; 线速度]。
    snap->entities.reserve(entityToBody.size());
    for (std::size_t i = 0; i < entityToBody.size(); ++i) {
        const int b = entityToBody[i];
        const auto bi = static_cast<std::size_t>(b);
        const mjtNum* p = d->xpos + 3 * bi;
        const mjtNum* q = d->xquat + 4 * bi;
        mjtNum v[6];
        mj_objectVelocity(m, d, mjOBJ_XBODY, b, v, 0);
        terra::data::EntityState e;
        e.id = description.entities[i].id;
        e.pose.position = {p[0], p[1], p[2]};
        e.pose.orientation = {q[0], q[1], q[2], q[3]};
        e.angularVelocity = {v[0], v[1], v[2]};
        e.linearVelocity = {v[3], v[4], v[5]};
        finite = finite && allFinite(p, 3) && allFinite(q, 4) && allFinite(v, 6);
        snap->entities.push_back(e);
    }

    // 关节：按类型复制配置量与速度量（维度由 ModelDescription 给出）。
    snap->joints.reserve(description.joints.size());
    for (std::size_t j = 0; j < description.joints.size(); ++j) {
        const auto& jd = description.joints[j];
        terra::data::JointState js;
        js.id = jd.id;
        js.type = jd.type;
        js.positionSize = jd.configurationSize;
        js.velocitySize = jd.velocitySize;
        const mjtNum* qpos = d->qpos + m->jnt_qposadr[j];
        const mjtNum* qvel = d->qvel + m->jnt_dofadr[j];
        for (int k = 0; k < js.positionSize; ++k) js.position[static_cast<std::size_t>(k)] = qpos[k];
        for (int k = 0; k < js.velocitySize; ++k) js.velocity[static_cast<std::size_t>(k)] = qvel[k];
        finite = finite && allFinite(qpos, js.positionSize) && allFinite(qvel, js.velocitySize);
        snap->joints.push_back(js);
    }

    // 接触：只统计参与约束求解（exclude==0 且 efc_address>=0）且法向力 > 0 的接触点。
    // 法向力由 mj_contactForce 在接触坐标系中给出，result[0] 为法向分量（正值表示相互推开）。
    // flex 接触（geom 为 -1）在 M0 中不计入实体对。
    auto& contacts = snap->contacts;
    contacts.candidateCount = static_cast<std::uint32_t>(d->ncon);
    std::map<std::pair<std::uint32_t, std::uint32_t>, terra::data::ContactPair> pairs;
    for (int i = 0; i < d->ncon; ++i) {
        const mjContact& c = d->contact[i];
        if (c.exclude != 0 || c.efc_address < 0 || c.geom[0] < 0 || c.geom[1] < 0) {
            continue;
        }
        mjtNum force[6];
        mj_contactForce(m, d, i, force);
        if (!(force[0] > 0)) {
            continue;
        }
        EntityId a = bodyToEntity[static_cast<std::size_t>(m->geom_bodyid[c.geom[0]])];
        EntityId b = bodyToEntity[static_cast<std::size_t>(m->geom_bodyid[c.geom[1]])];
        if (b < a) std::swap(a, b);
        auto& pair = pairs[{a.value, b.value}];
        pair.entityA = a;
        pair.entityB = b;
        ++pair.pointCount;
        pair.normalForce += force[0];
        ++contacts.activeCount;
        finite = finite && std::isfinite(force[0]);
    }
    contacts.pairs.reserve(pairs.size());
    for (const auto& [key, pair] : pairs) {
        contacts.pairs.push_back(pair);
    }

    int iterations = 0;
    const int islands = std::min(std::max(d->nisland, 1), static_cast<int>(mjNISLAND));
    for (int k = 0; k < islands; ++k) {
        iterations = std::max(iterations, d->solver_niter[k]);
    }
    snap->diagnostics.finite = finite;
    snap->diagnostics.solverIterations = iterations;
    return snap;
}

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
        model->njnt > std::numeric_limits<std::uint32_t>::max() ||
        model->nactuator > std::numeric_limits<std::uint32_t>::max()) {
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
    std::vector<EntityId>& bodyToEntity = impl->bodyToEntity;
    bodyToEntity.assign(static_cast<std::size_t>(nbody), EntityId{});  // world 保持无效 ID
    desc.entities.reserve(static_cast<std::size_t>(nbody > 0 ? nbody - 1 : 0));
    impl->entityToBody.reserve(desc.entities.capacity());
    for (int b = 1; b < nbody; ++b) {
        const EntityId id{static_cast<std::uint32_t>(desc.entities.size() + 1)};
        bodyToEntity[static_cast<std::size_t>(b)] = id;
        const int parentBody = model->body_parentid[b];
        desc.entities.push_back({id, nameOf(model.get(), mjOBJ_BODY, b),
                                 bodyToEntity[static_cast<std::size_t>(parentBody)], ModelInstanceId{1}});
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

    // 执行器：一个 MuJoCo actuator 对应一个 Terra 执行器，按顺序分配 ID。
    // 注意控制量数 nu 不一定等于执行器数：每个执行器有 actuator_ctrlnum 个输入，
    // ctrllimited/ctrlrange 按输入（nu）索引（M0-07 报告第 2 节）。
    const int nact = static_cast<int>(model->nactuator);
    desc.actuators.reserve(static_cast<std::size_t>(nact));
    for (int a = 0; a < nact; ++a) {
        terra::data::ActuatorDescription act;
        act.id = ActuatorId{static_cast<std::uint32_t>(a + 1)};
        act.name = nameOf(model.get(), mjOBJ_ACTUATOR, a);
        const std::string label = actuatorLabel(act.id, act.name);
        // 带延迟或历史缓冲时，力由历史控制计算，“下一步生效”不成立；插件执行器的语义不在 M0 范围内。
        if (model->actuator_delay[a] > 0 || model->actuator_history[2 * a] > 0) {
            return failure(LoadErrorKind::Unsupported,
                           label + " 带控制延迟或历史缓冲，M0 不支持（控制须在下一步生效）");
        }
        if (model->actuator_plugin[a] >= 0) {
            return failure(LoadErrorKind::Unsupported, label + " 是插件执行器，M0 不支持");
        }
        const int trntype = model->actuator_trntype[a];
        if (trntype == mjTRN_JOINT || trntype == mjTRN_JOINTINPARENT) {
            act.transmission = terra::data::ActuatorTransmission::Joint;
            act.joint = JointId{static_cast<std::uint32_t>(model->actuator_trnid[2 * a] + 1)};
        }
        const int ctrladr = model->actuator_ctrladr[a];
        for (int k = 0; k < model->actuator_ctrlnum[a]; ++k) {
            const auto u = static_cast<std::size_t>(ctrladr + k);
            terra::data::ControlInput input;
            if (const char* inputName = mj_actuatorInputName(model.get(), a, k)) {
                input.name = inputName;
            }
            input.limited = model->actuator_ctrllimited[u] != 0;
            input.min = model->actuator_ctrlrange[2 * u];
            input.max = model->actuator_ctrlrange[2 * u + 1];
            act.inputs.push_back(std::move(input));
        }
        desc.actuators.push_back(std::move(act));
    }

    // 直接加载单个 MJCF：唯一的模型实例，名字为空（不加前缀），放置位姿为单位变换（M2-03）。
    desc.modelInstances.push_back({ModelInstanceId{1}, std::string{}, desc.sourcePath, terra::data::Pose{}});
    describeGeometry(model.get(), bodyToEntity, ModelInstanceId{1}, desc);

    // 只在加载成功时分配实例号，失败的加载不消耗编号。
    static std::atomic<std::uint64_t> nextWorldInstance{1};
    desc.worldInstance = nextWorldInstance.fetch_add(1, std::memory_order_relaxed);

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
    w.cachedSnapshot.reset();
    {
        const ActiveWorldGuard active(&w.description, &w.stepIndex, w.data.get(), "step");
        mj_step(w.model.get(), w.data.get());
        mj_forward(w.model.get(), w.data.get());
    }

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
    // mj_resetData 恢复 qpos0、零速度、time=0，并通过 mj_resetCtrl 把 ctrl 恢复为中性值
    // （普通输入为 0，四元数输入为单位四元数）。
    w.cachedSnapshot.reset();
    {
        const ActiveWorldGuard active(&w.description, &w.stepIndex, w.data.get(), "reset");
        mj_resetData(w.model.get(), w.data.get());
        mj_forward(w.model.get(), w.data.get());
    }
    w.stepIndex = 0;
    ++w.runEpoch;
    w.faulted = false;
    w.captureWarningBaseline();
}

std::optional<ControlError> MujocoWorld::setControl(const std::vector<ActuatorCommand>& commands) {
    Impl& w = *impl_;
    if (w.faulted) {
        return controlError(ControlErrorKind::Faulted, ActuatorId{}, -1, "世界处于故障状态，需要 reset 后才能设置控制");
    }

    // 先校验全部命令，全部通过后才写入（原子）。
    std::vector<bool> seen(w.description.actuators.size(), false);
    for (const ActuatorCommand& command : commands) {
        const terra::data::ActuatorDescription* act = w.description.actuator(command.actuator);
        if (act == nullptr) {
            return controlError(ControlErrorKind::UnknownActuator, command.actuator, -1,
                                "未知执行器 ID " + std::to_string(command.actuator.value));
        }
        const std::string label = actuatorLabel(act->id, act->name);
        const auto index = static_cast<std::size_t>(command.actuator.value - 1);
        if (seen[index]) {
            return controlError(ControlErrorKind::DuplicateActuator, command.actuator, -1,
                                label + " 在同一次调用中出现多次");
        }
        seen[index] = true;
        if (command.values.size() != act->inputs.size()) {
            return controlError(ControlErrorKind::SizeMismatch, command.actuator, -1,
                                label + " 需要 " + std::to_string(act->inputs.size()) + " 个控制量，收到 " +
                                    std::to_string(command.values.size()) + " 个");
        }
        for (std::size_t k = 0; k < command.values.size(); ++k) {
            const double v = command.values[k];
            const terra::data::ControlInput& input = act->inputs[k];
            const int inputIndex = static_cast<int>(k);
            const std::string where = label + " 的输入 " + std::to_string(k);
            if (!std::isfinite(v)) {
                return controlError(ControlErrorKind::NonFinite, command.actuator, inputIndex,
                                    where + " 不是有限值：" + std::to_string(v));
            }
            if (input.limited && (v < input.min || v > input.max)) {
                return controlError(ControlErrorKind::OutOfRange, command.actuator, inputIndex,
                                    where + " = " + std::to_string(v) + " 超出范围 [" + std::to_string(input.min) +
                                        ", " + std::to_string(input.max) + "]");
            }
            // MuJoCo 把 |u| > mjMAXVAL 视为坏值，会清零全部控制并发出警告；在这里提前拒绝。
            if (v > mjMAXVAL || v < -mjMAXVAL) {
                return controlError(ControlErrorKind::OutOfRange, command.actuator, inputIndex,
                                    where + " = " + std::to_string(v) + " 超出后端有效数值范围（|u| <= 1e10）");
            }
        }
    }

    // 写入 ctrl，下一次 mj_step 起生效。不重算物理，当前观测与已缓存的快照保持不变。
    for (const ActuatorCommand& command : commands) {
        const int a = static_cast<int>(command.actuator.value - 1);
        mjtNum* ctrl = w.data->ctrl + w.model->actuator_ctrladr[a];
        for (std::size_t k = 0; k < command.values.size(); ++k) {
            ctrl[k] = command.values[k];
        }
    }
    return std::nullopt;
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

std::shared_ptr<const terra::data::PhysicsSnapshot> MujocoWorld::snapshot() const {
    if (!impl_->cachedSnapshot) {
        impl_->cachedSnapshot = impl_->buildSnapshot();
    }
    return impl_->cachedSnapshot;
}

LoadResult loadWorld(const terra::data::ModelSource& source) {
    return MujocoWorld::load(source);
}

}  // namespace terra::physics::mujoco
