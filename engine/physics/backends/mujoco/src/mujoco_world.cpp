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
#include <tuple>
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

// 组合世界中的名字（M2-04）：按前缀确定所属实例。组合前，后端给没有名字的 body 与几何体起了临时名字
// （kHiddenNameMark 开头），使每个元素都能按前缀归属；对外显示时去掉，仍为空名字，与 M0 一致。
constexpr const char* kHiddenNameMark = "__terra_unnamed_";

class InstanceNames {
public:
    explicit InstanceNames(std::vector<std::string> prefixes) : prefixes_(std::move(prefixes)) {}

    // 最长前缀匹配；只有一个实例时都属于它（单模型加载，前缀为空）。
    ModelInstanceId instanceOf(const std::string& raw) const {
        if (prefixes_.size() == 1) return ModelInstanceId{1};
        std::size_t best = 0;
        std::size_t bestLength = 0;
        for (std::size_t i = 0; i < prefixes_.size(); ++i) {
            const std::string& p = prefixes_[i];
            if (!p.empty() && p.size() >= bestLength && raw.compare(0, p.size(), p) == 0) {
                best = i + 1;
                bestLength = p.size();
            }
        }
        return ModelInstanceId{static_cast<std::uint32_t>(best)};
    }

    static std::string visible(const std::string& raw) {
        return raw.find(kHiddenNameMark) != std::string::npos ? std::string{} : raw;
    }

private:
    std::vector<std::string> prefixes_;
};

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
void describeGeometry(const mjModel* m, const std::vector<EntityId>& bodyToEntity, const InstanceNames& names,
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
        // M3-03：法线与纹理坐标（索引相对本网格的 normaladr / texcoordadr，与 MuJoCo 渲染器 render_context.c 相同）
        const int normaladr = m->mesh_normaladr[k];
        const int normalnum = m->mesh_normalnum[k];
        if (normalnum > 0) {
            mesh.normals.reserve(static_cast<std::size_t>(normalnum));
            for (int v = 0; v < normalnum; ++v) {
                const float* n = m->mesh_normal + 3 * static_cast<std::size_t>(normaladr + v);
                mesh.normals.push_back({n[0], n[1], n[2]});
            }
            mesh.triangleNormals.reserve(static_cast<std::size_t>(facenum));
            for (int f = 0; f < facenum; ++f) {
                const int* t = m->mesh_facenormal + 3 * static_cast<std::size_t>(faceadr + f);
                mesh.triangleNormals.push_back({static_cast<std::uint32_t>(t[0]), static_cast<std::uint32_t>(t[1]),
                                                static_cast<std::uint32_t>(t[2])});
            }
        }
        const int texcoordadr = m->mesh_texcoordadr[k];
        const int texcoordnum = m->mesh_texcoordnum[k];
        if (texcoordadr >= 0 && texcoordnum > 0) {
            mesh.texcoords.reserve(static_cast<std::size_t>(texcoordnum));
            for (int v = 0; v < texcoordnum; ++v) {
                const float* uv = m->mesh_texcoord + 2 * static_cast<std::size_t>(texcoordadr + v);
                mesh.texcoords.push_back({uv[0], uv[1]});
            }
            mesh.triangleTexcoords.reserve(static_cast<std::size_t>(facenum));
            for (int f = 0; f < facenum; ++f) {
                const int* t = m->mesh_facetexcoord + 3 * static_cast<std::size_t>(faceadr + f);
                mesh.triangleTexcoords.push_back({static_cast<std::uint32_t>(t[0]), static_cast<std::uint32_t>(t[1]),
                                                  static_cast<std::uint32_t>(t[2])});
            }
        }
        desc.meshes.push_back(std::move(mesh));
    }

    // M3-03：贴图与材质
    const int ntex = static_cast<int>(m->ntex);
    desc.textures.reserve(static_cast<std::size_t>(ntex));
    for (int t = 0; t < ntex; ++t) {
        terra::data::TextureDescription tex;
        tex.name = nameOf(m, mjOBJ_TEXTURE, t);
        tex.kind = m->tex_type[t] == mjTEXTURE_2D     ? terra::data::TextureKind::TwoD
                   : m->tex_type[t] == mjTEXTURE_CUBE ? terra::data::TextureKind::Cube
                                                      : terra::data::TextureKind::Skybox;
        tex.width = m->tex_width[t];
        tex.height = m->tex_height[t];
        const int channels = m->tex_nchannel[t];
        if (tex.kind == terra::data::TextureKind::TwoD && tex.width > 0 && tex.height > 0 && channels >= 1 && channels <= 4) {
            const auto pixels = static_cast<std::size_t>(tex.width) * static_cast<std::size_t>(tex.height);
            const mjtByte* src = m->tex_data + m->tex_adr[t];
            tex.rgba.resize(pixels * 4);
            for (std::size_t p = 0; p < pixels; ++p) {
                const mjtByte* px = src + p * static_cast<std::size_t>(channels);
                std::uint8_t* out = tex.rgba.data() + 4 * p;
                if (channels <= 2) {  // 灰度（第二通道为透明度）
                    out[0] = out[1] = out[2] = px[0];
                    out[3] = channels == 2 ? px[1] : std::uint8_t{255};
                } else {
                    out[0] = px[0];
                    out[1] = px[1];
                    out[2] = px[2];
                    out[3] = channels == 4 ? px[3] : std::uint8_t{255};
                }
            }
        }
        desc.textures.push_back(std::move(tex));
    }
    const int nmat = static_cast<int>(m->nmat);
    desc.materials.reserve(static_cast<std::size_t>(nmat));
    for (int i = 0; i < nmat; ++i) {
        const auto mi = static_cast<std::size_t>(i);
        terra::data::MaterialDescription mat;
        mat.name = nameOf(m, mjOBJ_MATERIAL, i);
        const float* rgba = m->mat_rgba + 4 * mi;
        mat.rgba = {rgba[0], rgba[1], rgba[2], rgba[3]};
        mat.emission = m->mat_emission[i];
        mat.specular = m->mat_specular[i];
        mat.shininess = m->mat_shininess[i];
        mat.reflectance = m->mat_reflectance[i];
        mat.metallic = m->mat_metallic[i];
        mat.roughness = m->mat_roughness[i];
        // <material texture="…"> 写入 RGB 角色；也接受 RGBA 角色与未指定角色
        const int* texid = m->mat_texid + static_cast<std::size_t>(mjNTEXROLE) * mi;
        for (const int role : {static_cast<int>(mjTEXROLE_RGB), static_cast<int>(mjTEXROLE_RGBA), static_cast<int>(mjTEXROLE_USER)}) {
            if (texid[role] >= 0) {
                mat.texture = texid[role];
                break;
            }
        }
        mat.texRepeat = {m->mat_texrepeat[2 * mi], m->mat_texrepeat[2 * mi + 1]};
        mat.texUniform = m->mat_texuniform[i] != 0;
        desc.materials.push_back(std::move(mat));
    }

    const int ngeom = static_cast<int>(m->ngeom);
    desc.geoms.reserve(static_cast<std::size_t>(ngeom));
    for (int g = 0; g < ngeom; ++g) {
        const auto gi = static_cast<std::size_t>(g);
        terra::data::GeomDescription geom;
        geom.id = GeomId{static_cast<std::uint32_t>(g + 1)};
        const std::string raw = nameOf(m, mjOBJ_GEOM, g);
        geom.name = InstanceNames::visible(raw);
        geom.entity = bodyToEntity[static_cast<std::size_t>(m->geom_bodyid[g])];
        geom.modelInstance = names.instanceOf(raw);
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
        geom.material = matid;
        desc.geoms.push_back(std::move(geom));
    }
}

// 执行器类别与参数（M3-02）。MuJoCo 的 <motor>、<position>、<velocity> 都是 <general> 的简写，编译后只留下增益与偏置：
//   motor：增益固定，无偏置；position：偏置 = [0, −kp, −kv]；velocity：偏置 = [0, 0, −kv]（增益分别为 kp、kv）。
// 带动力学（dyntype ≠ none）或不符合以上形式的记为 Other。
void describeActuatorKind(const mjModel* m, int a, terra::data::ActuatorDescription& act) {
    const auto ai = static_cast<std::size_t>(a);
    const mjtNum* gain = m->actuator_gainprm + mjNGAIN * ai;
    const mjtNum* bias = m->actuator_biasprm + mjNBIAS * ai;
    const auto close = [](mjtNum x, mjtNum y) { return std::fabs(x - y) <= 1e-12 * std::max<mjtNum>(1.0, std::fabs(y)); };
    act.kind = terra::data::ActuatorKind::Other;
    if (m->actuator_dyntype[a] == mjDYN_NONE && m->actuator_gaintype[a] == mjGAIN_FIXED) {
        if (m->actuator_biastype[a] == mjBIAS_NONE) {
            act.kind = terra::data::ActuatorKind::Motor;
            act.gain = gain[0];
        } else if (m->actuator_biastype[a] == mjBIAS_AFFINE && close(bias[0], 0.0)) {
            if (close(bias[1], -gain[0]) && !close(gain[0], 0.0)) {
                act.kind = terra::data::ActuatorKind::Position;
                act.gain = gain[0];
                act.damping = -bias[2];
            } else if (close(bias[1], 0.0) && close(bias[2], -gain[0]) && !close(gain[0], 0.0)) {
                act.kind = terra::data::ActuatorKind::Velocity;
                act.gain = gain[0];
            }
        }
    }
    act.gear = m->actuator_gear[6 * static_cast<std::size_t>(m->actuator_outadr[a])];
    act.forceLimited = m->actuator_forcelimited[a] != 0;
    act.forceMin = m->actuator_forcerange[2 * ai];
    act.forceMax = m->actuator_forcerange[2 * ai + 1];
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

    // IMU（M4-02）：安装点下标与加速度计、陀螺仪在 sensordata 中的地址，与 description.sensors 一一对应
    struct ImuChannel {
        int site = -1;
        int accelerometer = -1;
        int gyro = -1;
    };
    std::vector<ImuChannel> imus;

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

    // 执行器（M3-04）：控制量与输出力
    snap->actuators.reserve(description.actuators.size());
    for (std::size_t a = 0; a < description.actuators.size(); ++a) {
        terra::data::ActuatorState as;
        as.id = description.actuators[a].id;
        const mjtNum* ctrl = d->ctrl + m->actuator_ctrladr[a];
        for (int k = 0; k < m->actuator_ctrlnum[a]; ++k) as.controls.push_back(ctrl[k]);
        as.force = d->actuator_force[a];
        finite = finite && allFinite(ctrl, m->actuator_ctrlnum[a]) && std::isfinite(as.force);
        snap->actuators.push_back(std::move(as));
    }

    // 传感器（M4-02）：IMU 的比力与角速度取 MuJoCo 传感器的输出（安装点坐标系），姿态由安装点的世界旋转矩阵换算
    snap->sensors.reserve(imus.size());
    for (std::size_t k = 0; k < imus.size(); ++k) {
        const ImuChannel& ch = imus[k];
        terra::data::SensorReading r;
        r.id = description.sensors[k].id;
        r.values.resize(10);
        const mjtNum* acc = d->sensordata + ch.accelerometer;
        const mjtNum* gyro = d->sensordata + ch.gyro;
        mjtNum quat[4];
        mju_mat2Quat(quat, d->site_xmat + 9 * static_cast<std::size_t>(ch.site));
        for (std::size_t c = 0; c < 3; ++c) {
            r.values[c] = acc[c];
            r.values[3 + c] = gyro[c];
        }
        for (std::size_t c = 0; c < 4; ++c) r.values[6 + c] = quat[c];
        finite = finite && allFinite(r.values.data(), 10);
        snap->sensors.push_back(std::move(r));
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

    // 直接加载单个 MJCF：唯一的模型实例，名字为空（不加前缀），放置位姿为单位变换（M2-03）。
    BuildInput input;
    input.sourcePath = path.string();
    input.instances.push_back({ModelInstanceId{1}, std::string{}, path.string(), terra::data::Pose{}});
    input.prefixes.emplace_back();
    return build(model.release(), input, std::move(result));
}

LoadResult MujocoWorld::build(mjModel* rawModel, const BuildInput& inputs, LoadResult result) {
    ModelPtr model{rawModel};
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
    desc.sourcePath = inputs.sourcePath;
    desc.timestep = timestep;
    desc.modelInstances = inputs.instances;
    const InstanceNames names{inputs.prefixes};
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
        const std::string raw = nameOf(model.get(), mjOBJ_BODY, b);
        desc.entities.push_back({id, InstanceNames::visible(raw), bodyToEntity[static_cast<std::size_t>(parentBody)],
                                 names.instanceOf(raw), model->body_mass[b]});
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
        // M3-02：限位、轴向、位置与第一个自由度的被动参数
        const auto ji = static_cast<std::size_t>(j);
        joint.limited = model->jnt_limited[j] != 0 && (joint.type == JointType::Hinge || joint.type == JointType::Slide);
        joint.rangeMin = model->jnt_range[2 * ji];
        joint.rangeMax = model->jnt_range[2 * ji + 1];
        if (joint.type == JointType::Hinge || joint.type == JointType::Slide) {
            joint.axis = {model->jnt_axis[3 * ji], model->jnt_axis[3 * ji + 1], model->jnt_axis[3 * ji + 2]};
        }
        joint.anchor = {model->jnt_pos[3 * ji], model->jnt_pos[3 * ji + 1], model->jnt_pos[3 * ji + 2]};
        const auto dof = static_cast<std::size_t>(model->jnt_dofadr[j]);
        joint.damping = model->dof_damping[dof];
        joint.armature = model->dof_armature[dof];
        joint.frictionLoss = model->dof_frictionloss[dof];
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
        describeActuatorKind(model.get(), a, act);
        desc.actuators.push_back(std::move(act));
    }

    describeGeometry(model.get(), bodyToEntity, names, desc);
    for (int k = 0; k < static_cast<int>(model->nkey); ++k) {
        desc.keyframes.push_back({nameOf(model.get(), mjOBJ_KEY, k), model->key_time[k]});
    }

    // 相机（M4-03）：全部列出；跟踪模式与正交投影只标记，由使用方决定是否支持
    for (int c = 0; c < static_cast<int>(model->ncam); ++c) {
        const auto ci = static_cast<std::size_t>(c);
        const std::string raw = nameOf(model.get(), mjOBJ_CAMERA, c);
        terra::data::CameraDescription camera;
        camera.name = InstanceNames::visible(raw);
        camera.entity = bodyToEntity[static_cast<std::size_t>(model->cam_bodyid[c])];
        camera.modelInstance = names.instanceOf(raw);
        const mjtNum* pos = model->cam_pos + 3 * ci;
        const mjtNum* quat = model->cam_quat + 4 * ci;
        camera.mount.position = {pos[0], pos[1], pos[2]};
        camera.mount.orientation = {quat[0], quat[1], quat[2], quat[3]};
        camera.fovyDegrees = model->cam_fovy[c];
        const int* resolution = model->cam_resolution + 2 * ci;
        if (resolution[0] > 1 && resolution[1] > 1) {
            camera.width = resolution[0];
            camera.height = resolution[1];
        }
        camera.fixed = model->cam_mode[c] == mjCAMLIGHT_FIXED;
        camera.perspective = model->cam_projection[c] == mjPROJ_PERSPECTIVE;
        desc.cameras.push_back(std::move(camera));
    }

    // 传感器（M4-02）：同一安装点上同时有加速度计与陀螺仪即为一个 IMU（各取第一个），按安装点顺序。
    // 场景文件插入的传感器名字带隐藏标记，据此区分“MJCF 中原有”与“场景添加”。
    struct ImuSensors {
        int accelerometer = -1;
        int gyro = -1;
        bool added = false;
    };
    std::map<int, ImuSensors> imuBySite;
    for (int k = 0; k < static_cast<int>(model->nsensor); ++k) {
        const int type = model->sensor_type[k];
        if (model->sensor_objtype[k] != mjOBJ_SITE || (type != mjSENS_ACCELEROMETER && type != mjSENS_GYRO)) continue;
        ImuSensors& entry = imuBySite[model->sensor_objid[k]];
        int& slot = type == mjSENS_ACCELEROMETER ? entry.accelerometer : entry.gyro;
        if (slot >= 0) continue;
        slot = model->sensor_adr[k];
        entry.added = entry.added || nameOf(model.get(), mjOBJ_SENSOR, k).find(kHiddenNameMark) != std::string::npos;
    }
    for (const auto& [site, entry] : imuBySite) {
        if (entry.accelerometer < 0 || entry.gyro < 0) continue;
        const auto si = static_cast<std::size_t>(site);
        const std::string raw = nameOf(model.get(), mjOBJ_SITE, site);
        terra::data::SensorDescription sensor;
        sensor.id = terra::data::SensorId{static_cast<std::uint32_t>(desc.sensors.size() + 1)};
        sensor.name = InstanceNames::visible(raw);
        sensor.kind = terra::data::SensorKind::Imu;
        sensor.entity = bodyToEntity[static_cast<std::size_t>(model->site_bodyid[site])];
        sensor.modelInstance = names.instanceOf(raw);
        const mjtNum* pos = model->site_pos + 3 * si;
        const mjtNum* quat = model->site_quat + 4 * si;
        sensor.mount.position = {pos[0], pos[1], pos[2]};
        sensor.mount.orientation = {quat[0], quat[1], quat[2], quat[3]};
        sensor.added = entry.added;
        desc.sensors.push_back(std::move(sensor));
        impl->imus.push_back({site, entry.accelerometer, entry.gyro});
    }

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

bool MujocoWorld::resetToKeyframe(std::size_t index) {
    Impl& w = *impl_;
    if (index >= static_cast<std::size_t>(w.model->nkey)) return false;
    w.cachedSnapshot.reset();
    {
        const ActiveWorldGuard active(&w.description, &w.stepIndex, w.data.get(), "reset");
        mj_resetDataKeyframe(w.model.get(), w.data.get(), static_cast<int>(index));
        // Terra 的仿真时间从重置起算（第 0 步 ⇔ 时间 0，与 reset 相同）；关键帧自带的时间只记录在描述中
        w.data->time = 0;
        mj_forward(w.model.get(), w.data.get());
    }
    w.stepIndex = 0;
    w.runEpoch += 1;
    w.faulted = false;
    w.captureWarningBaseline();
    return true;
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

// 射线检测（M4-03b）。Collision：MuJoCo 的射线不区分碰撞与显示，打到不参与碰撞的几何体时从命中点稍向前继续，
// 直到打到参与碰撞的几何体、超出量程或没有命中（最多 64 次）。Visual：只打第 0～2 组（默认显示的分组）。
// M5-11：第一次求交用 mj_multiRay（同一起点的多条射线共用几何体的包围检测，3D 雷达每帧 20,000 条），
// 只有打到不参与碰撞的几何体的射线再逐条用 mj_ray 继续。mj_multiRay 需要可写的 mjData（只用其临时栈，结束时恢复），
// 这里去掉 const：不改变任何仿真状态。
std::vector<double> MujocoWorld::castRays(const RayQuery& query) const {
    const mjModel* m = impl_->model.get();
    mjData* d = const_cast<mjData*>(impl_->data.get());
    const std::size_t n = query.directions.size();
    std::vector<double> out(n, std::numeric_limits<double>::infinity());
    if (n == 0) return out;
    int exclude = -1;
    if (query.exclude.valid() && query.exclude.value <= impl_->entityToBody.size()) {
        exclude = impl_->entityToBody[query.exclude.value - 1];
    }
    const mjtByte visualGroups[mjNGROUP] = {1, 1, 1, 0, 0, 0};
    const mjtByte* groups = query.hits == RayTargets::Visual ? visualGroups : nullptr;
    constexpr double kStepPast = 1e-9;
    std::vector<mjtNum> vec(3 * n), dist(n), normal(3 * n);
    std::vector<int> geom(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        vec[3 * i] = query.directions[i].x;
        vec[3 * i + 1] = query.directions[i].y;
        vec[3 * i + 2] = query.directions[i].z;
    }
    const mjtNum origin[3] = {query.origin.x, query.origin.y, query.origin.z};
    const mjtNum cutoff = query.maxRange > 0 ? query.maxRange : mjMAXVAL;
    mj_multiRay(m, d, origin, vec.data(), groups, 1, exclude, geom.data(), dist.data(), normal.data(), static_cast<int>(n), cutoff);
    for (std::size_t i = 0; i < n; ++i) {
        if (dist[i] < 0 || geom[i] < 0 || dist[i] > query.maxRange) continue;
        const bool collides = m->geom_contype[geom[i]] != 0 || m->geom_conaffinity[geom[i]] != 0;
        if (query.hits == RayTargets::Visual || collides) {
            out[i] = dist[i];
            continue;
        }
        // 打到不参与碰撞的几何体：从命中点稍向前逐条继续
        const terra::data::Vec3& dir = query.directions[i];
        const mjtNum v[3] = {dir.x, dir.y, dir.z};
        double travelled = dist[i] + kStepPast;
        for (int hop = 1; hop < 64; ++hop) {
            const mjtNum pnt[3] = {query.origin.x + travelled * dir.x, query.origin.y + travelled * dir.y,
                                   query.origin.z + travelled * dir.z};
            int g = -1;
            const mjtNum dd = mj_ray(m, d, pnt, v, groups, 1, exclude, &g, nullptr);
            if (dd < 0 || g < 0) break;
            const double total = travelled + dd;
            if (total > query.maxRange) break;
            if (m->geom_contype[g] != 0 || m->geom_conaffinity[g] != 0) {
                out[i] = total;
                break;
            }
            travelled = total + kStepPast;
        }
    }
    return out;
}

// ---- 世界组合（M2-04）----

namespace {

struct SpecDeleter {
    void operator()(mjSpec* s) const noexcept { mj_deleteSpec(s); }
};
using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

std::string trimmed(std::string text) {
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.pop_back();
    return text;
}

PhysicsOptions optionsOf(const mjOption& o) {
    PhysicsOptions p;
    p.timestep = o.timestep;
    p.gravity = {o.gravity[0], o.gravity[1], o.gravity[2]};
    p.integrator = integratorName(o.integrator);
    p.solver = solverName(o.solver);
    p.iterations = o.iterations;
    p.tolerance = o.tolerance;
    return p;
}

std::optional<int> integratorFromName(const std::string& name) {
    for (const int i : {mjINT_EULER, mjINT_RK4, mjINT_IMPLICIT, mjINT_IMPLICITFAST, mjINT_DISCRETE}) {
        if (integratorName(i) == name) return i;
    }
    return std::nullopt;
}

std::optional<int> solverFromName(const std::string& name) {
    for (const int s : {mjSOL_PGS, mjSOL_CG, mjSOL_NEWTON}) {
        if (solverName(s) == name) return s;
    }
    return std::nullopt;
}

// 6 项之外的全局物理设置中，与第一个实例不同的字段名（组合后一律取第一个实例的值）。
std::vector<std::string> otherOptionDifferences(const mjSpec& a, const mjSpec& b) {
    std::vector<std::string> fields;
    const mjOption& x = a.option;
    const mjOption& y = b.option;
    const auto num = [&](const char* name, mjtNum p, mjtNum q) {
        if (p != q) fields.emplace_back(name);
    };
    const auto vec = [&](const char* name, const mjtNum* p, const mjtNum* q, int n) {
        for (int i = 0; i < n; ++i) {
            if (p[i] != q[i]) {
                fields.emplace_back(name);
                return;
            }
        }
    };
    const auto integer = [&](const char* name, long long p, long long q) {
        if (p != q) fields.emplace_back(name);
    };
    num("impratio", x.impratio, y.impratio);
    num("ls_tolerance", x.ls_tolerance, y.ls_tolerance);
    num("noslip_tolerance", x.noslip_tolerance, y.noslip_tolerance);
    num("ccd_tolerance", x.ccd_tolerance, y.ccd_tolerance);
    num("sleep_tolerance", x.sleep_tolerance, y.sleep_tolerance);
    vec("wind", x.wind, y.wind, 3);
    vec("magnetic", x.magnetic, y.magnetic, 3);
    num("density", x.density, y.density);
    num("viscosity", x.viscosity, y.viscosity);
    num("o_margin", x.o_margin, y.o_margin);
    vec("o_solref", x.o_solref, y.o_solref, mjNREF);
    vec("o_solimp", x.o_solimp, y.o_solimp, mjNIMP);
    vec("o_friction", x.o_friction, y.o_friction, 5);
    integer("cone", x.cone, y.cone);
    integer("jacobian", x.jacobian, y.jacobian);
    integer("ls_iterations", x.ls_iterations, y.ls_iterations);
    integer("noslip_iterations", x.noslip_iterations, y.noslip_iterations);
    integer("ccd_iterations", x.ccd_iterations, y.ccd_iterations);
    integer("disableflags", x.disableflags, y.disableflags);
    integer("enableflags", x.enableflags, y.enableflags);
    integer("disableactuator", x.disableactuator, y.disableactuator);
    integer("sdf_initpoints", x.sdf_initpoints, y.sdf_initpoints);
    integer("sdf_iterations", x.sdf_iterations, y.sdf_iterations);
    integer("size/memory", a.memory, b.memory);
    integer("size/nemax", a.nemax, b.nemax);
    integer("size/nuserdata", a.nuserdata, b.nuserdata);
    integer("size/njmax", a.njmax, b.njmax);
    integer("size/nconmax", a.nconmax, b.nconmax);
    integer("size/nstack", a.nstack, b.nstack);
    return fields;
}

// 全局设置随第一个实例：物理选项与尺寸（编译器设置由各实例自己保留，MuJoCo 按元素所属的子模型应用）。
void copyGlobals(mjSpec& to, const mjSpec& from) {
    to.option = from.option;
    to.memory = from.memory;
    to.nemax = from.nemax;
    to.nuserdata = from.nuserdata;
    to.njmax = from.njmax;
    to.nconmax = from.nconmax;
    to.nstack = from.nstack;
}

void applyOptions(mjOption& o, const PhysicsOptions& p, int integrator, int solver) {
    o.timestep = p.timestep;
    o.gravity[0] = p.gravity.x;
    o.gravity[1] = p.gravity.y;
    o.gravity[2] = p.gravity.z;
    o.integrator = integrator;
    o.solver = solver;
    o.iterations = p.iterations;
    o.tolerance = p.tolerance;
}

// 给没有名字的 body 与几何体起临时名字，使组合后每个元素都能按实例前缀归属（显示时去掉，见 InstanceNames）。
void nameUnnamed(mjSpec* spec, mjtObj type, const char* kind) {
    int counter = 0;
    for (mjsElement* e = mjs_firstElement(spec, type); e != nullptr; e = mjs_nextElement(spec, e)) {
        const char* name = mjs_getString(mjs_getName(e));
        if (name == nullptr || name[0] == '\0') {
            const std::string hidden = std::string{kHiddenNameMark} + kind + std::to_string(counter++);
            mjs_setName(e, hidden.c_str());
        }
    }
}

// 场景文件添加的 IMU（M4-02）：在实例自己的 spec 中（组合之前，名字不带前缀）插入安装点与传感器。
// 返回空表示成功，否则为错误原因。插入的传感器以隐藏标记命名（对外不可见，也不与模型中的名字冲突）。
std::string addImu(mjSpec* spec, const ImuMount& imu) {
    const auto listOf = [&](mjtObj type, const std::string& what) {
        std::string list;
        int count = 0;
        for (mjsElement* e = mjs_firstElement(spec, type); e != nullptr; e = mjs_nextElement(spec, e)) {
            const char* name = mjs_getString(mjs_getName(e));
            if (name == nullptr || name[0] == '\0' || std::string{name}.find(kHiddenNameMark) != std::string::npos) continue;
            if (count < 8) list += (list.empty() ? "" : "、") + std::string{name};
            ++count;
        }
        if (count == 0) return "；模型中没有命名的" + what;
        return "；模型中的" + what + "：" + list + (count > 8 ? " 等 " + std::to_string(count) + " 个" : std::string{});
    };
    std::string site = imu.site;
    if (site.empty()) {
        mjsBody* body = mjs_findBody(spec, imu.body.c_str());
        if (body == nullptr) return "找不到实体“" + imu.body + "”" + listOf(mjOBJ_BODY, "实体");
        if (mjs_findElement(spec, mjOBJ_SITE, imu.name.c_str()) != nullptr) {
            return "模型中已有名为“" + imu.name + "”的安装点；要装在它上面时请写 site，否则换一个名字";
        }
        mjsSite* added = mjs_addSite(body, nullptr);
        if (added == nullptr || mjs_setName(added->element, imu.name.c_str()) != 0) return "无法添加安装点“" + imu.name + "”";
        const terra::data::Pose& p = imu.pose;
        added->pos[0] = p.position.x;
        added->pos[1] = p.position.y;
        added->pos[2] = p.position.z;
        added->quat[0] = p.orientation.w;
        added->quat[1] = p.orientation.x;
        added->quat[2] = p.orientation.y;
        added->quat[3] = p.orientation.z;
        site = imu.name;
    } else if (mjs_findElement(spec, mjOBJ_SITE, site.c_str()) == nullptr) {
        return "找不到安装点“" + site + "”" + listOf(mjOBJ_SITE, "安装点");
    }
    bool hasAccelerometer = false;
    bool hasGyro = false;
    for (mjsElement* e = mjs_firstElement(spec, mjOBJ_SENSOR); e != nullptr; e = mjs_nextElement(spec, e)) {
        const mjsSensor* sensor = mjs_asSensor(e);
        const char* object = sensor != nullptr ? mjs_getString(sensor->objname) : nullptr;
        if (sensor == nullptr || sensor->objtype != mjOBJ_SITE || object == nullptr || site != object) continue;
        hasAccelerometer = hasAccelerometer || sensor->type == mjSENS_ACCELEROMETER;
        hasGyro = hasGyro || sensor->type == mjSENS_GYRO;
    }
    for (const auto& [type, have, tag] : {std::make_tuple(mjSENS_ACCELEROMETER, hasAccelerometer, "accelerometer_"),
                                          std::make_tuple(mjSENS_GYRO, hasGyro, "gyro_")}) {
        if (have) continue;
        mjsSensor* sensor = mjs_addSensor(spec);
        if (sensor == nullptr) return "无法为“" + site + "”添加传感器";
        sensor->type = type;
        sensor->objtype = mjOBJ_SITE;
        mjs_setString(sensor->objname, site.c_str());
        const std::string hidden = std::string{kHiddenNameMark} + tag + site;
        if (mjs_setName(sensor->element, hidden.c_str()) != 0) return "无法为“" + site + "”添加传感器（名字冲突）";
    }
    return {};
}

// 关键帧中的自由关节位姿相对实例的放置框架（M3-02）。MuJoCo 把关键帧里自由关节的值当作世界位姿，组合时不随
// mjs_attach 的框架变换，放在别处的机器人按关键帧重置后会回到原点附近。这里把属于实例 i 的关键帧（名字以其前缀开头）
// 中、属于实例 i 的自由关节的位置与姿态乘上放置位姿，线速度一并旋转（角速度在 body 坐标系中，不变）。
// 关键帧没有给出该关节时 MuJoCo 以 qpos0 补齐，而 qpos0 已含放置位姿：与 qpos0 逐位相同的值不再变换。
void placeKeyframes(mjModel* m, const std::vector<ModelPlacement>& models) {
    for (int k = 0; k < static_cast<int>(m->nkey); ++k) {
        const std::string key = nameOf(m, mjOBJ_KEY, k);
        for (const ModelPlacement& placed : models) {
            const terra::data::Pose& p = placed.placement;
            if (placed.prefix.empty() || key.rfind(placed.prefix, 0) != 0) continue;
            const mjtNum t[3] = {p.position.x, p.position.y, p.position.z};
            const mjtNum r[4] = {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
            for (int j = 0; j < static_cast<int>(m->njnt); ++j) {
                if (m->jnt_type[j] != mjJNT_FREE) continue;
                if (nameOf(m, mjOBJ_BODY, m->jnt_bodyid[j]).rfind(placed.prefix, 0) != 0) continue;
                const auto q0 = static_cast<std::size_t>(m->jnt_qposadr[j]);
                mjtNum* qpos = m->key_qpos + static_cast<std::size_t>(k) * static_cast<std::size_t>(m->nq) + q0;
                if (std::equal(qpos, qpos + 7, m->qpos0 + q0)) continue;
                mjtNum pos[3];
                mju_rotVecQuat(pos, qpos, r);
                mjtNum quat[4];
                mju_mulQuat(quat, r, qpos + 3);
                for (int c = 0; c < 3; ++c) qpos[c] = pos[c] + t[c];
                for (int c = 0; c < 4; ++c) qpos[3 + c] = quat[c];
                mjtNum* qvel = m->key_qvel + static_cast<std::size_t>(k) * static_cast<std::size_t>(m->nv) +
                               static_cast<std::size_t>(m->jnt_dofadr[j]);
                mjtNum lin[3];
                mju_rotVecQuat(lin, qvel, r);
                for (int c = 0; c < 3; ++c) qvel[c] = lin[c];
            }
        }
    }
}

}  // namespace

ComposeResult MujocoWorld::compose(const WorldComposition& composition) {
    configureMujocoLogging();
    namespace fs = std::filesystem;
    using Severity = ComposeIssue::Severity;
    ComposeResult out;
    const auto error = [&](int model, LoadErrorKind kind, std::string message) {
        out.issues.push_back({Severity::Error, model, kind, std::move(message)});
    };
    const auto warning = [&](int model, std::string message) {
        out.issues.push_back({Severity::Warning, model, LoadErrorKind::InvalidModel, std::move(message)});
    };
    const auto hasError = [&] {
        return std::any_of(out.issues.begin(), out.issues.end(),
                           [](const ComposeIssue& i) { return i.severity == Severity::Error; });
    };
    const auto& models = composition.models;
    if (models.empty()) {
        error(-1, LoadErrorKind::InvalidModel, "组合请求中没有模型实例");
        return out;
    }

    // 1. 逐个解析
    std::vector<SpecPtr> children(models.size());
    std::vector<std::string> paths(models.size());
    for (std::size_t i = 0; i < models.size(); ++i) {
        const int index = static_cast<int>(i);
        std::error_code ec;
        fs::path path = fs::absolute(models[i].mjcfPath, ec);
        if (ec) path = models[i].mjcfPath;
        path = path.lexically_normal();
        paths[i] = path.string();
        if (models[i].mjcfPath.empty()) {
            error(index, LoadErrorKind::FileUnavailable, "模型路径为空");
            continue;
        }
        if (std::string problem = checkFile(path); !problem.empty()) {
            error(index, LoadErrorKind::FileUnavailable, std::move(problem));
            continue;
        }
        std::array<char, 1024> message{};
        children[i].reset(mj_parseXML(paths[i].c_str(), nullptr, message.data(), static_cast<int>(message.size())));
        if (!children[i]) {
            error(index, LoadErrorKind::InvalidModel,
                  "MJCF 解析失败（" + paths[i] + "）：" + trimmed(message[0] != '\0' ? message.data() : "未知错误"));
        } else if (message[0] != '\0') {
            warning(index, "MJCF 解析警告（" + paths[i] + "）：" + trimmed(message.data()));
        }
    }
    if (hasError()) return out;

    // 2. 全局配置：6 项由场景决定，其余随第一个实例
    std::vector<PhysicsOptions> options;
    options.reserve(children.size());
    for (const auto& child : children) options.push_back(optionsOf(child->option));
    const PhysicsOptions chosen = composition.resolveOptions ? composition.resolveOptions(options) : options.front();
    const std::optional<int> integrator = integratorFromName(chosen.integrator);
    const std::optional<int> solver = solverFromName(chosen.solver);
    if (!(std::isfinite(chosen.timestep) && chosen.timestep > 0)) {
        error(-1, LoadErrorKind::InvalidModel, "全局步长无效：" + std::to_string(chosen.timestep));
    }
    if (!integrator) error(-1, LoadErrorKind::InvalidModel, "未知的积分器：" + chosen.integrator);
    if (!solver) error(-1, LoadErrorKind::InvalidModel, "未知的求解器：" + chosen.solver);
    if (!(chosen.iterations > 0)) error(-1, LoadErrorKind::InvalidModel, "求解器迭代次数无效：" + std::to_string(chosen.iterations));
    if (!(std::isfinite(chosen.tolerance) && chosen.tolerance >= 0)) {
        error(-1, LoadErrorKind::InvalidModel, "求解器容差无效：" + std::to_string(chosen.tolerance));
    }
    if (!std::isfinite(chosen.gravity.x) || !std::isfinite(chosen.gravity.y) || !std::isfinite(chosen.gravity.z)) {
        error(-1, LoadErrorKind::InvalidModel, "重力含非有限值");
    }
    if (!integrator || !solver || hasError()) return out;

    SpecPtr parent{mj_makeSpec()};
    if (!parent) {
        error(-1, LoadErrorKind::Internal, "无法创建组合模型");
        return out;
    }
    copyGlobals(*parent, *children.front());
    parent->authored = children.front()->authored;
    applyOptions(parent->option, chosen, *integrator, *solver);
    for (std::size_t i = 0; i < children.size(); ++i) {
        if (i > 0) {
            const std::vector<std::string> fields = otherOptionDifferences(*children[i], *children.front());
            if (!fields.empty()) {
                std::string list;
                for (const auto& f : fields) list += (list.empty() ? "" : "、") + f;
                warning(static_cast<int>(i), "以下全局物理设置与第一个实例“" + models.front().instanceName +
                                                 "”不同，组合后采用第一个实例的值：" + list);
            }
        }
        // 使每个实例的全局设置与组合模型完全相同，组合时不产生属性冲突
        copyGlobals(*children[i], *parent);
    }

    // 2b. 悬挂模式（M3-04）：删掉根部 body 的自由关节。MuJoCo 只有在模型编译过之后，删除元素时才会把关键帧的值
    //     按元素保存、在下次编译时重新排列（user_model.cc）；未编译时关键帧仍按旧的 qpos 长度，会编译失败。
    //     因此先编译一次、删除、再编译一次（第二次编译使关键帧不再处于待定状态，组合时才能正确加上实例前缀）。
    for (std::size_t i = 0; i < children.size(); ++i) {
        if (!models[i].fixedBase) continue;
        const int index = static_cast<int>(i);
        ModelPtr before{mj_compile(children[i].get(), nullptr)};
        if (!before) {
            error(index, LoadErrorKind::InvalidModel, "悬挂模式：模型编译失败（" + paths[i] + "）：" + trimmed(mjs_getError(children[i].get())));
            return out;
        }
        mjsBody* root = mjs_findBody(children[i].get(), "world");
        std::vector<mjsElement*> freeJoints;
        for (mjsElement* b = root ? mjs_firstChild(root, mjOBJ_BODY, 0) : nullptr; b != nullptr; b = mjs_nextChild(root, b, 0)) {
            mjsBody* body = mjs_asBody(b);
            for (mjsElement* j = mjs_firstChild(body, mjOBJ_JOINT, 0); j != nullptr; j = mjs_nextChild(body, j, 0)) {
                if (mjs_asJoint(j)->type == mjJNT_FREE) freeJoints.push_back(j);
            }
        }
        if (freeJoints.empty()) {
            warning(index, "fixedBase：模型根部没有自由关节，悬挂模式不起作用");
            continue;
        }
        for (mjsElement* j : freeJoints) {
            if (mjs_delete(children[i].get(), j) != 0) {
                error(index, LoadErrorKind::InvalidModel, "悬挂模式：删除自由关节失败：" + trimmed(mjs_getError(children[i].get())));
                return out;
            }
        }
        ModelPtr after{mj_compile(children[i].get(), nullptr)};
        if (!after) {
            error(index, LoadErrorKind::InvalidModel,
                  "悬挂模式：删除自由关节后编译失败（" + paths[i] + "）：" + trimmed(mjs_getError(children[i].get())));
            return out;
        }
    }

    // 2b'. 位置执行器改为力矩执行器（M4-06b，宇树低层控制）：增益 1、无偏置；控制范围取力限，没有力限时不限。
    //      inheritrange 置 0（否则编译时又按关节范围设置控制范围）。只改“增益固定、仿射偏置且 bias[1] = −gain[0]”的执行器
    for (std::size_t i = 0; i < children.size(); ++i) {
        if (!models[i].torqueActuators) continue;
        int converted = 0;
        for (mjsElement* e = mjs_firstElement(children[i].get(), mjOBJ_ACTUATOR); e != nullptr;
             e = mjs_nextElement(children[i].get(), e)) {
            mjsActuator* a = mjs_asActuator(e);
            const bool position = a->dyntype == mjDYN_NONE && a->gaintype == mjGAIN_FIXED && a->biastype == mjBIAS_AFFINE &&
                                  a->biasprm[0] == 0.0 && a->biasprm[1] == -a->gainprm[0] && a->gainprm[0] != 0.0;
            if (!position) continue;
            for (double& g : a->gainprm) g = 0.0;
            a->gainprm[0] = 1.0;
            a->biastype = mjBIAS_NONE;
            for (double& b : a->biasprm) b = 0.0;
            a->inheritrange = 0.0;
            a->type = mjACTUATOR_MOTOR;
            const bool forceRange = a->forcelimited != mjLIMITED_FALSE && a->forcerange[0] < a->forcerange[1];
            if (forceRange) {
                a->ctrllimited = mjLIMITED_TRUE;
                a->ctrlrange[0] = a->forcerange[0];
                a->ctrlrange[1] = a->forcerange[1];
            } else {
                a->ctrllimited = mjLIMITED_FALSE;
            }
            ++converted;
        }
        if (converted == 0) warning(static_cast<int>(i), "torqueActuators：模型中没有位置执行器，没有改动");
    }

    // 2c. 场景文件添加的 IMU（M4-02）
    for (std::size_t i = 0; i < children.size(); ++i) {
        for (const ImuMount& imu : models[i].imus) {
            if (std::string problem = addImu(children[i].get(), imu); !problem.empty()) {
                error(static_cast<int>(i), LoadErrorKind::InvalidModel, "IMU“" + imu.name + "”：" + problem);
            }
        }
    }
    if (hasError()) return out;

    // 3. 组合：每个实例放在世界中的一个框架上
    mjsBody* world = mjs_findBody(parent.get(), "world");
    for (std::size_t i = 0; i < children.size(); ++i) {
        nameUnnamed(children[i].get(), mjOBJ_BODY, "body_");
        nameUnnamed(children[i].get(), mjOBJ_GEOM, "geom_");
        mjsFrame* frame = mjs_addFrame(world, nullptr);
        const terra::data::Pose& p = models[i].placement;
        frame->pos[0] = p.position.x;
        frame->pos[1] = p.position.y;
        frame->pos[2] = p.position.z;
        frame->quat[0] = p.orientation.w;
        frame->quat[1] = p.orientation.x;
        frame->quat[2] = p.orientation.y;
        frame->quat[3] = p.orientation.z;
        if (mjs_attach(frame->element, children[i]->element, models[i].prefix.c_str(), "") == nullptr) {
            error(static_cast<int>(i), LoadErrorKind::InvalidModel,
                  "组合失败（" + paths[i] + "）：" + trimmed(mjs_getError(parent.get())));
            return out;
        }
    }

    // 4. 编译。编译期间的错误由 MuJoCo 编译器截获并返回，不会终止进程
    ModelPtr model{mj_compile(parent.get(), nullptr)};
    if (!model) {
        error(-1, LoadErrorKind::InvalidModel, "组合后编译失败：" + trimmed(mjs_getError(parent.get())));
        return out;
    }
    for (int k = 0; k < mjs_numWarnings(parent.get()); ++k) {
        if (const char* text = mjs_getWarning(parent.get(), k)) warning(-1, "编译警告：" + trimmed(text));
    }
    placeKeyframes(model.get(), models);

    BuildInput input;
    input.sourcePath = composition.sourceLabel;
    for (std::size_t i = 0; i < models.size(); ++i) {
        input.instances.push_back({ModelInstanceId{static_cast<std::uint32_t>(i + 1)}, models[i].instanceName, paths[i],
                                   models[i].placement});
        input.prefixes.push_back(models[i].prefix);
    }
    LoadResult built = build(model.release(), input, LoadResult{});
    if (!built.world) {
        const LoadError e = built.error.value_or(LoadError{LoadErrorKind::Internal, "未知错误"});
        error(-1, e.kind, e.message);
        return out;
    }
    for (auto& w : built.warnings) warning(-1, std::move(w));
    out.world = std::move(built.world);
    return out;
}

LoadResult loadWorld(const terra::data::ModelSource& source) {
    return MujocoWorld::load(source);
}

ComposeResult composeWorld(const WorldComposition& composition) {
    return MujocoWorld::compose(composition);
}

}  // namespace terra::physics::mujoco
