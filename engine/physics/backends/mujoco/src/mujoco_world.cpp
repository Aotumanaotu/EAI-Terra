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
};

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
    result.world.reset(new MujocoWorld(std::move(impl)));
    return result;
}

const terra::data::ModelDescription& MujocoWorld::description() const noexcept {
    return impl_->description;
}

double MujocoWorld::simulationTime() const noexcept {
    return impl_->data->time;
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
