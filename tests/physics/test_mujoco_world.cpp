// MuJoCo 后端（经 PhysicsWorld 接口）加载与初始化测试（M0-04）。
// 只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif
#ifndef TERRA_TEST_WORK_DIR
#error "需要 TERRA_TEST_WORK_DIR"
#endif

namespace {

namespace fs = std::filesystem;
using terra::data::EntityId;
using terra::data::JointId;
using terra::data::JointType;
using terra::data::ModelSource;
using terra::physics::LoadErrorKind;
using terra::physics::LoadResult;
using terra::physics::PhysicsWorld;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-12) {
    return std::fabs(a - b) <= tol;
}

const fs::path kCube = fs::path{TERRA_TEST_ASSETS_DIR} / "falling_cube.xml";

fs::path workDir() {
    static const fs::path dir = [] {
        fs::path d{TERRA_TEST_WORK_DIR};
        fs::remove_all(d);
        fs::create_directories(d);
        return d;
    }();
    return dir;
}

fs::path writeFile(const fs::path& relative, const std::string& content) {
    const fs::path path = workDir() / relative;
    fs::create_directories(path.parent_path());
    std::ofstream{path} << content;
    return path;
}

LoadResult load(const fs::path& path) {
    return terra::physics::mujoco::loadWorld(ModelSource{path.string()});
}

// 从快照中取实体位姿；未知 ID 返回空。
std::optional<terra::data::Pose> poseOf(const PhysicsWorld& world, EntityId id) {
    const auto snap = world.snapshot();
    for (const auto& e : snap->entities) {
        if (e.id == id) return e.pose;
    }
    return std::nullopt;
}

// 期望加载失败：没有世界，错误类型一致，错误信息包含指定片段。
void expectFailure(const LoadResult& r, LoadErrorKind kind, const std::string& fragment,
                   const std::string& what) {
    check(!r.world, what + "：失败时不应返回世界");
    check(r.error.has_value(), what + "：应有错误");
    if (r.error) {
        check(r.error->kind == kind, what + "：错误类型不符，信息为 " + r.error->message);
        check(r.error->message.find(fragment) != std::string::npos,
              what + "：错误信息应包含 '" + fragment + "'，实际为 " + r.error->message);
    }
}

void testFallingCube() {
    const LoadResult r = load(kCube);
    check(r.world != nullptr && !r.error, "立方体夹具加载成功" + (r.error ? "：" + r.error->message : ""));
    if (!r.world) return;
    check(r.warnings.empty(), "立方体夹具没有编译警告");

    const PhysicsWorld& world = *r.world;
    const auto& desc = world.description();
    check(desc.timestep == 0.001, "步长 0.001");
    check(fs::path{desc.sourcePath}.is_absolute(), "sourcePath 为绝对路径");
    check(fs::equivalent(desc.sourcePath, kCube), "sourcePath 指向夹具文件");

    check(desc.entities.size() == 1, "只有一个实体（地面挂在世界上，不是实体）");
    const auto* cube = desc.findEntity("cube");
    check(cube != nullptr, "按名称找到 cube");
    check(desc.findEntity("ground") == nullptr, "ground 不是实体");
    check(desc.findEntity("") == nullptr, "空名称不参与查找");
    if (cube != nullptr) {
        check(cube->id == EntityId{1}, "cube 的 ID 为 1");
        check(!cube->parent.valid(), "cube 直接挂在世界上");
        check(desc.entity(cube->id) == cube, "entity(id) 与 findEntity 一致");
    }

    check(desc.joints.size() == 1, "只有一个关节");
    const auto* joint = desc.findJoint("cube_free");
    check(joint != nullptr, "按名称找到 cube_free");
    if (joint != nullptr && cube != nullptr) {
        check(joint->id == JointId{1}, "关节 ID 为 1");
        check(joint->type == JointType::Free, "自由关节");
        check(joint->entity == cube->id, "关节属于 cube");
        check(joint->configurationSize == 7 && joint->velocitySize == 6, "自由关节维度 7/6");
    }

    check(world.simulationTime() == 0.0, "初始时间为 0");
    if (cube != nullptr) {
        const auto pose = poseOf(world, cube->id);
        check(pose.has_value(), "cube 有位姿");
        if (pose) {
            check(near(pose->position.x, 0) && near(pose->position.y, 0) && near(pose->position.z, 1),
                  "初始位置 (0, 0, 1)");
            check(near(pose->orientation.w, 1) && near(pose->orientation.x, 0) &&
                      near(pose->orientation.y, 0) && near(pose->orientation.z, 0),
                  "初始姿态为单位四元数");
        }
    }
    check(!poseOf(world, EntityId{}).has_value(), "无效 ID 没有位姿");
    check(!poseOf(world, EntityId{2}).has_value(), "越界 ID 没有位姿");
}

void testRelativePaths() {
    // 相对于当前工作目录的模型路径
    const fs::path relative = fs::relative(kCube, fs::current_path());
    const LoadResult r = load(relative);
    check(r.world != nullptr, "相对路径加载成功：" + relative.string());
    if (r.world) {
        check(fs::path{r.world->description().sourcePath}.is_absolute(), "相对路径解析为绝对路径");
    }

    // MJCF 内部的 include 以模型文件所在目录为基准，而不是当前工作目录
    writeFile("include_test/part.xml",
              "<mujoco><worldbody><body name=\"included\"><geom type=\"sphere\" size=\"0.1\"/></body>"
              "</worldbody></mujoco>");
    const fs::path main = writeFile("include_test/main.xml",
                                    "<mujoco><include file=\"part.xml\"/></mujoco>");
    const LoadResult inc = load(main);
    check(inc.world != nullptr, "include 按模型目录解析" + (inc.error ? "：" + inc.error->message : ""));
    if (inc.world) {
        check(inc.world->description().findEntity("included") != nullptr, "找到 include 进来的实体");
    }
}

void testHierarchyAndJointTypes() {
    const fs::path path = writeFile("hierarchy.xml", R"(
<mujoco>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="arm" pos="0 0 0.2">
        <joint name="shoulder" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/>
        <body name="slider" pos="0.3 0 0">
          <joint name="extend" type="slide" axis="1 0 0"/>
          <geom type="sphere" size="0.03"/>
          <body name="wrist">
            <joint name="wrist_ball" type="ball"/>
            <geom type="sphere" size="0.02"/>
            <body>
              <geom type="sphere" size="0.01"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>)");
    const LoadResult r = load(path);
    check(r.world != nullptr, "层级模型加载" + (r.error ? "：" + r.error->message : ""));
    if (!r.world) return;
    const auto& desc = r.world->description();

    check(desc.entities.size() == 5, "5 个实体（含未命名 body）");
    const auto* base = desc.findEntity("base");
    const auto* arm = desc.findEntity("arm");
    const auto* slider = desc.findEntity("slider");
    const auto* wrist = desc.findEntity("wrist");
    check(base && arm && slider && wrist, "按名称找到各实体");
    if (base && arm && slider && wrist) {
        check(!base->parent.valid(), "base 挂在世界上");
        check(arm->parent == base->id, "arm 的父实体为 base");
        check(slider->parent == arm->id, "slider 的父实体为 arm");
        check(wrist->parent == slider->id, "wrist 的父实体为 slider");
        const auto& unnamed = desc.entities.back();
        check(unnamed.name.empty(), "未命名 body 的名称为空");
        check(unnamed.parent == wrist->id, "未命名 body 的父实体为 wrist");
    }
    for (std::size_t i = 0; i < desc.entities.size(); ++i) {
        check(desc.entities[i].id.value == i + 1, "实体 ID 连续分配");
    }

    struct Expect {
        const char* name;
        JointType type;
        int nq;
        int nv;
        const char* entity;
    };
    for (const Expect& e : {Expect{"shoulder", JointType::Hinge, 1, 1, "arm"},
                            Expect{"extend", JointType::Slide, 1, 1, "slider"},
                            Expect{"wrist_ball", JointType::Ball, 4, 3, "wrist"}}) {
        const auto* j = desc.findJoint(e.name);
        check(j != nullptr, std::string{"找到关节 "} + e.name);
        if (j == nullptr) continue;
        check(j->type == e.type, std::string{"关节类型 "} + e.name);
        check(j->configurationSize == e.nq && j->velocitySize == e.nv, std::string{"关节维度 "} + e.name);
        const auto* owner = desc.findEntity(e.entity);
        check(owner != nullptr && j->entity == owner->id, std::string{"关节所属实体 "} + e.name);
    }
}

void testInitialOrientation() {
    // 绕 X 轴旋转 90°：wxyz = (cos45°, sin45°, 0, 0)。用来确认分量顺序没有错位。
    const fs::path path = writeFile("rotated.xml", R"(
<mujoco>
  <worldbody>
    <body name="rotated" pos="1 2 3" quat="0.7071067811865476 0.7071067811865476 0 0">
      <freejoint/>
      <geom type="box" size="0.1 0.2 0.3"/>
    </body>
  </worldbody>
</mujoco>)");
    const LoadResult r = load(path);
    check(r.world != nullptr, "旋转模型加载");
    if (!r.world) return;
    const auto* e = r.world->description().findEntity("rotated");
    check(e != nullptr, "找到旋转实体");
    if (e == nullptr) return;
    const std::optional<terra::data::Pose> pose = poseOf(*r.world, e->id);
    check(pose.has_value(), "旋转实体有位姿");
    if (!pose) return;
    const terra::data::Quaternion q = pose->orientation;
    const double s = std::sqrt(0.5);
    check(near(pose->position.x, 1) && near(pose->position.y, 2) && near(pose->position.z, 3), "位置 (1, 2, 3)");
    check(near(q.w, s, 1e-9) && near(q.x, s, 1e-9) && near(q.y, 0, 1e-9) && near(q.z, 0, 1e-9),
          "四元数为 (cos45°, sin45°, 0, 0)，顺序 wxyz");
    check(near(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z, 1.0, 1e-12), "四元数已归一化");
}

void testLoadFailures() {
    expectFailure(load(workDir() / "no_such_model.xml"), LoadErrorKind::FileUnavailable, "no_such_model.xml",
                  "文件不存在");
    expectFailure(terra::physics::mujoco::loadWorld(ModelSource{""}), LoadErrorKind::FileUnavailable, "为空", "空路径");
    expectFailure(load(workDir()), LoadErrorKind::FileUnavailable, "不是常规文件", "路径是目录");

    expectFailure(load(writeFile("malformed.xml", "<mujoco><worldbody><body name=\"a\">")),
                  LoadErrorKind::InvalidModel, "MJCF 加载失败", "XML 不完整");
    // 注意：mj_loadXML 会自动识别 URDF（根元素 <robot>），所以这里用既非 MJCF 也非 URDF 的根元素。
    // M0 只承诺 MJCF（D-010），URDF 的行为不在测试与承诺范围内。
    expectFailure(load(writeFile("not_mjcf.xml", "<scene name=\"x\"/>")), LoadErrorKind::InvalidModel,
                  "MJCF 加载失败", "根元素既不是 mujoco 也不是 robot");
    expectFailure(load(writeFile("bad_attribute.xml",
                                 "<mujoco><worldbody><body no_such_attribute=\"1\"/></worldbody></mujoco>")),
                  LoadErrorKind::InvalidModel, "no_such_attribute", "未知属性");
    expectFailure(load(writeFile("missing_asset.xml",
                                 "<mujoco><asset><texture name=\"t\" type=\"2d\" file=\"missing_texture.png\"/>"
                                 "</asset></mujoco>")),
                  LoadErrorKind::InvalidModel, "missing_texture.png", "引用的资源缺失");
    expectFailure(load(writeFile("missing_include.xml", "<mujoco><include file=\"missing_part.xml\"/></mujoco>")),
                  LoadErrorKind::InvalidModel, "missing_part.xml", "include 的文件缺失");
    expectFailure(load(writeFile("zero_timestep.xml", "<mujoco><option timestep=\"0\"/></mujoco>")),
                  LoadErrorKind::InvalidModel, "", "步长为 0");
}

void testLifecycle() {
    // 重复加载与释放
    for (int i = 0; i < 50; ++i) {
        const LoadResult r = load(kCube);
        if (!r.world) {
            check(false, "第 " + std::to_string(i) + " 次加载失败");
            return;
        }
    }
    // 失败与成功交替，失败不应影响后续加载
    for (int i = 0; i < 10; ++i) {
        check(!load(workDir() / "no_such_model.xml").world, "交替：失败");
        check(load(kCube).world != nullptr, "交替：成功");
    }
    // 多个世界同时存在，彼此独立
    std::vector<LoadResult> worlds;
    for (int i = 0; i < 4; ++i) {
        worlds.push_back(load(kCube));
    }
    worlds.push_back(load(workDir() / "hierarchy.xml"));
    for (const auto& r : worlds) {
        check(r.world != nullptr, "同时存在的世界均加载成功");
    }
    if (worlds.front().world && worlds.back().world) {
        check(worlds.front().world->description().entities.size() == 1, "立方体世界不受其他世界影响");
        check(worlds.back().world->description().entities.size() == 5, "层级世界不受其他世界影响");
    }
    worlds.erase(worlds.begin() + 1);  // 中途释放一个世界
    if (worlds.front().world) {
        const auto pose = poseOf(*worlds.front().world, EntityId{1});
        check(pose && near(pose->position.z, 1), "释放其他世界后，剩余世界状态不变");
    }
}

}  // namespace

int main() {
    testFallingCube();
    testRelativePaths();
    testHierarchyAndJointTypes();
    testInitialOrientation();
    testLoadFailures();
    testLifecycle();

    // 不应在当前工作目录留下 MuJoCo 日志文件
    check(!fs::exists(fs::current_path() / "MUJOCO_LOG.TXT"), "没有生成 MUJOCO_LOG.TXT");

    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
