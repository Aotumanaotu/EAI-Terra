// 通信配置文件与命令合并的单元测试（M4-06a，ros2.profile；只在 ROS2 构建中）。不启动 rclcpp。
//   - 仓库中的通用配置能读取，字段与默认值一致；
//   - 严格校验：未知字段、format / version 错误、频率非正、类型错、topicPrefix 不合法、kind 为 unitree（M4-06b）都报错；
//   - topicName：“-” 等字符换成 “_”，段首数字前加 “_”，“/” 保留；
//   - mergeCommands：同一执行器以后者为准，其余保留，顺序不变。

#include <cstdio>
#include <string>
#include <vector>

#include "terra/ros2/bridge.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace ros = terra::ros2;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool hasError(const ros::ProfileResult& r, const std::string& fragment) {
    for (const auto& e : r.errors) {
        if (e.find(fragment) != std::string::npos) return true;
    }
    return false;
}

void expectError(const std::string& text, const std::string& fragment) {
    const auto r = ros::parseProfile(text, "test.terra-ros2.json");
    std::string all;
    for (const auto& e : r.errors) all += " [" + e + "]";
    check(!r.profile && hasError(r, fragment), "应报错（含“" + fragment + "”）：" + text + "，实际" + all);
}

void testGeneric() {
    const std::string path = std::string{TERRA_ASSETS_DIR} + "/ros2/profiles/generic.terra-ros2.json";
    const auto r = ros::readProfile(path);
    for (const auto& e : r.errors) std::fprintf(stderr, "INFO %s\n", e.c_str());
    check(r.profile.has_value(), "通用配置可以读取");
    if (!r.profile) return;
    const auto& p = *r.profile;
    check(p.kind == "generic" && p.topicPrefix.empty() && p.clock && p.sensors && p.commands && p.services, "通用配置的开关");
    check(p.jointStatesHz == 100.0 && p.tfHz == 100.0, "通用配置的频率 100 Hz");
    check(p.path == path && !p.name.empty(), "路径与名字");

    const auto minimal = ros::parseProfile(R"({"format": "terra.ros2", "version": 1})", "m.json");
    check(minimal.profile && minimal.profile->kind == "generic" && minimal.profile->jointStatesHz == 100.0 && minimal.profile->clock,
          "只有 format 与 version 时取默认值");
    const auto prefixed = ros::parseProfile(R"({"format": "terra.ros2", "version": 1, "topicPrefix": "/sim", "tfHz": 30, "clock": false})", "p.json");
    check(prefixed.profile && prefixed.profile->topicPrefix == "/sim" && prefixed.profile->tfHz == 30.0 && !prefixed.profile->clock,
          "topicPrefix、tfHz、clock");
    check(!ros::readProfile(std::string{TERRA_ASSETS_DIR} + "/ros2/profiles/不存在.terra-ros2.json").profile, "文件不存在时报错");
}

void testStrict() {
    expectError("{", "JSON 语法错误");
    expectError("[]", "应为对象");
    expectError(R"({"format": "terra.scene", "version": 1})", "/format");
    expectError(R"({"format": "terra.ros2"})", "/version");
    expectError(R"({"format": "terra.ros2", "version": 2})", "不受支持");
    expectError(R"({"format": "terra.ros2", "version": 1, "clok": true})", "未知字段“clok”");
    expectError(R"({"format": "terra.ros2", "version": 1, "jointStatesHz": 0})", "/jointStatesHz");
    expectError(R"({"format": "terra.ros2", "version": 1, "tfHz": -5})", "/tfHz");
    expectError(R"({"format": "terra.ros2", "version": 1, "tfHz": "100"})", "/tfHz");
    expectError(R"({"format": "terra.ros2", "version": 1, "sensors": 1})", "/sensors");
    expectError(R"({"format": "terra.ros2", "version": 1, "name": 3})", "/name");
    expectError(R"({"format": "terra.ros2", "version": 1, "kind": "unitree"})", "需要 unitree 字段");
    expectError(R"({"format": "terra.ros2", "version": 1, "kind": "ros1"})", "未知的配置类型");
    expectError(R"({"format": "terra.ros2", "version": 1, "topicPrefix": "sim"})", "/topicPrefix");
    expectError(R"({"format": "terra.ros2", "version": 1, "topicPrefix": "/sim/"})", "/topicPrefix");
    expectError(R"({"format": "terra.ros2", "version": 1, "topicPrefix": "/s-m"})", "/topicPrefix");
}

// M4-06b：宇树配置
void testUnitree() {
    struct Expected {
        const char* file;
        const char* messages;
        const char* instance;
        std::size_t motors;
        const char* first;
        const char* imu;
    };
    const Expected profiles[] = {
        {"unitree_go2", "go", "go2", 12, "FR_hip_joint", "imu"},
        {"unitree_go2w", "go", "go2w", 16, "FR_hip_joint", "imu"},
        {"unitree_g1", "hg", "g1", 29, "left_hip_pitch_joint", "imu_in_pelvis"},
        {"unitree_as2", "hg", "as2", 12, "FR_hip_joint", "imu"},
    };
    for (const auto& e : profiles) {
        const auto r = ros::readProfile(std::string{TERRA_ASSETS_DIR} + "/ros2/profiles/" + e.file + ".terra-ros2.json");
        for (const auto& err : r.errors) std::fprintf(stderr, "INFO %s：%s\n", e.file, err.c_str());
        check(r.profile && r.profile->kind == "unitree" && r.profile->unitree, std::string{e.file} + "：可以读取，kind 为 unitree");
        if (!r.profile || !r.profile->unitree) continue;
        const auto& u = *r.profile->unitree;
        check(u.messages == e.messages && u.instance == e.instance && u.motors.size() == e.motors && u.motors[0] == e.first && u.imu == e.imu,
              std::string{e.file} + "：消息类型、实例、电机表与 IMU");
        check(u.checkCrc && u.lowstateHz == 500.0 && u.lfLowstateHz == 50.0, std::string{e.file} + "：校验 CRC，500 / 50 Hz");
        check(!r.profile->commands, std::string{e.file} + "：不订阅 /terra/joint_commands（命令走 /lowcmd）");
    }
    // 宇树 G1 的电机顺序与 G1JointIndex 相同：12 为 waist_yaw，22 为 right_shoulder_pitch
    const auto g1 = ros::readProfile(std::string{TERRA_ASSETS_DIR} + "/ros2/profiles/unitree_g1.terra-ros2.json");
    if (g1.profile && g1.profile->unitree) {
        const auto& m = g1.profile->unitree->motors;
        check(m[12] == "waist_yaw_joint" && m[22] == "right_shoulder_pitch_joint" && m[28] == "right_wrist_yaw_joint", "G1 电机顺序（G1JointIndex）");
    }

    const std::string head = R"({"format": "terra.ros2", "version": 1, "kind": "unitree", "unitree": )";
    const auto unitree = [&](const std::string& body) { return head + body + "}"; };
    const std::string ok = R"("messages": "go", "instance": "go2", "imu": "imu", "motors": ["FR_hip_joint", "", "FR_calf_joint"])";
    const auto minimal = ros::parseProfile(unitree("{" + ok + "}"), "u.json");
    check(minimal.profile && minimal.profile->unitree && minimal.profile->unitree->motors.size() == 3 &&
              minimal.profile->unitree->motors[1].empty() && minimal.profile->unitree->wirelessControllerHz == 50.0,
          "最小的宇树配置：空字符串表示该编号没有电机，其余取默认值");
    expectError(unitree("[]"), "/unitree");
    expectError(unitree("{" + ok + R"(, "lowcmdHz": 5})"), "未知字段“lowcmdHz”");
    expectError(unitree(R"({"instance": "go2", "imu": "imu", "motors": ["a"]})"), "/unitree/messages");
    expectError(unitree(R"({"messages": "h1", "instance": "go2", "imu": "imu", "motors": ["a"]})"), "/unitree/messages");
    expectError(unitree(R"({"messages": "go", "imu": "imu", "motors": ["a"]})"), "/unitree/instance");
    expectError(unitree(R"({"messages": "go", "instance": "go2", "motors": ["a"]})"), "/unitree/imu");
    expectError(unitree(R"({"messages": "go", "instance": "go2", "imu": "imu"})"), "/unitree/motors");
    expectError(unitree(R"({"messages": "go", "instance": "go2", "imu": "imu", "motors": ["", ""]})"), "没有电机");
    expectError(unitree(R"({"messages": "go", "instance": "go2", "imu": "imu", "motors": [1]})"), "/unitree/motors/0");
    std::string many = R"({"messages": "go", "instance": "go2", "imu": "imu", "motors": [)";
    for (int i = 0; i < 21; ++i) many += std::string{i == 0 ? "" : ", "} + "\"j" + std::to_string(i) + "\"";
    expectError(unitree(many + "]}"), "超过消息的 20 个电机位");
    expectError(unitree("{" + ok + R"(, "feet": ["a", "b"]})"), "/unitree/feet");
    expectError(unitree("{" + ok + R"(, "lowstateHz": 0})"), "/unitree/lowstateHz");
    expectError(unitree("{" + ok + R"(, "lfLowstateHz": -1})"), "/unitree/lfLowstateHz");
    expectError(unitree("{" + ok + R"(, "checkCrc": "no"})"), "/unitree/checkCrc");
    expectError(unitree("{" + ok + R"(, "modeMachine": 300})"), "/unitree/modeMachine");
    expectError(R"({"format": "terra.ros2", "version": 1, "unitree": {}})", "只有 kind 为 unitree");
}

void testTopicName() {
    check(ros::topicName("room/scan") == "room/scan", "“/” 保留");
    check(ros::topicName("my-robot/imu") == "my_robot/imu", "“-” 换成 “_”");
    check(ros::topicName("cam/2nd") == "cam/_2nd", "段首数字前加 “_”");
    check(ros::topicName("a b.c") == "a_b_c", "空格与点");
    check(ros::topicName("相机") == "______", "非 ASCII 字节逐个换成 “_”（UTF-8 每个汉字 3 字节）");
}

void testMerge() {
    using terra::physics::ActuatorCommand;
    using terra::data::ActuatorId;
    const std::vector<ActuatorCommand> base{{ActuatorId{1}, {0.1}}, {ActuatorId{2}, {0.2}}};
    const std::vector<ActuatorCommand> later{{ActuatorId{2}, {2.0}}, {ActuatorId{3}, {3.0}}};
    const auto merged = ros::mergeCommands(base, later);
    check(merged.size() == 3, "合并后 3 条");
    if (merged.size() == 3) {
        check(merged[0].actuator.value == 1 && merged[0].values == std::vector<double>{0.1}, "执行器 1 保留");
        check(merged[1].actuator.value == 2 && merged[1].values == std::vector<double>{2.0}, "执行器 2 以后者为准");
        check(merged[2].actuator.value == 3 && merged[2].values == std::vector<double>{3.0}, "执行器 3 追加在后");
    }
    check(ros::mergeCommands({}, later).size() == 2 && ros::mergeCommands(base, {}).size() == 2, "一边为空");
}

}  // namespace

int main() {
    testGeneric();
    testStrict();
    testUnitree();
    testTopicName();
    testMerge();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
