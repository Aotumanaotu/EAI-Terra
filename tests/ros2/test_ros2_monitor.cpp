// ROS2 监视器（M4-08d，ros2.monitor）：用 publish_known.py 发布内容已知的消息，逐项核对。
//   - 话题图：类型、发布者数、QoS；节点列表中有发布者节点；
//   - 频率（/terra_test/imu 100 Hz ± 20%，rclpy 定时器有抖动）与带宽；
//   - 运行时展开：Imu 的字段树、JointState 的 position、LaserScan 的 360 束（只展开前 100 项，实际长度 360）；
//   - 曲线字段：angular_velocity.z 与消息头时间的正弦一致；position[2] = 0.3；不存在的字段与越界的下标给出原因；
//   - 图像：rgb8 像素、16UC1 换算为米（0 为无效）、PNG 压缩图像（测试自带的 stb 解码）；
//   - TF 树：/tf_static 的 odom → base_link（静态）、/tf 的 base_link → laser；
//   - 宇树 unitree_hg/LowState（第三方消息）：motor_state 35 项，[34].q = 0.34（没有宇树消息包时跳过这一项）；
//   - 字段路径的解析；取消订阅。
// 用法：test_ros2_monitor <publish_known.py>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#include "stb_image.h"

#include "terra/ros2/monitor.hpp"

extern char** environ;

namespace {

namespace r2 = terra::ros2;

int failures = 0;

void check(bool condition, const std::string& what) {
    std::printf("%s %s\n", condition ? "PASS" : "FAILED:", what.c_str());
    if (!condition) ++failures;
}

std::vector<std::uint8_t> decode(const std::uint8_t* data, std::size_t size, int& width, int& height) {
    int channels = 0;
    unsigned char* pixels = stbi_load_from_memory(data, static_cast<int>(size), &width, &height, &channels, 4);
    if (pixels == nullptr) return {};
    std::vector<std::uint8_t> out(pixels, pixels + static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4);
    stbi_image_free(pixels);
    return out;
}

const r2::MessageValue* child(const r2::MessageValue& v, const std::string& name) {
    for (const auto& c : v.children) {
        if (c.name == name) return &c;
    }
    return nullptr;
}

template <typename Pred>
bool waitFor(Pred pred, double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return pred();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法：test_ros2_monitor <publish_known.py>\n");
        return 2;
    }
    // 单独的域，只在本机通信（与其他 ROS2 测试相同的做法）
    const std::string domain = std::to_string(101 + getpid() % 100);
    setenv("ROS_DOMAIN_ID", domain.c_str(), 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);

    // 字段路径
    const auto p = r2::parseFieldPath("motor_state[2].q");
    check(p && p->size() == 3 && (*p)[1] == "[2]", "字段路径：motor_state[2].q");
    check(r2::parseFieldPath("angular_velocity.z").has_value() && !r2::parseFieldPath("a..b") && !r2::parseFieldPath("[1]") &&
              !r2::parseFieldPath("a[x]") && !r2::parseFieldPath("a.") && !r2::parseFieldPath("a[1]b"),
          "字段路径：不合法的写法");

    std::string error;
    auto monitor = r2::Monitor::create(error, decode);
    check(monitor != nullptr, "创建监视器：" + error);
    if (!monitor) return 1;

    std::string python = "python3", script = argv[1], flag = "--seconds", seconds = "40";
    char* args[] = {python.data(), script.data(), flag.data(), seconds.data(), nullptr};
    pid_t publisher = 0;
    if (posix_spawnp(&publisher, "python3", nullptr, nullptr, args, environ) != 0) {
        check(false, "启动 publish_known.py");
        return 1;
    }
    const auto stopPublisher = [&] {
        if (publisher > 0) {
            kill(publisher, SIGINT);
            int status = 0;
            waitpid(publisher, &status, 0);
            publisher = 0;
        }
    };

    const bool seen = waitFor([&] { return monitor->topic("/terra_test/imu").has_value() && monitor->topic("/terra_test/compressed").has_value(); }, 20.0);
    check(seen, "话题图中出现 publish_known 的话题");
    if (!seen) {
        stopPublisher();
        return 1;
    }
    const auto imuInfo = monitor->topic("/terra_test/imu");
    check(imuInfo && imuInfo->type == "sensor_msgs/msg/Imu" && imuInfo->publishers == 1 && !imuInfo->qos.empty(), "话题信息：类型、发布者数、QoS");
    const auto scanInfo = monitor->topic("/terra_test/scan");
    check(scanInfo && scanInfo->bestEffort && scanInfo->qos.find("best effort") != std::string::npos, "/terra_test/scan 的发布者为 best effort");
    bool nodeFound = false;
    for (const auto& n : monitor->nodes()) {
        if (n.name == "/terra_test_publisher") {
            nodeFound = true;
            check(std::find(n.publishes.begin(), n.publishes.end(), "/terra_test/imu") != n.publishes.end(), "节点的发布话题列表");
        }
    }
    check(nodeFound, "节点列表中有 /terra_test_publisher");

    for (const char* t : {"/terra_test/imu", "/terra_test/joints", "/terra_test/scan", "/terra_test/image", "/terra_test/depth", "/terra_test/compressed",
                          "/lowstate"}) {
        monitor->watch(t);
    }
    monitor->addField("/terra_test/imu", "angular_velocity.z");
    monitor->addField("/terra_test/joints", "position[2]");
    monitor->addField("/terra_test/joints", "position[9]");
    monitor->addField("/terra_test/imu", "angular_velocity.w");
    monitor->watchTf(true);
    std::this_thread::sleep_for(std::chrono::seconds(6));

    // 频率与带宽
    const auto s = monitor->stats("/terra_test/imu");
    std::printf("INFO /terra_test/imu：%.1f Hz，%.0f B/s，%llu 条\n", s.hz, s.bytesPerSecond, static_cast<unsigned long long>(s.messages));
    check(std::fabs(s.hz - 100.0) < 20.0 && s.bytesPerSecond > 1000.0 && s.messages > 200, "频率约 100 Hz，带宽与条数");
    check(monitor->watching("/terra_test/imu") && monitor->typeError("/terra_test/imu").empty(), "已订阅，类型可用");

    // 展开
    const auto imu = monitor->latestMessage("/terra_test/imu");
    check(imu && child(*imu, "header") && child(*imu, "orientation") && child(*imu, "angular_velocity"), "Imu 的字段树");
    if (imu) {
        const auto* acc = child(*imu, "linear_acceleration");
        const auto* z = acc ? child(*acc, "z") : nullptr;
        check(z && z->kind == r2::MessageValue::Kind::Number && std::fabs(z->number - 9.81) < 1e-12 && z->type == "float64", "linear_acceleration.z = 9.81");
        const auto* header = child(*imu, "header");
        const auto* frame = header ? child(*header, "frame_id") : nullptr;
        check(frame && frame->kind == r2::MessageValue::Kind::String && frame->text == "imu_link", "header.frame_id 为字符串");
        check(imu->type == "sensor_msgs/msg/Imu", "消息类型名");
    }
    const auto joints = monitor->latestMessage("/terra_test/joints");
    const auto* position = joints ? child(*joints, "position") : nullptr;
    check(position && position->kind == r2::MessageValue::Kind::Array && position->arraySize == 4 && position->children.size() == 4 &&
              std::fabs(position->children[2].number - 0.3) < 1e-12,
          "JointState.position：4 项，[2] = 0.3");
    const auto* names = joints ? child(*joints, "name") : nullptr;
    check(names && names->children.size() == 4 && names->children[3].text == "j3", "字符串数组");
    const auto scan = monitor->latestMessage("/terra_test/scan", 100);
    const auto* ranges = scan ? child(*scan, "ranges") : nullptr;
    check(ranges && ranges->arraySize == 360 && ranges->children.size() == 100 && std::fabs(ranges->children[99].number - 1.099) < 1e-6,
          "LaserScan.ranges：长度 360，只展开前 100 项");

    // 曲线字段
    const auto samples = monitor->samples("/terra_test/imu", "angular_velocity.z");
    double worst = 0.0;
    for (const auto& x : samples) worst = std::max(worst, std::fabs(x.value - std::sin(2.0 * 3.14159265358979323846 * 0.5 * x.stamp)));
    std::printf("INFO angular_velocity.z：%zu 个点，与 sin(πt) 的最大偏差 %.3g\n", samples.size(), worst);
    check(samples.size() > 200 && worst < 1e-6, "曲线：每条消息取值，与消息头时间的正弦一致");
    const auto j2 = monitor->samples("/terra_test/joints", "position[2]");
    check(!j2.empty() && std::fabs(j2.back().value - 0.3) < 1e-12 && std::isfinite(j2.back().stamp), "position[2] = 0.3，带消息头时间");
    check(monitor->fieldError("/terra_test/joints", "position[9]").find("超出") != std::string::npos, "下标越界：给出原因");
    check(monitor->fieldError("/terra_test/imu", "angular_velocity.w").find("没有字段") != std::string::npos, "字段不存在：给出原因");

    // 图像
    const auto rgb = monitor->latestImage("/terra_test/image");
    check(rgb && rgb->error.empty() && rgb->width == 4 && rgb->height == 2 && rgb->rgba.size() == 32 && rgb->rgba[(1 * 4 + 3) * 4 + 0] == 30 &&
              rgb->rgba[(1 * 4 + 3) * 4 + 1] == 20 && rgb->rgba[(1 * 4 + 3) * 4 + 2] == 30 && rgb->rgba[3] == 255,
          "rgb8 图像：像素 (3, 1) = (30, 20, 30)");
    check(rgb && !monitor->latestImage("/terra_test/image", monitor->messageCount("/terra_test/image") + 100).has_value(), "没有新帧时返回空");
    const auto depth = monitor->latestImage("/terra_test/depth");
    check(depth && depth->depth.size() == 4 && std::fabs(depth->depth[0] - 1.0f) < 1e-6f && std::fabs(depth->depth[3] - 0.5f) < 1e-6f &&
              std::isnan(depth->depth[2]) && depth->rgba.empty(),
          "16UC1：毫米换算为米，0 为无效");
    const auto png = monitor->latestImage("/terra_test/compressed");
    check(png && png->error.empty() && png->width == 2 && png->rgba.size() == 16 && png->rgba[0] == 255 && png->rgba[5] == 255 && png->rgba[10] == 255 &&
              png->encoding == "png",
          "PNG 压缩图像解码");

    // TF
    bool staticOk = false, dynamicOk = false;
    for (const auto& t : monitor->transforms()) {
        if (t.child == "base_link" && t.parent == "odom" && t.isStatic && t.translation[1] == 2.0) staticOk = true;
        if (t.child == "laser" && t.parent == "base_link" && !t.isStatic && std::fabs(t.translation[2] - 0.3) < 1e-12) dynamicOk = true;
    }
    check(staticOk && dynamicOk, "TF 树：/tf_static 的 odom → base_link 与 /tf 的 base_link → laser");

    // 第三方消息
    if (monitor->topic("/lowstate")) {
        const auto low = monitor->latestMessage("/lowstate");
        const auto* motors = low ? child(*low, "motor_state") : nullptr;
        const auto* last = motors && motors->children.size() == 35 ? child(motors->children[34], "q") : nullptr;
        check(motors && motors->arraySize == 35 && last && std::fabs(last->number - 0.34) < 1e-6, "unitree_hg/LowState：motor_state 35 项，[34].q = 0.34");
    } else {
        std::printf("INFO 没有宇树消息包，跳过 unitree_hg/LowState 的检查\n");
    }

    // 取消订阅
    monitor->removeField("/terra_test/imu", "angular_velocity.z");
    monitor->removeField("/terra_test/imu", "angular_velocity.w");
    monitor->unwatch("/terra_test/imu");
    check(!monitor->watching("/terra_test/imu") && monitor->samples("/terra_test/imu", "angular_velocity.z").empty(), "取消订阅后不再统计");

    stopPublisher();
    monitor.reset();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
