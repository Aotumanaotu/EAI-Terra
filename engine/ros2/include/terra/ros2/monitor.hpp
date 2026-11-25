#pragma once

// ROS2 监视器（M4-08d，设计稿第 2 节；RViz A 层）。公共头不含任何 ROS 类型，与桥接相同的约束。
//
// 与桥接（Bridge）相互独立（设计稿问题 1）：监视器是自己的节点 terra_monitor，不需要打开场景，连真机时同样可用。
// 线程：rclcpp 的执行器在监视器自己的线程中运行（收消息、每秒刷新一次节点与话题图）；以下函数都可在界面线程调用，
// 返回的是加锁复制的快照。
//
// 只订阅界面打开的话题（watch）：
//   - 收到的是序列化消息；按话题类型在运行时加载 rosidl_typesupport_introspection_cpp 展开为值树（latestMessage），
//     不需要事先编译该类型的代码（宇树等第三方消息只要在环境中就能看）；找不到类型库时 typeError 给出原因；
//   - 统计最近 5 秒的频率与带宽（与 ros2 topic hz / bw 相同的口径）；
//   - 加入曲线的字段（addField）在每条消息到达时取值，最多保留 kSeriesSeconds 秒；
//   - Image 与 CompressedImage 话题保留最新一帧（像素换为 RGBA8；深度保留原始数值）；
//   - /tf 与 /tf_static 建 TF 树（watchTf）。
// QoS：订阅时按发布者的 QoS 选择（有 best effort 的发布者就用 best effort；全部为 transient local 时用 transient local，
// 例如 /tf_static）。

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace terra::ros2 {

struct TopicInfo {
    std::string name;
    std::string type;  // 第一个类型，如 "sensor_msgs/msg/Imu"（同名话题有多个类型时其余见 otherTypes）
    std::vector<std::string> otherTypes;
    std::size_t publishers = 0;
    std::size_t subscribers = 0;  // 不含监视器自己
    std::string qos;              // 第一个发布者的 QoS，如 "best effort · volatile · keep last 5"
    bool bestEffort = false;
};

struct NodeInfo {
    std::string name;  // 全名（含命名空间）
    std::vector<std::string> publishes;
    std::vector<std::string> subscribes;
};

struct TopicStats {
    double hz = 0.0;              // 最近 5 秒
    double bytesPerSecond = 0.0;  // 最近 5 秒
    std::uint64_t messages = 0;   // 开始订阅以来
    double lastReceive = -1.0;    // 监视器时钟（秒，见 now()）；没有收到时为 -1
};

// 展开后的消息（值树）
struct MessageValue {
    enum class Kind { Number, Bool, String, Message, Array };
    std::string name;  // 字段名；数组元素为 "[i]"
    std::string type;  // 如 "float64"、"geometry_msgs/Vector3"、"float32[]"
    Kind kind = Kind::Message;
    double number = 0.0;
    bool boolean = false;
    std::string text;
    std::vector<MessageValue> children;  // Message 的字段；Array 的元素（最多 maxArrayItems 个）
    std::size_t arraySize = 0;           // Array 的实际长度
};

struct Sample {
    double receive = 0.0;  // 接收时间（监视器时钟，秒）
    double stamp = 0.0;    // 消息头时间（秒；没有 header 时为 NaN）
    double value = 0.0;
};

struct ImageFrame {
    std::uint64_t sequence = 0;  // 第几帧（从 1 开始）
    int width = 0;
    int height = 0;
    std::string encoding;          // 原编码，如 "rgb8"、"16UC1"、"jpeg"
    std::vector<std::uint8_t> rgba;  // 彩色与灰度图换成 RGBA8；深度图为空
    std::vector<float> depth;      // 深度图（16UC1 换算为米，32FC1 原样）；彩色图为空
    double receive = 0.0;
    double stamp = 0.0;
    std::string error;  // 不支持的编码、解码失败
};

struct TransformInfo {
    std::string parent;
    std::string child;
    double translation[3] = {0, 0, 0};
    double rotation[4] = {0, 0, 0, 1};  // x y z w
    bool isStatic = false;
    double stamp = 0.0;    // 消息头时间
    double receive = 0.0;  // 接收时间（监视器时钟）
};

// 压缩图像的解码由程序提供（stb 只在渲染模块中使用）：输入 JPEG / PNG 字节，输出 RGBA8；失败返回空
using ImageDecoder = std::vector<std::uint8_t> (*)(const std::uint8_t* data, std::size_t size, int& width, int& height);

class Monitor {
public:
    static constexpr double kSeriesSeconds = 60.0;

    // 创建失败（rclcpp 初始化失败等）时返回空，error 为原因
    static std::unique_ptr<Monitor> create(std::string& error, ImageDecoder decoder = nullptr);
    ~Monitor();
    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;

    double now() const;  // 监视器时钟：创建以来的秒数（墙钟）

    // 节点与话题图（每秒刷新）；不含监视器自己
    std::vector<TopicInfo> topics() const;
    std::vector<NodeInfo> nodes() const;
    std::optional<TopicInfo> topic(const std::string& name) const;

    // 订阅：界面打开话题时 watch，关闭时 unwatch（按次数计，次数为 0 时取消订阅）
    void watch(const std::string& topic);
    void unwatch(const std::string& topic);
    bool watching(const std::string& topic) const;
    TopicStats stats(const std::string& topic) const;
    std::string typeError(const std::string& topic) const;  // 空表示类型可用

    // 最新一条消息展开为值树；没有新消息时返回缓存；没有收到过时为空
    std::optional<MessageValue> latestMessage(const std::string& topic, std::size_t maxArrayItems = 1000);
    std::uint64_t messageCount(const std::string& topic) const;

    // 曲线：字段路径如 "angular_velocity.z"、"position[3]"、"motor_state[2].q"（数值或布尔字段）
    void addField(const std::string& topic, const std::string& field);
    void removeField(const std::string& topic, const std::string& field);
    std::vector<Sample> samples(const std::string& topic, const std::string& field, double since = -1.0) const;
    std::string fieldError(const std::string& topic, const std::string& field) const;  // 路径不存在等；空表示正常

    // 图像：只在 Image 与 CompressedImage 话题上有效
    // 换算在调用时进行（只换算界面要显示的最新一帧）；newerThan 之后没有新帧时返回空
    std::optional<ImageFrame> latestImage(const std::string& topic, std::uint64_t newerThan = 0);

    // TF 树（订阅 /tf 与 /tf_static）
    void watchTf(bool on);
    std::vector<TransformInfo> transforms() const;

private:
    struct Impl;
    explicit Monitor(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

// 字段路径的解析（纯字符串处理）："a.b[3].c" → {"a", "b", "[3]", "c"}；不合法时返回空
std::optional<std::vector<std::string>> parseFieldPath(const std::string& path);

}  // namespace terra::ros2
