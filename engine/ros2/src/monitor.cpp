// ROS2 监视器（M4-08d）：节点与话题图、通用订阅、运行时按 introspection 展开任意消息、曲线字段、图像、TF 树。

#include "terra/ros2/monitor.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <set>
#include <thread>
#include <unistd.h>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rmw/rmw.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "runtime.hpp"

namespace terra::ros2 {

namespace {

namespace ti = rosidl_typesupport_introspection_cpp;
using Clock = std::chrono::steady_clock;

constexpr double kStatsWindow = 5.0;  // 秒
const char* const kImageType = "sensor_msgs/msg/Image";
const char* const kCompressedType = "sensor_msgs/msg/CompressedImage";

// ---- 类型支持（运行时加载）----

struct TypeSupport {
    std::shared_ptr<rcpputils::SharedLibrary> introspectionLibrary;
    std::shared_ptr<rcpputils::SharedLibrary> cppLibrary;
    const rosidl_message_type_support_t* introspection = nullptr;
    const rosidl_message_type_support_t* cpp = nullptr;
    const ti::MessageMembers* members() const { return static_cast<const ti::MessageMembers*>(introspection->data); }
};

// 一条反序列化后的动态消息（按 introspection 给出的大小分配，构造与析构用类型自己的函数）
class DynamicMessage {
public:
    explicit DynamicMessage(const ti::MessageMembers* m)
        : members_(m), storage_((m->size_of_ + sizeof(std::max_align_t) - 1) / sizeof(std::max_align_t) + 1) {
        m->init_function(storage_.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
    }
    ~DynamicMessage() { members_->fini_function(storage_.data()); }
    DynamicMessage(const DynamicMessage&) = delete;
    DynamicMessage& operator=(const DynamicMessage&) = delete;
    void* data() { return storage_.data(); }
    const void* data() const { return storage_.data(); }

private:
    const ti::MessageMembers* members_;
    std::vector<std::max_align_t> storage_;
};

bool deserialize(const TypeSupport& ts, const rclcpp::SerializedMessage& message, DynamicMessage& out) {
    return rmw_deserialize(&message.get_rcl_serialized_message(), ts.cpp, out.data()) == RMW_RET_OK;
}

std::string typeNameOf(const ti::MessageMembers* m) {
    std::string ns = m->message_namespace_;
    for (std::size_t p = ns.find("::"); p != std::string::npos; p = ns.find("::")) ns.replace(p, 2, "/");
    return ns + "/" + m->message_name_;
}

const char* scalarName(std::uint8_t type) {
    switch (type) {
        case ti::ROS_TYPE_FLOAT: return "float32";
        case ti::ROS_TYPE_DOUBLE: return "float64";
        case ti::ROS_TYPE_LONG_DOUBLE: return "long double";
        case ti::ROS_TYPE_CHAR: return "char";
        case ti::ROS_TYPE_WCHAR: return "wchar";
        case ti::ROS_TYPE_BOOLEAN: return "bool";
        case ti::ROS_TYPE_OCTET: return "byte";
        case ti::ROS_TYPE_UINT8: return "uint8";
        case ti::ROS_TYPE_INT8: return "int8";
        case ti::ROS_TYPE_UINT16: return "uint16";
        case ti::ROS_TYPE_INT16: return "int16";
        case ti::ROS_TYPE_UINT32: return "uint32";
        case ti::ROS_TYPE_INT32: return "int32";
        case ti::ROS_TYPE_UINT64: return "uint64";
        case ti::ROS_TYPE_INT64: return "int64";
        case ti::ROS_TYPE_STRING: return "string";
        case ti::ROS_TYPE_WSTRING: return "wstring";
        default: return "?";
    }
}

template <typename T>
double as(const void* p) {
    T v;
    std::memcpy(&v, p, sizeof v);
    return static_cast<double>(v);
}

// 标量读为数值；不是数值类型时返回空
std::optional<double> readNumber(std::uint8_t type, const void* p) {
    switch (type) {
        case ti::ROS_TYPE_FLOAT: return as<float>(p);
        case ti::ROS_TYPE_DOUBLE: return as<double>(p);
        case ti::ROS_TYPE_LONG_DOUBLE: return as<long double>(p);
        case ti::ROS_TYPE_CHAR:
        case ti::ROS_TYPE_OCTET:
        case ti::ROS_TYPE_UINT8: return as<std::uint8_t>(p);
        case ti::ROS_TYPE_WCHAR: return as<char16_t>(p);
        case ti::ROS_TYPE_BOOLEAN: return as<bool>(p) ? 1.0 : 0.0;
        case ti::ROS_TYPE_INT8: return as<std::int8_t>(p);
        case ti::ROS_TYPE_UINT16: return as<std::uint16_t>(p);
        case ti::ROS_TYPE_INT16: return as<std::int16_t>(p);
        case ti::ROS_TYPE_UINT32: return as<std::uint32_t>(p);
        case ti::ROS_TYPE_INT32: return as<std::int32_t>(p);
        case ti::ROS_TYPE_UINT64: return as<std::uint64_t>(p);
        case ti::ROS_TYPE_INT64: return as<std::int64_t>(p);
        default: return std::nullopt;
    }
}

bool isBoolSequence(const ti::MessageMember& f) { return f.type_id_ == ti::ROS_TYPE_BOOLEAN && f.is_array_ && f.array_size_ == 0 && !f.is_upper_bound_; }

std::size_t arraySize(const ti::MessageMember& f, const void* fieldPtr) {
    if (isBoolSequence(f)) return static_cast<const std::vector<bool>*>(fieldPtr)->size();
    if (f.size_function != nullptr) return f.size_function(fieldPtr);
    return f.array_size_;
}

// 数组元素的地址；std::vector<bool> 的元素没有地址，由 boolValue 给出
const void* elementPtr(const ti::MessageMember& f, const void* fieldPtr, std::size_t i) {
    if (isBoolSequence(f) || f.get_const_function == nullptr) return nullptr;
    return f.get_const_function(fieldPtr, i);
}

MessageValue scalarValue(std::uint8_t type, const void* p, const std::string& name) {
    MessageValue v;
    v.name = name;
    v.type = scalarName(type);
    if (type == ti::ROS_TYPE_STRING) {
        v.kind = MessageValue::Kind::String;
        v.text = *static_cast<const std::string*>(p);
    } else if (type == ti::ROS_TYPE_WSTRING) {
        v.kind = MessageValue::Kind::String;
        for (const char16_t c : *static_cast<const std::u16string*>(p)) v.text += c < 0x80 ? static_cast<char>(c) : '?';
    } else if (type == ti::ROS_TYPE_BOOLEAN) {
        v.kind = MessageValue::Kind::Bool;
        v.boolean = as<bool>(p) != 0.0;
        v.number = v.boolean ? 1.0 : 0.0;
    } else {
        v.kind = MessageValue::Kind::Number;
        v.number = readNumber(type, p).value_or(std::numeric_limits<double>::quiet_NaN());
    }
    return v;
}

MessageValue messageValue(const ti::MessageMembers* m, const void* msg, const std::string& name, std::size_t maxItems);

MessageValue fieldValue(const ti::MessageMember& f, const void* fieldPtr, std::size_t maxItems) {
    const auto* sub = f.type_id_ == ti::ROS_TYPE_MESSAGE ? static_cast<const ti::MessageMembers*>(f.members_->data) : nullptr;
    if (!f.is_array_) return sub ? messageValue(sub, fieldPtr, f.name_, maxItems) : scalarValue(f.type_id_, fieldPtr, f.name_);
    MessageValue a;
    a.name = f.name_;
    a.kind = MessageValue::Kind::Array;
    a.type = (sub ? typeNameOf(sub) : std::string{scalarName(f.type_id_)}) +
             (f.array_size_ > 0 && !f.is_upper_bound_ ? "[" + std::to_string(f.array_size_) + "]" : "[]");
    a.arraySize = arraySize(f, fieldPtr);
    const std::size_t shown = std::min(a.arraySize, maxItems);
    a.children.reserve(shown);
    for (std::size_t i = 0; i < shown; ++i) {
        const std::string itemName = "[" + std::to_string(i) + "]";
        if (isBoolSequence(f)) {
            MessageValue b;
            b.name = itemName;
            b.type = "bool";
            b.kind = MessageValue::Kind::Bool;
            b.boolean = (*static_cast<const std::vector<bool>*>(fieldPtr))[i];
            b.number = b.boolean ? 1.0 : 0.0;
            a.children.push_back(std::move(b));
            continue;
        }
        const void* e = elementPtr(f, fieldPtr, i);
        if (e == nullptr) break;
        a.children.push_back(sub ? messageValue(sub, e, itemName, maxItems) : scalarValue(f.type_id_, e, itemName));
    }
    return a;
}

MessageValue messageValue(const ti::MessageMembers* m, const void* msg, const std::string& name, std::size_t maxItems) {
    MessageValue v;
    v.name = name;
    v.type = typeNameOf(m);
    v.kind = MessageValue::Kind::Message;
    const auto* base = static_cast<const std::uint8_t*>(msg);
    for (std::uint32_t i = 0; i < m->member_count_; ++i) {
        const ti::MessageMember& f = m->members_[i];
        v.children.push_back(fieldValue(f, base + f.offset_, maxItems));
    }
    return v;
}

// 按路径取数值（数值或布尔字段）；失败时 error 为原因
std::optional<double> extract(const ti::MessageMembers* root, const void* msg, const std::vector<std::string>& path, std::string& error) {
    const ti::MessageMembers* m = root;
    const void* base = msg;
    const ti::MessageMember* field = nullptr;  // 当前停在的字段（数组或标量）
    const void* fieldPtr = nullptr;
    bool element = false;  // 当前是数组中的一个标量元素
    std::optional<double> elementValue;
    for (std::size_t k = 0; k < path.size(); ++k) {
        const std::string& seg = path[k];
        if (seg.front() == '[') {
            if (field == nullptr || !field->is_array_ || element) {
                error = "“" + seg + "”之前不是数组";
                return std::nullopt;
            }
            const std::size_t i = std::stoul(seg.substr(1, seg.size() - 2));
            const std::size_t n = arraySize(*field, fieldPtr);
            if (i >= n) {
                error = "下标 " + std::to_string(i) + " 超出数组长度 " + std::to_string(n);
                return std::nullopt;
            }
            if (field->type_id_ == ti::ROS_TYPE_MESSAGE) {
                m = static_cast<const ti::MessageMembers*>(field->members_->data);
                base = elementPtr(*field, fieldPtr, i);
                field = nullptr;
                if (base == nullptr) {
                    error = "无法取数组元素";
                    return std::nullopt;
                }
            } else {
                element = true;
                if (isBoolSequence(*field)) {
                    elementValue = (*static_cast<const std::vector<bool>*>(fieldPtr))[i] ? 1.0 : 0.0;
                } else if (const void* e = elementPtr(*field, fieldPtr, i)) {
                    elementValue = readNumber(field->type_id_, e);
                }
                if (!elementValue) {
                    error = "数组元素不是数值";
                    return std::nullopt;
                }
            }
            continue;
        }
        if (element || (field != nullptr && field->type_id_ != ti::ROS_TYPE_MESSAGE) || (field != nullptr && field->is_array_)) {
            error = "“" + seg + "”之前不是消息（数组请先写下标）";
            return std::nullopt;
        }
        if (field != nullptr) {  // 进入子消息
            m = static_cast<const ti::MessageMembers*>(field->members_->data);
            base = fieldPtr;
        }
        const ti::MessageMember* found = nullptr;
        for (std::uint32_t i = 0; i < m->member_count_; ++i) {
            if (seg == m->members_[i].name_) found = &m->members_[i];
        }
        if (found == nullptr) {
            error = typeNameOf(m) + " 中没有字段“" + seg + "”";
            return std::nullopt;
        }
        field = found;
        fieldPtr = static_cast<const std::uint8_t*>(base) + found->offset_;
    }
    if (element) return elementValue;
    if (field == nullptr || field->is_array_ || field->type_id_ == ti::ROS_TYPE_MESSAGE) {
        error = "路径没有停在数值字段上";
        return std::nullopt;
    }
    const auto v = readNumber(field->type_id_, fieldPtr);
    if (!v) error = "字段不是数值（" + std::string{scalarName(field->type_id_)} + "）";
    return v;
}

double headerStamp(const ti::MessageMembers* m, const void* msg) {
    std::string ignored;
    const auto sec = extract(m, msg, {"header", "stamp", "sec"}, ignored);
    const auto nsec = extract(m, msg, {"header", "stamp", "nanosec"}, ignored);
    return sec && nsec ? *sec + *nsec * 1e-9 : std::numeric_limits<double>::quiet_NaN();
}

double stampOf(const builtin_interfaces::msg::Time& t) { return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9; }

std::string qosText(const rclcpp::QoS& q) {
    std::string s = q.reliability() == rclcpp::ReliabilityPolicy::BestEffort ? "best effort" : "reliable";
    s += q.durability() == rclcpp::DurabilityPolicy::TransientLocal ? " · transient local" : " · volatile";
    if (q.history() == rclcpp::HistoryPolicy::KeepAll) s += " · keep all";
    else s += " · keep last " + std::to_string(q.depth());
    return s;
}

// ---- 图像换算 ----

void convertImage(const sensor_msgs::msg::Image& msg, ImageFrame& f) {
    f.width = static_cast<int>(msg.width);
    f.height = static_cast<int>(msg.height);
    f.encoding = msg.encoding;
    f.stamp = stampOf(msg.header.stamp);
    const std::size_t w = msg.width, h = msg.height, step = msg.step;
    const std::string& e = msg.encoding;
    const auto need = [&](std::size_t bytesPerPixel) {
        if (step < w * bytesPerPixel || msg.data.size() < step * h) {
            f.error = "图像数据长度与宽、高、step 不符";
            return false;
        }
        return true;
    };
    if (e == "rgb8" || e == "bgr8" || e == "rgba8" || e == "bgra8" || e == "mono8" || e == "8UC1" || e == "mono16") {
        const std::size_t bpp = e == "rgb8" || e == "bgr8" ? 3 : e == "rgba8" || e == "bgra8" ? 4 : e == "mono16" ? 2 : 1;
        if (!need(bpp)) return;
        f.rgba.resize(w * h * 4);
        for (std::size_t y = 0; y < h; ++y) {
            const std::uint8_t* row = msg.data.data() + y * step;
            for (std::size_t x = 0; x < w; ++x) {
                const std::uint8_t* p = row + x * bpp;
                std::uint8_t* o = f.rgba.data() + (y * w + x) * 4;
                if (bpp >= 3) {
                    const bool bgr = e[0] == 'b';
                    o[0] = bgr ? p[2] : p[0];
                    o[1] = p[1];
                    o[2] = bgr ? p[0] : p[2];
                    o[3] = bpp == 4 ? p[3] : 255;
                } else {
                    std::uint8_t g = p[0];
                    if (bpp == 2) {
                        std::uint16_t v;
                        std::memcpy(&v, p, 2);
                        if (msg.is_bigendian) v = static_cast<std::uint16_t>((v >> 8) | (v << 8));
                        g = static_cast<std::uint8_t>(v >> 8);
                    }
                    o[0] = o[1] = o[2] = g;
                    o[3] = 255;
                }
            }
        }
    } else if (e == "16UC1" || e == "32FC1") {
        const std::size_t bpp = e == "16UC1" ? 2 : 4;
        if (!need(bpp)) return;
        f.depth.resize(w * h);
        for (std::size_t y = 0; y < h; ++y) {
            const std::uint8_t* row = msg.data.data() + y * step;
            for (std::size_t x = 0; x < w; ++x) {
                float d = 0.0f;
                if (bpp == 2) {
                    std::uint16_t v;
                    std::memcpy(&v, row + x * 2, 2);
                    if (msg.is_bigendian) v = static_cast<std::uint16_t>((v >> 8) | (v << 8));
                    d = v == 0 ? std::numeric_limits<float>::quiet_NaN() : static_cast<float>(v) * 0.001f;  // 毫米 → 米；0 为无效
                } else {
                    std::memcpy(&d, row + x * 4, 4);
                }
                f.depth[y * w + x] = d;
            }
        }
    } else {
        f.error = "不支持的图像编码“" + e + "”（支持 rgb8、bgr8、rgba8、bgra8、mono8、mono16、16UC1、32FC1）";
    }
}

}  // namespace

std::optional<std::vector<std::string>> parseFieldPath(const std::string& path) {
    std::vector<std::string> out;
    std::size_t i = 0;
    while (i < path.size()) {
        if (path[i] == '.') {
            if (out.empty() || i + 1 >= path.size() || path[i + 1] == '.' || path[i + 1] == '[') return std::nullopt;
            ++i;
            continue;
        }
        if (path[i] == '[') {
            const std::size_t close = path.find(']', i);
            if (close == std::string::npos || close == i + 1 || out.empty()) return std::nullopt;
            for (std::size_t k = i + 1; k < close; ++k) {
                if (path[k] < '0' || path[k] > '9') return std::nullopt;
            }
            if (close - i - 1 > 9) return std::nullopt;
            out.push_back(path.substr(i, close - i + 1));
            i = close + 1;
            if (i < path.size() && path[i] != '.' && path[i] != '[') return std::nullopt;
            continue;
        }
        std::size_t j = i;
        while (j < path.size() && path[j] != '.' && path[j] != '[') {
            const char c = path[j];
            if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_')) return std::nullopt;
            ++j;
        }
        if (!out.empty() && out.back().front() != '[' && i > 0 && path[i - 1] != '.') return std::nullopt;
        out.push_back(path.substr(i, j - i));
        i = j;
    }
    if (out.empty()) return std::nullopt;
    return out;
}

struct Monitor::Impl {
    struct Field {
        std::vector<std::string> path;
        std::deque<Sample> samples;
        std::string error;
        int refs = 0;
    };
    struct Watched {
        int refs = 0;
        std::string type;
        rclcpp::GenericSubscription::SharedPtr subscription;
        std::shared_ptr<TypeSupport> typeSupport;
        std::string typeError;
        std::deque<std::pair<double, std::size_t>> window;  // 接收时间、字节数
        std::uint64_t count = 0;
        double last = -1.0;
        std::shared_ptr<rclcpp::SerializedMessage> latest;
        double latestReceive = 0.0;
        // 展开的缓存
        std::uint64_t decodedCount = 0;
        std::size_t decodedItems = 0;
        std::optional<MessageValue> decoded;
        std::map<std::string, Field> fields;
        // 图像的缓存
        std::uint64_t imageCount = 0;
        std::optional<ImageFrame> image;
    };

    Clock::time_point start = Clock::now();
    ImageDecoder decoder = nullptr;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread spinner;
    std::atomic<bool> stopping{false};
    rclcpp::TimerBase::SharedPtr graphTimer;

    mutable std::mutex mutex;  // 保护以下全部
    std::vector<TopicInfo> topicList;
    std::vector<NodeInfo> nodeList;
    std::map<std::string, Watched> watched;
    std::map<std::string, std::shared_ptr<TypeSupport>> types;
    std::map<std::string, std::string> typeErrors;  // 类型 → 加载失败的原因
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf, tfStatic;
    std::map<std::string, TransformInfo> tree;  // 子坐标系 → 变换

    double now() const { return std::chrono::duration<double>(Clock::now() - start).count(); }

    std::shared_ptr<TypeSupport> loadType(const std::string& type, std::string& error) {
        if (const auto it = types.find(type); it != types.end()) return it->second;
        if (const auto it = typeErrors.find(type); it != typeErrors.end()) {
            error = it->second;
            return nullptr;
        }
        auto ts = std::make_shared<TypeSupport>();
        try {
            ts->introspectionLibrary = rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
            ts->introspection = rclcpp::get_typesupport_handle(type, "rosidl_typesupport_introspection_cpp", *ts->introspectionLibrary);
            ts->cppLibrary = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
            ts->cpp = rclcpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", *ts->cppLibrary);
        } catch (const std::exception& e) {
            error = "无法解析的类型 " + type + "：找不到它的类型库（" + e.what() + "）。请先 source 提供该消息包的工作空间";
            typeErrors[type] = error;
            return nullptr;
        }
        types[type] = ts;
        return ts;
    }

    // 按发布者的 QoS 选择订阅的 QoS（调用时持有锁）
    rclcpp::QoS subscriptionQos(const std::string& topic) {
        rclcpp::QoS q{rclcpp::KeepLast(10)};
        const auto infos = node->get_publishers_info_by_topic(topic);
        bool allTransient = !infos.empty();
        for (const auto& i : infos) allTransient = allTransient && i.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
        if (allTransient) q.reliable().transient_local();
        else q.best_effort().durability_volatile();  // best effort 的订阅与两种发布者都能匹配
        return q;
    }

    // 为已 watch 但还没有订阅的话题创建订阅（调用时持有锁）；话题还没有发布者时等下一次刷新
    void subscribe(const std::string& topic, Watched& w) {
        if (w.subscription) return;
        const auto all = node->get_topic_names_and_types();
        const auto it = all.find(topic);
        if (it == all.end() || it->second.empty()) return;
        w.type = it->second.front();
        std::string error;
        w.typeSupport = loadType(w.type, error);
        w.typeError = error;
        Impl* self = this;
        const std::string name = topic;
        try {
            w.subscription = node->create_generic_subscription(
                topic, w.type, subscriptionQos(topic), [self, name](std::shared_ptr<rclcpp::SerializedMessage> message) { self->receive(name, std::move(message)); });
        } catch (const std::exception& e) {
            w.typeError = std::string{"订阅失败："} + e.what();
        }
    }

    void receive(const std::string& topic, std::shared_ptr<rclcpp::SerializedMessage> message) {
        const double t = now();
        std::shared_ptr<TypeSupport> ts;
        std::vector<std::pair<std::string, std::vector<std::string>>> fieldPaths;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto it = watched.find(topic);
            if (it == watched.end()) return;
            Watched& w = it->second;
            ++w.count;
            w.last = t;
            w.window.emplace_back(t, message->size());
            while (!w.window.empty() && w.window.front().first < t - kStatsWindow) w.window.pop_front();
            w.latest = message;
            w.latestReceive = t;
            ts = w.typeSupport;
            for (const auto& [name, f] : w.fields) {
                if (f.refs > 0) fieldPaths.emplace_back(name, f.path);
            }
        }
        if (fieldPaths.empty() || !ts) return;
        // 曲线字段：每条消息都取值（锁外反序列化）
        DynamicMessage msg(ts->members());
        if (!deserialize(*ts, *message, msg)) return;
        const double stamp = headerStamp(ts->members(), msg.data());
        std::vector<std::pair<std::string, std::optional<double>>> values;
        std::vector<std::string> errors;
        for (const auto& [name, path] : fieldPaths) {
            std::string error;
            values.emplace_back(name, extract(ts->members(), msg.data(), path, error));
            errors.push_back(error);
        }
        std::lock_guard<std::mutex> lock(mutex);
        const auto it = watched.find(topic);
        if (it == watched.end()) return;
        for (std::size_t k = 0; k < values.size(); ++k) {
            const auto f = it->second.fields.find(values[k].first);
            if (f == it->second.fields.end()) continue;
            f->second.error = errors[k];
            if (values[k].second) f->second.samples.push_back({t, stamp, *values[k].second});
            while (!f->second.samples.empty() && f->second.samples.front().receive < t - Monitor::kSeriesSeconds) f->second.samples.pop_front();
        }
    }

    void refreshGraph() {
        std::vector<TopicInfo> topicsNow;
        std::vector<NodeInfo> nodesNow;
        const std::string self = node->get_fully_qualified_name();
        try {
            const auto all = node->get_topic_names_and_types();
            for (const auto& [name, typeList] : all) {
                TopicInfo info;
                info.name = name;
                if (!typeList.empty()) info.type = typeList.front();
                for (std::size_t k = 1; k < typeList.size(); ++k) info.otherTypes.push_back(typeList[k]);
                const auto pubs = node->get_publishers_info_by_topic(name);
                const auto subs = node->get_subscriptions_info_by_topic(name);
                for (const auto& p : pubs) {
                    if (p.node_namespace() + (p.node_namespace() == "/" ? "" : "/") + p.node_name() == self) continue;
                    ++info.publishers;
                    if (info.qos.empty()) info.qos = qosText(p.qos_profile());
                    info.bestEffort = info.bestEffort || p.qos_profile().reliability() == rclcpp::ReliabilityPolicy::BestEffort;
                }
                for (const auto& s : subs) {
                    if (s.node_namespace() + (s.node_namespace() == "/" ? "" : "/") + s.node_name() == self) continue;
                    ++info.subscribers;
                }
                if (info.publishers == 0 && info.subscribers == 0) continue;  // 只有监视器自己
                topicsNow.push_back(std::move(info));
            }
            auto graph = node->get_node_graph_interface();
            for (const auto& [nodeName, ns] : graph->get_node_names_and_namespaces()) {
                NodeInfo n;
                n.name = ns + (ns == "/" ? "" : "/") + nodeName;
                if (n.name == self) continue;
                try {
                    for (const auto& [topicName, typeList] : graph->get_publisher_names_and_types_by_node(nodeName, ns)) {
                        (void)typeList;
                        n.publishes.push_back(topicName);
                    }
                    for (const auto& [topicName, typeList] : graph->get_subscriber_names_and_types_by_node(nodeName, ns)) {
                        (void)typeList;
                        n.subscribes.push_back(topicName);
                    }
                } catch (const std::exception&) {
                    // 节点刚退出
                }
                nodesNow.push_back(std::move(n));
            }
        } catch (const std::exception&) {
            return;
        }
        std::sort(nodesNow.begin(), nodesNow.end(), [](const NodeInfo& a, const NodeInfo& b) { return a.name < b.name; });
        std::lock_guard<std::mutex> lock(mutex);
        topicList = std::move(topicsNow);
        nodeList = std::move(nodesNow);
        for (auto& [name, w] : watched) {
            if (w.refs > 0 && !w.subscription) subscribe(name, w);
        }
    }

    void receiveTf(const tf2_msgs::msg::TFMessage& msg, bool isStatic) {
        const double t = now();
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& x : msg.transforms) {
            TransformInfo info;
            info.parent = x.header.frame_id;
            info.child = x.child_frame_id;
            info.translation[0] = x.transform.translation.x;
            info.translation[1] = x.transform.translation.y;
            info.translation[2] = x.transform.translation.z;
            info.rotation[0] = x.transform.rotation.x;
            info.rotation[1] = x.transform.rotation.y;
            info.rotation[2] = x.transform.rotation.z;
            info.rotation[3] = x.transform.rotation.w;
            info.isStatic = isStatic;
            info.stamp = stampOf(x.header.stamp);
            info.receive = t;
            tree[info.child] = info;
        }
    }
};

Monitor::Monitor(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Monitor::~Monitor() {
    impl_->stopping = true;
    if (impl_->spinner.joinable()) impl_->spinner.join();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->watched.clear();
        impl_->tf.reset();
        impl_->tfStatic.reset();
    }
    impl_->graphTimer.reset();
    impl_->executor.reset();
    impl_->node.reset();
    releaseRclcpp();
}

std::unique_ptr<Monitor> Monitor::create(std::string& error, ImageDecoder decoder) {
    auto impl = std::make_unique<Impl>();
    impl->decoder = decoder;
    acquireRclcpp();
    try {
        impl->node = std::make_shared<rclcpp::Node>("terra_monitor_" + std::to_string(::getpid()));
        Impl* self = impl.get();
        impl->graphTimer = impl->node->create_wall_timer(std::chrono::seconds(1), [self] { self->refreshGraph(); });
        impl->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        impl->executor->add_node(impl->node);
        rclcpp::executors::SingleThreadedExecutor* executor = impl->executor.get();
        std::atomic<bool>* stopping = &impl->stopping;
        impl->spinner = std::thread([executor, stopping, self] {
            self->refreshGraph();  // 启动时先刷新一次
            while (!*stopping && rclcpp::ok()) executor->spin_once(std::chrono::milliseconds(20));
        });
    } catch (const std::exception& e) {
        error = std::string{"ROS2 监视器创建失败："} + e.what();
        impl->stopping = true;
        if (impl->spinner.joinable()) impl->spinner.join();
        impl->graphTimer.reset();
        impl->executor.reset();
        impl->node.reset();
        releaseRclcpp();
        return nullptr;
    }
    return std::unique_ptr<Monitor>(new Monitor(std::move(impl)));
}

double Monitor::now() const { return impl_->now(); }

std::vector<TopicInfo> Monitor::topics() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->topicList;
}

std::vector<NodeInfo> Monitor::nodes() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->nodeList;
}

std::optional<TopicInfo> Monitor::topic(const std::string& name) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (const auto& t : impl_->topicList) {
        if (t.name == name) return t;
    }
    return std::nullopt;
}

void Monitor::watch(const std::string& topic) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto& w = impl_->watched[topic];
    ++w.refs;
    impl_->subscribe(topic, w);
}

void Monitor::unwatch(const std::string& topic) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->watched.find(topic);
    if (it == impl_->watched.end()) return;
    if (--it->second.refs <= 0) impl_->watched.erase(it);  // 订阅随之销毁
}

bool Monitor::watching(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->watched.find(topic);
    return it != impl_->watched.end() && it->second.subscription != nullptr;
}

TopicStats Monitor::stats(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    TopicStats s;
    const auto it = impl_->watched.find(topic);
    if (it == impl_->watched.end()) return s;
    const auto& w = it->second;
    s.messages = w.count;
    s.lastReceive = w.last;
    // 窗口内：频率 = (条数 − 1) / 首尾间隔；带宽 = 字节数 / 窗口长度（不足 5 秒按实际间隔）
    const double t = impl_->now();
    std::size_t n = 0, bytes = 0;
    double first = 0.0, lastT = 0.0;
    for (const auto& [at, size] : w.window) {
        if (at < t - kStatsWindow) continue;
        if (n == 0) first = at;
        lastT = at;
        ++n;
        bytes += size;
    }
    if (n >= 2 && lastT > first) {
        s.hz = static_cast<double>(n - 1) / (lastT - first);
        s.bytesPerSecond = static_cast<double>(bytes) / std::max(lastT - first, 1e-6) * static_cast<double>(n - 1) / static_cast<double>(n);
    }
    if (n > 0 && t - lastT > 2.0 * kStatsWindow) s.hz = 0.0;
    return s;
}

std::string Monitor::typeError(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->watched.find(topic);
    return it == impl_->watched.end() ? std::string{} : it->second.typeError;
}

std::uint64_t Monitor::messageCount(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->watched.find(topic);
    return it == impl_->watched.end() ? 0 : it->second.count;
}

std::optional<MessageValue> Monitor::latestMessage(const std::string& topic, std::size_t maxArrayItems) {
    std::shared_ptr<rclcpp::SerializedMessage> latest;
    std::shared_ptr<TypeSupport> ts;
    std::uint64_t count = 0;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const auto it = impl_->watched.find(topic);
        if (it == impl_->watched.end()) return std::nullopt;
        auto& w = it->second;
        if (w.decoded && w.decodedCount == w.count && w.decodedItems == maxArrayItems) return w.decoded;
        latest = w.latest;
        ts = w.typeSupport;
        count = w.count;
    }
    if (!latest || !ts) return std::nullopt;
    DynamicMessage msg(ts->members());
    if (!deserialize(*ts, *latest, msg)) return std::nullopt;
    MessageValue v = messageValue(ts->members(), msg.data(), topic, maxArrayItems);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->watched.find(topic);
    if (it != impl_->watched.end()) {
        it->second.decoded = v;
        it->second.decodedCount = count;
        it->second.decodedItems = maxArrayItems;
    }
    return v;
}

void Monitor::addField(const std::string& topic, const std::string& field) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto& w = impl_->watched[topic];
    ++w.refs;  // 曲线也算一次 watch
    impl_->subscribe(topic, w);
    auto& f = w.fields[field];
    ++f.refs;
    if (const auto path = parseFieldPath(field)) f.path = *path;
    else f.error = "字段路径不合法：“" + field + "”";
}

void Monitor::removeField(const std::string& topic, const std::string& field) {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const auto it = impl_->watched.find(topic);
        if (it == impl_->watched.end()) return;
        const auto f = it->second.fields.find(field);
        if (f != it->second.fields.end() && --f->second.refs <= 0) it->second.fields.erase(f);
    }
    unwatch(topic);
}

std::vector<Sample> Monitor::samples(const std::string& topic, const std::string& field, double since) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<Sample> out;
    const auto it = impl_->watched.find(topic);
    if (it == impl_->watched.end()) return out;
    const auto f = it->second.fields.find(field);
    if (f == it->second.fields.end()) return out;
    for (const auto& s : f->second.samples) {
        if (s.receive > since) out.push_back(s);
    }
    return out;
}

std::string Monitor::fieldError(const std::string& topic, const std::string& field) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->watched.find(topic);
    if (it == impl_->watched.end()) return {};
    if (!it->second.typeError.empty()) return it->second.typeError;
    const auto f = it->second.fields.find(field);
    return f == it->second.fields.end() ? std::string{} : f->second.error;
}

std::optional<ImageFrame> Monitor::latestImage(const std::string& topic, std::uint64_t newerThan) {
    std::shared_ptr<rclcpp::SerializedMessage> latest;
    std::string type;
    std::uint64_t count = 0;
    double receive = 0.0;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const auto it = impl_->watched.find(topic);
        if (it == impl_->watched.end()) return std::nullopt;
        auto& w = it->second;
        if (w.count <= newerThan) return std::nullopt;
        if (w.image && w.imageCount == w.count) return w.image;
        latest = w.latest;
        type = w.type;
        count = w.count;
        receive = w.latestReceive;
    }
    if (!latest) return std::nullopt;
    ImageFrame frame;
    frame.sequence = count;
    frame.receive = receive;
    try {
        if (type == kImageType) {
            sensor_msgs::msg::Image msg;
            rclcpp::Serialization<sensor_msgs::msg::Image>().deserialize_message(latest.get(), &msg);
            convertImage(msg, frame);
        } else if (type == kCompressedType) {
            sensor_msgs::msg::CompressedImage msg;
            rclcpp::Serialization<sensor_msgs::msg::CompressedImage>().deserialize_message(latest.get(), &msg);
            frame.encoding = msg.format;
            frame.stamp = stampOf(msg.header.stamp);
            if (impl_->decoder == nullptr) {
                frame.error = "没有可用的图像解码器";
            } else {
                frame.rgba = impl_->decoder(msg.data.data(), msg.data.size(), frame.width, frame.height);
                if (frame.rgba.empty()) frame.error = "解码失败（支持 JPEG 与 PNG；格式为“" + msg.format + "”）";
            }
        } else {
            frame.error = "不是图像话题（" + type + "）";
        }
    } catch (const std::exception& e) {
        frame.error = std::string{"图像反序列化失败："} + e.what();
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->watched.find(topic);
    if (it != impl_->watched.end()) {
        it->second.image = frame;
        it->second.imageCount = count;
    }
    return frame;
}

void Monitor::watchTf(bool on) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!on) {
        impl_->tf.reset();
        impl_->tfStatic.reset();
        return;
    }
    if (impl_->tf) return;
    Impl* self = impl_.get();
    impl_->tf = impl_->node->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(), [self](const tf2_msgs::msg::TFMessage& m) { self->receiveTf(m, false); });
    impl_->tfStatic = impl_->node->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(),
        [self](const tf2_msgs::msg::TFMessage& m) { self->receiveTf(m, true); });
}

std::vector<TransformInfo> Monitor::transforms() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<TransformInfo> out;
    for (const auto& entry : impl_->tree) out.push_back(entry.second);
    return out;
}

}  // namespace terra::ros2
