// ONNX 文件的信息（M4-10a）：直接读 protobuf 的线格式，不需要推理库与 protobuf 库。
// 只读用到的字段（onnx.proto3）：
//   ModelProto：2 producer_name，7 graph，8 opset_import（OperatorSetIdProto：1 domain，2 version），14 metadata_props（1 key，2 value）
//   GraphProto：1 node（NodeProto：4 op_type），5 initializer（TensorProto：1 dims），11 input，12 output
//   ValueInfoProto：1 name，2 type（TypeProto：1 tensor_type（2 shape：TensorShapeProto 1 dim（1 dim_value，2 dim_param））））

#include "terra/policy/policy.hpp"

#include <cstdint>
#include <fstream>
#include <iterator>
#include <set>
#include <string_view>

namespace terra::policy {

namespace {

struct Field {
    std::uint32_t number = 0;
    int wire = 0;
    std::uint64_t varint = 0;
    std::string_view bytes;  // wire 2
};

// 逐个读取一条消息中的字段；格式不对时 ok 为假
class Message {
public:
    explicit Message(std::string_view data) : data_(data) {}
    bool ok() const noexcept { return ok_; }

    bool next(Field& f) {
        if (!ok_ || pos_ >= data_.size()) return false;
        std::uint64_t key = 0;
        if (!readVarint(key)) return fail();
        f.number = static_cast<std::uint32_t>(key >> 3);
        f.wire = static_cast<int>(key & 7);
        f.varint = 0;
        f.bytes = {};
        switch (f.wire) {
            case 0:
                if (!readVarint(f.varint)) return fail();
                return true;
            case 1:
                if (data_.size() - pos_ < 8) return fail();
                pos_ += 8;
                return true;
            case 5:
                if (data_.size() - pos_ < 4) return fail();
                pos_ += 4;
                return true;
            case 2: {
                std::uint64_t length = 0;
                if (!readVarint(length) || length > data_.size() - pos_) return fail();
                f.bytes = data_.substr(pos_, static_cast<std::size_t>(length));
                pos_ += static_cast<std::size_t>(length);
                return true;
            }
            default: return fail();
        }
    }

private:
    bool readVarint(std::uint64_t& out) {
        out = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            if (pos_ >= data_.size()) return false;
            const auto c = static_cast<std::uint8_t>(data_[pos_++]);
            out |= static_cast<std::uint64_t>(c & 0x7f) << shift;
            if (c < 0x80) return true;
        }
        return false;
    }
    bool fail() {
        ok_ = false;
        return false;
    }

    std::string_view data_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

// 打包或不打包的 int64 重复字段
void appendInts(const Field& f, std::vector<long long>& out) {
    if (f.wire == 0) {
        out.push_back(static_cast<long long>(f.varint));
        return;
    }
    if (f.wire != 2) return;
    std::string_view d = f.bytes;
    std::size_t pos = 0;
    while (pos < d.size()) {
        std::uint64_t v = 0;
        int shift = 0;
        for (; pos < d.size() && shift < 64; shift += 7) {
            const auto c = static_cast<std::uint8_t>(d[pos++]);
            v |= static_cast<std::uint64_t>(c & 0x7f) << shift;
            if (c < 0x80) break;
        }
        out.push_back(static_cast<long long>(v));
    }
}

TensorShape valueInfo(std::string_view data) {
    TensorShape t;
    Message vi(data);
    Field f;
    while (vi.next(f)) {
        if (f.number == 1 && f.wire == 2) t.name = std::string{f.bytes};
        if (f.number != 2 || f.wire != 2) continue;
        Message type(f.bytes);
        Field tf;
        while (type.next(tf)) {
            if (tf.number != 1 || tf.wire != 2) continue;  // tensor_type
            Message tensor(tf.bytes);
            Field sf;
            while (tensor.next(sf)) {
                if (sf.number != 2 || sf.wire != 2) continue;  // shape
                Message shape(sf.bytes);
                Field df;
                while (shape.next(df)) {
                    if (df.number != 1 || df.wire != 2) continue;  // dim
                    Message dim(df.bytes);
                    Field vf;
                    long long value = -1;
                    while (dim.next(vf)) {
                        if (vf.number == 1 && vf.wire == 0) value = static_cast<long long>(vf.varint);
                    }
                    t.dims.push_back(value);
                }
            }
        }
    }
    return t;
}

}  // namespace

std::optional<OnnxInfo> readOnnxInfo(const std::string& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "无法打开：" + path;
        return std::nullopt;
    }
    const std::string data{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    OnnxInfo info;
    Message model(data);
    Field f;
    std::string_view graph;
    bool sawGraph = false;
    while (model.next(f)) {
        if (f.number == 2 && f.wire == 2) info.producer = std::string{f.bytes};
        if (f.number == 7 && f.wire == 2) {
            graph = f.bytes;
            sawGraph = true;
        }
        if (f.number == 8 && f.wire == 2) {
            Message op(f.bytes);
            Field of;
            std::string domain;
            long long version = 0;
            while (op.next(of)) {
                if (of.number == 1 && of.wire == 2) domain = std::string{of.bytes};
                if (of.number == 2 && of.wire == 0) version = static_cast<long long>(of.varint);
            }
            if (domain.empty() || domain == "ai.onnx") info.opset = version;
        }
        if (f.number == 14 && f.wire == 2) {
            Message kv(f.bytes);
            Field kf;
            std::string key, value;
            while (kv.next(kf)) {
                if (kf.number == 1 && kf.wire == 2) key = std::string{kf.bytes};
                if (kf.number == 2 && kf.wire == 2) value = std::string{kf.bytes};
            }
            info.metadata[key] = value;
        }
    }
    if (!model.ok() || !sawGraph) {
        error = "不是有效的 ONNX 文件（protobuf 无法解析或没有计算图）：" + path;
        return std::nullopt;
    }
    Message g(graph);
    std::set<std::string> initializers;
    std::vector<TensorShape> inputs;
    while (g.next(f)) {
        if (f.wire != 2) continue;
        if (f.number == 1) {
            Message node(f.bytes);
            Field nf;
            while (node.next(nf)) {
                if (nf.number == 4 && nf.wire == 2) ++info.operators[std::string{nf.bytes}];
            }
        } else if (f.number == 5) {
            Message tensor(f.bytes);
            Field tf;
            std::vector<long long> dims;
            std::string name;
            while (tensor.next(tf)) {
                if (tf.number == 1) appendInts(tf, dims);
                if (tf.number == 8 && tf.wire == 2) name = std::string{tf.bytes};
            }
            long long count = 1;
            for (long long d : dims) count *= d;
            info.parameters += count;
            initializers.insert(name);
        } else if (f.number == 11) {
            inputs.push_back(valueInfo(f.bytes));
        } else if (f.number == 12) {
            info.outputs.push_back(valueInfo(f.bytes));
        }
    }
    if (!g.ok()) {
        error = "ONNX 的计算图无法解析：" + path;
        return std::nullopt;
    }
    // 旧版本（IR < 4）把初始化张量也列为输入
    for (auto& t : inputs) {
        if (!initializers.count(t.name)) info.inputs.push_back(std::move(t));
    }
    return info;
}

}  // namespace terra::policy
