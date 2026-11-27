// 不依赖图形的 glTF 2.0 读取（M5-02b），说明见 gltf_reader.hpp。

#include "gltf_reader.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>

#include <nlohmann/json.hpp>

namespace terra::importer::gltf {

namespace fs = std::filesystem;
using Json = nlohmann::json;

Mat4 identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

Mat4 multiply(const Mat4& a, const Mat4& b) {
    Mat4 out{};
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[static_cast<std::size_t>(k * 4 + r)] * b[static_cast<std::size_t>(c * 4 + k)];
            out[static_cast<std::size_t>(c * 4 + r)] = s;
        }
    }
    return out;
}

Vec3d transformPoint(const Mat4& m, const Vec3d& p) {
    return {m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12], m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13], m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
}

Mat4 inverseAffine(const Mat4& m) {
    // 3×3 部分求逆（伴随矩阵），平移取 −A⁻¹ t
    const double a = m[0], b = m[4], c = m[8], d = m[1], e = m[5], f = m[9], g = m[2], h = m[6], i = m[10];
    const double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const double det = a * A + b * B + c * C;
    Mat4 out = identity();
    if (std::fabs(det) < 1e-300) return out;
    const double inv = 1.0 / det;
    const double r[9] = {A * inv, -(b * i - c * h) * inv, (b * f - c * e) * inv,
                         B * inv, (a * i - c * g) * inv,  -(a * f - c * d) * inv,
                         C * inv, -(a * h - b * g) * inv, (a * e - b * d) * inv};
    // r 按行存放：r[row * 3 + col]
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) out[static_cast<std::size_t>(col * 4 + row)] = r[row * 3 + col];
    }
    const double tx = m[12], ty = m[13], tz = m[14];
    out[12] = -(r[0] * tx + r[1] * ty + r[2] * tz);
    out[13] = -(r[3] * tx + r[4] * ty + r[5] * tz);
    out[14] = -(r[6] * tx + r[7] * ty + r[8] * tz);
    return out;
}

namespace {

std::string percentDecoded(const std::string& uri) {
    std::string out;
    for (std::size_t i = 0; i < uri.size(); ++i) {
        if (uri[i] == '%' && i + 2 < uri.size()) {
            const std::string hex = uri.substr(i + 1, 2);
            char* end = nullptr;
            const long v = std::strtol(hex.c_str(), &end, 16);
            if (end != nullptr && *end == '\0') {
                out += static_cast<char>(v);
                i += 2;
                continue;
            }
        }
        out += uri[i];
    }
    return out;
}

bool base64(const std::string& text, std::vector<std::uint8_t>& out) {
    static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    unsigned value = 0;
    int bits = -8;
    for (const char ch : text) {
        if (ch == '=') break;
        const std::size_t k = chars.find(ch);
        if (k == std::string::npos) {
            if (ch == '\n' || ch == '\r' || ch == ' ') continue;
            return false;
        }
        value = (value << 6) | static_cast<unsigned>(k);
        bits += 6;
        if (bits >= 0) {
            out.push_back(static_cast<std::uint8_t>((value >> bits) & 0xFFu));
            bits -= 8;
        }
    }
    return true;
}

Mat4 nodeMatrix(const Json& n) {
    if (n.contains("matrix") && n["matrix"].is_array() && n["matrix"].size() == 16) {
        Mat4 m{};
        for (std::size_t k = 0; k < 16; ++k) m[k] = n["matrix"][k].get<double>();
        return m;
    }
    double t[3] = {0, 0, 0}, q[4] = {0, 0, 0, 1}, s[3] = {1, 1, 1};
    if (n.contains("translation")) {
        for (std::size_t k = 0; k < 3; ++k) t[k] = n["translation"][k].get<double>();
    }
    if (n.contains("rotation")) {
        for (std::size_t k = 0; k < 4; ++k) q[k] = n["rotation"][k].get<double>();
    }
    if (n.contains("scale")) {
        for (std::size_t k = 0; k < 3; ++k) s[k] = n["scale"][k].get<double>();
    }
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                            {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                            {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
    Mat4 m = identity();
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 3; ++r) m[static_cast<std::size_t>(c * 4 + r)] = R[r][c] * s[c];
    }
    m[12] = t[0];
    m[13] = t[1];
    m[14] = t[2];
    return m;
}

}  // namespace

bool read(const std::string& path, Document& doc, std::string& error) {
    doc = Document{};
    doc.path = path;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "无法读取 glTF：" + path;
        return false;
    }
    const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    Json j;
    std::vector<std::uint8_t> glbBin;
    bool glb = false;
    try {
        if (bytes.size() >= 12 && std::memcmp(bytes.data(), "glTF", 4) == 0) {
            glb = true;
            std::size_t at = 12;
            while (at + 8 <= bytes.size()) {
                std::uint32_t length = 0, type = 0;
                std::memcpy(&length, bytes.data() + at, 4);
                std::memcpy(&type, bytes.data() + at + 4, 4);
                if (at + 8 + length > bytes.size()) {
                    error = "glb 块越界：" + path;
                    return false;
                }
                if (type == 0x4E4F534Au) j = Json::parse(bytes.begin() + static_cast<std::ptrdiff_t>(at + 8), bytes.begin() + static_cast<std::ptrdiff_t>(at + 8 + length));
                else if (type == 0x004E4942u) glbBin.assign(bytes.begin() + static_cast<std::ptrdiff_t>(at + 8), bytes.begin() + static_cast<std::ptrdiff_t>(at + 8 + length));
                at += 8 + length;
            }
        } else {
            j = Json::parse(bytes.begin(), bytes.end());
        }
    } catch (const Json::exception& e) {
        error = std::string{"glTF 的 JSON 无法解析："} + e.what();
        return false;
    }
    if (!j.is_object()) {
        error = "不是 glTF 文件：" + path;
        return false;
    }
    try {
        const fs::path base = fs::path{path}.parent_path();
        // 缓冲
        if (j.contains("buffers")) {
            for (std::size_t b = 0; b < j["buffers"].size(); ++b) {
                const Json& buffer = j["buffers"][b];
                std::vector<std::uint8_t> data;
                if (!buffer.contains("uri")) {
                    if (glb && b == 0) data = glbBin;
                } else {
                    const std::string uri = buffer["uri"].get<std::string>();
                    if (uri.rfind("data:", 0) == 0) {
                        const std::size_t comma = uri.find(',');
                        if (comma == std::string::npos || !base64(uri.substr(comma + 1), data)) {
                            error = "缓冲 " + std::to_string(b) + " 的 data URI 无法解码";
                            return false;
                        }
                    } else {
                        std::ifstream bin(base / percentDecoded(uri), std::ios::binary);
                        if (!bin) {
                            error = "缓冲文件不存在：" + (base / percentDecoded(uri)).string();
                            return false;
                        }
                        data.assign(std::istreambuf_iterator<char>(bin), std::istreambuf_iterator<char>());
                    }
                }
                doc.buffers.push_back(std::move(data));
            }
        }
        if (j.contains("bufferViews")) {
            for (const Json& v : j["bufferViews"]) {
                Document::View view;
                view.buffer = v.value("buffer", -1);
                view.offset = v.value("byteOffset", std::size_t{0});
                view.length = v.value("byteLength", std::size_t{0});
                view.stride = v.value("byteStride", std::size_t{0});
                doc.views.push_back(view);
            }
        }
        if (j.contains("accessors")) {
            for (const Json& a : j["accessors"]) {
                Document::Accessor acc;
                acc.view = a.value("bufferView", -1);
                acc.offset = a.value("byteOffset", std::size_t{0});
                acc.count = a.value("count", std::size_t{0});
                acc.componentType = a.value("componentType", 0);
                const std::string type = a.value("type", std::string{});
                acc.components = type == "SCALAR" ? 1 : type == "VEC2" ? 2 : type == "VEC3" ? 3 : type == "VEC4" ? 4 : 0;
                std::array<Vec3d, 2> bounds{Vec3d{1e300, 1e300, 1e300}, Vec3d{-1e300, -1e300, -1e300}};
                if (a.contains("min") && a.contains("max") && a["min"].size() >= 3 && a["max"].size() >= 3) {
                    acc.hasBounds = true;
                    bounds = {Vec3d{a["min"][0].get<double>(), a["min"][1].get<double>(), a["min"][2].get<double>()},
                              Vec3d{a["max"][0].get<double>(), a["max"][1].get<double>(), a["max"][2].get<double>()}};
                }
                doc.accessors.push_back(acc);
                doc.accessorBounds.push_back(bounds);
            }
        }
        if (j.contains("meshes")) {
            for (const Json& m : j["meshes"]) {
                std::vector<Primitive> prims;
                if (m.contains("primitives")) {
                    for (const Json& p : m["primitives"]) {
                        Primitive prim;
                        if (p.contains("attributes") && p["attributes"].contains("POSITION")) prim.positions = p["attributes"]["POSITION"].get<int>();
                        prim.indices = p.value("indices", -1);
                        prim.mode = p.value("mode", 4);
                        prims.push_back(prim);
                    }
                }
                doc.meshes.push_back(std::move(prims));
            }
        }
        if (j.contains("nodes")) {
            for (const Json& n : j["nodes"]) {
                Node node;
                node.name = n.value("name", std::string{});
                node.mesh = n.value("mesh", -1);
                if (n.contains("children")) {
                    for (const Json& c : n["children"]) node.children.push_back(c.get<int>());
                }
                node.local = nodeMatrix(n);
                doc.nodes.push_back(std::move(node));
            }
        }
        for (std::size_t i = 0; i < doc.nodes.size(); ++i) {
            for (const int c : doc.nodes[i].children) {
                if (c < 0 || static_cast<std::size_t>(c) >= doc.nodes.size()) {
                    error = "节点 " + std::to_string(i) + " 的子节点越界";
                    return false;
                }
                doc.nodes[static_cast<std::size_t>(c)].parent = static_cast<int>(i);
            }
        }
        const int scene = j.value("scene", 0);
        if (j.contains("scenes") && scene >= 0 && static_cast<std::size_t>(scene) < j["scenes"].size() && j["scenes"][static_cast<std::size_t>(scene)].contains("nodes")) {
            for (const Json& r : j["scenes"][static_cast<std::size_t>(scene)]["nodes"]) doc.roots.push_back(r.get<int>());
        } else {
            for (std::size_t i = 0; i < doc.nodes.size(); ++i) {
                if (doc.nodes[i].parent < 0) doc.roots.push_back(static_cast<int>(i));
            }
        }
    } catch (const Json::exception& e) {
        error = std::string{"glTF 的结构不对："} + e.what();
        return false;
    }
    // 世界矩阵（自上而下）
    std::vector<int> stack;
    for (const int r : doc.roots) {
        if (r < 0 || static_cast<std::size_t>(r) >= doc.nodes.size()) continue;
        doc.nodes[static_cast<std::size_t>(r)].world = doc.nodes[static_cast<std::size_t>(r)].local;
        stack.push_back(r);
    }
    std::size_t guard = 0;
    while (!stack.empty() && guard++ < doc.nodes.size() * 4 + 16) {
        const int n = stack.back();
        stack.pop_back();
        for (const int c : doc.nodes[static_cast<std::size_t>(n)].children) {
            doc.nodes[static_cast<std::size_t>(c)].world = multiply(doc.nodes[static_cast<std::size_t>(n)].world, doc.nodes[static_cast<std::size_t>(c)].local);
            stack.push_back(c);
        }
    }
    return true;
}

namespace {

const std::uint8_t* accessorData(const Document& doc, int index, std::size_t elementSize, std::size_t& stride, std::size_t& count, std::string& error) {
    if (index < 0 || static_cast<std::size_t>(index) >= doc.accessors.size()) {
        error = "访问器越界：" + std::to_string(index);
        return nullptr;
    }
    const auto& a = doc.accessors[static_cast<std::size_t>(index)];
    if (a.view < 0 || static_cast<std::size_t>(a.view) >= doc.views.size()) {
        error = "访问器 " + std::to_string(index) + " 没有缓冲视图（稀疏访问器不支持）";
        return nullptr;
    }
    const auto& v = doc.views[static_cast<std::size_t>(a.view)];
    if (v.buffer < 0 || static_cast<std::size_t>(v.buffer) >= doc.buffers.size()) {
        error = "缓冲视图越界";
        return nullptr;
    }
    const auto& buffer = doc.buffers[static_cast<std::size_t>(v.buffer)];
    stride = v.stride != 0 ? v.stride : elementSize;
    count = a.count;
    const std::size_t start = v.offset + a.offset;
    if (count > 0 && start + stride * (count - 1) + elementSize > buffer.size()) {
        error = "访问器 " + std::to_string(index) + " 的数据越界";
        return nullptr;
    }
    return buffer.data() + start;
}

}  // namespace

std::vector<Vec3d> positions(const Document& doc, int accessor, std::string& error) {
    std::vector<Vec3d> out;
    if (accessor < 0 || static_cast<std::size_t>(accessor) >= doc.accessors.size()) {
        error = "没有 POSITION";
        return out;
    }
    const auto& a = doc.accessors[static_cast<std::size_t>(accessor)];
    if (a.componentType != 5126 || a.components != 3) {
        error = "POSITION 应为 float VEC3";
        return out;
    }
    std::size_t stride = 0, count = 0;
    const std::uint8_t* p = accessorData(doc, accessor, 12, stride, count, error);
    if (p == nullptr) return out;
    out.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        float v[3];
        std::memcpy(v, p + i * stride, 12);
        out[i] = {static_cast<double>(v[0]), static_cast<double>(v[1]), static_cast<double>(v[2])};
    }
    return out;
}

std::vector<std::uint32_t> indices(const Document& doc, int accessor, std::string& error) {
    std::vector<std::uint32_t> out;
    if (accessor < 0 || static_cast<std::size_t>(accessor) >= doc.accessors.size()) {
        error = "索引访问器越界";
        return out;
    }
    const auto& a = doc.accessors[static_cast<std::size_t>(accessor)];
    const std::size_t size = a.componentType == 5121 ? 1 : a.componentType == 5123 ? 2 : a.componentType == 5125 ? 4 : 0;
    if (size == 0 || a.components != 1) {
        error = "索引应为 u8 / u16 / u32 标量";
        return out;
    }
    std::size_t stride = 0, count = 0;
    const std::uint8_t* p = accessorData(doc, accessor, size, stride, count, error);
    if (p == nullptr) return out;
    out.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint8_t* q = p + i * stride;
        if (size == 1) {
            out[i] = q[0];
        } else if (size == 2) {
            std::uint16_t v = 0;
            std::memcpy(&v, q, 2);
            out[i] = v;
        } else {
            std::memcpy(&out[i], q, 4);
        }
    }
    return out;
}

int findNode(const Document& doc, const std::string& name) {
    for (std::size_t i = 0; i < doc.nodes.size(); ++i) {
        if (doc.nodes[i].name == name) return static_cast<int>(i);
    }
    return -1;
}

bool inSubtree(const Document& doc, int node, int root) {
    for (int n = node, guard = 0; n >= 0 && guard < 10000; n = doc.nodes[static_cast<std::size_t>(n)].parent, ++guard) {
        if (n == root) return true;
    }
    return false;
}

}  // namespace terra::importer::gltf
