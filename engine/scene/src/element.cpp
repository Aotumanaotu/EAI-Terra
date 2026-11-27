// 场景元素的 MJCF 生成（M5-02a），说明见 element.hpp。元素文件的解析在 scene_file.cpp（与场景文件共用读取器）。

#include <cstdio>
#include <string>

#include "terra/scene/element.hpp"

namespace terra::scene {

namespace {

std::string num(double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

std::string vec(const terra::data::Vec3& v) { return num(v.x) + " " + num(v.y) + " " + num(v.z); }

std::string quat(const terra::data::Quaternion& q) { return num(q.w) + " " + num(q.x) + " " + num(q.y) + " " + num(q.z); }

std::string escaped(const std::string& text) {
    std::string out;
    for (const char c : text) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            default: out += c;
        }
    }
    return out;
}

}  // namespace

std::string elementMjcf(const ElementDocument& e) {
    const bool dynamic = e.movable == ElementMovable::Dynamic;
    // 碰撞体的半透明灰：有 glTF 外观时 MJCF 几何体默认隐藏，打开碰撞体显示时可见
    const std::string common = " friction=\"" + num(e.friction) + " 0.005 0.0001\" rgba=\"0.62 0.64 0.68 0.55\"";
    std::string assets;
    std::string geoms;
    // 盒体的总体积（含部件）：dynamic 的质量按体积分到各盒体
    const auto volumeOf = [](const ElementBox& b) { return 8.0 * b.halfSize.x * b.halfSize.y * b.halfSize.z; };
    double volume = 0.0;
    if (e.collision == ElementCollision::Boxes) {
        for (const ElementBox& b : e.boxes) volume += volumeOf(b);
        for (const ElementPart& part : e.parts) {
            for (const ElementBox& b : part.boxes) volume += volumeOf(b);
        }
    }
    const auto boxGeoms = [&](const std::vector<ElementBox>& boxes, const std::string& prefix, const std::string& indent) {
        std::string out;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const ElementBox& b = boxes[i];
            out += indent + "<geom name=\"" + prefix + std::to_string(i) + "\" type=\"box\" pos=\"" + vec(b.pose.position) + "\" quat=\"" +
                   quat(b.pose.orientation) + "\" size=\"" + vec(b.halfSize) + "\"" + common;
            if (dynamic && volume > 0) out += " mass=\"" + num(e.mass * volumeOf(b) / volume) + "\"";
            out += "/>\n";
        }
        return out;
    };
    if (e.collision == ElementCollision::Boxes) {
        geoms += boxGeoms(e.boxes, "c", "      ");
    }
    // 网格（凸包）：dynamic 的质量按个数平分（网格的体积由 MuJoCo 算）
    std::size_t meshCount = e.meshes.size();
    for (const ElementPart& part : e.parts) meshCount += part.meshes.size();
    std::size_t meshIndex = 0;
    const auto meshGeoms = [&](const std::vector<ElementMesh>& meshes, const std::string& indent) {
        std::string out;
        for (const ElementMesh& m : meshes) {
            const std::string id = std::to_string(meshIndex++);
            assets += "    <mesh name=\"m" + id + "\" file=\"" + escaped(m.file) + "\" scale=\"" + num(m.scale) + " " + num(m.scale) + " " + num(m.scale) + "\"/>\n";
            out += indent + "<geom name=\"m" + id + "\" type=\"mesh\" mesh=\"m" + id + "\" pos=\"" + vec(m.pose.position) + "\" quat=\"" +
                   quat(m.pose.orientation) + "\"" + common;
            if (dynamic && meshCount > 0) out += " mass=\"" + num(e.mass / static_cast<double>(meshCount)) + "\"";
            out += "/>\n";
        }
        return out;
    };
    if (e.collision == ElementCollision::Mesh) geoms += meshGeoms(e.meshes, "      ");
    // 部件（M5-02b）是焊在元素刚体上的子刚体（没有关节）；先生成刚体的内容，网格资源收集齐后再写 asset
    std::string body;
    if (dynamic) body += "      <freejoint name=\"free\"/>\n";
    // dynamic 而元素本身没有碰撞体（都在部件上）时，给根刚体一个很小的惯量（MuJoCo 要求运动的刚体质量大于 0）
    if (dynamic && geoms.empty()) body += "      <inertial pos=\"0 0 0\" mass=\"0.001\" diaginertia=\"1e-06 1e-06 1e-06\"/>\n";
    body += geoms;
    for (std::size_t k = 0; k < e.parts.size(); ++k) {
        const ElementPart& part = e.parts[k];
        body += "      <body name=\"" + escaped(part.name) + "\" pos=\"" + vec(part.pose.position) + "\" quat=\"" + quat(part.pose.orientation) + "\">\n";
        if (e.collision == ElementCollision::Boxes) body += boxGeoms(part.boxes, "p" + std::to_string(k) + "_", "        ");
        if (e.collision == ElementCollision::Mesh) body += meshGeoms(part.meshes, "        ");
        body += "      </body>\n";
    }
    std::string xml = "<mujoco model=\"element\">\n";
    if (!assets.empty()) xml += "  <asset>\n" + assets + "  </asset>\n";
    xml += "  <worldbody>\n    <body name=\"" + std::string{kElementBody} + "\">\n" + body + "    </body>\n  </worldbody>\n</mujoco>\n";
    return xml;
}

}  // namespace terra::scene
