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
    if (e.collision == ElementCollision::Boxes) {
        double volume = 0.0;
        for (const ElementBox& b : e.boxes) volume += 8.0 * b.halfSize.x * b.halfSize.y * b.halfSize.z;
        for (std::size_t i = 0; i < e.boxes.size(); ++i) {
            const ElementBox& b = e.boxes[i];
            geoms += "      <geom name=\"c" + std::to_string(i) + "\" type=\"box\" pos=\"" + vec(b.pose.position) + "\" quat=\"" + quat(b.pose.orientation) +
                     "\" size=\"" + vec(b.halfSize) + "\"" + common;
            // dynamic：质量按体积分到各盒体（总质量与元素文件相同）
            if (dynamic && volume > 0) geoms += " mass=\"" + num(e.mass * 8.0 * b.halfSize.x * b.halfSize.y * b.halfSize.z / volume) + "\"";
            geoms += "/>\n";
        }
    } else if (e.collision == ElementCollision::Mesh) {
        for (std::size_t i = 0; i < e.meshes.size(); ++i) {
            const ElementMesh& m = e.meshes[i];
            assets += "    <mesh name=\"m" + std::to_string(i) + "\" file=\"" + escaped(m.file) + "\" scale=\"" + num(m.scale) + " " + num(m.scale) + " " +
                      num(m.scale) + "\"/>\n";
            geoms += "      <geom name=\"c" + std::to_string(i) + "\" type=\"mesh\" mesh=\"m" + std::to_string(i) + "\" pos=\"" + vec(m.pose.position) +
                     "\" quat=\"" + quat(m.pose.orientation) + "\"" + common;
            // dynamic：网格的体积由 MuJoCo 算，这里按个数平分质量
            if (dynamic) geoms += " mass=\"" + num(e.mass / static_cast<double>(e.meshes.size())) + "\"";
            geoms += "/>\n";
        }
    }
    std::string xml = "<mujoco model=\"element\">\n";
    if (!assets.empty()) xml += "  <asset>\n" + assets + "  </asset>\n";
    xml += "  <worldbody>\n    <body name=\"" + std::string{kElementBody} + "\">\n";
    if (dynamic) xml += "      <freejoint name=\"free\"/>\n";
    xml += geoms;
    xml += "    </body>\n  </worldbody>\n</mujoco>\n";
    return xml;
}

}  // namespace terra::scene
