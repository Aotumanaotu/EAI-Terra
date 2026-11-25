#pragma once

// URDF 的读取（M4-08a，内部）。只读导入需要的部分：连杆（惯量、视觉、碰撞）、关节、材质、传动（transmission 中的关节名）。

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace terra::importer::detail {

using Vec3 = std::array<double, 3>;
using Rgba = std::array<double, 4>;

struct Origin {
    Vec3 xyz{0.0, 0.0, 0.0};
    Vec3 rpy{0.0, 0.0, 0.0};
};

struct Geometry {
    enum class Type { None, Box, Cylinder, Sphere, Mesh };
    Type type = Type::None;
    Vec3 size{0.0, 0.0, 0.0};  // box：全长
    double radius = 0.0;
    double length = 0.0;
    std::string filename;      // mesh
    Vec3 scale{1.0, 1.0, 1.0};
    bool hasScale = false;
};

struct Material {
    std::string name;
    std::optional<Rgba> rgba;
    std::string texture;
};

struct Visual {
    std::string name;
    Origin origin;
    Geometry geometry;
    Material material;  // 内联或引用全局材质（解析时合并）
};

struct Collision {
    std::string name;
    Origin origin;
    Geometry geometry;
};

struct Inertial {
    Origin origin;
    double mass = 0.0;
    double ixx = 0.0, ixy = 0.0, ixz = 0.0, iyy = 0.0, iyz = 0.0, izz = 0.0;
};

struct Link {
    std::string name;
    std::optional<Inertial> inertial;
    std::vector<Visual> visuals;
    std::vector<Collision> collisions;
};

struct Joint {
    std::string name;
    std::string type;  // revolute、continuous、prismatic、fixed、floating、planar
    std::string parent;
    std::string child;
    Origin origin;
    Vec3 axis{1.0, 0.0, 0.0};
    bool hasLimit = false;
    double lower = 0.0, upper = 0.0, effort = 0.0, velocity = 0.0;
    double damping = 0.0;
    double friction = 0.0;
    bool mimic = false;
};

struct UrdfModel {
    std::string name;
    std::vector<Link> links;
    std::vector<Joint> joints;
    std::vector<std::string> transmissionJoints;
    const Link* findLink(const std::string& name) const;
    const Joint* jointWithChild(const std::string& child) const;
};

// 解析 URDF 文本；失败时返回空，error 为原因（带行号）
std::optional<UrdfModel> parseUrdf(const std::string& text, std::string& error);

// 文本看起来是否含 xacro（xacro 命名空间的元素或 ${…}、$(…) 替换）
bool looksLikeXacro(const std::string& text);

}  // namespace terra::importer::detail
