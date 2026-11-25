#include "urdf.hpp"

#include <sstream>

#include "tinyxml2.h"

namespace terra::importer::detail {

namespace {

using tinyxml2::XMLElement;

std::vector<double> numbers(const char* text) {
    std::vector<double> out;
    if (text == nullptr) return out;
    std::istringstream in(text);
    double v = 0.0;
    while (in >> v) out.push_back(v);
    return out;
}

Vec3 vec3(const char* text, Vec3 fallback) {
    const auto v = numbers(text);
    if (v.size() != 3) return fallback;
    return {v[0], v[1], v[2]};
}

double number(const XMLElement* e, const char* name, double fallback = 0.0) {
    double v = fallback;
    if (e != nullptr) e->QueryDoubleAttribute(name, &v);
    return v;
}

std::string text(const XMLElement* e, const char* name) {
    const char* v = e != nullptr ? e->Attribute(name) : nullptr;
    return v != nullptr ? std::string{v} : std::string{};
}

Origin readOrigin(const XMLElement* parent) {
    Origin o;
    if (const XMLElement* e = parent->FirstChildElement("origin")) {
        o.xyz = vec3(e->Attribute("xyz"), o.xyz);
        o.rpy = vec3(e->Attribute("rpy"), o.rpy);
    }
    return o;
}

Geometry readGeometry(const XMLElement* parent) {
    Geometry g;
    const XMLElement* geometry = parent->FirstChildElement("geometry");
    if (geometry == nullptr) return g;
    if (const XMLElement* e = geometry->FirstChildElement("box")) {
        g.type = Geometry::Type::Box;
        g.size = vec3(e->Attribute("size"), g.size);
    } else if (const XMLElement* c = geometry->FirstChildElement("cylinder")) {
        g.type = Geometry::Type::Cylinder;
        g.radius = number(c, "radius");
        g.length = number(c, "length");
    } else if (const XMLElement* s = geometry->FirstChildElement("sphere")) {
        g.type = Geometry::Type::Sphere;
        g.radius = number(s, "radius");
    } else if (const XMLElement* m = geometry->FirstChildElement("mesh")) {
        g.type = Geometry::Type::Mesh;
        g.filename = text(m, "filename");
        if (m->Attribute("scale") != nullptr) {
            const auto v = numbers(m->Attribute("scale"));
            if (v.size() == 3) g.scale = {v[0], v[1], v[2]};
            else if (v.size() == 1) g.scale = {v[0], v[0], v[0]};
            g.hasScale = true;
        }
    }
    return g;
}

Material readMaterial(const XMLElement* e) {
    Material m;
    m.name = text(e, "name");
    if (const XMLElement* c = e->FirstChildElement("color")) {
        const auto v = numbers(c->Attribute("rgba"));
        if (v.size() == 4) m.rgba = Rgba{v[0], v[1], v[2], v[3]};
    }
    if (const XMLElement* t = e->FirstChildElement("texture")) m.texture = text(t, "filename");
    return m;
}

}  // namespace

const Link* UrdfModel::findLink(const std::string& n) const {
    for (const auto& l : links) {
        if (l.name == n) return &l;
    }
    return nullptr;
}

const Joint* UrdfModel::jointWithChild(const std::string& c) const {
    for (const auto& j : joints) {
        if (j.child == c) return &j;
    }
    return nullptr;
}

bool looksLikeXacro(const std::string& t) {
    return t.find("<xacro:") != std::string::npos || t.find("${") != std::string::npos || t.find("$(") != std::string::npos;
}

std::optional<UrdfModel> parseUrdf(const std::string& source, std::string& error) {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(source.c_str(), source.size()) != tinyxml2::XML_SUCCESS) {
        error = std::string{"XML 语法错误（第 "} + std::to_string(doc.ErrorLineNum()) + " 行）：" + doc.ErrorStr();
        return std::nullopt;
    }
    const XMLElement* robot = doc.FirstChildElement("robot");
    if (robot == nullptr) {
        error = "根元素不是 <robot>，不是 URDF";
        return std::nullopt;
    }
    UrdfModel model;
    model.name = text(robot, "name");
    std::map<std::string, Material> materials;
    for (const XMLElement* e = robot->FirstChildElement("material"); e != nullptr; e = e->NextSiblingElement("material")) {
        const Material m = readMaterial(e);
        if (!m.name.empty()) materials[m.name] = m;
    }
    for (const XMLElement* e = robot->FirstChildElement("link"); e != nullptr; e = e->NextSiblingElement("link")) {
        Link link;
        link.name = text(e, "name");
        if (const XMLElement* in = e->FirstChildElement("inertial")) {
            Inertial inertial;
            inertial.origin = readOrigin(in);
            inertial.mass = number(in->FirstChildElement("mass"), "value");
            if (const XMLElement* i = in->FirstChildElement("inertia")) {
                inertial.ixx = number(i, "ixx");
                inertial.ixy = number(i, "ixy");
                inertial.ixz = number(i, "ixz");
                inertial.iyy = number(i, "iyy");
                inertial.iyz = number(i, "iyz");
                inertial.izz = number(i, "izz");
            }
            link.inertial = inertial;
        }
        for (const XMLElement* v = e->FirstChildElement("visual"); v != nullptr; v = v->NextSiblingElement("visual")) {
            Visual visual;
            visual.name = text(v, "name");
            visual.origin = readOrigin(v);
            visual.geometry = readGeometry(v);
            if (const XMLElement* m = v->FirstChildElement("material")) {
                visual.material = readMaterial(m);
                // 只写名字时引用全局材质；内联定义优先
                const auto global = materials.find(visual.material.name);
                if (global != materials.end()) {
                    if (!visual.material.rgba) visual.material.rgba = global->second.rgba;
                    if (visual.material.texture.empty()) visual.material.texture = global->second.texture;
                }
            }
            link.visuals.push_back(visual);
        }
        for (const XMLElement* c = e->FirstChildElement("collision"); c != nullptr; c = c->NextSiblingElement("collision")) {
            Collision collision;
            collision.name = text(c, "name");
            collision.origin = readOrigin(c);
            collision.geometry = readGeometry(c);
            link.collisions.push_back(collision);
        }
        model.links.push_back(std::move(link));
    }
    for (const XMLElement* e = robot->FirstChildElement("joint"); e != nullptr; e = e->NextSiblingElement("joint")) {
        Joint joint;
        joint.name = text(e, "name");
        joint.type = text(e, "type");
        joint.parent = text(e->FirstChildElement("parent"), "link");
        joint.child = text(e->FirstChildElement("child"), "link");
        joint.origin = readOrigin(e);
        if (const XMLElement* a = e->FirstChildElement("axis")) joint.axis = vec3(a->Attribute("xyz"), joint.axis);
        if (const XMLElement* l = e->FirstChildElement("limit")) {
            joint.hasLimit = true;
            joint.lower = number(l, "lower");
            joint.upper = number(l, "upper");
            joint.effort = number(l, "effort");
            joint.velocity = number(l, "velocity");
        }
        if (const XMLElement* d = e->FirstChildElement("dynamics")) {
            joint.damping = number(d, "damping");
            joint.friction = number(d, "friction");
        }
        joint.mimic = e->FirstChildElement("mimic") != nullptr;
        model.joints.push_back(joint);
    }
    for (const XMLElement* t = robot->FirstChildElement("transmission"); t != nullptr; t = t->NextSiblingElement("transmission")) {
        for (const XMLElement* j = t->FirstChildElement("joint"); j != nullptr; j = j->NextSiblingElement("joint")) {
            model.transmissionJoints.push_back(text(j, "name"));
        }
    }
    if (model.links.empty()) {
        error = "URDF 中没有连杆（<link>）";
        return std::nullopt;
    }
    return model;
}

}  // namespace terra::importer::detail
