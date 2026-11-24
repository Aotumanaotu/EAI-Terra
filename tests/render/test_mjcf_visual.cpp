// MJCF 几何体的显示换算（M3-03，apps/viewer/mjcf_visual.hpp）：纯 CPU，不创建 OpenGL 上下文。
//   - 网格：每个三角形角一个顶点；使用给出的平滑法线；与面法线夹角超过 60° 的角改用面法线（折角保护）；
//     没有法线时平直着色；纹理坐标乘重复次数；没有纹理坐标时为 0；
//   - 材质：颜色 sRGB → 线性；粗糙度、金属度、反射率参数、自发光的换算；默认材质与 M2 的显示参数相同；
//   - 贴图：二维贴图用于带纹理坐标的网格与基本形状；平面 texuniform 与按尺寸归一化；不支持的情况只显示颜色并标记。

#include <cmath>
#include <cstdio>
#include <string>

#include "mjcf_visual.hpp"

namespace {

namespace td = terra::data;
namespace r = terra::render;
namespace vw = terra::viewer;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-5) { return std::fabs(a - b) <= tol; }

// 两个三角形：(0,0,0)(1,0,0)(0,1,0) 面法线 +Z；第二个与第一个相同位置，用于检查折角保护
td::MeshDescription hinge() {
    td::MeshDescription m;
    m.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    m.triangles = {{0, 1, 2}, {0, 1, 2}};
    const double s = std::sin(30.0 * 3.14159265358979 / 180.0), c = std::cos(30.0 * 3.14159265358979 / 180.0);
    const double s80 = std::sin(80.0 * 3.14159265358979 / 180.0), c80 = std::cos(80.0 * 3.14159265358979 / 180.0);
    m.normals = {{0, 0, 1}, {s, 0, c}, {s80, 0, c80}, {0, 0, 2}};  // 0°、30°、80°；最后一个未归一化
    m.triangleNormals = {{0, 1, 3}, {2, 2, 0}};
    m.texcoords = {{0, 0}, {1, 0}, {0, 1}};
    m.triangleTexcoords = {{0, 1, 2}, {0, 1, 2}};
    return m;
}

void testMesh() {
    const td::MeshDescription m = hinge();
    const r::MeshData data = vw::mjcfMesh(m, {2.0f, 3.0f});
    check(data.vertices.size() == 6 && data.indices.size() == 6, "每个三角形角一个顶点（6 个）");
    if (data.vertices.size() != 6) return;
    const auto& v1 = data.vertices[1];
    check(near(v1.normal[0], 0.5) && near(v1.normal[2], std::sqrt(0.75)), "30° 的平滑法线原样使用");
    const auto& v2 = data.vertices[2];
    check(near(v2.normal[2], 1.0), "未归一化的法线被归一化");
    const auto& v3 = data.vertices[3];
    check(near(v3.normal[0], 0.0) && near(v3.normal[2], 1.0), "与面法线夹角 80° 的角改用面法线（折角保护）");
    check(near(v1.uv[0], 2.0) && near(v1.uv[1], 0.0) && near(data.vertices[2].uv[1], 3.0), "纹理坐标乘重复次数 2 × 3");
    check(near(data.boundsMax[0], 1.0) && near(data.boundsMin[1], 0.0), "包围盒");
    check(near(v1.tangent[0] * v1.tangent[0] + v1.tangent[1] * v1.tangent[1] + v1.tangent[2] * v1.tangent[2], 1.0, 1e-4),
          "切线已计算（单位长度）");

    td::MeshDescription flat = m;
    flat.normals.clear();
    flat.triangleNormals.clear();
    flat.texcoords.clear();
    flat.triangleTexcoords.clear();
    const r::MeshData f = vw::mjcfMesh(flat);
    bool allFace = f.vertices.size() == 6;
    for (const auto& v : f.vertices) allFace = allFace && near(v.normal[2], 1.0) && v.uv[0] == 0.0f && v.uv[1] == 0.0f;
    check(allFace, "没有法线：平直着色（面法线）；没有纹理坐标：UV 为 0");

    td::MeshDescription bad = m;
    bad.triangles.push_back({0, 1, 9});  // 越界的三角形被跳过
    bad.triangleNormals.push_back({0, 0, 0});
    bad.triangleTexcoords.push_back({0, 0, 0});
    check(vw::mjcfMesh(bad).vertices.size() == 6, "顶点索引越界的三角形被跳过");
}

td::ModelDescription model() {
    td::ModelDescription d;
    td::TextureDescription checker;
    checker.name = "checker";
    checker.width = 2;
    checker.height = 2;
    checker.rgba.assign(16, 200);
    td::TextureDescription cube;
    cube.name = "cube";
    cube.kind = td::TextureKind::Cube;
    cube.width = 4;
    cube.height = 24;
    td::TextureDescription sky;
    sky.kind = td::TextureKind::Skybox;
    d.textures = {checker, cube, sky};

    td::MaterialDescription shiny;  // 0
    shiny.specular = 0.9f;
    shiny.shininess = 0.95f;
    shiny.emission = 0.5f;
    td::MaterialDescription pbr;  // 1
    pbr.metallic = 1.0f;
    pbr.roughness = 0.3f;
    td::MaterialDescription decal;  // 2
    decal.texture = 0;
    decal.texRepeat = {2.0f, 3.0f};
    td::MaterialDescription floor;  // 3
    floor.texture = 0;
    floor.texRepeat = {4.0f, 4.0f};
    floor.texUniform = true;
    td::MaterialDescription cubed;  // 4
    cubed.texture = 1;
    td::MaterialDescription skyMat;  // 5
    skyMat.texture = 2;
    d.materials = {shiny, pbr, decal, floor, cubed, skyMat};

    td::MeshDescription withUv = hinge();
    td::MeshDescription noUv = hinge();
    noUv.texcoords.clear();
    noUv.triangleTexcoords.clear();
    d.meshes = {withUv, noUv};
    return d;
}

td::GeomDescription geom(td::GeomShape shape, int material, int mesh = -1, td::Vec3 size = {0.1, 0.1, 0.1}) {
    td::GeomDescription g;
    g.shape = shape;
    g.material = material;
    g.mesh = mesh;
    g.size = size;
    g.rgba = {0.5f, 0.5f, 0.5f, 1.0f};
    return g;
}

void testSurface() {
    const td::ModelDescription d = model();
    const auto plain = vw::mjcfSurface(geom(td::GeomShape::Box, -1), d);
    check(near(plain.material.baseColor.x, vw::srgbToLinear(0.5f)) && near(plain.material.baseColor.x, 0.21404, 1e-4),
          "颜色 sRGB 0.5 → 线性 0.214");
    check(near(plain.material.roughness, 0.55) && near(plain.material.metallic, 0.0) && near(plain.material.reflectance, 0.5),
          "没有材质：粗糙度 0.55、金属度 0、反射率参数 0.5（与 M2 相同）");
    check(plain.texture == -1 && !plain.textureUnsupported, "没有材质：没有贴图");

    const auto shiny = vw::mjcfSurface(geom(td::GeomShape::Sphere, 0), d);
    check(near(shiny.material.roughness, 1.0 - 0.9 * 0.95), "粗糙度 = 1 − 0.9 × shininess");
    check(near(shiny.material.reflectance, 0.9), "反射率参数 = specular");
    check(near(shiny.material.emissive.x, 0.5 * shiny.material.baseColor.x), "自发光 = emission × 基础色");

    const auto pbr = vw::mjcfSurface(geom(td::GeomShape::Capsule, 1), d);
    check(near(pbr.material.metallic, 1.0) && near(pbr.material.roughness, 0.3), "模型给出的 metallic、roughness 直接使用");

    const auto meshUv = vw::mjcfSurface(geom(td::GeomShape::Mesh, 2, 0), d);
    check(meshUv.texture == 0 && near(meshUv.uvScale[0], 2) && near(meshUv.uvScale[1], 3), "带纹理坐标的网格：二维贴图，重复 2 × 3");
    const auto meshNoUv = vw::mjcfSurface(geom(td::GeomShape::Mesh, 2, 1), d);
    check(meshNoUv.texture == -1 && meshNoUv.textureUnsupported, "没有纹理坐标的网格：不支持，只显示颜色");
    const auto box = vw::mjcfSurface(geom(td::GeomShape::Box, 2), d);
    check(box.texture == 0 && near(box.uvScale[0], 2), "长方体：二维贴图，生成器纹理坐标 × 重复次数");

    const auto floorUniform = vw::mjcfSurface(geom(td::GeomShape::Plane, 3, -1, {0, 0, 0.05}), d);
    check(floorUniform.texture == 0 && near(floorUniform.uvScale[0], 4), "平面 texuniform：每米 4 次");
    const auto finite = vw::mjcfSurface(geom(td::GeomShape::Plane, 2, -1, {2, 0.5, 0.05}), d);
    check(near(finite.uvScale[0], 2.0 / 4.0) && near(finite.uvScale[1], 3.0 / 1.0), "有限平面（非 texuniform）：按平面尺寸归一化");

    const auto cubed = vw::mjcfSurface(geom(td::GeomShape::Box, 4), d);
    check(cubed.texture == -1 && cubed.textureUnsupported, "立方体贴图：不支持，只显示颜色");
    const auto sky = vw::mjcfSurface(geom(td::GeomShape::Box, 5), d);
    check(sky.texture == -1 && !sky.textureUnsupported, "天空盒材质：忽略（不计为不支持）");
    const auto outOfRange = vw::mjcfSurface(geom(td::GeomShape::Box, 99), d);
    check(outOfRange.texture == -1 && near(outOfRange.material.roughness, 0.55), "材质下标越界：按没有材质处理");
}

}  // namespace

int main() {
    testMesh();
    testSurface();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
