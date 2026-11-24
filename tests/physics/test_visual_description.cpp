// 显示用描述（M3-03，模型描述修订 4）。只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。
// 夹具 assets/tests/visual_materials.xml（网格在 assets/tests/visual/）：
//   - 网格法线：textured_cube.obj 的法线由文件给出，每个角的法线等于所在面的法线；smooth_ball.obj 没有法线，
//     MuJoCo 计算的平滑法线接近球的径向，且与面法线不同；
//   - 纹理坐标：textured_cube.obj 有，且 v 已翻转（UV 有向面积变号）；内联的 tetra 没有；
//   - 贴图：二维内置棋盘格的尺寸与像素（两种颜色都出现）；天空盒与立方体贴图只有尺寸，没有像素；
//   - 材质：颜色、自发光、specular、shininess、metallic / roughness（给出与未给出）、贴图下标、重复次数、texuniform；
//   - 几何体的材质下标；没有材质的几何体为 −1。

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace td = terra::data;
using td::Vec3;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) <= tol; }

Vec3 normalized(const Vec3& v) {
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return n > 0 ? Vec3{v.x / n, v.y / n, v.z / n} : v;
}
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 faceNormal(const td::MeshDescription& m, std::size_t i) {
    const auto& t = m.triangles[i];
    const Vec3& a = m.vertices[t[0]];
    const Vec3& b = m.vertices[t[1]];
    const Vec3& c = m.vertices[t[2]];
    const Vec3 e1{b.x - a.x, b.y - a.y, b.z - a.z};
    const Vec3 e2{c.x - a.x, c.y - a.y, c.z - a.z};
    return normalized({e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x});
}

const td::MeshDescription* meshOf(const td::ModelDescription& d, const char* geomName) {
    const auto* g = d.findGeom(geomName);
    check(g != nullptr && g->mesh >= 0, std::string{"几何体 "} + geomName + " 是网格");
    return g != nullptr && g->mesh >= 0 ? &d.meshes[static_cast<std::size_t>(g->mesh)] : nullptr;
}

int materialIndex(const td::ModelDescription& d, const std::string& name) {
    for (std::size_t i = 0; i < d.materials.size(); ++i) {
        if (d.materials[i].name == name) return static_cast<int>(i);
    }
    check(false, "找到材质 " + name);
    return -1;
}

int textureIndex(const td::ModelDescription& d, const std::string& name) {
    for (std::size_t i = 0; i < d.textures.size(); ++i) {
        if (d.textures[i].name == name) return static_cast<int>(i);
    }
    check(false, "找到贴图 " + name);
    return -1;
}

void testMeshes(const td::ModelDescription& d) {
    if (const auto* cube = meshOf(d, "cube")) {
        std::printf("INFO textured_cube：顶点 %zu，三角形 %zu，法线 %zu，纹理坐标 %zu\n", cube->vertices.size(), cube->triangles.size(),
                    cube->normals.size(), cube->texcoords.size());
        check(cube->triangles.size() == 12, "立方体 12 个三角形");
        check(cube->triangleNormals.size() == cube->triangles.size() && !cube->normals.empty(), "立方体：每个三角形都有法线索引");
        check(cube->triangleTexcoords.size() == cube->triangles.size() && cube->texcoords.size() >= 4, "立方体：每个三角形都有纹理坐标索引");
        std::size_t aligned = 0;
        std::size_t corners = 0;
        bool indicesOk = true;
        for (std::size_t i = 0; i < cube->triangleNormals.size() && i < cube->triangles.size(); ++i) {
            const Vec3 f = faceNormal(*cube, i);
            for (std::size_t k = 0; k < 3; ++k) {
                const auto ni = cube->triangleNormals[i][k];
                if (ni >= cube->normals.size()) {
                    indicesOk = false;
                    continue;
                }
                ++corners;
                aligned += dot(normalized(cube->normals[ni]), f) > 0.999 ? 1u : 0u;
            }
        }
        check(indicesOk, "立方体：法线索引都在范围内");
        check(corners == 36 && aligned == corners,
              "立方体：文件给出的法线随网格变换后仍等于面法线（" + std::to_string(aligned) + "/" + std::to_string(corners) + "）");
        // 文件中 vt 为 (0,0)(1,0)(1,1)(0,1)；v 翻转后仍是这四个角
        bool flipped = !cube->texcoords.empty();
        for (const auto& uv : cube->texcoords) {
            const bool corner = (near(uv[0], 0) || near(uv[0], 1)) && (near(uv[1], 0) || near(uv[1], 1));
            flipped = flipped && corner;
        }
        check(flipped, "立方体：纹理坐标都在 [0, 1]² 的角上");
        // v 翻转的检验与网格编译时的旋转无关：文件中每个面的纹理坐标与三角形同为逆时针（UV 有向面积为正），
        // v 翻转后 UV 有向面积为负
        std::size_t mirrored = 0;
        for (std::size_t i = 0; i < cube->triangleTexcoords.size(); ++i) {
            const auto& t = cube->triangleTexcoords[i];
            if (t[0] >= cube->texcoords.size() || t[1] >= cube->texcoords.size() || t[2] >= cube->texcoords.size()) continue;
            const auto& a = cube->texcoords[t[0]];
            const auto& b = cube->texcoords[t[1]];
            const auto& c = cube->texcoords[t[2]];
            const double area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            mirrored += area < 0 ? 1u : 0u;
        }
        check(mirrored == cube->triangles.size(),
              "立方体：纹理坐标的 v 已翻转（12 个三角形的 UV 有向面积都为负，实际 " + std::to_string(mirrored) + " 个）");
    }

    if (const auto* ball = meshOf(d, "ball")) {
        check(ball->triangles.size() == 224 && ball->triangleNormals.size() == ball->triangles.size(), "球：224 个三角形，都有法线索引");
        check(ball->texcoords.empty() && ball->triangleTexcoords.empty(), "球：没有纹理坐标");
        std::size_t radial = 0;
        std::size_t smooth = 0;
        std::size_t corners = 0;
        for (std::size_t i = 0; i < ball->triangleNormals.size(); ++i) {
            const Vec3 f = faceNormal(*ball, i);
            for (std::size_t k = 0; k < 3; ++k) {
                const auto ni = ball->triangleNormals[i][k];
                if (ni >= ball->normals.size()) continue;
                ++corners;
                const Vec3 n = normalized(ball->normals[ni]);
                // 网格编译时平移到质心；球关于原点对称，顶点方向即径向
                const Vec3 r = normalized(ball->vertices[ball->triangles[i][k]]);
                radial += dot(n, r) > 0.98 ? 1u : 0u;
                smooth += dot(n, f) < 0.9999 ? 1u : 0u;
            }
        }
        std::printf("INFO smooth_ball：%zu 个角，法线接近径向 %zu，与面法线不同 %zu\n", corners, radial, smooth);
        check(corners == 3 * ball->triangles.size() && radial == corners, "球：MuJoCo 计算的法线接近径向（平滑）");
        check(smooth > corners / 2, "球：多数角的法线与面法线不同（不是平直着色）");
    }

    if (const auto* tetra = meshOf(d, "tetra")) {
        check(tetra->texcoords.empty(), "内联 tetra：没有纹理坐标");
        check(!tetra->normals.empty(), "内联 tetra：MuJoCo 计算了法线");
    }
}

void testTextures(const td::ModelDescription& d) {
    check(d.textures.size() == 3, "3 个贴图，实际 " + std::to_string(d.textures.size()));
    const int checker = textureIndex(d, "checker");
    if (checker >= 0) {
        const auto& t = d.textures[static_cast<std::size_t>(checker)];
        check(t.kind == td::TextureKind::TwoD && t.width == 64 && t.height == 64, "checker：二维，64 × 64");
        check(t.rgba.size() == 64u * 64u * 4u, "checker：RGBA8 像素");
        bool light = false, dark = false, opaque = true;
        for (std::size_t p = 0; p + 3 < t.rgba.size(); p += 4) {
            const int r = t.rgba[p], g = t.rgba[p + 1], b = t.rgba[p + 2];
            light = light || (std::abs(r - 230) <= 1 && std::abs(g - 230) <= 1 && std::abs(b - 230) <= 1);
            dark = dark || (std::abs(r - 26) <= 1 && std::abs(g - 77) <= 1 && std::abs(b - 153) <= 1);
            opaque = opaque && t.rgba[p + 3] == 255;
        }
        check(light && dark, "checker：rgb1 (0.9, 0.9, 0.9) 与 rgb2 (0.1, 0.3, 0.6) 两种颜色都出现");
        check(opaque, "checker：三通道贴图补齐为不透明");
    }
    const int sky = textureIndex(d, "sky");
    if (sky >= 0) {
        const auto& t = d.textures[static_cast<std::size_t>(sky)];
        check(t.kind == td::TextureKind::Skybox && t.rgba.empty() && t.width > 0, "sky：天空盒，只有尺寸");
    }
    const int cube = textureIndex(d, "cubetex");
    if (cube >= 0) {
        const auto& t = d.textures[static_cast<std::size_t>(cube)];
        check(t.kind == td::TextureKind::Cube && t.rgba.empty(), "cubetex：立方体贴图，只有尺寸");
    }
}

void testMaterials(const td::ModelDescription& d) {
    check(d.materials.size() == 5, "5 个材质，实际 " + std::to_string(d.materials.size()));
    const int checker = textureIndex(d, "checker");
    const int floor = materialIndex(d, "floor");
    if (floor >= 0) {
        const auto& m = d.materials[static_cast<std::size_t>(floor)];
        check(m.texture == checker && m.texUniform && near(m.texRepeat[0], 4) && near(m.texRepeat[1], 4), "floor：贴图 checker，texuniform，每米 4 次");
        check(near(m.reflectance, 0.2), "floor：reflectance 0.2（记录，不用于显示）");
    }
    const int decal = materialIndex(d, "decal");
    if (decal >= 0) {
        const auto& m = d.materials[static_cast<std::size_t>(decal)];
        check(m.texture == checker && !m.texUniform && near(m.texRepeat[0], 2) && near(m.texRepeat[1], 3), "decal：重复 2 × 3");
        check(near(m.rgba[0], 1) && near(m.rgba[3], 1), "decal：默认颜色为白色");
    }
    const int shiny = materialIndex(d, "shiny");
    if (shiny >= 0) {
        const auto& m = d.materials[static_cast<std::size_t>(shiny)];
        check(near(m.rgba[0], 0.8) && near(m.rgba[1], 0.6) && near(m.rgba[2], 0.2), "shiny：颜色");
        check(near(m.specular, 0.9) && near(m.shininess, 0.95) && near(m.emission, 0.5), "shiny：specular 0.9、shininess 0.95、emission 0.5");
        check(m.metallic < 0 && m.roughness < 0 && m.texture == -1, "shiny：没有给出 metallic、roughness 与贴图");
    }
    const int pbr = materialIndex(d, "pbr");
    if (pbr >= 0) {
        const auto& m = d.materials[static_cast<std::size_t>(pbr)];
        check(near(m.metallic, 1) && near(m.roughness, 0.3), "pbr：metallic 1、roughness 0.3");
        check(near(m.specular, 0.5) && near(m.shininess, 0.5), "pbr：specular、shininess 为 MuJoCo 默认值 0.5");
    }
    const int cubed = materialIndex(d, "cubed");
    if (cubed >= 0) check(d.materials[static_cast<std::size_t>(cubed)].texture == textureIndex(d, "cubetex"), "cubed：立方体贴图");

    const auto* ball = d.findGeom("ball");
    check(ball != nullptr && ball->material == shiny, "几何体 ball 的材质下标");
    if (ball != nullptr) check(near(ball->rgba[0], 0.8) && near(ball->rgba[1], 0.6), "ball：颜色取材质颜色（M2-03 规则）");
    const auto* plain = d.findGeom("plain");
    check(plain != nullptr && plain->material == -1, "没有材质的几何体：材质下标 −1");
}

}  // namespace

int main() {
    auto loaded = terra::physics::mujoco::loadWorld(td::ModelSource{std::string{TERRA_TEST_ASSETS_DIR} + "/visual_materials.xml"});
    if (loaded.error) std::fprintf(stderr, "加载错误：%s\n", loaded.error->message.c_str());
    check(loaded.world != nullptr, "加载 visual_materials.xml");
    if (!loaded.world) return 1;
    const td::ModelDescription& d = loaded.world->description();
    check(td::ModelDescription::kRevision >= 4, "模型描述修订号 ≥ 4（M3-06 起为 5）");
    testMeshes(d);
    testTextures(d);
    testMaterials(d);
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
