// glTF 资产加载测试（M1-09）。纯 CPU（不需要 OpenGL）：测试在临时目录中用代码生成小型 .gltf / .bin / .png / .glb，
// 检查节点层级与 Y→Z 换算、8/16/32 位索引、缺失法线的生成、切线、材质因子与贴图槽位、sRGB 标记、双面；
// 以及诊断：不支持的扩展（required 为错误、used 为警告）、纹理坐标变换、缺失贴图、缺失缓冲区、损坏的 JSON、文件不存在。
// assets/models/ 中有真实资产时，另检查它们能无错误加载。
//
// 用法：test_asset_gltf <临时目录>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "terra/asset/gltf.hpp"

namespace {

namespace fs = std::filesystem;
namespace a = terra::asset;
namespace r = terra::render;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double x, double y, double tol = 1e-5) { return std::fabs(x - y) <= tol; }

bool mentions(const a::LoadResult& result, a::Diagnostic::Severity severity, const std::string& part) {
    for (const auto& d : result.diagnostics) {
        if (d.severity == severity && d.message.find(part) != std::string::npos) return true;
    }
    return false;
}

// 2 × 2 RGBA 图像，每个像素 (255, 128, 0, 255)
const std::uint8_t kPng[] = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x02, 0x08, 0x06, 0x00, 0x00, 0x00, 0x72, 0xb6, 0x0d, 0x24, 0x00, 0x00, 0x00, 0x11, 0x49,
    0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0xf8, 0xdf, 0xc0, 0xf0, 0x1f, 0x84, 0x19, 0x60, 0x0c, 0x00, 0x5a, 0xca, 0x09,
    0xf9, 0xef, 0xae, 0x6a, 0xa1, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82};

void writeBytes(const fs::path& p, const std::vector<std::uint8_t>& bytes) {
    std::ofstream out(p, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

void writeText(const fs::path& p, const std::string& text) {
    std::ofstream out(p, std::ios::binary);
    out << text;
}

// 二进制缓冲区与对应的 bufferViews / accessors JSON。
struct Bin {
    std::vector<std::uint8_t> bytes;
    std::vector<std::string> views;
    std::vector<std::string> accessors;

    int view(const void* data, std::size_t size) {
        while (bytes.size() % 4 != 0) bytes.push_back(0);
        const std::size_t offset = bytes.size();
        const auto* p = static_cast<const std::uint8_t*>(data);
        bytes.insert(bytes.end(), p, p + size);
        views.push_back("{\"buffer\":0,\"byteOffset\":" + std::to_string(offset) + ",\"byteLength\":" + std::to_string(size) + "}");
        return static_cast<int>(views.size()) - 1;
    }

    int floats(const std::vector<float>& v, int components, const char* type, const std::string& minMax = "") {
        const int bv = view(v.data(), v.size() * sizeof(float));
        accessors.push_back("{\"bufferView\":" + std::to_string(bv) + ",\"componentType\":5126,\"count\":" +
                            std::to_string(v.size() / static_cast<std::size_t>(components)) + ",\"type\":\"" + type + "\"" + minMax + "}");
        return static_cast<int>(accessors.size()) - 1;
    }

    template <typename T>
    int indices(const std::vector<T>& idx, int componentType) {
        const int bv = view(idx.data(), idx.size() * sizeof(T));
        accessors.push_back("{\"bufferView\":" + std::to_string(bv) + ",\"componentType\":" + std::to_string(componentType) +
                            ",\"count\":" + std::to_string(idx.size()) + ",\"type\":\"SCALAR\"}");
        return static_cast<int>(accessors.size()) - 1;
    }

    static std::string join(const std::vector<std::string>& items) {
        std::string out;
        for (std::size_t i = 0; i < items.size(); ++i) out += (i ? "," : "") + items[i];
        return out;
    }
};

struct Scene {
    Bin bin;
    std::string meshes;
};

// 场景：三角形（16 位索引，带法线与 UV）挂在父子节点下；正方形（8 位索引，无法线）；三角形（32 位索引）。
Scene buildScene() {
    Scene s;
    Bin& b = s.bin;
    const int pos = b.floats({0, 0, 0, 1, 0, 0, 0, 1, 0}, 3, "VEC3", ",\"min\":[0,0,0],\"max\":[1,1,0]");
    const int nrm = b.floats({0, 0, 1, 0, 0, 1, 0, 0, 1}, 3, "VEC3");
    const int uv = b.floats({0, 0, 1, 0, 0, 1}, 2, "VEC2");
    const int i16 = b.indices<std::uint16_t>({0, 1, 2}, 5123);
    // 正方形在 glTF 的地面上（y = 0），逆时针朝 +Y
    const int qpos = b.floats({0, 0, 0, 1, 0, 0, 1, 0, -1, 0, 0, -1}, 3, "VEC3", ",\"min\":[0,0,-1],\"max\":[1,0,0]");
    const int quv = b.floats({0, 0, 1, 0, 1, 1, 0, 1}, 2, "VEC2");
    const int i8 = b.indices<std::uint8_t>({0, 1, 2, 0, 2, 3}, 5121);
    const int i32 = b.indices<std::uint32_t>({0, 1, 2}, 5125);
    const auto n = [](int v) { return std::to_string(v); };
    s.meshes = "{\"name\":\"tri16\",\"primitives\":[{\"attributes\":{\"POSITION\":" + n(pos) + ",\"NORMAL\":" + n(nrm) +
               ",\"TEXCOORD_0\":" + n(uv) + "},\"indices\":" + n(i16) + ",\"material\":0}]}," +
               "{\"name\":\"quad8\",\"primitives\":[{\"attributes\":{\"POSITION\":" + n(qpos) + ",\"TEXCOORD_0\":" + n(quv) +
               "},\"indices\":" + n(i8) + "}]}," + "{\"name\":\"tri32\",\"primitives\":[{\"attributes\":{\"POSITION\":" + n(pos) +
               ",\"NORMAL\":" + n(nrm) + "},\"indices\":" + n(i32) + "}]}";
    return s;
}

std::string gltfJson(const Scene& s, const std::string& bufferUri, std::size_t bufferLength, const std::string& extensions) {
    return "{\"asset\":{\"version\":\"2.0\"}," + extensions + "\"scene\":0,\"scenes\":[{\"nodes\":[0,2,3]}]," +
           "\"nodes\":[{\"name\":\"parent\",\"translation\":[1,2,3],\"children\":[1]}," +
           "{\"name\":\"child\",\"rotation\":[0,0.70710678,0,0.70710678],\"scale\":[2,2,2],\"mesh\":0}," +
           "{\"name\":\"flat\",\"mesh\":1},{\"name\":\"big\",\"mesh\":2}]," + "\"meshes\":[" + s.meshes + "]," +
           "\"materials\":[{\"name\":\"painted\",\"doubleSided\":true,\"emissiveFactor\":[1,1,1]," +
           "\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.5,0.6,0.7,1],\"metallicFactor\":0.3,\"roughnessFactor\":0.4," +
           "\"baseColorTexture\":{\"index\":0,\"extensions\":{\"KHR_texture_transform\":{\"offset\":[0.5,0]}}}," +
           "\"metallicRoughnessTexture\":{\"index\":1}}," + "\"normalTexture\":{\"index\":2,\"scale\":0.8}," +
           "\"occlusionTexture\":{\"index\":3,\"strength\":0.5},\"emissiveTexture\":{\"index\":4}}]," +
           "\"textures\":[{\"source\":0},{\"source\":1},{\"source\":2},{\"source\":3},{\"source\":4}]," +
           "\"images\":[{\"uri\":\"base.png\"},{\"uri\":\"mr.png\"},{\"uri\":\"nor.png\"},{\"uri\":\"ao.png\"},{\"uri\":\"missing.png\"}]," +
           "\"buffers\":[{\"uri\":\"" + bufferUri + "\",\"byteLength\":" + std::to_string(bufferLength) + "}]," +
           "\"bufferViews\":[" + Bin::join(s.bin.views) + "],\"accessors\":[" + Bin::join(s.bin.accessors) + "]}";
}

// 列主序矩阵乘点
std::array<float, 3> apply(const r::Matrix4& m, float x, float y, float z) {
    return {m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13], m[2] * x + m[6] * y + m[10] * z + m[14]};
}

void testScene(const fs::path& dir) {
    const Scene s = buildScene();
    writeBytes(dir / "scene.bin", s.bin.bytes);
    const std::vector<std::uint8_t> png(std::begin(kPng), std::end(kPng));
    for (const char* name : {"base.png", "mr.png", "nor.png", "ao.png"}) writeBytes(dir / name, png);
    writeText(dir / "scene.gltf", gltfJson(s, "scene.bin", s.bin.bytes.size(), "\"extensionsUsed\":[\"KHR_texture_transform\"],"));

    const auto result = a::loadGltf((dir / "scene.gltf").string());
    check(result.asset.has_value() && !result.hasErrors(), "生成的场景加载成功\n" + result.summary());
    if (!result.asset) return;
    const a::Asset& asset = *result.asset;
    using S = a::Diagnostic::Severity;
    check(mentions(result, S::Warning, "KHR_texture_transform"), "extensionsUsed 中不支持的扩展：警告");
    check(mentions(result, S::Warning, "纹理坐标变换"), "纹理坐标变换：警告");
    check(mentions(result, S::Warning, "missing.png") && mentions(result, S::Warning, (dir / "missing.png").string()),
          "缺失贴图：警告中给出 URI 与解析后的路径");
    check(mentions(result, S::Warning, "NORMAL"), "缺少法线：警告并生成");

    // 图像：按槽位决定 sRGB；缺失的图像标记 missing
    check(asset.images.size() == 5, "5 个图像（含缺失的一个）");
    if (asset.images.size() == 5) {
        check(asset.images[0].srgb && !asset.images[1].srgb && !asset.images[2].srgb && !asset.images[3].srgb && asset.images[4].srgb,
              "基础色与自发光贴图为 sRGB，金属度-粗糙度、法线、遮蔽贴图为线性");
        check(asset.images[4].missing && !asset.images[0].missing, "缺失的贴图标记为 missing");
        const auto& px = asset.images[0].pixels;
        check(px.width == 2 && px.height == 2 && px.pixels.size() == 16 && px.pixels[0] == 255 && px.pixels[1] == 128 &&
                  px.pixels[2] == 0 && px.pixels[3] == 255,
              "PNG 解码为 2 × 2 RGBA (255, 128, 0, 255)");
    }
    // 材质
    check(asset.materials.size() == 1, "1 个材质");
    if (!asset.materials.empty()) {
        const a::AssetMaterial& m = asset.materials[0];
        check(m.name == "painted" && near(m.factors.baseColor.x, 0.5) && near(m.factors.baseColor.y, 0.6) &&
                  near(m.factors.baseColor.z, 0.7) && near(m.factors.metallic, 0.3) && near(m.factors.roughness, 0.4),
              "材质因子：基础色 (0.5, 0.6, 0.7)、金属度 0.3、粗糙度 0.4");
        check(m.baseColorImage == 0 && m.metallicRoughnessImage == 1 && m.normalImage == 2 && m.occlusionImage == 3 &&
                  m.emissiveImage == -1,
              "贴图槽位对应正确，缺失的自发光贴图为 -1");
        check(near(m.factors.normalScale, 0.8) && near(m.factors.occlusionStrength, 0.5) && m.factors.doubleSided &&
                  near(m.factors.emissive.x, 1.0),
              "法线缩放 0.8、遮蔽强度 0.5、双面、自发光因子 1");
    }
    // 网格与索引
    check(asset.meshes.size() == 3 && asset.instances.size() == 3, "3 个网格、3 个实例");
    if (asset.meshes.size() == 3) {
        const auto& quad = asset.meshes[1].primitives;
        check(quad.size() == 1 && quad[0].mesh.indices.size() == 6 && quad[0].material == -1, "8 位索引的正方形：6 个索引，无材质");
        if (!quad.empty()) {
            const auto& v0 = quad[0].mesh.vertices[0];
            check(near(v0.normal[0], 0.0) && near(v0.normal[1], 1.0) && near(v0.normal[2], 0.0), "缺失的法线按面积加权生成（朝 glTF +Y）");
        }
        const auto& tri32 = asset.meshes[2].primitives;
        check(tri32.size() == 1 && tri32[0].mesh.indices.size() == 3 && tri32[0].mesh.indices[2] == 2, "32 位索引");
        const auto& tri = asset.meshes[0].primitives;
        check(!tri.empty() && (tri[0].mesh.vertices[0].tangent[3] == 1.0f || tri[0].mesh.vertices[0].tangent[3] == -1.0f),
              "缺少切线时已生成（手性 ±1）");
    }
    check(asset.triangleCount() == 4, "共 4 个三角形");
    // 变换：子节点 (1, 0, 0) → 缩放 2 → 绕 glTF Y 轴 +90° → 平移 (1, 2, 3) → Y→Z：(1, −1, 2)
    for (const auto& inst : asset.instances) {
        if (inst.node == "child") {
            const auto p = apply(inst.transform, 1.0f, 0.0f, 0.0f);
            check(near(p[0], 1.0, 1e-4) && near(p[1], -1.0, 1e-4) && near(p[2], 2.0, 1e-4),
                  "子节点变换与 Y→Z 换算：(1, 0, 0) → (1, −1, 2)，实际 (" + std::to_string(p[0]) + ", " + std::to_string(p[1]) + ", " +
                      std::to_string(p[2]) + ")");
        }
        if (inst.node == "flat") {
            const auto p = apply(inst.transform, 1.0f, 0.0f, -1.0f);
            const auto up = apply(inst.transform, 0.0f, 1.0f, 0.0f);
            check(near(p[0], 1.0) && near(p[1], 1.0) && near(p[2], 0.0) && near(up[2], 1.0), "glTF (1, 0, −1) → Terra (1, 1, 0)，glTF 的 +Y → Terra 的 +Z");
        }
    }
    check(asset.boundsMax[2] >= 2.0f - 1e-4f, "包围盒（Terra 坐标）包含子节点的点");
}

void testErrors(const fs::path& dir) {
    using S = a::Diagnostic::Severity;
    const Scene s = buildScene();
    writeText(dir / "required.gltf",
              gltfJson(s, "scene.bin", s.bin.bytes.size(),
                       "\"extensionsUsed\":[\"KHR_draco_mesh_compression\"],\"extensionsRequired\":[\"KHR_draco_mesh_compression\"],"));
    const auto required = a::loadGltf((dir / "required.gltf").string());
    check(!required.asset && mentions(required, S::Error, "KHR_draco_mesh_compression"), "extensionsRequired 中不支持的扩展：错误，不加载");

    writeText(dir / "nobin.gltf", gltfJson(s, "nope.bin", s.bin.bytes.size(), ""));
    const auto nobin = a::loadGltf((dir / "nobin.gltf").string());
    check(!nobin.asset && mentions(nobin, S::Error, "nope.bin") && mentions(nobin, S::Error, (dir / "nope.bin").string()),
          "缓冲区文件缺失：错误中给出 URI 与解析后的路径\n" + nobin.summary());

    writeText(dir / "bad.gltf", "{ \"asset\": { \"version\": ");
    const auto bad = a::loadGltf((dir / "bad.gltf").string());
    check(!bad.asset && mentions(bad, S::Error, "JSON"), "损坏的 JSON：错误\n" + bad.summary());

    const auto none = a::loadGltf((dir / "no_such.gltf").string());
    check(!none.asset && mentions(none, S::Error, "glTF 文件不存在"), "文件不存在：错误");
}

// .glb：缓冲区与 PNG 都内嵌
void testGlb(const fs::path& dir) {
    Bin b;
    const int pos = b.floats({0, 0, 0, 1, 0, 0, 0, 1, 0}, 3, "VEC3", ",\"min\":[0,0,0],\"max\":[1,1,0]");
    const int uv = b.floats({0, 0, 1, 0, 0, 1}, 2, "VEC2");
    const int idx = b.indices<std::uint16_t>({0, 1, 2}, 5123);
    const int imageView = b.view(kPng, sizeof(kPng));
    while (b.bytes.size() % 4 != 0) b.bytes.push_back(0);
    const auto n = [](int v) { return std::to_string(v); };
    std::string json = "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}]," +
                       std::string{"\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":"} + n(pos) + ",\"TEXCOORD_0\":" + n(uv) +
                       "},\"indices\":" + n(idx) + ",\"material\":0}]}]," +
                       "\"materials\":[{\"pbrMetallicRoughness\":{\"baseColorTexture\":{\"index\":0}}}]," +
                       "\"textures\":[{\"source\":0}],\"images\":[{\"bufferView\":" + n(imageView) + ",\"mimeType\":\"image/png\"}]," +
                       "\"buffers\":[{\"byteLength\":" + std::to_string(b.bytes.size()) + "}],\"bufferViews\":[" + Bin::join(b.views) +
                       "],\"accessors\":[" + Bin::join(b.accessors) + "]}";
    while (json.size() % 4 != 0) json += ' ';
    std::vector<std::uint8_t> glb;
    const auto u32 = [&](std::uint32_t v) {
        for (int k = 0; k < 4; ++k) glb.push_back(static_cast<std::uint8_t>((v >> (8 * k)) & 0xFFu));
    };
    const auto total = static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + b.bytes.size());
    u32(0x46546C67u);  // "glTF"
    u32(2);
    u32(total);
    u32(static_cast<std::uint32_t>(json.size()));
    u32(0x4E4F534Au);  // "JSON"
    glb.insert(glb.end(), json.begin(), json.end());
    u32(static_cast<std::uint32_t>(b.bytes.size()));
    u32(0x004E4942u);  // "BIN\0"
    glb.insert(glb.end(), b.bytes.begin(), b.bytes.end());
    writeBytes(dir / "embedded.glb", glb);

    const auto result = a::loadGltf((dir / "embedded.glb").string());
    check(result.asset.has_value() && !result.hasErrors(), ".glb 加载成功\n" + result.summary());
    if (!result.asset) return;
    const auto& images = result.asset->images;
    check(images.size() == 1 && images[0].srgb && !images[0].missing && images[0].pixels.width == 2 &&
              images[0].path.find("#images[0]") != std::string::npos,
          ".glb 内嵌 PNG 解码成功（sRGB，路径标注为内嵌）");
    check(result.asset->instances.size() == 1 && result.asset->triangleCount() == 1, ".glb：1 个实例、1 个三角形");
}

// ARM 打包约定（M1-10）：金属度-粗糙度贴图文件名含 "_arm" 且材质没有遮蔽贴图时，推断 R 通道为环境遮蔽。
void testArm(const fs::path& dir) {
    check(a::looksLikeArmTexture("textures/metal_toolbox_arm_2k.jpg") && a::looksLikeArmTexture("C:/x/FOO_ARM.PNG"),
          "文件名含 _arm（不区分大小写）：ARM 约定");
    check(!a::looksLikeArmTexture("armchair_diff.jpg") && !a::looksLikeArmTexture("textures_arm/base.png"),
          "只看文件名本身，且必须是 \"_arm\"");
    Bin b;
    const int pos = b.floats({0, 0, 0, 1, 0, 0, 0, 1, 0}, 3, "VEC3", ",\"min\":[0,0,0],\"max\":[1,1,0]");
    const int uv = b.floats({0, 0, 1, 0, 0, 1}, 2, "VEC2");
    writeBytes(dir / "arm.bin", b.bytes);
    writeBytes(dir / "thing_arm_1k.png", std::vector<std::uint8_t>(std::begin(kPng), std::end(kPng)));
    const auto n = [](int v) { return std::to_string(v); };
    const std::string json = "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}],"
                             "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":" + n(pos) + ",\"TEXCOORD_0\":" + n(uv) +
                             "},\"material\":0}]}],\"materials\":[{\"name\":\"arm\",\"pbrMetallicRoughness\":"
                             "{\"metallicRoughnessTexture\":{\"index\":0}}}],\"textures\":[{\"source\":0}],"
                             "\"images\":[{\"uri\":\"thing_arm_1k.png\"}],\"buffers\":[{\"uri\":\"arm.bin\",\"byteLength\":" +
                             std::to_string(b.bytes.size()) + "}],\"bufferViews\":[" + Bin::join(b.views) + "],\"accessors\":[" +
                             Bin::join(b.accessors) + "]}";
    writeText(dir / "arm.gltf", json);
    const auto on = a::loadGltf((dir / "arm.gltf").string());
    check(on.asset.has_value() && !on.hasErrors(), "ARM 场景加载成功\n" + on.summary());
    if (on.asset && !on.asset->materials.empty()) {
        const auto& m = on.asset->materials[0];
        check(m.occlusionFromArm && m.occlusionImage == m.metallicRoughnessImage && m.metallicRoughnessImage == 0 &&
                  mentions(on, a::Diagnostic::Severity::Info, "ARM"),
              "推断：遮蔽贴图 = 金属度-粗糙度贴图，并给出信息级诊断");
    }
    a::LoadOptions off;
    off.armOcclusion = false;
    const auto plain = a::loadGltf((dir / "arm.gltf").string(), off);
    check(plain.asset && !plain.asset->materials.empty() && plain.asset->materials[0].occlusionImage == -1 &&
              !plain.asset->materials[0].occlusionFromArm,
          "关闭推断时不使用遮蔽贴图");
    // 场景测试中的 mr.png 不符合约定：不推断
    check(!a::looksLikeArmTexture("mr.png"), "普通文件名不推断");
}

void testRealAssets() {
    struct Expect {
        const char* name;
        std::size_t instances;
        float height;
    };
    for (const Expect& e : {Expect{"metal_toolbox", 6, 0.0f}, Expect{"Drill_01", 1, 0.185f}, Expect{"brass_vase_01", 1, 0.693f}}) {
        const std::string path = std::string{TERRA_ASSETS_DIR} + "/models/" + e.name + "/" + e.name + "_2k.gltf";
        if (!fs::exists(path)) {
            std::printf("INFO 未找到 %s（未运行 scripts/assets/fetch_models.sh），跳过\n", path.c_str());
            continue;
        }
        const auto result = a::loadGltf(path);
        check(result.asset.has_value() && !result.hasErrors(), std::string{e.name} + "：加载无错误\n" + result.summary());
        if (!result.asset) continue;
        const a::Asset& asset = *result.asset;
        bool imagesOk = asset.images.size() == 3;
        for (const auto& img : asset.images) imagesOk = imagesOk && !img.missing && img.pixels.width == 2048;
        check(imagesOk, std::string{e.name} + "：3 张 2048 × 2048 贴图全部读取");
        check(asset.instances.size() == e.instances, std::string{e.name} + "：实例数 " + std::to_string(asset.instances.size()));
        check(!asset.materials.empty() && asset.materials[0].occlusionFromArm, std::string{e.name} + "：按 ARM 约定推断出遮蔽贴图");
        const float h = asset.boundsMax[2] - asset.boundsMin[2];
        if (e.height > 0.0f) check(std::fabs(h - e.height) < 0.01f, std::string{e.name} + "：Z 向上后高度 " + std::to_string(h) + " m");
        std::printf("INFO %s：%zu 个实例、%zu 个三角形、%zu 张贴图、包围盒 Z [%.3f, %.3f]、诊断 %zu 条\n", e.name, asset.instances.size(),
                    asset.triangleCount(), asset.images.size(), static_cast<double>(asset.boundsMin[2]),
                    static_cast<double>(asset.boundsMax[2]), result.diagnostics.size());
        for (const auto& d : result.diagnostics) std::printf("INFO   %s\n", d.message.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法：test_asset_gltf <临时目录>\n");
        return 2;
    }
    const fs::path dir{argv[1]};
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);
    testScene(dir);
    testErrors(dir);
    testGlb(dir);
    testArm(dir);
    testRealAssets();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
