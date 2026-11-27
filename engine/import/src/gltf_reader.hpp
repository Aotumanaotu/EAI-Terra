#pragma once

// 不依赖图形的 glTF 2.0 读取（M5-02b 的环境导入用）：.glb 与 .gltf（外部 .bin 或 base64 的 data URI）；节点层级与世界矩阵、
// 网格的 POSITION 与索引。只读导入需要的部分，不解析材质、贴图与动画。显示仍由资产模块（cgltf，渲染构建）负责。
// 坐标为 glTF 的（Y 向上），换算到 Terra（Z 向上）由调用方做。

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace terra::importer::gltf {

using Mat4 = std::array<double, 16>;  // 列主序
struct Vec3d {
    double x = 0, y = 0, z = 0;
};

Mat4 identity();
Mat4 multiply(const Mat4& a, const Mat4& b);
Vec3d transformPoint(const Mat4& m, const Vec3d& p);
Mat4 inverseAffine(const Mat4& m);  // 一般仿射（含缩放与旋转）

struct Primitive {
    int positions = -1;  // 访问器下标
    int indices = -1;    // 没有时为 -1（按顺序每三个点一个三角形）
    int mode = 4;        // 只用三角形（4）
};

struct Node {
    std::string name;
    int mesh = -1;
    std::vector<int> children;
    int parent = -1;
    Mat4 local = identity();
    Mat4 world = identity();
};

struct Document {
    std::string path;
    std::vector<Node> nodes;
    std::vector<int> roots;                            // 默认场景的根节点
    std::vector<std::vector<Primitive>> meshes;        // 每个网格的图元
    std::vector<std::array<Vec3d, 2>> accessorBounds;  // POSITION 访问器的 min / max（没有时为空盒）
    // 内部：访问器与缓冲
    struct Accessor {
        int view = -1;
        std::size_t offset = 0;
        std::size_t count = 0;
        int componentType = 0;
        int components = 0;
        bool hasBounds = false;
    };
    struct View {
        int buffer = -1;
        std::size_t offset = 0;
        std::size_t length = 0;
        std::size_t stride = 0;
    };
    std::vector<Accessor> accessors;
    std::vector<View> views;
    std::vector<std::vector<std::uint8_t>> buffers;
};

// 读取；失败时返回 false，error 为原因
bool read(const std::string& path, Document& doc, std::string& error);

// 访问器的数据：float VEC3 位置；索引（u8 / u16 / u32）。越界或类型不对时返回空并写 error
std::vector<Vec3d> positions(const Document& doc, int accessor, std::string& error);
std::vector<std::uint32_t> indices(const Document& doc, int accessor, std::string& error);

// 节点名 → 下标（重名取第一个；没有时为 -1）
int findNode(const Document& doc, const std::string& name);
bool inSubtree(const Document& doc, int node, int root);

}  // namespace terra::importer::gltf
