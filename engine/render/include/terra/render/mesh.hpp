#pragma once

// Terra Render：网格数据、程序生成与 GPU 网格（M1-08）。
//
// 坐标约定沿用 D-012：右手系、Z 向上、米。三角形逆时针为正面（法线朝外）。
// 几何体尺寸约定与 MuJoCo 一致：立方体用半边长，圆柱与胶囊沿局部 Z 轴、用半径与半长（胶囊的半长不含两端半球）。
// 网格生成是纯 CPU 计算，不需要 OpenGL 上下文；GpuMesh 需要。

#include <array>
#include <cstdint>
#include <vector>

#include "terra/render/resources.hpp"

namespace terra::render {

class RenderContext;

struct MeshVertex {
    float position[3];
    float normal[3];
    float uv[2];
};

struct MeshData {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;  // 每 3 个一个三角形
    std::array<float, 3> boundsMin{0.0f, 0.0f, 0.0f};
    std::array<float, 3> boundsMax{0.0f, 0.0f, 0.0f};

    std::size_t triangleCount() const noexcept { return indices.size() / 3; }
};

namespace meshes {

// z = 0 平面，法线 +Z；UV 以米为单位（便于以后平铺纹理）。
MeshData plane(float halfX, float halfY);
// 以原点为中心的长方体，每个面 4 个顶点（法线不平滑）。
MeshData box(float halfX, float halfY, float halfZ);
// UV 球：segments 为经线方向分段（≥ 3），rings 为纬线方向分段（≥ 2）。
MeshData sphere(float radius, int segments = 48, int rings = 24);
// 沿 Z 轴的圆柱，带上下底面。
MeshData cylinder(float radius, float halfLength, int segments = 48);
// 沿 Z 轴的胶囊：中间圆柱段半长 halfLength，两端半径为 radius 的半球；ringsPerCap ≥ 1。
MeshData capsule(float radius, float halfLength, int segments = 48, int ringsPerCap = 12);

}  // namespace meshes

// 上传到 GPU 的网格：位置（location 0）、法线（1）、UV（2），32 位索引。
class GpuMesh {
public:
    GpuMesh() = default;
    static GpuMesh create(const MeshData& data);

    void draw(RenderContext& context) const;  // 绑定顶点数组并绘制全部三角形
    int indexCount() const noexcept { return indexCount_; }
    const std::array<float, 3>& boundsMin() const noexcept { return boundsMin_; }
    const std::array<float, 3>& boundsMax() const noexcept { return boundsMax_; }

private:
    Buffer vertices_;
    Buffer indices_;
    VertexArray vao_;
    int indexCount_ = 0;
    std::array<float, 3> boundsMin_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> boundsMax_{0.0f, 0.0f, 0.0f};
};

}  // namespace terra::render
