#pragma once

// MJCF 几何体的显示换算（M3-03）：网格（平滑法线、纹理坐标）与材质（MuJoCo 经典着色参数 → PBR）。
// 纯 CPU，不调用 OpenGL，便于单独测试（render.mjcf_visual）；GPU 资源由 SceneView 创建。
//
// 法线：取 MuJoCo 编译后的法线（MeshDescription::normals，按三角形角索引）。某个角的法线与所在三角形的面法线夹角
// 超过 kCreaseAngleDeg 时改用面法线，使尖锐的棱保持分明（MuJoCo 只去掉了大折角面对平均法线的贡献，个别角仍可能
// 偏得很远）。网格没有法线时退回平直着色（每个三角形用面法线）。
//
// 材质：
//   基础色 = 几何体颜色（sRGB → 线性；已按 MuJoCo 规则合成材质颜色）；有二维贴图时再乘贴图（sRGB）
//   粗糙度 = 材质的 roughness（模型给出时），否则 1 − 0.9 × shininess（MuJoCo 默认 0.5 → 0.55，与 M2 的默认值相同）
//   金属度 = 材质的 metallic（模型给出时），否则 0
//   非金属反射率参数 = specular（MuJoCo 默认 0.5，与 Terra 默认相同）
//   自发光 = emission × 基础色
//   MuJoCo 的 reflectance（平面倒影）不使用；透明度不支持（按不透明显示）。
// 纹理坐标：网格用自带的纹理坐标 × texrepeat；平面的纹理坐标以米为单位，texuniform 时 × texrepeat（每米重复次数），
// 否则按平面尺寸归一化后 × texrepeat；其他基本形状用生成器的纹理坐标 × texrepeat。
// 不支持（只显示颜色，并计数提示）：没有纹理坐标的网格上的二维贴图；立方体贴图。

#include <array>
#include <cstddef>

#include "terra/data/model_description.hpp"
#include "terra/render/mesh.hpp"
#include "terra/render/pbr.hpp"

namespace terra::viewer {

constexpr double kCreaseAngleDeg = 60.0;

// MJCF 网格 → 渲染网格。uvScale 乘到纹理坐标上；没有纹理坐标时 UV 为 0。每个三角形角一个顶点。
terra::render::MeshData mjcfMesh(const terra::data::MeshDescription& mesh, std::array<float, 2> uvScale = {1.0f, 1.0f});

// 纹理坐标整体缩放（基本形状的贴图重复），之后重算切线。
void scaleUv(terra::render::MeshData& data, std::array<float, 2> scale);

struct MjcfSurface {
    terra::render::Material material;  // 不含 GL 贴图名称
    int texture = -1;                  // 要使用的二维贴图（TextureDescription 下标）；-1 表示只用颜色
    std::array<float, 2> uvScale{1.0f, 1.0f};
    bool textureUnsupported = false;   // 材质带贴图但无法显示（立方体贴图，或网格没有纹理坐标）
};

// 几何体的显示材质。geom.material 越界时按没有材质处理。
MjcfSurface mjcfSurface(const terra::data::GeomDescription& geom, const terra::data::ModelDescription& model);

// MJCF 颜色（sRGB 0–1）→ 线性
float srgbToLinear(float c);

}  // namespace terra::viewer
