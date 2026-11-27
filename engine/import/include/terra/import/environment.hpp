#pragma once

// glTF 环境导入（M5-02b，M5-01 设计稿 2.1 节）：把一个 glTF 场景（如整座变电站）按顶层节点拆成场景元素，
// 自动生成碰撞代理，写出元素文件与引用它们的场景文件。不依赖图形（自带只读的 glTF 读取），Headless 构建中也可用。
//
// 输出目录 <output>/：
//   <名字>.terra-scene.json     场景：models 为空，elements 列出全部元素（放置位姿 = 节点的位姿）
//   elements/<id>.terra-element.json  每个顶层节点一个元素；外观引用原 glTF 文件的该节点
//   elements/meshes/*.obj        凸包模式的碰撞网格
//   import-report.json           导入报告：每个元素的盒体数、部件、警告
//
// 碰撞代理：
//   - boxes（默认）：节点子树中每个网格图元先按共享顶点拆成互不相连的部分（如楼梯的每个踏步），每部分取三种包围盒中
//     体积最小的一个——元素坐标系的轴对齐盒、网格自身坐标系的轴对齐盒、主方向（PCA）盒（斜着的扶手、导线）；
//     一个节点超过 maxBoxesPerNode 个时，把轴对齐盒两两合并（每次取合并后体积增加最少的一对）直到不超过；
//   - hulls：每个网格节点导出一个 OBJ，由 MuJoCo 按凸包碰撞（适合坡道等斜面）。
//   hullPatterns 中的子串出现在类别或 id 中的元素用 hulls（默认 "ramp"）。
//
// 清单（可选）：与 Terra 的 blender/ 资产流程相同的 manifest.json（elements: [{id, name, category, movable, mass,
//   license, thermal: {default, parts: {节点: 温度}}, inspection: [{name, node, view: {distance, heightDeg}}]}]）。
//   不给时在 glTF 所在目录及其上一级找 manifest.json。thermal.parts 中的节点成为元素的部件（单独的刚体、温度与分割 ID）；
//   检查点的位置取节点网格包围盒的中心。没有清单的节点：类别 "environment"、fixed。
//
// 坐标：glTF 为 Y 向上；Terra 为 Z 向上：(x, y, z) → (x, −z, y)。元素坐标系 = 节点的旋转与平移（不含缩放），
// 节点的统一缩放记入外观绑定的 scale 并计入碰撞体的尺寸；非统一缩放取平均并警告。

#include <cstddef>
#include <string>
#include <vector>

namespace terra::importer {

struct EnvironmentImportOptions {
    std::string input;     // .glb 或 .gltf
    std::string manifest;  // 空：自动查找
    std::string output;    // 输出目录
    std::string name;      // 场景名（文件名）；空时取 glTF 的文件名
    bool hulls = false;    // 全部用凸包
    std::vector<std::string> hullPatterns{"ramp"};
    int maxBoxesPerNode = 24;
    bool force = false;  // 输出目录中已有场景文件时覆盖
};

struct EnvironmentImportResult {
    bool ok = false;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    std::string scene;     // 场景文件
    std::string manifest;  // 实际使用的清单；空表示没有
    std::size_t elements = 0;
    std::size_t boxes = 0;
    std::size_t meshes = 0;  // 凸包网格数
    std::size_t parts = 0;
    std::size_t inspection = 0;
};

EnvironmentImportResult importEnvironment(const EnvironmentImportOptions& options);

}  // namespace terra::importer
