#pragma once

// 网格（M4-08a，内部）：STL 与 OBJ 的读取（包围盒、按材质拆分）、OBJ 写出、Collada 的转换（assimp 命令行工具）。
//
// Collada 的约定与 RViz 相同（rviz_rendering 的 AssimpLoader 设置 AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION）：
// 网格保持文件中的坐标轴（不按 <up_axis> 旋转），长度按 <unit meter> 换算为米。assimp 命令行工具不能关闭“按 up_axis 旋转”，
// 因此转换后与文件中的原始顶点比较包围盒，识别 assimp 做了哪种变换并换算回上述约定；识别不了时给出警告。

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace terra::importer::detail {

struct Bounds {
    std::array<double, 3> min{0.0, 0.0, 0.0};
    std::array<double, 3> max{0.0, 0.0, 0.0};
    bool valid = false;
    void add(double x, double y, double z);
    double extent() const;  // 最长边
};

// 一个材质的三角形（顶点直接展开，不共享）
struct MeshPart {
    std::string material;
    std::array<double, 4> rgba{0.7, 0.7, 0.7, 1.0};
    std::string texture;  // 贴图文件的绝对路径；空表示没有
    std::vector<std::array<double, 3>> positions;  // 3 个一组
    std::vector<std::array<double, 2>> texcoords;  // 与 positions 一一对应，或为空
};

struct LoadedMesh {
    std::vector<MeshPart> parts;
    Bounds bounds() const;
};

std::optional<Bounds> stlBounds(const std::string& path, std::string& error);
std::optional<LoadedMesh> readObj(const std::string& path, std::string& error);
bool writeObj(const std::string& path, const MeshPart& part, bool withTexcoords);

struct ColladaInfo {
    double unit = 1.0;         // <unit meter>
    std::string upAxis = "Y_UP";
    Bounds raw;                // 原始顶点（未经节点变换、未换算单位）的包围盒
};
std::optional<ColladaInfo> readColladaInfo(const std::string& path, std::string& error);

struct ConversionResult {
    std::optional<LoadedMesh> mesh;  // 已换算为“文件坐标轴、米”
    std::string error;
    std::string warning;             // 例如方向无法核对
    std::string transform;           // 识别出的 assimp 变换（报告用）
};

// 用 assimp 把 source 转为 OBJ（写在 workDir 中）并读回、换算
ConversionResult convertWithAssimp(const std::string& assimp, const std::string& source, const std::string& workDir);

}  // namespace terra::importer::detail
