#include "mesh.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>

#include "process.hpp"
#include "tinyxml2.h"

namespace terra::importer::detail {

namespace fs = std::filesystem;

void Bounds::add(double x, double y, double z) {
    const double p[3] = {x, y, z};
    for (std::size_t k = 0; k < 3; ++k) {
        if (!valid || p[k] < min[k]) min[k] = p[k];
        if (!valid || p[k] > max[k]) max[k] = p[k];
    }
    valid = true;
}

double Bounds::extent() const {
    if (!valid) return 0.0;
    return std::max({max[0] - min[0], max[1] - min[1], max[2] - min[2]});
}

Bounds LoadedMesh::bounds() const {
    Bounds b;
    for (const auto& part : parts) {
        for (const auto& p : part.positions) b.add(p[0], p[1], p[2]);
    }
    return b;
}

std::optional<Bounds> stlBounds(const std::string& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "无法读取 " + path;
        return std::nullopt;
    }
    std::vector<char> data{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    Bounds b;
    // 二进制 STL：80 字节头 + 三角形数 + 每个三角形 50 字节；大小吻合时按二进制读，否则按文本读
    if (data.size() >= 84) {
        std::uint32_t count = 0;
        std::memcpy(&count, data.data() + 80, 4);
        if (84 + static_cast<std::size_t>(count) * 50 == data.size()) {
            for (std::size_t t = 0; t < count; ++t) {
                const char* tri = data.data() + 84 + t * 50 + 12;
                for (std::size_t v = 0; v < 3; ++v) {
                    float p[3];
                    std::memcpy(p, tri + v * 12, 12);
                    b.add(p[0], p[1], p[2]);
                }
            }
            if (!b.valid) error = "STL 中没有三角形：" + path;
            return b.valid ? std::optional<Bounds>(b) : std::nullopt;
        }
    }
    std::istringstream text(std::string(data.begin(), data.end()));
    std::string word;
    while (text >> word) {
        if (word != "vertex") continue;
        double x = 0, y = 0, z = 0;
        if (text >> x >> y >> z) b.add(x, y, z);
    }
    if (!b.valid) {
        error = "不是有效的 STL：" + path;
        return std::nullopt;
    }
    return b;
}

namespace {

struct MtlMaterial {
    std::array<double, 4> rgba{0.7, 0.7, 0.7, 1.0};
    std::string texture;
};

std::map<std::string, MtlMaterial> readMtl(const fs::path& path) {
    std::map<std::string, MtlMaterial> out;
    std::ifstream in(path);
    std::string line;
    MtlMaterial* current = nullptr;
    while (std::getline(in, line)) {
        std::istringstream s(line);
        std::string key;
        s >> key;
        if (key == "newmtl") {
            std::string name;
            std::getline(s >> std::ws, name);
            current = &out[name];
        } else if (current != nullptr && key == "Kd") {
            s >> current->rgba[0] >> current->rgba[1] >> current->rgba[2];
        } else if (current != nullptr && key == "d") {
            s >> current->rgba[3];
        } else if (current != nullptr && key == "Tr") {
            double tr = 0.0;
            if (s >> tr) current->rgba[3] = 1.0 - tr;
        } else if (current != nullptr && key == "map_Kd") {
            std::string file;
            std::getline(s >> std::ws, file);
            // 选项（如 -s 1 1）之后的最后一项是文件名
            const auto space = file.find_last_of(' ');
            if (space != std::string::npos && !file.empty() && file[0] == '-') file = file.substr(space + 1);
            const fs::path p = fs::path{file}.is_absolute() ? fs::path{file} : path.parent_path() / file;
            current->texture = p.lexically_normal().string();
        }
    }
    return out;
}

// OBJ 的索引：正数从 1 开始；负数从末尾倒数
long resolveIndex(long index, std::size_t count) {
    if (index > 0) return index - 1;
    if (index < 0) return static_cast<long>(count) + index;
    return -1;
}

}  // namespace

std::optional<LoadedMesh> readObj(const std::string& path, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "无法读取 " + path;
        return std::nullopt;
    }
    std::vector<std::array<double, 3>> v;
    std::vector<std::array<double, 2>> vt;
    std::map<std::string, MtlMaterial> materials;
    std::map<std::string, std::size_t> partIndex;
    LoadedMesh mesh;
    std::size_t current = 0;
    const auto usePart = [&](const std::string& name) {
        auto it = partIndex.find(name);
        if (it == partIndex.end()) {
            MeshPart part;
            part.material = name;
            const auto m = materials.find(name);
            if (m != materials.end()) {
                part.rgba = m->second.rgba;
                part.texture = m->second.texture;
            }
            mesh.parts.push_back(part);
            it = partIndex.emplace(name, mesh.parts.size() - 1).first;
        }
        current = it->second;
    };
    usePart("");
    bool anyTexcoord = false;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream s(line);
        std::string key;
        s >> key;
        if (key == "v") {
            std::array<double, 3> p{};
            s >> p[0] >> p[1] >> p[2];
            v.push_back(p);
        } else if (key == "vt") {
            std::array<double, 2> t{};
            s >> t[0] >> t[1];
            vt.push_back(t);
        } else if (key == "mtllib") {
            std::string file;
            std::getline(s >> std::ws, file);
            const auto loaded = readMtl(fs::path{path}.parent_path() / file);
            materials.insert(loaded.begin(), loaded.end());
        } else if (key == "usemtl") {
            std::string name;
            std::getline(s >> std::ws, name);
            usePart(name);
        } else if (key == "f") {
            std::vector<std::pair<long, long>> corners;
            std::string token;
            while (s >> token) {
                const auto slash = token.find('/');
                const long vi = resolveIndex(std::strtol(token.c_str(), nullptr, 10), v.size());
                long ti = -1;
                if (slash != std::string::npos && slash + 1 < token.size() && token[slash + 1] != '/') {
                    ti = resolveIndex(std::strtol(token.c_str() + slash + 1, nullptr, 10), vt.size());
                }
                corners.emplace_back(vi, ti);
            }
            for (std::size_t k = 1; k + 1 < corners.size(); ++k) {  // 多边形按扇形分成三角形
                for (const auto& c : {corners[0], corners[k], corners[k + 1]}) {
                    if (c.first < 0 || static_cast<std::size_t>(c.first) >= v.size()) {
                        error = "OBJ 的面引用了不存在的顶点：" + path;
                        return std::nullopt;
                    }
                    auto& part = mesh.parts[current];
                    part.positions.push_back(v[static_cast<std::size_t>(c.first)]);
                    const bool hasTex = c.second >= 0 && static_cast<std::size_t>(c.second) < vt.size();
                    part.texcoords.push_back(hasTex ? vt[static_cast<std::size_t>(c.second)] : std::array<double, 2>{0.0, 0.0});
                    anyTexcoord = anyTexcoord || hasTex;
                }
            }
        }
    }
    mesh.parts.erase(std::remove_if(mesh.parts.begin(), mesh.parts.end(), [](const MeshPart& p) { return p.positions.empty(); }),
                     mesh.parts.end());
    if (!anyTexcoord) {
        for (auto& p : mesh.parts) p.texcoords.clear();
    }
    if (mesh.parts.empty()) {
        error = "OBJ 中没有三角形：" + path;
        return std::nullopt;
    }
    return mesh;
}

bool writeObj(const std::string& path, const MeshPart& part, bool withTexcoords) {
    std::ofstream out(path);
    if (!out) return false;
    out.precision(9);
    out << "# 由 Terra 导入时生成（M4-08a）\n";
    for (const auto& p : part.positions) out << "v " << p[0] << ' ' << p[1] << ' ' << p[2] << '\n';
    const bool tex = withTexcoords && part.texcoords.size() == part.positions.size();
    if (tex) {
        for (const auto& t : part.texcoords) out << "vt " << t[0] << ' ' << t[1] << '\n';
    }
    for (std::size_t i = 0; i + 2 < part.positions.size(); i += 3) {
        if (tex) out << "f " << i + 1 << '/' << i + 1 << ' ' << i + 2 << '/' << i + 2 << ' ' << i + 3 << '/' << i + 3 << '\n';
        else out << "f " << i + 1 << ' ' << i + 2 << ' ' << i + 3 << '\n';
    }
    return static_cast<bool>(out);
}

std::optional<ColladaInfo> readColladaInfo(const std::string& path, std::string& error) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        error = "Collada 文件无法解析：" + path + "（" + doc.ErrorStr() + "）";
        return std::nullopt;
    }
    const auto* root = doc.FirstChildElement("COLLADA");
    if (root == nullptr) {
        error = "不是 Collada 文件：" + path;
        return std::nullopt;
    }
    ColladaInfo info;
    if (const auto* asset = root->FirstChildElement("asset")) {
        if (const auto* unit = asset->FirstChildElement("unit")) unit->QueryDoubleAttribute("meter", &info.unit);
        if (const auto* up = asset->FirstChildElement("up_axis"); up != nullptr && up->GetText() != nullptr) info.upAxis = up->GetText();
    }
    // 各几何体中 <vertices> 的 POSITION 所引用的 <source>
    const auto* geometries = root->FirstChildElement("library_geometries");
    for (const auto* g = geometries ? geometries->FirstChildElement("geometry") : nullptr; g != nullptr; g = g->NextSiblingElement("geometry")) {
        const auto* mesh = g->FirstChildElement("mesh");
        if (mesh == nullptr) continue;
        std::string positionSource;
        for (const auto* vtx = mesh->FirstChildElement("vertices"); vtx != nullptr; vtx = vtx->NextSiblingElement("vertices")) {
            for (const auto* input = vtx->FirstChildElement("input"); input != nullptr; input = input->NextSiblingElement("input")) {
                const char* semantic = input->Attribute("semantic");
                const char* source = input->Attribute("source");
                if (semantic != nullptr && source != nullptr && std::strcmp(semantic, "POSITION") == 0 && source[0] == '#') positionSource = source + 1;
            }
        }
        for (const auto* src = mesh->FirstChildElement("source"); src != nullptr; src = src->NextSiblingElement("source")) {
            const char* id = src->Attribute("id");
            if (id == nullptr || positionSource != id) continue;
            const auto* array = src->FirstChildElement("float_array");
            if (array == nullptr || array->GetText() == nullptr) continue;
            std::istringstream values(array->GetText());
            double x = 0, y = 0, z = 0;
            while (values >> x >> y >> z) info.raw.add(x, y, z);
        }
    }
    return info;
}

namespace {

// 原始顶点包围盒经候选变换后的包围盒
Bounds transformBounds(const Bounds& raw, bool rotate, double scale) {
    Bounds b;
    for (int cx = 0; cx < 2; ++cx) {
        for (int cy = 0; cy < 2; ++cy) {
            for (int cz = 0; cz < 2; ++cz) {
                const double x = cx ? raw.max[0] : raw.min[0];
                const double y = cy ? raw.max[1] : raw.min[1];
                const double z = cz ? raw.max[2] : raw.min[2];
                if (rotate) b.add(x * scale, z * scale, -y * scale);  // assimp 把 Z_UP 转为 Y_UP：(x, y, z) → (x, z, −y)
                else b.add(x * scale, y * scale, z * scale);
            }
        }
    }
    return b;
}

double mismatch(const Bounds& a, const Bounds& b) {
    const double size = std::max({a.extent(), b.extent(), 1e-12});
    double worst = 0.0;
    for (std::size_t k = 0; k < 3; ++k) {
        worst = std::max({worst, std::fabs(a.min[k] - b.min[k]), std::fabs(a.max[k] - b.max[k])});
    }
    return worst / size;
}

}  // namespace

ConversionResult convertWithAssimp(const std::string& assimp, const std::string& source, const std::string& workDir) {
    ConversionResult out;
    std::error_code ec;
    fs::create_directories(workDir, ec);
    const std::string stem = fs::path{source}.stem().string();
    const std::string obj = (fs::path{workDir} / (stem + ".obj")).string();
    const auto run = runProcess({assimp, "export", source, obj});
    if (!run.started) {
        out.error = run.errors;
        return out;
    }
    if (run.exitCode != 0 || !fs::exists(obj)) {
        out.error = "assimp 转换失败（退出码 " + std::to_string(run.exitCode) + "）：" + run.errors.substr(0, 400);
        return out;
    }
    std::string error;
    auto mesh = readObj(obj, error);
    if (!mesh) {
        out.error = error;
        return out;
    }
    // 贴图：MTL 中的路径是相对 Collada 文件写的（assimp 原样抄过来），按 Collada 所在目录重新解析
    for (auto& part : mesh->parts) {
        if (part.texture.empty() || fs::exists(part.texture, ec)) continue;
        const fs::path relative = fs::path{part.texture}.lexically_relative(workDir);
        const fs::path sourceDir = fs::path{source}.parent_path();
        for (const fs::path& candidate : {sourceDir / relative, sourceDir / fs::path{part.texture}.filename()}) {
            if (!relative.empty() && fs::exists(candidate, ec)) {
                part.texture = candidate.lexically_normal().string();
                break;
            }
        }
    }
    std::string ext = fs::path{source}.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext != ".dae") {
        out.transform = "非 Collada：按 assimp 的输出使用";
        out.mesh = std::move(mesh);
        return out;
    }
    const auto info = readColladaInfo(source, error);
    if (!info || !info->raw.valid) {
        out.warning = "无法读取 Collada 的原始顶点，网格方向未核对";
        out.mesh = std::move(mesh);
        return out;
    }
    // 识别 assimp 做了哪种变换：是否按 up_axis 旋转、是否换算了单位
    const Bounds converted = mesh->bounds();
    struct Candidate {
        bool rotate;
        double scale;
        const char* text;
    };
    const Candidate candidates[] = {{false, info->unit, "未旋转，已换算单位"},
                                    {false, 1.0, "未旋转，未换算单位"},
                                    {true, info->unit, "按 Z_UP 旋转为 Y_UP，已换算单位"},
                                    {true, 1.0, "按 Z_UP 旋转为 Y_UP，未换算单位"}};
    const Candidate* best = nullptr;
    double bestError = 1e9;
    for (const auto& c : candidates) {
        const double e = mismatch(transformBounds(info->raw, c.rotate, c.scale), converted);
        if (e < bestError) {
            bestError = e;
            best = &c;
        }
    }
    bool rotated = info->upAxis == "Z_UP";
    double applied = info->unit;
    if (best != nullptr && bestError < 1e-3) {
        rotated = best->rotate;
        applied = best->scale;
        out.transform = best->text;
    } else {
        out.transform = std::string{"无法核对（文件中有节点变换），按 assimp 的默认处理换算"};
        out.warning = "网格 " + fs::path{source}.filename().string() + " 的方向与单位无法与原始顶点核对（文件中有节点变换），请在场景中目视检查";
    }
    // 换算到“文件坐标轴、米”：撤销旋转 (x', y', z') → (x', −z', y')，补上未换算的单位
    const double extra = applied == info->unit ? 1.0 : info->unit;
    for (auto& part : mesh->parts) {
        for (auto& p : part.positions) {
            std::array<double, 3> q = p;
            if (rotated) q = {p[0], -p[2], p[1]};
            p = {q[0] * extra, q[1] * extra, q[2] * extra};
        }
    }
    out.mesh = std::move(mesh);
    return out;
}

}  // namespace terra::importer::detail
