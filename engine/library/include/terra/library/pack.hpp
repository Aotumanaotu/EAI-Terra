#pragma once

// 资产包（M4-08c；M4-08 设计稿 8.3 节）：把一个导入目录（模型、网格、贴图、场景、配方、报告与许可文件）打包成一个 .terra-pack 文件，
// 发给别人后解包即可使用，不需要重新转换。
//
// 格式：zip。根目录下的 manifest.json 记录格式、名字与每个文件的相对路径、大小和 sha256；文件放在 <名字>/ 下。
// 导出时文件按路径排序、时间戳固定，同一目录两次导出的包逐字节相同。不打包隐藏文件与符号链接。
// 解包时：拒绝绝对路径、含 “..” 的路径与 manifest 之外的条目；逐个核对大小与 sha256；先解到输出目录下的隐藏临时目录，
// 全部通过后再改名为 <输出目录>/<名字>（已存在时除非 overwrite，否则报错），中途失败不留下半个目录。

#include <cstddef>
#include <cstdint>
#include <string>

namespace terra::library {

inline constexpr const char* kPackSuffix = ".terra-pack";

struct PackResult {
    bool ok = false;
    std::string error;
    std::string directory;  // 导入：解包后的目录；导出：被打包的目录
    std::string name;       // 包名（目录名）
    std::size_t files = 0;
    std::uint64_t bytes = 0;  // 文件的总大小（解压后）
};

PackResult exportPack(const std::string& directory, const std::string& packFile, bool overwrite = false);
PackResult importPack(const std::string& packFile, const std::string& outputRoot, bool overwrite = false);

}  // namespace terra::library
