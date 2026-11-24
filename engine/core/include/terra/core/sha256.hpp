#pragma once

// SHA-256（FIPS 180-4），M3-07：实验记录中场景与动作文件的内容摘要。纯标准库实现，不依赖外部库。

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace terra {

class Sha256 {
public:
    Sha256() noexcept;
    void update(const void* data, std::size_t size) noexcept;
    void update(std::string_view text) noexcept { update(text.data(), text.size()); }
    // 结束并返回 64 个小写十六进制字符；之后对象不应再使用
    std::string finishHex();

private:
    void block(const std::uint8_t* p) noexcept;
    std::uint32_t h_[8];
    std::uint8_t buffer_[64];
    std::size_t used_ = 0;
    std::uint64_t total_ = 0;  // 字节数
};

std::string sha256Hex(std::string_view data);
// 文件内容的 SHA-256；读不出时返回空
std::optional<std::string> sha256File(const std::string& path);

}  // namespace terra
