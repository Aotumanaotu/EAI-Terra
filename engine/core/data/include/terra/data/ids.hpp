#pragma once

#include <cstdint>
#include <functional>

namespace terra::data {

// Terra 运行时标识。值 0 表示无效；有效值由 Terra 分配，不是任何物理后端的内部索引。
// M0：同一次加载内保持稳定；不保证跨进程或重新加载后数值相同。
template <typename Tag>
struct Id {
    std::uint32_t value = 0;

    constexpr bool valid() const noexcept { return value != 0; }

    friend constexpr bool operator==(Id a, Id b) noexcept { return a.value == b.value; }
    friend constexpr bool operator!=(Id a, Id b) noexcept { return a.value != b.value; }
    friend constexpr bool operator<(Id a, Id b) noexcept { return a.value < b.value; }
};

using EntityId = Id<struct EntityTag>;
using JointId = Id<struct JointTag>;

}  // namespace terra::data

template <typename Tag>
struct std::hash<terra::data::Id<Tag>> {
    std::size_t operator()(terra::data::Id<Tag> id) const noexcept {
        return std::hash<std::uint32_t>{}(id.value);
    }
};
