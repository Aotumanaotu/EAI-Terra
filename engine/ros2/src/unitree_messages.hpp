#pragma once

// 宇树消息的 CRC（M4-06b，内部头文件）。与宇树官方示例（unitree_ros2 668d1ec5，BSD-3-Clause）的
// example/src/src/common/motor_crc.cpp、motor_crc_hg.cpp 相同：
//   - 把消息逐字段放进自然对齐的 C 结构体（填充字节为 0），按 32 位字（小端）计算 CRC-32：
//     多项式 0x04C11DB7、初值 0xFFFFFFFF、不反转、不异或输出；最后一个字（crc 本身）不参与；
//   - unitree_hg 的 LowCmd：官方代码只拷贝 reserve 的第一个元素（memcpy 4 字节），其余三个元素按 0 计算，这里照做。
// LowState 的 CRC 按同样的规则（消息字段顺序、自然对齐）计算；真机的 LowState 是否也是这个布局，待真机核对。

#include <cstddef>
#include <cstdint>
#include <vector>

#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

namespace terra::ros2::detail {

// 官方的 crc32_core：words 个 32 位字
std::uint32_t crc32Core(const std::uint32_t* words, std::size_t count);
// bytes 的长度为 4 的倍数；对除最后一个字以外的全部字计算
std::uint32_t crcOfPacked(const std::vector<std::uint8_t>& bytes);

// 消息 → 官方结构体的字节（含末尾的 crc 字，按消息中的值）
std::vector<std::uint8_t> pack(const unitree_go::msg::LowCmd& msg);
std::vector<std::uint8_t> pack(const unitree_go::msg::LowState& msg);
std::vector<std::uint8_t> pack(const unitree_hg::msg::LowCmd& msg);
std::vector<std::uint8_t> pack(const unitree_hg::msg::LowState& msg);

template <class Message>
std::uint32_t crcOf(const Message& msg) {
    return crcOfPacked(pack(msg));
}

}  // namespace terra::ros2::detail
