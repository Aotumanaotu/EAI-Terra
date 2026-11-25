// 宇树消息的 CRC 与结构体布局（M4-06b，ros2.unitree_crc；只在找到宇树消息包的 ROS2 构建中）。
//   - crc32Core 的已知值：{0x31323334, 0x35363738} 即 CRC-32/MPEG-2("12345678") = 0x49E3C2FB；{0} = 0xC704DD7B；空 = 0xFFFFFFFF；
//   - 与官方代码逐字节对照：下面的 OfficialGo / OfficialHg 照抄 unitree_ros2（668d1ec5，BSD-3-Clause，Copyright (c) 2020,
//     Unitree Robotics.Co.Ltd.）example/src/include/common/motor_crc.h、motor_crc_hg.h 的结构体与 get_crc 的拷贝方式。
//     随机填满的消息，Terra 的 pack() 与官方结构体的字节完全相同，CRC 也相同；
//   - 结构体大小：go LowCmd 812、hg LowCmd 1004 字节；LowState 按同样规则为 go 1180、hg 2092 字节；
//   - hg LowCmd 的 reserve 只有第一个元素参与 CRC（官方代码只拷贝 4 字节）。

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "unitree_messages.hpp"

namespace {

namespace dt = terra::ros2::detail;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

// ---- 官方结构体（照抄，仅改名以免冲突）----
namespace OfficialGo {
typedef struct {
    uint8_t off;
    std::array<uint8_t, 3> reserve;
} BmsCmd;
typedef struct {
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;
    std::array<uint32_t, 3> reserve;
} MotorCmd;
typedef struct {
    std::array<uint8_t, 2> head;
    uint8_t levelFlag;
    uint8_t frameReserve;
    std::array<uint32_t, 2> SN;
    std::array<uint32_t, 2> version;
    uint16_t bandWidth;
    std::array<MotorCmd, 20> motorCmd;
    BmsCmd bms;
    std::array<uint8_t, 40> wirelessRemote;
    std::array<uint8_t, 12> led;
    std::array<uint8_t, 2> fan;
    uint8_t gpio;
    uint32_t reserve;
    uint32_t crc;
} LowCmd;
}  // namespace OfficialGo

namespace OfficialHg {
typedef struct {
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;
    uint32_t reserve = 0;
} MotorCmd;
typedef struct {
    uint8_t modePr;
    uint8_t modeMachine;
    std::array<MotorCmd, 35> motorCmd;
    std::array<uint32_t, 4> reserve;
    uint32_t crc;
} LowCmd;
}  // namespace OfficialHg

// 官方 get_crc 的拷贝方式（motor_crc.cpp）；结构体先清零（官方为 LowCmd raw{}），使填充字节为 0
std::vector<std::uint8_t> officialBytes(const unitree_go::msg::LowCmd& msg) {
    OfficialGo::LowCmd raw;
    std::memset(static_cast<void*>(&raw), 0, sizeof(raw));
    std::memcpy(&raw.head[0], &msg.head[0], 2);
    raw.levelFlag = msg.level_flag;
    raw.frameReserve = msg.frame_reserve;
    std::memcpy(&raw.SN[0], &msg.sn[0], 8);
    std::memcpy(&raw.version[0], &msg.version[0], 8);
    raw.bandWidth = msg.bandwidth;
    for (std::size_t i = 0; i < 20; i++) {
        raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
        raw.motorCmd[i].q = msg.motor_cmd[i].q;
        raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
        raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
        raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
        raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;
        std::memcpy(&raw.motorCmd[i].reserve[0], &msg.motor_cmd[i].reserve[0], 12);
    }
    raw.bms.off = msg.bms_cmd.off;
    std::memcpy(&raw.bms.reserve[0], &msg.bms_cmd.reserve[0], 3);
    std::memcpy(&raw.wirelessRemote[0], &msg.wireless_remote[0], 40);
    std::memcpy(&raw.led[0], &msg.led[0], 12);
    std::memcpy(&raw.fan[0], &msg.fan[0], 2);
    raw.gpio = msg.gpio;
    raw.reserve = msg.reserve;
    raw.crc = msg.crc;
    std::vector<std::uint8_t> out(sizeof(raw));
    std::memcpy(out.data(), &raw, sizeof(raw));
    return out;
}

// 官方 get_crc 的拷贝方式（motor_crc_hg.cpp）：reserve 只拷贝 4 字节
std::vector<std::uint8_t> officialBytes(const unitree_hg::msg::LowCmd& msg) {
    OfficialHg::LowCmd raw;
    std::memset(static_cast<void*>(&raw), 0, sizeof(raw));
    raw.modePr = msg.mode_pr;
    raw.modeMachine = msg.mode_machine;
    for (std::size_t i = 0; i < 35; i++) {
        raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
        raw.motorCmd[i].q = msg.motor_cmd[i].q;
        raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
        raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
        raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
        raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;
        raw.motorCmd[i].reserve = msg.motor_cmd[i].reserve;
    }
    std::memcpy(&raw.reserve[0], &msg.reserve[0], 4);
    raw.crc = msg.crc;
    std::vector<std::uint8_t> out(sizeof(raw));
    std::memcpy(out.data(), &raw, sizeof(raw));
    return out;
}

std::uint32_t officialCrc(const std::vector<std::uint8_t>& bytes) {
    std::vector<std::uint32_t> words(bytes.size() / 4);
    std::memcpy(words.data(), bytes.data(), words.size() * 4);
    return dt::crc32Core(words.data(), words.size() - 1);
}

std::mt19937 rng(20261124);
std::uint8_t u8() { return static_cast<std::uint8_t>(rng() & 0xFFu); }
std::uint16_t u16() { return static_cast<std::uint16_t>(rng() & 0xFFFFu); }
std::uint32_t u32() { return static_cast<std::uint32_t>(rng()); }
float f32() { return std::uniform_real_distribution<float>(-100.0f, 100.0f)(rng); }

unitree_go::msg::LowCmd randomGo() {
    unitree_go::msg::LowCmd m;
    for (auto& v : m.head) v = u8();
    m.level_flag = u8();
    m.frame_reserve = u8();
    for (auto& v : m.sn) v = u32();
    for (auto& v : m.version) v = u32();
    m.bandwidth = u16();
    for (auto& c : m.motor_cmd) {
        c.mode = u8();
        c.q = f32();
        c.dq = f32();
        c.tau = f32();
        c.kp = f32();
        c.kd = f32();
        for (auto& v : c.reserve) v = u32();
    }
    m.bms_cmd.off = u8();
    for (auto& v : m.bms_cmd.reserve) v = u8();
    for (auto& v : m.wireless_remote) v = u8();
    for (auto& v : m.led) v = u8();
    for (auto& v : m.fan) v = u8();
    m.gpio = u8();
    m.reserve = u32();
    m.crc = u32();
    return m;
}

unitree_hg::msg::LowCmd randomHg() {
    unitree_hg::msg::LowCmd m;
    m.mode_pr = u8();
    m.mode_machine = u8();
    for (auto& c : m.motor_cmd) {
        c.mode = u8();
        c.q = f32();
        c.dq = f32();
        c.tau = f32();
        c.kp = f32();
        c.kd = f32();
        c.reserve = u32();
    }
    for (auto& v : m.reserve) v = u32();
    m.crc = u32();
    return m;
}

void testCore() {
    const std::uint32_t text[] = {0x31323334u, 0x35363738u};
    check(dt::crc32Core(text, 2) == 0x49E3C2FBu, "CRC-32/MPEG-2(\"12345678\") = 0x49E3C2FB");
    const std::uint32_t zero[] = {0u};
    check(dt::crc32Core(zero, 1) == 0xC704DD7Bu, "{0} → 0xC704DD7B");
    check(dt::crc32Core(zero, 0) == 0xFFFFFFFFu, "空 → 初值 0xFFFFFFFF");
}

void testGo() {
    check(sizeof(OfficialGo::LowCmd) == 812, "官方 go LowCmd 结构体 812 字节（" + std::to_string(sizeof(OfficialGo::LowCmd)) + "）");
    for (int round = 0; round < 50; ++round) {
        const auto msg = randomGo();
        const auto ours = dt::pack(msg);
        const auto theirs = officialBytes(msg);
        check(ours == theirs, "go LowCmd 第 " + std::to_string(round) + " 组：pack() 与官方结构体逐字节相同（" +
                                  std::to_string(ours.size()) + " / " + std::to_string(theirs.size()) + " 字节）");
        check(dt::crcOf(msg) == officialCrc(theirs), "go LowCmd 第 " + std::to_string(round) + " 组：CRC 相同");
    }
    // 改任何一个参与计算的字段，CRC 都变；改 crc 字段本身不变
    auto msg = randomGo();
    const auto base = dt::crcOf(msg);
    msg.motor_cmd[7].kp += 1.0f;
    check(dt::crcOf(msg) != base, "go LowCmd：kp 改变后 CRC 改变");
    msg.motor_cmd[7].kp -= 1.0f;
    msg.crc ^= 0xFFFFFFFFu;
    check(dt::crcOf(msg) == base, "go LowCmd：crc 字段不参与计算");
    check(dt::pack(unitree_go::msg::LowState{}).size() == 1180, "go LowState 1180 字节（" +
                                                                    std::to_string(dt::pack(unitree_go::msg::LowState{}).size()) + "）");
}

void testHg() {
    check(sizeof(OfficialHg::LowCmd) == 1004, "官方 hg LowCmd 结构体 1004 字节（" + std::to_string(sizeof(OfficialHg::LowCmd)) + "）");
    for (int round = 0; round < 50; ++round) {
        const auto msg = randomHg();
        const auto ours = dt::pack(msg);
        const auto theirs = officialBytes(msg);
        check(ours == theirs, "hg LowCmd 第 " + std::to_string(round) + " 组：pack() 与官方结构体逐字节相同（" +
                                  std::to_string(ours.size()) + " / " + std::to_string(theirs.size()) + " 字节）");
        check(dt::crcOf(msg) == officialCrc(theirs), "hg LowCmd 第 " + std::to_string(round) + " 组：CRC 相同");
    }
    auto msg = randomHg();
    const auto base = dt::crcOf(msg);
    msg.reserve[1] ^= 0x12345678u;
    msg.reserve[3] ^= 0x1u;
    check(dt::crcOf(msg) == base, "hg LowCmd：reserve[1..3] 不参与 CRC（与官方代码相同）");
    msg.reserve[0] ^= 0x1u;
    check(dt::crcOf(msg) != base, "hg LowCmd：reserve[0] 参与 CRC");
    check(dt::pack(unitree_hg::msg::LowState{}).size() == 2092, "hg LowState 2092 字节（" +
                                                                    std::to_string(dt::pack(unitree_hg::msg::LowState{}).size()) + "）");
}

}  // namespace

int main() {
    testCore();
    testGo();
    testHg();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
