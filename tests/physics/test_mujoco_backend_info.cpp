// terra_physics_mujoco 后端信息测试：只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。

#include <cstdio>
#include <string_view>

#include "terra/physics/mujoco/backend_info.hpp"

#ifndef TERRA_MUJOCO_EXPECTED_VERSION
#error "需要 TERRA_MUJOCO_EXPECTED_VERSION（由 CMake 根据 mujoco.lock 定义）"
#endif

namespace {

int failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what);
        ++failures;
    }
}

}  // namespace

int main() {
    const auto info = terra::physics::mujoco::backendInfo();

    check(info.name == "MuJoCo", "后端名称");
    check(info.versionsMatch(), "头文件与运行库版本一致");
    check(info.libraryVersionString == std::string_view{TERRA_MUJOCO_EXPECTED_VERSION},
          "运行库版本与 mujoco.lock 一致");

    return failures == 0 ? 0 : 1;
}
