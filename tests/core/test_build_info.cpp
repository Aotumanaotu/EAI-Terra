// terra_core 构建信息测试：验证 C++17 在库与调用方都生效，版本字段自洽。

#include <cstdio>
#include <string>

#include "terra/core/build_info.hpp"

static_assert(__cplusplus >= 201703L, "测试目标未以 C++17 编译");

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
    const terra::BuildInfo& info = terra::buildInfo();

    check(info.cxxStandard >= 201703L, "terra_core 以 C++17 或更高标准编译");
    check(!info.version.empty(), "版本字符串非空");
    check(!info.compilerId.empty(), "编译器标识非空");
    check(!info.compilerVersion.empty(), "编译器版本非空");

    const std::string expected = std::to_string(info.versionMajor) + "." +
                                 std::to_string(info.versionMinor) + "." +
                                 std::to_string(info.versionPatch);
    check(info.version == expected, "版本字符串与版本号字段一致");
    check(&terra::buildInfo() == &info, "buildInfo() 返回同一实例");

    return failures == 0 ? 0 : 1;
}
