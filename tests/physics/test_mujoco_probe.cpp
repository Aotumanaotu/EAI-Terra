// MuJoCo 依赖探针（M0-02）：只验证库装载、版本一致与所需 C API 可链接。
// 按开发计划，这里不创建任何场景；第一个物理测试是 M0-04/05 的立方体自由落体。
// 本测试是唯一允许直接包含 MuJoCo 头文件的测试，用于在适配层之外独立定位依赖问题。

#include <cstdio>
#include <cstring>

#include <mujoco/mujoco.h>

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

template <typename Fn>
void checkSymbol(Fn* fn, const char* name) {
    if (fn == nullptr) {
        std::fprintf(stderr, "FAILED: 缺少 C API 符号 %s\n", name);
        ++failures;
    }
}

}  // namespace

int main() {
    std::printf("header=%d library=%d string=%s expected=%s\n", mjVERSION_HEADER, mj_version(),
                mj_versionString(), TERRA_MUJOCO_EXPECTED_VERSION);

    check(mj_version() == mjVERSION_HEADER, "头文件版本与运行库版本一致");
    check(std::strcmp(mj_versionString(), TERRA_MUJOCO_EXPECTED_VERSION) == 0,
          "运行库版本字符串与 mujoco.lock 一致");

    // M0-03–07 计划使用的 C API。取地址要求链接期与装载期都能解析这些符号。
    checkSymbol(&mj_loadXML, "mj_loadXML");
    checkSymbol(&mj_makeData, "mj_makeData");
    checkSymbol(&mj_step, "mj_step");
    checkSymbol(&mj_forward, "mj_forward");
    checkSymbol(&mj_resetData, "mj_resetData");
    checkSymbol(&mj_deleteData, "mj_deleteData");
    checkSymbol(&mj_deleteModel, "mj_deleteModel");
    checkSymbol(&mj_name2id, "mj_name2id");
    checkSymbol(&mj_objectVelocity, "mj_objectVelocity");

    return failures == 0 ? 0 : 1;
}
