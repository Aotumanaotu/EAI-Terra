#include "terra/core/build_info.hpp"

#include "terra/core/build_config.hpp"

static_assert(__cplusplus >= 201703L, "EAI Terra 需要 C++17");

namespace terra {

const BuildInfo& buildInfo() noexcept {
    static const BuildInfo info{
        TERRA_VERSION_MAJOR,
        TERRA_VERSION_MINOR,
        TERRA_VERSION_PATCH,
        TERRA_VERSION_STRING,
        TERRA_CXX_COMPILER_ID,
        TERRA_CXX_COMPILER_VERSION,
        __cplusplus,
    };
    return info;
}

}  // namespace terra
