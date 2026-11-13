#include "terra/physics/mujoco/backend_info.hpp"

#include <mujoco/mujoco.h>

namespace terra::physics::mujoco {

BackendInfo backendInfo() noexcept {
    return BackendInfo{
        "MuJoCo",
        mjVERSION_HEADER,
        mj_version(),
        mj_versionString(),
    };
}

}  // namespace terra::physics::mujoco
