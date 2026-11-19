#pragma once

// MuJoCo 后端入口（M0-07）。只使用 Terra 类型；MuJoCo 的模型、数据和对象索引都封装在后端内部。

#include "terra/data/model_description.hpp"
#include "terra/physics/physics_world.hpp"

namespace terra::physics::mujoco {

// 解析并编译 MJCF，分配运行数据，计算初始派生状态，返回就绪的世界。
// 文件内的相对资源路径以模型文件所在目录为基准。
LoadResult loadWorld(const terra::data::ModelSource& source);

}  // namespace terra::physics::mujoco
