#pragma once

// MuJoCo 后端入口（M0-07）。只使用 Terra 类型；MuJoCo 的模型、数据和对象索引都封装在后端内部。

#include "terra/data/model_description.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/physics/world_composition.hpp"

namespace terra::physics::mujoco {

// 解析并编译 MJCF，分配运行数据，计算初始派生状态，返回就绪的世界。
// 文件内的相对资源路径以模型文件所在目录为基准。
LoadResult loadWorld(const terra::data::ModelSource& source);

// 按组合请求合成一个世界（M2-04）：逐个解析 MJCF（mj_parseXML），用 mjSpec 组合（mjs_attach，加前缀、放在各自的
// 放置位姿上）并编译。规则见 terra/physics/world_composition.hpp。不修改 MJCF 文件。
ComposeResult composeWorld(const WorldComposition& composition);

}  // namespace terra::physics::mujoco
