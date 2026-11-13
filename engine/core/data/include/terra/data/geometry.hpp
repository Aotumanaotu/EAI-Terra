#pragma once

namespace terra::data {

// 暂定坐标约定（D-012，待 M0-06 验证冻结）：SI 单位，右手世界坐标，Z 向上、X 向前、Y 向左，双精度。

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 单位四元数，分量顺序 w, x, y, z；表示实体局部坐标到世界坐标的旋转。
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 实体原点（不是质心）在世界坐标中的位姿。
struct Pose {
    Vec3 position;
    Quaternion orientation;
};

}  // namespace terra::data
