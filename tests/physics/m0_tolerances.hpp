#pragma once

// M0 物理场景判定的容差（M0-08 于 2026-11-18 基线验证后冻结，开发计划 4.2 / 4.3 节）。
// 唯一数值来源：physics.scenarios 与 physics.continuous 只从这里取门槛；修改须同步更新
// docs/validation/M0-08-scenarios.md 的容差表及理由，并重新在基线机器验证。
//
// 适用条件：MuJoCo 4.0.1、falling_cube.xml / floating_cube.xml / hinge_motor.xml 当前参数、
// dt = 0.001 s、Euler 积分。换版本、换积分器或改夹具参数时必须重新评审。

namespace terra::test::m0 {

// ---- 主场景（falling_cube.xml，5000 步）----
constexpr int kMainSteps = 5000;

// 时间：N 步后 |t - N·dt|。浮点累加误差远小于 1 ns；门槛对应计划值。
constexpr double kTimeError = 1e-8;  // s

// 自由落体窗口与误差：与连续解析解 z = 1 - g·t²/2 比较。
// 半隐式 Euler 的理论偏差为 g·dt·t/2，在 t = 0.2 s 时 9.81e-4 m；门槛留出 5 倍余量。
constexpr double kFreeFallWindowBegin = 0.05;  // s
constexpr double kFreeFallWindowEnd = 0.20;    // s
constexpr double kFreeFallError = 0.005;       // m
// 平面内漂移：没有任何水平力，x/y 应保持 0；门槛只容纳舍入误差。
constexpr double kPlanarDrift = 1e-9;  // m

// 首次有效接触时刻窗口。理想几何触地 ≈ 0.428 s；软接触与离散步长使实际时刻略有偏移。
constexpr double kFirstContactBegin = 0.35;  // s
constexpr double kFirstContactEnd = 0.55;    // s

// 稳定窗口：最后 0.5 s。高度门槛包含软接触允许的穿透（实测约 1e-4 m）。
constexpr double kRestWindow = 0.5;          // s
constexpr double kRestHeight = 0.1;          // m，立方体半边长
constexpr double kRestHeightError = 0.005;   // m
constexpr double kRestLinearSpeed = 0.01;    // m/s
constexpr double kRestAngularSpeed = 0.05;   // rad/s
// 静止时法向力之和与 m·g 的相对偏差。
constexpr double kRestNormalForceRelative = 0.01;

// 数据有效性：四元数范数误差；最终姿态与单位姿态的旋转角（按 q 与 -q 等价比较）。
// 立方体平落、无水平力，理想情况下不发生转动。
constexpr double kQuaternionNormError = 1e-6;
constexpr double kFinalOrientationAngle = 0.01;  // rad

// ---- 辅助用例 ----
// 无重力、无初速度（floating_cube.xml）：没有任何力，位姿与速度只允许舍入误差。
constexpr int kFloatingSteps = 5000;
constexpr double kFloatingPoseError = 1e-12;
constexpr double kFloatingSpeed = 1e-12;

// reset / 再运行：同进程同环境的轨迹比较（计划候选值）。
constexpr int kResetCycles = 20;
constexpr double kRerunTrajectoryError = 1e-9;

// 连续运行：主夹具 60 s 仿真时间。
constexpr int kContinuousSteps = 60000;

}  // namespace terra::test::m0
