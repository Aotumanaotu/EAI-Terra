#!/usr/bin/env bash
# ROS2 环境检查（M4-05 准备）：只读检查安装情况，并在本机做一次发布 / 订阅往返；不编译、不改动系统。
#
# 检查内容（日志在 build/check-ros2/，屏幕只打印结论）：
#   1. ROS2 安装：/opt/ros/humble、环境变量（ROS_DISTRO、RMW_IMPLEMENTATION、ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY）
#   2. 已安装的 ROS2 软件包：桥接需要的消息与库（rclcpp、sensor_msgs、tf2_msgs、rosgraph_msgs、std_srvs 等），
#      RViz、rclpy、两种 DDS 实现（Fast DDS、Cyclone DDS）
#   3. CMake 配置文件：Terra 构建时用 find_package 查找的包
#   4. rclpy 可以导入
#   5. 本机往返：在一个不常用的域 ID 上、只走本机回环，用 ros2 topic pub / echo 收发一条消息；每种已安装的 DDS 各做一次
#   6. 厂商消息（只记录）：宇树 unitree_go / unitree_hg（unitree_ros2）是否已编译并可见
# 必需项：1 的 ROS_DISTRO=humble、3 的 rclcpp 与 sensor_msgs、5 中默认 DDS 的往返。其余只记录。
# 用法：bash scripts/ros2/check_ros2.sh [--setup <setup.bash 路径>]（默认 /opt/ros/humble/setup.bash）
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

setup=/opt/ros/humble/setup.bash
while [ $# -gt 0 ]; do
    case "$1" in
        --setup) setup=$2; shift 2 ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/check-ros2"
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"

summary=()
failed=0
record() {  # record <名称> <结果文字> <是否通过 0/1>
    summary+=("$(printf '%-28s %s' "$1" "$2")")
    [ "$3" = 1 ] || failed=1
    echo "${summary[-1]}"
}
note() { summary+=("$(printf '%-28s %s' "$1" "$2")"); echo "${summary[-1]}"; }

echo "== 1. 安装与环境变量"
if [ ! -f "$setup" ]; then
    record "ROS2 setup.bash" "未找到 $setup" 0
    printf '%s\n' "${summary[@]}"
    exit 1
fi
{
    echo "调用 setup 之前的环境："
    env | grep -E "^(ROS_|RMW_|AMENT_|COLCON_|CYCLONEDDS|FASTRTPS)" | sort
} >"$logs/env_before.txt"
# setup.bash 中引用未设置的变量，临时关闭 -u
set +u
# shellcheck disable=SC1090
source "$setup"
set -u
env | grep -E "^(ROS_|RMW_|AMENT_PREFIX|CYCLONEDDS|FASTRTPS)" | sort >"$logs/env_after.txt"
record "ROS_DISTRO" "${ROS_DISTRO:-未设置}（ROS_VERSION ${ROS_VERSION:-?}，Python ${ROS_PYTHON_VERSION:-?}）" "$([ "${ROS_DISTRO:-}" = humble ] && echo 1 || echo 0)"
note "当前 DDS 设置" "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-未设置（Humble 默认 Fast DDS）}，ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设置（0）}，ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-未设置}"
note "AMENT_PREFIX_PATH" "$(echo "${AMENT_PREFIX_PATH:-}" | tr ':' ' ')"
if [ -f "$HOME/.bashrc" ] && grep -qE "source .*(ros|setup\.bash)" "$HOME/.bashrc"; then
    note "~/.bashrc" "已自动加载 ROS 环境：$(grep -E "source .*(ros|setup\.bash)" "$HOME/.bashrc" | tr '\n' ' ')"
else
    note "~/.bashrc" "没有自动加载 ROS 环境（Terra 的脚本会自己加载，不影响）"
fi

echo "== 2. 软件包"
dpkg -l 'ros-humble-*' 2>/dev/null | awk '/^ii/ {print $2, $3}' >"$logs/dpkg.txt"
note "ros-humble-* 软件包" "$(wc -l <"$logs/dpkg.txt") 个（清单见 $logs/dpkg.txt）"
for meta in ros-humble-desktop ros-humble-ros-base ros-humble-ros-core; do
    if grep -q "^$meta " "$logs/dpkg.txt"; then note "元包" "$(grep "^$meta " "$logs/dpkg.txt")"; fi
done
ros2 pkg list >"$logs/pkg_list.txt" 2>"$logs/pkg_list.err"
note "ros2 pkg list" "$(wc -l <"$logs/pkg_list.txt") 个包"
missing=()
for p in rclcpp rclpy sensor_msgs geometry_msgs nav_msgs tf2_msgs tf2_ros rosgraph_msgs std_msgs std_srvs builtin_interfaces \
         trajectory_msgs control_msgs visualization_msgs image_transport cv_bridge rosidl_runtime_cpp rosidl_typesupport_introspection_cpp \
         rosbag2_cpp rviz2 rmw_fastrtps_cpp rmw_cyclonedds_cpp; do
    grep -qx "$p" "$logs/pkg_list.txt" || missing+=("$p")
done
if [ ${#missing[@]} = 0 ]; then
    note "常用包" "全部可见（rclcpp、消息包、tf2、image_transport、rosbag2、rviz2、两种 DDS 等）"
else
    note "常用包缺少" "${missing[*]}"
fi

echo "== 3. CMake 配置文件"
cm_missing=()
for p in rclcpp sensor_msgs geometry_msgs tf2_msgs rosgraph_msgs std_srvs builtin_interfaces rosidl_typesupport_introspection_cpp; do
    found=""
    for prefix in $(echo "${AMENT_PREFIX_PATH:-}" | tr ':' ' '); do
        if [ -f "$prefix/share/$p/cmake/${p}Config.cmake" ] || [ -f "$prefix/share/$p/cmake/${p}-config.cmake" ]; then found=1; break; fi
    done
    [ -n "$found" ] || cm_missing+=("$p")
done
if [ ${#cm_missing[@]} = 0 ]; then
    record "find_package 所需配置" "rclcpp、sensor_msgs 等 8 个包的 CMake 配置都在" 1
else
    req=0
    for p in "${cm_missing[@]}"; do [ "$p" = rclcpp ] || [ "$p" = sensor_msgs ] && req=1; done
    record "find_package 所需配置" "缺少：${cm_missing[*]}" "$([ $req = 0 ] && echo 1 || echo 0)"
fi

echo "== 4. rclpy"
python3 -c "import rclpy, sys; print(sys.version.split()[0])" >"$logs/rclpy.txt" 2>&1
rc=$?
note "rclpy" "$([ $rc = 0 ] && echo "可以导入（Python $(cat "$logs/rclpy.txt")）" || echo "导入失败（见 $logs/rclpy.txt）")"

echo "== 5. 本机往返"
# 不常用的域 ID + 只走本机：不会与网络上的真机或其他人的节点互相干扰
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
roundtrip() {  # roundtrip <rmw 或 default>
    local rmw=$1 tag=$1 log
    log="$logs/roundtrip_$tag.txt"
    (
        [ "$rmw" = default ] && unset RMW_IMPLEMENTATION || export RMW_IMPLEMENTATION=$rmw
        ros2 daemon stop >/dev/null 2>&1
        timeout 20 ros2 topic echo --once /terra_check std_msgs/msg/String >"$log" 2>&1 &
        echo_pid=$!
        sleep 2
        timeout 15 ros2 topic pub -r 2 -t 10 /terra_check std_msgs/msg/String "{data: terra}" >"$logs/pub_$tag.txt" 2>&1 &
        pub_pid=$!
        wait $echo_pid
        rc=$?
        kill $pub_pid 2>/dev/null
        wait $pub_pid 2>/dev/null
        ros2 daemon stop >/dev/null 2>&1
        exit $rc
    )
    local rc=$?
    grep -q "data: terra" "$log" && [ $rc = 0 ]
}
if roundtrip default; then
    record "往返（默认 DDS）" "收到消息" 1
else
    record "往返（默认 DDS）" "没有收到（见 $logs/roundtrip_default.txt）" 0
fi
for rmw in rmw_fastrtps_cpp rmw_cyclonedds_cpp; do
    if grep -qx "$rmw" "$logs/pkg_list.txt"; then
        if roundtrip "$rmw"; then note "往返 $rmw" "收到消息"; else note "往返 $rmw" "没有收到（见 $logs/roundtrip_$rmw.txt）"; fi
    else
        note "往返 $rmw" "未安装"
    fi
done

echo "== 6. 厂商消息（只记录）"
for iface in unitree_go/msg/LowState unitree_go/msg/LowCmd unitree_hg/msg/LowState unitree_hg/msg/LowCmd; do
    if ros2 interface show "$iface" >"$logs/iface_${iface//\//_}.txt" 2>&1; then
        note "$iface" "可见（$(grep -cvE '^\s*(#|$)' "$logs/iface_${iface//\//_}.txt") 行定义）"
    else
        note "$iface" "不可见（宇树 unitree_ros2 的消息包未编译或未加载）"
    fi
done

{
    echo "### 系统"
    lsb_release -ds 2>/dev/null
    uname -r
    echo "### 网络接口（DDS 发现用）"
    ip -br addr 2>/dev/null
} >"$logs/system.txt"

echo
echo "================ 汇总 ================"
printf '%s\n' "${summary[@]}"
echo "日志目录：$logs"
if [ $failed = 0 ]; then
    echo "结论：必需项全部满足"
else
    echo "结论：存在不满足的必需项"
fi
exit $failed
