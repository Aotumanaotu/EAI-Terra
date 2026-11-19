#!/usr/bin/env bash
# M1 前置：图形环境与 OpenGL 版本清单。只读收集信息，不安装、不修改任何东西。
#
# 请在基线机器的**桌面会话终端**中运行（需要 DISPLAY 或 WAYLAND_DISPLAY 才能查询 OpenGL 上下文）：
#   bash scripts/gfx_inventory.sh | tee gfx.txt
# 输出用于确定 M1 的 OpenGL 最低版本、窗口库与依赖获取方式（开发计划第 5 节 M1 前置依赖）。
set -u

section() { printf '\n## %s\n' "$1"; }
have() { command -v "$1" >/dev/null 2>&1; }

printf '# EAI Terra 图形环境清单\n'
printf '采集时间: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"

section "会话"
printf 'XDG_SESSION_TYPE: %s\n' "${XDG_SESSION_TYPE:-<未设置>}"
printf 'DISPLAY: %s\n' "${DISPLAY:-<未设置>}"
printf 'WAYLAND_DISPLAY: %s\n' "${WAYLAND_DISPLAY:-<未设置>}"
printf 'XDG_CURRENT_DESKTOP: %s\n' "${XDG_CURRENT_DESKTOP:-<未设置>}"

section "GPU 硬件"
if have lspci; then
    lspci -nn | grep -Ei 'vga|3d|display' || echo "lspci 未列出显示设备"
else
    echo "lspci 未安装"
fi
for f in /sys/class/drm/card*/device/driver; do
    [ -e "$f" ] && printf '%s -> %s\n' "${f%/device/driver}" "$(basename "$(readlink -f "$f")")"
done
[ -r /proc/driver/nvidia/version ] && head -1 /proc/driver/nvidia/version

glx_summary() {
    glxinfo -B 2>&1 | grep -E 'direct rendering|OpenGL vendor|OpenGL renderer|OpenGL core profile version|OpenGL core profile shading|OpenGL version string|Max core profile|Accelerated|Video memory' \
        || echo "glxinfo 执行失败"
}

section "OpenGL（GLX）"
if ! have glxinfo; then
    echo "glxinfo 未安装（apt 包 mesa-utils）：sudo apt install mesa-utils"
else
    echo "### 1) 桌面会话默认（手动运行渲染程序时实际使用的 GPU）"
    if [ -n "${DISPLAY:-}" ]; then glx_summary; else echo "DISPLAY 未设置，跳过（请在桌面会话中运行）"; fi

    echo "### 2) NVIDIA PRIME offload（双显卡时显式选择 NVIDIA）"
    if [ -n "${DISPLAY:-}" ]; then
        __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glx_summary
    else
        echo "DISPLAY 未设置，跳过"
    fi

    echo "### 3) Xvfb + Mesa 软件渲染（自动化图形测试候选，无需显示器）"
    if have xvfb-run; then
        env -u DISPLAY -u WAYLAND_DISPLAY __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 \
            xvfb-run -a -s "-screen 0 1280x720x24" bash -c "$(declare -f glx_summary); glx_summary"
    else
        echo "xvfb-run 不可用"
    fi
fi

section "EGL（无窗口上下文，M4 离屏相机相关，仅记录）"
if have eglinfo; then
    eglinfo -B 2>&1 | grep -E 'EGL vendor|EGL version|OpenGL core profile version|OpenGL renderer|platform' | head -20
else
    echo "eglinfo 未安装（apt 包 mesa-utils-extra 或 mesa-utils 新版本）"
fi

section "Vulkan（D-101 参考，仅记录）"
if have vulkaninfo; then
    vulkaninfo --summary 2>&1 | grep -E 'apiVersion|deviceName|driverName|driverInfo' | head -10
else
    echo "vulkaninfo 未安装"
fi

section "Mesa / 驱动软件包"
# 只列出已安装的包（通配符会匹配到已知但未安装的包）；多架构包可能出现两行
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version} ${Architecture}\n' 'libgl1-mesa-dri' 'mesa-vulkan-drivers' \
    'libegl-mesa0' 'nvidia-driver-*' 'xserver-xorg-video-*' 2>/dev/null | awk '$1 == "ii" {print $2, $3, $4}' | sort -u || true

section "M1 候选开发包（已安装/未安装）"
for pkg in libgl-dev libegl-dev libopengl-dev libglx-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev \
           libxi-dev libwayland-dev libxkbcommon-dev libglfw3-dev libsdl2-dev libglm-dev xvfb; do
    v=$(dpkg-query -W -f='${Status} ${Version}' "$pkg" 2>/dev/null)
    case "$v" in
        "install ok installed "*) printf '%-20s 已安装 %s\n' "$pkg" "${v#install ok installed }" ;;
        *) printf '%-20s 未安装（apt 候选：%s）\n' "$pkg" "$(apt-cache policy "$pkg" 2>/dev/null | awk '/Candidate:/ {print $2}')" ;;
    esac
done

section "显示器"
if have xrandr && [ -n "${DISPLAY:-}" ]; then
    xrandr --current 2>/dev/null | grep -E ' connected|\*' | head -8
else
    echo "xrandr 不可用或无 DISPLAY"
fi
