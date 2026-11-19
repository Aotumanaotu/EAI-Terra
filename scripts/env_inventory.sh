#!/usr/bin/env bash
# M0-01 环境清单：只读收集系统与工具链信息，不安装、不修改任何东西。
# M0-09 起增加工具来源（dpkg 归属），用于核实 CMake/Ninja 的安装来源。
# 用法：scripts/env_inventory.sh > env.txt
# 在 Ubuntu 22.04 / x86-64 基线机器上运行，输出用于填写 docs/validation/M0-01-environment.md。
set -u

section() { printf '\n## %s\n' "$1"; }
show() {  # show <标签> <命令...>：命令缺失或失败时如实写出
    local label=$1; shift
    if ! command -v "$1" >/dev/null 2>&1; then
        printf '%s: 未安装 (%s)\n' "$label" "$1"
        return
    fi
    local out
    if out=$("$@" 2>&1); then
        printf '%s: %s\n' "$label" "$(printf '%s' "$out" | head -n 1)"
    else
        printf '%s: 执行失败: %s\n' "$label" "$(printf '%s' "$out" | head -n 1)"
    fi
}

printf '# EAI Terra 环境清单\n'
printf '采集时间: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
printf '主机名: %s\n' "$(hostname)"

section "系统"
if [ -r /etc/os-release ]; then
    . /etc/os-release
    printf '%s: %s\n' "OS" "${PRETTY_NAME:-未知}"
fi
show "内核" uname -r
show "架构" uname -m
show "glibc" ldd --version
printf '%s: %s\n' "CPU" "$(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2- | sed 's/^ *//')"
printf '%s: %s\n' "CPU 核数" "$(nproc 2>/dev/null)"
printf '%s: %s\n' "内存" "$(awk '/MemTotal/ {printf "%.1f GiB", $2/1048576}' /proc/meminfo 2>/dev/null)"
# systemd-detect-virt 在物理机上输出 none 且返回非零，只在无输出时记为未知
virt=$(systemd-detect-virt 2>/dev/null)
printf '%s: %s\n' "容器/虚拟化" "${virt:-未知}"

section "工具链"
show "gcc" gcc --version
show "g++" g++ --version
show "cmake" cmake --version
show "ninja" ninja --version
show "ctest" ctest --version
show "git" git --version
show "ld" ld --version

section "工具来源（dpkg 归属；不属于任何软件包时为手动安装）"
for tool in gcc g++ cmake ctest ninja; do
    path=$(command -v "$tool" 2>/dev/null) || { printf '%s: 未安装\n' "$tool"; continue; }
    real=$(readlink -f "$path")
    owner=$(dpkg -S "$real" 2>/dev/null | head -n 1 | cut -d: -f1)
    [ -n "$owner" ] || owner=$(dpkg -S "$path" 2>/dev/null | head -n 1 | cut -d: -f1)
    if [ -n "$owner" ]; then
        version=$(dpkg-query -W -f='${Version}' "$owner" 2>/dev/null)
        printf '%s: %s -> 软件包 %s %s\n' "$tool" "$real" "$owner" "$version"
    else
        printf '%s: %s -> 不属于 dpkg 软件包（手动安装）\n' "$tool" "$real"
    fi
done

section "显示与图形（M0 不应依赖，仅记录现状）"
printf '%s: %s\n' "DISPLAY" "${DISPLAY:-<未设置>}"
printf '%s: %s\n' "WAYLAND_DISPLAY" "${WAYLAND_DISPLAY:-<未设置>}"
# ldconfig -p 每行以制表符开头："\tlibGL.so.1 (libc6,x86-64) => /path"；按库名前缀匹配任意 soname 版本
ldconfig_bin=$(command -v ldconfig || echo /sbin/ldconfig)
if ! cache=$("$ldconfig_bin" -p 2>/dev/null); then
    printf '图形库检查: 无法执行 ldconfig，未检查\n'
    exit 0
fi
for lib in libGL libEGL libOSMesa libX11 libvulkan; do
    found=$(printf '%s\n' "$cache" | awk -v p="$lib.so" 'index($1, p) == 1 {print $1}' | sort -u | paste -sd ' ' -)
    printf '%s: %s\n' "$lib" "${found:-未找到}"
done
