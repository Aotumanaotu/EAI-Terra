# 验证脚本的终端进度条（M5-11 起）。用法：在验证脚本中
#   source "$root/scripts/verify/lib/progress.sh"
#   progress_init <脚本名> <部分总数>          # 读上次运行的耗时，用来估计剩余时间
#   section <序号> <标题>                      # 代替 echo "== 序号. 标题"
#   run_step <名称> <日志> <命令...>           # 命令的输出追加到日志；运行期间在终端底行刷新进度
#   progress_finish                            # 保存本次各步的耗时，供下次估计
# 进度行写到 stderr，只在 stderr 是终端时显示（输出重定向到文件或 CI 中不显示，日志内容不变）：
#   [第 2/9 部分]  ████████░░░░░░░░░░░░   24%  412/1650 个编译单元            （彩色：绿色为已完成）
#   渲染 Debug 编译 · 正在运行的测试名            本步 03:12 · 总计 25:40 · 约剩 41 分（第二次运行起，按上次的耗时）
# 步内进度来自日志：Ninja 的 [已完成/总数]、CTest 的 “N/M Test”（并显示正在运行的测试名）。
# 命令在前台运行（Ctrl-C 照常中断它，脚本的 trap 照常执行），刷新进度的是一个后台循环。

_progress_script=""
_progress_sections=0
_progress_section=0
_progress_start=$SECONDS
_progress_timing_file=""
_progress_done=0         # 本次已完成各步的耗时之和（秒）
_progress_prev_total=0   # 上次运行各步的耗时之和（秒）；0 表示没有记录
declare -A _progress_prev=()
_progress_lines=()

progress_init() {  # progress_init <脚本名> <部分总数>
    _progress_script=$1
    _progress_sections=$2
    _progress_start=$SECONDS
    _progress_timing_file="${root:-.}/build/verify-timings/$1.tsv"
    if [ -f "$_progress_timing_file" ]; then
        local name secs
        while IFS=$'\t' read -r name secs; do
            [ -n "$name" ] && [ -n "$secs" ] || continue
            _progress_prev[$name]=$secs
            _progress_prev_total=$((_progress_prev_total + secs))
        done <"$_progress_timing_file"
    fi
}

section() {  # section <序号> <标题>
    _progress_section=$1
    echo "== $1. $2"
}

_progress_clock() {  # 秒 → mm:ss 或 h:mm:ss
    local s=$1
    if [ "$s" -ge 3600 ]; then printf '%d:%02d:%02d' $((s / 3600)) $((s % 3600 / 60)) $((s % 60))
    else printf '%02d:%02d' $((s / 60)) $((s % 60)); fi
}

_progress_bar() {  # _progress_bar <已完成> <总数> <宽度>
    local done=$1 total=$2 width=$3 filled i out=""
    [ "$total" -gt 0 ] || total=1
    filled=$((done * width / total))
    [ $filled -le $width ] || filled=$width
    for ((i = 0; i < width; ++i)); do
        if [ $i -lt $filled ]; then out+="█"; else out+="░"; fi
    done
    printf '%s' "$out"
}

# 显示宽度（列）：ASCII 与 U+2500～U+259F 的制表、方块字符 1 列，其余（汉字、全角标点）2 列
_progress_width() {
    local text=$1 i=0 width=0 code
    while [ $i -lt ${#text} ]; do
        printf -v code '%d' "'${text:$i:1}"
        if [ "$code" -lt 128 ] || { [ "$code" -ge 9472 ] && [ "$code" -le 9631 ]; }; then width=$((width + 1)); else width=$((width + 2)); fi
        i=$((i + 1))
    done
    echo $width
}

# 截断到 max 列（超出时以 … 结尾）
_progress_cut() {
    local text=$1 max=$2 i=0 width=0 code w out=""
    [ "$(_progress_width "$text")" -le "$max" ] && { printf '%s' "$text"; return; }
    while [ $i -lt ${#text} ]; do
        printf -v code '%d' "'${text:$i:1}"
        if [ "$code" -lt 128 ] || { [ "$code" -ge 9472 ] && [ "$code" -le 9631 ]; }; then w=1; else w=2; fi
        [ $((width + w)) -le $((max - 1)) ] || break
        out+=${text:$i:1}
        width=$((width + w))
        i=$((i + 1))
    done
    printf '%s…' "$out"
}

# 两行进度（彩色）：
#   第 1 行：[第 2/9 部分]  ████████████░░░░░░░░  45%  412/1650   （绿色为已完成）
#   第 2 行：渲染 Debug 编译 · 正在运行的测试名          本步 03:12 · 总计 25:40 · 约剩 41 分
# 每行按终端宽度截断（先截第 2 行的名称，时间总是完整显示），避免折行后无法覆盖
_progress_render() {
    local label=$1 log=$2 offset=$3 begin=$4
    local now=$SECONDS chunk count="" total="" unit="" running=""
    chunk=$(tail -c +$((offset + 1)) "$log" 2>/dev/null | tail -c 8000)
    # CTest：“ 12/168 Test  #12: 名称 ...”；正在运行的测试取最后一个 “Start  N: 名称”
    local found
    found=$(printf '%s\n' "$chunk" | grep -oE '^ *[0-9]+/[0-9]+ Test' | tail -1 | grep -oE '[0-9]+/[0-9]+')
    if [ -n "$found" ]; then
        running=$(printf '%s\n' "$chunk" | grep -oE 'Start +[0-9]+: [^ ]+' | tail -1 | sed -E 's/Start +[0-9]+: //')
        total=${found#*/}
        count=${found%/*}
        unit="项测试"
    else
        # Ninja：“[412/1650] Building CXX object ...”
        found=$(printf '%s\n' "$chunk" | grep -oE '^\[[0-9]+/[0-9]+\]' | tail -1 | tr -d '[]')
        if [ -n "$found" ]; then
            total=${found#*/}
            count=${found%/*}
            unit="个编译单元"
        elif printf '%s\n' "$chunk" | grep -q 'Start '; then
            running="第一项测试尚未结束"
        fi
    fi
    local cols="" rows
    # 终端的真实宽度（stty size 读当前终端；拿不到时用 tput，再不行按 100 列）
    read -r rows cols < <(stty size </dev/tty 2>/dev/null) || true
    [ -n "$cols" ] && [ "$cols" -gt 0 ] 2>/dev/null || cols=$(tput cols 2>/dev/null || echo 100)
    [ "$cols" -ge 40 ] || cols=40
    local reset=$'\033[0m' bold=$'\033[1m' cyan=$'\033[36m' green=$'\033[32m' gray=$'\033[90m' yellow=$'\033[33m' magenta=$'\033[35m'

    # 第 1 行
    local head="[第 ${_progress_section}/${_progress_sections} 部分]"
    local line1="${cyan}${bold}${head}${reset}"
    local used
    used=$(_progress_width "$head")
    if [ -n "$total" ] && [ "$total" -gt 0 ]; then
        local pct=$((count * 100 / total)) tail_text bar_width filled i bar=""
        tail_text=$(printf '%4d%%  %s/%s %s' "$pct" "$count" "$total" "$unit")
        bar_width=$((cols - 1 - used - 2 - $(_progress_width "$tail_text") - 2))
        [ $bar_width -le 40 ] || bar_width=40
        if [ $bar_width -ge 8 ]; then
            filled=$((count * bar_width / total))
            [ $filled -le $bar_width ] || filled=$bar_width
            bar+="$green"
            for ((i = 0; i < filled; ++i)); do bar+="█"; done
            bar+="$gray"
            for ((i = filled; i < bar_width; ++i)); do bar+="░"; done
            bar+="$reset"
            line1+="  ${bar}  ${bold}$(printf '%4d%%' "$pct")${reset}  ${gray}${count}/${total} ${unit}${reset}"
        else
            line1+="  ${bold}${pct}%${reset}"
        fi
    else
        line1+="  ${gray}（准备中，没有进度信息）${reset}"
    fi

    # 第 2 行：时间（完整显示）在右，名称在左（放不下时截断）
    local elapsed=$((now - begin)) all=$((now - _progress_start))
    local times="本步 $(_progress_clock $elapsed) · 总计 $(_progress_clock $all)"
    if [ "$_progress_prev_total" -gt 0 ]; then
        local expect=${_progress_prev[$label]:-0} current=$elapsed
        [ "$current" -le "$expect" ] || current=$expect
        local left=$((_progress_prev_total - _progress_done - current))
        [ $left -ge 0 ] || left=0
        times+=" · 约剩 $(((left + 59) / 60)) 分"
    fi
    local name=$label
    [ -n "$running" ] && name+=" · $running"
    local room=$((cols - 1 - $(_progress_width "$times") - 3))
    local shown="" pad=0
    if [ $room -ge 6 ]; then
        shown=$(_progress_cut "$name" "$room")
        pad=$((room - $(_progress_width "$shown")))
    fi
    local spaces
    printf -v spaces '%*s' $((pad + 3)) ''
    local line2="${magenta}${bold}${shown%% · *}${reset}"
    [ "$shown" != "${shown%% · *}" ] && line2+="${gray} · ${shown#* · }${reset}"
    line2+="${spaces}${yellow}${times}${reset}"
    # 画两行后回到第 1 行行首
    printf '\r\033[K%s\n\r\033[K%s\033[1A\r' "$line1" "$line2" >&2
}

run_step() {  # run_step <名称> <日志> <命令...>；返回命令的退出码
    local label=$1 log=$2
    shift 2
    local begin=$SECONDS offset=0 rc ticker=""
    [ -f "$log" ] && offset=$(stat -c %s "$log")
    if [ -t 2 ]; then
        (
            trap 'exit 0' TERM
            while kill -0 $$ 2>/dev/null; do
                _progress_render "$label" "$log" "$offset" "$begin"
                sleep 1 &
                wait $!  # 在 wait 中收到 TERM 立即退出（前台 sleep 会让 trap 推迟到 sleep 结束）
            done
        ) &
        ticker=$!
    fi
    "$@" >>"$log" 2>&1
    rc=$?
    if [ -n "$ticker" ]; then
        kill "$ticker" 2>/dev/null
        wait "$ticker" 2>/dev/null
        printf '\r\033[K\n\r\033[K\033[1A\r' >&2  # 清掉两行进度
    fi
    local secs=$((SECONDS - begin))
    _progress_done=$((_progress_done + secs))
    _progress_lines+=("$label"$'\t'"$secs")
    return $rc
}

progress_finish() {  # 保存本次各步的耗时（只在本次运行了足够多的步时覆盖，避免 --mutation-only 等部分运行冲掉完整记录）
    [ -n "$_progress_timing_file" ] || return 0
    local all=$((SECONDS - _progress_start))
    echo "总耗时 $(_progress_clock $all)"
    [ ${#_progress_lines[@]} -gt 0 ] || return 0
    if [ "$_progress_prev_total" -gt 0 ] && [ "$_progress_done" -lt $((_progress_prev_total / 2)) ]; then
        return 0
    fi
    mkdir -p "$(dirname "$_progress_timing_file")"
    printf '%s\n' "${_progress_lines[@]}" >"$_progress_timing_file"
}
