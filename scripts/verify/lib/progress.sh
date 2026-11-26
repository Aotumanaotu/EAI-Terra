# 验证脚本的终端进度条（M5-11 起）。用法：在验证脚本中
#   source "$root/scripts/verify/lib/progress.sh"
#   progress_init <脚本名> <部分总数>          # 读上次运行的耗时，用来估计剩余时间
#   section <序号> <标题>                      # 代替 echo "== 序号. 标题"
#   run_step <名称> <日志> <命令...>           # 命令的输出追加到日志；运行期间在终端底行刷新进度
#   progress_finish                            # 保存本次各步的耗时，供下次估计
# 进度行写到 stderr，只在 stderr 是终端时显示（输出重定向到文件或 CI 中不显示，日志内容不变）：
#   [2/9] 渲染 Debug 编译 ████░░░░░░░░░░░░ 412/1650 24% │ 本步 03:12 │ 总计 25:40 │ 约剩 41 分（第二次运行起，按上次的耗时）
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

_progress_render() {  # _progress_render <名称> <日志> <日志起始字节> <本步开始的 SECONDS>
    local label=$1 log=$2 offset=$3 begin=$4
    local now=$SECONDS chunk detail="" count total
    chunk=$(tail -c +$((offset + 1)) "$log" 2>/dev/null | tail -c 8000)
    # CTest：“ 12/168 Test  #12: 名称 ...”；正在运行的测试取最后一个 “Start  N: 名称”
    count=$(printf '%s\n' "$chunk" | grep -oE '^ *[0-9]+/[0-9]+ Test' | tail -1 | grep -oE '[0-9]+/[0-9]+')
    if [ -n "$count" ]; then
        local running
        running=$(printf '%s\n' "$chunk" | grep -oE 'Start +[0-9]+: [^ ]+' | tail -1 | sed -E 's/Start +[0-9]+: //')
        total=${count#*/}
        count=${count%/*}
        detail="$(_progress_bar "$count" "$total" 16) $count/$total 项测试 $((count * 100 / total))%"
        [ -n "$running" ] && detail+=" ${running:0:32}"
    else
        # Ninja：“[412/1650] Building CXX object ...”
        count=$(printf '%s\n' "$chunk" | grep -oE '^\[[0-9]+/[0-9]+\]' | tail -1 | tr -d '[]')
        if [ -n "$count" ]; then
            total=${count#*/}
            count=${count%/*}
            detail="$(_progress_bar "$count" "$total" 16) $count/$total $((count * 100 / total))%"
        elif printf '%s\n' "$chunk" | grep -q 'Start '; then
            detail="测试运行中（第一项尚未结束）"
        fi
    fi
    local elapsed=$((now - begin)) all=$((now - _progress_start)) eta=""
    if [ "$_progress_prev_total" -gt 0 ]; then
        local expect=${_progress_prev[$label]:-0} current=$elapsed
        [ "$current" -le "$expect" ] || current=$expect
        local left=$((_progress_prev_total - _progress_done - current))
        [ $left -ge 0 ] || left=0
        eta=" │ 约剩 $(((left + 59) / 60)) 分"
    fi
    local line="[${_progress_section}/${_progress_sections}] $label ${detail} │ 本步 $(_progress_clock $elapsed) │ 总计 $(_progress_clock $all)$eta"
    # 按终端宽度截断（ASCII 与进度条字符 1 列，其余按 2 列估算），避免换行后 \r 无法覆盖
    local cols chars=0 width=0 c code out=""
    cols=$(tput cols 2>/dev/null || echo 100)
    while [ $chars -lt ${#line} ]; do
        c=${line:$chars:1}
        printf -v code '%d' "'$c"
        if [ "$code" -lt 128 ] || { [ "$code" -ge 9472 ] && [ "$code" -le 9631 ]; }; then  # ASCII、U+2500～U+259F 制表与方块字符
            width=$((width + 1))
        else
            width=$((width + 2))
        fi
        [ $width -lt "$cols" ] || break
        out+=$c
        chars=$((chars + 1))
    done
    printf '\r\033[K%s' "$out" >&2
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
        printf '\r\033[K' >&2
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
