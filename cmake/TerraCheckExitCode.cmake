# 用法：cmake -DCOMMAND=<程序> -DEXPECTED=<退出码>
#             [-DSTDOUT_REGEX=<正则>] [-DSTDERR_REGEX=<正则>]
#             -P TerraCheckExitCode.cmake [程序参数...]
# 精确比较退出码；进程崩溃或返回其他值都判定失败（区别于 CTest 的 WILL_FAIL）。
# 可选地用正则检查 stdout / stderr。
if(NOT DEFINED COMMAND OR NOT DEFINED EXPECTED)
    message(FATAL_ERROR "需要 -DCOMMAND 和 -DEXPECTED")
endif()

# 脚本路径之后的命令行参数原样传给被测程序。
set(args "")
set(after_script FALSE)
math(EXPR last "${CMAKE_ARGC} - 1")
foreach(i RANGE 0 ${last})
    if(after_script)
        list(APPEND args "${CMAKE_ARGV${i}}")
    elseif(CMAKE_ARGV${i} STREQUAL CMAKE_SCRIPT_MODE_FILE
           OR CMAKE_ARGV${i} MATCHES "TerraCheckExitCode\\.cmake$")
        set(after_script TRUE)
    endif()
endforeach()

execute_process(
    COMMAND "${COMMAND}" ${args}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)

set(report "命令：${COMMAND} ${args}\n--- stdout ---\n${out}--- stderr ---\n${err}")
if(NOT "${result}" STREQUAL "${EXPECTED}")
    message(FATAL_ERROR "退出码不符：期望 ${EXPECTED}，实际 ${result}\n${report}")
endif()
if(DEFINED STDOUT_REGEX AND NOT out MATCHES "${STDOUT_REGEX}")
    message(FATAL_ERROR "stdout 不匹配：${STDOUT_REGEX}\n${report}")
endif()
if(DEFINED STDERR_REGEX AND NOT err MATCHES "${STDERR_REGEX}")
    message(FATAL_ERROR "stderr 不匹配：${STDERR_REGEX}\n${report}")
endif()
