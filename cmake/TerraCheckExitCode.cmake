# 用法：cmake -DCOMMAND=<程序> [-DARGS=<分号分隔参数>] -DEXPECTED=<退出码> -P TerraCheckExitCode.cmake
# 精确比较退出码；进程崩溃或返回其他值都判定失败（区别于 CTest 的 WILL_FAIL）。
if(NOT DEFINED COMMAND OR NOT DEFINED EXPECTED)
    message(FATAL_ERROR "需要 -DCOMMAND 和 -DEXPECTED")
endif()

execute_process(
    COMMAND "${COMMAND}" ${ARGS}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)

if(NOT "${result}" STREQUAL "${EXPECTED}")
    message(FATAL_ERROR
        "退出码不符：期望 ${EXPECTED}，实际 ${result}\n"
        "命令：${COMMAND} ${ARGS}\n--- stdout ---\n${out}--- stderr ---\n${err}")
endif()
