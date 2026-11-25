# 用法：cmake -DIMPORT=<terra_import> -DHEADLESS=<terra_headless> -DINPUT=<demo.urdf> -DOUT=<目录> -P TerraCheckImportPack.cmake
# M4-08c（import.cli_pack）：命令行的导入登记与资产包——
#   1. terra_import 导入 demo.urdf 到 <OUT>/first/assets，--library 登记；
#   2. --export-pack 打包导入目录；
#   3. --import-pack 解包到另一个资源库 <OUT>/second/assets 并登记；解包后的文件与原导入目录逐字节相同；
#   4. terra_headless --list-assets --kind robot 列出解包后的机器人（来源为 imported）。
foreach(v IMPORT HEADLESS INPUT OUT)
    if(NOT DEFINED ${v})
        message(FATAL_ERROR "需要 -D${v}")
    endif()
endforeach()
file(REMOVE_RECURSE "${OUT}")

function(run expect_regex stream)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    message(STATUS "$ ${ARGN}\n${out}${err}")
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "退出码 ${rc}（应为 0）")
    endif()
    if(stream STREQUAL "STDOUT")
        set(text "${out}")
    else()
        set(text "${err}")
    endif()
    if(NOT text MATCHES "${expect_regex}")
        message(FATAL_ERROR "${stream} 中没有匹配：${expect_regex}")
    endif()
endfunction()

run("已登记到资源库 [^\n]*first/library.sqlite3：[0-9]+ 个文件、1 个机器人" STDERR
    "${IMPORT}" "${INPUT}" --output "${OUT}/first/assets" --library "${OUT}/first/library.sqlite3")
run("[{].type.:.export_pack.,.ok.:true,.name.:.demo_robot.,[^\n]*.files.:[1-9]" STDOUT
    "${IMPORT}" --export-pack "${OUT}/first/assets/demo_robot" "${OUT}/demo_robot.terra-pack")
run("已登记到资源库 [^\n]*second/library.sqlite3" STDERR
    "${IMPORT}" --import-pack "${OUT}/demo_robot.terra-pack" --output "${OUT}/second/assets" --library "${OUT}/second/library.sqlite3")

file(GLOB_RECURSE original RELATIVE "${OUT}/first/assets/demo_robot" "${OUT}/first/assets/demo_robot/*")
file(GLOB_RECURSE unpacked RELATIVE "${OUT}/second/assets/demo_robot" "${OUT}/second/assets/demo_robot/*")
list(SORT original)
list(SORT unpacked)
if(NOT original STREQUAL unpacked)
    message(FATAL_ERROR "解包后的文件列表不同：\n${original}\n${unpacked}")
endif()
foreach(f IN LISTS original)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUT}/first/assets/demo_robot/${f}" "${OUT}/second/assets/demo_robot/${f}"
                    RESULT_VARIABLE same)
    if(NOT same EQUAL 0)
        message(FATAL_ERROR "解包后的文件不同：${f}")
    endif()
endforeach()
list(LENGTH original count)
message(STATUS "解包后 ${count} 个文件逐字节相同")

run("[{].type.:.asset.,.kind.:.robot.,.source.:.imported.,.title.:.demo_robot.,.path.:.[^\n]*second/assets/demo_robot/demo_robot.xml" STDOUT
    "${HEADLESS}" --list-assets --kind robot --library "${OUT}/second/library.sqlite3")
message(STATUS "import.cli_pack 通过")
