# 用法：cmake -DSOURCE_DIR=<仓库根目录> -P TerraCheckNoMuJoCoInPublicHeaders.cmake
# 检查 Terra 公共头（engine/**/include/）与应用（apps/）中没有包含 MuJoCo 头文件（D-009）。
if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "需要 -DSOURCE_DIR")
endif()

file(GLOB_RECURSE files
    "${SOURCE_DIR}/engine/*/include/*"
    "${SOURCE_DIR}/engine/*/backends/*/include/*"
    "${SOURCE_DIR}/apps/*")

set(violations "")
foreach(f IN LISTS files)
    if(NOT f MATCHES "\\.(h|hh|hpp|hxx|c|cc|cpp|cxx|inl|ipp)$")
        continue()
    endif()
    file(STRINGS "${f}" hits REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]mujoco/")
    if(hits)
        list(APPEND violations "${f}: ${hits}")
    endif()
endforeach()

if(violations)
    string(REPLACE ";" "\n" violations "${violations}")
    message(FATAL_ERROR "公共头或应用中包含了 MuJoCo 头文件：\n${violations}")
endif()
list(LENGTH files n)
message(STATUS "已检查 ${n} 个文件，未发现 MuJoCo 头文件包含")
