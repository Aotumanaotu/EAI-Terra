# terra_import：模型导入（M4-08a）。URDF / xacro / MJCF → Terra 场景；不含 MuJoCo 与图形头文件（检查时用调用方给的 WorldFactory）。
# 第三方：tinyxml2（zlib，third_party/tinyxml2）读写 XML；xacro（BSD-3-Clause，third_party/xacro）经 tools/xacro_expand.py 调用。

# tinyxml2：第三方源码，不套用 Terra 的严格警告
add_library(terra_tinyxml2 STATIC "${PROJECT_SOURCE_DIR}/third_party/tinyxml2/tinyxml2.cpp")
target_include_directories(terra_tinyxml2 SYSTEM PUBLIC "${PROJECT_SOURCE_DIR}/third_party/tinyxml2")
set_target_properties(terra_tinyxml2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(terra_import STATIC
    src/importer.cpp
    src/mesh.cpp
    src/process.cpp
    src/urdf.cpp)
add_library(terra::import ALIAS terra_import)
target_include_directories(terra_import PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_import PUBLIC terra::scene PRIVATE terra::json terra_tinyxml2)
target_compile_definitions(terra_import PRIVATE
    TERRA_IMPORT_XACRO_DIR="${PROJECT_SOURCE_DIR}/third_party/xacro"
    TERRA_IMPORT_XACRO_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/tools/xacro_expand.py")
terra_configure_target(terra_import)
