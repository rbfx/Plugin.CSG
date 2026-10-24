# Copyright (c) 2025-2026 the rbfx project.
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

set (TARGET_NAME Tests.CSG)

set (SOURCE_FILES
    TestMain.cpp
    TestExact.cpp
    TestGeometry.cpp
    TestGeometry.h
    TestCommon.cpp
    TestFlatMap.cpp
    TestSolid.cpp
)

# The tests compile the needed CSG sources directly. They do not link Plugin.CSG.
# This keeps the tests independent from the plugin entry point and from the plugin loader.
set (CSG_DIR ${CMAKE_SOURCE_DIR}/Source/Urho3D/CSG)
set (CSG_SOURCE_FILES
    ${CSG_DIR}/CsgBsp.cpp
    ${CSG_DIR}/CsgCommon.cpp
    ${CSG_DIR}/CsgExact.cpp
    ${CSG_DIR}/CsgOptimization.cpp
    ${CSG_DIR}/CsgPolygon.cpp
    ${CSG_DIR}/CsgSolid.cpp
    ${CSG_DIR}/CsgTriangulation.cpp
)

add_executable (${TARGET_NAME} ${SOURCE_FILES} ${CSG_SOURCE_FILES})
target_link_libraries (${TARGET_NAME} PRIVATE Urho3D catch2)
target_compile_features (${TARGET_NAME} PRIVATE cxx_std_17)
target_compile_definitions (${TARGET_NAME} PRIVATE Plugin_RbfxCsg_EXPORT=1)
target_include_directories (${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Source ${CMAKE_CURRENT_SOURCE_DIR})

add_test (NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
