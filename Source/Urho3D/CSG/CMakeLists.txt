# Copyright (c) 2025-2026 the rbfx project.
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

set (TARGET_NAME "Plugin.CSG")

set (SOURCE_FILES
    _Plugin.cpp
    _Plugin.h
    Csg.cpp
    CsgBrush.cpp
    CsgBsp.cpp
    CsgPolygon.cpp
    CsgSolid.cpp
    CsgCommon.cpp
    CsgComponent.cpp
    CsgExact.cpp
    CsgFaceBvh.cpp
    CsgOptimization.cpp
    CsgRoot.cpp
    CsgTriangulation.cpp
    Csg.h
    CsgBrush.h
    CsgBsp.h
    CsgPolygon.h
    CsgSolid.h
    CsgCommon.h
    CsgComponent.h
    CsgExact.h
    CsgFlatMap.h
    CsgExactFilter.h
    CsgFaceBvh.h
    CsgOptimization.h
    CsgRoot.h
    CsgSimd.h
    CsgTriangulation.h
)

add_plugin (${TARGET_NAME} "${SOURCE_FILES}")

target_include_directories (${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Source>)
target_compile_features (${TARGET_NAME} PUBLIC cxx_std_17)
target_link_libraries (${TARGET_NAME} PUBLIC Urho3D)
