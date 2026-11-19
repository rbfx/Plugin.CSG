# Copyright (c) 2025-2026 the rbfx project.
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

set (TARGET_NAME "Plugin.CSG")

set (SOURCE_FILES
    _Plugin.cpp
    _Plugin.h
    Csg.cpp
    CsgBoxBrush.cpp
    CsgBrush.cpp
    CsgBoolean.cpp
    CsgPolygon.cpp
    CsgSolid.cpp
    CsgCommon.cpp
    CsgComponent.cpp
    CsgCorefine.cpp
    CsgExact.cpp
    CsgExpansion.cpp
    CsgFaceBvh.cpp
    CsgProceduralBrush.cpp
    CsgRoot.cpp
    CsgShapes.cpp
    CsgSphereBrush.cpp
    CsgCylinderBrush.cpp
    CsgTriangulation.cpp
    Csg.h
    CsgBoolean.h
    CsgBoxBrush.h
    CsgSphereBrush.h
    CsgCylinderBrush.h
    CsgRingBrush.cpp
    CsgRingBrush.h
    CsgBrush.h
    CsgPolygon.h
    CsgSolid.h
    CsgCommon.h
    CsgComponent.h
    CsgOperationColor.h
    CsgCorefine.h
    CsgExact.h
    CsgFlatMap.h
    CsgExactFilter.h
    CsgExpansion.h
    CsgFaceBvh.h
    CsgProceduralBrush.h
    CsgRoot.h
    CsgRootInternal.h
    CsgShapes.h
    CsgSimd.h
    CsgTriangulation.h
)

add_plugin (${TARGET_NAME} "${SOURCE_FILES}")

target_include_directories (${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${RbfxCsg_SOURCE_DIR}/Source>)
target_compile_features (${TARGET_NAME} PUBLIC cxx_std_17)
target_link_libraries (${TARGET_NAME} PUBLIC Urho3D)
target_compile_options (${TARGET_NAME} PRIVATE ${CSG_STRICT_FP_OPTIONS})
if (CSG_EXACT_USE_DOUBLE)
    target_compile_definitions (${TARGET_NAME} PRIVATE CSG_EXACT_USE_DOUBLE=1)
endif ()
