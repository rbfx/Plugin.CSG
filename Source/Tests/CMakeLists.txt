# Copyright (c) 2025-2026 the rbfx project.
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

set (TARGET_NAME Tests.CSG)

set (SOURCE_FILES
    TestMain.cpp
    TestExact.cpp
    TestExactReference.cpp
    TestExactReference.h
    TestInt128.h
    TestGeometry.cpp
    TestGeometry.h
    TestCommon.cpp
    TestFlatMap.cpp
    TestSolid.cpp
    TestBvh.cpp
    TestCorefine.cpp
    TestSplit.cpp
    TestBoolean.cpp
    TestTriangulation.cpp
    TestRoot.cpp
    TestEditorOps.cpp
    TestGeneratedBrush.h
    TestParallel.cpp
    TestBenchmark.cpp
    TestGolden.cpp
    TestShapes.cpp
)

# The tests compile the needed CSG sources directly. They do not link Plugin.CSG.
# This keeps the tests independent from the plugin entry point and from the plugin loader.
set (CSG_DIR ${RbfxCsg_SOURCE_DIR}/Source/Urho3D/CSG)
set (CSG_SOURCE_FILES
    ${CSG_DIR}/Csg.cpp
    ${CSG_DIR}/CsgBoxBrush.cpp
    ${CSG_DIR}/CsgSphereBrush.cpp
    ${CSG_DIR}/CsgBrush.cpp
    ${CSG_DIR}/CsgComponent.cpp
    ${CSG_DIR}/CsgProceduralBrush.cpp
    ${CSG_DIR}/CsgRoot.cpp
    ${CSG_DIR}/CsgShapes.cpp
    ${CSG_DIR}/CsgBoolean.cpp
    ${CSG_DIR}/CsgCommon.cpp
    ${CSG_DIR}/CsgCorefine.cpp
    ${CSG_DIR}/CsgExact.cpp
    ${CSG_DIR}/CsgExpansion.cpp
    ${CSG_DIR}/CsgFaceBvh.cpp
    ${CSG_DIR}/CsgPolygon.cpp
    ${CSG_DIR}/CsgSolid.cpp
    ${CSG_DIR}/CsgTriangulation.cpp
)

# The editor scene functions use reflection only. The tests compile them directly.
set (CSG_EDITOR_SOURCE_FILES
    ${RbfxCsg_SOURCE_DIR}/Source/Urho3D/CSGEditor/CsgEditorOps.cpp
)

add_executable (${TARGET_NAME} ${SOURCE_FILES} ${CSG_SOURCE_FILES} ${CSG_EDITOR_SOURCE_FILES})
target_link_libraries (${TARGET_NAME} PRIVATE Urho3D catch2)
target_compile_features (${TARGET_NAME} PRIVATE cxx_std_17)
target_compile_definitions (${TARGET_NAME} PRIVATE Plugin_RbfxCsg_EXPORT=1)
# The diagnostic tests ([.diagnose]) read counters inside the boolean. The plugin build does not define this macro.
target_compile_definitions (${TARGET_NAME} PRIVATE CSG_DIAGNOSTICS=1)
target_compile_definitions (${TARGET_NAME} PRIVATE CSG_SAMPLE_DATA_DIR="${RbfxCsg_SOURCE_DIR}/SampleData")
# The preset test loads the brush preset models from the CoreData of the engine package.
target_compile_definitions (${TARGET_NAME} PRIVATE CSG_CORE_DATA_DIR="${Urho3D_PACKAGE_ROOT}/bin/CoreData")
# CI sets this option. Then a missing preset model fails the test. The variable CSG_REQUIRE_CORE_DATA does the same at run time.
option (RBFX_CSG_REQUIRE_CORE_DATA "Fail the preset test if a CoreData model is missing" OFF)
if (RBFX_CSG_REQUIRE_CORE_DATA)
    target_compile_definitions (${TARGET_NAME} PRIVATE CSG_REQUIRE_CORE_DATA=1)
endif ()
target_compile_options (${TARGET_NAME} PRIVATE ${CSG_STRICT_FP_OPTIONS})
if (CSG_EXACT_USE_DOUBLE)
    target_compile_definitions (${TARGET_NAME} PRIVATE CSG_EXACT_USE_DOUBLE=1)
endif ()
target_include_directories (${TARGET_NAME} PRIVATE ${RbfxCsg_SOURCE_DIR}/Source ${CMAKE_CURRENT_SOURCE_DIR})

add_test (NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
