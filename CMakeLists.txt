# Copyright (c) 2025-2026 the rbfx project.
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

cmake_minimum_required(VERSION 3.21)

project(RbfxCsg LANGUAGES CXX)

# C++17 is required for this project.
set (CMAKE_CXX_STANDARD 17)

# Setup output directories and data links only for a standalone build.
# A parent project owns these settings when it uses this project as a subdirectory.
if (PROJECT_IS_TOP_LEVEL)
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif ()

# A parent project can build the engine in-tree. Then the target already exists.
# A second find_package call would add the engine a second time.
if (NOT TARGET Urho3D)
    find_package(Urho3D REQUIRED)
endif ()
if (PROJECT_IS_TOP_LEVEL)
    create_symlink(${Urho3D_PACKAGE_ROOT}/bin/CoreData ${CMAKE_BINARY_DIR}/../CoreData)
    create_symlink(${Urho3D_PACKAGE_ROOT}/bin/Data ${CMAKE_BINARY_DIR}/../Data)
    create_symlink(${Urho3D_PACKAGE_ROOT}/bin/EditorData ${CMAKE_BINARY_DIR}/../EditorData)
    create_symlink(${RbfxCsg_SOURCE_DIR}/Project/Data ${CMAKE_BINARY_DIR}/../ProjectData)
endif ()

# Exact CSG arithmetic needs strict floating point rules.
option (CSG_EXACT_USE_DOUBLE "Use double for the exact CSG arithmetic" OFF)
set (CSG_STRICT_FP_OPTIONS
    "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>"
    "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-fno-fast-math>"
    "$<$<CXX_COMPILER_ID:MSVC>:/fp:strict>"
)

# CSG plugin.
add_subdirectory(Source/Urho3D/CSG)

# CSG addon of the editor. It is a static library. It is present only if the editor library is present.
add_subdirectory(Source/Urho3D/CSGEditor)


# CSG tests.
option (RBFXCSG_TESTING "Build CSG tests" ${PROJECT_IS_TOP_LEVEL})
if (RBFXCSG_TESTING AND NOT EMSCRIPTEN AND NOT MOBILE AND NOT UWP AND NOT IOS)
    enable_testing ()
    add_subdirectory (Source/Tests)
endif ()
