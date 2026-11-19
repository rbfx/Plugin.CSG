# Copyright (c) 2025-2026 the rbfx project.
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

# Plugin.CSG.Editor: the CSG addon of the scene view. It needs the editor library.
# A custom editor links it and registers RbfxCsg_EditorAddon with EditorPluginManager::AddPlugin.
# It does not link Plugin.CSG. It finds the CSG types by name at run time.
if (NOT TARGET EditorLibrary)
    return ()
endif ()

add_library (Plugin.CSG.Editor STATIC
    CsgEditorAddon.cpp
    CsgEditorAddon.h
    CsgEditorOps.cpp
    CsgEditorOps.h
    CsgShapeEditOps.cpp
    CsgShapeEditOps.h
)
target_link_libraries (Plugin.CSG.Editor PUBLIC EditorLibrary)
target_include_directories (Plugin.CSG.Editor PUBLIC $<BUILD_INTERFACE:${RbfxCsg_SOURCE_DIR}/Source>)
target_compile_features (Plugin.CSG.Editor PRIVATE cxx_std_17)
