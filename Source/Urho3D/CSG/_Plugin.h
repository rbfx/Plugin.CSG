// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Urho3D/Urho3D.h>

#ifdef URHO3D_STATIC
#define PLUGIN_RBFXCSG_API
#else
#if Plugin_RbfxCsg_EXPORT
#define PLUGIN_RBFXCSG_API URHO3D_EXPORT_API
#else
#define PLUGIN_RBFXCSG_API URHO3D_IMPORT_API
#endif
#endif
