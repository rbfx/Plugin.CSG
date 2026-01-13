// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include <Urho3D/Plugins/PluginApplication.h>

#include "Urho3D/CSG/Csg.h"

using namespace Urho3D;

namespace
{

void RegisterPluginObjects(PluginApplication& plugin)
{
    RegisterCsgLibrary(plugin);
}

void UnregisterPluginObjects(PluginApplication& plugin)
{
}

} // namespace

URHO3D_DEFINE_PLUGIN_MAIN_SIMPLE(RegisterPluginObjects, UnregisterPluginObjects);
