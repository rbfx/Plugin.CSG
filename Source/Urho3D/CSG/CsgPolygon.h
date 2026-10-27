// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Math/Plane.h>

#include <EASTL/vector.h>

namespace Urho3D
{

struct PLUGIN_RBFXCSG_API CsgPolygon
{
    ea::vector<ModelVertex> vertices_;
    unsigned shared_{};  // Canonical representative of a compatible (sourceMeshId << 16) | sourceGeometryIndex group.
    Plane plane_;

    CsgPolygon();
    CsgPolygon(ea::vector<ModelVertex>&& verts, unsigned sharedKey);

    CsgPolygon(CsgPolygon&&);
    CsgPolygon& operator=(CsgPolygon&&);

    CsgPolygon(const CsgPolygon&);
    CsgPolygon& operator=(const CsgPolygon&);

    void Flip();
};

} // namespace Urho3D
