// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgPolygon.h"
#include <Urho3D/Math/Matrix3x4.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/span.h>
#include <EASTL/vector.h>

namespace Urho3D
{

/// Make the 6 quads of a box. The winding is counter-clockwise when seen from outside.
ea::vector<CsgPolygon> MakeBoxPolygons(const Vector3& center, const Vector3& halfSize, unsigned shared,
    const Matrix3x4& transform = Matrix3x4::IDENTITY);
/// Make a UV sphere. It has quads, and triangles at the two poles.
ea::vector<CsgPolygon> MakeSpherePolygons(
    const Vector3& center, float radius, unsigned rings, unsigned segments, unsigned shared);
/// Return the signed volume of a closed polygon mesh. Fan triangulation is used.
double SignedVolume(ea::span<const CsgPolygon> polygons);
/// Return true if each directed edge has its opposite edge. Positions are quantized with 1/epsilon.
bool IsEdgeClosed(ea::span<const CsgPolygon> polygons, float epsilon);

} // namespace Urho3D
