// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

/// \file
/// Pure helpers of CsgRoot. The helpers do not depend on Node. Tests call them directly.

#pragma once

#include "Urho3D/CSG/CsgPolygon.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Math/Vector3.h>

namespace Urho3D
{
namespace CsgInternal
{

/// Return the world scale of a root as the metric scale.
/// An axis that is not finite or not greater than 1e-6 gets the value 1.
PLUGIN_RBFXCSG_API Vector3 GetMetricScale(const Vector3& worldScale);

/// Convert a triangulated model from the metric root frame to node space.
/// Positions use the inverse scale. Normals use the scale and become unit length.
/// Tangent and binormal use the inverse scale. The function keeps the w component of the tangent.
PLUGIN_RBFXCSG_API void ToNodeSpace(CsgTriangulatedModel& model, const Vector3& scale);

/// Return the faces of a solid as polygons with the positions multiplied by the factor.
/// The normals use the inverse factor and become unit length. The plane of each polygon is computed again.
/// The polygons keep their shared keys. If the factor is one, the result is a plain copy.
PLUGIN_RBFXCSG_API ea::vector<CsgPolygon> RescalePolygons(const CsgSolid& solid, const Vector3& factor);

/// Return the triangles of a model with the source of each triangle. The positions keep the model space.
PLUGIN_RBFXCSG_API CsgPickTable BuildPickTable(const CsgTriangulatedModel& model);

/// Cast a world ray into the triangles of a table. `worldTransform` maps the table space to world space.
/// Only front faces count. The nearest hit wins. A tie keeps the lower triangle index.
PLUGIN_RBFXCSG_API CsgPickHit PickTriangles(const CsgPickTable& table, const Matrix3x4& worldTransform,
    const Vector3& origin, const Vector3& direction, float maxDistance);

} // namespace CsgInternal
} // namespace Urho3D
