// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

/// \file
/// Pure helpers of CsgRoot. The helpers do not depend on Node. Tests call them directly.

#pragma once

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

} // namespace CsgInternal
} // namespace Urho3D
