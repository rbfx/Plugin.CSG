// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Urho3D/Core/Object.h>
#include <Urho3D/Core/Variant.h>
#include <Urho3D/Math/StringHash.h>
#include <Urho3D/Math/Vector3.h>

namespace Urho3D
{

/// The protocol of the CSG root. The header has no link dependency.
/// The editor addon uses it without the CSG library.
///
/// The editor sends the event to the root component with `root->SendEvent(...)`.
/// The root subscribes to the event on itself and writes the results into the event data.
/// A component that is not a root does not answer. Then P_HIT is not in the event data.

/// The current version of the pick event.
constexpr int CsgRootPickVersion = 1;

/// Find the brush that made a face of the applied result.
/// In: P_ORIGIN (Vector3) and P_DIRECTION (Vector3) in world space. The direction can have any length.
/// In: P_MAXDISTANCE (float, optional, default 1e9) in world units.
/// Out: P_VERSION (int), P_HIT (bool), P_SOURCE (unsigned), P_POSITION (Vector3), P_NORMAL (Vector3)
/// and P_DISTANCE (float).
/// P_SOURCE is the node ID of the brush that made the hit face. 0 is the base model or an unknown source.
/// P_POSITION is in world space. P_NORMAL is the unit world normal of the front face of the hit triangle.
/// P_DISTANCE is the distance from the origin to the hit position in world units.
/// The ray hits only front faces: the outward side of the solid, also for a mirrored root. The nearest hit wins.
/// The root uses only the applied result. A disabled root or a root that needs a rebuild still answers.
/// P_HIT is false if no result is applied, the direction is zero or the ray hits no front face.
URHO3D_EVENT(E_CSGROOTPICK, CsgRootPick)
{
    URHO3D_PARAM(P_ORIGIN, Origin); // Vector3
    URHO3D_PARAM(P_DIRECTION, Direction); // Vector3
    URHO3D_PARAM(P_MAXDISTANCE, MaxDistance); // float
    URHO3D_PARAM(P_VERSION, Version); // int
    URHO3D_PARAM(P_HIT, Hit); // bool
    URHO3D_PARAM(P_SOURCE, Source); // unsigned
    URHO3D_PARAM(P_POSITION, Position); // Vector3
    URHO3D_PARAM(P_NORMAL, Normal); // Vector3
    URHO3D_PARAM(P_DISTANCE, Distance); // float
}

} // namespace Urho3D
