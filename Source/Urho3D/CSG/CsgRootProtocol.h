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

/// The current version of the stats event.
constexpr int CsgRootQueryStatsVersion = 1;

/// Read the diagnostics of the root.
/// In: none. The editor sends the event to the root component.
/// Out: P_VERSION (int), P_VALID (bool), P_BRUSHCOUNT (unsigned), P_TRIANGLES (unsigned), P_VERTICES (unsigned),
/// P_REBUILDMS (float), P_REUSEDOPERANDS (unsigned), P_BUILDING (bool), P_DIRTY (bool) and P_ERROR (string).
/// The numbers describe the last applied build. A root that built nothing has zeros and P_VALID false.
/// P_VALID is the valid flag of the fold result. P_BRUSHCOUNT is the number of operands of that build.
/// P_TRIANGLES and P_VERTICES are the sizes of the applied result model.
/// P_REBUILDMS is the job time in milliseconds: boolean, triangulation and model build.
/// P_REUSEDOPERANDS is the number of operands that the build took from the cached fold states (the prefix).
/// P_BUILDING is true while a build job runs.
/// P_DIRTY is true while a rebuild is requested or pending and the result is not applied yet.
/// P_ERROR is empty for a valid result.
/// Else it tells why: a failed build, an open solid, open edges, duplicate or degenerate triangles, or no volume.
/// The root does not know which brush made the problem, so the event has no brush ID.
/// A component that is not a root does not answer. Then P_VERSION is not in the event data.
URHO3D_EVENT(E_CSGROOTQUERYSTATS, CsgRootQueryStats)
{
    URHO3D_PARAM(P_VERSION, Version); // int
    URHO3D_PARAM(P_VALID, Valid); // bool
    URHO3D_PARAM(P_BRUSHCOUNT, BrushCount); // unsigned
    URHO3D_PARAM(P_TRIANGLES, Triangles); // unsigned
    URHO3D_PARAM(P_VERTICES, Vertices); // unsigned
    URHO3D_PARAM(P_REBUILDMS, RebuildMs); // float
    URHO3D_PARAM(P_REUSEDOPERANDS, ReusedOperands); // unsigned
    URHO3D_PARAM(P_BUILDING, Building); // bool
    URHO3D_PARAM(P_DIRTY, Dirty); // bool
    URHO3D_PARAM(P_ERROR, Error); // string
}

} // namespace Urho3D
