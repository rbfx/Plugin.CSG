// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Urho3D/Container/Str.h>
#include <Urho3D/Core/Object.h>
#include <Urho3D/Core/Variant.h>
#include <Urho3D/Math/StringHash.h>
#include <Urho3D/Math/Vector3.h>

namespace Urho3D
{

/// The protocol of the shape handles. A handle is a point that the user drags in the viewport.
/// The header has no link dependency. The editor addon uses it without the CSG library.
///
/// The editor sends the events to the brush component with `component->SendEvent(...)`.
/// A procedural brush subscribes to both events on itself and writes the results into the event data.
/// A component that is not a procedural brush does not answer. Then the event data keeps its input.
/// All positions and axes are in the local space of the brush.

/// The kind of a handle.
enum class CsgShapeHandleKind
{
    /// The handle moves freely. `axis_` is zero.
    Point = 0,
    /// The handle moves along `axis_`. The drag uses the part of the delta along the axis.
    Axis = 1,
};

/// A handle of a shape.
struct CsgShapeHandle
{
    /// The stable id. It is a file contract for each shape type.
    unsigned id_{};
    ea::string name_;
    CsgShapeHandleKind kind_{CsgShapeHandleKind::Point};
    /// The position in the local space of the brush.
    Vector3 position_;
    /// The unit axis in the local space of the brush. Zero for a point.
    Vector3 axis_;
};

namespace CsgShapeProtocol
{

/// The version of the protocol.
inline constexpr int Version = 1;

/// The keys of the VariantMap of one handle in `P_HANDLES`.
/// @{
inline const StringHash HandleIdKey{"Id"};
inline const StringHash HandleNameKey{"Name"};
/// Int. 0 is a point and 1 is an axis. See `CsgShapeHandleKind`.
inline const StringHash HandleKindKey{"Kind"};
inline const StringHash HandlePositionKey{"Position"};
inline const StringHash HandleAxisKey{"Axis"};
/// @}

} // namespace CsgShapeProtocol

/// Query the handles of a shape.
/// Out: P_VERSION (int), P_VALID (bool) and P_HANDLES (VariantVector of VariantMap, see the keys above).
/// If the shape is not valid, P_VALID is false and P_HANDLES is empty.
URHO3D_EVENT(E_CSGSHAPEQUERYHANDLES, CsgShapeQueryHandles)
{
    URHO3D_PARAM(P_VERSION, Version); // int
    URHO3D_PARAM(P_VALID, Valid); // bool
    URHO3D_PARAM(P_HANDLES, Handles); // VariantVector
}

/// Move handles of a shape.
/// In: P_INDICES (VariantVector of int handle ids) and P_DELTA (Vector3).
/// The delta is the TOTAL delta from the drag start, in the local space of the brush.
/// The editor restores the attributes before each call, so the call is stateless.
/// Out: P_ACCEPTED (bool). It is false if an id is not known, the delta is not finite,
/// the result is not valid or the parameters do not change. Then the parameters stay as they are.
URHO3D_EVENT(E_CSGSHAPEMOVEHANDLES, CsgShapeMoveHandles)
{
    URHO3D_PARAM(P_INDICES, Indices); // VariantVector
    URHO3D_PARAM(P_DELTA, Delta); // Vector3
    URHO3D_PARAM(P_ACCEPTED, Accepted); // bool
}

} // namespace Urho3D
