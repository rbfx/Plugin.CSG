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
    /// The group (adjustment mode) of the handle. It is one of the strings of `P_GROUPS`.
    /// The name 'All' is reserved for the editor. A shape must not use it.
    ea::string group_;
};

namespace CsgShapeProtocol
{

/// The version of the protocol.
/// Version 2 adds `E_CSGSHAPERECENTER`. The handle ids do not change.
/// The handle key `HandleGroupKey` and the reply parameter `P_GROUPS` are optional additions.
/// They do not change the version. A reader ignores a key that it does not know.
inline constexpr int Version = 2;

/// The keys of the VariantMap of one handle in `P_HANDLES`.
/// @{
inline const StringHash HandleIdKey{"Id"};
inline const StringHash HandleNameKey{"Name"};
/// Int. 0 is a point and 1 is an axis. See `CsgShapeHandleKind`.
inline const StringHash HandleKindKey{"Kind"};
inline const StringHash HandlePositionKey{"Position"};
inline const StringHash HandleAxisKey{"Axis"};
/// String. The group of the handle. See `CsgShapeHandle::group_`. A reader treats a missing key as no group.
inline const StringHash HandleGroupKey{"Group"};
/// @}

} // namespace CsgShapeProtocol

/// Query the handles of a shape.
/// Out: P_VERSION (int), P_VALID (bool) and P_HANDLES (VariantVector of VariantMap, see the keys above).
/// If the shape is not valid, P_VALID is false and P_HANDLES is empty.
/// Optional out: P_GROUPS (StringVector). The groups in the order of the mode bar.
/// The first entry is the default mode. The list is fixed for a shape type, also if a handle is not present.
/// An old runtime does not write P_GROUPS and the Group keys. Then the editor shows all handles.
URHO3D_EVENT(E_CSGSHAPEQUERYHANDLES, CsgShapeQueryHandles)
{
    URHO3D_PARAM(P_VERSION, Version); // int
    URHO3D_PARAM(P_VALID, Valid); // bool
    URHO3D_PARAM(P_HANDLES, Handles); // VariantVector
    URHO3D_PARAM(P_GROUPS, Groups); // StringVector
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

/// Move the shape parameters so the shape is centered around the origin of the brush.
/// The shape in the local space moves by -P_OFFSET. The caller moves the node so the world shape stays.
/// Out: P_ACCEPTED (bool) and P_OFFSET (Vector3).
/// P_OFFSET is the amount in the local space of the brush that the shape center was away from the origin.
/// P_ACCEPTED is false if the shape is already centered, is always centered, or the new parameters are not valid.
/// Then the parameters stay as they are. The handle ids do not change.
URHO3D_EVENT(E_CSGSHAPERECENTER, CsgShapeRecenter)
{
    URHO3D_PARAM(P_ACCEPTED, Accepted); // bool
    URHO3D_PARAM(P_OFFSET, Offset); // Vector3
}

/// Query the local bounding box of the generated mesh of a shape. The event does not change the protocol version.
/// Out: P_VALID (bool), P_MIN (Vector3) and P_MAX (Vector3). The box is in the local space of the brush, before the
/// node scale. P_VALID is false if the shape is not valid. Then P_MIN and P_MAX are zero.
URHO3D_EVENT(E_CSGSHAPEQUERYBOUNDS, CsgShapeQueryBounds)
{
    URHO3D_PARAM(P_VALID, Valid); // bool
    URHO3D_PARAM(P_MIN, Min); // Vector3
    URHO3D_PARAM(P_MAX, Max); // Vector3
}

} // namespace Urho3D
