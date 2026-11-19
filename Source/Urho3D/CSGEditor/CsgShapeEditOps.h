// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgShapeProtocol.h"

#include <Urho3D/Core/Variant.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/optional.h>
#include <EASTL/utility.h>
#include <EASTL/vector.h>

/// The functions of the shape handles in the editor. They use no ImGui.
/// They use only reflection and the events of `CsgShapeProtocol.h`. They do not link Plugin.CSG.

namespace Urho3D
{

class Component;
class Node;

/// A handle of a shape. All positions and axes are in the local space of the brush.
using ShapeHandle = CsgShapeHandle;

/// The result of a handle query.
struct ShapeHandleSet
{
    /// False if the component does not answer or the shape is not valid.
    bool valid_{};
    ea::vector<ShapeHandle> handles_;
};

/// The serializable attributes of a component in registration order.
using ShapeAttributeSnapshot = ea::vector<ea::pair<ea::string, Variant>>;

/// The screen position of a handle for the pick.
struct HandleScreenPoint
{
    unsigned id_{};
    Vector2 position_;
    /// False if the handle is behind the camera. The pick skips it.
    bool visible_{true};
};

/// Return true if the type of the component derives from "CsgProceduralBrush".
bool IsShapeBrush(Component* component);
/// Return the first shape brush component of the node, or null.
Component* FindShapeBrush(Node* node);

/// Query the handles of the brush with the event E_CSGSHAPEQUERYHANDLES.
ShapeHandleSet QueryShapeHandles(Component* brush);
/// Move the handles with the event E_CSGSHAPEMOVEHANDLES. The delta is in the local space of the brush.
/// Return the P_ACCEPTED answer. A component that does not answer gives false.
bool MoveShapeHandles(Component* brush, const ea::vector<unsigned>& ids, const Vector3& localDelta);

/// Return all attributes of the component with the AM_FILE mode.
ShapeAttributeSnapshot SnapshotShapeAttributes(Component* component);
/// Set the attributes of the snapshot and call ApplyAttributes once.
/// Return false and change nothing if all values are already equal.
bool RestoreShapeAttributes(Component* component, const ShapeAttributeSnapshot& snapshot);

/// Return the index in `points` of the nearest visible point in the radius, or none.
/// The nearest point wins. Equal distances go to the lower id.
ea::optional<unsigned> PickNearestHandle(
    const ea::vector<HandleScreenPoint>& points, const Vector2& mouse, float radius);
/// Return the mean position of the handles with the ids. Unknown ids are skipped. No handle gives none.
ea::optional<Vector3> ComputeSelectionCentroid(const ea::vector<ShapeHandle>& handles, const ea::vector<unsigned>& ids);
/// Return the value rounded to the nearest multiple of the step. A step of 0 or less returns the value.
float SnapToGrid(float value, float step);
/// Return the part of the delta along the axis. The axis need not be a unit vector. A zero axis gives zero.
Vector3 ProjectDeltaOnAxis(const Vector3& delta, const Vector3& axis);
/// Return the delta that puts `start + delta` on the grid.
/// A zero axis snaps each component. A non-zero axis snaps only the coordinate along the axis.
/// Then the result is along the axis. A step of 0 or less returns the delta.
Vector3 SnapHandleDelta(const Vector3& start, const Vector3& delta, const Vector3& axis, float step);

} // namespace Urho3D
