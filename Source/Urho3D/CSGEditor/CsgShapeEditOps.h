// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgShapeProtocol.h"

#include <Urho3D/Core/Variant.h>
#include <Urho3D/Math/Ray.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>

#include <Urho3D/Container/Str.h>

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
    /// The distance from the camera. A tie of the screen distance goes to the nearer handle.
    float depth_{};
};

/// The screen distance in pixels below which two handles are a tie for the pick.
inline constexpr float HandlePickTieEpsilon = 1.0f;

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

/// The attributes of a brush and the position of its node. An undo step keeps two of them.
struct ShapeEditState
{
    ShapeAttributeSnapshot attributes_;
    Vector3 nodePosition_;

    bool operator==(const ShapeEditState& rhs) const
    {
        return attributes_ == rhs.attributes_ && nodePosition_ == rhs.nodePosition_;
    }
    bool operator!=(const ShapeEditState& rhs) const { return !(*this == rhs); }
};

/// Return the attributes of the component and the local position of its node.
ShapeEditState CaptureShapeEditState(Component* component);
/// Set the attributes (one ApplyAttributes call) and the local position of the node.
/// Return false and change nothing if the state is already the current one.
bool RestoreShapeEditState(Component* component, const ShapeEditState& state);

/// Move the shape parameters so the shape is centered around the origin of the node. It uses the event
/// E_CSGSHAPERECENTER. Then it moves the node by `node->GetTransform() * offset`, so the world shape stays.
/// Only the local transform of the node changes. Return false and change nothing if the brush
/// does not answer, the shape is already centered, or the new parameters are not valid.
bool RecenterShapePivot(Component* brush);

/// Apply one frame of a handle drag. Restore the start snapshot, then move by the total delta.
/// A zero delta is the start shape. It is accepted with no move call.
/// A refused move applies the last accepted delta again, so the shape keeps the last valid state.
/// Update `acceptedDelta` and return true if the delta is accepted.
bool ApplyShapeHandleDrag(Component* brush, const ShapeAttributeSnapshot& start, const ea::vector<unsigned>& ids,
    const Vector3& delta, ea::optional<Vector3>& acceptedDelta);

/// The result of a pick of the root result.
struct PickResult
{
    /// True if the ray hits a front face of the applied result.
    bool hit_{};
    /// The node ID of the brush that made the face. 0 is the base model or an unknown source.
    unsigned source_{};
    /// The hit position and the unit normal in world space.
    Vector3 position_;
    Vector3 normal_;
};

/// Pick the result of the root component with the world ray. It uses the event E_CSGROOTPICK.
/// A null root or a root that does not answer gives no hit.
PickResult PickRootSource(Component* root, const Ray& worldRay);

/// Choose the face handle for a hit on the face of a brush. The point and the normal are in the local
/// space of the brush. The normal is a unit vector. Only Axis handles count.
/// An Axis handle matches if |dot(axis, normal)| > 0.9. The sign does not matter: a subtract brush
/// seen from inside has a flipped normal. Among the matches, the handle with the smallest
/// |dot(point - handle.position, axis)| wins. A tie goes to the lower id. No match gives none.
/// A sphere has no face. Its radius handle on an axis matches an axis-aligned normal by the same rule.
ea::optional<unsigned> ChooseFaceHandle(
    const ea::vector<ShapeHandle>& handles, const Vector3& localPoint, const Vector3& localNormal);

/// Convert a world hit on the brush node to its local space and call ChooseFaceHandle.
/// The point uses the inverse world transform. The normal uses the transpose of the world 3x3 matrix, so it
/// stays a normal for a non-uniform scale. It is normalized again. A zero normal gives none.
ea::optional<unsigned> ChooseFaceHandleForWorldHit(
    const Node* brushNode, const ea::vector<ShapeHandle>& handles, const Vector3& worldPoint, const Vector3& worldNormal);

/// Return the index in `points` of the nearest visible point in the radius, or none.
/// The nearest point wins. Screen distances within HandlePickTieEpsilon are a tie.
/// A tie goes to the smaller depth, then to the lower id.
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

/// The diagnostics of a root. See `E_CSGROOTQUERYSTATS`.
struct RootStats
{
    bool valid_{};
    unsigned brushCount_{};
    unsigned triangles_{};
    unsigned vertices_{};
    float rebuildMs_{};
    unsigned reusedOperands_{};
    bool building_{};
    /// The reason of an invalid result. Empty for a valid result.
    ea::string error_;
};

/// Read the diagnostics of the root component with the event E_CSGROOTQUERYSTATS.
/// Return false for a null component or a component that does not answer.
bool QueryRootStats(Component* root, RootStats& stats);

/// Return the line of the overlay: "Brushes N, tris N, last build X ms, reused K/N".
ea::string FormatRootStatsLine(const RootStats& stats);

/// Compute the size of the generated mesh of a procedural brush: the local axis aligned bounds of the mesh
/// scaled by the world scale of the node. The size is not negative. It uses the event E_CSGSHAPEQUERYBOUNDS.
/// Return false for a null component, a component that does not answer or a shape that is not valid.
bool ComputeShapeWorldSize(Component* brush, Vector3& size);

} // namespace Urho3D
