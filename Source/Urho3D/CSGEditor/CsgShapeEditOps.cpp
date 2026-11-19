// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSGEditor/CsgShapeEditOps.h"

#include <Urho3D/Core/Attribute.h>
#include <Urho3D/Scene/Component.h>
#include <Urho3D/Scene/Node.h>

#include <cmath>

namespace Urho3D
{

namespace
{

const StringHash ProceduralBrushType("CsgProceduralBrush");

} // namespace

bool IsShapeBrush(Component* component)
{
    const TypeInfo* typeInfo = component ? component->GetTypeInfo() : nullptr;
    return typeInfo && typeInfo->IsTypeOf(ProceduralBrushType);
}

Component* FindShapeBrush(Node* node)
{
    if (!node)
        return nullptr;
    for (Component* component : node->GetComponents())
    {
        if (IsShapeBrush(component))
            return component;
    }
    return nullptr;
}

ShapeHandleSet QueryShapeHandles(Component* brush)
{
    ShapeHandleSet result;
    if (!IsShapeBrush(brush))
        return result;

    using namespace CsgShapeQueryHandles;
    VariantMap& eventData = brush->GetEventDataMap();
    eventData[P_VERSION] = 0;
    eventData[P_VALID] = false;
    eventData[P_HANDLES] = VariantVector{};
    brush->SendEvent(E_CSGSHAPEQUERYHANDLES, eventData);

    // A brush that does not answer keeps version 0.
    if (eventData[P_VERSION].GetInt() != CsgShapeProtocol::Version || !eventData[P_VALID].GetBool())
        return result;

    result.valid_ = true;
    for (const Variant& item : eventData[P_HANDLES].GetVariantVector())
    {
        const VariantMap& map = item.GetVariantMap();
        const auto get = [&](StringHash key) -> const Variant&
        {
            const auto iter = map.find(key);
            return iter != map.end() ? iter->second : Variant::EMPTY;
        };

        ShapeHandle handle;
        handle.id_ = static_cast<unsigned>(get(CsgShapeProtocol::HandleIdKey).GetInt());
        handle.name_ = get(CsgShapeProtocol::HandleNameKey).GetString();
        handle.kind_ = get(CsgShapeProtocol::HandleKindKey).GetInt() == static_cast<int>(CsgShapeHandleKind::Axis)
            ? CsgShapeHandleKind::Axis
            : CsgShapeHandleKind::Point;
        handle.position_ = get(CsgShapeProtocol::HandlePositionKey).GetVector3();
        handle.axis_ = get(CsgShapeProtocol::HandleAxisKey).GetVector3();
        result.handles_.push_back(handle);
    }
    return result;
}

bool MoveShapeHandles(Component* brush, const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    if (!IsShapeBrush(brush) || ids.empty())
        return false;

    using namespace CsgShapeMoveHandles;
    VariantVector indices;
    for (const unsigned id : ids)
        indices.push_back(static_cast<int>(id));

    VariantMap& eventData = brush->GetEventDataMap();
    eventData[P_INDICES] = indices;
    eventData[P_DELTA] = localDelta;
    eventData[P_ACCEPTED] = false;
    brush->SendEvent(E_CSGSHAPEMOVEHANDLES, eventData);
    return eventData[P_ACCEPTED].GetBool();
}

ShapeAttributeSnapshot SnapshotShapeAttributes(Component* component)
{
    ShapeAttributeSnapshot result;
    const ea::vector<AttributeInfo>* attributes = component ? component->GetAttributes() : nullptr;
    if (!attributes)
        return result;

    for (unsigned index = 0; index < attributes->size(); ++index)
    {
        const AttributeInfo& info = attributes->at(index);
        if (!(info.mode_ & AM_FILE))
            continue;
        Variant value;
        info.accessor_->Get(component, value);
        result.emplace_back(info.name_, value);
    }
    return result;
}

bool RestoreShapeAttributes(Component* component, const ShapeAttributeSnapshot& snapshot)
{
    if (!component)
        return false;

    bool changed = false;
    for (const auto& [name, value] : snapshot)
    {
        if (component->GetAttribute(name) != value)
        {
            changed = true;
            break;
        }
    }
    if (!changed)
        return false;

    // All values are set in the snapshot order. A value can depend on an earlier one.
    // For example, the box "Size" scales the corners, and the corners come after it.
    for (const auto& [name, value] : snapshot)
        component->SetAttribute(name, value);
    component->ApplyAttributes();
    return true;
}

bool ApplyShapeHandleDrag(Component* brush, const ShapeAttributeSnapshot& start, const ea::vector<unsigned>& ids,
    const Vector3& delta, ea::optional<Vector3>& acceptedDelta)
{
    if (!brush)
        return false;

    RestoreShapeAttributes(brush, start);
    // The brush refuses a move without a change. A zero delta is the start shape, so it is valid.
    if (delta == Vector3::ZERO)
    {
        acceptedDelta = Vector3::ZERO;
        return true;
    }
    if (MoveShapeHandles(brush, ids, delta))
    {
        acceptedDelta = delta;
        return true;
    }
    if (acceptedDelta && *acceptedDelta != Vector3::ZERO)
        MoveShapeHandles(brush, ids, *acceptedDelta);
    return false;
}

ea::optional<unsigned> PickNearestHandle(
    const ea::vector<HandleScreenPoint>& points, const Vector2& mouse, float radius)
{
    ea::optional<unsigned> best;
    float bestDistance = 0.0f;
    for (unsigned index = 0; index < points.size(); ++index)
    {
        const HandleScreenPoint& point = points[index];
        if (!point.visible_)
            continue;
        const float distance = (point.position_ - mouse).Length();
        if (distance > radius)
            continue;

        bool better = !best;
        if (best)
        {
            const HandleScreenPoint& current = points[*best];
            if (distance < bestDistance - HandlePickTieEpsilon)
                better = true;
            else if (distance <= bestDistance + HandlePickTieEpsilon)
            {
                if (point.depth_ != current.depth_)
                    better = point.depth_ < current.depth_;
                else
                    better = point.id_ < current.id_;
            }
        }
        if (better)
        {
            best = index;
            bestDistance = distance;
        }
    }
    return best;
}

ea::optional<Vector3> ComputeSelectionCentroid(const ea::vector<ShapeHandle>& handles, const ea::vector<unsigned>& ids)
{
    Vector3 sum;
    unsigned count = 0;
    for (const unsigned id : ids)
    {
        for (const ShapeHandle& handle : handles)
        {
            if (handle.id_ == id)
            {
                sum += handle.position_;
                ++count;
                break;
            }
        }
    }
    if (count == 0)
        return ea::nullopt;
    return sum / static_cast<float>(count);
}

float SnapToGrid(float value, float step)
{
    if (!(step > 0.0f))
        return value;
    return std::round(value / step) * step;
}

Vector3 ProjectDeltaOnAxis(const Vector3& delta, const Vector3& axis)
{
    const float lengthSquared = axis.LengthSquared();
    if (lengthSquared <= 0.0f)
        return Vector3::ZERO;
    return axis * (delta.DotProduct(axis) / lengthSquared);
}

Vector3 SnapHandleDelta(const Vector3& start, const Vector3& delta, const Vector3& axis, float step)
{
    if (!(step > 0.0f))
        return delta;

    if (axis.LengthSquared() <= 0.0f)
    {
        const Vector3 target = start + delta;
        const Vector3 snapped{SnapToGrid(target.x_, step), SnapToGrid(target.y_, step), SnapToGrid(target.z_, step)};
        return snapped - start;
    }

    const Vector3 unitAxis = axis.Normalized();
    const float startCoord = start.DotProduct(unitAxis);
    const float targetCoord = SnapToGrid(startCoord + delta.DotProduct(unitAxis), step);
    return unitAxis * (targetCoord - startCoord);
}

} // namespace Urho3D
