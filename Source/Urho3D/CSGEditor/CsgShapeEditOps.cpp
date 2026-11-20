// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSGEditor/CsgShapeEditOps.h"

#include "Urho3D/CSG/CsgRootProtocol.h"

#include <Urho3D/Core/Attribute.h>
#include <Urho3D/Core/Format.h>
#include <Urho3D/Scene/Component.h>
#include <Urho3D/Scene/Node.h>

#include <EASTL/sort.h>

#include <cmath>

namespace Urho3D
{

namespace
{

const StringHash ProceduralBrushType("CsgProceduralBrush");

/// Read the group list of the reply. A missing or wrong value gives an empty list. An empty name is skipped.
ea::vector<ea::string> ParseHandleGroups(const Variant& value)
{
    ea::vector<ea::string> groups;
    if (value.GetType() != VAR_STRINGVECTOR)
        return groups;
    for (const ea::string& group : value.GetStringVector())
    {
        if (!group.empty())
            groups.push_back(group);
    }
    return groups;
}

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
    eventData[P_GROUPS] = StringVector{};
    eventData[P_CAGE] = VariantVector{};
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
        handle.group_ = get(CsgShapeProtocol::HandleGroupKey).GetString();
        result.handles_.push_back(handle);
    }
    result.groups_ = ParseHandleGroups(eventData[P_GROUPS]);
    for (const Variant& item : eventData[P_CAGE].GetVariantVector())
    {
        if (item.GetType() != VAR_INTVECTOR2)
            continue;
        const IntVector2 pair = item.GetIntVector2();
        if (pair.x_ >= 0 && pair.y_ >= 0)
            result.cage_.emplace_back(static_cast<unsigned>(pair.x_), static_cast<unsigned>(pair.y_));
    }
    return result;
}

ea::vector<ea::pair<Vector3, Vector3>> ResolveCageSegments(const ShapeHandleSet& set)
{
    ea::vector<ea::pair<Vector3, Vector3>> segments;
    const auto find = [&](unsigned id) -> const ShapeHandle*
    {
        for (const ShapeHandle& handle : set.handles_)
        {
            if (handle.id_ == id)
                return &handle;
        }
        return nullptr;
    };
    for (const auto& line : set.cage_)
    {
        const ShapeHandle* first = find(line.first);
        const ShapeHandle* second = find(line.second);
        if (first && second)
            segments.emplace_back(first->position_, second->position_);
    }
    return segments;
}

ea::vector<ea::pair<Vector2, Vector2>> DashSegments(const Vector2& a, const Vector2& b, float on, float off)
{
    ea::vector<ea::pair<Vector2, Vector2>> dashes;
    const float length = (b - a).Length();
    if (!(length > 0.0f) || !(on > 0.0f) || !std::isfinite(length))
        return dashes;
    if (!(off > 0.0f))
        off = 0.0f;
    const Vector2 direction = (b - a) / length;
    for (float start = 0.0f; start < length; start += on + off)
    {
        const float end = ea::min(start + on, length);
        dashes.emplace_back(a + direction * start, end >= length ? b : a + direction * end);
        if (on + off <= 0.0f)
            break;
    }
    return dashes;
}

ea::vector<ea::string> GetHandleModes(const ShapeHandleSet& set)
{
    ea::vector<ea::string> modes;
    if (set.groups_.size() < 2)
        return modes;
    modes = set.groups_;
    modes.push_back(HandleModeAll);
    return modes;
}

ea::string ResolveHandleMode(const ea::vector<ea::string>& modes, const ea::string& remembered)
{
    if (modes.empty())
        return {};
    return ea::find(modes.begin(), modes.end(), remembered) != modes.end() ? remembered : modes.front();
}

ea::string CycleHandleMode(const ea::vector<ea::string>& modes, const ea::string& current, int step)
{
    if (modes.empty())
        return {};
    const int count = static_cast<int>(modes.size());
    const auto iter = ea::find(modes.begin(), modes.end(), current);
    const int index = iter != modes.end() ? static_cast<int>(iter - modes.begin()) : 0;
    return modes[(((index + step) % count) + count) % count];
}

ea::vector<ShapeHandle> FilterHandlesByMode(const ea::vector<ShapeHandle>& handles, const ea::string& mode)
{
    if (mode.empty() || mode == HandleModeAll)
        return handles;
    ea::vector<ShapeHandle> result;
    for (const ShapeHandle& handle : handles)
    {
        if (handle.group_ == mode)
            result.push_back(handle);
    }
    return result;
}

ea::string FormatHandleModesIni(const ea::unordered_map<ea::string, ea::string>& modes)
{
    ea::vector<ea::pair<ea::string, ea::string>> entries;
    for (const auto& [type, mode] : modes)
    {
        if (!type.empty() && !mode.empty())
            entries.emplace_back(type, mode);
    }
    ea::sort(entries.begin(), entries.end());
    ea::string result;
    for (const auto& [type, mode] : entries)
    {
        if (!result.empty())
            result += ';';
        result += type + "=" + mode;
    }
    return result;
}

ea::unordered_map<ea::string, ea::string> ParseHandleModesIni(const ea::string& text)
{
    ea::unordered_map<ea::string, ea::string> result;
    unsigned start = 0;
    while (start <= text.size())
    {
        unsigned end = text.find(';', start);
        if (end == ea::string::npos)
            end = text.size();
        const ea::string entry = text.substr(start, end - start);
        const unsigned equals = entry.find('=');
        if (equals != ea::string::npos)
        {
            const ea::string type = entry.substr(0, equals).trimmed();
            const ea::string mode = entry.substr(equals + 1).trimmed();
            if (!type.empty() && !mode.empty())
                result[type] = mode;
        }
        start = end + 1;
    }
    return result;
}

void HandleSelectionParking::Switch(ea::vector<unsigned>& active, const ea::string& from, const ea::string& to)
{
    if (from == to)
        return;
    if (active.empty())
        parked_.erase(from);
    else
        parked_[from] = active;
    active.clear();
    const auto iter = parked_.find(to);
    if (iter != parked_.end())
    {
        active = iter->second;
        parked_.erase(iter);
    }
}

const ea::vector<unsigned>* HandleSelectionParking::Find(const ea::string& mode) const
{
    const auto iter = parked_.find(mode);
    return iter != parked_.end() ? &iter->second : nullptr;
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

ShapeEditState CaptureShapeEditState(Component* component)
{
    ShapeEditState result;
    result.attributes_ = SnapshotShapeAttributes(component);
    if (component && component->GetNode())
        result.nodePosition_ = component->GetNode()->GetPosition();
    return result;
}

bool RestoreShapeEditState(Component* component, const ShapeEditState& state)
{
    if (!component)
        return false;
    bool changed = RestoreShapeAttributes(component, state.attributes_);
    Node* node = component->GetNode();
    if (node && node->GetPosition() != state.nodePosition_)
    {
        node->SetPosition(state.nodePosition_);
        changed = true;
    }
    return changed;
}

bool RecenterShapePivot(Component* brush)
{
    Node* node = brush ? brush->GetNode() : nullptr;
    if (!IsShapeBrush(brush) || !node)
        return false;

    using namespace CsgShapeRecenter;
    VariantMap& eventData = brush->GetEventDataMap();
    eventData[P_ACCEPTED] = false;
    eventData[P_OFFSET] = Vector3::ZERO;
    brush->SendEvent(E_CSGSHAPERECENTER, eventData);
    if (!eventData[P_ACCEPTED].GetBool())
        return false;

    // The shape moved by -offset in the local space. The node moves by its own local transform of the offset.
    const Vector3 offset = eventData[P_OFFSET].GetVector3();
    node->SetPosition(node->GetTransform() * offset);
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

PickResult PickRootSource(Component* root, const Ray& worldRay)
{
    PickResult result;
    if (!root)
        return result;

    using namespace CsgRootPick;
    VariantMap& eventData = root->GetEventDataMap();
    eventData[P_ORIGIN] = worldRay.origin_;
    eventData[P_DIRECTION] = worldRay.direction_;
    eventData[P_VERSION] = 0;
    eventData[P_HIT] = false;
    root->SendEvent(E_CSGROOTPICK, eventData);

    // A root that does not answer keeps version 0.
    if (eventData[P_VERSION].GetInt() == 0 || !eventData[P_HIT].GetBool())
        return result;
    result.hit_ = true;
    result.source_ = eventData[P_SOURCE].GetUInt();
    result.position_ = eventData[P_POSITION].GetVector3();
    result.normal_ = eventData[P_NORMAL].GetVector3();
    return result;
}

ea::optional<unsigned> ChooseFaceHandle(
    const ea::vector<ShapeHandle>& handles, const Vector3& localPoint, const Vector3& localNormal)
{
    constexpr float minAlignment = 0.9f;
    ea::optional<unsigned> best;
    float bestDistance = M_INFINITY;
    for (const ShapeHandle& handle : handles)
    {
        if (handle.kind_ != CsgShapeHandleKind::Axis)
            continue;
        if (Abs(handle.axis_.DotProduct(localNormal)) <= minAlignment)
            continue;
        const float distance = Abs((localPoint - handle.position_).DotProduct(handle.axis_));
        if (distance < bestDistance || (distance == bestDistance && handle.id_ < *best))
        {
            best = handle.id_;
            bestDistance = distance;
        }
    }
    return best;
}

ea::optional<unsigned> ChooseFaceHandleForWorldHit(
    const Node* brushNode, const ea::vector<ShapeHandle>& handles, const Vector3& worldPoint, const Vector3& worldNormal)
{
    if (!brushNode)
        return ea::nullopt;
    const Vector3 localPoint = brushNode->WorldToLocal(worldPoint);
    // A normal uses the transpose of the world 3x3 matrix M = R * S: M^T n = S R^T n.
    // The inverse (WorldToLocal) is right only for a direction, not for a normal.
    Vector3 localNormal = brushNode->GetWorldTransform().ToMatrix3().Transpose() * worldNormal;
    if (localNormal.LengthSquared() <= 0.0f)
        return ea::nullopt;
    localNormal.Normalize();
    return ChooseFaceHandle(handles, localPoint, localNormal);
}

bool QueryRootStats(Component* root, RootStats& stats)
{
    if (!root)
        return false;

    using namespace CsgRootQueryStats;
    VariantMap& eventData = root->GetEventDataMap();
    eventData[P_VERSION] = 0;
    root->SendEvent(E_CSGROOTQUERYSTATS, eventData);
    // A component that does not answer keeps version 0.
    if (eventData[P_VERSION].GetInt() == 0)
        return false;

    stats.valid_ = eventData[P_VALID].GetBool();
    stats.brushCount_ = eventData[P_BRUSHCOUNT].GetUInt();
    stats.triangles_ = eventData[P_TRIANGLES].GetUInt();
    stats.vertices_ = eventData[P_VERTICES].GetUInt();
    stats.rebuildMs_ = eventData[P_REBUILDMS].GetFloat();
    stats.reusedOperands_ = eventData[P_REUSEDOPERANDS].GetUInt();
    stats.building_ = eventData[P_BUILDING].GetBool();
    stats.dirty_ = eventData[P_DIRTY].GetBool();
    stats.error_ = eventData[P_ERROR].GetString();
    return true;
}

ea::string FormatRootStatsLine(const RootStats& stats)
{
    return Format("Brushes {}, tris {}, last build {:.1f} ms, reused {}/{}", stats.brushCount_, stats.triangles_,
        stats.rebuildMs_, stats.reusedOperands_, stats.brushCount_);
}

bool ComputeShapeWorldSize(Component* brush, Vector3& size)
{
    if (!brush || !brush->GetNode())
        return false;

    using namespace CsgShapeQueryBounds;
    VariantMap& eventData = brush->GetEventDataMap();
    eventData[P_VALID] = false;
    brush->SendEvent(E_CSGSHAPEQUERYBOUNDS, eventData);
    if (!eventData[P_VALID].GetBool())
        return false;

    const Vector3 localSize = eventData[P_MAX].GetVector3() - eventData[P_MIN].GetVector3();
    size = localSize * brush->GetNode()->GetWorldScale().Abs();
    return true;
}

} // namespace Urho3D
