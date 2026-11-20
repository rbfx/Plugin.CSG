// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgOperationColor.h"
#include "Urho3D/CSG/CsgShapes.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/IO/Log.h>

#include <EASTL/algorithm.h>

#include <cmath>

namespace Urho3D
{

CsgProceduralBrush::CsgProceduralBrush(Context* context)
    : CsgBrush(context)
{
    uvMode_ = CsgUvMode::Root;
    SubscribeToEvent(this, E_CSGSHAPEQUERYHANDLES, URHO3D_HANDLER(CsgProceduralBrush, HandleQueryHandles));
    SubscribeToEvent(this, E_CSGSHAPEMOVEHANDLES, URHO3D_HANDLER(CsgProceduralBrush, HandleMoveHandles));
    SubscribeToEvent(this, E_CSGSHAPERECENTER, URHO3D_HANDLER(CsgProceduralBrush, HandleRecenter));
    SubscribeToEvent(this, E_CSGSHAPEQUERYBOUNDS, URHO3D_HANDLER(CsgProceduralBrush, HandleQueryBounds));
    SubscribeToEvent(this, E_CSGSHAPEQUERYSLOT, URHO3D_HANDLER(CsgProceduralBrush, HandleQuerySlot));
}

CsgProceduralBrush::~CsgProceduralBrush() = default;

void CsgProceduralBrush::RegisterObject(Context* context)
{
    // The class is abstract, so it has no factory. The derived types copy these attributes.
    // The attributes are the ones of CsgBrush without "Model". A generated shape has no model resource.
    URHO3D_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);
    URHO3D_ENUM_ACCESSOR_ATTRIBUTE(
        "Operation", GetOperation, SetOperation, CsgOperation, CsgOperationNames, CsgOperation::Union, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Shape Material", GetShapeMaterialAttr, SetShapeMaterialAttr, ResourceRef,
        ResourceRef(Material::GetTypeStatic()), AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Materials", GetMaterialsAttr, SetMaterials, ResourceRefList,
        ResourceRefList(Material::GetTypeStatic()), AM_DEFAULT);
    URHO3D_ENUM_ACCESSOR_ATTRIBUTE(
        "UV Mode", GetUvMode, SetUvMode, CsgUvMode, CsgUvModeNames, CsgUvMode::Root, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("UV Scale", GetUvScale, SetUvScale, float, 1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("UV Offset", GetUvOffset, SetUvOffset, Vector2, Vector2::ZERO, AM_DEFAULT);
}

Model* CsgProceduralBrush::GetShapeModel() const
{
    if (!meshDirty_)
        return model_;

    meshDirty_ = false;
    model_ = nullptr;

    CsgShapeMesh mesh;
    if (GenerateMesh(mesh))
        model_ = CsgBuildModelFromMesh(context_, mesh);

    const bool valid = model_ != nullptr;
    if (!valid && lastValid_)
    {
        URHO3D_LOGWARNING("{}: The shape parameters are not valid. The brush gives no operand.", GetTypeName());
    }
    lastValid_ = valid;
    return model_;
}

bool CsgProceduralBrush::GetGeneratedModel(SharedPtr<Model>& model) const
{
    model = GetShapeModel();
    return true;
}

void CsgProceduralBrush::HandleQueryBounds(StringHash /*eventType*/, VariantMap& eventData)
{
    using namespace CsgShapeQueryBounds;
    const Model* model = GetShapeModel();
    const BoundingBox box = model ? model->GetBoundingBox() : BoundingBox();
    eventData[P_VALID] = model != nullptr;
    eventData[P_MIN] = model ? box.min_ : Vector3::ZERO;
    eventData[P_MAX] = model ? box.max_ : Vector3::ZERO;
}

void CsgProceduralBrush::HandleQuerySlot(StringHash /*eventType*/, VariantMap& eventData)
{
    using namespace CsgShapeQuerySlot;
    ea::optional<unsigned> slot;
    CsgShapeMesh mesh;
    if (IsShapeValid() && GenerateMesh(mesh))
        slot = CsgFindSlotAtPoint(mesh, eventData[P_POSITION].GetVector3(), eventData[P_NORMAL].GetVector3());
    eventData[P_VALID] = slot.has_value();
    eventData[P_SLOT] = slot.value_or(0u);
    eventData[P_SLOTNAME] = slot ? CsgGetSlotName(GetTypeName(), *slot) : EMPTY_STRING;
}

void CsgProceduralBrush::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
{
    // The alpha keeps the wireframe light, as for a model brush.
    Color wireColor = GetOperationColor(static_cast<int>(GetOperation()));
    wireColor.a_ = 0.4f;
    DrawDebugGeometryInternal(debug, depthTest, GetShapeModel(), wireColor);
}

bool CsgProceduralBrush::IsFinite(const Vector3& value)
{
    return std::isfinite(value.x_) && std::isfinite(value.y_) && std::isfinite(value.z_);
}

void CsgProceduralBrush::MoveBoxFace(
    Vector3& size, Vector3& center, unsigned axisIndex, float sign, float change, float minSize, float maxSize)
{
    float& component = axisIndex == 0 ? size.x_ : axisIndex == 1 ? size.y_ : size.z_;
    float& centerComponent = axisIndex == 0 ? center.x_ : axisIndex == 1 ? center.y_ : center.z_;
    const float newSize = Clamp(component + change, minSize, maxSize);
    centerComponent += sign * (newSize - component) * 0.5f;
    component = newSize;
}

bool CsgProceduralBrush::PrepareHandleMove(
    const ea::vector<unsigned>& ids, const Vector3& delta, ea::vector<unsigned>& unique)
{
    unique.clear();
    if (ids.empty() || !IsFinite(delta))
        return false;
    for (const unsigned id : ids)
    {
        if (ea::find(unique.begin(), unique.end(), id) == unique.end())
            unique.push_back(id);
    }
    return true;
}

void CsgProceduralBrush::HandleQueryHandles(StringHash eventType, VariantMap& eventData)
{
    using namespace CsgShapeQueryHandles;
    using namespace CsgShapeProtocol;

    VariantVector list;
    StringVector groupList;
    const bool valid = IsShapeValid();
    if (valid)
    {
        ea::vector<CsgShapeHandle> handles;
        GetHandles(handles);
        for (const CsgShapeHandle& handle : handles)
        {
            VariantMap entry;
            entry[HandleIdKey] = static_cast<int>(handle.id_);
            entry[HandleNameKey] = handle.name_;
            entry[HandleKindKey] = static_cast<int>(handle.kind_);
            entry[HandlePositionKey] = handle.position_;
            entry[HandleAxisKey] = handle.axis_;
            entry[HandleGroupKey] = handle.group_;
            list.push_back(Variant(entry));
        }
    }
    eventData[P_VERSION] = Version;
    eventData[P_VALID] = valid;
    eventData[P_HANDLES] = list;
    ea::vector<ea::string> groups;
    GetHandleGroups(groups);
    for (const ea::string& group : groups)
        groupList.push_back(group);
    eventData[P_GROUPS] = groupList;
    VariantVector cage;
    if (valid)
    {
        ea::vector<ea::pair<unsigned, unsigned>> lines;
        GetCageLines(lines);
        for (const auto& line : lines)
            cage.push_back(Variant(IntVector2(static_cast<int>(line.first), static_cast<int>(line.second))));
    }
    eventData[P_CAGE] = cage;
}

void CsgProceduralBrush::HandleMoveHandles(StringHash eventType, VariantMap& eventData)
{
    using namespace CsgShapeMoveHandles;

    bool accepted = false;
    const auto indices = eventData.find(P_INDICES);
    const auto delta = eventData.find(P_DELTA);
    if (indices != eventData.end() && delta != eventData.end() && indices->second.GetType() == VAR_VARIANTVECTOR
        && delta->second.GetType() == VAR_VECTOR3)
    {
        ea::vector<unsigned> ids;
        bool idsValid = true;
        for (const Variant& value : indices->second.GetVariantVector())
        {
            if (value.GetType() != VAR_INT || value.GetInt() < 0)
            {
                idsValid = false;
                break;
            }
            ids.push_back(static_cast<unsigned>(value.GetInt()));
        }
        if (idsValid)
            accepted = MoveHandles(ids, delta->second.GetVector3());
    }
    eventData[P_ACCEPTED] = accepted;
}

void CsgProceduralBrush::HandleRecenter(StringHash eventType, VariantMap& eventData)
{
    using namespace CsgShapeRecenter;

    Vector3 offset;
    const bool accepted = Recenter(offset);
    eventData[P_ACCEPTED] = accepted;
    eventData[P_OFFSET] = accepted ? offset : Vector3::ZERO;
}

void CsgProceduralBrush::OnParametersChanged()
{
    SanitizeParameters();
    meshDirty_ = true;
    MarkDirty();
}

void CsgProceduralBrush::ApplyAttributes()
{
    SanitizeParameters();
    meshDirty_ = true;
    UpdateMaterialSlots();
    BaseClassName::ApplyAttributes();
}

void CsgProceduralBrush::OnNodeSet(Node* previousNode, Node* currentNode)
{
    // A new brush gets its material slots before the first build.
    UpdateMaterialSlots();
    BaseClassName::OnNodeSet(previousNode, currentNode);
}

void CsgProceduralBrush::SetShapeMaterialAttr(const ResourceRef& value)
{
    shapeMaterial_ = ResourceRef(Material::GetTypeStatic(), value.name_);
    MarkDirty();
}

void CsgProceduralBrush::SetMaterials(const ResourceRefList& value)
{
    // SetModel is not used here: it resizes the materials from the "Model" attribute.
    ResourceRefList resized = value;
    resized.type_ = Material::GetTypeStatic();
    resized.names_.resize(GetNumMaterialSlots());
    BaseClassName::SetMaterials(resized);
}

void CsgProceduralBrush::UpdateMaterialSlots()
{
    const ResourceRefList& materials = GetMaterialsAttr();
    if (materials.names_.size() != GetNumMaterialSlots() || materials.type_ != Material::GetTypeStatic())
        SetMaterials(materials);
}

ea::string ResolveSlotMaterial(const ea::string& shape, const ResourceRefList& faces, unsigned slot)
{
    if (slot < faces.names_.size() && !faces.names_[slot].empty())
        return faces.names_[slot];
    if (!shape.empty())
        return shape;
    return faces.names_.empty() ? ea::string() : faces.names_[0];
}

} // namespace Urho3D
