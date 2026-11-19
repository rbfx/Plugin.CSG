// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgWedgeBrush.h"
#include <Urho3D/Core/Context.h>

namespace Urho3D
{

CsgWedgeBrush::CsgWedgeBrush(Context* context)
    : CsgProceduralBrush(context)
{
}

CsgWedgeBrush::~CsgWedgeBrush() = default;

void CsgWedgeBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgWedgeBrush>(Category_Scene);

    const CsgWedgeParams defaults;
    URHO3D_COPY_BASE_ATTRIBUTES(CsgProceduralBrush);
    URHO3D_ACCESSOR_ATTRIBUTE("Size", GetSize, SetSize, Vector3, defaults.size_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Center", GetCenter, SetCenter, Vector3, defaults.center_, AM_DEFAULT);
}

void CsgWedgeBrush::SetSize(const Vector3& size)
{
    CsgWedgeParams params = params_;
    params.size_ = size;
    SetParams(params);
}

void CsgWedgeBrush::SetCenter(const Vector3& center)
{
    CsgWedgeParams params = params_;
    params.center_ = center;
    SetParams(params);
}

void CsgWedgeBrush::SetParams(const CsgWedgeParams& params)
{
    CsgWedgeParams sanitized = params;
    CsgSanitizeWedgeParams(sanitized);
    if (sanitized.size_ == params_.size_ && sanitized.center_ == params_.center_)
        return;

    params_ = sanitized;
    OnParametersChanged();
}

namespace
{

/// The axis index and the face sign of each handle id.
const unsigned WedgeHandleAxis[6] = {0, 0, 2, 2, 1, 1};
const float WedgeHandleSign[6] = {-1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f};

} // namespace

void CsgWedgeBrush::GetHandles(ea::vector<CsgShapeHandle>& handles) const
{
    handles.clear();
    const Vector3 h = params_.size_ * 0.5f;
    const Vector3& c = params_.center_;

    const auto add = [&](unsigned id, const char* name, const Vector3& offset, const Vector3& axis)
    {
        CsgShapeHandle handle;
        handle.id_ = id;
        handle.name_ = name;
        handle.kind_ = CsgShapeHandleKind::Axis;
        handle.position_ = c + offset;
        handle.axis_ = axis;
        handles.push_back(handle);
    };
    add(0, "Left", Vector3(-h.x_, -h.y_ / 3.0f, h.z_ / 3.0f), Vector3::LEFT);
    add(1, "Right", Vector3(h.x_, -h.y_ / 3.0f, h.z_ / 3.0f), Vector3::RIGHT);
    add(2, "Front", Vector3(0.0f, -h.y_, -h.z_), Vector3::BACK);
    add(3, "Back", Vector3(0.0f, 0.0f, h.z_), Vector3::FORWARD);
    add(4, "Top", Vector3(0.0f, h.y_, h.z_), Vector3::UP);
    add(5, "Bottom", Vector3(0.0f, -h.y_, 0.0f), Vector3::DOWN);
}

bool CsgWedgeBrush::MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    ea::vector<unsigned> unique;
    if (!PrepareHandleMove(ids, localDelta, unique))
        return false;

    ea::vector<CsgShapeHandle> handles;
    GetHandles(handles);
    CsgWedgeParams params = params_;
    for (const unsigned id : unique)
    {
        if (id >= handles.size())
            return false;
        const float change = localDelta.DotProduct(handles[id].axis_);
        MoveBoxFace(params.size_, params.center_, WedgeHandleAxis[id], WedgeHandleSign[id], change,
            CsgWedgeParams::MinSize, CsgWedgeParams::MaxSize);
    }

    if (!IsFinite(params.size_) || !IsFinite(params.center_))
        return false;
    CsgSanitizeWedgeParams(params);
    if (params.size_ == params_.size_ && params.center_ == params_.center_)
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateWedge(params, mesh))
        return false;

    SetParams(params);
    return true;
}

bool CsgWedgeBrush::Recenter(Vector3& localOffset)
{
    const Vector3 center = params_.center_;
    if (!IsFinite(center) || center.Length() < 1e-6f)
        return false;
    CsgWedgeParams params = params_;
    params.center_ = Vector3::ZERO;
    CsgShapeMesh mesh;
    if (!CsgGenerateWedge(params, mesh))
        return false;

    SetParams(params);
    localOffset = center;
    return true;
}

void CsgWedgeBrush::SanitizeParameters() { CsgSanitizeWedgeParams(params_); }

bool CsgWedgeBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateWedge(params_, mesh); }

} // namespace Urho3D
