// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgStairsBrush.h"
#include <Urho3D/Core/Context.h>

namespace Urho3D
{

CsgStairsBrush::CsgStairsBrush(Context* context)
    : CsgProceduralBrush(context)
{
}

CsgStairsBrush::~CsgStairsBrush() = default;

void CsgStairsBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgStairsBrush>(Category_Scene);

    const CsgStairsParams defaults;
    URHO3D_COPY_BASE_ATTRIBUTES(CsgProceduralBrush);
    URHO3D_ACCESSOR_ATTRIBUTE("Size", GetSize, SetSize, Vector3, defaults.size_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Center", GetCenter, SetCenter, Vector3, defaults.center_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Steps", GetSteps, SetSteps, int, defaults.steps_, AM_DEFAULT);
}

void CsgStairsBrush::SetSize(const Vector3& size)
{
    CsgStairsParams params = params_;
    params.size_ = size;
    SetParams(params);
}

void CsgStairsBrush::SetCenter(const Vector3& center)
{
    CsgStairsParams params = params_;
    params.center_ = center;
    SetParams(params);
}

void CsgStairsBrush::SetSteps(int steps)
{
    CsgStairsParams params = params_;
    params.steps_ = steps;
    SetParams(params);
}

void CsgStairsBrush::SetParams(const CsgStairsParams& params)
{
    CsgStairsParams sanitized = params;
    CsgSanitizeStairsParams(sanitized);
    if (sanitized.size_ == params_.size_ && sanitized.center_ == params_.center_ && sanitized.steps_ == params_.steps_)
        return;

    params_ = sanitized;
    OnParametersChanged();
}

namespace
{

/// The axis index and the face sign of each handle id.
const unsigned StairsHandleAxis[6] = {0, 0, 2, 2, 1, 1};
const float StairsHandleSign[6] = {-1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f};

} // namespace

void CsgStairsBrush::GetHandles(ea::vector<CsgShapeHandle>& handles) const
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
    const float steps = static_cast<float>(params_.steps_);
    add(0, "Left", Vector3(-h.x_, 0.0f, 0.0f), Vector3::LEFT);
    add(1, "Right", Vector3(h.x_, 0.0f, 0.0f), Vector3::RIGHT);
    add(2, "Front", Vector3(0.0f, -h.y_ + h.y_ / steps, -h.z_), Vector3::BACK);
    add(3, "Back", Vector3(0.0f, 0.0f, h.z_), Vector3::FORWARD);
    add(4, "Top", Vector3(0.0f, h.y_, h.z_ - h.z_ / steps), Vector3::UP);
    add(5, "Bottom", Vector3(0.0f, -h.y_, 0.0f), Vector3::DOWN);
}

bool CsgStairsBrush::MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    ea::vector<unsigned> unique;
    if (!PrepareHandleMove(ids, localDelta, unique))
        return false;

    ea::vector<CsgShapeHandle> handles;
    GetHandles(handles);
    CsgStairsParams params = params_;
    for (const unsigned id : unique)
    {
        if (id >= handles.size())
            return false;
        const float change = localDelta.DotProduct(handles[id].axis_);
        MoveBoxFace(params.size_, params.center_, StairsHandleAxis[id], StairsHandleSign[id], change,
            CsgStairsParams::MinSize, CsgStairsParams::MaxSize);
    }

    if (!IsFinite(params.size_) || !IsFinite(params.center_))
        return false;
    CsgSanitizeStairsParams(params);
    if (params.size_ == params_.size_ && params.center_ == params_.center_)
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateStairs(params, mesh))
        return false;

    SetParams(params);
    return true;
}

bool CsgStairsBrush::Recenter(Vector3& localOffset)
{
    const Vector3 center = params_.center_;
    if (!IsFinite(center) || center.Length() < 1e-6f)
        return false;
    CsgStairsParams params = params_;
    params.center_ = Vector3::ZERO;
    CsgShapeMesh mesh;
    if (!CsgGenerateStairs(params, mesh))
        return false;

    SetParams(params);
    localOffset = center;
    return true;
}

void CsgStairsBrush::SanitizeParameters() { CsgSanitizeStairsParams(params_); }

bool CsgStairsBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateStairs(params_, mesh); }

} // namespace Urho3D
