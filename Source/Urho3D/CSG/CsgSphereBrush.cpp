// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgSphereBrush.h"
#include <Urho3D/Core/Context.h>

#include <cmath>

namespace Urho3D
{

CsgSphereBrush::CsgSphereBrush(Context* context)
    : CsgProceduralBrush(context)
{
}

CsgSphereBrush::~CsgSphereBrush() = default;

void CsgSphereBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgSphereBrush>(Category_Scene);

    const CsgSphereParams defaults;
    URHO3D_COPY_BASE_ATTRIBUTES(CsgProceduralBrush);
    URHO3D_ACCESSOR_ATTRIBUTE("Radius", GetRadius, SetRadius, Vector3, defaults.radius_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Segments", GetSegments, SetSegments, int, defaults.segments_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Rings", GetRings, SetRings, int, defaults.rings_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE(
        "Smooth Normals", GetSmoothNormals, SetSmoothNormals, bool, defaults.smoothNormals_, AM_DEFAULT);
}

void CsgSphereBrush::SetRadius(const Vector3& radius)
{
    CsgSphereParams params = params_;
    params.radius_ = radius;
    SetParams(params);
}

void CsgSphereBrush::SetSegments(int segments)
{
    CsgSphereParams params = params_;
    params.segments_ = segments;
    SetParams(params);
}

void CsgSphereBrush::SetRings(int rings)
{
    CsgSphereParams params = params_;
    params.rings_ = rings;
    SetParams(params);
}

void CsgSphereBrush::SetSmoothNormals(bool smooth)
{
    CsgSphereParams params = params_;
    params.smoothNormals_ = smooth;
    SetParams(params);
}

void CsgSphereBrush::SetParams(const CsgSphereParams& params)
{
    CsgSphereParams sanitized = params;
    CsgSanitizeSphereParams(sanitized);
    if (sanitized.radius_ == params_.radius_ && sanitized.segments_ == params_.segments_
        && sanitized.rings_ == params_.rings_ && sanitized.smoothNormals_ == params_.smoothNormals_)
        return;

    params_ = sanitized;
    OnParametersChanged();
}

namespace
{

/// The outward axis of a radius handle: +X, -X, +Y, -Y, +Z, -Z.
Vector3 GetRadiusAxis(unsigned id)
{
    Vector3 axis;
    axis.MutableData()[id / 2] = id % 2 ? -1.0f : 1.0f;
    return axis;
}

const char* const RadiusNames[6] = {"Radius +X", "Radius -X", "Radius +Y", "Radius -Y", "Radius +Z", "Radius -Z"};

} // namespace

void CsgSphereBrush::GetHandles(ea::vector<CsgShapeHandle>& handles) const
{
    handles.clear();
    for (unsigned id = 0; id < 6; ++id)
    {
        CsgShapeHandle handle;
        handle.id_ = id;
        handle.name_ = RadiusNames[id];
        handle.kind_ = CsgShapeHandleKind::Axis;
        handle.axis_ = GetRadiusAxis(id);
        handle.position_ = handle.axis_ * params_.radius_.Data()[id / 2];
        handles.push_back(handle);
    }
}

bool CsgSphereBrush::MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    ea::vector<unsigned> unique;
    if (!PrepareHandleMove(ids, localDelta, unique))
        return false;

    CsgSphereParams params = params_;
    for (const unsigned id : unique)
    {
        if (id >= 6)
            return false;
        params.radius_.MutableData()[id / 2] += localDelta.DotProduct(GetRadiusAxis(id));
    }
    if (!IsFinite(params.radius_))
        return false;
    CsgSanitizeSphereParams(params);
    if (params.radius_ == params_.radius_)
        return false;

    SetParams(params);
    return true;
}

void CsgSphereBrush::SanitizeParameters() { CsgSanitizeSphereParams(params_); }

bool CsgSphereBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateSphere(params_, mesh); }

} // namespace Urho3D
