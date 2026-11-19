// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCylinderBrush.h"
#include <Urho3D/Core/Context.h>

namespace Urho3D
{

CsgCylinderBrush::CsgCylinderBrush(Context* context)
    : CsgProceduralBrush(context)
{
}

CsgCylinderBrush::~CsgCylinderBrush() = default;

void CsgCylinderBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgCylinderBrush>(Category_Scene);

    const CsgCylinderParams defaults;
    URHO3D_COPY_BASE_ATTRIBUTES(CsgProceduralBrush);
    URHO3D_ACCESSOR_ATTRIBUTE(
        "Bottom Center", GetBottomCenter, SetBottomCenter, Vector3, defaults.bottomCenter_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Top Center", GetTopCenter, SetTopCenter, Vector3, defaults.topCenter_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE(
        "Bottom Radius", GetBottomRadius, SetBottomRadius, Vector2, defaults.bottomRadius_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Top Radius", GetTopRadius, SetTopRadius, Vector2, defaults.topRadius_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Segments", GetSegments, SetSegments, int, defaults.segments_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE(
        "Height Segments", GetHeightSegments, SetHeightSegments, int, defaults.heightSegments_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Bulge", GetBulge, SetBulge, float, defaults.bulge_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE(
        "Smooth Normals", GetSmoothNormals, SetSmoothNormals, bool, defaults.smoothNormals_, AM_DEFAULT);
}

void CsgCylinderBrush::SetBottomCenter(const Vector3& center)
{
    CsgCylinderParams params = params_;
    params.bottomCenter_ = center;
    SetParams(params);
}

void CsgCylinderBrush::SetTopCenter(const Vector3& center)
{
    CsgCylinderParams params = params_;
    params.topCenter_ = center;
    SetParams(params);
}

void CsgCylinderBrush::SetBottomRadius(const Vector2& radius)
{
    CsgCylinderParams params = params_;
    params.bottomRadius_ = radius;
    SetParams(params);
}

void CsgCylinderBrush::SetTopRadius(const Vector2& radius)
{
    CsgCylinderParams params = params_;
    params.topRadius_ = radius;
    SetParams(params);
}

void CsgCylinderBrush::SetSegments(int segments)
{
    CsgCylinderParams params = params_;
    params.segments_ = segments;
    SetParams(params);
}

void CsgCylinderBrush::SetHeightSegments(int heightSegments)
{
    CsgCylinderParams params = params_;
    params.heightSegments_ = heightSegments;
    SetParams(params);
}

void CsgCylinderBrush::SetBulge(float bulge)
{
    CsgCylinderParams params = params_;
    params.bulge_ = bulge;
    SetParams(params);
}

void CsgCylinderBrush::SetSmoothNormals(bool smooth)
{
    CsgCylinderParams params = params_;
    params.smoothNormals_ = smooth;
    SetParams(params);
}

void CsgCylinderBrush::SetParams(const CsgCylinderParams& params)
{
    CsgCylinderParams sanitized = params;
    CsgSanitizeCylinderParams(sanitized);
    if (sanitized.bottomCenter_ == params_.bottomCenter_ && sanitized.topCenter_ == params_.topCenter_
        && sanitized.bottomRadius_ == params_.bottomRadius_ && sanitized.topRadius_ == params_.topRadius_
        && sanitized.segments_ == params_.segments_ && sanitized.heightSegments_ == params_.heightSegments_
        && sanitized.bulge_ == params_.bulge_ && sanitized.smoothNormals_ == params_.smoothNormals_)
        return;

    params_ = sanitized;
    OnParametersChanged();
}

void CsgCylinderBrush::SanitizeParameters() { CsgSanitizeCylinderParams(params_); }

bool CsgCylinderBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateCylinder(params_, mesh); }

} // namespace Urho3D
