// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgRingBrush.h"
#include <Urho3D/Core/Context.h>

namespace Urho3D
{

CsgRingBrush::CsgRingBrush(Context* context)
    : CsgProceduralBrush(context)
{
}

CsgRingBrush::~CsgRingBrush() = default;

void CsgRingBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgRingBrush>(Category_Scene);

    const CsgRingParams defaults;
    URHO3D_COPY_BASE_ATTRIBUTES(CsgProceduralBrush);
    URHO3D_ACCESSOR_ATTRIBUTE("Radius", GetRadius, SetRadius, float, defaults.radius_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Tube Radius", GetTubeRadius, SetTubeRadius, Vector2, defaults.tubeRadius_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Segments", GetSegments, SetSegments, int, defaults.segments_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE(
        "Tube Segments", GetTubeSegments, SetTubeSegments, int, defaults.tubeSegments_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Arc", GetArc, SetArc, float, defaults.arc_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Tube Angle", GetTubeAngle, SetTubeAngle, float, defaults.tubeAngle_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE(
        "Smooth Normals", GetSmoothNormals, SetSmoothNormals, bool, defaults.smoothNormals_, AM_DEFAULT);
}

void CsgRingBrush::SetRadius(float radius)
{
    CsgRingParams params = params_;
    params.radius_ = radius;
    SetParams(params);
}

void CsgRingBrush::SetTubeRadius(const Vector2& tubeRadius)
{
    CsgRingParams params = params_;
    params.tubeRadius_ = tubeRadius;
    SetParams(params);
}

void CsgRingBrush::SetSegments(int segments)
{
    CsgRingParams params = params_;
    params.segments_ = segments;
    SetParams(params);
}

void CsgRingBrush::SetTubeSegments(int tubeSegments)
{
    CsgRingParams params = params_;
    params.tubeSegments_ = tubeSegments;
    SetParams(params);
}

void CsgRingBrush::SetArc(float arc)
{
    CsgRingParams params = params_;
    params.arc_ = arc;
    SetParams(params);
}

void CsgRingBrush::SetTubeAngle(float tubeAngle)
{
    CsgRingParams params = params_;
    params.tubeAngle_ = tubeAngle;
    SetParams(params);
}

void CsgRingBrush::SetSmoothNormals(bool smooth)
{
    CsgRingParams params = params_;
    params.smoothNormals_ = smooth;
    SetParams(params);
}

void CsgRingBrush::SetParams(const CsgRingParams& params)
{
    CsgRingParams sanitized = params;
    CsgSanitizeRingParams(sanitized);
    if (sanitized.radius_ == params_.radius_ && sanitized.tubeRadius_ == params_.tubeRadius_
        && sanitized.segments_ == params_.segments_ && sanitized.tubeSegments_ == params_.tubeSegments_
        && sanitized.arc_ == params_.arc_ && sanitized.tubeAngle_ == params_.tubeAngle_
        && sanitized.smoothNormals_ == params_.smoothNormals_)
        return;

    params_ = sanitized;
    OnParametersChanged();
}

void CsgRingBrush::SanitizeParameters() { CsgSanitizeRingParams(params_); }

bool CsgRingBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateRing(params_, mesh); }

} // namespace Urho3D
