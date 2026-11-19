// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgRingBrush.h"
#include <Urho3D/Core/Context.h>

#include <cmath>

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

namespace
{

/// The angle of the arc end in radians. It is computed as in `CsgGenerateRing`.
double GetArcEndAngle(float arc) { return arc * (M_PI / 180.0); }

} // namespace

void CsgRingBrush::GetHandles(ea::vector<CsgShapeHandle>& handles) const
{
    handles.clear();
    const float radius = params_.radius_;
    const Vector2& tube = params_.tubeRadius_;

    const auto add = [&](unsigned id, const char* name, const Vector3& position, const Vector3& axis)
    {
        CsgShapeHandle handle;
        handle.id_ = id;
        handle.name_ = name;
        handle.kind_ = CsgShapeHandleKind::Axis;
        handle.position_ = position;
        handle.axis_ = axis;
        handles.push_back(handle);
    };
    add(0, "Radius", Vector3(radius, 0.0f, 0.0f), Vector3(1, 0, 0));
    add(1, "Tube Radial", Vector3(radius + tube.x_, 0.0f, 0.0f), Vector3(1, 0, 0));
    add(2, "Tube Axial", Vector3(radius, tube.y_, 0.0f), Vector3(0, 1, 0));

    const double angle = GetArcEndAngle(params_.arc_);
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    add(3, "Arc End", Vector3(static_cast<float>(radius * c), 0.0f, static_cast<float>(radius * s)),
        Vector3(static_cast<float>(-s), 0.0f, static_cast<float>(c)));
}

bool CsgRingBrush::MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    ea::vector<unsigned> unique;
    if (!PrepareHandleMove(ids, localDelta, unique))
        return false;

    ea::vector<CsgShapeHandle> handles;
    GetHandles(handles);
    CsgRingParams params = params_;
    for (const unsigned id : unique)
    {
        if (id >= handles.size())
            return false;
        const float change = localDelta.DotProduct(handles[id].axis_);
        switch (id)
        {
        case 0: params.radius_ += change; break;
        case 1: params.tubeRadius_.x_ += change; break;
        case 2: params.tubeRadius_.y_ += change; break;
        default: params.arc_ += change / params_.radius_ * (180.0f / M_PI); break;
        }
    }

    if (!std::isfinite(params.radius_) || !std::isfinite(params.tubeRadius_.x_) || !std::isfinite(params.tubeRadius_.y_)
        || !std::isfinite(params.arc_))
        return false;
    CsgSanitizeRingParams(params);
    if (params.radius_ == params_.radius_ && params.tubeRadius_ == params_.tubeRadius_ && params.arc_ == params_.arc_)
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateRing(params, mesh))
        return false;

    SetParams(params);
    return true;
}

void CsgRingBrush::SanitizeParameters() { CsgSanitizeRingParams(params_); }

bool CsgRingBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateRing(params_, mesh); }

} // namespace Urho3D
