// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCylinderBrush.h"
#include <Urho3D/Core/Context.h>

#include <cmath>

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
    URHO3D_ACCESSOR_ATTRIBUTE("Start Angle", GetStartAngle, SetStartAngle, float, defaults.startAngle_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Face Radius", GetFaceRadius, SetFaceRadius, bool, defaults.faceRadius_, AM_DEFAULT);
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

void CsgCylinderBrush::SetStartAngle(float degrees)
{
    CsgCylinderParams params = params_;
    params.startAngle_ = degrees;
    SetParams(params);
}

void CsgCylinderBrush::SetFaceRadius(bool faceRadius)
{
    CsgCylinderParams params = params_;
    params.faceRadius_ = faceRadius;
    SetParams(params);
}

void CsgCylinderBrush::SetParams(const CsgCylinderParams& params)
{
    CsgCylinderParams sanitized = params;
    CsgSanitizeCylinderParams(sanitized);
    if (sanitized == params_)
        return;

    params_ = sanitized;
    OnParametersChanged();
}

namespace
{

constexpr unsigned BulgeHandle = 10;
constexpr unsigned BottomScaleHandle = 11;
constexpr unsigned TopScaleHandle = 12;
constexpr double ExactPi = 3.14159265358979323846;
/// The minimum mean X radius for the bulge handle.
constexpr float MinBulgeRadius = 0.001f;

/// The outward direction of a radius handle: +X, -X, +Z, -Z.
Vector3 GetRadiusDirection(unsigned index)
{
    constexpr Vector3 directions[4] = {Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 0, 1), Vector3(0, 0, -1)};
    return directions[index];
}

const char* const BottomRadiusNames[4] = {"Bottom Radius +X", "Bottom Radius -X", "Bottom Radius +Z", "Bottom Radius -Z"};
const char* const TopRadiusNames[4] = {"Top Radius +X", "Top Radius -X", "Top Radius +Z", "Top Radius -Z"};

bool IsApex(const Vector2& radius) { return radius.x_ <= 0.0f && radius.y_ <= 0.0f; }

/// Return the distance from the center to the boundary of the unit polygon along the angle (in XZ, from +X to +Z).
/// The unit polygon has its corners at the Start Angle. Its corner distance is 1, or 1 / cos(pi / n) with Face Radius.
/// A ray through a corner returns the corner distance exactly.
double GetPolygonRayDistance(const CsgCylinderParams& params, double angle)
{
    const double n = params.segments_;
    const double step = 2.0 * ExactPi / n;
    const double corner = params.faceRadius_ ? 1.0 / std::cos(ExactPi / n) : 1.0;
    const double apothem = params.faceRadius_ ? 1.0 : std::cos(ExactPi / n);
    const double position = (angle - static_cast<double>(params.startAngle_) * ExactPi / 180.0) / step;
    const double side = std::floor(position);
    const double fraction = position - side;
    if (fraction < 1e-9 || fraction > 1.0 - 1e-9)
        return corner;
    const double normalAngle = (side + 0.5) * step + static_cast<double>(params.startAngle_) * ExactPi / 180.0;
    return apothem / std::cos(angle - normalAngle);
}

/// The angle of the direction of a radius handle: +X, -X, +Z, -Z.
double GetRadiusAngle(unsigned index)
{
    constexpr double angles[4] = {0.0, ExactPi, 0.5 * ExactPi, 1.5 * ExactPi};
    return angles[index];
}

/// The unit polygon point (x, z) of the scale handle, on the rim at 45 degrees.
Vector2 GetScalePoint(const CsgCylinderParams& params)
{
    const double k = GetPolygonRayDistance(params, 0.25 * ExactPi);
    const double c = std::cos(0.25 * ExactPi);
    return Vector2(static_cast<float>(k * c), static_cast<float>(k * c));
}

float GetMeanRadiusX(const CsgCylinderParams& params)
{
    return (params.bottomRadius_.x_ + params.topRadius_.x_) * 0.5f;
}

} // namespace

void CsgCylinderBrush::GetCageLines(ea::vector<ea::pair<unsigned, unsigned>>& lines) const
{
    lines.clear();
    lines.emplace_back(0u, 1u);
}

void CsgCylinderBrush::GetHandles(ea::vector<CsgShapeHandle>& handles) const
{
    handles.clear();
    const Vector3 centers[2] = {params_.bottomCenter_, params_.topCenter_};
    const Vector2 radii[2] = {params_.bottomRadius_, params_.topRadius_};

    for (unsigned end = 0; end < 2; ++end)
    {
        CsgShapeHandle handle;
        handle.id_ = end;
        handle.name_ = end ? "Top Center" : "Bottom Center";
        handle.kind_ = CsgShapeHandleKind::Point;
        handle.position_ = centers[end];
        handle.group_ = (handle.id_ <= 1 || handle.id_ >= 11) ? "Ends" : "Radius";
        handles.push_back(handle);
    }
    for (unsigned end = 0; end < 2; ++end)
    {
        if (IsApex(radii[end]))
            continue;
        for (unsigned i = 0; i < 4; ++i)
        {
            CsgShapeHandle handle;
            handle.id_ = 2 + end * 4 + i;
            handle.name_ = end ? TopRadiusNames[i] : BottomRadiusNames[i];
            handle.kind_ = CsgShapeHandleKind::Axis;
            handle.axis_ = GetRadiusDirection(i);
            const float reach = static_cast<float>(GetPolygonRayDistance(params_, GetRadiusAngle(i)));
            handle.position_ = centers[end] + handle.axis_ * ((i < 2 ? radii[end].x_ : radii[end].y_) * reach);
            handle.group_ = (handle.id_ <= 1 || handle.id_ >= 11) ? "Ends" : "Radius";
            handles.push_back(handle);
        }
    }
    const float radiusMid = GetMeanRadiusX(params_);
    if (radiusMid > MinBulgeRadius)
    {
        CsgShapeHandle handle;
        handle.id_ = BulgeHandle;
        handle.name_ = "Bulge";
        handle.kind_ = CsgShapeHandleKind::Axis;
        handle.axis_ = Vector3(1, 0, 0);
        handle.position_ = (params_.bottomCenter_ + params_.topCenter_) * 0.5f
            + handle.axis_ * (radiusMid * (1.0f + params_.bulge_));
        handle.group_ = (handle.id_ <= 1 || handle.id_ >= 11) ? "Ends" : "Radius";
        handles.push_back(handle);
    }
    // The scale handles sit on the rim at 45 degrees. At an apex the handle sits at the end center.
    const Vector2 scalePoint = GetScalePoint(params_);
    for (unsigned end = 0; end < 2; ++end)
    {
        CsgShapeHandle handle;
        handle.id_ = end ? TopScaleHandle : BottomScaleHandle;
        handle.name_ = end ? "Top Scale" : "Bottom Scale";
        handle.kind_ = CsgShapeHandleKind::Axis;
        if (IsApex(radii[end]))
        {
            handle.axis_ = Vector3(1, 0, 1).Normalized();
            handle.position_ = centers[end];
        }
        else
        {
            const Vector3 offset(radii[end].x_ * scalePoint.x_, 0.0f, radii[end].y_ * scalePoint.y_);
            handle.axis_ = offset.Normalized();
            handle.position_ = centers[end] + offset;
        }
        handle.group_ = (handle.id_ <= 1 || handle.id_ >= 11) ? "Ends" : "Radius";
        handles.push_back(handle);
    }
}

void CsgCylinderBrush::GetHandleGroups(ea::vector<ea::string>& groups) const
{
    groups = {"Ends", "Radius"};
}

bool CsgCylinderBrush::MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    ea::vector<unsigned> unique;
    if (!PrepareHandleMove(ids, localDelta, unique))
        return false;

    CsgCylinderParams params = params_;
    for (const unsigned id : unique)
    {
        if (id == 0)
            params.bottomCenter_ += localDelta;
        else if (id == 1)
            params.topCenter_ += localDelta;
        else if (id >= 2 && id < 10)
        {
            const unsigned end = (id - 2) / 4;
            const unsigned index = (id - 2) % 4;
            Vector2& radius = end ? params.topRadius_ : params.bottomRadius_;
            // An apex end has no radius handle.
            if (IsApex(end ? params_.topRadius_ : params_.bottomRadius_))
                return false;
            // The handle sits at the radius times the ray distance of the polygon.
            const float reach = static_cast<float>(GetPolygonRayDistance(params_, GetRadiusAngle(index)));
            const float change = localDelta.DotProduct(GetRadiusDirection(index)) / reach;
            if (index < 2)
                radius.x_ += change;
            else
                radius.y_ += change;
        }
        else if (id == BottomScaleHandle || id == TopScaleHandle)
        {
            const bool top = id == TopScaleHandle;
            const Vector2 current = top ? params_.topRadius_ : params_.bottomRadius_;
            Vector2& radius = top ? params.topRadius_ : params.bottomRadius_;
            const Vector2 scalePoint = GetScalePoint(params_);
            if (IsApex(current))
            {
                // The apex grows back. The handle stays under the cursor: the distance along the axis is r * reach.
                const float along = localDelta.DotProduct(Vector3(1, 0, 1).Normalized());
                const float size = ea::max(along / (scalePoint.x_ * 1.41421356f), 0.0f);
                radius = Vector2(size, size);
            }
            else
            {
                // Both components scale by the same factor. The new handle position is the old one plus the delta.
                const Vector3 offset(current.x_ * scalePoint.x_, 0.0f, current.y_ * scalePoint.y_);
                const float length = offset.Length();
                const float factor = (length + localDelta.DotProduct(offset / length)) / length;
                radius = current * factor;
            }
        }
        else if (id == BulgeHandle)
        {
            const float radiusMid = GetMeanRadiusX(params_);
            if (!(radiusMid > MinBulgeRadius))
                return false;
            params.bulge_ += localDelta.x_ / radiusMid;
        }
        else
            return false;
    }

    if (!IsFinite(params.bottomCenter_) || !IsFinite(params.topCenter_) || !std::isfinite(params.bottomRadius_.x_)
        || !std::isfinite(params.bottomRadius_.y_) || !std::isfinite(params.topRadius_.x_)
        || !std::isfinite(params.topRadius_.y_) || !std::isfinite(params.bulge_))
        return false;
    CsgSanitizeCylinderParams(params);
    if (params == params_)
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateCylinder(params, mesh))
        return false;

    SetParams(params);
    return true;
}

bool CsgCylinderBrush::Recenter(Vector3& localOffset)
{
    const Vector3 center = (params_.bottomCenter_ + params_.topCenter_) * 0.5f;
    if (!IsFinite(center) || center.Length() < 1e-6f)
        return false;

    CsgCylinderParams params = params_;
    params.bottomCenter_ -= center;
    params.topCenter_ -= center;

    // The sanitizer must not change the result, and the shape must stay valid.
    const CsgCylinderParams wanted = params;
    CsgSanitizeCylinderParams(params);
    if (params.bottomCenter_ != wanted.bottomCenter_ || params.topCenter_ != wanted.topCenter_)
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateCylinder(params, mesh))
        return false;

    SetParams(params);
    localOffset = center;
    return true;
}

void CsgCylinderBrush::SanitizeParameters() { CsgSanitizeCylinderParams(params_); }

bool CsgCylinderBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateCylinder(params_, mesh); }

} // namespace Urho3D
