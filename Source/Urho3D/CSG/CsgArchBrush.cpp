// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgArchBrush.h"
#include <Urho3D/Core/Context.h>

#include <cmath>

namespace Urho3D
{

CsgArchBrush::CsgArchBrush(Context* context)
    : CsgProceduralBrush(context)
{
}

CsgArchBrush::~CsgArchBrush() = default;

void CsgArchBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgArchBrush>(Category_Scene);

    const CsgArchParams defaults;
    URHO3D_COPY_BASE_ATTRIBUTES(CsgProceduralBrush);
    URHO3D_ACCESSOR_ATTRIBUTE("Radius", GetRadius, SetRadius, Vector2, defaults.radius_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Thickness", GetThickness, SetThickness, float, defaults.thickness_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Depth", GetDepth, SetDepth, float, defaults.depth_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Arc", GetArc, SetArc, float, defaults.arc_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Segments", GetSegments, SetSegments, int, defaults.segments_, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Center", GetCenter, SetCenter, Vector3, defaults.center_, AM_DEFAULT);
}

void CsgArchBrush::SetRadius(const Vector2& radius)
{
    CsgArchParams params = params_;
    params.radius_ = radius;
    SetParams(params);
}

void CsgArchBrush::SetThickness(float thickness)
{
    CsgArchParams params = params_;
    params.thickness_ = thickness;
    SetParams(params);
}

void CsgArchBrush::SetDepth(float depth)
{
    CsgArchParams params = params_;
    params.depth_ = depth;
    SetParams(params);
}

void CsgArchBrush::SetArc(float arc)
{
    CsgArchParams params = params_;
    params.arc_ = arc;
    SetParams(params);
}

void CsgArchBrush::SetSegments(int segments)
{
    CsgArchParams params = params_;
    params.segments_ = segments;
    SetParams(params);
}

void CsgArchBrush::SetCenter(const Vector3& center)
{
    CsgArchParams params = params_;
    params.center_ = center;
    SetParams(params);
}

namespace
{

bool AreArchParamsEqual(const CsgArchParams& a, const CsgArchParams& b)
{
    return a.radius_ == b.radius_ && a.thickness_ == b.thickness_ && a.depth_ == b.depth_ && a.arc_ == b.arc_
        && a.segments_ == b.segments_ && a.center_ == b.center_;
}

/// The end angle of the arc in radians.
double GetArchEndAngle(float arc) { return (90.0 + arc * 0.5) * (M_PI / 180.0); }

/// The start angle of the arc in radians.
double GetArchStartAngle(float arc) { return (90.0 - arc * 0.5) * (M_PI / 180.0); }

} // namespace

void CsgArchBrush::SetParams(const CsgArchParams& params)
{
    CsgArchParams sanitized = params;
    CsgSanitizeArchParams(sanitized);
    if (AreArchParamsEqual(sanitized, params_))
        return;

    params_ = sanitized;
    OnParametersChanged();
}

void CsgArchBrush::GetHandles(ea::vector<CsgShapeHandle>& handles) const
{
    handles.clear();
    const Vector2& r = params_.radius_;
    const Vector2 inner = CsgGetArchInnerRadius(params_);
    const Vector3& c = params_.center_;

    const auto add = [&](unsigned id, const char* name, const Vector3& offset, const Vector3& axis)
    {
        CsgShapeHandle handle;
        handle.id_ = id;
        handle.name_ = name;
        handle.kind_ = CsgShapeHandleKind::Axis;
        handle.position_ = c + offset;
        handle.axis_ = axis;
        handle.group_ = "Shape";
        handles.push_back(handle);
    };
    const double a0 = GetArchStartAngle(params_.arc_);
    add(0, "Radius X", Vector3(static_cast<float>(r.x_ * std::cos(a0)), static_cast<float>(r.y_ * std::sin(a0)), 0.0f),
        Vector3::RIGHT);
    add(1, "Radius Y", Vector3(0.0f, r.y_, 0.0f), Vector3::UP);
    add(2, "Thickness", Vector3(0.0f, inner.y_, 0.0f), Vector3::DOWN);
    add(3, "Depth", Vector3(0.0f, (r.y_ + inner.y_) * 0.5f, params_.depth_ * 0.5f), Vector3::FORWARD);

    const double a1 = GetArchEndAngle(params_.arc_);
    const double tx = -r.x_ * std::sin(a1);
    const double ty = r.y_ * std::cos(a1);
    const double speed = std::sqrt(tx * tx + ty * ty);
    add(4, "Arc End", Vector3(static_cast<float>(r.x_ * std::cos(a1)), static_cast<float>(r.y_ * std::sin(a1)), 0.0f),
        Vector3(static_cast<float>(tx / speed), static_cast<float>(ty / speed), 0.0f));
}

void CsgArchBrush::GetHandleGroups(ea::vector<ea::string>& groups) const
{
    groups = {"Shape"};
}

bool CsgArchBrush::MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    ea::vector<unsigned> unique;
    if (!PrepareHandleMove(ids, localDelta, unique))
        return false;

    ea::vector<CsgShapeHandle> handles;
    GetHandles(handles);
    CsgArchParams params = params_;
    for (const unsigned id : unique)
    {
        if (id >= handles.size())
            return false;
        const float change = localDelta.DotProduct(handles[id].axis_);
        switch (id)
        {
        case 0: params.radius_.x_ += change; break;
        case 1: params.radius_.y_ += change; break;
        case 2: params.thickness_ += change; break;
        case 3:
        {
            const float depth = Clamp(params.depth_ + change, CsgArchParams::MinDepth, CsgArchParams::MaxDepth);
            params.center_.z_ += (depth - params.depth_) * 0.5f;
            params.depth_ = depth;
            break;
        }
        default:
        {
            const double a1 = GetArchEndAngle(params_.arc_);
            const double tx = -params_.radius_.x_ * std::sin(a1);
            const double ty = params_.radius_.y_ * std::cos(a1);
            const double speed = std::sqrt(tx * tx + ty * ty);
            params.arc_ += static_cast<float>(2.0 * change / speed * (180.0 / M_PI));
            break;
        }
        }
    }

    if (!std::isfinite(params.radius_.x_) || !std::isfinite(params.radius_.y_) || !std::isfinite(params.thickness_)
        || !std::isfinite(params.depth_) || !std::isfinite(params.arc_) || !IsFinite(params.center_))
        return false;
    CsgSanitizeArchParams(params);
    if (AreArchParamsEqual(params, params_))
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateArch(params, mesh))
        return false;

    SetParams(params);
    return true;
}

bool CsgArchBrush::Recenter(Vector3& localOffset)
{
    const Vector3 center = params_.center_;
    if (!IsFinite(center) || center.Length() < 1e-6f)
        return false;
    CsgArchParams params = params_;
    params.center_ = Vector3::ZERO;
    CsgShapeMesh mesh;
    if (!CsgGenerateArch(params, mesh))
        return false;

    SetParams(params);
    localOffset = center;
    return true;
}

void CsgArchBrush::SanitizeParameters() { CsgSanitizeArchParams(params_); }

bool CsgArchBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateArch(params_, mesh); }

} // namespace Urho3D
