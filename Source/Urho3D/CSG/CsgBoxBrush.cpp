// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgBoxBrush.h"
#include <Urho3D/Core/Context.h>

namespace Urho3D
{

namespace
{

const char* const CornerAttributeNames[8] = {
    "Corner 0", "Corner 1", "Corner 2", "Corner 3", "Corner 4", "Corner 5", "Corner 6", "Corner 7"};

/// The axis of the nominal face. The face order is -X, +X, -Y, +Y, -Z, +Z.
Vector3 GetFaceAxis(unsigned face)
{
    Vector3 axis;
    axis.MutableData()[face / 2] = face % 2 ? 1.0f : -1.0f;
    return axis;
}

/// Return true if the corner is on the nominal face.
bool IsCornerOnFace(unsigned corner, unsigned face)
{
    return ((corner >> (face / 2)) & 1u) == (face % 2);
}

ea::string GetCornerName(unsigned corner)
{
    ea::string name = "Corner";
    name += corner & 1u ? " +X" : " -X";
    name += corner & 2u ? "+Y" : "-Y";
    name += corner & 4u ? "+Z" : "-Z";
    return name;
}

/// Return the two corners of an edge. The edge is parallel to axis (id - FirstEdgeHandle) / 4.
/// The low corner has a zero bit on the axis. The other two bits come from the index in the group.
void GetEdgeCorners(unsigned edge, unsigned& low, unsigned& high)
{
    const unsigned axis = edge / 4;
    const unsigned b = axis == 0 ? 1 : 0; // first other axis
    const unsigned c = axis == 2 ? 1 : 2; // second other axis
    low = (((edge >> 0) & 1u) << b) | (((edge >> 1) & 1u) << c);
    high = low | (1u << axis);
}

ea::string GetEdgeName(unsigned edge)
{
    unsigned low, high;
    GetEdgeCorners(edge, low, high);
    const char letters[3] = {'X', 'Y', 'Z'};
    ea::string name = "Edge";
    for (unsigned axis = 0; axis < 3; ++axis)
    {
        name += ' ';
        if (axis == edge / 4)
            name += letters[axis];
        else
        {
            name += (low >> axis) & 1u ? '+' : '-';
            name += letters[axis];
        }
    }
    return name;
}

const char* const FaceNames[6] = {"Face -X", "Face +X", "Face -Y", "Face +Y", "Face -Z", "Face +Z"};

} // namespace

CsgBoxBrush::CsgBoxBrush(Context* context)
    : CsgProceduralBrush(context)
{
}

CsgBoxBrush::~CsgBoxBrush() = default;

void CsgBoxBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgBoxBrush>(Category_Scene);

    URHO3D_COPY_BASE_ATTRIBUTES(CsgProceduralBrush);
    URHO3D_ACCESSOR_ATTRIBUTE("Size", GetSize, SetSize, Vector3, Vector3::ONE, AM_DEFAULT);
    for (unsigned i = 0; i < 8; ++i)
    {
        context->Reflect<CsgBoxBrush>()->AddAttribute(AttributeInfo(VAR_VECTOR3, CornerAttributeNames[i],
            MakeVariantAttributeAccessor<CsgBoxBrush>(
                [i](const CsgBoxBrush& self, Variant& value) { value = self.GetCorner(i); },
                [i](CsgBoxBrush& self, const Variant& value) { self.SetCorner(i, value.GetVector3()); }),
            nullptr, CsgBoxParams::GetDefaultCorner(i), AM_DEFAULT));
    }
}

void CsgBoxBrush::SetSize(const Vector3& size)
{
    CsgBoxParams params = params_;
    params.size_ = size;
    SetParams(params);
}

void CsgBoxBrush::SetCorner(unsigned index, const Vector3& corner)
{
    if (index >= 8)
        return;
    CsgBoxParams params = params_;
    params.corners_[index] = corner;
    SetParams(params);
}

void CsgBoxBrush::SetParams(const CsgBoxParams& params)
{
    CsgBoxParams sanitized = params;
    CsgSanitizeBoxParams(sanitized);
    if (sanitized.size_ == params_.size_ && sanitized.corners_ == params_.corners_)
        return;

    params_ = sanitized;
    OnParametersChanged();
}

void CsgBoxBrush::GetHandles(ea::vector<CsgShapeHandle>& handles) const
{
    handles.clear();
    for (unsigned corner = 0; corner < 8; ++corner)
    {
        CsgShapeHandle handle;
        handle.id_ = corner;
        handle.name_ = GetCornerName(corner);
        handle.kind_ = CsgShapeHandleKind::Point;
        handle.position_ = params_.size_ * params_.corners_[corner];
        handles.push_back(handle);
    }
    for (unsigned face = 0; face < 6; ++face)
    {
        Vector3 sum;
        for (unsigned corner = 0; corner < 8; ++corner)
        {
            if (IsCornerOnFace(corner, face))
                sum += params_.size_ * params_.corners_[corner];
        }
        CsgShapeHandle handle;
        handle.id_ = FirstFaceHandle + face;
        handle.name_ = FaceNames[face];
        handle.kind_ = CsgShapeHandleKind::Axis;
        handle.position_ = sum / 4.0f;
        handle.axis_ = GetFaceAxis(face);
        handles.push_back(handle);
    }
    for (unsigned edge = 0; edge < 12; ++edge)
    {
        unsigned low, high;
        GetEdgeCorners(edge, low, high);
        CsgShapeHandle handle;
        handle.id_ = FirstEdgeHandle + edge;
        handle.name_ = GetEdgeName(edge);
        handle.kind_ = CsgShapeHandleKind::Point;
        handle.position_ = params_.size_ * (params_.corners_[low] + params_.corners_[high]) * 0.5f;
        handles.push_back(handle);
    }
}

bool CsgBoxBrush::MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta)
{
    ea::vector<unsigned> unique;
    if (!PrepareHandleMove(ids, localDelta, unique))
        return false;

    const Vector3 size = VectorMax(params_.size_, Vector3::ONE * CsgBoxParams::MinSize);
    CsgBoxParams params = params_;
    // A corner or edge handle moves its corners by the delta. A corner moves once, also if two
    // handles name it. A face handle adds its axis move on top.
    unsigned pointMask = 0;
    for (const unsigned id : unique)
    {
        if (id < 8)
        {
            pointMask |= 1u << id;
        }
        else if (id >= FirstEdgeHandle && id < FirstEdgeHandle + 12)
        {
            unsigned low, high;
            GetEdgeCorners(id - FirstEdgeHandle, low, high);
            pointMask |= (1u << low) | (1u << high);
        }
        else if (id >= FirstFaceHandle && id < FirstFaceHandle + 6)
        {
            const unsigned face = id - FirstFaceHandle;
            const Vector3 axis = GetFaceAxis(face);
            const float distance = localDelta.DotProduct(axis);
            for (unsigned corner = 0; corner < 8; ++corner)
            {
                if (IsCornerOnFace(corner, face))
                    params.corners_[corner] += axis * distance / size;
            }
        }
        else
            return false;
    }
    for (unsigned corner = 0; corner < 8; ++corner)
    {
        if (pointMask & (1u << corner))
            params.corners_[corner] += localDelta / size;
    }

    // A value that is not finite (an overflow) is not accepted. The sanitizer would replace it.
    for (const Vector3& corner : params.corners_)
    {
        if (!IsFinite(corner))
            return false;
    }
    CsgSanitizeBoxParams(params);
    if (params.corners_ == params_.corners_)
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateBox(params, mesh))
        return false;

    SetParams(params);
    return true;
}

bool CsgBoxBrush::Recenter(Vector3& localOffset)
{
    Vector3 center;
    for (unsigned corner = 0; corner < 8; ++corner)
        center += params_.size_ * params_.corners_[corner];
    center /= 8.0f;
    if (!IsFinite(center) || center.Length() < 1e-6f)
        return false;

    const Vector3 size = VectorMax(params_.size_, Vector3::ONE * CsgBoxParams::MinSize);
    CsgBoxParams params = params_;
    for (Vector3& corner : params.corners_)
        corner -= center / size;

    // The sanitizer must not change the result, and the shape must stay valid.
    const CsgBoxParams wanted = params;
    CsgSanitizeBoxParams(params);
    if (params.corners_ != wanted.corners_)
        return false;
    CsgShapeMesh mesh;
    if (!CsgGenerateBox(params, mesh))
        return false;

    SetParams(params);
    localOffset = center;
    return true;
}

void CsgBoxBrush::SanitizeParameters() { CsgSanitizeBoxParams(params_); }

bool CsgBoxBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateBox(params_, mesh); }

} // namespace Urho3D
