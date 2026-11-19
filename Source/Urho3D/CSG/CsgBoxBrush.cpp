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

void CsgBoxBrush::SanitizeParameters() { CsgSanitizeBoxParams(params_); }

bool CsgBoxBrush::GenerateMesh(CsgShapeMesh& mesh) const { return CsgGenerateBox(params_, mesh); }

} // namespace Urho3D
