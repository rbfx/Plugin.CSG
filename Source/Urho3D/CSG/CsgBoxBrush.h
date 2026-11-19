// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural box brush with a size and 8 movable corners. See `CsgGenerateBox`.
///
/// Attributes: "Size" (each component in [0.001, 16384]) and "Corner 0" to "Corner 7" (each component in [-64, 64]).
/// Corner index bits: bit 0 is +X, bit 1 is +Y, bit 2 is +Z. The local corner position is Size * Corner.
/// A corner that goes through the opposite face makes the box not valid. Then the brush gives no operand.
///
/// The box has 6 material slots, one for each nominal face, in this order: -X, +X, -Y, +Y, -Z, +Z.
/// The order is a file contract. An empty slot uses the material of slot 0.
class PLUGIN_RBFXCSG_API CsgBoxBrush : public CsgProceduralBrush
{
    URHO3D_OBJECT(CsgBoxBrush, CsgProceduralBrush);

public:
    explicit CsgBoxBrush(Context* context);
    ~CsgBoxBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Set the size. The value is clamped. The root rebuilds.
    void SetSize(const Vector3& size);
    const Vector3& GetSize() const { return params_.size_; }

    /// Set a corner. An index of 8 or more does nothing. The value is clamped. The root rebuilds.
    void SetCorner(unsigned index, const Vector3& corner);
    /// Return a corner. An index of 8 or more gives zero.
    Vector3 GetCorner(unsigned index) const { return index < 8 ? params_.corners_[index] : Vector3::ZERO; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgBoxParams& params);
    const CsgBoxParams& GetParams() const { return params_; }

protected:
    void SanitizeParameters() override;
    bool GenerateMesh(CsgShapeMesh& mesh) const override;
    unsigned GetNumMaterialSlots() const override { return 6; }

private:
    CsgBoxParams params_;
};

} // namespace Urho3D
