// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural cylinder, cone or frustum brush. See `CsgGenerateCylinder`.
///
/// Attributes: "Bottom Center", "Top Center", "Bottom Radius", "Top Radius", "Segments", "Height Segments",
/// "Bulge" and "Smooth Normals". The values are always clamped. See `CsgSanitizeCylinderParams`.
/// A radius pair (0, 0) makes an apex. The shape is not valid if the centers are too close or both ends are apexes.
/// Then the brush gives no operand.
///
/// The brush has three material slots: 0 the side, 1 the top cap and 2 the bottom cap.
/// An apex end has no cap. Its slot stays free.
class PLUGIN_RBFXCSG_API CsgCylinderBrush : public CsgProceduralBrush
{
    URHO3D_OBJECT(CsgCylinderBrush, CsgProceduralBrush);

public:
    explicit CsgCylinderBrush(Context* context);
    ~CsgCylinderBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Set the center of the bottom end. The value is clamped. The root rebuilds.
    void SetBottomCenter(const Vector3& center);
    const Vector3& GetBottomCenter() const { return params_.bottomCenter_; }

    /// Set the center of the top end. The value is clamped. The root rebuilds.
    void SetTopCenter(const Vector3& center);
    const Vector3& GetTopCenter() const { return params_.topCenter_; }

    /// Set the radii of the bottom end along X and Z. The value is clamped. The root rebuilds.
    void SetBottomRadius(const Vector2& radius);
    const Vector2& GetBottomRadius() const { return params_.bottomRadius_; }

    /// Set the radii of the top end along X and Z. The value is clamped. The root rebuilds.
    void SetTopRadius(const Vector2& radius);
    const Vector2& GetTopRadius() const { return params_.topRadius_; }

    /// Set the number of divisions around the axis. The value is clamped. The root rebuilds.
    void SetSegments(int segments);
    int GetSegments() const { return params_.segments_; }

    /// Set the number of body bands. The value is clamped. The root rebuilds.
    void SetHeightSegments(int heightSegments);
    int GetHeightSegments() const { return params_.heightSegments_; }

    /// Set the bulge of the body. The value is clamped. The root rebuilds.
    void SetBulge(float bulge);
    float GetBulge() const { return params_.bulge_; }

    /// Set smooth or flat normals. The root rebuilds.
    void SetSmoothNormals(bool smooth);
    bool GetSmoothNormals() const { return params_.smoothNormals_; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgCylinderParams& params);
    const CsgCylinderParams& GetParams() const { return params_; }

protected:
    void SanitizeParameters() override;
    bool GenerateMesh(CsgShapeMesh& mesh) const override;
    unsigned GetNumMaterialSlots() const override { return 3; }

private:
    CsgCylinderParams params_;
};

} // namespace Urho3D
