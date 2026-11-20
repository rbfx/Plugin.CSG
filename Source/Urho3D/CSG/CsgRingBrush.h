// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural ring brush (a torus with an elliptic tube). See `CsgGenerateRing`.
///
/// Attributes: "Radius", "Tube Radius", "Segments", "Tube Segments", "Arc", "Tube Angle" and "Smooth Normals".
/// The angles are in degrees. The values are always clamped. See `CsgSanitizeRingParams`.
/// The ring lies around the local Y axis.
///
/// The brush has six material slots, in this order: 0 the outer surface, 1 the inner surface, 2 the top,
/// 3 the bottom, 4 the start cap, 5 the end cap. A full ring (arc 360) has no caps. Their slots stay free.
/// A quadrant of the tube section without a quad also leaves its slot free.
class PLUGIN_RBFXCSG_API CsgRingBrush : public CsgProceduralBrush
{
    URHO3D_OBJECT(CsgRingBrush, CsgProceduralBrush);

public:
    explicit CsgRingBrush(Context* context);
    ~CsgRingBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Set the major radius. The value is clamped. The root rebuilds.
    void SetRadius(float radius);
    float GetRadius() const { return params_.radius_; }

    /// Set the radial and the axial tube radius. The value is clamped. The root rebuilds.
    void SetTubeRadius(const Vector2& tubeRadius);
    const Vector2& GetTubeRadius() const { return params_.tubeRadius_; }

    /// Set the number of divisions around the axis. The value is clamped. The root rebuilds.
    void SetSegments(int segments);
    int GetSegments() const { return params_.segments_; }

    /// Set the number of divisions around the tube. The value is clamped. The root rebuilds.
    void SetTubeSegments(int tubeSegments);
    int GetTubeSegments() const { return params_.tubeSegments_; }

    /// Set the arc in degrees. The value is clamped. The root rebuilds.
    void SetArc(float arc);
    float GetArc() const { return params_.arc_; }

    /// Set the tube angle in degrees. The value is clamped. The root rebuilds.
    void SetTubeAngle(float tubeAngle);
    float GetTubeAngle() const { return params_.tubeAngle_; }

    /// Set smooth or flat normals. The root rebuilds.
    void SetSmoothNormals(bool smooth);
    bool GetSmoothNormals() const { return params_.smoothNormals_; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgRingParams& params);
    const CsgRingParams& GetParams() const { return params_; }

    /// Handles: 0 the major radius (axis +X at (radius, 0, 0)), 1 the radial tube radius (axis +X at
    /// (radius + tube x, 0, 0)), 2 the axial tube radius (axis +Y at (radius, tube y, 0)),
    /// 3 the arc end (axis is the tangent (-sin a, 0, cos a) at the point on the major circle at the arc angle a).
    void GetHandles(ea::vector<CsgShapeHandle>& handles) const override;
    /// Groups: Shape. The first one is the default.
    void GetHandleGroups(ea::vector<ea::string>& groups) const override;
    /// Each radius grows by dot(delta, axis). The arc grows by dot(delta, axis) / radius, in degrees.
    /// The sanitizer clamps the result. A shape that is not valid, or no change, returns false.
    bool MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta) override;

protected:
    void SanitizeParameters() override;
    bool GenerateMesh(CsgShapeMesh& mesh) const override;
    unsigned GetNumMaterialSlots() const override { return 6; }

private:
    CsgRingParams params_;
};

} // namespace Urho3D
