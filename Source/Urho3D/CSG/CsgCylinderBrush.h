// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural cylinder, cone, frustum, prism or pyramid brush. See `CsgGenerateCylinder`.
///
/// Attributes: "Bottom Center", "Top Center", "Bottom Radius", "Top Radius", "Segments", "Height Segments",
/// "Bulge", "Smooth Normals", "Start Angle" and "Face Radius". The values are always clamped. See `CsgSanitizeCylinderParams`.
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

    /// Set the angle of the first vertex in degrees. It wraps into [0, 360). The root rebuilds.
    void SetStartAngle(float degrees);
    float GetStartAngle() const { return params_.startAngle_; }

    /// Set face radius. True: the radii reach the middle of each side face of the polygon. The root rebuilds.
    void SetFaceRadius(bool faceRadius);
    bool GetFaceRadius() const { return params_.faceRadius_; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgCylinderParams& params);
    const CsgCylinderParams& GetParams() const { return params_; }

    /// Handles, with stable ids:
    /// - 0 the bottom center and 1 the top center (points).
    /// - 2 to 5 the bottom radius at +X, -X, +Z, -Z and 6 to 9 the top radius at the same places (axes).
    ///   The +X and -X handles of one end edit the same component.
    ///   A handle sits on the polygon boundary along its axis: center + axis * radius * k.
    ///   k is the distance from the center to the unit polygon along the axis. For the default cylinder k is 1.
    ///   An end that is an apex (radius pair (0, 0)) has no radius handle. The other ids do not shift.
    /// - 10 the bulge (axis +X, at the middle of the center line). It is absent if the mean X radius
    ///   is not above 0.001.
    /// - 11 the bottom scale and 12 the top scale (axes). Each sits on the rim of its end at 45 degrees in XZ.
    ///   The axis points from the center to the handle. At an apex the handle sits at the end center
    ///   and the axis is normalize(1, 0, 1). The handle count is 13 at most.
    void GetHandles(ea::vector<CsgShapeHandle>& handles) const override;
    /// Groups: Ends, Radius. The first one is the default.
    void GetHandleGroups(ea::vector<ea::string>& groups) const override;
    /// One line: the axis from the bottom center (id 0) to the top center (id 1).
    void GetCageLines(ea::vector<ea::pair<unsigned, unsigned>>& lines) const override;
    /// A center moves by the full delta. A radius component grows by dot(delta, axis) / k.
    /// A scale handle multiplies both radius components by the same factor, so the handle follows the delta.
    /// If both components fall below 0.001 the end becomes an apex. A drag of an apex handle sets the radius pair
    /// to (r, r) with r from the distance along the axis, or keeps the apex.
    /// The bulge grows by dot(delta, axis) divided by the mean X radius.
    /// The sanitizer clamps the result. A shape that is not valid, or no change, returns false.
    bool MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta) override;
    /// The center is the midpoint of the two end centers. Both centers move by -center.
    bool Recenter(Vector3& localOffset) override;

protected:
    void SanitizeParameters() override;
    bool GenerateMesh(CsgShapeMesh& mesh) const override;
    unsigned GetNumMaterialSlots() const override { return 3; }

private:
    CsgCylinderParams params_;
};

} // namespace Urho3D
