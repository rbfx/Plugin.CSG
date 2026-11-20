// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural arch brush (an elliptic annular sector, extruded along Z). See `CsgGenerateArch`.
///
/// Attributes: "Radius", "Thickness", "Depth", "Arc", "Segments" and "Center". The arc is in degrees.
/// The values are always clamped. See `CsgSanitizeArchParams`. The arch is symmetric about +Y.
///
/// The brush has five material slots, in this order: 0 the outer curved face, 1 the inner curved face,
/// 2 the front (+Z), 3 the back (-Z), 4 the two ends. A filled sector has no inner face. Its slot stays free.
class PLUGIN_RBFXCSG_API CsgArchBrush : public CsgProceduralBrush
{
    URHO3D_OBJECT(CsgArchBrush, CsgProceduralBrush);

public:
    explicit CsgArchBrush(Context* context);
    ~CsgArchBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Set the outer radius along X and Y. The value is clamped. The root rebuilds.
    void SetRadius(const Vector2& radius);
    const Vector2& GetRadius() const { return params_.radius_; }

    /// Set the wall thickness. The value is clamped. The root rebuilds.
    void SetThickness(float thickness);
    float GetThickness() const { return params_.thickness_; }

    /// Set the extrusion along Z. The value is clamped. The root rebuilds.
    void SetDepth(float depth);
    float GetDepth() const { return params_.depth_; }

    /// Set the arc in degrees. The value is clamped. The root rebuilds.
    void SetArc(float arc);
    float GetArc() const { return params_.arc_; }

    /// Set the number of divisions of the arc. The value is clamped. The root rebuilds.
    void SetSegments(int segments);
    int GetSegments() const { return params_.segments_; }

    /// Set the center of the curves in the local space. The value is clamped. The root rebuilds.
    void SetCenter(const Vector3& center);
    const Vector3& GetCenter() const { return params_.center_; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgArchParams& params);
    const CsgArchParams& GetParams() const { return params_; }

    /// Handles (axes), with c = Center, r = Radius, i = the inner radius (`CsgGetArchInnerRadius`),
    /// a0 and a1 = the start and the end angle:
    /// 0 the X radius at the start end of the outer curve c + (r.x cos a0, r.y sin a0, 0), axis +X;
    /// 1 the Y radius at the top of the outer curve c + (0, r.y, 0), axis +Y;
    /// 2 the thickness at the top of the inner curve c + (0, i.y, 0), axis -Y (into the hole);
    /// 3 the depth at the front c + (0, (r.y + i.y) / 2, Depth / 2), axis +Z;
    /// 4 the arc end at the end of the outer curve c + (r.x cos a1, r.y sin a1, 0),
    ///   axis = the unit tangent (-r.x sin a1, r.y cos a1, 0) in the direction of a larger angle.
    /// The top handles are on the mesh if "Segments" is even. Otherwise they are on the ellipse, a little outside.
    void GetHandles(ea::vector<CsgShapeHandle>& handles) const override;
    /// Groups: Shape. The first one is the default.
    void GetHandleGroups(ea::vector<ea::string>& groups) const override;
    /// With d = dot(delta, axis): 0 radius.x += d; 1 radius.y += d; 2 thickness += d;
    /// 3 depth += d and center.z += d / 2, so the back stays fixed;
    /// 4 arc += 2 d / s in radians, with s = |(-r.x sin a1, r.y cos a1)|. Both ends move by d / s, so the handle
    /// follows the cursor. The sanitizer clamps the result. A shape that is not valid, or no change, returns false.
    bool MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta) override;
    /// The offset is "Center". The center becomes zero.
    bool Recenter(Vector3& localOffset) override;

protected:
    void SanitizeParameters() override;
    bool GenerateMesh(CsgShapeMesh& mesh) const override;
    unsigned GetNumMaterialSlots() const override { return 5; }

private:
    CsgArchParams params_;
};

} // namespace Urho3D
