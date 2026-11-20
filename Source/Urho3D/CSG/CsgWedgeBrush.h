// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural wedge brush (a right-angle ramp). See `CsgGenerateWedge`.
///
/// Attributes: "Size" and "Center". The values are always clamped. See `CsgSanitizeWedgeParams`.
/// The slope rises from the front bottom edge (-Z, -Y) to the back top edge (+Z, +Y).
///
/// The brush has five material slots, in this order: 0 the bottom, 1 the back, 2 the slope,
/// 3 the left triangle (-X), 4 the right triangle (+X).
class PLUGIN_RBFXCSG_API CsgWedgeBrush : public CsgProceduralBrush
{
    URHO3D_OBJECT(CsgWedgeBrush, CsgProceduralBrush);

public:
    explicit CsgWedgeBrush(Context* context);
    ~CsgWedgeBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Set the size of the bounding box. The value is clamped. The root rebuilds.
    void SetSize(const Vector3& size);
    const Vector3& GetSize() const { return params_.size_; }

    /// Set the center of the shape in the local space. The value is clamped. The root rebuilds.
    void SetCenter(const Vector3& center);
    const Vector3& GetCenter() const { return params_.center_; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgWedgeParams& params);
    const CsgWedgeParams& GetParams() const { return params_; }

    /// Handles (axes), with h = Size / 2 and c = Center:
    /// 0 -X at the centroid of the left triangle (c.x - h.x, c.y - h.y / 3, c.z + h.z / 3), axis -X;
    /// 1 +X at the centroid of the right triangle, axis +X;
    /// 2 -Z at the middle of the front bottom edge (c.x, c.y - h.y, c.z - h.z), axis -Z;
    /// 3 +Z at the center of the back face (c.x, c.y, c.z + h.z), axis +Z;
    /// 4 +Y at the middle of the back top edge (c.x, c.y + h.y, c.z + h.z), axis +Y;
    /// 5 -Y at the center of the bottom face (c.x, c.y - h.y, c.z), axis -Y.
    /// Ids 6 to 8 are the size handles (axes) +X, +Y and +Z, named 'Size X', 'Size Y' and 'Size Z'.
    /// Each one is at the position of the + face handle of its axis (ids 1, 4 and 3), with the + axis.
    /// A size handle grows the size on its axis by 2 d, d = dot(delta, axis). The center stays.
    /// Groups: Face (0 to 5, default) and Size (6 to 8). The ids are a file contract.
    void GetHandles(ea::vector<CsgShapeHandle>& handles) const override;
    void GetHandleGroups(ea::vector<ea::string>& groups) const override;
    /// The size on the axis of the handle grows by d = dot(delta, axis). The center moves by axis * d / 2,
    /// so the opposite face stays fixed. The sanitizer clamps the result. No change returns false.
    bool MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta) override;
    /// The offset is "Center". The center becomes zero.
    bool Recenter(Vector3& localOffset) override;

protected:
    void SanitizeParameters() override;
    bool GenerateMesh(CsgShapeMesh& mesh) const override;
    unsigned GetNumMaterialSlots() const override { return 5; }

private:
    CsgWedgeParams params_;
};

} // namespace Urho3D
