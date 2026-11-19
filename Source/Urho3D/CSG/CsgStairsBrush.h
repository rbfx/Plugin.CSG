// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural solid stairs brush. See `CsgGenerateStairs`.
///
/// Attributes: "Size", "Center" and "Steps". The values are always clamped. See `CsgSanitizeStairsParams`.
/// The first riser is at the front (-Z). The top tread ends at the back top edge (+Z, +Y).
///
/// The brush has five material slots, in this order: 0 the treads, 1 the risers, 2 the sides (-X and +X),
/// 3 the bottom, 4 the back.
class PLUGIN_RBFXCSG_API CsgStairsBrush : public CsgProceduralBrush
{
    URHO3D_OBJECT(CsgStairsBrush, CsgProceduralBrush);

public:
    explicit CsgStairsBrush(Context* context);
    ~CsgStairsBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Set the size of the bounding box. The value is clamped. The root rebuilds.
    void SetSize(const Vector3& size);
    const Vector3& GetSize() const { return params_.size_; }

    /// Set the center of the shape in the local space. The value is clamped. The root rebuilds.
    void SetCenter(const Vector3& center);
    const Vector3& GetCenter() const { return params_.center_; }

    /// Set the number of steps. The value is clamped. The root rebuilds.
    void SetSteps(int steps);
    int GetSteps() const { return params_.steps_; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgStairsParams& params);
    const CsgStairsParams& GetParams() const { return params_; }

    /// Handles (axes), with h = Size / 2, c = Center and n = Steps:
    /// 0 -X at the center of the left side (c.x - h.x, c.y, c.z), axis -X. The point is inside the profile.
    /// 1 +X at the center of the right side (c.x + h.x, c.y, c.z), axis +X;
    /// 2 -Z at the center of the first riser (c.x, c.y - h.y + Size.y / (2 n), c.z - h.z), axis -Z;
    /// 3 +Z at the center of the back face (c.x, c.y, c.z + h.z), axis +Z;
    /// 4 +Y at the center of the top tread (c.x, c.y + h.y, c.z + h.z - Size.z / (2 n)), axis +Y;
    /// 5 -Y at the center of the bottom face (c.x, c.y - h.y, c.z), axis -Y.
    /// There is no handle for the steps.
    void GetHandles(ea::vector<CsgShapeHandle>& handles) const override;
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
    CsgStairsParams params_;
};

} // namespace Urho3D
