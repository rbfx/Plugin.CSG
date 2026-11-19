// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

/// A procedural UV sphere brush with three radii. See `CsgGenerateSphere`.
///
/// Attributes: "Radius" (each component in [0.001, 16384]), "Segments" (3 to 128), "Rings" (2 to 64)
/// and "Smooth Normals". The poles are on the Y axis. The values are always valid after clamping.
///
/// The sphere has one material slot for the whole surface.
class PLUGIN_RBFXCSG_API CsgSphereBrush : public CsgProceduralBrush
{
    URHO3D_OBJECT(CsgSphereBrush, CsgProceduralBrush);

public:
    explicit CsgSphereBrush(Context* context);
    ~CsgSphereBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Set the radius. The value is clamped. The root rebuilds.
    void SetRadius(const Vector3& radius);
    const Vector3& GetRadius() const { return params_.radius_; }

    /// Set the number of longitude divisions. The value is clamped. The root rebuilds.
    void SetSegments(int segments);
    int GetSegments() const { return params_.segments_; }

    /// Set the number of latitude bands. The value is clamped. The root rebuilds.
    void SetRings(int rings);
    int GetRings() const { return params_.rings_; }

    /// Set smooth or flat normals. The root rebuilds.
    void SetSmoothNormals(bool smooth);
    bool GetSmoothNormals() const { return params_.smoothNormals_; }

    /// Set all parameters. The values are clamped. The root rebuilds.
    void SetParams(const CsgSphereParams& params);
    const CsgSphereParams& GetParams() const { return params_; }

protected:
    void SanitizeParameters() override;
    bool GenerateMesh(CsgShapeMesh& mesh) const override;
    unsigned GetNumMaterialSlots() const override { return 1; }

private:
    CsgSphereParams params_;
};

} // namespace Urho3D
