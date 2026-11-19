// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Test helper: measure a triangle model.

#pragma once

#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/ModelView.h>

namespace Urho3D
{

/// Return the volume of a closed triangle model: the sum of the signed tetrahedra to the origin.
inline double MeasureModelVolume(Context* context, const Model* model)
{
    ModelView view(context);
    if (!model || !view.ImportModel(model))
        return 0.0;
    double sum = 0.0;
    for (const GeometryView& geometry : view.GetGeometries())
    {
        const GeometryLODView& lod = geometry.lods_[0];
        for (unsigned i = 0; i + 2 < lod.indices_.size(); i += 3)
        {
            const Vector3 a = lod.vertices_[lod.indices_[i]].GetPosition();
            const Vector3 b = lod.vertices_[lod.indices_[i + 1]].GetPosition();
            const Vector3 c = lod.vertices_[lod.indices_[i + 2]].GetPosition();
            sum += static_cast<double>(a.DotProduct(b.CrossProduct(c))) / 6.0;
        }
    }
    return sum;
}

} // namespace Urho3D
