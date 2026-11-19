// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Test helpers for brushes with a generated model.

#pragma once

#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgOperationColor.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Scene/Serializable.h>

namespace Urho3D
{

/// Make a box model from raw vertex and index data. The model has 24 vertices and 36 indices.
/// The model has no name and is not in the resource cache.
/// If `skipFace` is below 6, the model has no triangles for that face, so the box is open.
inline SharedPtr<Model> MakeTestBoxModel(Context* context, const Vector3& halfSize, unsigned skipFace = 6)
{
    const Vector3 normals[6] = {Vector3::RIGHT, Vector3::LEFT, Vector3::UP, Vector3::DOWN, Vector3::FORWARD,
        Vector3::BACK};

    GeometryLODView lod;
    lod.primitiveType_ = TRIANGLE_LIST;
    lod.vertexFormat_.position_ = TYPE_VECTOR3;
    lod.vertexFormat_.normal_ = TYPE_VECTOR3;

    unsigned faceIndex = 0;
    for (const Vector3& normal : normals)
    {
        const bool skip = faceIndex++ == skipFace;
        // Pick two axes that are perpendicular to the normal.
        const Vector3 u = normal.y_ != 0.0f ? Vector3::RIGHT : Vector3::UP;
        const Vector3 v = normal.CrossProduct(u);
        const unsigned first = lod.vertices_.size();
        const float signs[4][2] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
        for (const auto& sign : signs)
        {
            ModelVertex vertex{};
            vertex.SetPosition((normal + u * sign[0] + v * sign[1]) * halfSize);
            vertex.SetNormal(normal);
            lod.vertices_.push_back(vertex);
        }
        // The cross product of the first two sides points out of the box.
        const Vector3 p0 = lod.vertices_[first].GetPosition();
        const Vector3 p1 = lod.vertices_[first + 1].GetPosition();
        const Vector3 p2 = lod.vertices_[first + 2].GetPosition();
        const bool flip = (p1 - p0).CrossProduct(p2 - p0).DotProduct(normal) < 0.0f;
        const unsigned order[6] = {0, flip ? 2u : 1u, flip ? 1u : 2u, 0, flip ? 3u : 2u, flip ? 2u : 3u};
        if (skip)
            continue;
        for (const unsigned index : order)
            lod.indices_.push_back(first + index);
    }

    GeometryView geometry;
    geometry.lods_.push_back(lod);

    ModelView modelView(context);
    modelView.SetGeometries({geometry});
    return modelView.ExportModel();
}

/// A brush with a generated model. It has no "Model" attribute.
class TestGeneratedBrush : public CsgBrush
{
    URHO3D_OBJECT(TestGeneratedBrush, CsgBrush);

public:
    explicit TestGeneratedBrush(Context* context)
        : CsgBrush(context)
    {
    }

    static void RegisterObject(Context* context)
    {
        context->AddFactoryReflection<TestGeneratedBrush>();
        URHO3D_COPY_BASE_ATTRIBUTES(CsgBrush);
        URHO3D_REMOVE_ATTRIBUTE("Model");
    }

    bool GetGeneratedModel(SharedPtr<Model>& model) const override
    {
        model = model_;
        return true;
    }

    /// Draw the generated model. A brush with a generated model has no model name for the cache.
    void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override
    {
        DrawDebugGeometryInternal(debug, depthTest, model_.Get(), GetOperationColor(static_cast<int>(GetOperation())));
    }

    /// Set the generated model. A null model makes the brush invalid. The root rebuilds.
    void SetGeneratedModel(Model* model)
    {
        model_ = model;
        MarkDirty();
    }

private:
    SharedPtr<Model> model_;
};

} // namespace Urho3D
