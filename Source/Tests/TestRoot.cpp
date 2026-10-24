// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Headless regression test for CsgRoot. The test uses no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgRoot.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Timer.h>
#include <Urho3D/Core/WorkQueue.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>

#include <catch2/catch_amalgamated.hpp>

using namespace Urho3D;

namespace
{

/// Create a context with the subsystems and the object factories that CsgRoot needs.
SharedPtr<Context> MakeCsgTestContext()
{
    auto context = MakeShared<Context>();
    context->RegisterSubsystem<Log>();
    context->RegisterSubsystem<Time>();
    context->RegisterSubsystem<WorkQueue>();
    context->RegisterSubsystem<ResourceCache>();
    RegisterSceneLibrary(context);
    RegisterGraphicsLibrary(context);
    RegisterCsgLibrary(context);
    return context;
}

/// Add a box model to the resource cache. The model has 24 vertices and 36 indices.
SharedPtr<Model> AddBoxModel(Context* context, const ea::string& name, const Vector3& halfSize)
{
    const Vector3 normals[6] = {Vector3::RIGHT, Vector3::LEFT, Vector3::UP, Vector3::DOWN, Vector3::FORWARD,
        Vector3::BACK};

    GeometryLODView lod;
    lod.primitiveType_ = TRIANGLE_LIST;
    lod.vertexFormat_.position_ = TYPE_VECTOR3;
    lod.vertexFormat_.normal_ = TYPE_VECTOR3;

    for (const Vector3& normal : normals)
    {
        // Pick two axes that are perpendicular to the normal.
        const Vector3 u = normal.y_ != 0.0f ? Vector3::RIGHT : Vector3::UP;
        const Vector3 v = normal.CrossProduct(u);
        const unsigned first = lod.vertices_.size();
        const float signs[4][2] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
        for (const auto& sign : signs)
        {
            ModelVertex vertex{};
            const Vector3 offset = normal + u * sign[0] + v * sign[1];
            vertex.SetPosition(offset * halfSize);
            vertex.SetNormal(normal);
            lod.vertices_.push_back(vertex);
        }
        // The winding is clockwise when seen from outside.
        const Vector3 p0 = lod.vertices_[first].GetPosition();
        const Vector3 p1 = lod.vertices_[first + 1].GetPosition();
        const Vector3 p2 = lod.vertices_[first + 2].GetPosition();
        const bool flip = (p1 - p0).CrossProduct(p2 - p0).DotProduct(normal) > 0.0f;
        const unsigned order[6] = {0, flip ? 2u : 1u, flip ? 1u : 2u, 0, flip ? 3u : 2u, flip ? 2u : 3u};
        for (const unsigned index : order)
            lod.indices_.push_back(first + index);
    }

    GeometryView geometry;
    geometry.lods_.push_back(lod);

    ModelView modelView(context);
    modelView.SetName(name);
    modelView.SetGeometries({geometry});

    SharedPtr<Model> model = modelView.ExportModel();
    model->SetName(name);
    context->GetSubsystem<ResourceCache>()->AddManualResource(model);
    return model;
}

/// Run the work queue and the time until the output model of the root has geometry.
bool RunUntilOutput(CsgRoot* root, WorkQueue* workQueue, unsigned maxIterations)
{
    auto* time = root->GetSubsystem<Time>();
    for (unsigned i = 0; i < maxIterations; ++i)
    {
        workQueue->CompleteAll();
        time->BeginFrame(1.0f / 60.0f);
        root->GetScene()->Update(1.0f / 60.0f);
        time->EndFrame();

        auto* output = root->GetNode()->GetComponent<StaticModel>();
        if (output && output->GetModel() && output->GetModel()->GetNumGeometries() > 0)
            return true;
    }
    return false;
}

unsigned CountIndices(const Model* model)
{
    unsigned count = 0;
    for (unsigned i = 0; i < model->GetNumGeometries(); ++i)
        count += model->GetGeometry(i, 0)->GetIndexCount();
    return count;
}

} // namespace

TEST_CASE("CsgRoot builds a model from a root and a brush")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();
    Node* rootNode = scene->CreateChild("Root");
    auto* root = rootNode->CreateComponent<CsgRoot>();
    root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxA.mdl"));
    root->SetDeferredDelaySec(0.0f);

    Node* brushNode = rootNode->CreateChild("Brush");
    brushNode->SetPosition(Vector3(1.0f, 1.0f, 1.0f));
    auto* brush = brushNode->CreateComponent<CsgBrush>();
    brush->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxB.mdl"));
    brush->SetOperation(CsgOperation::DifferenceAB);

    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));

    auto* output = rootNode->GetComponent<StaticModel>();
    REQUIRE(output);
    const Model* model = output->GetModel();
    REQUIRE(model);
    CHECK(model->GetNumGeometries() >= 1);
    CHECK(CountIndices(model) > 36);
}
