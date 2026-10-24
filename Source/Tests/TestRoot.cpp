// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Headless regression test for CsgRoot. The test uses no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSG/CsgRootInternal.h"
#include "Urho3D/CSG/CsgSolid.h"

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

#include <cmath>
#include <limits>

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

/// Build a box minus a box with a root that has the given scale. Return the bounds of the output model.
BoundingBox BuildDifferenceBounds(Context* context, Scene* scene, WorkQueue* workQueue, const ea::string& nodeName,
    const Vector3& rootScale)
{
    Node* rootNode = scene->CreateChild(nodeName);
    rootNode->SetScale(rootScale);
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
    REQUIRE(output->GetModel());
    return output->GetModel()->GetBoundingBox();
}

/// Create a root node with the box A base model and no delay for the rebuild.
CsgRoot* MakeBoxRoot(Scene* scene, const ea::string& nodeName)
{
    auto* root = scene->CreateChild(nodeName)->CreateComponent<CsgRoot>();
    root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxA.mdl"));
    root->SetDeferredDelaySec(0.0f);
    return root;
}

/// Add a brush with the box B model under the node of the root.
CsgBrush* AddBoxBrush(CsgRoot* root, const ea::string& name, const Vector3& position, CsgOperation op)
{
    Node* brushNode = root->GetNode()->CreateChild(name);
    brushNode->SetPosition(position);
    auto* brush = brushNode->CreateComponent<CsgBrush>();
    brush->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxB.mdl"));
    brush->SetOperation(op);
    return brush;
}

/// Run the work queue and the time for a fixed number of frames.
void RunFrames(CsgRoot* root, WorkQueue* workQueue, unsigned frames)
{
    auto* time = root->GetSubsystem<Time>();
    for (unsigned i = 0; i < frames; ++i)
    {
        workQueue->CompleteAll();
        time->BeginFrame(1.0f / 60.0f);
        root->GetScene()->Update(1.0f / 60.0f);
        time->EndFrame();
    }
}

StaticModel* GetOutput(CsgRoot* root) { return root->GetNode()->GetComponent<StaticModel>(); }

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

TEST_CASE("CsgRoot output is in node space for a scaled root", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    const BoundingBox expected = BuildDifferenceBounds(context, scene, workQueue, "Unscaled", Vector3::ONE);
    const BoundingBox scaled = BuildDifferenceBounds(context, scene, workQueue, "Scaled", Vector3(2.0f, 1.0f, 1.0f));

    CHECK(scaled.min_.Equals(expected.min_, 1e-3f));
    CHECK(scaled.max_.Equals(expected.max_, 1e-3f));
    CHECK(expected.max_.x_ - expected.min_.x_ > 1.0f);
}

TEST_CASE("CsgInternal::GetMetricScale replaces degenerate axes", "[root]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    CHECK(CsgInternal::GetMetricScale(Vector3(0.0f, nan, -1.0f)) == Vector3::ONE);
    CHECK(CsgInternal::GetMetricScale(Vector3(inf, 2.0f, 1e-7f)) == Vector3(1.0f, 2.0f, 1.0f));
    CHECK(CsgInternal::GetMetricScale(Vector3(2.0f, 3.0f, 0.5f)) == Vector3(2.0f, 3.0f, 0.5f));
}

TEST_CASE("CsgInternal::ToNodeSpace applies the inverse scale", "[root]")
{
    auto model = MakeShared<CsgTriangulatedModel>();
    CsgTriangulatedModel::Geometry geometry;
    ModelVertex vertex{};
    vertex.SetPosition(Vector3(4.0f, 2.0f, 6.0f));
    vertex.SetNormal(Vector3(1.0f, 1.0f, 0.0f).Normalized());
    vertex.tangent_ = Vector4(1.0f, 0.0f, 0.0f, -1.0f);
    geometry.vertices_.push_back(vertex);
    model->geometries_.push_back(geometry);
    model->boundingBox_ = BoundingBox(Vector3(-4.0f, -2.0f, -6.0f), Vector3(4.0f, 2.0f, 6.0f));
    model->hasAnyVertex_ = true;

    CsgInternal::ToNodeSpace(*model, Vector3(2.0f, 1.0f, 1.0f));

    const ModelVertex& result = model->geometries_[0].vertices_[0];
    CHECK(result.GetPosition().Equals(Vector3(2.0f, 2.0f, 6.0f), 1e-5f));
    CHECK(result.GetNormal().Length() == Catch::Approx(1.0f).margin(1e-5f));
    CHECK(result.GetNormal().x_ > result.GetNormal().y_);
    CHECK(result.tangent_.w_ == -1.0f);
    CHECK(result.GetTangent().Length() == Catch::Approx(1.0f).margin(1e-5f));
    CHECK(model->boundingBox_.min_.Equals(Vector3(-2.0f, -2.0f, -6.0f), 1e-5f));
    CHECK(model->boundingBox_.max_.Equals(Vector3(2.0f, 2.0f, 6.0f), 1e-5f));
}

TEST_CASE("CsgSolid from an empty operand is empty and works in a boolean", "[root]")
{
    auto context = MakeCsgTestContext();
    SharedPtr<Model> box = AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);

    SharedPtr<CsgSolid> empty = CsgSolid::FromOperand({}, 1e-4f);
    REQUIRE(empty);
    CHECK(empty->FaceCount() == 0);

    SharedPtr<CsgSolid> solid = CsgSolid::FromOperand(CsgBuildPolygonsFromModel(box, Matrix3x4::IDENTITY, 0), 1e-4f);
    REQUIRE(solid);
    REQUIRE(solid->FaceCount() != 0);

    for (const CsgOperation op : {CsgOperation::Union, CsgOperation::DifferenceAB, CsgOperation::Intersection})
    {
        SharedPtr<const CsgSolid> result = CsgBooleanOperation(empty.Get(), solid.Get(), op, 1e-4f);
        REQUIRE(result);
        CHECK((result->FaceCount() != 0) == (op == CsgOperation::Union));
        result = CsgBooleanOperation(solid.Get(), empty.Get(), op, 1e-4f);
        REQUIRE(result);
        CHECK((result->FaceCount() != 0) == (op != CsgOperation::Intersection));
    }
}

TEST_CASE("CsgRoot appends a brush on the cached solid", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // Build with one brush. The root caches the solid.
    CsgRoot* root = MakeBoxRoot(scene, "Appended");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    const unsigned oneBrushIndices = CountIndices(GetOutput(root)->GetModel());

    // Append a second brush. The rebuild evaluates only the new brush.
    AddBoxBrush(root, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    const Model* appended = GetOutput(root)->GetModel();
    REQUIRE(appended);
    CHECK(CountIndices(appended) > oneBrushIndices);
    CHECK(appended->GetBoundingBox().max_.x_ == Catch::Approx(4.0f).margin(1e-3f));

    // A fresh root without a cache must give the same output.
    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    AddBoxBrush(fresh, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    AddBoxBrush(fresh, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    fresh->Rebuild();
    REQUIRE(RunUntilOutput(fresh, workQueue, 1000));
    const Model* expected = GetOutput(fresh)->GetModel();
    CHECK(CountIndices(appended) == CountIndices(expected));
    CHECK(appended->GetNumGeometries() == expected->GetNumGeometries());
    CHECK(appended->GetBoundingBox().min_.Equals(expected->GetBoundingBox().min_, 1e-3f));
    CHECK(appended->GetBoundingBox().max_.Equals(expected->GetBoundingBox().max_, 1e-3f));
}

TEST_CASE("CsgRoot does not reuse the cached solid after a prefix brush moves", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Moved");
    CsgBrush* cut = AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    const unsigned cutIndices = CountIndices(GetOutput(root)->GetModel());
    CHECK(cutIndices > 36);

    // Move the first brush away from the base and append a brush that cuts nothing.
    cut->GetNode()->SetPosition(Vector3(10.0f, 10.0f, 10.0f));
    AddBoxBrush(root, "Far", Vector3(-10.0f, -10.0f, -10.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    RunFrames(root, workQueue, 60);

    // Neither brush touches the base, so the output is the plain box. A stale solid keeps the cut.
    CHECK(CountIndices(GetOutput(root)->GetModel()) == 36);
}

TEST_CASE("CsgRoot queues a result while auto-apply is off", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Pending");
    CHECK(root->GetAutoApplyResults());
    root->SetAutoApplyResults(false);
    CHECK(!root->GetAutoApplyResults());
    CHECK(!root->HasPendingResult());
    CHECK(root->GetPendingResultRevision() == 0);

    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    RunFrames(root, workQueue, 60);

    CHECK(root->HasPendingResult());
    CHECK(root->GetPendingResultRevision() == root->GetRebuildRevision());
    StaticModel* output = GetOutput(root);
    // A rebuild without brushes applies the plain base model at once. The queued result is not in the output.
    CHECK((!output || !output->GetModel() || CountIndices(output->GetModel()) == 36));

    CHECK(root->ApplyPendingResult());
    output = GetOutput(root);
    REQUIRE(output);
    REQUIRE(output->GetModel());
    CHECK(CountIndices(output->GetModel()) > 36);
    CHECK(!root->HasPendingResult());
    CHECK(!root->ApplyPendingResult());
}
