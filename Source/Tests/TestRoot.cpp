// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Headless regression test for CsgRoot. The test uses no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoxBrush.h"
#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgCylinderBrush.h"
#include "Urho3D/CSG/CsgRingBrush.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSG/CsgOperationColor.h"
#include "Urho3D/CSG/CsgRootInternal.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/CsgSphereBrush.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "TestGeneratedBrush.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Timer.h>
#include <Urho3D/Core/WorkQueue.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/IOEvents.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/IO/VectorBuffer.h>
#include <Urho3D/IO/VirtualFileSystem.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Resource/ResourceEvents.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>

#include <catch2/catch_amalgamated.hpp>

#include <atomic>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <limits>
#include <chrono>
#include <stdexcept>
#include <thread>

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
    TestGeneratedBrush::RegisterObject(context);
    return context;
}

/// Add a box model to the resource cache. The model has 24 vertices and 36 indices.
/// If `skipFace` is below 6, the model has no triangles for that face, so the box is open.
SharedPtr<Model> AddBoxModel(Context* context, const ea::string& name, const Vector3& halfSize, unsigned skipFace = 6)
{
    SharedPtr<Model> model = MakeTestBoxModel(context, halfSize, skipFace);
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

        auto* output = root->GetResultModel();
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

    auto* output = root->GetResultModel();
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

StaticModel* GetOutput(CsgRoot* root) { return root->GetResultModel(); }

/// Count the direct children of the node with the name of the result node.
unsigned CountResultNodes(Node* node)
{
    unsigned count = 0;
    for (Node* child : node->GetChildren())
        count += child->GetName() == "CsgResult" ? 1 : 0;
    return count;
}

/// Run the scene and the work queue for a fixed number of frames. The function needs no root.
void RunSceneFrames(Scene* scene, WorkQueue* workQueue, unsigned frames)
{
    auto* time = scene->GetSubsystem<Time>();
    for (unsigned i = 0; i < frames; ++i)
    {
        workQueue->CompleteAll();
        time->BeginFrame(1.0f / 60.0f);
        scene->Update(1.0f / 60.0f);
        time->EndFrame();
    }
}

/// Remove the test build hook when the scope ends. A failed check must not leave the hook for other tests.
struct TestBuildHookGuard
{
    TestBuildHookGuard() = default;
    TestBuildHookGuard(const TestBuildHookGuard&) = delete;
    TestBuildHookGuard& operator=(const TestBuildHookGuard&) = delete;
    ~TestBuildHookGuard() { CsgRoot::SetTestBuildHook(nullptr); }
};

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

    auto* output = root->GetResultModel();
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

TEST_CASE("CsgInternal::RescalePolygons scales positions and fixes normals", "[root]")
{
    auto context = MakeCsgTestContext();
    SharedPtr<Model> box = AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    SharedPtr<CsgSolid> solid = CsgSolid::FromOperand(CsgBuildPolygonsFromModel(box, Matrix3x4::IDENTITY, 0), 1e-4f);
    REQUIRE(solid);
    REQUIRE(solid->FaceCount() != 0);

    const ea::vector<CsgPolygon> scaled = CsgInternal::RescalePolygons(*solid, Vector3(2.0f, 1.0f, 0.5f));
    REQUIRE(scaled.size() == solid->FaceCount());
    for (unsigned i = 0; i < scaled.size(); ++i)
    {
        const CsgPolygon& source = *solid->GetFace(i);
        const CsgPolygon& result = scaled[i];
        CHECK(result.shared_ == source.shared_);
        REQUIRE(result.vertices_.size() == source.vertices_.size());
        for (unsigned v = 0; v < result.vertices_.size(); ++v)
        {
            CHECK(result.vertices_[v].GetPosition().Equals(source.vertices_[v].GetPosition() * Vector3(2.0f, 1.0f, 0.5f), 1e-5f));
            CHECK(result.vertices_[v].GetNormal().Length() == Catch::Approx(1.0f).margin(1e-5f));
            // The test box can have normals that differ from the winding. The scale keeps their direction.
            CHECK(result.vertices_[v].GetNormal().DotProduct(source.vertices_[v].GetNormal()) > 0.999f);
            CHECK(result.plane_.normal_.DotProduct(source.plane_.normal_) > 0.999f);
        }
        // The plane passes through the scaled corners.
        CHECK(Abs(result.plane_.Distance(result.vertices_[0].GetPosition())) < 1e-4f);
    }

    const ea::vector<CsgPolygon> same = CsgInternal::RescalePolygons(*solid, Vector3::ONE);
    REQUIRE(same.size() == solid->FaceCount());
    for (unsigned i = 0; i < same.size(); ++i)
    {
        const CsgPolygon& source = *solid->GetFace(i);
        REQUIRE(same[i].vertices_.size() == source.vertices_.size());
        CHECK(same[i].shared_ == source.shared_);
        CHECK(same[i].plane_.normal_ == source.plane_.normal_);
        for (unsigned v = 0; v < source.vertices_.size(); ++v)
        {
            CHECK(same[i].vertices_[v].GetPosition() == source.vertices_[v].GetPosition());
            CHECK(same[i].vertices_[v].GetNormal() == source.vertices_[v].GetNormal());
        }
    }
}

TEST_CASE("CsgRoot bakes the brushes and appends on the baked result", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Baked");
    // A root without brushes cannot bake.
    CHECK(!root->Bake());

    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    AddBoxBrush(root, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    // The root needs a rebuild: the bake fails.
    CHECK(!root->Bake());
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 10);
    const CsgBuildStats stats = root->GetLastBuildStats();
    CHECK(stats.valid_);
    CHECK(stats.degenerates_ == 0);

    const Model* bakedModel = GetOutput(root)->GetModel();
    const unsigned bakedIndices = CountIndices(bakedModel);
    REQUIRE(root->Bake());
    CHECK(root->GetNode()->GetChild("Cut") == nullptr);
    CHECK(root->GetNode()->GetChild("Extension") == nullptr);
    // Nothing to bake any more.
    CHECK(!root->Bake());
    RunFrames(root, workQueue, 60);
    CHECK(GetOutput(root)->GetModel() == bakedModel);
    CHECK(CountIndices(GetOutput(root)->GetModel()) == bakedIndices);

    // A new brush applies to the baked result.
    AddBoxBrush(root, "Third", Vector3(0.0f, 3.0f, 0.0f), CsgOperation::Union);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    const Model* appended = GetOutput(root)->GetModel();
    REQUIRE(appended);
    CHECK(appended != bakedModel);
    CHECK(CountIndices(appended) > bakedIndices);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    AddBoxBrush(fresh, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    AddBoxBrush(fresh, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    AddBoxBrush(fresh, "Third", Vector3(0.0f, 3.0f, 0.0f), CsgOperation::Union);
    fresh->Rebuild();
    REQUIRE(RunUntilOutput(fresh, workQueue, 1000));
    const CsgBuildStats freshStats = fresh->GetLastBuildStats();
    CHECK(freshStats.valid_);
    CHECK(freshStats.degenerates_ == 0);
    const Model* expected = GetOutput(fresh)->GetModel();
    CHECK(CountIndices(appended) == CountIndices(expected));
    CHECK(appended->GetBoundingBox().min_.Equals(expected->GetBoundingBox().min_, 1e-3f));
    CHECK(appended->GetBoundingBox().max_.Equals(expected->GetBoundingBox().max_, 1e-3f));
}

TEST_CASE("CsgRoot refuses to bake while a result is pending", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "PendingBake");
    root->SetAutoApplyResults(false);
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    REQUIRE(root->HasPendingResult());
    CHECK(!root->Bake());
    CHECK(root->GetNode()->GetChild("Cut") != nullptr);
}

TEST_CASE("CsgRoot discards the bake when the root model changes", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxC.mdl", Vector3(2.0f, 2.0f, 2.0f));

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "BakeDiscard");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 10);
    const CsgBuildStats stats = root->GetLastBuildStats();
    CHECK(stats.valid_);
    CHECK(stats.degenerates_ == 0);
    REQUIRE(root->Bake());
    RunFrames(root, workQueue, 60);
    CHECK(CountIndices(GetOutput(root)->GetModel()) > 36);

    // The new base model replaces the baked result. The output is the plain new box.
    root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxC.mdl"));
    RunFrames(root, workQueue, 60);
    const Model* model = GetOutput(root)->GetModel();
    REQUIRE(model);
    CHECK(CountIndices(model) == 36);
    CHECK(model->GetBoundingBox().max_.Equals(Vector3(2.0f, 2.0f, 2.0f), 1e-3f));
}

TEST_CASE("CsgRoot keeps the baked result for a transformed root", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    const Vector3 positions[] = {Vector3(0.5f, 0.0f, 0.0f), Vector3::ZERO};
    const Vector3 scales[] = {Vector3::ONE, Vector3(2.0f, 1.0f, 1.0f)};
    for (unsigned i = 0; i < 2; ++i)
    {
        CsgRoot* root = MakeBoxRoot(scene, i == 0 ? "MovedBake" : "ScaledBake");
        root->GetNode()->SetPosition(positions[i]);
        root->GetNode()->SetScale(scales[i]);
        AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
        root->Rebuild();
        REQUIRE(RunUntilOutput(root, workQueue, 1000));
        RunFrames(root, workQueue, 10);
        const CsgBuildStats stats = root->GetLastBuildStats();
        CHECK(stats.valid_);
        CHECK(stats.degenerates_ == 0);

        const Model* bakedModel = GetOutput(root)->GetModel();
        const unsigned bakedIndices = CountIndices(bakedModel);
        const BoundingBox bakedBounds = bakedModel->GetBoundingBox();
        REQUIRE(bakedIndices > 36);
        REQUIRE(root->Bake());
        RunFrames(root, workQueue, 60);

        const Model* after = GetOutput(root)->GetModel();
        REQUIRE(after);
        CHECK(after == bakedModel);
        CHECK(CountIndices(after) == bakedIndices);
        CHECK(after->GetBoundingBox().min_.Equals(bakedBounds.min_, 1e-3f));
        CHECK(after->GetBoundingBox().max_.Equals(bakedBounds.max_, 1e-3f));
    }
}

TEST_CASE("CsgRoot reports the statistics of the last build", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Stats");

    // A root that built nothing has the default statistics.
    const CsgBuildStats& empty = root->GetLastBuildStats();
    CHECK(empty.booleanMs_ == 0.0f);
    CHECK(empty.triangulateMs_ == 0.0f);
    CHECK(empty.buildModelMs_ == 0.0f);
    CHECK(empty.faceCount_ == 0);
    CHECK(empty.triangleCount_ == 0);
    CHECK(!empty.closed_);
    CHECK(empty.volume_ == 0.0);
    CHECK(empty.openEdges_ == 0);
    CHECK(empty.duplicates_ == 0);
    CHECK(empty.degenerates_ == 0);
    CHECK(!empty.valid_);

    // Box A has the size 2 and box B has the size 2. The overlap is a cube with the size 1.
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));

    const CsgBuildStats stats = root->GetLastBuildStats();
    CHECK(stats.faceCount_ > 0);
    CHECK(stats.triangleCount_ > 0);
    CHECK(stats.triangleCount_ * 3 == CountIndices(GetOutput(root)->GetModel()));
    CHECK(stats.closed_);
    CHECK(stats.volume_ == Catch::Approx(7.0).margin(1e-3));
    CHECK(stats.booleanMs_ >= 0.0f);
    CHECK(stats.triangulateMs_ >= 0.0f);
    CHECK(stats.buildModelMs_ >= 0.0f);
}

TEST_CASE("CsgRoot applies the statistics of a queued result with the result", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "PendingStats");
    root->SetAutoApplyResults(false);
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    RunFrames(root, workQueue, 60);

    REQUIRE(root->HasPendingResult());
    CHECK(root->GetLastBuildStats().faceCount_ == 0);
    REQUIRE(root->ApplyPendingResult());
    CHECK(root->GetLastBuildStats().faceCount_ > 0);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));
}

TEST_CASE("CsgRoot keeps a valid result as the cache and skips the cache of an invalid result", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    // The open box has no triangle for its top face.
    AddBoxModel(context, "Csg/OpenBoxA.mdl", Vector3::ONE, 2);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // The valid root has a cached solid. The Bake function needs it.
    CsgRoot* valid = MakeBoxRoot(scene, "Valid");
    AddBoxBrush(valid, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    valid->Rebuild();
    REQUIRE(RunUntilOutput(valid, workQueue, 1000));
    const CsgBuildStats validStats = valid->GetLastBuildStats();
    CHECK(validStats.faceCount_ > 0);
    CHECK(validStats.closed_);
    CHECK(validStats.openEdges_ == 0);
    CHECK(validStats.duplicates_ == 0);
    CHECK(validStats.degenerates_ == 0);
    CHECK(validStats.valid_);
    CHECK(valid->Bake());

    // The open root shows its result, but has no cached solid. The build is not a failed build.
    CsgRoot* open = scene->CreateChild("Open")->CreateComponent<CsgRoot>();
    open->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/OpenBoxA.mdl"));
    open->SetDeferredDelaySec(0.0f);
    AddBoxBrush(open, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    open->Rebuild();
    REQUIRE(RunUntilOutput(open, workQueue, 1000));
    const CsgBuildStats stats = open->GetLastBuildStats();
    CHECK(stats.faceCount_ > 0);
    CHECK(!stats.valid_);
    CHECK(!stats.closed_);
    CHECK(stats.openEdges_ > 0);
    CHECK(!open->GetLastBuildFailed());
    CHECK(CountIndices(GetOutput(open)->GetModel()) == stats.triangleCount_ * 3);
    CHECK(!open->Bake());
}

TEST_CASE("CsgRoot reports a positive volume and a valid result for a union", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // The boxes A and B have the size 2. They touch no more: the union has the volume 16.
    CsgRoot* root = MakeBoxRoot(scene, "Union");
    AddBoxBrush(root, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 10);
    const CsgBuildStats stats = root->GetLastBuildStats();
    CHECK(stats.volume_ > 0.0);
    CHECK(stats.volume_ == Catch::Approx(16.0).margin(1e-3));
    CHECK(stats.closed_);
    CHECK(stats.valid_);
    CHECK(stats.degenerates_ == 0);
}

// The corner of the cut lies on the diagonal of a box face. The output has no flat triangle there.
TEST_CASE("CsgRoot cuts a corner of a box without a degenerate triangle", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Corner");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 10);
    const CsgBuildStats stats = root->GetLastBuildStats();
    CHECK(stats.volume_ == Catch::Approx(7.0).margin(1e-3));
    CHECK(stats.degenerates_ == 0);
    CHECK(stats.valid_);
}

TEST_CASE("CsgRoot refuses the build when the boolean throws", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Failing");
    CHECK(!root->GetLastBuildFailed());
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    CHECK(!root->GetLastBuildFailed());
    const Model* oneBrushModel = GetOutput(root)->GetModel();
    const unsigned oneBrushIndices = CountIndices(oneBrushModel);
    const CsgBuildStats oneBrushStats = root->GetLastBuildStats();

    // The build throws. The output, and the statistics, stay as they are.
    // Let the events of the new brush settle first, so that no later change makes the failed job stale.
    RunFrames(root, workQueue, 10);
    CsgRoot::SetTestBuildHook([] { throw std::runtime_error("test failure"); });
    root->SetEpsilon(2e-4f);
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildFailed());
    CHECK(GetOutput(root)->GetModel() == oneBrushModel);
    CHECK(CountIndices(GetOutput(root)->GetModel()) == oneBrushIndices);
    CHECK(root->GetLastBuildStats().faceCount_ == oneBrushStats.faceCount_);

    // The root is not in flight. The next build works and clears the flag.
    CsgRoot::SetTestBuildHook(nullptr);
    AddBoxBrush(root, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(!root->GetLastBuildFailed());
    const Model* extendedModel = GetOutput(root)->GetModel();
    REQUIRE(extendedModel);
    CHECK(CountIndices(extendedModel) > oneBrushIndices);
    CHECK(extendedModel->GetBoundingBox().max_.x_ == Catch::Approx(4.0f).margin(1e-3f));
}

TEST_CASE("CsgRoot refuses the build when the boolean throws a non-standard exception", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "FailingInt");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 10);
    const Model* builtModel = GetOutput(root)->GetModel();

    CsgRoot::SetTestBuildHook([] { throw 42; });
    root->SetEpsilon(2e-4f);
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildFailed());
    CHECK(GetOutput(root)->GetModel() == builtModel);

    CsgRoot::SetTestBuildHook(nullptr);
    root->SetEpsilon(3e-4f);
    RunFrames(root, workQueue, 60);
    CHECK(!root->GetLastBuildFailed());
    CHECK(GetOutput(root)->GetModel() != builtModel);
}

TEST_CASE("CsgRoot queues a failed build while auto-apply is off", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "PendingFailing");
    root->SetAutoApplyResults(false);
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    REQUIRE(root->ApplyPendingResult());
    CHECK(!root->HasPendingResult());

    CsgRoot::SetTestBuildHook([] { throw std::runtime_error("test failure"); });
    root->SetEpsilon(2e-4f);
    RunFrames(root, workQueue, 60);

    // The flag changes when the root takes the queued result.
    REQUIRE(root->HasPendingResult());
    CHECK(!root->GetLastBuildFailed());
    CHECK(!root->ApplyPendingResult());
    CHECK(root->GetLastBuildFailed());
    CHECK(!root->HasPendingResult());

    CsgRoot::SetTestBuildHook(nullptr);
    root->SetEpsilon(3e-4f);
    RunFrames(root, workQueue, 60);
    REQUIRE(root->HasPendingResult());
    CHECK(root->ApplyPendingResult());
    CHECK(!root->GetLastBuildFailed());
}

TEST_CASE("CsgRoot runs the test build hook once for each build", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Counting");
    std::atomic<int> calls{0};
    CsgRoot::SetTestBuildHook([&calls] { ++calls; });
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    CHECK(calls == 1);
    CHECK(!root->GetLastBuildFailed());

    // The job takes the hook, so a build without a new hook does not call it.
    RunFrames(root, workQueue, 10);
    root->SetEpsilon(2e-4f);
    RunFrames(root, workQueue, 60);
    CHECK(calls == 1);

    CsgRoot::SetTestBuildHook([&calls] { ++calls; });
    root->SetEpsilon(3e-4f);
    RunFrames(root, workQueue, 60);
    CHECK(calls == 2);
}

TEST_CASE("CsgRoot refuses to bake after a failed build", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "BakeAfterFailure");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    AddBoxBrush(root, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 10);
    const CsgBuildStats stats = root->GetLastBuildStats();
    CHECK(stats.valid_);
    CHECK(stats.degenerates_ == 0);

    // The epsilon changes, but the build fails. The cache keeps the old epsilon.
    CsgRoot::SetTestBuildHook([] { throw std::runtime_error("test failure"); });
    root->SetEpsilon(2e-4f);
    RunFrames(root, workQueue, 60);
    REQUIRE(root->GetLastBuildFailed());
    CHECK(!root->Bake());

    // A later successful build lets the root bake again.
    CsgRoot::SetTestBuildHook(nullptr);
    root->SetEpsilon(3e-4f);
    RunFrames(root, workQueue, 60);
    REQUIRE(!root->GetLastBuildFailed());
    CHECK(root->Bake());
}

TEST_CASE("CsgRoot clears the failed flag when no brushes remain", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "NoBrushes");
    CsgBrush* brush = AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 10);

    CsgRoot::SetTestBuildHook([] { throw std::runtime_error("test failure"); });
    root->SetEpsilon(2e-4f);
    RunFrames(root, workQueue, 60);
    REQUIRE(root->GetLastBuildFailed());

    // The rebuild without brushes shows the base model. This is a valid output.
    brush->GetNode()->Remove();
    RunFrames(root, workQueue, 60);
    CHECK(!root->GetLastBuildFailed());
    REQUIRE(GetOutput(root));
    CHECK(CountIndices(GetOutput(root)->GetModel()) == 36);
}

namespace
{

/// Add a flat wide brush that cuts the root box into a lower and an upper piece.
CsgBrush* AddSlabBrush(CsgRoot* root, const ea::string& name, float height)
{
    CsgBrush* brush = AddBoxBrush(root, name, Vector3(0.0f, height, 0.0f), CsgOperation::DifferenceAB);
    brush->GetNode()->SetScale(Vector3(3.0f, 0.1f, 3.0f));
    return brush;
}

} // namespace

TEST_CASE("CsgRoot reports the component of each geometry", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("The root keeps all shells by default")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Keep");
        CHECK(root->GetMinComponentVolumeFraction() == 0.0f);
        AddSlabBrush(root, "Slab", 0.0f);
        root->Rebuild();
        REQUIRE(RunUntilOutput(root, workQueue, 1000));

        const Model* model = GetOutput(root)->GetModel();
        REQUIRE(model);
        CHECK(model->GetNumGeometries() == 2);
        const ea::span<const unsigned> components = root->GetLastGeometryComponents();
        REQUIRE(components.size() == 2);
        CHECK(components[0] == 0);
        CHECK(components[1] == 1);
        CHECK(root->GetLastBuildStats().componentCount_ == 2);
        CHECK(root->GetLastBuildStats().droppedComponents_ == 0);
    }

    SECTION("The root drops a small shell and keeps the drop on the cached solid")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Drop");
        root->SetMinComponentVolumeFraction(2.0f);
        CHECK(root->GetMinComponentVolumeFraction() == 1.0f);
        AddSlabBrush(root, "Slab", 0.4f);
        root->Rebuild();
        REQUIRE(RunUntilOutput(root, workQueue, 1000));

        const Model* model = GetOutput(root)->GetModel();
        REQUIRE(model);
        CHECK(model->GetNumGeometries() == 1);
        ea::span<const unsigned> components = root->GetLastGeometryComponents();
        REQUIRE(components.size() == 1);
        CHECK(components[0] == 0);
        CHECK(root->GetLastBuildStats().droppedComponents_ == 1);
        CHECK(root->GetLastBuildStats().componentCount_ == 1);
        CHECK(root->GetLastBuildStats().valid_);
        const double keptVolume = root->GetLastBuildStats().volume_;
        // The lower piece is about 0.65 of the box volume (8).
        CHECK(keptVolume > 4.0);
        CHECK(keptVolume < 6.0);

        // The new brush cuts a corner of the kept piece only. The build reuses the cached solid.
        CsgBrush* notch = AddBoxBrush(root, "Notch", Vector3(1.0f, -1.0f, 1.0f), CsgOperation::DifferenceAB);
        notch->GetNode()->SetScale(Vector3(0.2f, 0.2f, 0.2f));
        root->Rebuild();
        RunFrames(root, workQueue, 60);

        model = GetOutput(root)->GetModel();
        REQUIRE(model);
        CHECK(model->GetNumGeometries() == 1);
        CHECK(root->GetLastBuildStats().valid_);
        CHECK(root->GetLastBuildStats().componentCount_ == 1);
        CHECK(root->GetLastBuildStats().volume_ < keptVolume);
        CHECK(root->GetLastBuildStats().volume_ > 4.0);
        CHECK(model->GetBoundingBox().max_.y_ < 0.31f);
    }

    SECTION("The component values are empty after the last brush goes")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Removed");
        CsgBrush* slab = AddSlabBrush(root, "Slab", 0.0f);
        root->Rebuild();
        REQUIRE(RunUntilOutput(root, workQueue, 1000));
        REQUIRE(root->GetLastGeometryComponents().size() == 2);

        // The output is then the base model. It is not the result of a build, so the span is empty.
        slab->GetNode()->Remove();
        RunFrames(root, workQueue, 60);
        REQUIRE(GetOutput(root));
        const Model* model = GetOutput(root)->GetModel();
        REQUIRE(model);
        CHECK(model->GetNumGeometries() >= 1);
        CHECK(root->GetLastGeometryComponents().empty());
    }

    SECTION("The component values repeat for the geometries of one piece")
    {
        // A material needs the file system to load its default resources.
        context->RegisterSubsystem<VirtualFileSystem>();
        for (const char* name : {"Materials/A.xml", "Materials/B.xml"})
        {
            auto material = MakeShared<Material>(context);
            material->SetName(name);
            context->GetSubsystem<ResourceCache>()->AddManualResource(material);
        }

        CsgRoot* root = MakeBoxRoot(scene, "Materials");
        root->SetMaterials(ResourceRefList(Material::GetTypeStatic(), {"Materials/A.xml"}));
        CsgBrush* slab = AddSlabBrush(root, "Slab", 0.0f);
        slab->SetMaterials(ResourceRefList(Material::GetTypeStatic(), {"Materials/B.xml"}));
        root->Rebuild();
        REQUIRE(RunUntilOutput(root, workQueue, 1000));

        const Model* model = GetOutput(root)->GetModel();
        REQUIRE(model);
        const ea::span<const unsigned> components = root->GetLastGeometryComponents();
        REQUIRE(components.size() == model->GetNumGeometries());
        // Each piece has the faces of the box and the cut face of the slab: two materials.
        CHECK(components.size() == 4);
        CHECK(root->GetLastBuildStats().componentCount_ == 2);
        ea::vector<unsigned> counts(2, 0u);
        for (const unsigned component : components)
        {
            REQUIRE(component < 2);
            ++counts[component];
        }
        CHECK(counts[0] == 2);
        CHECK(counts[1] == 2);
    }
}

TEST_CASE("CsgRoot ignores a brush below a direct child", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("A brush below a group node has no effect")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Grouped");
        Node* group = root->GetNode()->CreateChild("Group");
        Node* brushNode = group->CreateChild("Cut");
        brushNode->SetPosition(Vector3(1.0f, 1.0f, 1.0f));
        auto* brush = brushNode->CreateComponent<CsgBrush>();
        brush->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxB.mdl"));
        brush->SetOperation(CsgOperation::DifferenceAB);

        root->Rebuild();
        REQUIRE(RunUntilOutput(root, workQueue, 1000));
        RunFrames(root, workQueue, 60);
        CHECK(CountIndices(GetOutput(root)->GetModel()) == 36);
    }

    SECTION("A brush below a brush has no effect")
    {
        // The grandchild removes the whole child box. If the root used it, the cut would not occur.
        CsgRoot* root = MakeBoxRoot(scene, "Stacked");
        CsgBrush* cut = AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
        auto* inner = cut->GetNode()->CreateChild("Inner")->CreateComponent<CsgBrush>();
        inner->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxB.mdl"));
        inner->SetOperation(CsgOperation::DifferenceAB);

        root->Rebuild();
        REQUIRE(RunUntilOutput(root, workQueue, 1000));
        RunFrames(root, workQueue, 60);
        const unsigned cutIndices = CountIndices(GetOutput(root)->GetModel());
        CHECK(cutIndices > 36);

        // The same cut without the grandchild gives the same output.
        CsgRoot* plain = MakeBoxRoot(scene, "Plain");
        AddBoxBrush(plain, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
        plain->Rebuild();
        REQUIRE(RunUntilOutput(plain, workQueue, 1000));
        RunFrames(plain, workQueue, 60);
        CHECK(CountIndices(GetOutput(plain)->GetModel()) == cutIndices);
    }
}

TEST_CASE("CsgRoot applies the sibling brushes in child order", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // The union adds [0, 2] on x. The difference then removes [1, 3], so the result ends at x = 1.
    // The cut is larger on y and z, so it has no face on the sides of the union.
    CsgRoot* root = MakeBoxRoot(scene, "Ordered");
    CsgBrush* grow = AddBoxBrush(root, "Grow", Vector3(1.0f, 0.0f, 0.0f), CsgOperation::Union);
    CsgBrush* cut = AddBoxBrush(root, "Cut", Vector3(2.0f, 0.0f, 0.0f), CsgOperation::DifferenceAB);
    cut->GetNode()->SetScale(Vector3(1.0f, 2.0f, 2.0f));
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 60);
    CHECK(GetOutput(root)->GetModel()->GetBoundingBox().max_.x_ == Catch::Approx(1.0f).margin(1e-3f));

    // Move the union to the end. The difference comes first, so the union reaches x = 2.
    Node* growNode = grow->GetNode();
    growNode->SetParent(scene);
    growNode->SetParent(root->GetNode());
    REQUIRE(root->GetNode()->GetChildren().back() == growNode);
    RunFrames(root, workQueue, 60);
    CHECK(GetOutput(root)->GetModel()->GetBoundingBox().max_.x_ == Catch::Approx(2.0f).margin(1e-3f));
}

TEST_CASE("CsgRoot rebuilds when a brush leaves the root", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Source");
    CsgRoot* target = MakeBoxRoot(scene, "Target");
    CsgBrush* cut = AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    root->Rebuild();
    target->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 60);
    REQUIRE(RunUntilOutput(target, workQueue, 1000));
    RunFrames(target, workQueue, 60);
    CHECK(CountIndices(GetOutput(root)->GetModel()) > 36);
    CHECK(CountIndices(GetOutput(target)->GetModel()) == 36);

    SECTION("A move to a plain node rebuilds the old root")
    {
        cut->GetNode()->SetParent(scene);
        RunFrames(root, workQueue, 60);
        CHECK(CountIndices(GetOutput(root)->GetModel()) == 36);
    }

    SECTION("A move to another root rebuilds both roots")
    {
        cut->GetNode()->SetParent(target->GetNode());
        RunFrames(root, workQueue, 60);
        CHECK(CountIndices(GetOutput(root)->GetModel()) == 36);
        CHECK(CountIndices(GetOutput(target)->GetModel()) > 36);

        // The old root no longer reacts to the brush.
        const unsigned revision = root->GetRebuildRevision();
        cut->SetOperation(CsgOperation::Union);
        cut->GetNode()->SetPosition(Vector3(0.5f, 0.5f, 0.5f));
        RunFrames(root, workQueue, 60);
        CHECK(root->GetRebuildRevision() == revision);
    }
}

namespace
{

/// Add three brushes that change the base box in different places.
ea::vector<CsgBrush*> AddFoldBrushes(CsgRoot* root)
{
    ea::vector<CsgBrush*> brushes;
    brushes.push_back(AddBoxBrush(root, "First", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB));
    brushes.push_back(AddBoxBrush(root, "Second", Vector3(-1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB));
    brushes.push_back(AddBoxBrush(root, "Third", Vector3(1.5f, 0.0f, 0.0f), CsgOperation::Union));
    for (CsgBrush* brush : brushes)
        brush->GetNode()->SetScale(Vector3(0.5f, 0.5f, 0.5f));
    return brushes;
}

/// Build the root and wait until all requested builds are done.
void BuildFully(CsgRoot* root, WorkQueue* workQueue)
{
    root->Rebuild();
    REQUIRE(RunUntilOutput(root, workQueue, 1000));
    RunFrames(root, workQueue, 60);
}

/// Check that two roots have the same output.
void CheckSameOutput(CsgRoot* actual, CsgRoot* expected)
{
    const Model* a = GetOutput(actual)->GetModel();
    const Model* b = GetOutput(expected)->GetModel();
    REQUIRE(a);
    REQUIRE(b);
    CHECK(CountIndices(a) == CountIndices(b));
    CHECK(a->GetNumGeometries() == b->GetNumGeometries());
    CHECK(a->GetBoundingBox().min_.Equals(b->GetBoundingBox().min_, 1e-4f));
    CHECK(a->GetBoundingBox().max_.Equals(b->GetBoundingBox().max_, 1e-4f));
    CHECK(actual->GetLastBuildStats().volume_ == Catch::Approx(expected->GetLastBuildStats().volume_).epsilon(1e-9));
    CHECK(actual->GetLastBuildStats().faceCount_ == expected->GetLastBuildStats().faceCount_);
    CHECK(actual->GetLastBuildStats().valid_);
}

} // namespace

TEST_CASE("CsgRoot evaluates only the fold steps from the first changed operand", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Folded");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    // An explicit rebuild with no change evaluates all steps again.
    BuildFully(root, workQueue);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);

    SECTION("A change of the last operand evaluates one step")
    {
        brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 1);

        CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
        ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
        expected[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    }

    SECTION("A change of the first operand evaluates all steps")
    {
        brushes[0]->SetOperation(CsgOperation::Union);
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 3);

        CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
        ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
        expected[0]->SetOperation(CsgOperation::Union);
        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    }

    SECTION("A change of the middle operand evaluates the steps from that operand")
    {
        brushes[1]->GetNode()->SetPosition(Vector3(-1.0f, -1.0f, 1.0f));
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 2);

        CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
        ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
        expected[1]->GetNode()->SetPosition(Vector3(-1.0f, -1.0f, 1.0f));
        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    }

    SECTION("A reorder evaluates the steps from the first moved operand")
    {
        // The order changes from First, Second, Third to First, Third, Second.
        Node* secondNode = brushes[1]->GetNode();
        secondNode->SetParent(scene);
        secondNode->SetParent(root->GetNode());
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 2);

        CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
        ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
        Node* expectedSecond = expected[1]->GetNode();
        expectedSecond->SetParent(scene);
        expectedSecond->SetParent(fresh->GetNode());
        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    }

    SECTION("A disabled operand evaluates the steps from its index")
    {
        brushes[1]->SetEnabled(false);
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    }

    SECTION("A change of the shell filter evaluates all steps")
    {
        root->SetMinComponentVolumeFraction(0.01f);
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 3);
    }
}

TEST_CASE("CsgRoot uses the baked base for a brush that comes before the next rebuild", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Baked");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);

    // A long delay keeps the rebuild that Bake requests in the queue.
    root->SetDeferredDelaySec(100.0f);
    REQUIRE(root->Bake());
    AddBoxBrush(root, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    AddBoxBrush(fresh, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    AddBoxBrush(fresh, "Extension", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

TEST_CASE("CsgRoot reuses the fold states when the last operand goes", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Shrunk");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    brushes[2]->GetNode()->Remove();
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 0);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
    expected[2]->GetNode()->Remove();
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);

    // The next change of the last operand evaluates one step on the reused states.
    brushes[1]->GetNode()->SetPosition(Vector3(-1.0f, -1.0f, 1.0f));
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
}

TEST_CASE("CsgRoot discards the fold states when a model reloads", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    // Model::Load looks for a metadata file. The file system has no mount points, so the search finds nothing.
    context->RegisterSubsystem<VirtualFileSystem>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    SharedPtr<Model> reloaded = AddBoxModel(context, "Csg/BoxC.mdl", Vector3::ONE);
    SharedPtr<Model> small = AddBoxModel(context, "Csg/Small.mdl", Vector3(0.5f, 0.5f, 0.5f));

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Reloaded");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[0]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxC.mdl"));
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    // A reload loads the new data into the same model object, then sends the event.
    VectorBuffer data;
    REQUIRE(small->Save(data));
    data.Seek(0);
    REQUIRE(reloaded->Load(data));

    // The expected output uses the smaller box for the first brush.
    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
    expected[0]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Small.mdl"));

    SECTION("The event alone requests a rebuild of all steps")
    {
        const unsigned revision = root->GetRebuildRevision();
        reloaded->SendEvent(E_RELOADFINISHED);
        CHECK(root->GetRebuildRevision() != revision);
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 3);

        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    }

    SECTION("A move of the last operand after the reload evaluates all steps")
    {
        // The long delay keeps the rebuild of the event in the queue until the move.
        root->SetDeferredDelaySec(100.0f);
        reloaded->SendEvent(E_RELOADFINISHED);
        brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
        root->Rebuild();
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildStats().foldSteps_ == 3);

        expected[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    }

    SECTION("The root does not listen to a model that it no longer uses")
    {
        brushes[0]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxB.mdl"));
        RunFrames(root, workQueue, 60);
        const unsigned revision = root->GetRebuildRevision();
        reloaded->SendEvent(E_RELOADFINISHED);
        CHECK(root->GetRebuildRevision() == revision);
    }
}

TEST_CASE("CsgRoot discards the fold states while it is inactive", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    // Model::Load looks for a metadata file. The file system has no mount points, so the search finds nothing.
    context->RegisterSubsystem<VirtualFileSystem>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    SharedPtr<Model> reloaded = AddBoxModel(context, "Csg/BoxC.mdl", Vector3::ONE);
    SharedPtr<Model> small = AddBoxModel(context, "Csg/Small.mdl", Vector3(0.5f, 0.5f, 0.5f));

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Inactive");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[0]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxC.mdl"));
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    // The model reloads while the root is disabled, so the root does not receive the event.
    root->SetEnabled(false);
    VectorBuffer data;
    REQUIRE(small->Save(data));
    data.Seek(0);
    REQUIRE(reloaded->Load(data));
    reloaded->SendEvent(E_RELOADFINISHED);
    root->SetEnabled(true);

    brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
    expected[0]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Small.mdl"));
    expected[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

TEST_CASE("CsgRoot keeps the fold states after a failed build", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Failed");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    CsgRoot::SetTestBuildHook([] { throw std::runtime_error("test failure"); });
    brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    RunFrames(root, workQueue, 60);
    REQUIRE(root->GetLastBuildFailed());

    // The cache of the last good build stays, so the next build evaluates only the last step.
    CsgRoot::SetTestBuildHook(nullptr);
    brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.5f, 0.0f));
    RunFrames(root, workQueue, 60);
    REQUIRE(!root->GetLastBuildFailed());
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
    expected[2]->GetNode()->SetPosition(Vector3(1.5f, 0.5f, 0.0f));
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

namespace
{

/// Load the data of the source model into the target model, then send the reload event.
void ReloadModel(Model* target, Model* source)
{
    VectorBuffer data;
    REQUIRE(source->Save(data));
    data.Seek(0);
    REQUIRE(target->Load(data));
    target->SendEvent(E_RELOADFINISHED);
}

} // namespace

TEST_CASE("CsgRoot sees a reload after a build that ends while the root is inactive", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    // Model::Load looks for a metadata file. The file system has no mount points, so the search finds nothing.
    context->RegisterSubsystem<VirtualFileSystem>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    SharedPtr<Model> reloaded = AddBoxModel(context, "Csg/BoxC.mdl", Vector3::ONE);
    SharedPtr<Model> small = AddBoxModel(context, "Csg/Small.mdl", Vector3(0.5f, 0.5f, 0.5f));

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "InFlight");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[0]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxC.mdl"));
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    // The work queue of the test has no worker threads, so the job waits until the next frame.
    brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.1f, 0.0f));
    root->Rebuild();
    root->SetEnabled(false);
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);

    ReloadModel(reloaded, small);
    root->SetEnabled(true);
    brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
    expected[0]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Small.mdl"));
    expected[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

TEST_CASE("CsgRoot sees a reload of a cached model during a build that becomes stale", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    // Model::Load looks for a metadata file. The file system has no mount points, so the search finds nothing.
    context->RegisterSubsystem<VirtualFileSystem>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    SharedPtr<Model> reloaded = AddBoxModel(context, "Csg/BoxC.mdl", Vector3::ONE);
    SharedPtr<Model> small = AddBoxModel(context, "Csg/Small.mdl", Vector3(0.5f, 0.5f, 0.5f));

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Stale");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[1]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxC.mdl"));
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    // The removal starts a build without the second brush. Only the cache uses the model of that brush now.
    Node* secondNode = brushes[1]->GetNode();
    const auto& children = root->GetNode()->GetChildren();
    const unsigned secondIndex =
        static_cast<unsigned>(ea::find(children.begin(), children.end(), SharedPtr<Node>(secondNode)) - children.begin());
    secondNode->SetParent(scene);
    ReloadModel(reloaded, small);
    // The second brush comes back at its old index. This makes the build in flight stale.
    root->GetNode()->AddChild(secondNode, secondIndex);
    REQUIRE(root->GetNode()->GetChildren()[secondIndex] == secondNode);
    brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    // The reload discards all states, so the build evaluates all steps.
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
    expected[1]->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Small.mdl"));
    expected[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

namespace
{

/// This object counts the error messages that the engine logs.
class ErrorCounter : public Object
{
    URHO3D_OBJECT(ErrorCounter, Object);

public:
    explicit ErrorCounter(Context* context)
        : Object(context)
    {
        SubscribeToEvent(E_LOGMESSAGE,
            [this](StringHash, VariantMap& eventData)
            {
                if (eventData[LogMessage::P_LEVEL].GetInt() >= LOG_ERROR)
                    ++count_;
            });
    }

    unsigned GetCount() const { return count_; }

private:
    unsigned count_{};
};

} // namespace

namespace
{

/// Create a root node with no base model and no delay for the rebuild.
CsgRoot* MakeEmptyBaseRoot(Scene* scene, const ea::string& nodeName)
{
    auto* root = scene->CreateChild(nodeName)->CreateComponent<CsgRoot>();
    root->SetDeferredDelaySec(0.0f);
    return root;
}

/// The output is empty if it has no model or no geometry.
bool IsOutputEmpty(CsgRoot* root)
{
    const StaticModel* output = GetOutput(root);
    return !output || !output->GetModel() || output->GetModel()->GetNumGeometries() == 0;
}

} // namespace

TEST_CASE("CsgRoot with an empty base model and one Union brush gives the brush shape", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
    AddBoxBrush(root, "Add", Vector3::ZERO, CsgOperation::Union);
    BuildFully(root, workQueue);
    CHECK(!root->GetLastBuildFailed());
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);

    // A root with the same box as base and no brush has the same shape.
    CsgRoot* expected = MakeBoxRoot(scene, "BoxOnly");
    BuildFully(expected, workQueue);
    CHECK(GetOutput(root)->GetModel()->GetBoundingBox().min_.Equals(Vector3(-1.0f, -1.0f, -1.0f), 1e-4f));
    CHECK(GetOutput(root)->GetModel()->GetBoundingBox().max_.Equals(Vector3(1.0f, 1.0f, 1.0f), 1e-4f));
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(8.0).margin(1e-3));
    CHECK(root->GetLastBuildStats().faceCount_ == 12);
    CHECK(CountIndices(GetOutput(root)->GetModel()) == 36);
    CHECK(GetOutput(root)->GetModel()->GetBoundingBox().min_.Equals(
        GetOutput(expected)->GetModel()->GetBoundingBox().min_, 1e-4f));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot with an empty base model folds Union then Subtract like a base made by the Union", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
    AddBoxBrush(root, "Add", Vector3::ZERO, CsgOperation::Union);
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    CHECK(!root->GetLastBuildFailed());

    CsgRoot* expected = MakeBoxRoot(scene, "BaseA");
    AddBoxBrush(expected, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(expected, workQueue);
    CheckSameOutput(root, expected);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot with an empty base model and only a Subtract brush gives an empty output", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("Subtract")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
        AddBoxBrush(root, "Cut", Vector3::ZERO, CsgOperation::DifferenceAB);
        root->Rebuild();
        RunFrames(root, workQueue, 60);
        CHECK(IsOutputEmpty(root));
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().faceCount_ == 0);
    }

    SECTION("Intersection")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
        AddBoxBrush(root, "Keep", Vector3::ZERO, CsgOperation::Intersection);
        root->Rebuild();
        RunFrames(root, workQueue, 60);
        CHECK(IsOutputEmpty(root));
        CHECK(!root->GetLastBuildFailed());
    }
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot with an empty base model and no brush gives an empty output", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(IsOutputEmpty(root));
    CHECK(!root->GetLastBuildFailed());
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot with an empty base model reuses the fold states", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
    AddBoxBrush(root, "Add", Vector3::ZERO, CsgOperation::Union);
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    CsgBrush* last = AddBoxBrush(root, "Add2", Vector3(3.0f, 0.0f, 0.0f), CsgOperation::Union);
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);

    last->GetNode()->SetPosition(Vector3(3.0f, 0.5f, 0.0f));
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot with an empty base model bakes and keeps building", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
    AddBoxBrush(root, "Add", Vector3::ZERO, CsgOperation::Union);
    BuildFully(root, workQueue);
    REQUIRE(root->Bake());
    RunFrames(root, workQueue, 60);
    CHECK(!root->GetLastBuildFailed());
    CHECK(CountIndices(GetOutput(root)->GetModel()) == 36);

    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    RunFrames(root, workQueue, 60);
    CHECK(!root->GetLastBuildFailed());
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot leaves out a removed brush at once with no delay", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Removed");
    root->SetDeferredDelaySec(0.0f);
    AddBoxBrush(root, "First", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    CsgBrush* second = AddBoxBrush(root, "Second", Vector3(-1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().foldSteps_ == 2);

    second->GetNode()->Remove();
    // The result of the build arrives in the first frame. The test runs only two frames.
    RunFrames(root, workQueue, 2);
    CHECK(root->GetLastBuildStats().foldSteps_ <= 1);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));

    CsgRoot* expected = MakeBoxRoot(scene, "OneBrush");
    AddBoxBrush(expected, "First", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(expected, workQueue);
    CheckSameOutput(root, expected);
}

TEST_CASE("CsgRoot with an empty base model empties a warm output when the last brush goes", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeEmptyBaseRoot(scene, "EmptyBase");
    CsgBrush* brush = AddBoxBrush(root, "Add", Vector3::ZERO, CsgOperation::Union);
    BuildFully(root, workQueue);
    REQUIRE(!IsOutputEmpty(root));

    brush->GetNode()->Remove();
    RunFrames(root, workQueue, 60);
    CHECK(IsOutputEmpty(root));
    CHECK(GetOutput(root)->GetMaterialsAttr().names_.empty());
    CHECK(!root->GetLastBuildFailed());
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot runs all steps when the base model toggles between empty and set", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    const ResourceRef boxA(Model::GetTypeStatic(), "Csg/BoxA.mdl");
    const auto addBrushes = [](CsgRoot* target)
    {
        AddBoxBrush(target, "Add", Vector3(0.5f, 0.0f, 0.0f), CsgOperation::Union);
        AddBoxBrush(target, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    };

    CsgRoot* root = MakeBoxRoot(scene, "Toggled");
    addBrushes(root);
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    const auto checkState = [&](const char* name, bool withBase)
    {
        CsgRoot* fresh = withBase ? MakeBoxRoot(scene, name) : MakeEmptyBaseRoot(scene, name);
        addBrushes(fresh);
        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    };

    root->SetModel(ResourceRef(Model::GetTypeStatic(), ""));
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 2);
    checkState("FreshEmpty1", false);

    root->SetModel(boxA);
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 2);
    checkState("FreshBase", true);

    root->SetModel(ResourceRef(Model::GetTypeStatic(), ""));
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().foldSteps_ == 2);
    checkState("FreshEmpty2", false);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot puts the result on a temporary child node", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    root->GetNode()->SetPosition(Vector3(5.0f, 0.0f, 0.0f));
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);

    // A root with the default delay makes nothing before its first build.
    auto* fresh = scene->CreateChild("Fresh")->CreateComponent<CsgRoot>();
    CHECK(fresh->GetResultNode() == nullptr);
    CHECK(fresh->GetResultModel() == nullptr);

    // The result node does not request a rebuild. The revision stays the same through the first build.
    const unsigned revision = root->GetRebuildRevision();
    BuildFully(root, workQueue);
    CHECK(root->GetRebuildRevision() == revision);

    Node* rootNode = root->GetNode();
    CHECK(rootNode->GetComponent<StaticModel>() == nullptr);
    Node* resultNode = root->GetResultNode();
    REQUIRE(resultNode);
    CHECK(resultNode->GetName() == "CsgResult");
    CHECK(resultNode->GetParent() == rootNode);
    CHECK(resultNode->IsTemporary());
    CHECK(resultNode->GetComponent<CsgBrush>() == nullptr);
    CHECK(resultNode->GetTransformMatrix() == Matrix3x4::IDENTITY);
    CHECK(resultNode->GetWorldPosition().Equals(rootNode->GetWorldPosition()));
    REQUIRE(root->GetResultModel());
    CHECK(root->GetResultModel()->GetNode() == resultNode);
    CHECK(CountResultNodes(rootNode) == 1);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);

    // A second explicit build reuses the same node.
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(root->GetResultNode() == resultNode);
    CHECK(CountResultNodes(rootNode) == 1);
}

TEST_CASE("CsgRoot result node follows the enabled state of the root", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    Node* resultNode = root->GetResultNode();
    REQUIRE(resultNode);
    CHECK(resultNode->IsEnabled());
    CHECK(root->GetResultModel()->IsEnabledEffective());

    root->SetEnabled(false);
    CHECK(!resultNode->IsEnabled());
    CHECK(!root->GetResultModel()->IsEnabledEffective());
    root->SetEnabled(true);
    CHECK(resultNode->IsEnabled());
    CHECK(root->GetResultModel()->IsEnabledEffective());

    // A disabled root node disables the root, so the result goes off too.
    root->GetNode()->SetEnabled(false);
    CHECK(!resultNode->IsEnabled());
    root->GetNode()->SetEnabled(true);
    CHECK(resultNode->IsEnabled());

    // A deep enable of the root node enables the child node. The model of a disabled root stays off.
    root->SetEnabled(false);
    root->GetNode()->SetDeepEnabled(true);
    CHECK(!root->GetResultModel()->IsEnabledEffective());
    root->SetEnabled(true);
    CHECK(root->GetResultModel()->IsEnabledEffective());
}

TEST_CASE("CsgRoot removes the result node with the root component", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    Node* rootNode = root->GetNode();
    WeakPtr<Node> resultNode{root->GetResultNode()};
    REQUIRE(resultNode);

    rootNode->RemoveComponent(root);
    CHECK(!resultNode);
    CHECK(CountResultNodes(rootNode) == 0);
    CHECK(rootNode->GetChild("Cut") != nullptr);
    RunSceneFrames(scene, workQueue, 10);
}

TEST_CASE("CsgRoot survives the destruction of its node and of the scene", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // The node goes away after a build.
    {
        CsgRoot* root = MakeBoxRoot(scene, "Built");
        AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
        BuildFully(root, workQueue);
        WeakPtr<CsgRoot> weakRoot{root};
        WeakPtr<Node> resultNode{root->GetResultNode()};
        REQUIRE(resultNode);
        root->GetNode()->Remove();
        CHECK(!weakRoot);
        CHECK(!resultNode);
        RunSceneFrames(scene, workQueue, 10);
    }

    // The node goes away while a build is in flight.
    {
        CsgRoot* root = MakeBoxRoot(scene, "InFlight");
        AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
        root->Rebuild();
        root->GetNode()->Remove();
        RunSceneFrames(scene, workQueue, 10);
    }

    // The scene goes away with a built root and a build in flight.
    {
        CsgRoot* root = MakeBoxRoot(scene, "SceneOwned");
        AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
        BuildFully(root, workQueue);
        WeakPtr<Node> resultNode{root->GetResultNode()};
        root->GetNode()->SetPosition(Vector3(0.5f, 0.0f, 0.0f));
        AddBoxBrush(root, "Second", Vector3(-1.0f, 0.0f, 0.0f), CsgOperation::Union);
        scene->Clear();
        CHECK(!resultNode);
        RunSceneFrames(scene, workQueue, 10);
        scene.Reset();
        workQueue->CompleteAll();
    }
}

TEST_CASE("CsgRoot does not save the result node and rebuilds it after a load", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    REQUIRE(root->GetResultNode());
    const unsigned indices = CountIndices(GetOutput(root)->GetModel());

    VectorBuffer buffer;
    REQUIRE(scene->Save(buffer));
    const ea::string_view data(reinterpret_cast<const char*>(buffer.GetData()), buffer.GetSize());
    CHECK(data.find("Cut") != ea::string_view::npos);
    CHECK(data.find("CsgResult") == ea::string_view::npos);

    auto loaded = MakeShared<Scene>(context);
    buffer.Seek(0);
    REQUIRE(loaded->Load(buffer));
    Node* loadedRootNode = loaded->GetChild("Root");
    REQUIRE(loadedRootNode);
    auto* loadedRoot = loadedRootNode->GetComponent<CsgRoot>();
    REQUIRE(loadedRoot);
    // The load can start a build at once. The root never makes a second result node.
    CHECK(CountResultNodes(loadedRootNode) <= 1);
    REQUIRE(RunUntilOutput(loadedRoot, workQueue, 1000));
    RunFrames(loadedRoot, workQueue, 60);
    CHECK(CountResultNodes(loadedRootNode) == 1);
    CHECK(loadedRootNode->GetComponent<StaticModel>() == nullptr);
    CHECK(CountIndices(GetOutput(loadedRoot)->GetModel()) == indices);
}

TEST_CASE("CsgRoot clone has exactly one result node", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    REQUIRE(root->GetResultNode());
    const unsigned indices = CountIndices(GetOutput(root)->GetModel());

    Node* cloneNode = root->GetNode()->Clone();
    REQUIRE(cloneNode);
    auto* cloneRoot = cloneNode->GetComponent<CsgRoot>();
    REQUIRE(cloneRoot);
    // The clone does not copy the temporary result node. The clone can start a build at once.
    CHECK(cloneRoot->GetResultNode() != root->GetResultNode());
    CHECK(CountResultNodes(cloneNode) <= 1);
    REQUIRE(RunUntilOutput(cloneRoot, workQueue, 1000));
    RunFrames(cloneRoot, workQueue, 60);
    CHECK(CountResultNodes(cloneNode) == 1);
    CHECK(cloneRoot->GetResultNode() != root->GetResultNode());
    CHECK(cloneRoot->GetResultNode()->GetParent() == cloneNode);
    CHECK(CountResultNodes(root->GetNode()) == 1);
    CHECK(CountIndices(GetOutput(cloneRoot)->GetModel()) == indices);

    // A node with the result name that is already there is reused. The root removes a duplicate.
    Node* staleNode = scene->CreateChild("Stale");
    Node* first = staleNode->CreateChild("CsgResult");
    staleNode->CreateChild("CsgResult");
    auto* stale = staleNode->CreateComponent<CsgRoot>();
    stale->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxA.mdl"));
    stale->SetDeferredDelaySec(0.0f);
    AddBoxBrush(stale, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(stale, workQueue);
    CHECK(CountResultNodes(stale->GetNode()) == 1);
    CHECK(stale->GetResultNode() == first);
    CHECK(first->IsTemporary());
}

TEST_CASE("CsgRoot shows the build status", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/OpenBoxA.mdl", Vector3::ONE, 0);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    auto* fresh = scene->CreateChild("Fresh")->CreateComponent<CsgRoot>();
    CHECK(fresh->GetBuildStatus() == "Not built");

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    RunFrames(root, workQueue, 10);
    root->SetEpsilon(2e-4f);
    CHECK(root->GetBuildStatus() == "Building...");
    BuildFully(root, workQueue);
    const ea::string expected = Format("{} triangles", root->GetLastBuildStats().triangleCount_);
    const ea::string status = root->GetBuildStatus();
    CHECK(status.starts_with("OK: "));
    CHECK(status.ends_with(" ms, " + expected));
    // The attribute gives the same text. The scene does not save it.
    CHECK(root->GetAttribute("Build Status").GetString() == status);
    const AttributeInfo* info = context->GetReflection<CsgRoot>()->GetAttribute("Build Status");
    REQUIRE(info);
    CHECK(!info->ShouldSave());
    CHECK(!(info->mode_ & AM_FILE));
    CHECK(!(info->mode_ & AM_PREFAB));
    root->SetAttribute("Build Status", Variant(ea::string("Other")));
    CHECK(root->GetBuildStatus() == status);

    {
        TestBuildHookGuard guard;
        CsgRoot::SetTestBuildHook([] { throw std::runtime_error("test failure"); });
        root->SetEpsilon(3e-4f);
        RunFrames(root, workQueue, 60);
        CHECK(root->GetLastBuildFailed());
        CHECK(root->GetBuildStatus() == "Failed");
    }

    CsgRoot* open = scene->CreateChild("Open")->CreateComponent<CsgRoot>();
    open->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/OpenBoxA.mdl"));
    open->SetDeferredDelaySec(0.0f);
    AddBoxBrush(open, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(open, workQueue);
    CHECK(!open->GetLastBuildStats().valid_);
    CHECK(open->GetBuildStatus() == "Invalid");
}

TEST_CASE("CsgRoot has the Add Brush action and CsgBrush does not", "[root]")
{
    auto context = MakeCsgTestContext();
    const AttributeInfo* rootAction = context->GetReflection<CsgRoot>()->GetAttribute("Add Brush");
    REQUIRE(rootAction);
    CHECK(rootAction->GetMetadata(AttributeMetadata::IsAction).GetBool());
    CHECK(context->GetReflection<CsgBrush>()->GetAttribute("Add Brush") == nullptr);

    auto scene = MakeShared<Scene>(context);
    auto* root = scene->CreateChild("Root")->CreateComponent<CsgRoot>();
    // The editor runs an action when it sets the attribute.
    root->SetAttribute("Add Brush", true);
    Node* brushNode = root->GetNode()->GetChild("Brush");
    REQUIRE(brushNode);
    CHECK(brushNode->GetComponent<CsgBrush>() != nullptr);
}

TEST_CASE("CsgRoot leaves a user node with the result name alone", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // The user node has a light and a child. The root must not change it or take it.
    Node* rootNode = scene->CreateChild("Root");
    Node* userNode = rootNode->CreateChild("CsgResult");
    userNode->SetPosition(Vector3(2.0f, 0.0f, 0.0f));
    userNode->CreateComponent<Light>();
    userNode->CreateChild("Child");
    // A node with a light and a model is not a candidate too.
    Node* userModelNode = rootNode->CreateChild("CsgResult");
    userModelNode->CreateComponent<StaticModel>();
    userModelNode->CreateComponent<Light>();
    auto* root = rootNode->CreateComponent<CsgRoot>();
    root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxA.mdl"));
    root->SetDeferredDelaySec(0.0f);
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);

    Node* resultNode = root->GetResultNode();
    REQUIRE(resultNode);
    CHECK(resultNode != userNode);
    CHECK(resultNode != userModelNode);
    CHECK(resultNode->IsTemporary());
    CHECK(CountResultNodes(rootNode) == 3);
    CHECK(userNode->GetParent() == rootNode);
    CHECK(!userNode->IsTemporary());
    CHECK(userNode->GetPosition().Equals(Vector3(2.0f, 0.0f, 0.0f)));
    CHECK(userNode->GetChild("Child") != nullptr);
    CHECK(userNode->GetComponent<Light>() != nullptr);
    CHECK(userNode->GetComponent<StaticModel>() == nullptr);
    CHECK(userModelNode->GetParent() == rootNode);
    CHECK(!userModelNode->IsTemporary());

    // Removal of the root leaves the user nodes.
    rootNode->RemoveComponent(root);
    CHECK(CountResultNodes(rootNode) == 2);
    CHECK(userNode->GetParent() == rootNode);
    CHECK(userModelNode->GetParent() == rootNode);
}

TEST_CASE("CsgRoot reuses a result node that has only a StaticModel", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // This is the shape of a result node in a scene file that a tool saved with temporary nodes.
    Node* rootNode = scene->CreateChild("Root");
    Node* saved = rootNode->CreateChild("CsgResult");
    saved->SetPosition(Vector3(3.0f, 0.0f, 0.0f));
    auto* savedModel = saved->CreateComponent<StaticModel>();
    Node* duplicate = rootNode->CreateChild("CsgResult");
    duplicate->CreateComponent<StaticModel>();
    WeakPtr<Node> weakDuplicate{duplicate};
    auto* root = rootNode->CreateComponent<CsgRoot>();
    root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxA.mdl"));
    root->SetDeferredDelaySec(0.0f);
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);

    CHECK(root->GetResultNode() == saved);
    CHECK(root->GetResultModel() == savedModel);
    CHECK(saved->IsTemporary());
    CHECK(saved->GetTransformMatrix() == Matrix3x4::IDENTITY);
    CHECK(!weakDuplicate);
    CHECK(CountResultNodes(rootNode) == 1);
    REQUIRE(savedModel->GetModel());
    CHECK(savedModel->GetModel()->GetNumGeometries() > 0);
}

namespace
{

/// Collect all CsgRoot components and all CsgBrush components in the scene.
void CollectCsgComponents(Node* node, ea::vector<CsgRoot*>& roots, ea::vector<CsgBrush*>& brushes)
{
    if (auto* root = node->GetComponent<CsgRoot>())
        roots.push_back(root);
    if (auto* brush = node->GetComponent<CsgBrush>())
        brushes.push_back(brush);
    for (Node* child : node->GetChildren())
        CollectCsgComponents(child, roots, brushes);
}

} // namespace

TEST_CASE("CsgShowcase scene builds all roots without errors", "[root][scene]")
{
    const ea::string scenePath = ea::string(CSG_SAMPLE_DATA_DIR) + "/Scenes/CsgShowcase.scene";
    if (!std::ifstream(scenePath.c_str()).good())
    {
        WARN("Test skipped. The scene file is missing: " << scenePath.c_str());
        return;
    }

    auto context = MakeCsgTestContext();
    // A material needs the file system to load its default resources.
    context->RegisterSubsystem<VirtualFileSystem>();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    auto* time = context->GetSubsystem<Time>();
    auto* cache = context->GetSubsystem<ResourceCache>();

    AddBoxModel(context, "Models/Box.mdl", Vector3::ONE);
    AddBoxModel(context, "Models/Sphere.mdl", Vector3(0.8f, 0.8f, 0.8f));
    for (const char* name : {"Materials/Stone.xml", "Materials/CheckboardProperties.xml"})
    {
        auto material = MakeShared<Material>(context);
        material->SetName(name);
        cache->AddManualResource(material);
    }

    // The load of the non-CSG scene content (skybox, zone, render pipeline) logs errors in this
    // context. The test context has no such components and resources. Count errors after the load.
    ErrorCounter loadErrors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();
    {
        File file(context, scenePath.c_str(), FILE_READ);
        REQUIRE(file.IsOpen());
        REQUIRE(scene->LoadXML(file));
    }

    WARN("CsgShowcase load error count (non-CSG content): " << loadErrors.GetCount());
    ErrorCounter errors(context);

    ea::vector<CsgRoot*> roots;
    ea::vector<CsgBrush*> brushes;
    CollectCsgComponents(scene, roots, brushes);
    INFO("Root count: " << roots.size());
    WARN("CsgShowcase root count: " << roots.size());
    REQUIRE(!roots.empty());

    // Run frames until every root has finished its build.
    bool allDone = false;
    for (unsigned i = 0; i < 2000 && !allDone; ++i)
    {
        workQueue->CompleteAll();
        time->BeginFrame(1.0f / 60.0f);
        scene->Update(1.0f / 60.0f);
        time->EndFrame();

        allDone = true;
        for (CsgRoot* root : roots)
        {
            const ea::string status = root->GetBuildStatus();
            if (status == "Building..." || status == "Not built")
                allDone = false;
        }
    }
    CHECK(allDone);

    for (CsgRoot* root : roots)
    {
        unsigned operands = 0;
        for (Node* child : root->GetNode()->GetChildren())
            operands += child->GetComponent<CsgBrush>() ? 1 : 0;
        INFO("Root node: " << root->GetNode()->GetName().c_str());
        if (!root->GetModelAttr().name_.empty() || operands > 0)
            CHECK(root->GetResultNode() != nullptr);
        CHECK(!root->GetLastBuildFailed());
    }

    // A brush must be a direct child of a node with a CsgRoot.
    for (CsgBrush* brush : brushes)
    {
        Node* parent = brush->GetNode()->GetParent();
        INFO("Brush node: " << brush->GetNode()->GetName().c_str());
        REQUIRE(parent);
        CHECK(parent->GetComponent<CsgRoot>() != nullptr);
    }
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot rebuilds on the CsgRequestRebuild event after a reorder", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // The union adds [0, 2] on x. The difference then removes [1, 3], so the result ends at x = 1.
    CsgRoot* root = MakeBoxRoot(scene, "Ordered");
    AddBoxBrush(root, "Grow", Vector3(1.0f, 0.0f, 0.0f), CsgOperation::Union);
    CsgBrush* cut = AddBoxBrush(root, "Cut", Vector3(2.0f, 0.0f, 0.0f), CsgOperation::DifferenceAB);
    cut->GetNode()->SetScale(Vector3(1.0f, 2.0f, 2.0f));
    BuildFully(root, workQueue);
    CHECK(GetOutput(root)->GetModel()->GetBoundingBox().max_.x_ == Catch::Approx(1.0f).margin(1e-3f));

    SECTION("The event requests a rebuild")
    {
        const unsigned revision = root->GetRebuildRevision();
        root->SendEvent(StringHash("CsgRequestRebuild"));
        CHECK(root->GetRebuildRevision() != revision);
    }

    SECTION("The output follows the new order")
    {
        // ReorderChild sends no event. The root does not know about the change.
        root->GetNode()->ReorderChild(cut->GetNode(), 0);
        const unsigned revision = root->GetRebuildRevision();
        RunFrames(root, workQueue, 60);
        CHECK(root->GetRebuildRevision() == revision);

        root->SendEvent(StringHash("CsgRequestRebuild"));
        RunFrames(root, workQueue, 60);
        CHECK(GetOutput(root)->GetModel()->GetBoundingBox().max_.x_ == Catch::Approx(2.0f).margin(1e-3f));
    }
}

TEST_CASE("GetOperationColor gives one distinct color for each operation", "[root][debug]")
{
    const Color unionColor = GetOperationColor(static_cast<int>(CsgOperation::Union));
    const Color differenceColor = GetOperationColor(static_cast<int>(CsgOperation::DifferenceAB));
    const Color intersectionColor = GetOperationColor(static_cast<int>(CsgOperation::Intersection));

    // Union is green, DifferenceAB is red and Intersection is blue.
    CHECK(unionColor.g_ > unionColor.r_);
    CHECK(unionColor.g_ > unionColor.b_);
    CHECK(differenceColor.r_ > differenceColor.g_);
    CHECK(differenceColor.r_ > differenceColor.b_);
    CHECK(intersectionColor.b_ > intersectionColor.r_);
    CHECK(intersectionColor.b_ > intersectionColor.g_);
    CHECK(unionColor.a_ == 1.0f);

    // An index that is not an operation gives the fallback color.
    CHECK(GetOperationColor(-1) == CsgUnknownOperationColor);
    CHECK(GetOperationColor(3) == CsgUnknownOperationColor);
}

TEST_CASE("CsgBrush draws its debug shape with and without the depth test", "[root][debug]")
{
    auto context = MakeCsgTestContext();
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();
    auto* debug = scene->CreateComponent<DebugRenderer>();
    CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
    CsgBrush* brush = AddBoxBrush(root, "Brush", Vector3::ZERO, CsgOperation::Intersection);

    CHECK(!debug->HasContent());
    // The editor calls the Component virtual. It has no link to the CSG library.
    static_cast<Component*>(brush)->DrawDebugGeometry(debug, false);
    CHECK(debug->HasContent());
}

TEST_CASE("CsgBrush looks up a missing debug model once for each model name", "[root][debug]")
{
    auto context = MakeCsgTestContext();
    // The lookup of a missing file needs the file system.
    context->RegisterSubsystem<VirtualFileSystem>();
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();
    auto* debug = scene->CreateComponent<DebugRenderer>();
    CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
    auto* brush = root->GetNode()->CreateChild("Brush")->CreateComponent<CsgBrush>();
    brush->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Missing.mdl"));

    // The model change logs one warning. The first draw logs one error for the lookup.
    ErrorCounter errors(context);
    brush->DrawDebugGeometry(debug, true);
    const unsigned firstCount = errors.GetCount();
    for (unsigned i = 0; i < 10; ++i)
        brush->DrawDebugGeometry(debug, true);
    CHECK(errors.GetCount() == firstCount);
    CHECK(!debug->HasContent());

    // A model that appears later is found after the next model change.
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);
    brush->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxB.mdl"));
    brush->DrawDebugGeometry(debug, true);
    CHECK(debug->HasContent());
}

namespace
{

/// Load a model file of the engine CoreData into the resource cache. Return null if the file is missing.
SharedPtr<Model> LoadCoreDataModel(Context* context, const ea::string& name)
{
    const ea::string path = ea::string(CSG_CORE_DATA_DIR) + "/" + name;
    if (!std::ifstream(path.c_str()).good())
        return nullptr;
    File file(context, path, FILE_READ);
    if (!file.IsOpen())
        return nullptr;
    auto model = MakeShared<Model>(context);
    if (!model->Load(file))
        return nullptr;
    model->SetName(name);
    context->GetSubsystem<ResourceCache>()->AddManualResource(model);
    return model;
}

/// Return true if the preset test must find the CoreData models.
/// The CMake option RBFX_CSG_REQUIRE_CORE_DATA or the environment variable CSG_REQUIRE_CORE_DATA sets it.
bool IsCoreDataRequired()
{
#ifdef CSG_REQUIRE_CORE_DATA
    return true;
#else
    const char* value = std::getenv("CSG_REQUIRE_CORE_DATA");
    return value && *value && ea::string(value) != "0";
#endif
}

} // namespace

// The editor addon offers these models as brush presets. Each one must be a closed mesh that CSG accepts.
TEST_CASE("CSG accepts each CoreData brush preset model as a closed mesh", "[root][preset]")
{
    const char* const presets[] = {"Models/Box.mdl", "Models/Sphere.mdl", "Models/Cylinder.mdl", "Models/Cone.mdl",
        "Models/Pyramid.mdl", "Models/Torus.mdl"};

    for (const char* preset : presets)
    {
        DYNAMIC_SECTION(preset)
        {
            auto context = MakeCsgTestContext();
            context->RegisterSubsystem<VirtualFileSystem>();
            auto* workQueue = context->GetSubsystem<WorkQueue>();
            if (!LoadCoreDataModel(context, preset))
            {
                // CI sets the requirement. Then a missing model is a failure and not a silent skip.
                INFO("The model is missing in " << CSG_CORE_DATA_DIR << ": " << preset);
                REQUIRE(!IsCoreDataRequired());
                WARN("Test skipped. The model is missing in " << CSG_CORE_DATA_DIR << ": " << preset);
                continue;
            }

            ErrorCounter errors(context);
            auto scene = MakeShared<Scene>(context);
            scene->CreateComponent<Octree>();
            CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
            Node* brushNode = root->GetNode()->CreateChild("Brush");
            auto* brush = brushNode->CreateComponent<CsgBrush>();
            brush->SetModel(ResourceRef(Model::GetTypeStatic(), preset));
            brush->SetOperation(CsgOperation::Union);
            BuildFully(root, workQueue);

            const CsgBuildStats stats = root->GetLastBuildStats();
            CHECK(!root->GetLastBuildFailed());
            CHECK(stats.volume_ > 0.0);
            CHECK(stats.closed_);
            CHECK(stats.valid_);
            CHECK(errors.GetCount() == 0);
        }
    }
}

TEST_CASE("Node::GetComponent finds the exact type only", "[root][generated]")
{
    auto context = MakeCsgTestContext();
    auto scene = MakeShared<Scene>(context);
    Node* node = scene->CreateChild("Node");
    auto* brush = node->CreateComponent<TestGeneratedBrush>();

    // The lookup by type does not find a derived type. The derived lookup finds it.
    CHECK(node->GetComponent<CsgBrush>() == nullptr);
    CHECK(node->GetComponent(CsgBrush::GetTypeStatic()) == nullptr);
    CHECK(node->GetDerivedComponent<CsgBrush>() == brush);
    CHECK(node->GetComponent<TestGeneratedBrush>() == brush);
}

namespace
{

/// Add a brush with a generated model under the node of the root.
TestGeneratedBrush* AddGeneratedBrush(
    CsgRoot* root, const ea::string& name, const Vector3& position, CsgOperation op, Model* model)
{
    Node* brushNode = root->GetNode()->CreateChild(name);
    brushNode->SetPosition(position);
    auto* brush = brushNode->CreateComponent<TestGeneratedBrush>();
    brush->SetGeneratedModel(model);
    brush->SetOperation(op);
    return brush;
}

} // namespace

TEST_CASE("CsgRoot uses the generated model of a brush as an operand", "[root][generated]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("A generated box alone gives the box volume")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Generated");
        AddGeneratedBrush(root, "Box", Vector3::ZERO, CsgOperation::Union, MakeTestBoxModel(context, Vector3::ONE));
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().foldSteps_ == 1);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(8.0).margin(1e-3));
    }

    SECTION("A generated box gives the same result as a box model resource")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Generated");
        AddGeneratedBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB,
            MakeTestBoxModel(context, Vector3::ONE));
        BuildFully(root, workQueue);

        CsgRoot* expected = MakeBoxRoot(scene, "Resource");
        AddBoxBrush(expected, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
        BuildFully(expected, workQueue);

        CheckSameOutput(root, expected);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));
    }

    SECTION("A brush draws its generated model")
    {
        auto* debug = scene->CreateComponent<DebugRenderer>();
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Generated");
        TestGeneratedBrush* brush =
            AddGeneratedBrush(root, "Box", Vector3::ZERO, CsgOperation::Union, MakeTestBoxModel(context, Vector3::ONE));
        CHECK(!debug->HasContent());
        static_cast<Component*>(brush)->DrawDebugGeometry(debug, false);
        CHECK(debug->HasContent());
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot skips a brush with a null generated model", "[root][generated]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Generated");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    TestGeneratedBrush* invalid =
        AddGeneratedBrush(root, "Invalid", Vector3(-1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB, nullptr);
    BuildFully(root, workQueue);

    // The root builds the other brush. The invalid brush is no operand.
    CHECK(!root->GetLastBuildFailed());
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));
    CHECK(errors.GetCount() == 0);

    // A valid model makes the brush an operand. The root rebuilds with no explicit request.
    invalid->SetGeneratedModel(MakeTestBoxModel(context, Vector3::ONE));
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(6.0).margin(1e-3));

    // A null model again removes the operand.
    invalid->SetGeneratedModel(nullptr);
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRoot rebuilds from a changed generated model and reuses the fold steps before it", "[root][generated]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    // The third brush of the fold has a generated model. The result is the same as with the resource.
    CsgRoot* root = MakeBoxRoot(scene, "Generated");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[2]->GetNode()->Remove();
    TestGeneratedBrush* third = AddGeneratedBrush(
        root, "Third", Vector3(1.5f, 0.0f, 0.0f), CsgOperation::Union, MakeTestBoxModel(context, Vector3::ONE));
    third->GetNode()->SetScale(Vector3(0.5f, 0.5f, 0.5f));
    BuildFully(root, workQueue);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);

    CsgRoot* expected = MakeBoxRoot(scene, "Resource");
    AddFoldBrushes(expected);
    BuildFully(expected, workQueue);
    CheckSameOutput(root, expected);

    // A new model object marks the brush dirty. Only the last fold step runs again.
    const double oldVolume = root->GetLastBuildStats().volume_;
    third->SetGeneratedModel(MakeTestBoxModel(context, Vector3(2.0f, 1.0f, 1.0f)));
    RunFrames(root, workQueue, 60);
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ > oldVolume + 0.1);
    CHECK(errors.GetCount() == 0);
}

namespace
{

/// This object counts the warning messages that contain a text.
class WarningCounter : public Object
{
    URHO3D_OBJECT(WarningCounter, Object);

public:
    WarningCounter(Context* context, const ea::string& text)
        : Object(context)
        , text_(text)
    {
        SubscribeToEvent(E_LOGMESSAGE,
            [this](StringHash, VariantMap& eventData)
            {
                if (eventData[LogMessage::P_LEVEL].GetInt() == LOG_WARNING
                    && eventData[LogMessage::P_MESSAGE].GetString().contains(text_))
                    ++count_;
            });
    }

    unsigned GetCount() const { return count_; }

private:
    ea::string text_;
    unsigned count_{};
};

/// Add a procedural box brush under the node of the root.
CsgBoxBrush* AddProceduralBox(
    CsgRoot* root, const ea::string& name, const Vector3& position, const Vector3& size, CsgOperation op)
{
    Node* brushNode = root->GetNode()->CreateChild(name);
    brushNode->SetPosition(position);
    auto* brush = brushNode->CreateComponent<CsgBoxBrush>();
    brush->SetSize(size);
    brush->SetOperation(op);
    return brush;
}

/// Add the model of a generated box to the resource cache.
void AddGeneratedBoxModel(Context* context, const ea::string& name, const CsgBoxParams& params)
{
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateBox(params, mesh));
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    model->SetName(name);
    context->GetSubsystem<ResourceCache>()->AddManualResource(model);
}

} // namespace

TEST_CASE("CsgBoxBrush is an operand with the box volume", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    CsgBoxParams cutParams;
    cutParams.size_ = Vector3(2.0f, 2.0f, 2.0f);
    AddGeneratedBoxModel(context, "Csg/GeneratedBox.mdl", cutParams);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("A box alone gives the product of the size")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Alone");
        CsgBoxBrush* brush =
            AddProceduralBox(root, "Box", Vector3::ZERO, Vector3(2.0f, 3.0f, 4.0f), CsgOperation::Union);
        CHECK(brush->IsShapeValid());
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().closed_);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(24.0).margin(1e-3));
    }

    SECTION("A subtract box cuts the base and equals a model brush with the same mesh")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Procedural");
        AddProceduralBox(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), Vector3(2.0f, 2.0f, 2.0f), CsgOperation::DifferenceAB);
        BuildFully(root, workQueue);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));

        CsgRoot* expected = MakeBoxRoot(scene, "Model");
        Node* cutNode = expected->GetNode()->CreateChild("Cut");
        cutNode->SetPosition(Vector3(1.0f, 1.0f, 1.0f));
        auto* cut = cutNode->CreateComponent<CsgBrush>();
        cut->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/GeneratedBox.mdl"));
        cut->SetOperation(CsgOperation::DifferenceAB);
        BuildFully(expected, workQueue);
        CheckSameOutput(root, expected);
    }

    SECTION("The brush draws its generated model")
    {
        auto* debug = scene->CreateComponent<DebugRenderer>();
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Debug");
        CsgBoxBrush* brush = AddProceduralBox(root, "Box", Vector3::ZERO, Vector3::ONE, CsgOperation::Union);
        CHECK(!debug->HasContent());
        static_cast<Component*>(brush)->DrawDebugGeometry(debug, false);
        CHECK(debug->HasContent());
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgBoxBrush and a CoreData Box.mdl brush share one output geometry", "[root][procedural][preset]")
{
    auto context = MakeCsgTestContext();
    context->RegisterSubsystem<VirtualFileSystem>();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    if (!LoadCoreDataModel(context, "Models/Box.mdl"))
    {
        INFO("The model is missing in " << CSG_CORE_DATA_DIR << ": Models/Box.mdl");
        REQUIRE(!IsCoreDataRequired());
        WARN("Test skipped. The model is missing in " << CSG_CORE_DATA_DIR << ": Models/Box.mdl");
        return;
    }

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();
    CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
    Node* modelNode = root->GetNode()->CreateChild("Model");
    auto* modelBrush = modelNode->CreateComponent<CsgBrush>();
    modelBrush->SetModel(ResourceRef(Model::GetTypeStatic(), "Models/Box.mdl"));
    AddProceduralBox(root, "Box", Vector3(0.5f, 0.0f, 0.0f), Vector3::ONE, CsgOperation::Union);
    BuildFully(root, workQueue);

    // Both operands have the same layout and no material, so the output has one geometry.
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(1.5).margin(1e-3));
    CHECK(GetOutput(root)->GetModel()->GetNumGeometries() == 1);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgBoxBrush that is not valid gives no operand and one warning", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    WarningCounter warnings(context, "CsgBoxBrush");
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    CsgBoxBrush* box =
        AddProceduralBox(root, "Box", Vector3(-1.0f, 1.0f, 1.0f), Vector3(2.0f, 2.0f, 2.0f), CsgOperation::DifferenceAB);
    // Corner 7 goes through the opposite face.
    box->SetCorner(7, Vector3(-1.0f, 0.5f, 0.5f));
    BuildFully(root, workQueue);

    SharedPtr<Model> model;
    CHECK(box->GetGeneratedModel(model));
    CHECK(!model);
    CHECK(!box->IsShapeValid());
    CHECK(!root->GetLastBuildFailed());
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));

    // More frames and a debug draw give no more warnings.
    auto* debug = scene->CreateComponent<DebugRenderer>();
    static_cast<Component*>(box)->DrawDebugGeometry(debug, false);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(warnings.GetCount() == 1);

    // A valid corner makes the brush an operand again. The root rebuilds with no explicit request.
    box->SetCorner(7, CsgBoxParams::GetDefaultCorner(7));
    RunFrames(root, workQueue, 60);
    CHECK(box->IsShapeValid());
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(6.0).margin(1e-3));

    // The next change to a shape that is not valid gives one more warning.
    box->SetCorner(0, Vector3(1.0f, -0.5f, -0.5f));
    RunFrames(root, workQueue, 60);
    CHECK(warnings.GetCount() == 2);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(7.0).margin(1e-3));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgBoxBrush rebuilds after an attribute change and reuses the fold steps before it", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[2]->GetNode()->Remove();
    CsgBoxBrush* third =
        AddProceduralBox(root, "Third", Vector3(1.5f, 0.0f, 0.0f), Vector3(2.0f, 2.0f, 2.0f), CsgOperation::Union);
    third->GetNode()->SetScale(Vector3(0.5f, 0.5f, 0.5f));
    BuildFully(root, workQueue);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);
    const double oldVolume = root->GetLastBuildStats().volume_;

    // The attribute path: SetAttribute and ApplyAttributes. The value is clamped.
    Component* component = third;
    CHECK(component->SetAttribute("Size", Vector3(4.0f, 2.0f, 2.0f)));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetSize() == Vector3(4.0f, 2.0f, 2.0f));
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ > oldVolume + 0.1);

    CHECK(component->SetAttribute("Corner 7", Vector3(100.0f, 0.5f, 0.5f)));
    component->ApplyAttributes();
    CHECK(third->GetCorner(7) == Vector3(64.0f, 0.5f, 0.5f));
    CHECK(component->GetAttribute("Corner 7").GetVector3() == Vector3(64.0f, 0.5f, 0.5f));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgBoxBrush keeps its parameters and the result after a save and a load", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    CsgBoxBrush* box =
        AddProceduralBox(root, "Box", Vector3(1.0f, 1.0f, 1.0f), Vector3(2.0f, 1.5f, 2.0f), CsgOperation::DifferenceAB);
    box->SetCorner(7, Vector3(0.75f, 0.5f, 0.5f));
    BuildFully(root, workQueue);
    const double volume = root->GetLastBuildStats().volume_;
    CHECK(volume < 8.0);

    VectorBuffer buffer;
    REQUIRE(scene->Save(buffer));
    auto loaded = MakeShared<Scene>(context);
    buffer.Seek(0);
    REQUIRE(loaded->Load(buffer));
    Node* loadedRootNode = loaded->GetChild("Root");
    REQUIRE(loadedRootNode);
    auto* loadedRoot = loadedRootNode->GetComponent<CsgRoot>();
    REQUIRE(loadedRoot);
    Node* loadedBoxNode = loadedRootNode->GetChild("Box");
    REQUIRE(loadedBoxNode);
    auto* loadedBox = loadedBoxNode->GetComponent<CsgBoxBrush>();
    REQUIRE(loadedBox);
    CHECK(loadedBox->GetOperation() == CsgOperation::DifferenceAB);
    CHECK(loadedBox->GetSize() == box->GetSize());
    for (unsigned i = 0; i < 8; ++i)
        CHECK(loadedBox->GetCorner(i) == box->GetCorner(i));

    REQUIRE(RunUntilOutput(loadedRoot, workQueue, 1000));
    RunFrames(loadedRoot, workQueue, 60);
    CHECK(loadedRoot->GetLastBuildStats().volume_ == Catch::Approx(volume).epsilon(1e-9));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgBoxBrush has one material slot for each face and no Model attribute", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    // A material needs the file system to load its default resources.
    context->RegisterSubsystem<VirtualFileSystem>();
    for (const char* name : {"Materials/Box.xml", "Materials/Top.xml"})
    {
        auto material = MakeShared<Material>(context);
        material->SetName(name);
        context->GetSubsystem<ResourceCache>()->AddManualResource(material);
    }

    // The material logs an error for its missing default technique, so the counter starts after it.
    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("The attributes are the ones of a brush without Model")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgBoxBrush* box = AddProceduralBox(root, "Box", Vector3::ZERO, Vector3::ONE, CsgOperation::DifferenceAB);
        Component* component = box;
        CHECK(component->GetAttribute("Materials").GetResourceRefList().names_.size() == 6);
        CHECK(component->GetAttribute("Is Enabled").GetBool());
        CHECK(component->GetAttribute("Operation").GetInt() == static_cast<int>(CsgOperation::DifferenceAB));
        for (const AttributeInfo& info : *component->GetAttributes())
            CHECK(info.name_ != "Model");

        // A list of another length gets 6 slots. The first names stay.
        component->SetAttribute("Materials", ResourceRefList(Material::GetTypeStatic(), {"Materials/Box.xml"}));
        component->ApplyAttributes();
        REQUIRE(box->GetMaterialsAttr().names_.size() == 6);
        CHECK(box->GetMaterialsAttr().names_[0] == "Materials/Box.xml");
        CHECK(box->GetMaterialsAttr().names_[5].empty());
        box->SetMaterials(ResourceRefList(Material::GetTypeStatic(), ea::vector<ea::string>(9u, ea::string("Materials/Top.xml"))));
        CHECK(box->GetMaterialsAttr().names_.size() == 6);
    }

    SECTION("A default box gives one output geometry")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        AddProceduralBox(root, "Box", Vector3::ZERO, Vector3::ONE, CsgOperation::Union);
        BuildFully(root, workQueue);
        CHECK(GetOutput(root)->GetModel()->GetNumGeometries() == 1);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(1.0).margin(1e-3));
    }

    SECTION("Six equal materials give one output geometry")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgBoxBrush* box = AddProceduralBox(root, "Box", Vector3::ZERO, Vector3::ONE, CsgOperation::Union);
        box->SetMaterials(ResourceRefList(Material::GetTypeStatic(), ea::vector<ea::string>(6u, ea::string("Materials/Box.xml"))));
        BuildFully(root, workQueue);
        CHECK(GetOutput(root)->GetModel()->GetNumGeometries() == 1);
        CHECK(GetOutput(root)->GetMaterialsAttr().names_ == ea::vector<ea::string>{"Materials/Box.xml"});
    }

    SECTION("A material on +Y reaches the top face only")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgBoxBrush* box = AddProceduralBox(root, "Box", Vector3::ZERO, Vector3::ONE, CsgOperation::Union);
        ResourceRefList materials(Material::GetTypeStatic(), {"Materials/Box.xml", "", "", "Materials/Top.xml"});
        box->SetMaterials(materials);
        BuildFully(root, workQueue);

        const Model* output = GetOutput(root)->GetModel();
        REQUIRE(output->GetNumGeometries() == 2);
        const ea::vector<ea::string>& names = GetOutput(root)->GetMaterialsAttr().names_;
        REQUIRE(names.size() == 2);
        const unsigned topGeometry = names[0] == "Materials/Top.xml" ? 0 : 1;
        CHECK(names[1 - topGeometry] == "Materials/Box.xml");

        // Each triangle of the top geometry is at y = 0.5. No other triangle is there.
        unsigned topTriangles = 0;
        for (const CsgPolygon& polygon : CsgBuildPolygonsFromModel(output, Matrix3x4::IDENTITY))
        {
            bool onTop = true;
            for (const ModelVertex& vertex : polygon.vertices_)
                onTop = onTop && Abs(vertex.GetPosition().y_ - 0.5f) < 1e-4f;
            const bool inTopGeometry = (polygon.shared_ & 0xFFFFu) == topGeometry;
            CHECK(onTop == inTopGeometry);
            topTriangles += inTopGeometry ? 1 : 0;
        }
        CHECK(topTriangles >= 2);
    }

    SECTION("A subtract box gives its material to the cut faces")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Root");
        CsgBoxBrush* box = AddProceduralBox(
            root, "Box", Vector3(1.0f, 1.0f, 1.0f), Vector3(2.0f, 2.0f, 2.0f), CsgOperation::DifferenceAB);
        box->SetMaterials(ResourceRefList(Material::GetTypeStatic(), {"Materials/Box.xml"}));
        BuildFully(root, workQueue);
        const ea::vector<ea::string>& names = GetOutput(root)->GetMaterialsAttr().names_;
        CHECK(ea::find(names.begin(), names.end(), "Materials/Box.xml") != names.end());
        CHECK(names.size() == 2);
    }

    SECTION("A save and a load keep the six material slots")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgBoxBrush* box = AddProceduralBox(root, "Box", Vector3::ZERO, Vector3::ONE, CsgOperation::Union);
        const ResourceRefList materials(
            Material::GetTypeStatic(), {"Materials/Box.xml", "", "", "Materials/Top.xml", "", ""});
        box->SetMaterials(materials);

        VectorBuffer buffer;
        REQUIRE(scene->Save(buffer));
        auto loaded = MakeShared<Scene>(context);
        buffer.Seek(0);
        REQUIRE(loaded->Load(buffer));
        Node* loadedBoxNode = loaded->GetChild("Root")->GetChild("Box");
        REQUIRE(loadedBoxNode);
        auto* loadedBox = loadedBoxNode->GetComponent<CsgBoxBrush>();
        REQUIRE(loadedBox);
        CHECK(loadedBox->GetMaterialsAttr().names_ == materials.names_);
    }

    CHECK(errors.GetCount() == 0);
}

namespace
{

/// Run one frame with a single work queue update. CompleteAll waits for the delayed tasks, so the test avoids it.
void StepFrame(CsgRoot* root, WorkQueue* workQueue)
{
    auto* time = root->GetSubsystem<Time>();
    workQueue->Update();
    time->BeginFrame(1.0f / 60.0f);
    root->GetScene()->Update(1.0f / 60.0f);
    time->EndFrame();
}

/// Count the new output models of the root. Each applied result gives a new model.
class OutputCounter
{
public:
    explicit OutputCounter(CsgRoot* root)
        : root_(root)
    {
    }

    void Update()
    {
        const StaticModel* output = GetOutput(root_);
        Model* model = output ? output->GetModel() : nullptr;
        if (model && model != last_)
        {
            last_ = model;
            keep_.push_back(SharedPtr<Model>(model));
            ++count_;
        }
    }

    unsigned GetCount() const { return count_; }

private:
    CsgRoot* root_;
    Model* last_{};
    ea::vector<SharedPtr<Model>> keep_;
    unsigned count_{};
};

} // namespace

TEST_CASE("CsgRoot waits from the first change and does not reset the wait", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Drag");
    CsgBrush* brush = AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    root->SetDeferredDelaySec(0.1f);
    RunFrames(root, workQueue, 5);

    OutputCounter counter(root);
    counter.Update();
    const unsigned before = counter.GetCount();

    SECTION("A stream of changes gives live results and the last result has the final state")
    {
        Vector3 position;
        for (unsigned i = 0; i < 50; ++i)
        {
            position = Vector3(1.0f + 0.01f * static_cast<float>(i + 1), 1.0f, 1.0f);
            brush->GetNode()->SetPosition(position);
            // One frame for each change. A stale result still shows while the newer rebuild is pending.
            StepFrame(root, workQueue);
            counter.Update();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        CHECK(counter.GetCount() - before >= 5);

        for (unsigned i = 0; i < 30; ++i)
        {
            StepFrame(root, workQueue);
            counter.Update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
        AddBoxBrush(fresh, "Cut", position, CsgOperation::DifferenceAB);
        BuildFully(fresh, workQueue);
        CheckSameOutput(root, fresh);
    }

    SECTION("A single change gives exactly one rebuild after the wait")
    {
        brush->GetNode()->SetPosition(Vector3(1.2f, 1.0f, 1.0f));
        StepFrame(root, workQueue);
        counter.Update();
        CHECK(counter.GetCount() == before);

        for (unsigned i = 0; i < 40; ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            StepFrame(root, workQueue);
            counter.Update();
        }
        CHECK(counter.GetCount() - before == 1);
    }
}

TEST_CASE("CsgRoot shows the newest state after a stale result", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Stale");
    CsgBrush* brush = AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);

    OutputCounter counter(root);
    counter.Update();
    const unsigned before = counter.GetCount();

    // The first build is in flight when the second change comes. Its result is stale.
    brush->GetNode()->SetPosition(Vector3(1.1f, 1.0f, 1.0f));
    root->Rebuild();
    const Vector3 finalPosition(1.3f, 1.0f, 1.0f);
    brush->GetNode()->SetPosition(finalPosition);

    for (unsigned i = 0; i < 60; ++i)
    {
        StepFrame(root, workQueue);
        counter.Update();
    }
    // The stale result may show. The newer build replaces it, so the final state is what matters.
    CHECK(counter.GetCount() - before >= 1);
    CHECK(counter.GetCount() - before <= 2);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    AddBoxBrush(fresh, "Cut", finalPosition, CsgOperation::DifferenceAB);
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

namespace
{

/// Run frames until the counter sees one more output model or the frames end.
void StepUntilNewOutput(CsgRoot* root, WorkQueue* workQueue, OutputCounter& counter, unsigned frames)
{
    const unsigned start = counter.GetCount();
    for (unsigned i = 0; i < frames && counter.GetCount() == start; ++i)
    {
        StepFrame(root, workQueue);
        counter.Update();
    }
}

} // namespace

TEST_CASE("CsgRoot drops a stale result after a model reloads in place during a build", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    // Model::Load looks for a metadata file. The file system has no mount points, so the search finds nothing.
    context->RegisterSubsystem<VirtualFileSystem>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    SharedPtr<Model> small = AddBoxModel(context, "Csg/Small.mdl", Vector3(0.5f, 0.5f, 0.5f));

    // A model with two geometries. The second geometry is a smaller box.
    SharedPtr<Model> single = MakeTestBoxModel(context, Vector3::ONE);
    ModelView view(context);
    REQUIRE(view.ImportModel(single));
    ea::vector<GeometryView> geometries = view.GetGeometries();
    geometries.push_back(geometries[0]);
    view.SetGeometries(geometries);
    SharedPtr<Model> twoGeometries = view.ExportModel();
    twoGeometries->SetName("Csg/Two.mdl");
    context->GetSubsystem<ResourceCache>()->AddManualResource(twoGeometries);
    REQUIRE(twoGeometries->GetNumGeometries() == 2);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Reload");
    Node* brushNode = root->GetNode()->CreateChild("Cut");
    brushNode->SetPosition(Vector3(1.0f, 1.0f, 1.0f));
    auto* brush = brushNode->CreateComponent<CsgBrush>();
    brush->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Two.mdl"));
    brush->SetOperation(CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);

    // The build is in flight. The model loses its second geometry.
    root->Rebuild();
    ReloadModel(twoGeometries, small);
    REQUIRE(twoGeometries->GetNumGeometries() == 1);
    RunFrames(root, workQueue, 60);
    CHECK(!root->GetLastBuildFailed());

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    Node* freshNode = fresh->GetNode()->CreateChild("Cut");
    freshNode->SetPosition(Vector3(1.0f, 1.0f, 1.0f));
    auto* freshBrush = freshNode->CreateComponent<CsgBrush>();
    freshBrush->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Small.mdl"));
    freshBrush->SetOperation(CsgOperation::DifferenceAB);
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

TEST_CASE("CsgRoot keeps the cache and the dirty flags for a stale result", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Cache");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    BuildFully(root, workQueue);
    REQUIRE(root->GetLastBuildStats().valid_);

    OutputCounter counter(root);
    counter.Update();

    // A long wait keeps the newer rebuild pending. The job starts with the first position.
    root->SetDeferredDelaySec(100.0f);
    brushes[2]->GetNode()->SetPosition(Vector3(1.5f, 0.25f, 0.0f));
    root->Rebuild();
    const Vector3 finalPosition(1.5f, 0.5f, 0.0f);
    brushes[2]->GetNode()->SetPosition(finalPosition);
    // An attribute change marks the brush dirty. A transform change does not.
    brushes[2]->SetOperation(CsgOperation::DifferenceAB);
    StepUntilNewOutput(root, workQueue, counter, 60);
    REQUIRE(counter.GetCount() == 2);

    // The stale result shows. It leaves the dirty flag and the cache of the first build.
    CHECK(brushes[2]->IsDirty());
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    root->Rebuild();
    RunFrames(root, workQueue, 60);
    CHECK(!brushes[2]->IsDirty());
    // The cache of the first build still has the first two operands.
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);

    CsgRoot* fresh = MakeBoxRoot(scene, "Fresh");
    ea::vector<CsgBrush*> expected = AddFoldBrushes(fresh);
    expected[2]->GetNode()->SetPosition(finalPosition);
    expected[2]->SetOperation(CsgOperation::DifferenceAB);
    BuildFully(fresh, workQueue);
    CheckSameOutput(root, fresh);
}

TEST_CASE("CsgRoot drops a stale result in queued mode and after a failed job", "[root]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    TestBuildHookGuard guard;

    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Drop");
    CsgBrush* brush = AddBoxBrush(root, "Cut", Vector3(1.0f, 1.0f, 1.0f), CsgOperation::DifferenceAB);
    BuildFully(root, workQueue);
    OutputCounter counter(root);
    counter.Update();

    SECTION("Queued mode")
    {
        root->SetAutoApplyResults(false);
        root->Rebuild();
        brush->GetNode()->SetPosition(Vector3(1.2f, 1.0f, 1.0f));
        RunFrames(root, workQueue, 60);
        // Only the newest build gives a queued result.
        REQUIRE(root->HasPendingResult());
        CHECK(root->GetPendingResultRevision() == root->GetRebuildRevision());
        CHECK(root->ApplyPendingResult());
        CHECK(!root->HasPendingResult());
    }

    SECTION("Failed job")
    {
        CsgRoot::SetTestBuildHook([] { throw std::runtime_error("test failure"); });
        root->Rebuild();
        brush->GetNode()->SetPosition(Vector3(1.2f, 1.0f, 1.0f));
        for (unsigned i = 0; i < 60; ++i)
        {
            StepFrame(root, workQueue);
            counter.Update();
        }
        // The failed stale result does not show. The newer build works.
        CHECK(counter.GetCount() == 2);
        CHECK(!root->GetLastBuildFailed());
    }
}

namespace
{

/// Add a procedural sphere brush under the node of the root.
CsgSphereBrush* AddProceduralSphere(
    CsgRoot* root, const ea::string& name, const Vector3& position, const Vector3& radius, CsgOperation op)
{
    Node* brushNode = root->GetNode()->CreateChild(name);
    brushNode->SetPosition(position);
    auto* brush = brushNode->CreateComponent<CsgSphereBrush>();
    brush->SetRadius(radius);
    brush->SetOperation(op);
    return brush;
}

/// Add the model of a generated sphere to the resource cache.
void AddGeneratedSphereModel(Context* context, const ea::string& name, const CsgSphereParams& params)
{
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(params, mesh));
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    model->SetName(name);
    context->GetSubsystem<ResourceCache>()->AddManualResource(model);
}

/// Return the exact volume of the unit UV polyhedron. Each band between two ring planes is a frustum.
double UnitSphereVolume(unsigned segments, unsigned rings)
{
    const double pi = 3.14159265358979323846;
    const auto area = [&](double r) { return 0.5 * segments * r * r * std::sin(2.0 * pi / segments); };
    double volume = 0.0;
    for (unsigned row = 0; row < rings; ++row)
    {
        const double theta0 = pi * row / rings;
        const double theta1 = pi * (row + 1) / rings;
        const double a0 = area(std::sin(theta0));
        const double a1 = area(std::sin(theta1));
        volume += (std::cos(theta0) - std::cos(theta1)) / 3.0 * (a0 + a1 + std::sqrt(a0 * a1));
    }
    return volume;
}

} // namespace

TEST_CASE("CsgSphereBrush is an operand with the polyhedron volume", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    const double unit = UnitSphereVolume(24, 12);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("A sphere alone gives the volume of the polyhedron")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Alone");
        CsgSphereBrush* brush =
            AddProceduralSphere(root, "Sphere", Vector3::ZERO, Vector3(1.0f, 2.0f, 3.0f), CsgOperation::Union);
        CHECK(brush->IsShapeValid());
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().closed_);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(unit * 6.0).epsilon(1e-3));
    }

    SECTION("A subtract sphere on a face of the base removes half of its volume")
    {
        // The base box is the cube from -1 to 1. The sphere is centered on the +X face.
        // The plane x = 0 of the sphere has vertices on it, so the cut half is exact.
        CsgRoot* root = MakeBoxRoot(scene, "Procedural");
        AddProceduralSphere(root, "Cut", Vector3(1.0f, 0.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f), CsgOperation::DifferenceAB);
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(8.0 - 0.5 * unit * 0.125).epsilon(1e-3));
        CHECK(root->GetLastBuildStats().volume_ < 8.0 - 0.2);
        CHECK(errors.GetCount() == 0);

        // A model brush with the same mesh gives the same output.
        AddGeneratedSphereModel(context, "Csg/GeneratedSphere.mdl", CsgSphereParams{});
        CsgRoot* expected = MakeBoxRoot(scene, "Model");
        Node* cutNode = expected->GetNode()->CreateChild("Cut");
        cutNode->SetPosition(Vector3(1.0f, 0.0f, 0.0f));
        auto* cut = cutNode->CreateComponent<CsgBrush>();
        cut->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/GeneratedSphere.mdl"));
        cut->SetOperation(CsgOperation::DifferenceAB);
        BuildFully(expected, workQueue);
        CheckSameOutput(root, expected);
    }

    SECTION("A sphere subtracted from a box and a sphere united with a rotated box fold without a failure")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Fold");
        // A sphere inside the base box makes a closed cavity. A sphere on a corner makes a dent.
        AddProceduralSphere(root, "Cavity", Vector3(0.0f, 0.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f), CsgOperation::DifferenceAB);
        AddProceduralSphere(root, "Dent", Vector3(1.0f, 1.0f, 1.0f), Vector3(0.6f, 0.6f, 0.6f), CsgOperation::DifferenceAB);
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().valid_);
        CHECK(root->GetLastBuildStats().foldSteps_ == 2);
        CHECK(root->GetLastBuildStats().volume_ < 8.0 - 0.4);

        CsgRoot* united = MakeEmptyBaseRoot(scene, "United");
        AddProceduralBox(united, "Box", Vector3::ZERO, Vector3(2.0f, 1.0f, 1.0f), CsgOperation::Union)
            ->GetNode()->SetRotation(Quaternion(30.0f, Vector3(0.3f, 1.0f, 0.2f)));
        AddProceduralSphere(united, "Sphere", Vector3(0.7f, 0.1f, 0.2f), Vector3(0.8f, 0.6f, 0.7f), CsgOperation::Union);
        BuildFully(united, workQueue);
        CHECK(!united->GetLastBuildFailed());
        CHECK(united->GetLastBuildStats().valid_);
        CHECK(united->GetLastBuildStats().closed_);
        CHECK(united->GetLastBuildStats().foldSteps_ == 2);
        CHECK(united->GetLastBuildStats().volume_ > 2.0);
    }

    SECTION("The brush draws its generated model")
    {
        auto* debug = scene->CreateComponent<DebugRenderer>();
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Debug");
        CsgSphereBrush* brush =
            AddProceduralSphere(root, "Sphere", Vector3::ZERO, Vector3::ONE, CsgOperation::Union);
        CHECK(!debug->HasContent());
        static_cast<Component*>(brush)->DrawDebugGeometry(debug, false);
        CHECK(debug->HasContent());
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgSphereBrush rebuilds after an attribute change and reuses the fold steps before it", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[2]->GetNode()->Remove();
    CsgSphereBrush* third =
        AddProceduralSphere(root, "Third", Vector3(1.2f, 0.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f), CsgOperation::Union);
    BuildFully(root, workQueue);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);
    CHECK(!root->GetLastBuildFailed());
    double volume = root->GetLastBuildStats().volume_;

    // The attribute path: SetAttribute and ApplyAttributes. The value is clamped.
    Component* component = third;
    CHECK(component->SetAttribute("Radius", Vector3(1.0f, 0.5f, 0.5f)));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetRadius() == Vector3(1.0f, 0.5f, 0.5f));
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ > volume + 0.05);
    volume = root->GetLastBuildStats().volume_;

    CHECK(component->SetAttribute("Segments", 8));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetSegments() == 8);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ < volume);

    CHECK(component->SetAttribute("Rings", 4));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetRings() == 4);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);

    CHECK(component->SetAttribute("Smooth Normals", false));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(!third->GetSmoothNormals());
    CHECK(!component->GetAttribute("Smooth Normals").GetBool());
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().valid_);

    CHECK(component->SetAttribute("Radius", Vector3(-1.0f, 1e9f, 2.0f)));
    CHECK(component->SetAttribute("Segments", 1000));
    CHECK(component->SetAttribute("Rings", 0));
    component->ApplyAttributes();
    CHECK(third->GetRadius() == Vector3(CsgSphereParams::MinRadius, CsgSphereParams::MaxRadius, 2.0f));
    CHECK(third->GetSegments() == 128);
    CHECK(third->GetRings() == 2);
    CHECK(component->GetAttribute("Segments").GetInt() == 128);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgSphereBrush keeps its parameters and the result after a save and a load", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    CsgSphereBrush* sphere = AddProceduralSphere(
        root, "Sphere", Vector3(1.0f, 0.0f, 0.0f), Vector3(0.75f, 0.5f, 0.6f), CsgOperation::DifferenceAB);
    sphere->SetSegments(10);
    sphere->SetRings(5);
    sphere->SetSmoothNormals(false);
    BuildFully(root, workQueue);
    const double volume = root->GetLastBuildStats().volume_;
    CHECK(volume < 8.0);

    VectorBuffer buffer;
    REQUIRE(scene->Save(buffer));
    auto loaded = MakeShared<Scene>(context);
    buffer.Seek(0);
    REQUIRE(loaded->Load(buffer));
    Node* loadedRootNode = loaded->GetChild("Root");
    REQUIRE(loadedRootNode);
    auto* loadedRoot = loadedRootNode->GetComponent<CsgRoot>();
    REQUIRE(loadedRoot);
    Node* loadedNode = loadedRootNode->GetChild("Sphere");
    REQUIRE(loadedNode);
    auto* loadedSphere = loadedNode->GetComponent<CsgSphereBrush>();
    REQUIRE(loadedSphere);
    CHECK(loadedSphere->GetOperation() == CsgOperation::DifferenceAB);
    CHECK(loadedSphere->GetRadius() == sphere->GetRadius());
    CHECK(loadedSphere->GetSegments() == 10);
    CHECK(loadedSphere->GetRings() == 5);
    CHECK(!loadedSphere->GetSmoothNormals());

    REQUIRE(RunUntilOutput(loadedRoot, workQueue, 1000));
    RunFrames(loadedRoot, workQueue, 60);
    CHECK(loadedRoot->GetLastBuildStats().volume_ == Catch::Approx(volume).epsilon(1e-9));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgSphereBrush has one material slot and no Model attribute", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    context->RegisterSubsystem<VirtualFileSystem>();
    for (const char* name : {"Materials/Box.xml", "Materials/Top.xml"})
    {
        auto material = MakeShared<Material>(context);
        material->SetName(name);
        context->GetSubsystem<ResourceCache>()->AddManualResource(material);
    }

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("The attributes are the ones of a brush without Model")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgSphereBrush* sphere =
            AddProceduralSphere(root, "Sphere", Vector3::ZERO, Vector3::ONE, CsgOperation::DifferenceAB);
        Component* component = sphere;
        CHECK(component->GetAttribute("Materials").GetResourceRefList().names_.size() == 1);
        CHECK(component->GetAttribute("Operation").GetInt() == static_cast<int>(CsgOperation::DifferenceAB));
        for (const AttributeInfo& info : *component->GetAttributes())
            CHECK(info.name_ != "Model");
        CHECK(component->GetAttribute("Radius").GetVector3() == Vector3::ONE);
        CHECK(component->GetAttribute("Segments").GetInt() == 24);
        CHECK(component->GetAttribute("Rings").GetInt() == 12);
        CHECK(component->GetAttribute("Smooth Normals").GetBool());

        sphere->SetMaterials(
            ResourceRefList(Material::GetTypeStatic(), ea::vector<ea::string>(4u, ea::string("Materials/Top.xml"))));
        CHECK(sphere->GetMaterialsAttr().names_.size() == 1);
        CHECK(sphere->GetMaterialsAttr().names_[0] == "Materials/Top.xml");
    }

    SECTION("The material of the one slot reaches the result")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgSphereBrush* sphere =
            AddProceduralSphere(root, "Sphere", Vector3::ZERO, Vector3::ONE, CsgOperation::Union);
        sphere->SetMaterials(ResourceRefList(Material::GetTypeStatic(), {"Materials/Box.xml"}));
        BuildFully(root, workQueue);
        CHECK(GetOutput(root)->GetModel()->GetNumGeometries() == 1);
        CHECK(GetOutput(root)->GetMaterialsAttr().names_ == ea::vector<ea::string>{"Materials/Box.xml"});
    }

    SECTION("A subtract sphere gives its material to the cut faces")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Root");
        CsgSphereBrush* sphere = AddProceduralSphere(
            root, "Sphere", Vector3(1.0f, 0.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f), CsgOperation::DifferenceAB);
        sphere->SetMaterials(ResourceRefList(Material::GetTypeStatic(), {"Materials/Box.xml"}));
        BuildFully(root, workQueue);
        const ea::vector<ea::string>& names = GetOutput(root)->GetMaterialsAttr().names_;
        CHECK(ea::find(names.begin(), names.end(), "Materials/Box.xml") != names.end());
        CHECK(names.size() == 2);
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgSphereBrush generates the model fast", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    const auto measure = [&](const CsgSphereParams& params)
    {
        // The best of several runs. The limits are generous, so a loaded CI machine does not fail.
        double best = 1e9;
        for (unsigned run = 0; run < 5; ++run)
        {
            const auto start = std::chrono::steady_clock::now();
            CsgShapeMesh mesh;
            const bool generated = CsgGenerateSphere(params, mesh);
            SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
            const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
            REQUIRE(generated);
            REQUIRE(model);
            best = ea::min(best, elapsed.count());
        }
        return best;
    };

    const double defaultMs = measure(CsgSphereParams{});
    CsgSphereParams fine;
    fine.segments_ = 128;
    fine.rings_ = 64;
    const double fineMs = measure(fine);
    WARN("Sphere generation and model build: default " << defaultMs << " ms, 128x64 " << fineMs << " ms");
    // A sanitizer build is much slower, so it gets a wider limit.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
    const double slowFactor = 10.0;
#else
    const double slowFactor = 1.0;
#endif
    CHECK(defaultMs < 15.0 * slowFactor);
    CHECK(fineMs < 150.0 * slowFactor);
}

namespace
{

/// Add a procedural cylinder brush under the node of the root.
CsgCylinderBrush* AddProceduralCylinder(CsgRoot* root, const ea::string& name, const CsgCylinderParams& params,
    CsgOperation op, const Vector3& position = Vector3::ZERO)
{
    Node* brushNode = root->GetNode()->CreateChild(name);
    brushNode->SetPosition(position);
    auto* brush = brushNode->CreateComponent<CsgCylinderBrush>();
    brush->SetParams(params);
    brush->SetOperation(op);
    return brush;
}

/// Add the model of a generated cylinder to the resource cache.
void AddGeneratedCylinderModel(Context* context, const ea::string& name, const CsgCylinderParams& params)
{
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(params, mesh));
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    model->SetName(name);
    context->GetSubsystem<ResourceCache>()->AddManualResource(model);
}

/// Return the exact volume of a cylinder polyhedron with circular ends and a given height: a frustum.
double FrustumPolyhedronVolume(unsigned segments, double bottomRadius, double topRadius, double height)
{
    const double pi = 3.14159265358979323846;
    const double k = 0.5 * segments * std::sin(2.0 * pi / segments);
    const double a0 = k * bottomRadius * bottomRadius;
    const double a1 = k * topRadius * topRadius;
    return height / 3.0 * (a0 + a1 + std::sqrt(a0 * a1));
}

} // namespace

TEST_CASE("CsgCylinderBrush is an operand with the polyhedron volume", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("A cylinder alone gives the volume of the polyhedron")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Alone");
        CsgCylinderParams params;
        params.bottomRadius_ = Vector2(1.0f, 1.0f);
        params.topRadius_ = Vector2(1.0f, 1.0f);
        params.topCenter_ = Vector3(0.0f, 2.0f, 0.0f);
        CsgCylinderBrush* brush = AddProceduralCylinder(root, "Cylinder", params, CsgOperation::Union);
        CHECK(brush->IsShapeValid());
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().closed_);
        CHECK(root->GetLastBuildStats().volume_ ==
            Catch::Approx(FrustumPolyhedronVolume(24, 1.0, 1.0, 2.5)).epsilon(1e-3));
    }

    SECTION("A cone alone gives the volume of the polyhedron")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Cone");
        CsgCylinderParams params;
        params.topRadius_ = Vector2::ZERO;
        AddProceduralCylinder(root, "Cone", params, CsgOperation::Union);
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().closed_);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(FrustumPolyhedronVolume(24, 0.5, 0.0, 1.0)).epsilon(1e-3));
    }

    SECTION("A subtract cylinder makes a hole through the box")
    {
        // The base box is the cube from -1 to 1. The cylinder is longer than the box.
        CsgRoot* root = MakeBoxRoot(scene, "Procedural");
        CsgCylinderParams params;
        params.bottomCenter_ = Vector3(0.0f, -2.0f, 0.0f);
        params.topCenter_ = Vector3(0.0f, 2.0f, 0.0f);
        params.bottomRadius_ = Vector2(0.5f, 0.5f);
        params.topRadius_ = Vector2(0.5f, 0.5f);
        AddProceduralCylinder(root, "Hole", params, CsgOperation::DifferenceAB);
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().valid_);
        CHECK(root->GetLastBuildStats().volume_ ==
            Catch::Approx(8.0 - FrustumPolyhedronVolume(24, 0.5, 0.5, 2.0)).epsilon(1e-3));
        CHECK(errors.GetCount() == 0);

        // A model brush with the same mesh gives the same output.
        AddGeneratedCylinderModel(context, "Csg/GeneratedCylinder.mdl", params);
        CsgRoot* expected = MakeBoxRoot(scene, "Model");
        Node* cutNode = expected->GetNode()->CreateChild("Cut");
        auto* cut = cutNode->CreateComponent<CsgBrush>();
        cut->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/GeneratedCylinder.mdl"));
        cut->SetOperation(CsgOperation::DifferenceAB);
        BuildFully(expected, workQueue);
        CheckSameOutput(root, expected);
    }

    SECTION("A cone, a sheared cylinder and a bulge subtracted from a box fold without a failure")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Fold");
        CsgCylinderParams cone;
        cone.bottomCenter_ = Vector3(0.0f, -1.5f, 0.0f);
        cone.topCenter_ = Vector3(0.0f, 0.5f, 0.0f);
        cone.bottomRadius_ = Vector2(0.6f, 0.6f);
        cone.topRadius_ = Vector2::ZERO;
        AddProceduralCylinder(root, "Cone", cone, CsgOperation::DifferenceAB);
        CsgCylinderParams sheared;
        sheared.bottomCenter_ = Vector3(0.0f, 0.0f, 0.0f);
        sheared.topCenter_ = Vector3(0.6f, 1.5f, 0.2f);
        sheared.bottomRadius_ = Vector2(0.3f, 0.2f);
        sheared.topRadius_ = Vector2(0.2f, 0.2f);
        sheared.bulge_ = 1.0f;
        sheared.heightSegments_ = 3;
        AddProceduralCylinder(root, "Sheared", sheared, CsgOperation::DifferenceAB, Vector3(0.0f, -0.5f, 0.5f));
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().valid_);
        CHECK(root->GetLastBuildStats().foldSteps_ == 2);
        CHECK(root->GetLastBuildStats().volume_ < 8.0 - 0.2);
    }

    SECTION("A cylinder united with a rotated box folds without a failure")
    {
        CsgRoot* united = MakeEmptyBaseRoot(scene, "United");
        AddProceduralBox(united, "Box", Vector3::ZERO, Vector3(2.0f, 1.0f, 1.0f), CsgOperation::Union)
            ->GetNode()->SetRotation(Quaternion(30.0f, Vector3(0.3f, 1.0f, 0.2f)));
        CsgCylinderParams params;
        params.bottomRadius_ = Vector2(0.8f, 0.6f);
        params.topRadius_ = Vector2(0.4f, 0.3f);
        AddProceduralCylinder(united, "Cylinder", params, CsgOperation::Union, Vector3(0.7f, 0.1f, 0.2f));
        BuildFully(united, workQueue);
        CHECK(!united->GetLastBuildFailed());
        CHECK(united->GetLastBuildStats().valid_);
        CHECK(united->GetLastBuildStats().closed_);
        CHECK(united->GetLastBuildStats().foldSteps_ == 2);
        CHECK(united->GetLastBuildStats().volume_ > 2.0);
    }

    SECTION("A brush with a shape that is not valid gives no operand")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Invalid");
        CsgCylinderParams params;
        params.topCenter_ = params.bottomCenter_;
        CsgCylinderBrush* brush = AddProceduralCylinder(root, "Invalid", params, CsgOperation::DifferenceAB);
        CHECK(!brush->IsShapeValid());
        BuildFully(root, workQueue);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(8.0).epsilon(1e-6));
    }

    SECTION("The brush draws its generated model")
    {
        auto* debug = scene->CreateComponent<DebugRenderer>();
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Debug");
        CsgCylinderBrush* brush = AddProceduralCylinder(root, "Cylinder", CsgCylinderParams{}, CsgOperation::Union);
        CHECK(!debug->HasContent());
        static_cast<Component*>(brush)->DrawDebugGeometry(debug, false);
        CHECK(debug->HasContent());
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgCylinderBrush rebuilds after an attribute change and reuses the fold steps before it", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[2]->GetNode()->Remove();
    CsgCylinderBrush* third = AddProceduralCylinder(
        root, "Third", CsgCylinderParams{}, CsgOperation::Union, Vector3(1.2f, 0.0f, 0.0f));
    BuildFully(root, workQueue);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);
    CHECK(!root->GetLastBuildFailed());
    double volume = root->GetLastBuildStats().volume_;

    // The attribute path: SetAttribute and ApplyAttributes. The value is clamped.
    Component* component = third;
    CHECK(component->SetAttribute("Top Radius", Vector2(1.0f, 1.0f)));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetTopRadius() == Vector2(1.0f, 1.0f));
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ > volume + 0.05);
    volume = root->GetLastBuildStats().volume_;

    CHECK(component->SetAttribute("Segments", 8));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetSegments() == 8);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ < volume);

    CHECK(component->SetAttribute("Height Segments", 3));
    CHECK(component->SetAttribute("Bulge", 1.0f));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetHeightSegments() == 3);
    CHECK(third->GetBulge() == 1.0f);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().valid_);

    CHECK(component->SetAttribute("Top Center", Vector3(0.5f, 1.0f, 0.0f)));
    CHECK(component->SetAttribute("Bottom Center", Vector3(0.0f, -0.25f, 0.0f)));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetTopCenter() == Vector3(0.5f, 1.0f, 0.0f));
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().valid_);

    CHECK(component->SetAttribute("Smooth Normals", false));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(!third->GetSmoothNormals());
    CHECK(!component->GetAttribute("Smooth Normals").GetBool());
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().valid_);

    CHECK(component->SetAttribute("Bottom Radius", Vector2(-1.0f, 1e9f)));
    CHECK(component->SetAttribute("Segments", 1000));
    CHECK(component->SetAttribute("Height Segments", 0));
    CHECK(component->SetAttribute("Bulge", 100.0f));
    CHECK(component->SetAttribute("Top Center", Vector3(1e9f, 0.0f, 0.0f)));
    component->ApplyAttributes();
    CHECK(third->GetBottomRadius() == Vector2(CsgCylinderParams::MinRadius, CsgCylinderParams::MaxRadius));
    CHECK(third->GetSegments() == 128);
    CHECK(third->GetHeightSegments() == 1);
    CHECK(third->GetBulge() == CsgCylinderParams::MaxBulge);
    CHECK(third->GetTopCenter().x_ == CsgCylinderParams::MaxCoordinate);
    CHECK(component->GetAttribute("Segments").GetInt() == 128);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgCylinderBrush keeps its parameters and the result after a save and a load", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    CsgCylinderParams params;
    params.bottomCenter_ = Vector3(0.0f, -1.5f, 0.25f);
    params.topCenter_ = Vector3(0.5f, 1.5f, 0.0f);
    params.bottomRadius_ = Vector2(0.75f, 0.5f);
    params.topRadius_ = Vector2::ZERO;
    params.segments_ = 10;
    params.heightSegments_ = 3;
    params.bulge_ = 0.5f;
    params.smoothNormals_ = false;
    CsgCylinderBrush* cylinder = AddProceduralCylinder(
        root, "Cylinder", params, CsgOperation::DifferenceAB, Vector3(0.5f, 0.0f, 0.0f));
    BuildFully(root, workQueue);
    const double volume = root->GetLastBuildStats().volume_;
    CHECK(volume < 8.0);

    VectorBuffer buffer;
    REQUIRE(scene->Save(buffer));
    auto loaded = MakeShared<Scene>(context);
    buffer.Seek(0);
    REQUIRE(loaded->Load(buffer));
    Node* loadedRootNode = loaded->GetChild("Root");
    REQUIRE(loadedRootNode);
    auto* loadedRoot = loadedRootNode->GetComponent<CsgRoot>();
    REQUIRE(loadedRoot);
    Node* loadedNode = loadedRootNode->GetChild("Cylinder");
    REQUIRE(loadedNode);
    auto* loadedCylinder = loadedNode->GetComponent<CsgCylinderBrush>();
    REQUIRE(loadedCylinder);
    CHECK(loadedCylinder->GetOperation() == CsgOperation::DifferenceAB);
    const CsgCylinderParams& loadedParams = loadedCylinder->GetParams();
    CHECK(loadedParams.bottomCenter_ == cylinder->GetBottomCenter());
    CHECK(loadedParams.topCenter_ == cylinder->GetTopCenter());
    CHECK(loadedParams.bottomRadius_ == cylinder->GetBottomRadius());
    CHECK(loadedParams.topRadius_ == Vector2::ZERO);
    CHECK(loadedParams.segments_ == 10);
    CHECK(loadedParams.heightSegments_ == 3);
    CHECK(loadedParams.bulge_ == 0.5f);
    CHECK(!loadedParams.smoothNormals_);

    REQUIRE(RunUntilOutput(loadedRoot, workQueue, 1000));
    RunFrames(loadedRoot, workQueue, 60);
    CHECK(loadedRoot->GetLastBuildStats().volume_ == Catch::Approx(volume).epsilon(1e-9));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgCylinderBrush has three material slots and no Model attribute", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    context->RegisterSubsystem<VirtualFileSystem>();
    for (const char* name : {"Materials/Box.xml", "Materials/Top.xml", "Materials/Bottom.xml"})
    {
        auto material = MakeShared<Material>(context);
        material->SetName(name);
        context->GetSubsystem<ResourceCache>()->AddManualResource(material);
    }
    const ResourceRefList threeMaterials(
        Material::GetTypeStatic(), {"Materials/Box.xml", "Materials/Top.xml", "Materials/Bottom.xml"});

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("The attributes are the ones of a brush without Model")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgCylinderBrush* cylinder =
            AddProceduralCylinder(root, "Cylinder", CsgCylinderParams{}, CsgOperation::DifferenceAB);
        Component* component = cylinder;
        CHECK(component->GetAttribute("Materials").GetResourceRefList().names_.size() == 3);
        CHECK(component->GetAttribute("Operation").GetInt() == static_cast<int>(CsgOperation::DifferenceAB));
        for (const AttributeInfo& info : *component->GetAttributes())
            CHECK(info.name_ != "Model");
        CHECK(component->GetAttribute("Bottom Center").GetVector3() == Vector3(0.0f, -0.5f, 0.0f));
        CHECK(component->GetAttribute("Top Center").GetVector3() == Vector3(0.0f, 0.5f, 0.0f));
        CHECK(component->GetAttribute("Bottom Radius").GetVector2() == Vector2(0.5f, 0.5f));
        CHECK(component->GetAttribute("Top Radius").GetVector2() == Vector2(0.5f, 0.5f));
        CHECK(component->GetAttribute("Segments").GetInt() == 24);
        CHECK(component->GetAttribute("Height Segments").GetInt() == 1);
        CHECK(component->GetAttribute("Bulge").GetFloat() == 0.0f);
        CHECK(component->GetAttribute("Smooth Normals").GetBool());

        cylinder->SetMaterials(ResourceRefList(
            Material::GetTypeStatic(), ea::vector<ea::string>(5u, ea::string("Materials/Top.xml"))));
        CHECK(cylinder->GetMaterialsAttr().names_.size() == 3);
        cylinder->SetMaterials(ResourceRefList(Material::GetTypeStatic(), {"Materials/Top.xml"}));
        CHECK(cylinder->GetMaterialsAttr().names_.size() == 3);
        CHECK(cylinder->GetMaterialsAttr().names_[0] == "Materials/Top.xml");
        CHECK(cylinder->GetMaterialsAttr().names_[2].empty());
    }

    SECTION("The material of slot 1 reaches the top cap")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgCylinderBrush* cylinder =
            AddProceduralCylinder(root, "Cylinder", CsgCylinderParams{}, CsgOperation::Union);
        cylinder->SetMaterials(threeMaterials);
        BuildFully(root, workQueue);
        const Model* output = GetOutput(root)->GetModel();
        const ea::vector<ea::string>& names = GetOutput(root)->GetMaterialsAttr().names_;
        REQUIRE(output->GetNumGeometries() == 3);
        REQUIRE(names.size() == 3);
        // Find the geometry with the top cap: all its triangles have the normal +Y.
        bool found = false;
        for (unsigned i = 0; i < output->GetNumGeometries(); ++i)
        {
            const Geometry* geometry = output->GetGeometry(i, 0);
            ea::vector<CsgPolygon> polygons = CsgBuildPolygonsFromModel(output, Matrix3x4::IDENTITY);
            bool allUp = true;
            unsigned count = 0;
            for (const CsgPolygon& polygon : polygons)
            {
                if ((polygon.shared_ & 0xFFFFu) != i)
                    continue;
                ++count;
                allUp = allUp && polygon.vertices_[0].GetNormal().Equals(Vector3::UP, 1e-4f);
            }
            if (geometry && count > 0 && allUp)
            {
                CHECK(names[i] == "Materials/Top.xml");
                found = true;
            }
        }
        CHECK(found);
        for (const char* name : {"Materials/Box.xml", "Materials/Top.xml", "Materials/Bottom.xml"})
            CHECK(ea::find(names.begin(), names.end(), name) != names.end());
    }

    SECTION("A cone keeps the slots: the top cap is slot 1, the bottom cap is slot 2")
    {
        CsgRoot* bottomRoot = MakeEmptyBaseRoot(scene, "BottomApex");
        CsgCylinderParams bottomApex;
        bottomApex.bottomRadius_ = Vector2::ZERO;
        AddProceduralCylinder(bottomRoot, "Cone", bottomApex, CsgOperation::Union)->SetMaterials(threeMaterials);
        BuildFully(bottomRoot, workQueue);
        const ea::vector<ea::string>& bottomNames = GetOutput(bottomRoot)->GetMaterialsAttr().names_;
        CHECK(ea::find(bottomNames.begin(), bottomNames.end(), "Materials/Box.xml") != bottomNames.end());
        CHECK(ea::find(bottomNames.begin(), bottomNames.end(), "Materials/Top.xml") != bottomNames.end());
        CHECK(ea::find(bottomNames.begin(), bottomNames.end(), "Materials/Bottom.xml") == bottomNames.end());

        CsgRoot* topRoot = MakeEmptyBaseRoot(scene, "TopApex");
        CsgCylinderParams topApex;
        topApex.topRadius_ = Vector2::ZERO;
        AddProceduralCylinder(topRoot, "Cone", topApex, CsgOperation::Union)->SetMaterials(threeMaterials);
        BuildFully(topRoot, workQueue);
        const ea::vector<ea::string>& topNames = GetOutput(topRoot)->GetMaterialsAttr().names_;
        CHECK(ea::find(topNames.begin(), topNames.end(), "Materials/Box.xml") != topNames.end());
        CHECK(ea::find(topNames.begin(), topNames.end(), "Materials/Top.xml") == topNames.end());
        CHECK(ea::find(topNames.begin(), topNames.end(), "Materials/Bottom.xml") != topNames.end());
        CHECK(!topRoot->GetLastBuildFailed());
    }

    SECTION("A subtract cylinder gives its materials to the cut faces")
    {
        CsgRoot* root = MakeBoxRoot(scene, "Root");
        CsgCylinderParams params;
        params.bottomCenter_ = Vector3(0.0f, -2.0f, 0.0f);
        params.topCenter_ = Vector3(0.0f, 2.0f, 0.0f);
        CsgCylinderBrush* cylinder = AddProceduralCylinder(root, "Hole", params, CsgOperation::DifferenceAB);
        cylinder->SetMaterials(threeMaterials);
        BuildFully(root, workQueue);
        const ea::vector<ea::string>& names = GetOutput(root)->GetMaterialsAttr().names_;
        CHECK(ea::find(names.begin(), names.end(), "Materials/Box.xml") != names.end());
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgCylinderBrush generates the model fast", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    const auto measure = [&](const CsgCylinderParams& params)
    {
        // The best of several runs. The limits are generous, so a loaded CI machine does not fail.
        double best = 1e9;
        for (unsigned run = 0; run < 5; ++run)
        {
            const auto start = std::chrono::steady_clock::now();
            CsgShapeMesh mesh;
            const bool generated = CsgGenerateCylinder(params, mesh);
            SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
            const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
            REQUIRE(generated);
            REQUIRE(model);
            best = ea::min(best, elapsed.count());
        }
        return best;
    };

    const double defaultMs = measure(CsgCylinderParams{});
    CsgCylinderParams fine;
    fine.segments_ = 128;
    fine.heightSegments_ = 64;
    const double fineMs = measure(fine);
    WARN("Cylinder generation and model build: default " << defaultMs << " ms, 128x64 " << fineMs << " ms");
    // A sanitizer build is much slower, so it gets a wider limit.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
    const double slowFactor = 10.0;
#else
    const double slowFactor = 1.0;
#endif
    CHECK(defaultMs < 15.0 * slowFactor);
    CHECK(fineMs < 150.0 * slowFactor);
}

namespace
{

/// Add a procedural ring brush under the node of the root.
CsgRingBrush* AddProceduralRing(CsgRoot* root, const ea::string& name, const CsgRingParams& params, CsgOperation op,
    const Vector3& position = Vector3::ZERO)
{
    Node* brushNode = root->GetNode()->CreateChild(name);
    brushNode->SetPosition(position);
    auto* brush = brushNode->CreateComponent<CsgRingBrush>();
    brush->SetParams(params);
    brush->SetOperation(op);
    return brush;
}

/// Add the model of a generated ring to the resource cache.
void AddGeneratedRingModel(Context* context, const ea::string& name, const CsgRingParams& params)
{
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateRing(params, mesh));
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    model->SetName(name);
    context->GetSubsystem<ResourceCache>()->AddManualResource(model);
}

/// Return the exact volume of a ring polyhedron with a circular tube section and the tube angle 0.
/// It is sin(step) times the integral of the radial coordinate over the section polygon (shoelace formula).
double RingPolyhedronVolume(unsigned segments, unsigned tubeSegments, double radius, double tube, double arcDegrees)
{
    const double pi = 3.14159265358979323846;
    double integral = 0.0;
    for (unsigned j = 0; j < tubeSegments; ++j)
    {
        const double a0 = 2.0 * pi * j / tubeSegments;
        const double a1 = 2.0 * pi * (j + 1) / tubeSegments;
        const double x0 = radius + tube * std::cos(a0);
        const double x1 = radius + tube * std::cos(a1);
        integral += (x0 + x1) * (x0 * tube * std::sin(a1) - x1 * tube * std::sin(a0)) / 6.0;
    }
    return segments * std::sin(arcDegrees * pi / 180.0 / segments) * integral;
}

} // namespace

TEST_CASE("CsgRingBrush is an operand with the polyhedron volume", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("A ring alone gives the volume of the polyhedron")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Alone");
        CsgRingParams params;
        params.radius_ = 1.0f;
        params.tubeRadius_ = Vector2(0.3f, 0.3f);
        CsgRingBrush* brush = AddProceduralRing(root, "Ring", params, CsgOperation::Union);
        CHECK(brush->IsShapeValid());
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().closed_);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(RingPolyhedronVolume(32, 12, 1.0, 0.3, 360.0)).epsilon(3e-3));
    }

    SECTION("An arc with end caps is closed and gives the volume of the polyhedron")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Arc");
        CsgRingParams params;
        params.radius_ = 1.0f;
        params.tubeRadius_ = Vector2(0.3f, 0.3f);
        params.arc_ = 180.0f;
        AddProceduralRing(root, "Ring", params, CsgOperation::Union);
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().closed_);
        CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(RingPolyhedronVolume(32, 12, 1.0, 0.3, 180.0)).epsilon(3e-3));
    }

    SECTION("A subtract ring cuts the box and gives the same output as a model brush")
    {
        // The base box is the cube from -1 to 1. The ring crosses its side faces.
        CsgRoot* root = MakeBoxRoot(scene, "Procedural");
        CsgRingParams params;
        params.radius_ = 1.0f;
        params.tubeRadius_ = Vector2(0.3f, 0.25f);
        AddProceduralRing(root, "Cut", params, CsgOperation::DifferenceAB, Vector3(0.0f, 0.1f, 0.0f));
        BuildFully(root, workQueue);
        CHECK(!root->GetLastBuildFailed());
        CHECK(root->GetLastBuildStats().valid_);
        CHECK(root->GetLastBuildStats().volume_ < 8.0 - 0.1);
        CHECK(errors.GetCount() == 0);

        AddGeneratedRingModel(context, "Csg/GeneratedRing.mdl", params);
        CsgRoot* expected = MakeBoxRoot(scene, "Model");
        Node* cutNode = expected->GetNode()->CreateChild("Cut");
        cutNode->SetPosition(Vector3(0.0f, 0.1f, 0.0f));
        auto* cut = cutNode->CreateComponent<CsgBrush>();
        cut->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/GeneratedRing.mdl"));
        cut->SetOperation(CsgOperation::DifferenceAB);
        BuildFully(expected, workQueue);
        CheckSameOutput(root, expected);
    }

    SECTION("A ring united with a rotated box folds without a failure")
    {
        CsgRoot* united = MakeEmptyBaseRoot(scene, "United");
        AddProceduralBox(united, "Box", Vector3::ZERO, Vector3(2.0f, 1.0f, 1.0f), CsgOperation::Union)
            ->GetNode()->SetRotation(Quaternion(30.0f, Vector3(0.3f, 1.0f, 0.2f)));
        CsgRingParams params;
        params.radius_ = 0.8f;
        params.tubeRadius_ = Vector2(0.3f, 0.2f);
        params.tubeAngle_ = 45.0f;
        params.tubeSegments_ = 4;
        AddProceduralRing(united, "Ring", params, CsgOperation::Union, Vector3(0.7f, 0.1f, 0.2f));
        BuildFully(united, workQueue);
        CHECK(!united->GetLastBuildFailed());
        CHECK(united->GetLastBuildStats().valid_);
        CHECK(united->GetLastBuildStats().closed_);
        CHECK(united->GetLastBuildStats().foldSteps_ == 2);
        CHECK(united->GetLastBuildStats().volume_ > 2.0);
    }

    SECTION("The brush draws its generated model")
    {
        auto* debug = scene->CreateComponent<DebugRenderer>();
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Debug");
        CsgRingBrush* brush = AddProceduralRing(root, "Ring", CsgRingParams{}, CsgOperation::Union);
        CHECK(!debug->HasContent());
        static_cast<Component*>(brush)->DrawDebugGeometry(debug, false);
        CHECK(debug->HasContent());
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRingBrush rebuilds after an attribute change and reuses the fold steps before it", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    AddBoxModel(context, "Csg/BoxB.mdl", Vector3::ONE);

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    ea::vector<CsgBrush*> brushes = AddFoldBrushes(root);
    brushes[2]->GetNode()->Remove();
    CsgRingBrush* third = AddProceduralRing(root, "Third", CsgRingParams{}, CsgOperation::Union, Vector3(1.2f, 0.0f, 0.0f));
    BuildFully(root, workQueue);
    CHECK(root->GetLastBuildStats().foldSteps_ == 3);
    CHECK(!root->GetLastBuildFailed());
    double volume = root->GetLastBuildStats().volume_;

    // The attribute path: SetAttribute and ApplyAttributes. The value is clamped.
    Component* component = third;
    CHECK(component->SetAttribute("Tube Radius", Vector2(0.3f, 0.3f)));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetTubeRadius() == Vector2(0.3f, 0.3f));
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ > volume + 0.01);
    volume = root->GetLastBuildStats().volume_;

    CHECK(component->SetAttribute("Segments", 8));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetSegments() == 8);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().volume_ < volume);

    CHECK(component->SetAttribute("Tube Segments", 4));
    CHECK(component->SetAttribute("Tube Angle", 45.0f));
    CHECK(component->SetAttribute("Radius", 0.7f));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetTubeSegments() == 4);
    CHECK(third->GetTubeAngle() == 45.0f);
    CHECK(third->GetRadius() == 0.7f);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().valid_);

    CHECK(component->SetAttribute("Arc", 120.0f));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(third->GetArc() == 120.0f);
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().valid_);

    CHECK(component->SetAttribute("Smooth Normals", false));
    component->ApplyAttributes();
    RunFrames(root, workQueue, 60);
    CHECK(!third->GetSmoothNormals());
    CHECK(!component->GetAttribute("Smooth Normals").GetBool());
    CHECK(root->GetLastBuildStats().foldSteps_ == 1);
    CHECK(root->GetLastBuildStats().valid_);

    CHECK(component->SetAttribute("Radius", -1.0f));
    CHECK(component->SetAttribute("Tube Radius", Vector2(1e9f, -1.0f)));
    CHECK(component->SetAttribute("Segments", 1000));
    CHECK(component->SetAttribute("Tube Segments", 0));
    CHECK(component->SetAttribute("Arc", 1000.0f));
    CHECK(component->SetAttribute("Tube Angle", -1000.0f));
    component->ApplyAttributes();
    CHECK(third->GetRadius() == CsgRingParams::MinRadius);
    CHECK(third->GetTubeRadius().x_ == Catch::Approx(CsgRingParams::MinRadius * CsgRingParams::MaxInnerFraction));
    CHECK(third->GetTubeRadius().y_ == CsgRingParams::MinTubeRadius);
    CHECK(third->GetSegments() == 128);
    CHECK(third->GetTubeSegments() == 3);
    CHECK(third->GetArc() == 360.0f);
    CHECK(third->GetTubeAngle() == -180.0f);
    CHECK(component->GetAttribute("Segments").GetInt() == 128);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRingBrush keeps its parameters and the result after a save and a load", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    CsgRoot* root = MakeBoxRoot(scene, "Root");
    CsgRingParams params;
    params.radius_ = 1.0f;
    params.tubeRadius_ = Vector2(0.3f, 0.2f);
    params.segments_ = 10;
    params.tubeSegments_ = 7;
    params.arc_ = 200.0f;
    params.tubeAngle_ = -30.0f;
    params.smoothNormals_ = false;
    CsgRingBrush* ring = AddProceduralRing(root, "Ring", params, CsgOperation::DifferenceAB, Vector3(0.0f, 0.1f, 0.0f));
    BuildFully(root, workQueue);
    const double volume = root->GetLastBuildStats().volume_;
    CHECK(volume < 8.0);

    VectorBuffer buffer;
    REQUIRE(scene->Save(buffer));
    auto loaded = MakeShared<Scene>(context);
    buffer.Seek(0);
    REQUIRE(loaded->Load(buffer));
    Node* loadedRootNode = loaded->GetChild("Root");
    REQUIRE(loadedRootNode);
    auto* loadedRoot = loadedRootNode->GetComponent<CsgRoot>();
    REQUIRE(loadedRoot);
    Node* loadedNode = loadedRootNode->GetChild("Ring");
    REQUIRE(loadedNode);
    auto* loadedRing = loadedNode->GetComponent<CsgRingBrush>();
    REQUIRE(loadedRing);
    CHECK(loadedRing->GetOperation() == CsgOperation::DifferenceAB);
    const CsgRingParams& loadedParams = loadedRing->GetParams();
    CHECK(loadedParams.radius_ == ring->GetRadius());
    CHECK(loadedParams.tubeRadius_ == ring->GetTubeRadius());
    CHECK(loadedParams.segments_ == 10);
    CHECK(loadedParams.tubeSegments_ == 7);
    CHECK(loadedParams.arc_ == 200.0f);
    CHECK(loadedParams.tubeAngle_ == -30.0f);
    CHECK(!loadedParams.smoothNormals_);

    REQUIRE(RunUntilOutput(loadedRoot, workQueue, 1000));
    RunFrames(loadedRoot, workQueue, 60);
    CHECK(loadedRoot->GetLastBuildStats().volume_ == Catch::Approx(volume).epsilon(1e-9));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRingBrush has six material slots and no Model attribute", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    auto* workQueue = context->GetSubsystem<WorkQueue>();
    AddBoxModel(context, "Csg/BoxA.mdl", Vector3::ONE);
    context->RegisterSubsystem<VirtualFileSystem>();
    const char* const materialNames[6] = {"Materials/Outer.xml", "Materials/Inner.xml", "Materials/Top.xml",
        "Materials/Bottom.xml", "Materials/Start.xml", "Materials/End.xml"};
    for (const char* name : materialNames)
    {
        auto material = MakeShared<Material>(context);
        material->SetName(name);
        context->GetSubsystem<ResourceCache>()->AddManualResource(material);
    }
    const ResourceRefList sixMaterials(Material::GetTypeStatic(),
        {materialNames[0], materialNames[1], materialNames[2], materialNames[3], materialNames[4], materialNames[5]});

    ErrorCounter errors(context);
    auto scene = MakeShared<Scene>(context);
    scene->CreateComponent<Octree>();

    SECTION("The attributes are the ones of a brush without Model")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgRingBrush* ring = AddProceduralRing(root, "Ring", CsgRingParams{}, CsgOperation::DifferenceAB);
        Component* component = ring;
        CHECK(component->GetAttribute("Materials").GetResourceRefList().names_.size() == 6);
        CHECK(component->GetAttribute("Operation").GetInt() == static_cast<int>(CsgOperation::DifferenceAB));
        for (const AttributeInfo& info : *component->GetAttributes())
            CHECK(info.name_ != "Model");
        CHECK(component->GetAttribute("Radius").GetFloat() == 0.5f);
        CHECK(component->GetAttribute("Tube Radius").GetVector2() == Vector2(0.15f, 0.15f));
        CHECK(component->GetAttribute("Segments").GetInt() == 32);
        CHECK(component->GetAttribute("Tube Segments").GetInt() == 12);
        CHECK(component->GetAttribute("Arc").GetFloat() == 360.0f);
        CHECK(component->GetAttribute("Tube Angle").GetFloat() == 0.0f);
        CHECK(component->GetAttribute("Smooth Normals").GetBool());

        ring->SetMaterials(ResourceRefList(
            Material::GetTypeStatic(), ea::vector<ea::string>(9u, ea::string("Materials/Top.xml"))));
        CHECK(ring->GetMaterialsAttr().names_.size() == 6);
        ring->SetMaterials(ResourceRefList(Material::GetTypeStatic(), {"Materials/Top.xml"}));
        CHECK(ring->GetMaterialsAttr().names_.size() == 6);
        CHECK(ring->GetMaterialsAttr().names_[0] == "Materials/Top.xml");
        CHECK(ring->GetMaterialsAttr().names_[5].empty());
    }

    SECTION("The material of slot 3 reaches the bottom triangles")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgRingBrush* ring = AddProceduralRing(root, "Ring", CsgRingParams{}, CsgOperation::Union);
        ring->SetMaterials(sixMaterials);
        BuildFully(root, workQueue);
        const Model* output = GetOutput(root)->GetModel();
        const ea::vector<ea::string>& names = GetOutput(root)->GetMaterialsAttr().names_;
        REQUIRE(output->GetNumGeometries() == names.size());
        const ea::vector<CsgPolygon> polygons = CsgBuildPolygonsFromModel(output, Matrix3x4::IDENTITY);
        // The output geometry whose triangles all lie in the bottom half and face down has the bottom material.
        unsigned numBottom = 0;
        for (unsigned i = 0; i < output->GetNumGeometries(); ++i)
        {
            bool all = true;
            unsigned count = 0;
            for (const CsgPolygon& polygon : polygons)
            {
                if ((polygon.shared_ & 0xFFFFu) != i)
                    continue;
                ++count;
                for (const ModelVertex& vertex : polygon.vertices_)
                    all = all && vertex.GetPosition().y_ <= 1e-5f && vertex.GetNormal().y_ < 0.0f;
            }
            if (count > 0 && all)
            {
                CHECK(names[i] == "Materials/Bottom.xml");
                ++numBottom;
            }
        }
        CHECK(numBottom == 1);
        // A full ring has no caps: the materials of the caps are not in the output.
        CHECK(ea::find(names.begin(), names.end(), "Materials/Start.xml") == names.end());
        CHECK(ea::find(names.begin(), names.end(), "Materials/End.xml") == names.end());
        for (unsigned i = 0; i < 4; ++i)
            CHECK(ea::find(names.begin(), names.end(), materialNames[i]) != names.end());
    }

    SECTION("An arc has the cap materials")
    {
        CsgRoot* root = MakeEmptyBaseRoot(scene, "Root");
        CsgRingParams params;
        params.arc_ = 90.0f;
        AddProceduralRing(root, "Ring", params, CsgOperation::Union)->SetMaterials(sixMaterials);
        BuildFully(root, workQueue);
        const ea::vector<ea::string>& names = GetOutput(root)->GetMaterialsAttr().names_;
        for (const char* name : materialNames)
            CHECK(ea::find(names.begin(), names.end(), name) != names.end());
        CHECK(!root->GetLastBuildFailed());
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("CsgRingBrush generates the model fast", "[root][procedural]")
{
    auto context = MakeCsgTestContext();
    const auto measure = [&](const CsgRingParams& params)
    {
        // The best of several runs. The limits are generous, so a loaded CI machine does not fail.
        double best = 1e9;
        for (unsigned run = 0; run < 5; ++run)
        {
            const auto start = std::chrono::steady_clock::now();
            CsgShapeMesh mesh;
            const bool generated = CsgGenerateRing(params, mesh);
            SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
            const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
            REQUIRE(generated);
            REQUIRE(model);
            best = ea::min(best, elapsed.count());
        }
        return best;
    };

    const double defaultMs = measure(CsgRingParams{});
    CsgRingParams fine;
    fine.segments_ = 128;
    fine.tubeSegments_ = 64;
    const double fineMs = measure(fine);
    WARN("Ring generation and model build: default " << defaultMs << " ms, 128x64 " << fineMs << " ms");
    // A sanitizer build is much slower, so it gets a wider limit.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
    const double slowFactor = 10.0;
#else
    const double slowFactor = 1.0;
#endif
    CHECK(defaultMs < 15.0 * slowFactor);
    CHECK(fineMs < 150.0 * slowFactor);
}
