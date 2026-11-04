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

#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>

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
