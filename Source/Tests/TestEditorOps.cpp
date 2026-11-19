// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Headless tests for the editor scene functions. The test uses no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSGEditor/CsgEditorOps.h"

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
#include <Urho3D/IO/VirtualFileSystem.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>

#include <catch2/catch_amalgamated.hpp>

using namespace Urho3D;

namespace
{

SharedPtr<Context> MakeContext(bool withCsg)
{
    auto context = MakeShared<Context>();
    context->RegisterSubsystem<Log>();
    context->RegisterSubsystem<Time>();
    context->RegisterSubsystem<WorkQueue>();
    context->RegisterSubsystem<VirtualFileSystem>();
    context->RegisterSubsystem<ResourceCache>();
    RegisterSceneLibrary(context);
    RegisterGraphicsLibrary(context);
    if (withCsg)
        RegisterCsgLibrary(context);
    return context;
}

/// Add a box model to the resource cache.
void AddBox(Context* context, const ea::string& name, const Vector3& halfSize)
{
    const Vector3 normals[6] = {Vector3::RIGHT, Vector3::LEFT, Vector3::UP, Vector3::DOWN, Vector3::FORWARD,
        Vector3::BACK};

    GeometryLODView lod;
    lod.primitiveType_ = TRIANGLE_LIST;
    lod.vertexFormat_.position_ = TYPE_VECTOR3;
    lod.vertexFormat_.normal_ = TYPE_VECTOR3;

    for (const Vector3& normal : normals)
    {
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
        const Vector3 p0 = lod.vertices_[first].GetPosition();
        const Vector3 p1 = lod.vertices_[first + 1].GetPosition();
        const Vector3 p2 = lod.vertices_[first + 2].GetPosition();
        const bool flip = (p1 - p0).CrossProduct(p2 - p0).DotProduct(normal) < 0.0f;
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
}

void RunFrames(Scene* scene, unsigned frames)
{
    auto* time = scene->GetSubsystem<Time>();
    auto* workQueue = scene->GetSubsystem<WorkQueue>();
    for (unsigned i = 0; i < frames; ++i)
    {
        workQueue->CompleteAll();
        time->BeginFrame(1.0f / 60.0f);
        scene->Update(1.0f / 60.0f);
        time->EndFrame();
    }
}

struct Fixture
{
    Fixture()
        : context(MakeContext(true))
        , scene(MakeShared<Scene>(context))
    {
        AddBox(context, "Csg/BoxA.mdl", Vector3::ONE);
        AddBox(context, "Csg/BoxB.mdl", Vector3::ONE);
        scene->CreateComponent<Octree>();
    }

    /// Create a root node with the box A base model and no delay for the rebuild.
    Node* MakeRoot()
    {
        Node* node = CreateRootNode(scene, Vector3::ZERO);
        REQUIRE(node);
        auto* root = node->GetComponent<CsgRoot>();
        REQUIRE(root);
        root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/BoxA.mdl"));
        root->SetDeferredDelaySec(0.0f);
        return node;
    }

    /// Add the brushes "Grow" and "Cut". The union adds [0, 2] on x, then the difference cuts from x = 1.
    /// The result ends at x = 1. With the swapped order, it ends at x = 2.
    void AddGrowAndCut(Node* rootNode)
    {
        const int unionOp = FindOperation("Union");
        const int diffOp = FindOperation("DifferenceAB");
        Node* grow = AddBrushNode(rootNode, "Csg/BoxB.mdl", unionOp, Vector3(1.0f, 0.0f, 0.0f));
        Node* cut = AddBrushNode(rootNode, "Csg/BoxB.mdl", diffOp, Vector3(2.0f, 0.0f, 0.0f));
        REQUIRE(grow);
        REQUIRE(cut);
        grow->SetName("Grow");
        cut->SetName("Cut");
        cut->SetScale(Vector3(1.0f, 2.0f, 2.0f));
    }

    int FindOperation(const char* name)
    {
        for (int i = 0; i < OperationCount(context); ++i)
        {
            if (OperationName(context, i) == name)
                return i;
        }
        FAIL("Operation not found: " << name);
        return -1;
    }

    float MaxX(Node* rootNode)
    {
        RunFrames(scene, 60);
        auto* output = rootNode->GetComponent<CsgRoot>()->GetResultModel();
        REQUIRE(output);
        REQUIRE(output->GetModel());
        return output->GetModel()->GetBoundingBox().max_.x_;
    }

    SharedPtr<Context> context;
    SharedPtr<Scene> scene;
};

} // namespace

TEST_CASE("FindActiveRoot maps a node to its root", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    f.AddGrowAndCut(rootNode);
    Node* brush = rootNode->GetChild("Grow");
    Node* other = f.scene->CreateChild("Other");
    Node* deep = brush->CreateChild("Deep");

    CHECK(FindActiveRoot(rootNode) == rootNode);
    CHECK(FindActiveRoot(brush) == rootNode);
    CHECK(FindActiveRoot(other) == nullptr);
    CHECK(FindActiveRoot(deep) == nullptr);
    CHECK(FindActiveRoot(nullptr) == nullptr);

    RunFrames(f.scene, 60);
    Node* result = rootNode->GetChild("CsgResult");
    REQUIRE(result);
    REQUIRE(result->IsTemporary());
    CHECK(FindActiveRoot(result) == rootNode);

    CHECK(IsRootNode(rootNode));
    CHECK_FALSE(IsRootNode(brush));
    CHECK(IsBrushNode(brush));
    CHECK_FALSE(IsBrushNode(rootNode));
}

TEST_CASE("AddBrushNode appends and keeps the world position", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    rootNode->SetPosition(Vector3(5.0f, -2.0f, 3.0f));
    rootNode->SetRotation(Quaternion(40.0f, Vector3(1.0f, 2.0f, 0.5f)));
    rootNode->SetScale(Vector3(2.0f, 0.5f, 3.0f));

    const int diff = f.FindOperation("DifferenceAB");
    Node* first = AddBrushNode(rootNode, "Csg/BoxB.mdl", diff, Vector3(1.0f, 2.0f, 3.0f));
    Node* second = AddBrushNode(rootNode, "", 0, Vector3(-4.0f, 0.5f, 1.0f));
    REQUIRE(first);
    REQUIRE(second);
    CHECK(rootNode->GetChildren().back() == second);
    CHECK(first->GetParent() == rootNode);
    CHECK(first->GetName() == "Brush");
    CHECK(first->GetWorldPosition().Equals(Vector3(1.0f, 2.0f, 3.0f), 1e-3f));
    CHECK(second->GetWorldPosition().Equals(Vector3(-4.0f, 0.5f, 1.0f), 1e-3f));
    CHECK(GetOperation(first) == diff);
    CHECK(GetOperation(second) == 0);
    CHECK(first->GetComponent(StringHash("CsgBrush"))->GetAttribute("Model").GetResourceRef().name_ ==
        "Csg/BoxB.mdl");
    CHECK(second->GetComponent(StringHash("CsgBrush"))->GetAttribute("Model").GetResourceRef().name_.empty());

    Node* plain = f.scene->CreateChild("Plain");
    CHECK(AddBrushNode(plain, "", 0, Vector3::ZERO) == nullptr);
    CHECK(plain->GetNumChildren() == 0);
}

TEST_CASE("AddBrushNode gives the brush a world scale of 1", "[editorops]")
{
    Fixture f;
    AddBox(f.context, "Csg/Unit.mdl", Vector3::ONE * 0.5f);
    auto* model = f.context->GetSubsystem<ResourceCache>()->GetResource<Model>("Csg/Unit.mdl");
    REQUIRE(model);

    const auto worldBoxSize = [&](Node* node) { return model->GetBoundingBox().Transformed(node->GetWorldTransform()).Size(); };

    SECTION("Root with no rotation")
    {
        Node* rootNode = f.MakeRoot();
        rootNode->SetScale(Vector3(72.0f, 1.0f, 82.0f));
        Node* brush = AddBrushNode(rootNode, "Csg/Unit.mdl", 0, Vector3(3.0f, 1.0f, -2.0f));
        REQUIRE(brush);
        CHECK(brush->GetWorldPosition().Equals(Vector3(3.0f, 1.0f, -2.0f), 1e-3f));
        CHECK(brush->GetWorldScale().Equals(Vector3::ONE, 1e-4f));
        CHECK(worldBoxSize(brush).Equals(Vector3::ONE, 1e-4f));
    }

    SECTION("Root with a rotation")
    {
        Node* rootNode = f.MakeRoot();
        rootNode->SetScale(Vector3(72.0f, 1.0f, 82.0f));
        rootNode->SetRotation(Quaternion(90.0f, Vector3::UP));
        Node* brush = AddBrushNode(rootNode, "Csg/Unit.mdl", 0, Vector3(3.0f, 1.0f, -2.0f));
        REQUIRE(brush);
        CHECK(brush->GetWorldPosition().Equals(Vector3(3.0f, 1.0f, -2.0f), 1e-3f));
        CHECK(brush->GetWorldScale().Equals(Vector3::ONE, 1e-4f));
        CHECK(brush->GetWorldRotation().Equivalent(rootNode->GetWorldRotation(), 1e-4f));
        CHECK(worldBoxSize(brush).Equals(Vector3::ONE, 1e-3f));
    }

    SECTION("Root with a negative scale")
    {
        Node* rootNode = f.MakeRoot();
        rootNode->SetScale(Vector3(-72.0f, 1.0f, -82.0f));
        Node* brush = AddBrushNode(rootNode, "Csg/Unit.mdl", 0, Vector3(3.0f, 1.0f, -2.0f));
        REQUIRE(brush);
        CHECK(brush->GetWorldPosition().Equals(Vector3(3.0f, 1.0f, -2.0f), 1e-3f));
        CHECK(worldBoxSize(brush).Equals(Vector3::ONE, 1e-4f));
    }

    SECTION("Root under a scaled parent")
    {
        Node* parent = f.scene->CreateChild("Parent");
        parent->SetScale(Vector3(2.0f, 3.0f, 0.5f));
        Node* rootNode = CreateRootNode(parent, Vector3(1.0f, 2.0f, 3.0f));
        REQUIRE(rootNode);
        rootNode->SetScale(Vector3(72.0f, 1.0f, 82.0f));
        Node* brush = AddBrushNode(rootNode, "Csg/Unit.mdl", 0, Vector3(3.0f, 1.0f, -2.0f));
        REQUIRE(brush);
        CHECK(brush->GetWorldPosition().Equals(Vector3(3.0f, 1.0f, -2.0f), 1e-3f));
        CHECK(worldBoxSize(brush).Equals(Vector3::ONE, 1e-4f));
    }

    SECTION("Root with a zero scale component")
    {
        Node* rootNode = f.MakeRoot();
        rootNode->SetScale(Vector3(2.0f, 0.0f, 4.0f));
        Node* brush = AddBrushNode(rootNode, "Csg/Unit.mdl", 0, Vector3::ZERO);
        REQUIRE(brush);
        CHECK(brush->GetScale().Equals(Vector3(0.5f, 1.0f, 0.25f), 1e-6f));
    }
}

TEST_CASE("AddBrushNode with enabled false does not request a rebuild", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    auto* root = rootNode->GetComponent<CsgRoot>();
    RunFrames(f.scene, 5);
    const unsigned revision = root->GetRebuildRevision();

    Node* brush = AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3(1.0f, 0.0f, 0.0f), false);
    REQUIRE(brush);
    CHECK_FALSE(brush->GetComponent(StringHash("CsgBrush"))->IsEnabled());
    CHECK(root->GetRebuildRevision() == revision);
    CHECK(GetOperands(rootNode).empty());

    // A disabled brush can move without a rebuild.
    brush->SetWorldPosition(Vector3(2.0f, 0.0f, 0.0f));
    RunFrames(f.scene, 2);
    CHECK(root->GetRebuildRevision() == revision);

    REQUIRE(SetBrushEnabled(brush, true));
    CHECK(root->GetRebuildRevision() != revision);
    CHECK(GetOperands(rootNode).size() == 1);
}

TEST_CASE("Operation functions use the attribute enum names", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    Node* brush = AddBrushNode(rootNode, "", 0, Vector3::ZERO);
    REQUIRE(brush);

    REQUIRE(OperationCount(f.context) == 3);
    CHECK(OperationName(f.context, 0) == "Union");
    CHECK(OperationName(f.context, 1) == "DifferenceAB");
    CHECK(OperationName(f.context, 2) == "Intersection");
    CHECK(OperationName(f.context, 3).empty());
    CHECK(OperationName(f.context, -1).empty());

    CHECK(SetOperation(brush, 2));
    CHECK(GetOperation(brush) == 2);
    CHECK_FALSE(SetOperation(brush, 3));
    CHECK_FALSE(SetOperation(brush, -1));
    CHECK(GetOperation(brush) == 2);

    CHECK(CycleOperation(f.context, 0, 1) == 1);
    CHECK(CycleOperation(f.context, 2, 1) == 0);
    CHECK(CycleOperation(f.context, 0, -1) == 2);
    CHECK(CycleOperation(f.context, 1, 3) == 1);
}

TEST_CASE("SetBrushModel changes the model of the brush", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    Node* brush = AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3::ZERO);
    REQUIRE(brush);
    Component* component = brush->GetComponent(StringHash("CsgBrush"));

    CHECK(SetBrushModel(brush, "Csg/BoxA.mdl"));
    CHECK(component->GetAttribute("Model").GetResourceRef().name_ == "Csg/BoxA.mdl");
    CHECK(SetBrushModel(brush, ""));
    CHECK(component->GetAttribute("Model").GetResourceRef().name_.empty());
    CHECK_FALSE(SetBrushModel(rootNode, "Csg/BoxA.mdl"));
}

TEST_CASE("GetOperands lists the enabled brushes in child order", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    Node* a = AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3::ZERO);
    Node* plain = rootNode->CreateChild("Plain");
    Node* b = AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3::ZERO);
    Node* c = AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3::ZERO);
    Node* nested = c->CreateChild("Nested");
    AddBrushNode(c, "Csg/BoxB.mdl", 0, Vector3::ZERO);
    (void)nested;

    CHECK(GetOperands(rootNode) == ea::vector<Node*>{a, b, c});
    CHECK(GetAllBrushChildren(rootNode) == ea::vector<Node*>{a, b, c});

    REQUIRE(ToggleBrushEnabled(b));
    CHECK(GetOperands(rootNode) == ea::vector<Node*>{a, c});
    CHECK(GetAllBrushChildren(rootNode) == ea::vector<Node*>{a, b, c});
    (void)plain;

    CHECK(GetOperands(plain).empty());
    CHECK(GetOperands(nullptr).empty());
}

TEST_CASE("MoveOperand changes the result and moves only among brushes", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    Node* plain = rootNode->CreateChild("Plain");
    f.AddGrowAndCut(rootNode);
    Node* grow = rootNode->GetChild("Grow");
    Node* cut = rootNode->GetChild("Cut");
    REQUIRE(f.MaxX(rootNode) == Catch::Approx(1.0f).margin(1e-3f));

    // Child order: Plain, Grow, Cut. The brush indices are 0 and 1.
    CHECK_FALSE(MoveOperand(rootNode, 0, 2));
    CHECK_FALSE(MoveOperand(rootNode, 2, 0));
    CHECK_FALSE(MoveOperand(plain, 0, 1));

    REQUIRE(MoveOperand(rootNode, 1, 0));
    CHECK(rootNode->GetChildren()[0] == plain);
    CHECK(rootNode->GetChildren()[1] == cut);
    CHECK(rootNode->GetChildren()[2] == grow);
    CHECK(f.MaxX(rootNode) == Catch::Approx(2.0f).margin(1e-3f));

    REQUIRE(MoveOperand(rootNode, 0, 1));
    CHECK(rootNode->GetChildren()[0] == plain);
    CHECK(rootNode->GetChildren()[1] == grow);
    CHECK(rootNode->GetChildren()[2] == cut);
    CHECK(f.MaxX(rootNode) == Catch::Approx(1.0f).margin(1e-3f));
}

TEST_CASE("ToggleBrushEnabled changes the result", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    f.AddGrowAndCut(rootNode);
    Node* cut = rootNode->GetChild("Cut");
    REQUIRE(f.MaxX(rootNode) == Catch::Approx(1.0f).margin(1e-3f));

    REQUIRE(ToggleBrushEnabled(cut));
    CHECK(f.MaxX(rootNode) == Catch::Approx(2.0f).margin(1e-3f));
    REQUIRE(ToggleBrushEnabled(cut));
    CHECK(f.MaxX(rootNode) == Catch::Approx(1.0f).margin(1e-3f));
}

TEST_CASE("Editor functions fail without CSG", "[editorops]")
{
    SECTION("A node without CSG")
    {
        Fixture f;
        Node* plain = f.scene->CreateChild("Plain");
        CHECK(FindActiveRoot(plain) == nullptr);
        CHECK_FALSE(IsBrushNode(plain));
        CHECK_FALSE(IsRootNode(plain));
        CHECK(AddBrushNode(plain, "", 0, Vector3::ZERO) == nullptr);
        CHECK(GetOperation(plain) == -1);
        CHECK_FALSE(SetOperation(plain, 0));
        CHECK_FALSE(SetBrushModel(plain, ""));
        CHECK(GetOperands(plain).empty());
        CHECK_FALSE(MoveOperand(plain, 0, 0));
        CHECK_FALSE(ToggleBrushEnabled(plain));
        CHECK(DescribeRoot(plain).empty());
    }

    SECTION("The CSG types are not registered")
    {
        auto context = MakeContext(false);
        auto scene = MakeShared<Scene>(context);
        Node* node = scene->CreateChild("Node");
        CHECK(CreateRootNode(scene, Vector3::ZERO) == nullptr);
        CHECK(scene->GetNumChildren() == 1);
        CHECK(FindActiveRoot(node) == nullptr);
        CHECK(AddBrushNode(node, "", 0, Vector3::ZERO) == nullptr);
        CHECK(OperationCount(context) == 0);
        CHECK(OperationName(context, 0).empty());
    }
}

TEST_CASE("DescribeRoot lists the operands and the build status", "[editorops]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    f.AddGrowAndCut(rootNode);
    Node* extra = AddBrushNode(rootNode, "", 0, Vector3::ZERO);
    extra->SetName("Off");
    ToggleBrushEnabled(extra);
    RunFrames(f.scene, 60);

    const ea::string text = DescribeRoot(rootNode);
    CHECK(text.starts_with("0: Grow | Union | Csg/BoxB.mdl\n1: Cut | DifferenceAB | Csg/BoxB.mdl\n"));
    CHECK(text.find("Off") == ea::string::npos);
    const ea::string status = rootNode->GetComponent<CsgRoot>()->GetBuildStatus();
    CHECK_FALSE(status.empty());
    CHECK(text.find(status) != ea::string::npos);
}
