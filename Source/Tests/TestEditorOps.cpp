// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Headless tests for the editor scene functions. The test uses no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoxBrush.h"
#include "Urho3D/CSG/CsgCylinderBrush.h"
#include "Urho3D/CSG/CsgRingBrush.h"
#include "Urho3D/CSG/CsgSphereBrush.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSGEditor/CsgEditorOps.h"
#include "Urho3D/CSGEditor/CsgShapeEditOps.h"
#include "TestGeneratedBrush.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Timer.h>
#include <Urho3D/Core/WorkQueue.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/IOEvents.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/IO/VirtualFileSystem.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>

#include <catch2/catch_amalgamated.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include <chrono>

using namespace Urho3D;

namespace
{

SharedPtr<Context> MakeContext(bool withCsg)
{
    auto context = MakeShared<Context>();
    context->RegisterSubsystem<Log>();
    context->RegisterSubsystem<Time>();
    context->RegisterSubsystem<WorkQueue>();
    context->RegisterSubsystem<FileSystem>();
    context->RegisterSubsystem<VirtualFileSystem>();
    context->RegisterSubsystem<ResourceCache>();
    RegisterSceneLibrary(context);
    RegisterGraphicsLibrary(context);
    if (withCsg)
    {
        RegisterCsgLibrary(context);
        TestGeneratedBrush::RegisterObject(context);
    }
    return context;
}

/// Add a box model to the resource cache.
void AddBox(Context* context, const ea::string& name, const Vector3& halfSize)
{
    SharedPtr<Model> model = MakeTestBoxModel(context, halfSize);
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
    Component* component = FindBrushComponent(brush);

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

namespace
{

/// Count the error messages that the engine logs.
class EditorErrorCounter : public Object
{
    URHO3D_OBJECT(EditorErrorCounter, Object);

public:
    explicit EditorErrorCounter(Context* context)
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

/// Add the brushes "Grow" and "Cut" as in AddGrowAndCut. "Grow" has a generated model.
void AddGeneratedGrowAndCut(Fixture& f, Node* rootNode)
{
    Node* grow = rootNode->CreateChild("Grow");
    grow->SetPosition(Vector3(1.0f, 0.0f, 0.0f));
    auto* brush = grow->CreateComponent<TestGeneratedBrush>();
    brush->SetGeneratedModel(MakeTestBoxModel(f.context, Vector3::ONE));
    brush->SetOperation(CsgOperation::Union);

    Node* cut = AddBrushNode(rootNode, "Csg/BoxB.mdl", f.FindOperation("DifferenceAB"), Vector3(2.0f, 0.0f, 0.0f));
    REQUIRE(cut);
    cut->SetName("Cut");
    cut->SetScale(Vector3(1.0f, 2.0f, 2.0f));
}

} // namespace

TEST_CASE("Editor functions find a brush of a derived type", "[editorops][generated]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    Node* rootNode = f.MakeRoot();
    Node* plain = rootNode->CreateChild("Plain");
    AddGeneratedGrowAndCut(f, rootNode);
    Node* grow = rootNode->GetChild("Grow");
    Node* cut = rootNode->GetChild("Cut");
    REQUIRE(f.MaxX(rootNode) == Catch::Approx(1.0f).margin(1e-3f));

    CHECK(IsBrushNode(grow));
    CHECK(FindBrushComponent(grow) == grow->GetComponent<TestGeneratedBrush>());
    CHECK(FindBrushComponent(plain) == nullptr);
    CHECK(FindActiveRoot(grow) == rootNode);
    CHECK(GetOperands(rootNode) == ea::vector<Node*>{grow, cut});
    CHECK(GetAllBrushChildren(rootNode) == ea::vector<Node*>{grow, cut});

    SECTION("Operation")
    {
        CHECK(GetOperation(grow) == f.FindOperation("Union"));
        REQUIRE(SetOperation(grow, f.FindOperation("Intersection")));
        CHECK(GetOperation(grow) == f.FindOperation("Intersection"));
        CHECK(grow->GetComponent<TestGeneratedBrush>()->GetOperation() == CsgOperation::Intersection);
    }

    SECTION("Enabled state")
    {
        REQUIRE(ToggleBrushEnabled(grow));
        CHECK(GetOperands(rootNode) == ea::vector<Node*>{cut});
        CHECK(f.MaxX(rootNode) == Catch::Approx(1.0f).margin(1e-3f));
        REQUIRE(ToggleBrushEnabled(grow));
        CHECK(GetOperands(rootNode) == ea::vector<Node*>{grow, cut});
    }

    SECTION("Move")
    {
        // The swapped order cuts first and then grows. The result ends at x = 2.
        REQUIRE(MoveOperand(rootNode, 1, 0));
        CHECK(rootNode->GetChildren()[1] == cut);
        CHECK(rootNode->GetChildren()[2] == grow);
        CHECK(f.MaxX(rootNode) == Catch::Approx(2.0f).margin(1e-3f));
    }

    SECTION("Model")
    {
        // A brush with no "Model" attribute is not a target for a model. The refusal logs no error.
        CHECK_FALSE(HasModelAttribute(FindBrushComponent(grow)));
        CHECK_FALSE(IsModelBrushNode(grow));
        CHECK(IsModelBrushNode(cut));
        CHECK_FALSE(SetBrushModel(grow, "Csg/BoxA.mdl"));
        CHECK(SetBrushModel(cut, "Csg/BoxB.mdl"));
        CHECK(DescribeRoot(rootNode).find("TestGeneratedBrush") != ea::string::npos);
    }

    CHECK(errors.GetCount() == 0);
}

TEST_CASE("Editor functions treat a procedural box as a brush without a model", "[editorops][procedural]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    Node* rootNode = f.MakeRoot();
    Node* boxNode = rootNode->CreateChild("Box");
    auto* box = boxNode->CreateComponent<CsgBoxBrush>();

    CHECK(IsBrushNode(boxNode));
    CHECK(FindBrushComponent(boxNode) == box);
    CHECK(GetOperands(rootNode) == ea::vector<Node*>{boxNode});
    CHECK_FALSE(HasModelAttribute(box));
    CHECK_FALSE(IsModelBrushNode(boxNode));
    // The drop of a model does not replace the shape of a procedural brush.
    CHECK_FALSE(SetBrushModel(boxNode, "Csg/BoxA.mdl"));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("AddBrushNodeOfType creates a procedural box as the last operand", "[editorops][procedural]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    Node* rootNode = f.MakeRoot();
    rootNode->SetScale(Vector3(2.0f, 4.0f, 0.5f));
    AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3::ZERO);

    const int diff = f.FindOperation("DifferenceAB");
    Node* boxNode = AddBrushNodeOfType(rootNode, "CsgBoxBrush", diff, Vector3(1.0f, 2.0f, 3.0f));
    REQUIRE(boxNode);
    CHECK(rootNode->GetChildren().back() == boxNode);
    auto* box = boxNode->GetComponent<CsgBoxBrush>();
    REQUIRE(box);
    CHECK(GetOperation(boxNode) == diff);
    CHECK(GetOperands(rootNode).back() == boxNode);
    CHECK(boxNode->GetWorldPosition().Equals(Vector3(1.0f, 2.0f, 3.0f), 1e-3f));
    CHECK(boxNode->GetScale().Equals(Vector3(0.5f, 0.25f, 2.0f), 1e-5f));
    CHECK(boxNode->GetWorldScale().Equals(Vector3::ONE, 1e-5f));
    CHECK(DescribeRoot(rootNode).find("CsgBoxBrush") != ea::string::npos);

    // A disabled brush is not an operand.
    Node* disabled = AddBrushNodeOfType(rootNode, "CsgBoxBrush", 0, Vector3::ZERO, false);
    REQUIRE(disabled);
    CHECK(GetOperands(rootNode).back() == boxNode);

    // An unknown type and a type that is not a brush give null with no error log.
    const unsigned numChildren = rootNode->GetNumChildren();
    CHECK(AddBrushNodeOfType(rootNode, "NoSuchBrush", 0, Vector3::ZERO) == nullptr);
    CHECK(AddBrushNodeOfType(rootNode, "StaticModel", 0, Vector3::ZERO) == nullptr);
    CHECK(AddBrushNodeOfType(rootNode, "CsgProceduralBrush", 0, Vector3::ZERO) == nullptr);
    CHECK(AddBrushNodeOfType(rootNode, "", 0, Vector3::ZERO) == nullptr);
    CHECK(rootNode->GetNumChildren() == numChildren);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("The brush presets have the procedural box and the model presets", "[editorops][procedural]")
{
    Fixture f;
    const ea::span<const CsgBrushPreset> presets = GetBrushPresets();
    ea::vector<ea::string> names;
    for (const CsgBrushPreset& preset : presets)
    {
        names.push_back(preset.name_);
        // A preset has a model or a type, not both.
        CHECK((preset.model_ == nullptr) != (preset.typeName_ == nullptr));
        CHECK(IsBrushPresetAvailable(f.context, preset));
    }
    CHECK(names == ea::vector<ea::string>{
        "Box", "Sphere", "Cylinder", "Ring", "Box.mdl", "Sphere.mdl", "Cylinder.mdl", "Cone.mdl", "Pyramid.mdl", "Torus.mdl"});
    CHECK(ea::string(presets[0].typeName_) == "CsgBoxBrush");

    Node* rootNode = f.MakeRoot();
    Node* box = AddPresetBrushNode(rootNode, presets[0], 0, Vector3::ZERO);
    REQUIRE(box);
    CHECK(box->GetComponent<CsgBoxBrush>());
    CHECK(ea::string(presets[1].name_) == "Sphere");
    CHECK(ea::string(presets[1].typeName_) == "CsgSphereBrush");
    Node* sphere = AddPresetBrushNode(rootNode, presets[1], 0, Vector3::ZERO);
    REQUIRE(sphere);
    CHECK(sphere->GetComponent<CsgSphereBrush>());
    CHECK(DescribeRoot(rootNode).find("CsgSphereBrush") != ea::string::npos);
    CHECK(ea::string(presets[2].name_) == "Cylinder");
    CHECK(ea::string(presets[2].typeName_) == "CsgCylinderBrush");
    Node* cylinder = AddPresetBrushNode(rootNode, presets[2], 0, Vector3::ZERO);
    REQUIRE(cylinder);
    CHECK(cylinder->GetComponent<CsgCylinderBrush>());
    CHECK(DescribeRoot(rootNode).find("CsgCylinderBrush") != ea::string::npos);
    CHECK(ea::string(presets[3].name_) == "Ring");
    CHECK(ea::string(presets[3].typeName_) == "CsgRingBrush");
    Node* ring = AddPresetBrushNode(rootNode, presets[3], 0, Vector3::ZERO);
    REQUIRE(ring);
    CHECK(ring->GetComponent<CsgRingBrush>());
    CHECK(DescribeRoot(rootNode).find("CsgRingBrush") != ea::string::npos);
    Node* model = AddPresetBrushNode(rootNode, presets[4], 0, Vector3::ZERO);
    REQUIRE(model);
    CHECK(model->GetComponent<CsgBrush>()->GetModelAttr().name_ == "Models/Box.mdl");
}

TEST_CASE("No brush preset is available without CSG", "[editorops][procedural]")
{
    auto plain = MakeContext(false);
    for (const CsgBrushPreset& preset : GetBrushPresets())
        CHECK_FALSE(IsBrushPresetAvailable(plain, preset));
}

namespace
{

ea::vector<ea::string> GetMaterialNames(Node* brushNode)
{
    return FindBrushComponent(brushNode)->GetAttribute("Materials").GetResourceRefList().names_;
}

/// A temporary directory that the virtual file system mounts. The destructor removes it.
struct TempResourceDir
{
    explicit TempResourceDir(Context* context)
    {
        static unsigned counter = 0;
        path_ = std::filesystem::temp_directory_path()
            / ("rbfx-csg-material-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" + std::to_string(counter++));
        std::filesystem::create_directories(path_ / "Textures");
        context->GetSubsystem<VirtualFileSystem>()->MountDir(ea::string(path_.string().c_str()) + "/");
    }
    ~TempResourceDir()
    {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    void WriteFile(const char* relativeName, const char* text) const
    {
        std::ofstream stream(path_ / relativeName, std::ios::binary);
        stream << text;
    }

    std::string ReadFile(const char* relativeName) const
    {
        std::ifstream stream(path_ / relativeName, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
    }

    std::filesystem::path path_;
};

} // namespace

TEST_CASE("SetBrushMaterial sets one slot of a procedural brush and keeps the others", "[editorops][material]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    Node* rootNode = f.MakeRoot();
    Node* boxNode = AddBrushNodeOfType(rootNode, "CsgBoxBrush", 0, Vector3::ZERO);
    REQUIRE(boxNode);
    REQUIRE(GetMaterialNames(boxNode).size() == 6);

    CHECK(SetBrushMaterial(boxNode, "Materials/B.xml", 3));
    CHECK(SetBrushMaterial(boxNode, "Materials/A.xml", 0));
    const ea::vector<ea::string> names = GetMaterialNames(boxNode);
    REQUIRE(names.size() == 6);
    CHECK(names[0] == "Materials/A.xml");
    CHECK(names[3] == "Materials/B.xml");
    CHECK(names[1].empty());
    CHECK(names[5].empty());
    CHECK(FindBrushComponent(boxNode)->GetAttribute("Materials").GetResourceRefList().type_ == Material::GetTypeStatic());

    CHECK(SetAllBrushMaterialSlots(boxNode, "Materials/All.xml"));
    for (const ea::string& name : GetMaterialNames(boxNode))
        CHECK(name == "Materials/All.xml");
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("SetBrushMaterial works on a model brush", "[editorops][material]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    Node* rootNode = f.MakeRoot();
    Node* brushNode = AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3::ZERO);
    REQUIRE(brushNode);
    Component* brush = FindBrushComponent(brushNode);
    brush->SetAttribute("Materials", ResourceRefList(Material::GetTypeStatic(), {"Materials/Old0.xml", "Materials/Old1.xml", ""}));
    brush->ApplyAttributes();
    const unsigned numSlots = GetMaterialNames(brushNode).size();
    // The test brush model has the geometries that the check of slot 1 needs.
    REQUIRE(numSlots >= 2);

    CHECK(SetBrushMaterial(brushNode, "Materials/New.xml", 0));
    CHECK(GetMaterialNames(brushNode)[0] == "Materials/New.xml");
    CHECK(GetMaterialNames(brushNode)[1] == "Materials/Old1.xml");

    CHECK(SetAllBrushMaterialSlots(brushNode, "Materials/All.xml"));
    CHECK(GetMaterialNames(brushNode).size() == numSlots);
    for (const ea::string& name : GetMaterialNames(brushNode))
        CHECK(name == "Materials/All.xml");
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("SetBrushMaterial rejects a slot out of range and a node without a brush", "[editorops][material]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    Node* rootNode = f.MakeRoot();
    Node* boxNode = AddBrushNodeOfType(rootNode, "CsgBoxBrush", 0, Vector3::ZERO);
    REQUIRE(boxNode);
    const ea::vector<ea::string> before = GetMaterialNames(boxNode);

    CHECK_FALSE(SetBrushMaterial(boxNode, "Materials/A.xml", 6));
    CHECK_FALSE(SetBrushMaterial(boxNode, "Materials/A.xml", 1000));
    CHECK(GetMaterialNames(boxNode) == before);

    Node* plain = f.scene->CreateChild("Plain");
    CHECK_FALSE(SetBrushMaterial(plain, "Materials/A.xml", 0));
    CHECK_FALSE(SetAllBrushMaterialSlots(plain, "Materials/A.xml"));
    CHECK_FALSE(SetBrushMaterial(rootNode, "Materials/A.xml", 0));
    CHECK_FALSE(SetAllBrushMaterialSlots(rootNode, "Materials/A.xml"));
    CHECK_FALSE(SetBrushMaterial(nullptr, "Materials/A.xml", 0));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("A material of a brush reaches the CSG result", "[editorops][material]")
{
    Fixture f;
    // The fold uses only materials that the cache has.
    auto material = MakeShared<Material>(f.context);
    material->SetName("Materials/Brick.xml");
    f.context->GetSubsystem<ResourceCache>()->AddManualResource(material);

    Node* rootNode = f.MakeRoot();
    Node* boxNode = AddBrushNodeOfType(rootNode, "CsgBoxBrush", 0, Vector3(0.0f, 5.0f, 0.0f));
    REQUIRE(boxNode);
    CHECK(SetAllBrushMaterialSlots(boxNode, "Materials/Brick.xml"));
    RunFrames(f.scene, 60);

    const ea::vector<ea::string>& names = rootNode->GetComponent<CsgRoot>()->GetResultModel()->GetMaterialsAttr().names_;
    CHECK(ea::find(names.begin(), names.end(), ea::string("Materials/Brick.xml")) != names.end());
}

TEST_CASE("Texture names and material names", "[editorops][material]")
{
    for (const char* name : {"Textures/A.png", "a/b.JPG", "x.jpeg", "x.tga", "x.dds", "x.bmp", "x.webp", "x.ktx"})
        CHECK(IsTextureResourceName(name));
    for (const char* name : {"Textures/A.material", "x.mdl", "x", "x.png.txt", ""})
        CHECK_FALSE(IsTextureResourceName(name));
    CHECK(GetTextureMaterialName("Textures/Wall/Brick.png") == "Textures/Wall/Brick.png.material");
}

TEST_CASE("EnsureTextureMaterial writes the material next to the texture", "[editorops][material]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    TempResourceDir dir(f.context);
    dir.WriteFile("Textures/Brick.png", "not a real image");

    CHECK(EnsureTextureMaterial(f.context, "Textures/Brick.png") == "Textures/Brick.png.material");
    const std::string text = dir.ReadFile("Textures/Brick.png.material");
    CHECK(text.find("Techniques/LitOpaque.xml") != std::string::npos);
    CHECK(text.find("slot=\"Albedo\"") != std::string::npos);
    CHECK(text.find("name=\"Textures/Brick.png\"") != std::string::npos);
    CHECK(errors.GetCount() == 0);

    // An existing material stays as it is.
    dir.WriteFile("Textures/Brick.png.material", "keep me");
    CHECK(EnsureTextureMaterial(f.context, "Textures/Brick.png") == "Textures/Brick.png.material");
    CHECK(dir.ReadFile("Textures/Brick.png.material") == "keep me");
    CHECK(errors.GetCount() == 0);

    // A texture with the same base name and another extension gets its own material.
    dir.WriteFile("Textures/Brick.jpg", "not a real image");
    CHECK(EnsureTextureMaterial(f.context, "Textures/Brick.jpg") == "Textures/Brick.jpg.material");
    CHECK(dir.ReadFile("Textures/Brick.jpg.material").find("name=\"Textures/Brick.jpg\"") != std::string::npos);
    CHECK(dir.ReadFile("Textures/Brick.png.material") == "keep me");
}

TEST_CASE("EnsureTextureMaterial logs one error for a texture outside a resource directory", "[editorops][material]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    TempResourceDir dir(f.context);

    CHECK(EnsureTextureMaterial(f.context, "Textures/Missing.png").empty());
    CHECK(errors.GetCount() == 1);
    CHECK_FALSE(std::filesystem::exists(dir.path_ / "Textures/Missing.png.material"));
}

TEST_CASE("EnsureTextureMaterial logs one error and leaves no file when the write fails", "[editorops][material]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    TempResourceDir dir(f.context);
    dir.WriteFile("Textures/Brick.png", "not a real image");
    // A directory with the name of the material file stops the write.
    std::filesystem::create_directories(dir.path_ / "Textures/Brick.png.material/sub");

    CHECK(EnsureTextureMaterial(f.context, "Textures/Brick.png").empty());
    CHECK(errors.GetCount() == 1);
    CHECK_FALSE(std::filesystem::is_regular_file(dir.path_ / "Textures/Brick.png.material"));
}

TEST_CASE("DecideDrop maps payload, selection and Shift to an action", "[editorops][material]")
{
    DropSelection none;
    DropSelection model;
    model.activeIsModelBrush_ = model.activeIsBrush_ = model.anySelectedBrush_ = true;
    DropSelection procedural;
    procedural.activeIsBrush_ = procedural.anySelectedBrush_ = true;
    DropSelection root;
    root.activeIsRoot_ = true;
    DropSelection other;
    // The cursor is over the result node. A material drop needs it.
    model.hoveringResult_ = procedural.hoveringResult_ = root.hoveringResult_ = true;

    // Models keep the rules of the model drop. Shift does not matter.
    for (const bool shift : {false, true})
    {
        CHECK(DecideDrop(DropPayloadKind::Model, model, shift).action_ == DropAction::ReplaceModel);
        CHECK(DecideDrop(DropPayloadKind::Model, procedural, shift).action_ == DropAction::None);
        CHECK(DecideDrop(DropPayloadKind::Model, root, shift).action_ == DropAction::NewBrush);
        CHECK(DecideDrop(DropPayloadKind::Model, other, shift).action_ == DropAction::None);
        CHECK(DecideDrop(DropPayloadKind::Model, none, shift).action_ == DropAction::None);
        CHECK(DecideDrop(DropPayloadKind::Other, model, shift).action_ == DropAction::None);
        CHECK(DecideDrop(DropPayloadKind::Other, root, shift).action_ == DropAction::None);
    }

    for (const DropSelection* brush : {&model, &procedural})
    {
        CHECK(DecideDrop(DropPayloadKind::Material, *brush, false).action_ == DropAction::AssignMaterial);
        CHECK_FALSE(DecideDrop(DropPayloadKind::Material, *brush, false).allSlots_);
        CHECK(DecideDrop(DropPayloadKind::Material, *brush, true).allSlots_);
        CHECK(DecideDrop(DropPayloadKind::Texture, *brush, false).action_ == DropAction::AssignTexture);
        CHECK(DecideDrop(DropPayloadKind::Texture, *brush, true).allSlots_);
    }

    // A root with no selected brush shows a hint. Another selection leaves the drop to the stock handlers.
    CHECK(DecideDrop(DropPayloadKind::Material, root, false).action_ == DropAction::HintSelectBrush);
    CHECK(DecideDrop(DropPayloadKind::Texture, root, true).action_ == DropAction::HintSelectBrush);
    CHECK(DecideDrop(DropPayloadKind::Material, other, false).action_ == DropAction::None);
    CHECK(DecideDrop(DropPayloadKind::Texture, none, false).action_ == DropAction::None);

    // A selected brush that is not the active node still counts.
    DropSelection mixed = root;
    mixed.anySelectedBrush_ = true;
    CHECK(DecideDrop(DropPayloadKind::Material, mixed, false).action_ == DropAction::AssignMaterial);
}

TEST_CASE("DecideDrop leaves a material drop away from the result to the stock handler", "[editorops][material]")
{
    DropSelection brush;
    brush.activeIsBrush_ = brush.anySelectedBrush_ = true;
    DropSelection root;
    root.activeIsRoot_ = true;

    for (const bool shift : {false, true})
    {
        CHECK(DecideDrop(DropPayloadKind::Material, brush, shift).action_ == DropAction::None);
        CHECK(DecideDrop(DropPayloadKind::Material, root, shift).action_ == DropAction::None);
        // A texture has no stock handler. The hover does not matter.
        CHECK(DecideDrop(DropPayloadKind::Texture, brush, shift).action_ == DropAction::AssignTexture);
        CHECK(DecideDrop(DropPayloadKind::Texture, root, shift).action_ == DropAction::HintSelectBrush);
    }
}

TEST_CASE("IsResultNodeOfRoot accepts only the temporary result child of the root", "[editorops][material]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    Node* result = rootNode->CreateTemporaryChild("CsgResult");
    Node* named = rootNode->CreateChild("CsgResult");
    Node* other = rootNode->CreateTemporaryChild("Other");
    Node* otherRoot = f.MakeRoot();

    CHECK(IsResultNodeOfRoot(result, rootNode));
    CHECK_FALSE(IsResultNodeOfRoot(named, rootNode));
    CHECK_FALSE(IsResultNodeOfRoot(other, rootNode));
    CHECK_FALSE(IsResultNodeOfRoot(result, otherRoot));
    CHECK_FALSE(IsResultNodeOfRoot(nullptr, rootNode));
}

namespace
{

/// Return the handle with the id, or null.
const ShapeHandle* FindShapeHandle(const ShapeHandleSet& set, unsigned id)
{
    for (const ShapeHandle& handle : set.handles_)
    {
        if (handle.id_ == id)
            return &handle;
    }
    return nullptr;
}

/// Create a root with one procedural brush of the type.
template <class T> T* AddShapeBrush(Fixture& f)
{
    Node* rootNode = f.MakeRoot();
    Node* node = rootNode->CreateChild("Shape");
    return node->CreateComponent<T>();
}

} // namespace

TEST_CASE("QueryShapeHandles and MoveShapeHandles work on a box in a scene", "[editorops][handles]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    auto* box = AddShapeBrush<CsgBoxBrush>(f);

    const ShapeHandleSet set = QueryShapeHandles(box);
    CHECK(set.valid_);
    CHECK(set.handles_.size() == 14);
    REQUIRE(FindShapeHandle(set, 7));
    CHECK(FindShapeHandle(set, 7)->kind_ == CsgShapeHandleKind::Point);
    CHECK(FindShapeHandle(set, 7)->position_.Equals(Vector3(0.5f, 0.5f, 0.5f)));
    REQUIRE(FindShapeHandle(set, 21));
    CHECK(FindShapeHandle(set, 21)->kind_ == CsgShapeHandleKind::Axis);
    CHECK(FindShapeHandle(set, 21)->axis_.Equals(Vector3(1.0f, 0.0f, 0.0f)));
    CHECK_FALSE(FindShapeHandle(set, 7)->name_.empty());

    CHECK(MoveShapeHandles(box, {7}, Vector3(0.25f, 0.0f, 0.0f)));
    CHECK(box->GetAttribute("Corner 7").GetVector3().Equals(Vector3(0.75f, 0.5f, 0.5f)));
    // The -X face through the +X face is not valid. The corners stay.
    const Variant corner0 = box->GetAttribute("Corner 0");
    CHECK_FALSE(MoveShapeHandles(box, {20}, Vector3(3.0f, 0.0f, 0.0f)));
    CHECK(box->GetAttribute("Corner 0") == corner0);
    // An unknown id, no id and a null brush give false.
    CHECK_FALSE(MoveShapeHandles(box, {99}, Vector3::ONE));
    CHECK_FALSE(MoveShapeHandles(box, {}, Vector3::ONE));
    CHECK_FALSE(MoveShapeHandles(nullptr, {0}, Vector3::ONE));
    CHECK_FALSE(QueryShapeHandles(nullptr).valid_);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("QueryShapeHandles and MoveShapeHandles work on a sphere, a cylinder and a ring", "[editorops][handles]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);

    auto* sphere = AddShapeBrush<CsgSphereBrush>(f);
    CHECK(QueryShapeHandles(sphere).handles_.size() == 6);
    CHECK(MoveShapeHandles(sphere, {0}, Vector3(0.25f, 0.0f, 0.0f)));
    CHECK(sphere->GetAttribute("Radius").GetVector3().Equals(Vector3(0.75f, 0.5f, 0.5f)));
    CHECK_FALSE(MoveShapeHandles(sphere, {0}, Vector3(M_INFINITY, 0.0f, 0.0f)));

    auto* cylinder = AddShapeBrush<CsgCylinderBrush>(f);
    CHECK(QueryShapeHandles(cylinder).handles_.size() == 11);
    CHECK(MoveShapeHandles(cylinder, {1}, Vector3(0.0f, 0.5f, 0.0f)));
    CHECK(cylinder->GetAttribute("Top Center").GetVector3().Equals(Vector3(0.0f, 1.0f, 0.0f)));
    // The top center on the bottom center gives no height.
    CHECK_FALSE(MoveShapeHandles(cylinder, {1}, Vector3(0.0f, -1.5f, 0.0f)));

    auto* ring = AddShapeBrush<CsgRingBrush>(f);
    CHECK(QueryShapeHandles(ring).handles_.size() == 4);
    CHECK(MoveShapeHandles(ring, {0}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK(ring->GetAttribute("Radius").GetFloat() == Catch::Approx(0.6f));
    CHECK_FALSE(MoveShapeHandles(ring, {0}, Vector3::ZERO));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("IsShapeBrush finds the procedural brushes only", "[editorops][handles]")
{
    Fixture f;
    CHECK(IsShapeBrush(AddShapeBrush<CsgBoxBrush>(f)));
    CHECK(IsShapeBrush(AddShapeBrush<CsgSphereBrush>(f)));
    CHECK(IsShapeBrush(AddShapeBrush<CsgCylinderBrush>(f)));
    CHECK(IsShapeBrush(AddShapeBrush<CsgRingBrush>(f)));

    Node* rootNode = f.MakeRoot();
    Node* modelNode = AddBrushNode(rootNode, "Csg/BoxB.mdl", 0, Vector3::ZERO);
    REQUIRE(modelNode);
    Component* modelBrush = FindBrushComponent(modelNode);
    REQUIRE(modelBrush);
    CHECK_FALSE(IsShapeBrush(modelBrush));
    CHECK_FALSE(IsShapeBrush(rootNode->GetComponent<CsgRoot>()));
    CHECK_FALSE(IsShapeBrush(nullptr));
    CHECK(FindShapeBrush(modelNode) == nullptr);
    CHECK_FALSE(QueryShapeHandles(modelBrush).valid_);
    CHECK_FALSE(MoveShapeHandles(modelBrush, {0}, Vector3::ONE));
}

TEST_CASE("A shape attribute snapshot restores the attributes exactly", "[editorops][handles]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    auto* box = AddShapeBrush<CsgBoxBrush>(f);
    CHECK(FindShapeBrush(box->GetNode()) == box);

    const ShapeAttributeSnapshot start = SnapshotShapeAttributes(box);
    CHECK(start.size() >= 9);
    // No change: the restore does nothing.
    CHECK_FALSE(RestoreShapeAttributes(box, start));

    REQUIRE(MoveShapeHandles(box, {7}, Vector3(0.25f, 0.5f, 0.0f)));
    REQUIRE(MoveShapeHandles(box, {21}, Vector3(0.5f, 0.0f, 0.0f)));
    const ShapeAttributeSnapshot moved = SnapshotShapeAttributes(box);
    CHECK(moved != start);

    CHECK(RestoreShapeAttributes(box, start));
    CHECK(SnapshotShapeAttributes(box) == start);
    static_cast<Component*>(box)->ApplyAttributes();
    CHECK(SnapshotShapeAttributes(box) == start);

    // A drag restores the snapshot before each move. Two frames equal one move.
    REQUIRE(MoveShapeHandles(box, {7}, Vector3(0.25f, 0.0f, 0.0f)));
    const ShapeAttributeSnapshot once = SnapshotShapeAttributes(box);
    RestoreShapeAttributes(box, start);
    REQUIRE(MoveShapeHandles(box, {7}, Vector3(0.1f, 0.0f, 0.0f)));
    RestoreShapeAttributes(box, start);
    REQUIRE(MoveShapeHandles(box, {7}, Vector3(0.25f, 0.0f, 0.0f)));
    CHECK(SnapshotShapeAttributes(box) == once);
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("The undo logic of a handle drag switches between two snapshots", "[editorops][handles]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    auto* ring = AddShapeBrush<CsgRingBrush>(f);
    ring->SetArc(90.0f);

    const ShapeAttributeSnapshot start = SnapshotShapeAttributes(ring);
    REQUIRE(MoveShapeHandles(ring, {3}, Vector3(-0.1f, 0.0f, 0.0f)));
    const ShapeAttributeSnapshot end = SnapshotShapeAttributes(ring);
    REQUIRE(start != end);

    // Undo.
    CHECK(RestoreShapeAttributes(ring, start));
    CHECK(SnapshotShapeAttributes(ring) == start);
    // Redo.
    CHECK(RestoreShapeAttributes(ring, end));
    CHECK(SnapshotShapeAttributes(ring) == end);
    CHECK_FALSE(RestoreShapeAttributes(ring, end));
    CHECK_FALSE(RestoreShapeAttributes(nullptr, end));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("PickNearestHandle picks the nearest handle in the radius", "[editorops][handles]")
{
    const ea::vector<HandleScreenPoint> points{
        {5, Vector2(100.0f, 100.0f)},
        {2, Vector2(106.0f, 100.0f)},
        {1, Vector2(94.0f, 100.0f)},
        {0, Vector2(101.0f, 100.0f), false},
    };
    CHECK(PickNearestHandle(points, Vector2(99.0f, 100.0f), 10.0f) == 0u);
    CHECK(PickNearestHandle(points, Vector2(105.0f, 100.0f), 10.0f) == 1u);
    // A tie goes to the lower id: id 1 at index 2 wins over id 5 at index 0.
    CHECK(PickNearestHandle(points, Vector2(97.0f, 100.0f), 10.0f) == 2u);
    // Outside the radius.
    CHECK_FALSE(PickNearestHandle(points, Vector2(200.0f, 100.0f), 10.0f));
    CHECK(PickNearestHandle(points, Vector2(100.0f, 110.0f), 10.0f) == 0u);
    CHECK_FALSE(PickNearestHandle(points, Vector2(100.0f, 110.5f), 10.0f));
    // A hidden point is not picked.
    CHECK_FALSE(PickNearestHandle({{0, Vector2::ZERO, false}}, Vector2::ZERO, 10.0f));
    CHECK_FALSE(PickNearestHandle({}, Vector2::ZERO, 10.0f));
}

TEST_CASE("Handle math: centroid, grid snap and axis projection", "[editorops][handles]")
{
    ea::vector<ShapeHandle> handles(3);
    handles[0].id_ = 0;
    handles[0].position_ = Vector3(0.0f, 0.0f, 0.0f);
    handles[1].id_ = 4;
    handles[1].position_ = Vector3(2.0f, 0.0f, 0.0f);
    handles[2].id_ = 9;
    handles[2].position_ = Vector3(0.0f, 4.0f, 0.0f);
    CHECK(ComputeSelectionCentroid(handles, {0, 4}) == Vector3(1.0f, 0.0f, 0.0f));
    CHECK(ComputeSelectionCentroid(handles, {9, 77}) == Vector3(0.0f, 4.0f, 0.0f));
    CHECK_FALSE(ComputeSelectionCentroid(handles, {77}));
    CHECK_FALSE(ComputeSelectionCentroid(handles, {}));

    CHECK(SnapToGrid(0.3f, 0.125f) == Catch::Approx(0.25f));
    CHECK(SnapToGrid(0.33f, 0.125f) == Catch::Approx(0.375f));
    CHECK(SnapToGrid(-0.3f, 0.125f) == Catch::Approx(-0.25f));
    CHECK(SnapToGrid(1.0f, 0.125f) == Catch::Approx(1.0f));
    CHECK(SnapToGrid(0.3f, 0.0f) == 0.3f);
    CHECK(SnapToGrid(0.3f, -1.0f) == 0.3f);

    CHECK(ProjectDeltaOnAxis(Vector3(1.0f, 2.0f, 3.0f), Vector3(0.0f, 1.0f, 0.0f)) == Vector3(0.0f, 2.0f, 0.0f));
    CHECK(ProjectDeltaOnAxis(Vector3(1.0f, 2.0f, 3.0f), Vector3(0.0f, -2.0f, 0.0f)) == Vector3(0.0f, 2.0f, 0.0f));
    CHECK(ProjectDeltaOnAxis(Vector3(1.0f, 1.0f, 0.0f), Vector3(1.0f, -1.0f, 0.0f)).Equals(Vector3::ZERO));
    CHECK(ProjectDeltaOnAxis(Vector3(1.0f, 2.0f, 3.0f), Vector3::ZERO) == Vector3::ZERO);

    // A point snaps each coordinate of the target.
    CHECK(SnapHandleDelta(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.3f, -0.1f, 0.0f), Vector3::ZERO, 0.25f)
              .Equals(Vector3(0.25f, 0.0f, 0.0f)));
    // An axis snaps only the coordinate along the axis. The result is along the axis.
    CHECK(SnapHandleDelta(Vector3(0.1f, 0.5f, 0.0f), Vector3(0.3f, 0.2f, 0.0f), Vector3(0.0f, 1.0f, 0.0f), 0.25f)
              .Equals(Vector3(0.0f, 0.25f, 0.0f)));
    CHECK(SnapHandleDelta(Vector3(-0.5f, 0.0f, 0.0f), Vector3(-0.2f, 0.0f, 0.0f), Vector3(-1.0f, 0.0f, 0.0f), 0.25f)
              .Equals(Vector3(-0.25f, 0.0f, 0.0f)));
    // Step 0 is off.
    CHECK(SnapHandleDelta(Vector3::ONE, Vector3(0.3f, 0.2f, 0.1f), Vector3::ZERO, 0.0f) == Vector3(0.3f, 0.2f, 0.1f));
}
