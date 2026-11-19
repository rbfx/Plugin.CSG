// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Headless tests for the editor scene functions. The test uses no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoxBrush.h"
#include "Urho3D/CSG/CsgCylinderBrush.h"
#include "Urho3D/CSG/CsgRingBrush.h"
#include "Urho3D/CSG/CsgSphereBrush.h"
#include "Urho3D/CSG/CsgArchBrush.h"
#include "Urho3D/CSG/CsgStairsBrush.h"
#include "Urho3D/CSG/CsgWedgeBrush.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSGEditor/CsgEditorOps.h"
#include "Urho3D/CSGEditor/CsgShapeEditOps.h"
#include "TestGeneratedBrush.h"
#include "TestModelMeasure.h"

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
        "Box", "Sphere", "Cylinder", "Ring", "Wedge", "Stairs", "Arch", "Box.mdl", "Sphere.mdl", "Cylinder.mdl", "Cone.mdl", "Pyramid.mdl", "Torus.mdl"});
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
    CHECK(ea::string(presets[4].name_) == "Wedge");
    CHECK(ea::string(presets[4].typeName_) == "CsgWedgeBrush");
    Node* wedge = AddPresetBrushNode(rootNode, presets[4], 0, Vector3::ZERO);
    REQUIRE(wedge);
    CHECK(wedge->GetComponent<CsgWedgeBrush>());
    CHECK(DescribeRoot(rootNode).find("CsgWedgeBrush") != ea::string::npos);
    CHECK(ea::string(presets[5].name_) == "Stairs");
    CHECK(ea::string(presets[5].typeName_) == "CsgStairsBrush");
    Node* stairs = AddPresetBrushNode(rootNode, presets[5], 0, Vector3::ZERO);
    REQUIRE(stairs);
    CHECK(stairs->GetComponent<CsgStairsBrush>());
    CHECK(DescribeRoot(rootNode).find("CsgStairsBrush") != ea::string::npos);
    CHECK(ea::string(presets[6].name_) == "Arch");
    CHECK(ea::string(presets[6].typeName_) == "CsgArchBrush");
    Node* arch = AddPresetBrushNode(rootNode, presets[6], 0, Vector3::ZERO);
    REQUIRE(arch);
    CHECK(arch->GetComponent<CsgArchBrush>());
    CHECK(DescribeRoot(rootNode).find("CsgArchBrush") != ea::string::npos);
    Node* model = AddPresetBrushNode(rootNode, presets[7], 0, Vector3::ZERO);
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

TEST_CASE("ApplyShapeHandleDrag returns to the start shape on a zero delta", "[editorops][handles]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    auto* box = AddShapeBrush<CsgBoxBrush>(f);
    const ShapeAttributeSnapshot start = SnapshotShapeAttributes(box);
    const ea::vector<unsigned> ids{7};
    ea::optional<Vector3> accepted;

    // A valid move is accepted.
    CHECK(ApplyShapeHandleDrag(box, start, ids, Vector3(0.25f, 0.0f, 0.0f), accepted));
    CHECK(accepted == Vector3(0.25f, 0.0f, 0.0f));
    const ShapeAttributeSnapshot moved = SnapshotShapeAttributes(box);
    CHECK(moved != start);

    // A refused move keeps the last accepted shape.
    CHECK_FALSE(ApplyShapeHandleDrag(box, start, ids, Vector3(-3.0f, -3.0f, -3.0f), accepted));
    CHECK(accepted == Vector3(0.25f, 0.0f, 0.0f));
    CHECK(SnapshotShapeAttributes(box) == moved);

    // A zero delta, for example a snap back to the start cell, gives the start shape.
    CHECK(ApplyShapeHandleDrag(box, start, ids, Vector3::ZERO, accepted));
    CHECK(accepted == Vector3::ZERO);
    CHECK(SnapshotShapeAttributes(box) == start);

    // A refused move after the zero delta stays at the start shape.
    CHECK_FALSE(ApplyShapeHandleDrag(box, start, ids, Vector3(-3.0f, -3.0f, -3.0f), accepted));
    CHECK(SnapshotShapeAttributes(box) == start);

    // A refused first move stays at the start shape and has no accepted delta.
    ea::optional<Vector3> none;
    CHECK_FALSE(ApplyShapeHandleDrag(box, start, ids, Vector3(-3.0f, -3.0f, -3.0f), none));
    CHECK_FALSE(none);
    CHECK(SnapshotShapeAttributes(box) == start);
    CHECK_FALSE(ApplyShapeHandleDrag(nullptr, start, ids, Vector3::ONE, none));
    CHECK(errors.GetCount() == 0);
}

TEST_CASE("PickNearestHandle gives an overlap to the nearer handle", "[editorops][handles]")
{
    // The box faces -Z (id 24) and +Z (id 25) seen along the Z axis are on one screen point.
    const ea::vector<HandleScreenPoint> overlap{
        {24, Vector2(100.0f, 100.0f), true, 5.0f},
        {25, Vector2(100.0f, 100.0f), true, 4.0f},
    };
    CHECK(PickNearestHandle(overlap, Vector2(101.0f, 100.0f), 10.0f) == 1u);
    const ea::vector<HandleScreenPoint> swapped{
        {24, Vector2(100.0f, 100.0f), true, 4.0f},
        {25, Vector2(100.0f, 100.0f), true, 5.0f},
    };
    CHECK(PickNearestHandle(swapped, Vector2(101.0f, 100.0f), 10.0f) == 0u);

    // A screen distance difference within 1 px is a tie: the nearer depth wins.
    const ea::vector<HandleScreenPoint> nearTie{
        {1, Vector2(100.0f, 100.0f), true, 9.0f},
        {2, Vector2(100.5f, 100.0f), true, 3.0f},
    };
    CHECK(PickNearestHandle(nearTie, Vector2(99.0f, 100.0f), 10.0f) == 1u);

    // A difference of more than 1 px is not a tie: the nearest on the screen wins.
    const ea::vector<HandleScreenPoint> noTie{
        {1, Vector2(100.0f, 100.0f), true, 9.0f},
        {2, Vector2(103.0f, 100.0f), true, 3.0f},
    };
    CHECK(PickNearestHandle(noTie, Vector2(99.0f, 100.0f), 10.0f) == 0u);

    // The same depth goes to the lower id, in both orders.
    const ea::vector<HandleScreenPoint> sameDepth{
        {9, Vector2(100.0f, 100.0f), true, 2.0f},
        {3, Vector2(100.0f, 100.0f), true, 2.0f},
    };
    CHECK(PickNearestHandle(sameDepth, Vector2(100.0f, 100.0f), 10.0f) == 1u);
}

namespace
{

/// The handles of a brush node of the type, in the local space.
template <class T> ea::vector<ShapeHandle> HandlesOf(Fixture& f)
{
    return QueryShapeHandles(AddShapeBrush<T>(f)).handles_;
}

} // namespace

TEST_CASE("ChooseFaceHandle picks the box face of the normal", "[editorops][pick]")
{
    Fixture f;
    const ea::vector<ShapeHandle> handles = HandlesOf<CsgBoxBrush>(f);
    REQUIRE(handles.size() == 14);

    // The face ids are 20 to 25: -X, +X, -Y, +Y, -Z, +Z.
    const Vector3 normals[6] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
    for (unsigned face = 0; face < 6; ++face)
    {
        const Vector3 point = normals[face] * 0.5f;
        const auto chosen = ChooseFaceHandle(handles, point, normals[face]);
        REQUIRE(chosen);
        CHECK(*chosen == 20 + face);
        // A flipped normal picks the same face: a subtract brush seen from inside.
        const auto flipped = ChooseFaceHandle(handles, point, -normals[face]);
        REQUIRE(flipped);
        CHECK(*flipped == 20 + face);
    }

    // A point near a deformed face: the nearer of the two parallel faces wins.
    CHECK(*ChooseFaceHandle(handles, Vector3(0.45f, 0.1f, 0.1f), Vector3(1, 0, 0)) == 21);
    CHECK(*ChooseFaceHandle(handles, Vector3(-0.3f, 0.1f, 0.1f), Vector3(1, 0, 0)) == 20);
    // A slight tilt below the limit still matches.
    CHECK(*ChooseFaceHandle(handles, Vector3(0.5f, 0, 0), Vector3(1, 0.3f, 0).Normalized()) == 21);

    // A 45 degree normal and a zero normal match nothing. No handles give none.
    CHECK_FALSE(ChooseFaceHandle(handles, Vector3(0.5f, 0, 0), Vector3(1, 1, 0).Normalized()));
    CHECK_FALSE(ChooseFaceHandle(handles, Vector3(0.5f, 0, 0), Vector3::ZERO));
    CHECK_FALSE(ChooseFaceHandle({}, Vector3::ZERO, Vector3(1, 0, 0)));
}

TEST_CASE("ChooseFaceHandle uses only axis handles and breaks ties by the lower id", "[editorops][pick]")
{
    ShapeHandle point;
    point.id_ = 1;
    point.kind_ = CsgShapeHandleKind::Point;
    point.position_ = Vector3(1, 0, 0);
    ShapeHandle a;
    a.id_ = 9;
    a.kind_ = CsgShapeHandleKind::Axis;
    a.position_ = Vector3(1, 0, 0);
    a.axis_ = Vector3(1, 0, 0);
    ShapeHandle b = a;
    b.id_ = 4;
    CHECK(ChooseFaceHandle({point, a, b}, Vector3(1, 0, 0), Vector3(1, 0, 0)) == 4u);
    CHECK(ChooseFaceHandle({b, a}, Vector3(1, 0, 0), Vector3(1, 0, 0)) == 4u);
    CHECK_FALSE(ChooseFaceHandle({point}, Vector3(1, 0, 0), Vector3(1, 0, 0)));
}

TEST_CASE("ChooseFaceHandle gives the radius handle of a sphere for an axis normal", "[editorops][pick]")
{
    Fixture f;
    const ea::vector<ShapeHandle> handles = HandlesOf<CsgSphereBrush>(f);
    REQUIRE(handles.size() == 6);
    CHECK(*ChooseFaceHandle(handles, Vector3(0.5f, 0, 0), Vector3(1, 0, 0)) == 0);
    CHECK(*ChooseFaceHandle(handles, Vector3(-0.5f, 0, 0), Vector3(-1, 0, 0)) == 1);
    CHECK(*ChooseFaceHandle(handles, Vector3(0, 0, 0.5f), Vector3(0, 0, 1)) == 4);
    CHECK_FALSE(ChooseFaceHandle(handles, Vector3(0.35f, 0.35f, 0), Vector3(1, 1, 0).Normalized()));
}

TEST_CASE("ChooseFaceHandleForWorldHit gives the same face for a rotated and scaled brush", "[editorops][pick]")
{
    Fixture f;
    auto* box = AddShapeBrush<CsgBoxBrush>(f);
    const ea::vector<ShapeHandle> handles = QueryShapeHandles(box).handles_;
    Node* node = box->GetNode();
    node->SetWorldPosition(Vector3(3, -2, 1));
    node->SetWorldRotation(Quaternion(35.0f, Vector3(1, 2, 0.5f)));
    node->SetWorldScale(Vector3(2.0f, 0.5f, 3.0f));

    const Vector3 localPoints[3] = {{0.5f, 0.1f, 0.2f}, {0.1f, -0.5f, 0.2f}, {0.1f, 0.2f, 0.5f}};
    const Vector3 localNormals[3] = {{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    const unsigned expected[3] = {21, 22, 25};
    for (unsigned i = 0; i < 3; ++i)
    {
        const auto local = ChooseFaceHandle(handles, localPoints[i], localNormals[i]);
        REQUIRE(local);
        CHECK(*local == expected[i]);
        const Vector3 worldPoint = node->LocalToWorld(localPoints[i]);
        const Vector3 worldNormal = (node->GetWorldRotation() * localNormals[i]).Normalized();
        CHECK(ChooseFaceHandleForWorldHit(node, handles, worldPoint, worldNormal) == local);
    }
    CHECK_FALSE(ChooseFaceHandleForWorldHit(nullptr, handles, Vector3::ZERO, Vector3(1, 0, 0)));
    CHECK_FALSE(ChooseFaceHandleForWorldHit(node, handles, Vector3::ZERO, Vector3::ZERO));
}

TEST_CASE("PickRootSource finds the subtract brush through the cavity", "[editorops][pick]")
{
    Fixture f;
    EditorErrorCounter errors(f.context);
    Node* rootNode = f.MakeRoot();
    const int diff = f.FindOperation("DifferenceAB");
    Node* cut = AddBrushNodeOfType(rootNode, "CsgBoxBrush", diff, Vector3(0.0f, 0.0f, 1.0f));
    REQUIRE(cut);
    cut->SetScale(Vector3(0.5f, 0.5f, 0.5f));
    RunFrames(f.scene, 60);
    auto* root = rootNode->GetComponent<CsgRoot>();
    REQUIRE(root);

    // A ray into the cavity hits the floor. The floor is a face of the subtract brush.
    // The brush has the scale 0.5 at z = 1, so the floor is at z = 0.75.
    const PickResult cavity = PickRootSource(root, Ray(Vector3(0, 0, 5), Vector3(0, 0, -1)));
    REQUIRE(cavity.hit_);
    CHECK(cavity.source_ == cut->GetID());
    CHECK(cavity.position_.Equals(Vector3(0, 0, 0.75f), 1e-3f));
    CHECK(cavity.normal_.Equals(Vector3(0, 0, 1), 1e-3f));

    // The face handle of the floor is the -Z face of the brush.
    const auto handles = QueryShapeHandles(cut->GetComponent<CsgBoxBrush>()).handles_;
    CHECK(ChooseFaceHandleForWorldHit(cut, handles, cavity.position_, cavity.normal_) == 24u);

    // A ray on the rim hits the base.
    const PickResult rim = PickRootSource(root, Ray(Vector3(0.8f, 0.8f, 5), Vector3(0, 0, -1)));
    REQUIRE(rim.hit_);
    CHECK(rim.source_ == 0);

    // A miss, a null root and a component that is not a root give no hit.
    CHECK_FALSE(PickRootSource(root, Ray(Vector3(5, 5, 5), Vector3(0, 0, 1))).hit_);
    CHECK_FALSE(PickRootSource(nullptr, Ray(Vector3(0, 0, 5), Vector3(0, 0, -1))).hit_);
    CHECK_FALSE(PickRootSource(cut->GetComponent<CsgBoxBrush>(), Ray(Vector3(0, 0, 5), Vector3(0, 0, -1))).hit_);

    // A removed brush: the baked face id can point to a node that is gone.
    const unsigned cutId = cut->GetID();
    cut->Remove();
    CHECK(f.scene->GetNode(cutId) == nullptr);
    const PickResult stale = PickRootSource(root, Ray(Vector3(0, 0, 5), Vector3(0, 0, -1)));
    if (stale.hit_ && stale.source_ != 0)
        CHECK(f.scene->GetNode(stale.source_) == nullptr);
    CHECK(errors.GetCount() == 0);
}

namespace
{

/// Build a root with a scale and one box brush (size 1) on a child with a local scale. Return the output volume.
/// The base is empty. The output is in the node space of the root.
double MeasureScaledBoxBrush(Fixture& f, const Vector3& rootScale, const Vector3& brushScale, bool scaleBeforeBuild)
{
    Node* rootNode = f.scene->CreateChild("ScaleRoot");
    rootNode->SetScale(rootScale);
    auto* root = rootNode->CreateComponent<CsgRoot>();
    root->SetDeferredDelaySec(0.0f);
    Node* boxNode = rootNode->CreateChild("Box");
    if (scaleBeforeBuild)
        boxNode->SetScale(brushScale);
    boxNode->CreateComponent<CsgBoxBrush>();
    RunFrames(f.scene, 60);
    if (!scaleBeforeBuild)
        boxNode->SetScale(brushScale);
    RunFrames(f.scene, 60);
    auto* output = root->GetResultModel();
    REQUIRE(output);
    REQUIRE(output->GetModel());
    return MeasureModelVolume(f.context, output->GetModel());
}

} // namespace

TEST_CASE("A procedural box brush uses its node scale in the root frame", "[editorops][root][procedural]")
{
    for (const bool before : {true, false})
    {
        CAPTURE(before);
        Fixture f;
        // Root scale 1: the box of size 1 with the node scale 0.5 has the volume 0.5^3.
        CHECK(MeasureScaledBoxBrush(f, Vector3::ONE, Vector3::ONE * 0.5f, before) == Catch::Approx(0.125).margin(1e-3));
    }
    for (const bool before : {true, false})
    {
        CAPTURE(before);
        Fixture f;
        // Root scale 2: the world scale of the brush is 1, so the fold space volume is 1.
        // The output is in the node space of the root, so its volume is 1 / (2 * 2 * 2).
        const double volume = MeasureScaledBoxBrush(f, Vector3::ONE * 2.0f, Vector3::ONE * 0.5f, before);
        CHECK(volume == Catch::Approx(0.125).margin(1e-3));
        CHECK(volume * 8.0 == Catch::Approx(1.0).margin(1e-3));
    }
}

TEST_CASE("RecenterShapePivot moves the node and the undo state restores both", "[editorops][handles][recenter]")
{
    Fixture f;
    Node* rootNode = f.MakeRoot();
    Node* boxNode = AddBrushNodeOfType(rootNode, "CsgBoxBrush", 0, Vector3(1.0f, 2.0f, 3.0f));
    REQUIRE(boxNode);
    boxNode->SetRotation(Quaternion(30.0f, Vector3(1.0f, 2.0f, 3.0f)));
    boxNode->SetScale(Vector3(1.0f, 2.0f, 3.0f));
    Component* brush = FindShapeBrush(boxNode);
    REQUIRE(brush);

    // A face move puts the box off the node origin.
    const ShapeEditState start = CaptureShapeEditState(brush);
    REQUIRE(MoveShapeHandles(brush, {21}, Vector3(2.0f, 0.0f, 0.0f)));
    const auto worldHandle = [&](unsigned id)
    {
        for (const ShapeHandle& handle : QueryShapeHandles(brush).handles_)
        {
            if (handle.id_ == id)
                return boxNode->LocalToWorld(handle.position_);
        }
        return Vector3::ZERO;
    };
    ea::vector<Vector3> before;
    for (unsigned id : {0u, 7u, 20u, 21u, 25u})
        before.push_back(worldHandle(id));
    const ShapeEditState moved = CaptureShapeEditState(brush);
    CHECK(moved != start);

    REQUIRE(RecenterShapePivot(brush));
    const ShapeEditState centered = CaptureShapeEditState(brush);
    CHECK(centered.nodePosition_ != moved.nodePosition_);
    unsigned index = 0;
    for (unsigned id : {0u, 7u, 20u, 21u, 25u})
        CHECK(worldHandle(id).Equals(before[index++], 1e-4f));
    // The handle ids are the same and the handles stay valid.
    CHECK(QueryShapeHandles(brush).valid_);
    CHECK_FALSE(RecenterShapePivot(brush));

    // One undo step reverts the attributes and the position. Redo gives the centered state.
    CHECK(RestoreShapeEditState(brush, moved));
    CHECK(CaptureShapeEditState(brush) == moved);
    CHECK(boxNode->GetPosition() == moved.nodePosition_);
    CHECK_FALSE(RestoreShapeEditState(brush, moved));
    CHECK(RestoreShapeEditState(brush, centered));
    CHECK(CaptureShapeEditState(brush) == centered);
    index = 0;
    for (unsigned id : {0u, 7u, 20u, 21u, 25u})
        CHECK(worldHandle(id).Equals(before[index++], 1e-4f));

    // A non-shape component and a null pointer do nothing.
    CHECK_FALSE(RecenterShapePivot(nullptr));
    CHECK_FALSE(RecenterShapePivot(rootNode->GetComponent<CsgRoot>()));
}

namespace
{

/// Return the world normal of a local normal: the transpose of the inverse of the world 3x3 matrix.
Vector3 LocalToWorldNormal(const Node* node, const Vector3& localNormal)
{
    return (node->GetWorldTransform().ToMatrix3().Inverse().Transpose() * localNormal).Normalized();
}

} // namespace

TEST_CASE("ChooseFaceHandleForWorldHit uses the normal matrix for skewed normals", "[editorops][pick]")
{
    Fixture f;
    SECTION("A sphere on a node with the scale (1, 4, 1)")
    {
        Node* node = f.scene->CreateChild("Sphere");
        auto* sphere = node->CreateComponent<CsgSphereBrush>();
        node->SetScale(Vector3(1.0f, 4.0f, 1.0f));
        node->SetPosition(Vector3(1.0f, 2.0f, 3.0f));
        const auto handles = QueryShapeHandles(sphere).handles_;
        REQUIRE_FALSE(handles.empty());
        const float radius = sphere->GetRadius().x_;

        // The angle is measured from the +X axis in the local XY plane. 35 degrees is not in line with an axis.
        for (const float degrees : {0.0f, 10.0f, 21.0f, 35.0f, 55.0f, 80.0f})
        {
            CAPTURE(degrees);
            const Vector3 localNormal(Cos(degrees), Sin(degrees), 0.0f);
            const Vector3 localPoint = localNormal * radius;
            const Vector3 worldPoint = node->LocalToWorld(localPoint);
            const Vector3 worldNormal = LocalToWorldNormal(node, localNormal);
            CHECK(ChooseFaceHandleForWorldHit(node, handles, worldPoint, worldNormal)
                == ChooseFaceHandle(handles, localPoint, localNormal));
        }
        // The case that shows the difference: no handle is in line at 35 degrees.
        const Vector3 n35(Cos(35.0f), Sin(35.0f), 0.0f);
        CHECK_FALSE(ChooseFaceHandleForWorldHit(node, handles, node->LocalToWorld(n35 * radius), LocalToWorldNormal(node, n35)));
    }

    SECTION("A slanted box face under a rotated node with a parent of the scale (2, 1, 1)")
    {
        Node* parent = f.scene->CreateChild("Parent");
        parent->SetScale(Vector3(2.0f, 1.0f, 1.0f));
        parent->SetRotation(Quaternion(20.0f, Vector3::UP));
        Node* node = parent->CreateChild("Box");
        auto* box = node->CreateComponent<CsgBoxBrush>();
        node->SetRotation(Quaternion(50.0f, Vector3::FORWARD));
        // The +X face leans: the top corners of the face go out by 0.45.
        box->SetCorner(3, box->GetCorner(3) + Vector3(0.45f, 0.0f, 0.0f));
        box->SetCorner(7, box->GetCorner(7) + Vector3(0.45f, 0.0f, 0.0f));
        const auto handles = QueryShapeHandles(box).handles_;
        REQUIRE_FALSE(handles.empty());

        // The true local normal of the face: the plane through the corners 1, 3 and 5.
        const Vector3 p1 = box->GetSize() * box->GetCorner(1);
        const Vector3 p3 = box->GetSize() * box->GetCorner(3);
        const Vector3 p5 = box->GetSize() * box->GetCorner(5);
        const Vector3 localNormal = (p3 - p1).CrossProduct(p5 - p1).Normalized();
        REQUIRE(localNormal.x_ > 0.0f);
        const Vector3 localPoint = (p1 + p3 + p5) / 3.0f;
        const auto expected = ChooseFaceHandle(handles, localPoint, localNormal);
        REQUIRE(expected);
        CHECK(*expected == 21u);
        CHECK(ChooseFaceHandleForWorldHit(
                  node, handles, node->LocalToWorld(localPoint), LocalToWorldNormal(node, localNormal))
            == expected);
    }
}
