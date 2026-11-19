// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Tests for the shape handles of the procedural brushes. The tests use no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoxBrush.h"
#include "Urho3D/CSG/CsgCylinderBrush.h"
#include "Urho3D/CSG/CsgRingBrush.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSG/CsgShapeProtocol.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/CsgSphereBrush.h"
#include "TestGeneratedBrush.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Timer.h>
#include <Urho3D/Core/WorkQueue.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/IO/IOEvents.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>

#include <cmath>
#include <limits>

#include <catch2/catch_amalgamated.hpp>

using namespace Urho3D;

namespace
{

constexpr float Eps = 1e-5f;

/// Count the errors that the log gets.
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

/// A context, a scene and a node with a brush of the type T.
template <class T> struct HandleFixture
{
    HandleFixture()
        : context(MakeContext())
        , scene(MakeShared<Scene>(context))
        , errors(context)
    {
        scene->CreateComponent<Octree>();
        brush = scene->CreateChild("Brush")->CreateComponent<T>();
    }

    static SharedPtr<Context> MakeContext()
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

    ea::vector<CsgShapeHandle> Handles() const
    {
        ea::vector<CsgShapeHandle> handles;
        brush->GetHandles(handles);
        return handles;
    }

    SharedPtr<Context> context;
    SharedPtr<Scene> scene;
    ErrorCounter errors;
    T* brush{};
};

const CsgShapeHandle* FindHandle(const ea::vector<CsgShapeHandle>& handles, unsigned id)
{
    for (const CsgShapeHandle& handle : handles)
    {
        if (handle.id_ == id)
            return &handle;
    }
    return nullptr;
}

ea::vector<unsigned> GetIds(const ea::vector<CsgShapeHandle>& handles)
{
    ea::vector<unsigned> ids;
    for (const CsgShapeHandle& handle : handles)
        ids.push_back(handle.id_);
    return ids;
}

bool IsNear(const Vector3& a, const Vector3& b, float eps = Eps) { return (a - b).Length() <= eps; }

/// Return true if a vertex of the mesh is within eps of the position.
bool MeshHasVertexNear(const CsgShapeMesh& mesh, const Vector3& position, float eps = 1e-6f)
{
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        if (IsNear(vertex.GetPosition(), position, eps))
            return true;
    }
    return false;
}

unsigned CountChangedCorners(const CsgBoxParams& a, const CsgBoxParams& b)
{
    unsigned count = 0;
    for (unsigned i = 0; i < 8; ++i)
        count += a.corners_[i] == b.corners_[i] ? 0 : 1;
    return count;
}

/// Send the query event and return the data map.
VariantMap Query(Object* target)
{
    VariantMap data;
    target->SendEvent(E_CSGSHAPEQUERYHANDLES, data);
    return data;
}

/// Send the move event and return the P_ACCEPTED value.
bool SendMove(Object* target, const ea::vector<int>& ids, const Vector3& delta)
{
    VariantMap data;
    VariantVector list;
    for (const int id : ids)
        list.push_back(Variant(id));
    data[CsgShapeMoveHandles::P_INDICES] = list;
    data[CsgShapeMoveHandles::P_DELTA] = delta;
    target->SendEvent(E_CSGSHAPEMOVEHANDLES, data);
    return data[CsgShapeMoveHandles::P_ACCEPTED].GetBool();
}

} // namespace

TEST_CASE("The box brush has 8 corner handles and 6 face handles with stable ids", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;
    const auto handles = f.Handles();
    REQUIRE(handles.size() == 14);

    ea::vector<unsigned> expected;
    for (unsigned i = 0; i < 8; ++i)
        expected.push_back(i);
    for (unsigned i = 20; i < 26; ++i)
        expected.push_back(i);
    CHECK(GetIds(handles) == expected);

    for (unsigned i = 0; i < 8; ++i)
    {
        CHECK(handles[i].kind_ == CsgShapeHandleKind::Point);
        CHECK(handles[i].axis_ == Vector3::ZERO);
        CHECK_FALSE(handles[i].name_.empty());
    }
    CHECK(handles[7].name_ == "Corner +X+Y+Z");
    CHECK(handles[1].name_ == "Corner +X-Y-Z");
    for (unsigned i = 8; i < 14; ++i)
    {
        CHECK(handles[i].kind_ == CsgShapeHandleKind::Axis);
        CHECK(handles[i].axis_.Length() == Catch::Approx(1.0f));
    }
    CHECK(FindHandle(handles, 20)->axis_ == Vector3(-1, 0, 0));
    CHECK(FindHandle(handles, 21)->axis_ == Vector3(1, 0, 0));
    CHECK(FindHandle(handles, 22)->axis_ == Vector3(0, -1, 0));
    CHECK(FindHandle(handles, 23)->axis_ == Vector3(0, 1, 0));
    CHECK(FindHandle(handles, 24)->axis_ == Vector3(0, 0, -1));
    CHECK(FindHandle(handles, 25)->axis_ == Vector3(0, 0, 1));
}

TEST_CASE("Box corner handle positions equal the generated corners and face handles lie on the faces",
    "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;
    CsgBoxParams params;
    params.size_ = Vector3(2.0f, 3.0f, 4.0f);
    params.corners_[5] += Vector3(0.1f, -0.05f, 0.2f);
    f.brush->SetParams(params);

    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateBox(f.brush->GetParams(), mesh));
    const auto handles = f.Handles();
    for (unsigned i = 0; i < 8; ++i)
    {
        CHECK(handles[i].position_ == f.brush->GetSize() * f.brush->GetCorner(i));
        CHECK(MeshHasVertexNear(mesh, handles[i].position_));
    }

    // The face handle is the mean of the unique positions of the face group.
    REQUIRE(mesh.groups_.size() == 6);
    for (unsigned face = 0; face < 6; ++face)
    {
        const CsgShapeGroup& group = mesh.groups_[face];
        ea::vector<Vector3> unique;
        for (unsigned i = group.indexStart_; i < group.indexStart_ + group.indexCount_; ++i)
        {
            const Vector3 p = mesh.vertices_[mesh.indices_[i]].GetPosition();
            bool found = false;
            for (const Vector3& u : unique)
                found = found || u == p;
            if (!found)
                unique.push_back(p);
        }
        REQUIRE(unique.size() == 4);
        const Vector3 mean = (unique[0] + unique[1] + unique[2] + unique[3]) / 4.0f;
        CHECK(IsNear(FindHandle(handles, 20 + face)->position_, mean, 1e-5f));
    }

    // For the default box the face handle is at the center of the nominal face.
    f.brush->SetParams(CsgBoxParams());
    CHECK(FindHandle(f.Handles(), 21)->position_ == Vector3(0.5f, 0.0f, 0.0f));
    CHECK(FindHandle(f.Handles(), 24)->position_ == Vector3(0.0f, 0.0f, -0.5f));
}

TEST_CASE("A box corner handle move changes exactly one corner", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->SetSize(Vector3(2.0f, 4.0f, 8.0f));
    const CsgBoxParams before = f.brush->GetParams();

    REQUIRE(f.brush->MoveHandles({7}, Vector3(0.1f, 0.2f, 0.4f)));
    const CsgBoxParams after = f.brush->GetParams();
    CHECK(CountChangedCorners(before, after) == 1);
    CHECK(IsNear(after.corners_[7], before.corners_[7] + Vector3(0.05f, 0.05f, 0.05f)));
    CHECK(after.size_ == before.size_);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("A box face handle move changes exactly the 4 corners of the face along the axis", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->SetSize(Vector3(2.0f, 1.0f, 1.0f));
    const CsgBoxParams before = f.brush->GetParams();

    // The part of the delta that is not along the axis does not count.
    REQUIRE(f.brush->MoveHandles({21}, Vector3(0.5f, 7.0f, -3.0f)));
    const CsgBoxParams after = f.brush->GetParams();
    CHECK(CountChangedCorners(before, after) == 4);
    for (unsigned i = 0; i < 8; ++i)
    {
        if (i & 1u)
            CHECK(IsNear(after.corners_[i], before.corners_[i] + Vector3(0.25f, 0.0f, 0.0f)));
        else
            CHECK(after.corners_[i] == before.corners_[i]);
    }

    // The -X handle grows the box when it goes outward, along -X.
    REQUIRE(f.brush->MoveHandles({20}, Vector3(-1.0f, 0.0f, 0.0f)));
    CHECK(IsNear(f.brush->GetCorner(0), after.corners_[0] + Vector3(-0.5f, 0.0f, 0.0f)));

    // Two handles with a common corner: the corner moves by both.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({21, 23}, Vector3(0.2f, 0.1f, 0.0f)));
    CHECK(IsNear(f.brush->GetCorner(3), before.corners_[3] + Vector3(0.1f, 0.1f, 0.0f)));
    CHECK(IsNear(f.brush->GetCorner(1), before.corners_[1] + Vector3(0.1f, 0.0f, 0.0f)));
    CHECK(IsNear(f.brush->GetCorner(2), before.corners_[2] + Vector3(0.0f, 0.1f, 0.0f)));
    CHECK(f.brush->GetCorner(0) == before.corners_[0]);
    // A repeated id counts once.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({21, 21}, Vector3(0.5f, 0.0f, 0.0f)));
    CHECK(IsNear(f.brush->GetCorner(1), before.corners_[1] + Vector3(0.25f, 0.0f, 0.0f)));
}

TEST_CASE("A box move that inverts the box returns false and keeps the parameters", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->ClearDirty();
    const CsgBoxParams before = f.brush->GetParams();

    // The -X face goes to x = 1.5, through the +X face.
    CHECK_FALSE(f.brush->MoveHandles({20}, Vector3(2.0f, 0.0f, 0.0f)));
    CHECK(f.brush->GetParams().corners_ == before.corners_);
    CHECK(f.brush->GetParams().size_ == before.size_);
    CHECK_FALSE(f.brush->IsDirty());

    // One corner through the opposite corner.
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(3.0f, 3.0f, 3.0f)));
    CHECK(f.brush->GetParams().corners_ == before.corners_);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("Handle moves are stateless and idempotent after a restore of the snapshot", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->SetSize(Vector3(2.0f, 3.0f, 1.5f));
    const CsgBoxParams snapshot = f.brush->GetParams();
    const Vector3 d1(0.3f, 0.2f, -0.1f);
    const Vector3 d2(-0.2f, 0.5f, 0.25f);

    REQUIRE(f.brush->MoveHandles({3, 25}, d1));
    const CsgBoxParams once = f.brush->GetParams();

    f.brush->SetParams(snapshot);
    REQUIRE(f.brush->MoveHandles({3, 25}, d1));
    CHECK(f.brush->GetParams().corners_ == once.corners_);

    // The move d2 from the original is the same after an earlier move d1 and a restore.
    f.brush->SetParams(snapshot);
    REQUIRE(f.brush->MoveHandles({3, 25}, d2));
    const CsgBoxParams direct = f.brush->GetParams();
    f.brush->SetParams(snapshot);
    REQUIRE(f.brush->MoveHandles({3, 25}, d1));
    f.brush->SetParams(snapshot);
    REQUIRE(f.brush->MoveHandles({3, 25}, d2));
    CHECK(f.brush->GetParams().corners_ == direct.corners_);
}

TEST_CASE("A handle move with a bad delta or a bad id returns false without an error", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->ClearDirty();
    const CsgBoxParams before = f.brush->GetParams();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(nan, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(0.0f, inf, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({21}, Vector3(-inf, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(1e38f, 1e38f, 1e38f)));
    CHECK_FALSE(f.brush->MoveHandles({8}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({26}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0, 999}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK(f.brush->GetParams().corners_ == before.corners_);
    CHECK_FALSE(f.brush->IsDirty());
    CHECK(f.errors.GetCount() == 0);

    // The same through the event.
    CHECK_FALSE(SendMove(f.brush, {0}, Vector3(nan, 0.0f, 0.0f)));
    CHECK_FALSE(SendMove(f.brush, {-1}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(SendMove(f.brush, {12345}, Vector3(0.1f, 0.0f, 0.0f)));
    VariantMap bad;
    f.brush->SendEvent(E_CSGSHAPEMOVEHANDLES, bad);
    CHECK_FALSE(bad[CsgShapeMoveHandles::P_ACCEPTED].GetBool());
    CHECK(f.brush->GetParams().corners_ == before.corners_);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("The handle events round trip through a brush component in a scene", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;

    SECTION("The query gives the same handles as the virtual")
    {
        const VariantMap data = Query(f.brush);
        REQUIRE(data.contains(CsgShapeQueryHandles::P_VERSION));
        CHECK(data.at(CsgShapeQueryHandles::P_VERSION).GetInt() == 1);
        CHECK(data.at(CsgShapeQueryHandles::P_VALID).GetBool());
        const VariantVector& list = data.at(CsgShapeQueryHandles::P_HANDLES).GetVariantVector();
        const auto handles = f.Handles();
        REQUIRE(list.size() == handles.size());
        for (unsigned i = 0; i < handles.size(); ++i)
        {
            const VariantMap& entry = list[i].GetVariantMap();
            CHECK(entry.at(CsgShapeProtocol::HandleIdKey).GetInt() == static_cast<int>(handles[i].id_));
            CHECK(entry.at(CsgShapeProtocol::HandleNameKey).GetString() == handles[i].name_);
            CHECK(entry.at(CsgShapeProtocol::HandleKindKey).GetInt() == static_cast<int>(handles[i].kind_));
            CHECK(entry.at(CsgShapeProtocol::HandlePositionKey).GetVector3() == handles[i].position_);
            CHECK(entry.at(CsgShapeProtocol::HandleAxisKey).GetVector3() == handles[i].axis_);
        }
    }

    SECTION("The move changes the attributes and sets P_ACCEPTED")
    {
        const Variant before = f.brush->GetAttribute("Corner 7");
        CHECK(SendMove(f.brush, {7}, Vector3(0.25f, 0.0f, 0.0f)));
        const Variant after = f.brush->GetAttribute("Corner 7");
        CHECK(after.GetVector3() == before.GetVector3() + Vector3(0.25f, 0.0f, 0.0f));
        CHECK_FALSE(SendMove(f.brush, {0}, Vector3(5.0f, 5.0f, 5.0f)));
    }

    SECTION("The query for a shape that is not valid gives no handle")
    {
        // Both ends are at one height: the box has no volume.
        CsgBoxParams params;
        params.corners_[4] = Vector3(-0.5f, -0.5f, -0.5f);
        params.corners_[0] = Vector3(-0.5f, -0.5f, 0.5f);
        f.brush->SetParams(params);
        REQUIRE_FALSE(f.brush->IsShapeValid());
        const VariantMap data = Query(f.brush);
        CHECK_FALSE(data.at(CsgShapeQueryHandles::P_VALID).GetBool());
        CHECK(data.at(CsgShapeQueryHandles::P_HANDLES).GetVariantVector().empty());
    }
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("A handle move marks the brush dirty and the root rebuilds", "[shapes][handles][root]")
{
    HandleFixture<CsgBoxBrush> f;
    auto* workQueue = f.context->GetSubsystem<WorkQueue>();
    auto* time = f.context->GetSubsystem<Time>();

    SharedPtr<Model> base = MakeTestBoxModel(f.context, Vector3::ONE);
    base->SetName("Csg/Base.mdl");
    f.context->GetSubsystem<ResourceCache>()->AddManualResource(base);

    Node* rootNode = f.scene->CreateChild("Root");
    auto* root = rootNode->CreateComponent<CsgRoot>();
    root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Base.mdl"));
    root->SetDeferredDelaySec(0.0f);
    Node* brushNode = f.brush->GetNode();
    brushNode->SetParent(rootNode);
    brushNode->SetPosition(Vector3(5.0f, 0.0f, 0.0f));

    const auto run = [&]
    {
        for (unsigned i = 0; i < 60; ++i)
        {
            workQueue->CompleteAll();
            time->BeginFrame(1.0f / 60.0f);
            f.scene->Update(1.0f / 60.0f);
            time->EndFrame();
        }
    };
    run();
    REQUIRE(root->GetLastBuildStats().valid_);
    const double volumeBefore = root->GetLastBuildStats().volume_;
    CHECK_FALSE(f.brush->IsDirty());

    REQUIRE(f.brush->MoveHandles({21}, Vector3(1.0f, 0.0f, 0.0f)));
    CHECK(f.brush->IsDirty());
    run();
    CHECK(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().volume_ > volumeBefore + 0.5);
    CHECK(f.errors.GetCount() == 0);
}
