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
#include "Urho3D/CSG/CsgArchBrush.h"
#include "Urho3D/CSG/CsgStairsBrush.h"
#include "Urho3D/CSG/CsgWedgeBrush.h"
#include "Urho3D/CSGEditor/CsgShapeEditOps.h"
#include "TestGeneratedBrush.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Timer.h>
#include <Urho3D/Core/WorkQueue.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/IOEvents.h>
#include <Urho3D/IO/VectorBuffer.h>
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
    REQUIRE(handles.size() == 29);

    ea::vector<unsigned> expected;
    for (unsigned i = 0; i < 8; ++i)
        expected.push_back(i);
    for (unsigned i = 20; i < 26; ++i)
        expected.push_back(i);
    for (unsigned i = 30; i < 42; ++i)
        expected.push_back(i);
    for (unsigned i = 50; i < 53; ++i)
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

namespace
{

/// Return the world positions of the generated vertices as a flat list.
bool MeshHasVertexNearAny(const CsgShapeMesh& mesh, const Vector3& p) { return MeshHasVertexNear(mesh, p, 1e-5f); }

void CheckEdgeHandles(const HandleFixture<CsgBoxBrush>& f)
{
    const auto handles = f.Handles();
    REQUIRE(handles.size() == 29);
    for (unsigned edge = 0; edge < 12; ++edge)
    {
        const CsgShapeHandle* h = FindHandle(handles, 30 + edge);
        REQUIRE(h);
        CHECK(h->kind_ == CsgShapeHandleKind::Point);
        CHECK(h->axis_ == Vector3::ZERO);
        const unsigned axis = edge / 4;
        const unsigned b = axis == 0 ? 1 : 0;
        const unsigned c = axis == 2 ? 1 : 2;
        const unsigned low = ((edge & 1u) << b) | (((edge >> 1) & 1u) << c);
        const unsigned high = low | (1u << axis);
        const Vector3 mean = (handles[low].position_ + handles[high].position_) * 0.5f;
        CHECK(IsNear(h->position_, mean, 1e-5f));
    }
}

} // namespace

TEST_CASE("Box edge handles have ids 30 to 41, names and midpoint positions", "[shapes][handles][box][edges]")
{
    HandleFixture<CsgBoxBrush> f;
    const auto handles = f.Handles();
    REQUIRE(handles.size() == 29);
    const char* names[12] = {"Edge X -Y -Z", "Edge X +Y -Z", "Edge X -Y +Z", "Edge X +Y +Z", "Edge -X Y -Z",
        "Edge +X Y -Z", "Edge -X Y +Z", "Edge +X Y +Z", "Edge -X -Y Z", "Edge +X -Y Z", "Edge -X +Y Z",
        "Edge +X +Y Z"};
    for (unsigned i = 0; i < 12; ++i)
        CHECK(FindHandle(handles, 30 + i)->name_ == names[i]);
    CHECK(FindHandle(handles, 30)->position_ == Vector3(0.0f, -0.5f, -0.5f));
    CHECK(FindHandle(handles, 37)->position_ == Vector3(0.5f, 0.0f, 0.5f));
    CHECK(FindHandle(handles, 41)->position_ == Vector3(0.5f, 0.5f, 0.0f));
    CheckEdgeHandles(f);

    // Moved corners and a size.
    CsgBoxParams params;
    params.size_ = Vector3(2.0f, 3.0f, 4.0f);
    params.corners_[5] += Vector3(0.1f, -0.05f, 0.2f);
    params.corners_[2] += Vector3(-0.1f, 0.05f, 0.0f);
    f.brush->SetParams(params);
    CheckEdgeHandles(f);

    // A rotated and scaled node does not change the local handles.
    f.brush->GetNode()->SetRotation(Quaternion(30.0f, Vector3(1, 2, 3)));
    f.brush->GetNode()->SetScale(Vector3(2.0f, 1.0f, 0.5f));
    CheckEdgeHandles(f);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("A box edge handle move changes exactly its two corners", "[shapes][handles][box][edges]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->SetSize(Vector3(2.0f, 4.0f, 8.0f));
    const CsgBoxParams before = f.brush->GetParams();
    const Vector3 delta(0.1f, 0.2f, 0.4f);

    for (unsigned edge = 0; edge < 12; ++edge)
    {
        f.brush->SetParams(before);
        REQUIRE(f.brush->MoveHandles({30 + edge}, delta));
        const CsgBoxParams after = f.brush->GetParams();
        CHECK(CountChangedCorners(before, after) == 2);
        for (unsigned i = 0; i < 8; ++i)
        {
            if (after.corners_[i] != before.corners_[i])
                CHECK(IsNear(after.corners_[i], before.corners_[i] + delta / before.size_));
        }
        // Edge 0 is X -Y -Z: the corners 0 and 1.
        CsgShapeMesh mesh;
        REQUIRE(CsgGenerateBox(after, mesh));
        for (unsigned i = 0; i < 8; ++i)
            CHECK(MeshHasVertexNearAny(mesh, after.size_ * after.corners_[i]));
    }

    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({30}, delta));
    CHECK(f.brush->GetCorner(0) != before.corners_[0]);
    CHECK(f.brush->GetCorner(1) != before.corners_[1]);
    CHECK(CountChangedCorners(before, f.brush->GetParams()) == 2);

    // Edge 4 is Y -X -Z: the corners 0 and 2. Edge 8 is Z -X -Y: the corners 0 and 4. Edge 11 is +X +Y: 3 and 7.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({34}, delta));
    CHECK(f.brush->GetCorner(0) != before.corners_[0]);
    CHECK(f.brush->GetCorner(2) != before.corners_[2]);
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({41}, delta));
    CHECK(f.brush->GetCorner(3) != before.corners_[3]);
    CHECK(f.brush->GetCorner(7) != before.corners_[7]);
    CHECK(CountChangedCorners(before, f.brush->GetParams()) == 2);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("A box edge handle move rejects zero and invalid moves and is stateless",
    "[shapes][handles][box][edges]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->ClearDirty();
    const CsgBoxParams snapshot = f.brush->GetParams();

    CHECK_FALSE(f.brush->MoveHandles({33}, Vector3::ZERO));
    CHECK_FALSE(f.brush->IsDirty());
    // The edge of the -Y -Z side goes through the opposite side: zero volume.
    CHECK_FALSE(f.brush->MoveHandles({30}, Vector3(0.0f, 1.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({30}, Vector3(0.0f, 3.0f, 3.0f)));
    // A move over the clamp limit.
    CHECK_FALSE(f.brush->MoveHandles({30}, Vector3(1e30f, 0.0f, 0.0f)));
    CHECK(f.brush->GetParams().corners_ == snapshot.corners_);
    CHECK_FALSE(f.brush->IsDirty());

    const Vector3 d1(0.3f, 0.2f, -0.1f);
    const Vector3 d2(-0.2f, 0.5f, 0.25f);
    REQUIRE(f.brush->MoveHandles({35}, d2));
    const CsgBoxParams expected = f.brush->GetParams();
    f.brush->SetParams(snapshot);
    REQUIRE(f.brush->MoveHandles({35}, d1));
    f.brush->SetParams(snapshot);
    REQUIRE(f.brush->MoveHandles({35}, d2));
    CHECK(f.brush->GetParams().corners_ == expected.corners_);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("A shared corner moves once for an edge with a corner or another edge", "[shapes][handles][box][edges]")
{
    HandleFixture<CsgBoxBrush> f;
    const CsgBoxParams before = f.brush->GetParams();
    const Vector3 delta(0.1f, 0.0f, 0.2f);

    // Edge 30 has the corners 0 and 1. The corner 1 is also selected.
    REQUIRE(f.brush->MoveHandles({30, 1}, delta));
    CHECK(CountChangedCorners(before, f.brush->GetParams()) == 2);
    CHECK(IsNear(f.brush->GetCorner(1), before.corners_[1] + delta));
    CHECK(IsNear(f.brush->GetCorner(0), before.corners_[0] + delta));

    // Edge 30 (corners 0, 1) and edge 34 (corners 0, 2) share the corner 0: 3 corners move, once each.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({30, 34}, delta));
    CHECK(CountChangedCorners(before, f.brush->GetParams()) == 3);
    for (unsigned i = 0; i < 3; ++i)
        CHECK(IsNear(f.brush->GetCorner(i), before.corners_[i] + delta));

    // Edge 30 and the +X face (corners 1, 3, 5, 7) share the corner 1. The face adds its axis part.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({30, 21}, delta));
    CHECK(IsNear(f.brush->GetCorner(0), before.corners_[0] + delta));
    CHECK(IsNear(f.brush->GetCorner(1), before.corners_[1] + delta + Vector3(0.1f, 0.0f, 0.0f)));
    CHECK(IsNear(f.brush->GetCorner(3), before.corners_[3] + Vector3(0.1f, 0.0f, 0.0f)));
    CHECK(CountChangedCorners(before, f.brush->GetParams()) == 5);
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

namespace
{

/// Check the ids of a cage: all exist, no line joins a handle to itself, no duplicate pair.
template <class T> void CheckCage(HandleFixture<T>& f, unsigned expectedCount)
{
    const auto handles = f.Handles();
    ea::vector<ea::pair<unsigned, unsigned>> lines;
    f.brush->GetCageLines(lines);
    if (expectedCount != ~0u)
        CHECK(lines.size() == expectedCount);
    const auto exists = [&](unsigned id)
    { return ea::any_of(handles.begin(), handles.end(), [&](const CsgShapeHandle& h) { return h.id_ == id; }); };
    for (unsigned i = 0; i < lines.size(); ++i)
    {
        CHECK(exists(lines[i].first));
        CHECK(exists(lines[i].second));
        CHECK(lines[i].first != lines[i].second);
        for (unsigned j = i + 1; j < lines.size(); ++j)
        {
            const bool same = lines[i] == lines[j]
                || (lines[i].first == lines[j].second && lines[i].second == lines[j].first);
            CHECK(!same);
        }
    }
}

} // namespace

TEST_CASE("The cage lines of the shapes", "[shapes][handles][cage]")
{
    SECTION("The box has 48 lines on the box")
    {
        HandleFixture<CsgBoxBrush> f;
        CheckCage(f, 12 * 2 + 6 * 4);
        const auto handles = f.Handles();
        const auto find = [&](unsigned id) -> const CsgShapeHandle&
        {
            for (const CsgShapeHandle& h : handles)
                if (h.id_ == id)
                    return h;
            FAIL("missing handle");
            return handles[0];
        };
        ea::vector<ea::pair<unsigned, unsigned>> lines;
        f.brush->GetCageLines(lines);
        // The default box has the size 1: all points are on the cube of half size 0.5.
        for (const auto& line : lines)
        {
            for (const unsigned id : {line.first, line.second})
            {
                const Vector3 p = find(id).position_;
                CHECK(std::max({std::abs(p.x_), std::abs(p.y_), std::abs(p.z_)}) == Catch::Approx(0.5f).margin(Eps));
            }
        }
        // Each edge midpoint is the center of the corner-to-corner span of its two lines.
        for (unsigned edge = 0; edge < 12; ++edge)
        {
            const unsigned mid = CsgBoxBrush::FirstEdgeHandle + edge;
            unsigned count = 0;
            for (const auto& line : lines)
                count += (line.first == mid || line.second == mid) ? 1 : 0;
            CHECK(count == 4); // 2 half lines and 2 face lines
        }
    }

    SECTION("The other shapes have valid ids")
    {
        { HandleFixture<CsgCylinderBrush> f; CheckCage(f, 1); }
        { HandleFixture<CsgSphereBrush> f; CheckCage(f, ~0u); }
        { HandleFixture<CsgRingBrush> f; CheckCage(f, ~0u); }
        { HandleFixture<CsgArchBrush> f; CheckCage(f, ~0u); }
        { HandleFixture<CsgStairsBrush> f; CheckCage(f, ~0u); }
        { HandleFixture<CsgWedgeBrush> f; CheckCage(f, ~0u); }
    }

    SECTION("The event gives P_CAGE")
    {
        HandleFixture<CsgBoxBrush> f;
        const VariantMap data = Query(f.brush);
        REQUIRE(data.contains(CsgShapeQueryHandles::P_CAGE));
        const VariantVector& cage = data.at(CsgShapeQueryHandles::P_CAGE).GetVariantVector();
        ea::vector<ea::pair<unsigned, unsigned>> lines;
        f.brush->GetCageLines(lines);
        REQUIRE(cage.size() == lines.size());
        for (unsigned i = 0; i < lines.size(); ++i)
        {
            CHECK(cage[i].GetIntVector2().x_ == static_cast<int>(lines[i].first));
            CHECK(cage[i].GetIntVector2().y_ == static_cast<int>(lines[i].second));
        }
    }
}

TEST_CASE("The handle events round trip through a brush component in a scene", "[shapes][handles]")
{
    HandleFixture<CsgBoxBrush> f;

    SECTION("The query gives the same handles as the virtual")
    {
        const VariantMap data = Query(f.brush);
        REQUIRE(data.contains(CsgShapeQueryHandles::P_VERSION));
        CHECK(data.at(CsgShapeQueryHandles::P_VERSION).GetInt() == 2);
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

TEST_CASE("The sphere brush has 6 radius handles on the axes", "[shapes][handles]")
{
    HandleFixture<CsgSphereBrush> f;
    f.brush->SetRadius(Vector3(1.0f, 2.0f, 3.0f));
    const auto handles = f.Handles();
    CHECK(GetIds(handles) == ea::vector<unsigned>{0, 1, 2, 3, 4, 5});

    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(f.brush->GetParams(), mesh));
    const Vector3 axes[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    const Vector3 radius = f.brush->GetRadius();
    const float components[6] = {radius.x_, radius.x_, radius.y_, radius.y_, radius.z_, radius.z_};
    for (unsigned i = 0; i < 6; ++i)
    {
        CHECK(handles[i].kind_ == CsgShapeHandleKind::Axis);
        CHECK(handles[i].axis_ == axes[i]);
        CHECK(IsNear(handles[i].position_, axes[i] * components[i]));
        CHECK(MeshHasVertexNear(mesh, handles[i].position_, 1e-5f));
    }
}

TEST_CASE("A sphere radius handle move grows the radius and the sanitizer clamps it", "[shapes][handles]")
{
    HandleFixture<CsgSphereBrush> f;
    f.brush->ClearDirty();
    const CsgSphereParams before = f.brush->GetParams();

    REQUIRE(f.brush->MoveHandles({0}, Vector3(0.25f, 5.0f, 5.0f)));
    CHECK(f.brush->GetRadius() == Vector3(0.75f, 0.5f, 0.5f));
    CHECK(f.brush->IsDirty());

    // The - handle grows the radius when it goes outward.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({3}, Vector3(0.0f, -0.25f, 0.0f)));
    CHECK(f.brush->GetRadius() == Vector3(0.5f, 0.75f, 0.5f));

    // The clamp.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({4}, Vector3(0.0f, 0.0f, -100.0f)));
    CHECK(f.brush->GetRadius().z_ == CsgSphereParams::MinRadius);
    f.brush->ClearDirty();
    CHECK_FALSE(f.brush->MoveHandles({4}, Vector3(0.0f, 0.0f, -100.0f)));
    CHECK_FALSE(f.brush->IsDirty());
    REQUIRE(f.brush->MoveHandles({4}, Vector3(0.0f, 0.0f, 1.0e9f)));
    CHECK(f.brush->GetRadius().z_ == CsgSphereParams::MaxRadius);

    // Restore and move again.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({0}, Vector3(0.25f, 0.0f, 0.0f)));
    const Vector3 once = f.brush->GetRadius();
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({0}, Vector3(0.25f, 0.0f, 0.0f)));
    CHECK(f.brush->GetRadius() == once);

    CHECK_FALSE(f.brush->MoveHandles({6}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(std::nanf(""), 0.0f, 0.0f)));
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("The cylinder brush has center, radius and bulge handles", "[shapes][handles]")
{
    HandleFixture<CsgCylinderBrush> f;
    auto handles = f.Handles();
    CHECK(GetIds(handles) == ea::vector<unsigned>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
    CHECK(FindHandle(handles, 0)->kind_ == CsgShapeHandleKind::Point);
    CHECK(FindHandle(handles, 1)->position_ == Vector3(0.0f, 0.5f, 0.0f));
    CHECK(FindHandle(handles, 2)->position_ == Vector3(0.5f, -0.5f, 0.0f));
    CHECK(FindHandle(handles, 3)->position_ == Vector3(-0.5f, -0.5f, 0.0f));
    CHECK(FindHandle(handles, 4)->position_ == Vector3(0.0f, -0.5f, 0.5f));
    CHECK(FindHandle(handles, 9)->position_ == Vector3(0.0f, 0.5f, -0.5f));
    CHECK(FindHandle(handles, 10)->position_ == Vector3(0.5f, 0.0f, 0.0f));
    CHECK(FindHandle(handles, 10)->axis_ == Vector3(1, 0, 0));
    CHECK(FindHandle(handles, 5)->axis_ == Vector3(0, 0, -1));

    // The handles are on the mesh. The middle ring needs 2 height segments.
    f.brush->SetHeightSegments(2);
    f.brush->SetBulge(0.5f);
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(f.brush->GetParams(), mesh));
    handles = f.Handles();
    for (const unsigned id : {2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u})
        CHECK(MeshHasVertexNear(mesh, FindHandle(handles, id)->position_, 1e-5f));
    CHECK(IsNear(FindHandle(handles, 10)->position_, Vector3(0.75f, 0.0f, 0.0f)));
}

TEST_CASE("Cylinder handle moves", "[shapes][handles]")
{
    HandleFixture<CsgCylinderBrush> f;
    const CsgCylinderParams before = f.brush->GetParams();

    SECTION("A center moves by the full delta")
    {
        REQUIRE(f.brush->MoveHandles({0}, Vector3(0.1f, -0.2f, 0.3f)));
        CHECK(IsNear(f.brush->GetBottomCenter(), before.bottomCenter_ + Vector3(0.1f, -0.2f, 0.3f)));
        CHECK(f.brush->GetTopCenter() == before.topCenter_);
        CHECK(f.brush->GetBottomRadius() == before.bottomRadius_);
        f.brush->SetParams(before);
        REQUIRE(f.brush->MoveHandles({1}, Vector3(0.1f, 0.2f, 0.3f)));
        CHECK(IsNear(f.brush->GetTopCenter(), before.topCenter_ + Vector3(0.1f, 0.2f, 0.3f)));
        CHECK(f.brush->GetBottomCenter() == before.bottomCenter_);
    }

    SECTION("The +X and -X handles of one end edit the same component")
    {
        REQUIRE(f.brush->MoveHandles({2}, Vector3(0.25f, 0.0f, 9.0f)));
        CHECK(f.brush->GetBottomRadius() == Vector2(0.75f, 0.5f));
        f.brush->SetParams(before);
        REQUIRE(f.brush->MoveHandles({3}, Vector3(-0.25f, 0.0f, 9.0f)));
        CHECK(f.brush->GetBottomRadius() == Vector2(0.75f, 0.5f));
        f.brush->SetParams(before);
        REQUIRE(f.brush->MoveHandles({9}, Vector3(9.0f, 0.0f, -0.25f)));
        CHECK(f.brush->GetTopRadius() == Vector2(0.5f, 0.75f));
        f.brush->SetParams(before);
        REQUIRE(f.brush->MoveHandles({8}, Vector3(9.0f, 0.0f, 0.25f)));
        CHECK(f.brush->GetTopRadius() == Vector2(0.5f, 0.75f));
        CHECK(f.brush->GetBottomRadius() == before.bottomRadius_);
    }

    SECTION("The bulge handle gives the expected bulge")
    {
        REQUIRE(f.brush->MoveHandles({10}, Vector3(0.25f, 3.0f, 3.0f)));
        CHECK(f.brush->GetBulge() == Catch::Approx(0.5f));
        f.brush->SetParams(before);
        REQUIRE(f.brush->MoveHandles({10}, Vector3(100.0f, 0.0f, 0.0f)));
        CHECK(f.brush->GetBulge() == CsgCylinderParams::MaxBulge);
    }

    SECTION("A move that makes the shape not valid returns false")
    {
        f.brush->ClearDirty();
        // The centers get the same height.
        CHECK_FALSE(f.brush->MoveHandles({1}, Vector3(0.0f, -1.0f, 0.0f)));
        CHECK(f.brush->GetTopCenter() == before.topCenter_);
        CHECK_FALSE(f.brush->IsDirty());
        CHECK_FALSE(f.brush->MoveHandles({1}, Vector3(std::nanf(""), 0.0f, 0.0f)));
        CHECK_FALSE(f.brush->MoveHandles({13}, Vector3(0.1f, 0.0f, 0.0f)));
    }

    SECTION("A radius at zero becomes the minimum of the pair rule")
    {
        REQUIRE(f.brush->MoveHandles({6}, Vector3(-0.5f, 0.0f, 0.0f)));
        CHECK(f.brush->GetTopRadius() == Vector2(CsgCylinderParams::MinRadius, 0.5f));
    }

    SECTION("The radius of a cone end has no handle and the ids stay stable")
    {
        CsgCylinderParams cone = before;
        cone.bottomRadius_ = Vector2::ZERO;
        f.brush->SetParams(cone);
        REQUIRE(f.brush->IsShapeValid());
        const auto handles = f.Handles();
        CHECK(GetIds(handles) == ea::vector<unsigned>{0, 1, 6, 7, 8, 9, 10, 11, 12});
        CHECK(FindHandle(handles, 10)->position_ == Vector3(0.25f, 0.0f, 0.0f));
        CHECK_FALSE(f.brush->MoveHandles({2}, Vector3(0.1f, 0.0f, 0.0f)));
        CHECK(f.brush->GetBottomRadius() == Vector2::ZERO);
        REQUIRE(f.brush->MoveHandles({6}, Vector3(0.1f, 0.0f, 0.0f)));
        CHECK(f.brush->GetTopRadius() == Vector2(0.6f, 0.5f));

        // The apex of the other end: the top.
        cone.bottomRadius_ = Vector2(0.5f, 0.5f);
        cone.topRadius_ = Vector2::ZERO;
        f.brush->SetParams(cone);
        CHECK(GetIds(f.Handles()) == ea::vector<unsigned>{0, 1, 2, 3, 4, 5, 10, 11, 12});
    }
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("The ring brush has 4 handles and the arc end matches the generated arc", "[shapes][handles]")
{
    HandleFixture<CsgRingBrush> f;
    f.brush->SetArc(90.0f);
    f.brush->SetRadius(0.8f);
    const auto handles = f.Handles();
    CHECK(GetIds(handles) == ea::vector<unsigned>{0, 1, 2, 3});
    CHECK(handles[0].position_ == Vector3(0.8f, 0.0f, 0.0f));
    CHECK(handles[1].position_ == Vector3(0.8f + 0.15f, 0.0f, 0.0f));
    CHECK(handles[2].position_ == Vector3(0.8f, 0.15f, 0.0f));
    CHECK(handles[2].axis_ == Vector3(0, 1, 0));

    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateRing(f.brush->GetParams(), mesh));
    // The arc end is the section center of the end cap.
    CHECK(MeshHasVertexNear(mesh, handles[3].position_, 1e-5f));
    CHECK(IsNear(handles[3].position_, Vector3(0.0f, 0.0f, 0.8f), 1e-5f));
    CHECK(handles[3].axis_.Length() == Catch::Approx(1.0f));
    CHECK(IsNear(handles[3].axis_, Vector3(-1.0f, 0.0f, 0.0f), 1e-5f));
    CHECK(handles[3].axis_.DotProduct(handles[3].position_) == Catch::Approx(0.0f).margin(1e-6));
    CHECK(MeshHasVertexNear(mesh, handles[1].position_, 1e-5f));
    CHECK(MeshHasVertexNear(mesh, handles[2].position_, 1e-5f));

    // The end cap normal follows the tangent direction of the handle.
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        if (IsNear(vertex.GetPosition(), handles[3].position_, 1e-5f))
        {
            CHECK(IsNear(vertex.GetNormal(), handles[3].axis_, 1e-5f));
            break;
        }
    }
}

TEST_CASE("Ring handle moves", "[shapes][handles]")
{
    HandleFixture<CsgRingBrush> f;
    f.brush->SetArc(90.0f);
    const CsgRingParams before = f.brush->GetParams();

    REQUIRE(f.brush->MoveHandles({0}, Vector3(0.1f, 4.0f, 4.0f)));
    CHECK(f.brush->GetRadius() == Catch::Approx(0.6f));
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({1}, Vector3(0.05f, 4.0f, 4.0f)));
    CHECK(f.brush->GetTubeRadius().x_ == Catch::Approx(0.2f));
    CHECK(f.brush->GetTubeRadius().y_ == Catch::Approx(0.15f));
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({2}, Vector3(4.0f, 0.05f, 4.0f)));
    CHECK(f.brush->GetTubeRadius().y_ == Catch::Approx(0.2f));
    CHECK(f.brush->GetTubeRadius().x_ == Catch::Approx(0.15f));

    // The arc end handle at 90 degrees has the axis (-1, 0, 0). A move of radius * 10 degrees in radians adds 10.
    f.brush->SetParams(before);
    const float step = before.radius_ * 10.0f * 3.14159265f / 180.0f;
    REQUIRE(f.brush->MoveHandles({3}, Vector3(-step, 0.0f, 0.0f)));
    CHECK(f.brush->GetArc() == Catch::Approx(100.0f).margin(1e-3));
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({3}, Vector3(100.0f, 0.0f, 0.0f)));
    CHECK(f.brush->GetArc() == CsgRingParams::MinArc);
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({3}, Vector3(-100.0f, 0.0f, 0.0f)));
    CHECK(f.brush->GetArc() == CsgRingParams::MaxArc);

    // The radial tube radius is clamped to the fraction of the radius.
    f.brush->SetParams(before);
    REQUIRE(f.brush->MoveHandles({1}, Vector3(100.0f, 0.0f, 0.0f)));
    CHECK(f.brush->GetTubeRadius().x_ == Catch::Approx(0.99f * f.brush->GetRadius()));

    // The full ring has no change if the arc grows.
    f.brush->SetParams(CsgRingParams());
    CHECK_FALSE(f.brush->MoveHandles({3}, Vector3(0.0f, 0.0f, 1.0f)));
    REQUIRE(f.brush->MoveHandles({3}, Vector3(0.0f, 0.0f, -0.5f * 3.14159265f / 2.0f)));
    CHECK(f.brush->GetArc() == Catch::Approx(270.0f).margin(1e-3));

    CHECK_FALSE(f.brush->MoveHandles({4}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(INFINITY, 0.0f, 0.0f)));
    CHECK(f.errors.GetCount() == 0);
}

namespace
{

/// Return the 8 world positions of the box corners.
ea::array<Vector3, 8> WorldCorners(const CsgBoxBrush* brush)
{
    ea::array<Vector3, 8> result;
    for (unsigned i = 0; i < 8; ++i)
        result[i] = brush->GetNode()->LocalToWorld(brush->GetSize() * brush->GetCorner(i));
    return result;
}

Vector3 SendRecenter(Component* brush, bool& accepted)
{
    using namespace CsgShapeRecenter;
    VariantMap& data = brush->GetEventDataMap();
    data[P_ACCEPTED] = false;
    data[P_OFFSET] = Vector3(7.0f, 7.0f, 7.0f);
    brush->SendEvent(E_CSGSHAPERECENTER, data);
    accepted = data[P_ACCEPTED].GetBool();
    return data[P_OFFSET].GetVector3();
}

/// Recenter a box after a face move. The node moves by the editor operation. Check the world corners.
void CheckBoxRecenter(HandleFixture<CsgBoxBrush>& f)
{
    const auto before = WorldCorners(f.brush);
    REQUIRE(f.brush->GetShapeModel() != nullptr);

    REQUIRE(RecenterShapePivot(f.brush));
    const auto after = WorldCorners(f.brush);
    for (unsigned i = 0; i < 8; ++i)
        CHECK(IsNear(before[i], after[i], 1e-4f));

    // The box is centered now: the mean of the local positions is zero.
    Vector3 mean;
    for (unsigned i = 0; i < 8; ++i)
        mean += f.brush->GetSize() * f.brush->GetCorner(i);
    CHECK(IsNear(mean / 8.0f, Vector3::ZERO, 1e-5f));
    CHECK(f.brush->GetShapeModel() != nullptr);
    CHECK_FALSE(RecenterShapePivot(f.brush));
}

} // namespace

TEST_CASE("Box recenter keeps the world corners", "[shapes][handles][recenter]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->SetSize(Vector3(2.0f, 1.0f, 4.0f));
    // The +X face moves by 1.5 in the local space: the center goes to x = 0.75.
    REQUIRE(f.brush->MoveHandles({21}, Vector3(1.5f, 0.0f, 0.0f)));
    const Vector3 expected(0.75f, 0.0f, 0.0f);
    Node* node = f.brush->GetNode();

    SECTION("Offset and event round trip")
    {
        const CsgBoxParams old = f.brush->GetParams();
        bool accepted = false;
        const Vector3 offset = SendRecenter(f.brush, accepted);
        CHECK(accepted);
        CHECK(IsNear(offset, expected, 1e-5f));
        CHECK(f.brush->GetParams().size_ == old.size_);
        // Centered: the corners go by -offset / size.
        CHECK(IsNear(f.brush->GetCorner(1), old.corners_[1] - expected / old.size_, 1e-6f));
        // The second call has nothing to do and leaves the offset at zero.
        const CsgBoxParams centered = f.brush->GetParams();
        const Vector3 second = SendRecenter(f.brush, accepted);
        CHECK_FALSE(accepted);
        CHECK(second == Vector3::ZERO);
        CHECK(f.brush->GetParams().corners_ == centered.corners_);
    }

    SECTION("Identity node")
    {
        CheckBoxRecenter(f);
        CHECK(IsNear(node->GetPosition(), expected, 1e-5f));
    }

    SECTION("Rotated node")
    {
        node->SetPosition(Vector3(1.0f, 2.0f, 3.0f));
        node->SetRotation(Quaternion(30.0f, 50.0f, 70.0f));
        CheckBoxRecenter(f);
        CHECK(IsNear(node->GetPosition(), Vector3(1.0f, 2.0f, 3.0f) + node->GetRotation() * expected, 1e-4f));
    }

    SECTION("Node scale")
    {
        node->SetScale(Vector3(1.0f, 2.0f, 3.0f));
        node->SetRotation(Quaternion(20.0f, Vector3::UP));
        CheckBoxRecenter(f);
        CHECK(IsNear(node->GetPosition(), node->GetRotation() * (Vector3(1.0f, 2.0f, 3.0f) * expected), 1e-4f));
    }

    SECTION("Rotated and scaled parent")
    {
        Node* parent = f.scene->CreateChild("Parent");
        parent->SetScale(Vector3(2.0f, 1.0f, 1.0f));
        parent->SetRotation(Quaternion(40.0f, Vector3(1.0f, 1.0f, 0.0f)));
        parent->SetPosition(Vector3(5.0f, -1.0f, 2.0f));
        node->SetParent(parent);
        node->SetScale(Vector3(1.0f, 2.0f, 3.0f));
        node->SetRotation(Quaternion(25.0f, Vector3::FORWARD));
        CheckBoxRecenter(f);
    }
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("Recenter after an edge move keeps world positions and the events work for edges",
    "[shapes][handles][box][edges]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->GetNode()->SetRotation(Quaternion(20.0f, Vector3(0, 1, 1)));
    f.brush->GetNode()->SetPosition(Vector3(1, 2, 3));
    REQUIRE(f.brush->MoveHandles({32}, Vector3(0.4f, 0.3f, -0.2f)));
    CheckBoxRecenter(f);

    // The event round trip.
    HandleFixture<CsgBoxBrush>& g = f;
    g.brush->SetParams(CsgBoxParams());
    const VariantMap data = Query(g.brush);
    CHECK(data.at(CsgShapeQueryHandles::P_HANDLES).GetVariantVector().size() == 29);
    CHECK(SendMove(g.brush, {37}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK(CountChangedCorners(CsgBoxParams(), g.brush->GetParams()) == 2);
    CHECK_FALSE(SendMove(g.brush, {37}, Vector3(0.0f, 0.0f, 0.0f)));
    CHECK_FALSE(SendMove(g.brush, {42}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(SendMove(g.brush, {26}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK(g.errors.GetCount() == 0);
}

TEST_CASE("Box recenter keeps the shape valid and tilted boxes work", "[shapes][handles][recenter]")
{
    HandleFixture<CsgBoxBrush> f;
    // A tilted box: the top corners go along X.
    CsgBoxParams params;
    for (unsigned i : {4u, 5u, 6u, 7u})
        params.corners_[i] += Vector3(0.3f, 0.0f, 0.2f);
    params.corners_[7] += Vector3(0.0f, 0.2f, 0.0f);
    f.brush->SetParams(params);
    f.brush->GetNode()->SetRotation(Quaternion(15.0f, 25.0f, 35.0f));
    f.brush->GetNode()->SetPosition(Vector3(1.0f, 1.0f, 1.0f));
    const auto before = WorldCorners(f.brush);
    Vector3 expected;
    for (unsigned i = 0; i < 8; ++i)
        expected += params.size_ * params.corners_[i];
    expected /= 8.0f;

    bool accepted = false;
    const Vector3 offset = SendRecenter(f.brush, accepted);
    REQUIRE(accepted);
    CHECK(IsNear(offset, expected, 1e-5f));
    f.brush->GetNode()->SetPosition(f.brush->GetNode()->GetTransform() * offset);
    const auto after = WorldCorners(f.brush);
    for (unsigned i = 0; i < 8; ++i)
        CHECK(IsNear(before[i], after[i], 1e-4f));
    CHECK(f.brush->IsShapeValid());

    SECTION("A shape that is not valid returns false and keeps the parameters")
    {
        // The box has an off center and it is not valid: the corners 0 and 4 are on the wrong side.
        CsgBoxParams bad;
        bad.corners_[4] = Vector3(-0.5f, -0.5f, -0.5f);
        bad.corners_[0] = Vector3(-0.5f, -0.5f, 0.5f);
        for (Vector3& corner : bad.corners_)
            corner += Vector3(1.0f, 0.0f, 0.0f);
        f.brush->SetParams(bad);
        REQUIRE_FALSE(f.brush->IsShapeValid());
        const CsgBoxParams old = f.brush->GetParams();
        const Vector3 position = f.brush->GetNode()->GetPosition();
        SendRecenter(f.brush, accepted);
        CHECK_FALSE(accepted);
        CHECK(f.brush->GetParams().corners_ == old.corners_);
        CHECK_FALSE(RecenterShapePivot(f.brush));
        CHECK(f.brush->GetNode()->GetPosition() == position);
    }

    SECTION("A tiny center is not a recenter")
    {
        CsgBoxParams tiny;
        tiny.corners_[7] += Vector3(1e-8f, 0.0f, 0.0f);
        f.brush->SetParams(tiny);
        const CsgBoxParams old = f.brush->GetParams();
        SendRecenter(f.brush, accepted);
        CHECK_FALSE(accepted);
        CHECK(f.brush->GetParams().corners_ == old.corners_);
    }
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("Cylinder recenter", "[shapes][handles][recenter]")
{
    HandleFixture<CsgCylinderBrush> f;
    Node* node = f.brush->GetNode();
    node->SetRotation(Quaternion(20.0f, 30.0f, 40.0f));
    node->SetScale(Vector3(1.0f, 2.0f, 3.0f));
    node->SetPosition(Vector3(0.0f, 1.0f, 0.0f));

    // A tilted cylinder: both centers move off the origin.
    CsgCylinderParams params = f.brush->GetParams();
    params.bottomCenter_ = Vector3(1.0f, 0.0f, 0.5f);
    params.topCenter_ = Vector3(2.0f, 2.0f, 0.5f);
    f.brush->SetParams(params);
    const Vector3 expected(1.5f, 1.0f, 0.5f);
    const Vector3 bottomWorld = node->LocalToWorld(f.brush->GetParams().bottomCenter_);
    const Vector3 topWorld = node->LocalToWorld(f.brush->GetParams().topCenter_);
    const float radius = f.brush->GetParams().bottomRadius_.x_;

    bool accepted = false;
    const Vector3 offset = SendRecenter(f.brush, accepted);
    REQUIRE(accepted);
    CHECK(IsNear(offset, expected, 1e-5f));
    node->SetPosition(node->GetTransform() * offset);
    CHECK(IsNear(node->LocalToWorld(f.brush->GetParams().bottomCenter_), bottomWorld, 1e-4f));
    CHECK(IsNear(node->LocalToWorld(f.brush->GetParams().topCenter_), topWorld, 1e-4f));
    CHECK(f.brush->GetParams().bottomRadius_.x_ == radius);
    CHECK(IsNear(f.brush->GetParams().bottomCenter_ + f.brush->GetParams().topCenter_, Vector3::ZERO, 1e-5f));

    SendRecenter(f.brush, accepted);
    CHECK_FALSE(accepted);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("Sphere and ring recenter return false", "[shapes][handles][recenter]")
{
    bool accepted = true;
    {
        HandleFixture<CsgSphereBrush> f;
        const Variant before = f.brush->GetAttribute("Radius");
        Vector3 offset(1.0f, 1.0f, 1.0f);
        CHECK_FALSE(f.brush->Recenter(offset));
        SendRecenter(f.brush, accepted);
        CHECK_FALSE(accepted);
        CHECK(f.brush->GetAttribute("Radius") == before);
        CHECK_FALSE(RecenterShapePivot(f.brush));
    }
    {
        HandleFixture<CsgRingBrush> f;
        SendRecenter(f.brush, accepted);
        CHECK_FALSE(accepted);
        CHECK_FALSE(RecenterShapePivot(f.brush));
    }
}

TEST_CASE("The fold in a root is the same after a recenter", "[shapes][handles][recenter][root]")
{
    HandleFixture<CsgBoxBrush> f;
    auto* workQueue = f.context->GetSubsystem<WorkQueue>();
    auto* time = f.context->GetSubsystem<Time>();
    SharedPtr<Model> base = MakeTestBoxModel(f.context, Vector3::ONE);
    base->SetName("Csg/Base.mdl");
    f.context->GetSubsystem<ResourceCache>()->AddManualResource(base);

    Node* rootNode = f.scene->CreateChild("Root");
    rootNode->SetScale(Vector3(2.0f, 1.0f, 1.0f));
    rootNode->SetRotation(Quaternion(30.0f, Vector3(1.0f, 1.0f, 0.0f)));
    auto* root = rootNode->CreateComponent<CsgRoot>();
    root->SetModel(ResourceRef(Model::GetTypeStatic(), "Csg/Base.mdl"));
    root->SetDeferredDelaySec(0.0f);
    Node* brushNode = f.brush->GetNode();
    brushNode->SetParent(rootNode);
    brushNode->SetScale(Vector3(0.5f, 1.0f, 2.0f));
    brushNode->SetRotation(Quaternion(20.0f, Vector3::UP));
    brushNode->SetPosition(Vector3(0.5f, 0.2f, 0.1f));

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
    REQUIRE(f.brush->MoveHandles({21}, Vector3(1.0f, 0.0f, 0.0f)));
    run();
    REQUIRE(root->GetLastBuildStats().valid_);
    const double volumeBefore = root->GetLastBuildStats().volume_;
    const BoundingBox boundsBefore = root->GetResultModel()->GetModel()->GetBoundingBox();

    REQUIRE(RecenterShapePivot(f.brush));
    run();
    REQUIRE(root->GetLastBuildStats().valid_);
    CHECK(root->GetLastBuildStats().volume_ == Catch::Approx(volumeBefore).epsilon(1e-3));
    const BoundingBox boundsAfter = root->GetResultModel()->GetModel()->GetBoundingBox();
    CHECK(IsNear(boundsAfter.min_, boundsBefore.min_, 1e-2f));
    CHECK(IsNear(boundsAfter.max_, boundsBefore.max_, 1e-2f));
    CHECK(f.errors.GetCount() == 0);
}

namespace
{

/// Return true if the point lies on a triangle of the mesh, within eps.
bool IsPointOnMesh(const CsgShapeMesh& mesh, const Vector3& point, float eps = 1e-5f)
{
    for (unsigned i = 0; i + 2 < mesh.indices_.size(); i += 3)
    {
        const Vector3 a = mesh.vertices_[mesh.indices_[i]].GetPosition();
        const Vector3 b = mesh.vertices_[mesh.indices_[i + 1]].GetPosition();
        const Vector3 c = mesh.vertices_[mesh.indices_[i + 2]].GetPosition();
        const Vector3 n = (b - a).CrossProduct(c - a);
        const float area2 = n.Length();
        if (area2 <= 0.0f)
            continue;
        if (Abs((point - a).DotProduct(n) / area2) > eps)
            continue;
        // Barycentric test with a tolerance on each edge.
        const auto outside = [&](const Vector3& p0, const Vector3& p1)
        {
            const Vector3 edge = p1 - p0;
            return (edge.CrossProduct(point - p0)).DotProduct(n) / area2 < -eps * edge.Length();
        };
        if (!outside(a, b) && !outside(b, c) && !outside(c, a))
            return true;
    }
    return false;
}

/// Check the 6 resize handles of a box-shaped procedural brush (wedge, stairs).
/// The ids are 0 -X, 1 +X, 2 -Z, 3 +Z, 4 +Y, 5 -Y. The opposite handle of id is id ^ 1.
template <class T, class G> void CheckBoxResizeHandles(HandleFixture<T>& f, G generate)
{
    Node* node = f.brush->GetNode();
    node->SetPosition(Vector3(1.0f, -2.0f, 0.5f));
    node->SetRotation(Quaternion(20.0f, 30.0f, 40.0f));
    node->SetScale(Vector3(1.0f, 2.0f, 0.5f));
    const auto start = f.brush->GetParams();
    const auto startHandles = f.Handles();
    REQUIRE(startHandles.size() >= 6);
    for (unsigned i = 0; i < 6; ++i)
        REQUIRE(startHandles[i].id_ == i);
    const Vector3 axes[6] = {Vector3::LEFT, Vector3::RIGHT, Vector3::BACK, Vector3::FORWARD, Vector3::UP, Vector3::DOWN};
    const unsigned axisIndex[6] = {0, 0, 2, 2, 1, 1};

    // Each handle is on the generated shape.
    CsgShapeMesh mesh;
    REQUIRE(generate(start, mesh));
    for (const CsgShapeHandle& handle : startHandles)
    {
        INFO("Handle " << handle.id_);
        CHECK(handle.kind_ == CsgShapeHandleKind::Axis);
        if (handle.id_ >= 6)
            continue; // The size handles have their own test.
        CHECK(handle.axis_ == axes[handle.id_]);
        CHECK(IsPointOnMesh(mesh, handle.position_));
    }

    for (unsigned id = 0; id < 6; ++id)
    {
        INFO("Handle " << id);
        f.brush->SetParams(start);
        const unsigned opposite = id ^ 1u;
        // A part of the delta across the axis does nothing.
        const Vector3 across = axes[id].CrossProduct(Vector3(0.3f, 0.5f, 0.7f));
        REQUIRE(f.brush->MoveHandles({id}, axes[id] * 0.25f + across));
        const auto moved = f.brush->GetParams();
        CHECK(moved.size_.Data()[axisIndex[id]] == Catch::Approx(start.size_.Data()[axisIndex[id]] + 0.25f));
        const auto handles = f.Handles();
        // The opposite face stays fixed in the world. The handle follows the cursor on its axis.
        CHECK(IsNear(node->LocalToWorld(handles[opposite].position_),
            node->LocalToWorld(startHandles[opposite].position_), 1e-5f));
        CHECK(IsNear(handles[id].position_, startHandles[id].position_ + axes[id] * 0.25f, 1e-5f));

        // Stateless: the same delta after a restore gives the same parameters.
        f.brush->SetParams(start);
        REQUIRE(f.brush->MoveHandles({id, id}, axes[id] * 0.25f + across));
        CHECK(f.brush->GetParams().size_ == moved.size_);
        CHECK(f.brush->GetParams().center_ == moved.center_);

        // A shrink below the minimum clamps the size and keeps the opposite face.
        f.brush->SetParams(start);
        REQUIRE(f.brush->MoveHandles({id}, axes[id] * -100.0f));
        CHECK(f.brush->GetParams().size_.Data()[axisIndex[id]] == Catch::Approx(0.001f));
        CHECK(IsNear(node->LocalToWorld(f.Handles()[opposite].position_),
            node->LocalToWorld(startHandles[opposite].position_), 1e-4f));
        CHECK(f.brush->IsShapeValid());
    }

    // Two handles in one move: both faces move.
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({1, 4}, Vector3(0.5f, 0.5f, 0.0f)));
    CHECK(f.brush->GetParams().size_.x_ == Catch::Approx(start.size_.x_ + 0.5f));
    CHECK(f.brush->GetParams().size_.y_ == Catch::Approx(start.size_.y_ + 0.5f));

    // Moves that are not valid or give no change return false and keep the parameters.
    f.brush->SetParams(start);
    CHECK_FALSE(f.brush->MoveHandles({9}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(INFINITY, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(0.0f, 0.3f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({}, Vector3(0.3f, 0.0f, 0.0f)));
    CHECK(f.brush->GetParams().size_ == start.size_);
    CHECK(f.brush->GetParams().center_ == start.center_);

    // The events give the same handles.
    const VariantMap data = Query(f.brush);
    CHECK(data.find(CsgShapeQueryHandles::P_VALID)->second.GetBool());
    CHECK(data.find(CsgShapeQueryHandles::P_HANDLES)->second.GetVariantVector().size() == 9);
    CHECK(SendMove(f.brush, {3}, Vector3(0.0f, 0.0f, 0.5f)));
    CHECK(f.brush->GetParams().size_.z_ == Catch::Approx(start.size_.z_ + 0.5f));
    CHECK(f.errors.GetCount() == 0);
}

/// Move a face, then recenter: the world positions of the handles stay. The center becomes zero.
template <class T> void CheckCenterRecenter(HandleFixture<T>& f)
{
    Node* node = f.brush->GetNode();
    node->SetPosition(Vector3(1.0f, 2.0f, 3.0f));
    node->SetRotation(Quaternion(30.0f, 50.0f, 70.0f));
    node->SetScale(Vector3(1.0f, 2.0f, 3.0f));
    REQUIRE(f.brush->MoveHandles({1}, Vector3(1.5f, 0.0f, 0.0f)));
    const Vector3 expected = f.brush->GetParams().center_;
    REQUIRE(IsNear(expected, Vector3(0.75f, 0.0f, 0.0f), 1e-5f));
    const auto before = f.Handles();
    ea::vector<Vector3> worldBefore;
    for (const CsgShapeHandle& handle : before)
        worldBefore.push_back(node->LocalToWorld(handle.position_));

    bool accepted = false;
    const Vector3 offset = SendRecenter(f.brush, accepted);
    REQUIRE(accepted);
    CHECK(offset == expected);
    CHECK(f.brush->GetParams().center_ == Vector3::ZERO);
    node->SetPosition(node->GetTransform() * offset);
    const auto after = f.Handles();
    for (unsigned i = 0; i < after.size(); ++i)
        CHECK(IsNear(node->LocalToWorld(after[i].position_), worldBefore[i], 1e-4f));
    CHECK(f.brush->IsShapeValid());

    // The second call has nothing to do.
    SendRecenter(f.brush, accepted);
    CHECK_FALSE(accepted);
    CHECK_FALSE(RecenterShapePivot(f.brush));

    // The editor operation moves the node.
    auto params = f.brush->GetParams();
    params.center_ = Vector3(0.0f, 0.5f, -0.25f);
    f.brush->SetParams(params);
    const Vector3 worldHandle = node->LocalToWorld(f.Handles()[0].position_);
    REQUIRE(RecenterShapePivot(f.brush));
    CHECK(IsNear(node->LocalToWorld(f.Handles()[0].position_), worldHandle, 1e-4f));
    CHECK(f.errors.GetCount() == 0);
}

} // namespace

TEST_CASE("The wedge brush has 6 resize handles that keep the opposite face", "[shapes][handles]")
{
    HandleFixture<CsgWedgeBrush> f;
    CsgWedgeParams params;
    params.size_ = Vector3(2.0f, 1.0f, 3.0f);
    params.center_ = Vector3(0.5f, -0.25f, 1.0f);
    f.brush->SetParams(params);
    const auto handles = f.Handles();
    REQUIRE(handles.size() == 9);
    CHECK(IsNear(handles[0].position_, Vector3(-0.5f, -0.25f - 0.5f / 3.0f, 1.0f + 0.5f), 1e-5f));
    CHECK(IsNear(handles[2].position_, Vector3(0.5f, -0.75f, -0.5f), 1e-5f));
    CHECK(IsNear(handles[3].position_, Vector3(0.5f, -0.25f, 2.5f), 1e-5f));
    CHECK(IsNear(handles[4].position_, Vector3(0.5f, 0.25f, 2.5f), 1e-5f));
    CHECK(IsNear(handles[5].position_, Vector3(0.5f, -0.75f, 1.0f), 1e-5f));
    CheckBoxResizeHandles(f, [](const CsgWedgeParams& p, CsgShapeMesh& mesh) { return CsgGenerateWedge(p, mesh); });
}

TEST_CASE("Wedge recenter keeps the world positions", "[shapes][handles][recenter]")
{
    HandleFixture<CsgWedgeBrush> f;
    CheckCenterRecenter(f);
}

TEST_CASE("The stairs brush has 6 resize handles that keep the opposite face", "[shapes][handles]")
{
    for (const int steps : {1, 4, 64})
    {
        INFO("Steps " << steps);
        HandleFixture<CsgStairsBrush> f;
        CsgStairsParams params;
        params.size_ = Vector3(2.0f, 1.0f, 3.0f);
        params.center_ = Vector3(0.5f, -0.25f, 1.0f);
        params.steps_ = steps;
        f.brush->SetParams(params);
        const auto handles = f.Handles();
        REQUIRE(handles.size() == 9);
        CHECK(IsNear(handles[0].position_, Vector3(-0.5f, -0.25f, 1.0f), 1e-5f));
        CHECK(IsNear(handles[2].position_, Vector3(0.5f, -0.75f + 0.5f / steps, -0.5f), 1e-5f));
        CHECK(IsNear(handles[4].position_, Vector3(0.5f, 0.25f, 2.5f - 1.5f / steps), 1e-5f));
        CheckBoxResizeHandles(
            f, [](const CsgStairsParams& p, CsgShapeMesh& mesh) { return CsgGenerateStairs(p, mesh); });
        CHECK(f.brush->GetSteps() == steps);
    }
}

TEST_CASE("Stairs recenter keeps the world positions", "[shapes][handles][recenter]")
{
    HandleFixture<CsgStairsBrush> f;
    CheckCenterRecenter(f);
    CHECK(f.brush->GetSteps() == 5);
}

TEST_CASE("The arch brush has 5 handles on the shape", "[shapes][handles]")
{
    for (const float arc : {180.0f, 120.0f, 1.0f})
    {
        for (const float thickness : {0.25f, 1.0f})
        {
            INFO("Arc " << arc << " thickness " << thickness);
            HandleFixture<CsgArchBrush> f;
            CsgArchParams params;
            params.arc_ = arc;
            params.thickness_ = thickness;
            params.center_ = Vector3(0.5f, -1.0f, 2.0f);
            f.brush->SetParams(params);
            const auto handles = f.Handles();
            REQUIRE(GetIds(handles) == ea::vector<unsigned>{0, 1, 2, 3, 4});
            const float inner = thickness == 1.0f ? 0.0f : 0.75f;
            CHECK(IsNear(handles[1].position_, Vector3(0.5f, 0.0f, 2.0f)));
            CHECK(IsNear(handles[2].position_, Vector3(0.5f, -1.0f + inner, 2.0f)));
            CHECK(handles[2].axis_ == Vector3::DOWN);
            CHECK(IsNear(handles[3].position_, Vector3(0.5f, -1.0f + (1.0f + inner) * 0.5f, 2.5f)));
            CHECK(handles[3].axis_ == Vector3::FORWARD);
            const float a1 = (90.0f + arc * 0.5f) * 3.14159265f / 180.0f;
            CHECK(IsNear(handles[4].position_, Vector3(0.5f + std::cos(a1), -1.0f + std::sin(a1), 2.0f)));
            CHECK(IsNear(handles[4].axis_, Vector3(-std::sin(a1), std::cos(a1), 0.0f)));
            CsgShapeMesh mesh;
            REQUIRE(CsgGenerateArch(f.brush->GetParams(), mesh));
            for (const CsgShapeHandle& handle : handles)
            {
                INFO("Handle " << handle.id_);
                CHECK(handle.kind_ == CsgShapeHandleKind::Axis);
                CHECK(handle.axis_.Length() == Catch::Approx(1.0f));
                // The default 16 segments are even, so the top handles are on the mesh too.
                CHECK(IsPointOnMesh(mesh, handle.position_));
            }
            CHECK(f.errors.GetCount() == 0);
        }
    }
}

TEST_CASE("Arch handle moves", "[shapes][handles]")
{
    HandleFixture<CsgArchBrush> f;
    Node* node = f.brush->GetNode();
    node->SetRotation(Quaternion(10.0f, 20.0f, 30.0f));
    node->SetPosition(Vector3(1.0f, 2.0f, 3.0f));
    CsgArchParams start;
    start.arc_ = 120.0f;
    start.center_ = Vector3(0.0f, 0.5f, 0.25f);
    f.brush->SetParams(start);
    const auto startHandles = f.Handles();

    REQUIRE(f.brush->MoveHandles({0}, Vector3(0.2f, 4.0f, 4.0f)));
    CHECK(f.brush->GetRadius().x_ == Catch::Approx(1.2f));
    CHECK(f.brush->GetRadius().y_ == 1.0f);
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({1}, Vector3(4.0f, 0.3f, 4.0f)));
    CHECK(f.brush->GetRadius().y_ == Catch::Approx(1.3f));
    CHECK(IsNear(f.Handles()[1].position_, startHandles[1].position_ + Vector3(0.0f, 0.3f, 0.0f)));

    // The thickness grows when the handle moves into the hole (down).
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({2}, Vector3(0.0f, -0.1f, 0.0f)));
    CHECK(f.brush->GetThickness() == Catch::Approx(0.35f));
    CHECK(IsNear(f.Handles()[2].position_, startHandles[2].position_ + Vector3(0.0f, -0.1f, 0.0f)));
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({2}, Vector3(0.0f, -100.0f, 0.0f)));
    CHECK(f.brush->GetThickness() == 1.0f);
    CHECK(CsgGetArchInnerRadius(f.brush->GetParams()) == Vector2::ZERO);
    CHECK(f.brush->IsShapeValid());

    // The depth handle keeps the back face fixed in the world.
    f.brush->SetParams(start);
    const Vector3 backBefore = node->LocalToWorld(Vector3(0.0f, 0.0f, start.center_.z_ - start.depth_ * 0.5f));
    REQUIRE(f.brush->MoveHandles({3}, Vector3(0.0f, 0.0f, 0.5f)));
    CHECK(f.brush->GetDepth() == Catch::Approx(1.5f));
    CHECK(f.brush->GetCenter().z_ == Catch::Approx(0.5f));
    const CsgArchParams& moved = f.brush->GetParams();
    CHECK(IsNear(node->LocalToWorld(Vector3(0.0f, 0.0f, moved.center_.z_ - moved.depth_ * 0.5f)), backBefore));
    CHECK(IsNear(f.Handles()[3].position_, startHandles[3].position_ + Vector3(0.0f, 0.0f, 0.5f)));
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({3}, Vector3(0.0f, 0.0f, -100.0f)));
    CHECK(f.brush->GetDepth() == CsgArchParams::MinDepth);
    CHECK(IsNear(node->LocalToWorld(Vector3(0.0f, 0.0f, f.brush->GetCenter().z_ - f.brush->GetDepth() * 0.5f)),
        backBefore, 1e-4f));

    // The arc end: a move of radius * 10 degrees in radians along the tangent moves each end by 10 degrees.
    f.brush->SetParams(start);
    const float step = 10.0f * 3.14159265f / 180.0f;
    REQUIRE(f.brush->MoveHandles({4}, startHandles[4].axis_ * step));
    CHECK(f.brush->GetArc() == Catch::Approx(140.0f).margin(1e-3));
    const float a1 = 160.0f * 3.14159265f / 180.0f;
    CHECK(IsNear(f.Handles()[4].position_, start.center_ + Vector3(std::cos(a1), std::sin(a1), 0.0f), 1e-5f));
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({4}, startHandles[4].axis_ * 100.0f));
    CHECK(f.brush->GetArc() == CsgArchParams::MaxArc);
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({4}, startHandles[4].axis_ * -100.0f));
    CHECK(f.brush->GetArc() == CsgArchParams::MinArc);

    // Stateless: the same delta after a restore gives the same parameters.
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({0, 4, 0}, Vector3(0.1f, 0.1f, 0.0f)));
    const CsgArchParams once = f.brush->GetParams();
    f.brush->SetParams(start);
    REQUIRE(f.brush->MoveHandles({0, 4}, Vector3(0.1f, 0.1f, 0.0f)));
    CHECK(f.brush->GetParams().radius_ == once.radius_);
    CHECK(f.brush->GetParams().arc_ == once.arc_);

    // Moves that are not valid or give no change return false and keep the parameters.
    f.brush->SetParams(start);
    CHECK_FALSE(f.brush->MoveHandles({5}, Vector3(0.1f, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({0}, Vector3(INFINITY, 0.0f, 0.0f)));
    CHECK_FALSE(f.brush->MoveHandles({3}, Vector3(1.0f, 1.0f, 0.0f)));
    CHECK(f.brush->GetArc() == 120.0f);
    CHECK(f.brush->GetDepth() == 1.0f);
    CHECK(SendMove(f.brush, {1}, Vector3(0.0f, 0.5f, 0.0f)));
    CHECK(f.brush->GetRadius().y_ == Catch::Approx(1.5f));
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("Arch recenter keeps the world positions", "[shapes][handles][recenter]")
{
    HandleFixture<CsgArchBrush> f;
    Node* node = f.brush->GetNode();
    node->SetPosition(Vector3(1.0f, 2.0f, 3.0f));
    node->SetRotation(Quaternion(30.0f, 50.0f, 70.0f));
    node->SetScale(Vector3(1.0f, 2.0f, 3.0f));
    REQUIRE(f.brush->MoveHandles({3}, Vector3(0.0f, 0.0f, 1.0f)));
    CsgArchParams params = f.brush->GetParams();
    params.center_ += Vector3(0.25f, -0.5f, 0.0f);
    f.brush->SetParams(params);
    const Vector3 expected(0.25f, -0.5f, 0.5f);
    REQUIRE(IsNear(f.brush->GetCenter(), expected));
    ea::vector<Vector3> worldBefore;
    for (const CsgShapeHandle& handle : f.Handles())
        worldBefore.push_back(node->LocalToWorld(handle.position_));

    bool accepted = false;
    const Vector3 offset = SendRecenter(f.brush, accepted);
    REQUIRE(accepted);
    CHECK(IsNear(offset, expected));
    CHECK(f.brush->GetCenter() == Vector3::ZERO);
    node->SetPosition(node->GetTransform() * offset);
    const auto after = f.Handles();
    for (unsigned i = 0; i < after.size(); ++i)
        CHECK(IsNear(node->LocalToWorld(after[i].position_), worldBefore[i], 1e-4f));
    SendRecenter(f.brush, accepted);
    CHECK_FALSE(accepted);
    CHECK_FALSE(RecenterShapePivot(f.brush));
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("The cylinder brush saves and loads Start Angle and Face Radius, an old scene gets the defaults",
    "[shapes][procedural]")
{
    HandleFixture<CsgCylinderBrush> f;
    f.brush->SetStartAngle(-30.0f);
    f.brush->SetFaceRadius(true);
    CHECK(f.brush->GetStartAngle() == 330.0f);
    CHECK(f.brush->GetFaceRadius());
    f.brush->SetStartAngle(std::numeric_limits<float>::quiet_NaN());
    CHECK(f.brush->GetStartAngle() == 0.0f);
    f.brush->SetStartAngle(30.0f);

    VectorBuffer buffer;
    REQUIRE(f.scene->SaveXML(buffer));
    buffer.Seek(0);
    const ea::string text = buffer.ReadString();
    CHECK(text.contains("Start Angle"));
    CHECK(text.contains("Face Radius"));

    SECTION("A round trip keeps the values")
    {
        buffer.Seek(0);
        auto loaded = MakeShared<Scene>(f.context);
        REQUIRE(loaded->LoadXML(buffer));
        auto* brush = loaded->GetChild("Brush")->GetComponent<CsgCylinderBrush>();
        REQUIRE(brush);
        CHECK(brush->GetStartAngle() == 30.0f);
        CHECK(brush->GetFaceRadius());
    }

    SECTION("An old scene without the attributes gets start 0 and no face radius")
    {
        ea::string old;
        for (const ea::string& line : text.split('\n', true))
        {
            if (!line.contains("Start Angle") && !line.contains("Face Radius"))
                old += line + "\n";
        }
        CHECK_FALSE(old.contains("Start Angle"));
        CHECK_FALSE(old.contains("Face Radius"));
        VectorBuffer oldBuffer;
        oldBuffer.Write(old.c_str(), old.length());
        oldBuffer.Seek(0);
        auto loaded = MakeShared<Scene>(f.context);
        REQUIRE(loaded->LoadXML(oldBuffer));
        auto* brush = loaded->GetChild("Brush")->GetComponent<CsgCylinderBrush>();
        REQUIRE(brush);
        CHECK(brush->GetStartAngle() == 0.0f);
        CHECK_FALSE(brush->GetFaceRadius());
    }
    CHECK(f.errors.GetCount() == 0);
}

namespace
{

/// The distance from the XZ point to the closed polygon (a ring of XZ points).
float DistanceToPolygon(const Vector2& p, const ea::vector<Vector2>& ring)
{
    float best = 1e9f;
    for (unsigned i = 0; i < ring.size(); ++i)
    {
        const Vector2 a = ring[i];
        const Vector2 b = ring[(i + 1) % ring.size()];
        const Vector2 ab = b - a;
        const float t = Clamp((p - a).DotProduct(ab) / ab.DotProduct(ab), 0.0f, 1.0f);
        best = ea::min(best, (a + ab * t - p).Length());
    }
    return best;
}

/// The XZ positions of the corners of one end ring, in the order of the angle.
ea::vector<Vector2> GetEndRing(const CsgCylinderParams& params, bool top)
{
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(params, mesh));
    ea::vector<ea::pair<float, Vector2>> corners;
    for (unsigned i = 0; i < mesh.groups_[0].indexCount_; ++i)
    {
        const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i]];
        if (vertex.uv_[0].y_ != (top ? 1.0f : 0.0f) || vertex.uv_[0].x_ >= 1.0f)
            continue;
        bool known = false;
        for (const auto& corner : corners)
            known = known || corner.first == vertex.uv_[0].x_;
        if (!known)
            corners.push_back({vertex.uv_[0].x_, Vector2(vertex.GetPosition().x_, vertex.GetPosition().z_)});
    }
    ea::sort(corners.begin(), corners.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    ea::vector<Vector2> ring;
    for (const auto& corner : corners)
        ring.push_back(corner.second);
    return ring;
}

} // namespace

TEST_CASE("Cylinder radius and scale handles lie on the polygon boundary", "[shapes][handles]")
{
    for (const int segments : {3, 4, 5, 6, 8, 24})
    {
        for (const float start : {0.0f, 45.0f, 17.0f})
        {
            for (const bool face : {false, true})
            {
                HandleFixture<CsgCylinderBrush> f;
                CsgCylinderParams params;
                params.segments_ = segments;
                params.startAngle_ = start;
                params.faceRadius_ = face;
                params.bottomRadius_ = Vector2(0.75f, 0.5f);
                params.topRadius_ = Vector2(0.3f, 0.6f);
                f.brush->SetParams(params);
                INFO("Segments " << segments << " start " << start << " face " << face);
                const auto handles = f.Handles();
                REQUIRE(handles.size() == 13);
                for (const bool top : {false, true})
                {
                    const ea::vector<Vector2> ring = GetEndRing(f.brush->GetParams(), top);
                    REQUIRE(ring.size() == static_cast<unsigned>(segments));
                    const Vector3 center = top ? params.topCenter_ : params.bottomCenter_;
                    for (unsigned i = 0; i < 5; ++i)
                    {
                        const unsigned id = i < 4 ? 2 + (top ? 4 : 0) + i : (top ? 12 : 11);
                        const Vector3 p = FindHandle(handles, id)->position_ - center;
                        CHECK(p.y_ == 0.0f);
                        CHECK(DistanceToPolygon(Vector2(p.x_, p.z_), ring) < 1e-5f);
                    }
                }
            }
        }
    }
}

TEST_CASE("Cylinder radius handles follow the face in a pyramid", "[shapes][handles]")
{
    HandleFixture<CsgCylinderBrush> f;
    CsgCylinderParams params;
    params.segments_ = 4;
    params.startAngle_ = 45.0f;
    params.faceRadius_ = true;
    params.topRadius_ = Vector2::ZERO;
    f.brush->SetParams(params);
    auto handles = f.Handles();
    // The radius reaches the middle of the face: +X is on the face x = 0.5, between the corners (0.5, +-0.5).
    const Vector3 plusX = FindHandle(handles, 2)->position_;
    CHECK(plusX.x_ == Catch::Approx(0.5f).margin(1e-6f));
    CHECK(plusX.z_ == Catch::Approx(0.0f).margin(1e-6f));
    CHECK(IsNear(FindHandle(handles, 4)->position_, Vector3(0.0f, -0.5f, 0.5f), 1e-6f));
    // The scale handle is on the corner at 45 degrees.
    CHECK(IsNear(FindHandle(handles, 11)->position_, Vector3(0.5f, -0.5f, 0.5f), 1e-6f));
    // A drag along +X by 0.25 gives the radius 0.75 on that component.
    REQUIRE(f.brush->MoveHandles({2}, Vector3(0.25f, 9.0f, 9.0f)));
    CHECK(f.brush->GetBottomRadius().x_ == Catch::Approx(0.75f).margin(1e-6f));
    CHECK(f.brush->GetBottomRadius().y_ == 0.5f);
}

TEST_CASE("Cylinder scale handles keep the ratio and stay under the cursor", "[shapes][handles]")
{
    for (const int segments : {4, 6, 24})
    {
        for (const bool face : {false, true})
        {
            HandleFixture<CsgCylinderBrush> f;
            CsgCylinderParams params;
            params.segments_ = segments;
            params.startAngle_ = 17.0f;
            params.faceRadius_ = face;
            params.bottomRadius_ = Vector2(0.8f, 0.4f);
            params.topRadius_ = Vector2(0.6f, 0.3f);
            f.brush->SetParams(params);
            INFO("Segments " << segments << " face " << face);
            for (const unsigned id : {11u, 12u})
            {
                f.brush->SetParams(params);
                const CsgShapeHandle old = *FindHandle(f.Handles(), id);
                const Vector3 delta(0.3f, 0.7f, -0.2f);
                REQUIRE(f.brush->MoveHandles({id}, delta));
                const CsgShapeHandle moved = *FindHandle(f.Handles(), id);
                // The new position minus the old one is the delta projected on the axis.
                const Vector3 expected = old.axis_ * delta.DotProduct(old.axis_);
                CHECK(IsNear(moved.position_ - old.position_, expected, 1e-5f));
                const Vector2 before = id == 11 ? params.bottomRadius_ : params.topRadius_;
                const Vector2 after = id == 11 ? f.brush->GetBottomRadius() : f.brush->GetTopRadius();
                CHECK(after.x_ / after.y_ == Catch::Approx(before.x_ / before.y_).epsilon(1e-5));
                // The other end does not change.
                CHECK((id == 11 ? f.brush->GetTopRadius() : f.brush->GetBottomRadius())
                    == (id == 11 ? params.topRadius_ : params.bottomRadius_));
            }
        }
    }
}

TEST_CASE("Cylinder scale handle collapses to an apex and grows back", "[shapes][handles]")
{
    HandleFixture<CsgCylinderBrush> f;
    CsgCylinderParams params;
    params.segments_ = 6;
    params.startAngle_ = 30.0f;
    params.faceRadius_ = true;
    f.brush->SetParams(params);

    const CsgShapeHandle top = *FindHandle(f.Handles(), 12);
    const float length = (top.position_ - params.topCenter_).Length();
    // A drag by minus the length gives radius 0 (an apex). A longer drag gives the same.
    REQUIRE(f.brush->MoveHandles({12}, top.axis_ * -length));
    CHECK(f.brush->GetTopRadius() == Vector2::ZERO);
    f.brush->SetParams(params);
    REQUIRE(f.brush->MoveHandles({12}, top.axis_ * (-2.0f * length)));
    CHECK(f.brush->GetTopRadius() == Vector2::ZERO);
    // A drag that leaves both components below 0.001 gives the apex too.
    f.brush->SetParams(params);
    REQUIRE(f.brush->MoveHandles({12}, top.axis_ * (-length + 0.0001f)));
    CHECK(f.brush->GetTopRadius() == Vector2::ZERO);

    // The apex: no radius handles for the top, and the scale handle sits at the end center.
    const auto apexHandles = f.Handles();
    CHECK(GetIds(apexHandles) == ea::vector<unsigned>{0, 1, 2, 3, 4, 5, 10, 11, 12});
    const CsgShapeHandle apex = *FindHandle(apexHandles, 12);
    CHECK(apex.position_ == params.topCenter_);
    CHECK(IsNear(apex.axis_, Vector3(1.0f, 0.0f, 1.0f).Normalized(), 1e-6f));
    // A drag against the axis keeps the apex (no change).
    CHECK_FALSE(f.brush->MoveHandles({12}, -apex.axis_ * 0.2f));
    CHECK(f.brush->GetTopRadius() == Vector2::ZERO);

    // A drag along the axis grows the radius to (r, r). The handle stays under the cursor.
    const Vector3 delta = apex.axis_ * 0.3f + Vector3(0.4f, 0.0f, -0.4f);
    REQUIRE(f.brush->MoveHandles({12}, delta));
    const Vector2 radius = f.brush->GetTopRadius();
    CHECK(radius.x_ == radius.y_);
    CHECK(radius.x_ > 0.0f);
    const CsgShapeHandle grown = *FindHandle(f.Handles(), 12);
    CHECK(IsNear(grown.position_ - apex.position_, apex.axis_ * 0.3f, 1e-5f));
    CHECK(f.Handles().size() == 13);

    // Both ends: the bottom becomes an apex too, then the shape is not valid and the move returns false.
    f.brush->SetParams(params);
    REQUIRE(f.brush->MoveHandles({12}, top.axis_ * -length));
    const CsgShapeHandle bottom = *FindHandle(f.Handles(), 11);
    const CsgCylinderParams before = f.brush->GetParams();
    CHECK_FALSE(f.brush->MoveHandles({11}, bottom.axis_ * -10.0f));
    CHECK(f.brush->GetParams() == before);
    CHECK(f.errors.GetCount() == 0);
}

TEST_CASE("Cylinder handles restore without state and recenter keeps start and face radius", "[shapes][handles][recenter]")
{
    HandleFixture<CsgCylinderBrush> f;
    CsgCylinderParams params;
    params.segments_ = 5;
    params.startAngle_ = 17.0f;
    params.faceRadius_ = true;
    params.bottomCenter_ = Vector3(1.0f, 0.0f, 0.5f);
    params.topCenter_ = Vector3(2.0f, 2.0f, 0.5f);
    f.brush->SetParams(params);
    const auto original = f.Handles();
    const CsgCylinderParams before = f.brush->GetParams();

    // A move then the saved params give the same handles: the handles have no state.
    REQUIRE(f.brush->MoveHandles({11, 12}, Vector3(0.1f, 0.0f, 0.2f)));
    CHECK(f.brush->GetParams() != before);
    f.brush->SetParams(before);
    const auto restored = f.Handles();
    REQUIRE(restored.size() == original.size());
    for (unsigned i = 0; i < original.size(); ++i)
    {
        CHECK(restored[i].id_ == original[i].id_);
        CHECK(restored[i].position_ == original[i].position_);
        CHECK(restored[i].axis_ == original[i].axis_);
    }

    // Recenter moves the centers and keeps the rest. The handles move by minus the offset.
    Vector3 offset;
    REQUIRE(f.brush->Recenter(offset));
    CHECK(IsNear(offset, Vector3(1.5f, 1.0f, 0.5f)));
    CHECK(f.brush->GetStartAngle() == 17.0f);
    CHECK(f.brush->GetFaceRadius());
    CHECK(f.brush->GetSegments() == 5);
    const auto recentered = f.Handles();
    REQUIRE(recentered.size() == original.size());
    for (unsigned i = 0; i < original.size(); ++i)
        CHECK(IsNear(recentered[i].position_, original[i].position_ - offset, 1e-5f));
    CHECK(f.errors.GetCount() == 0);
}

namespace
{

/// The documented groups of a shape: the group name and its handle ids, in the order of the mode bar.
using GroupTable = ea::vector<ea::pair<ea::string, ea::vector<unsigned>>>;

ea::vector<unsigned> IdRange(unsigned first, unsigned last)
{
    ea::vector<unsigned> ids;
    for (unsigned id = first; id <= last; ++id)
        ids.push_back(id);
    return ids;
}

/// Check the group rules of a brush against the table. Handles that are absent (an apex) are allowed.
/// If `full` is true all ids of the table must be present.
template <class T> void CheckGroupTable(HandleFixture<T>& f, const GroupTable& table, bool full)
{
    ea::vector<ea::string> groups;
    f.brush->GetHandleGroups(groups);
    REQUIRE(groups.size() == table.size());
    for (unsigned i = 0; i < table.size(); ++i)
        CHECK(groups[i] == table[i].first);
    // The first entry is the default mode. No duplicate. No reserved name.
    for (unsigned i = 0; i < groups.size(); ++i)
    {
        CHECK(groups[i] != "All");
        CHECK_FALSE(groups[i].empty());
        for (unsigned j = i + 1; j < groups.size(); ++j)
            CHECK(groups[i] != groups[j]);
    }

    const auto handles = f.Handles();
    ea::hash_set<unsigned> seen;
    for (const CsgShapeHandle& handle : handles)
    {
        INFO("Handle " << handle.id_);
        CHECK_FALSE(handle.group_.empty());
        CHECK(ea::find(groups.begin(), groups.end(), handle.group_) != groups.end());
        CHECK(seen.insert(handle.id_).second);
        unsigned owners = 0;
        for (const auto& [name, ids] : table)
        {
            if (ea::find(ids.begin(), ids.end(), handle.id_) != ids.end())
            {
                ++owners;
                CHECK(handle.group_ == name);
            }
        }
        CHECK(owners == 1);
    }
    if (full)
    {
        unsigned total = 0;
        for (const auto& entry : table)
            total += entry.second.size();
        CHECK(handles.size() == total);
    }

    // The event round trip carries the groups and the Group keys.
    const VariantMap data = Query(f.brush);
    const StringVector& sent = data.at(CsgShapeQueryHandles::P_GROUPS).GetStringVector();
    CHECK(sent == groups);
    const VariantVector& list = data.at(CsgShapeQueryHandles::P_HANDLES).GetVariantVector();
    REQUIRE(list.size() == handles.size());
    for (unsigned i = 0; i < list.size(); ++i)
        CHECK(list[i].GetVariantMap().at(CsgShapeProtocol::HandleGroupKey).GetString() == handles[i].group_);
}

} // namespace

TEST_CASE("Every shape declares handle groups that partition its handle ids", "[shapes][handles][modes]")
{
    SECTION("Box")
    {
        HandleFixture<CsgBoxBrush> f;
        CheckGroupTable(f,
            {{"Face", IdRange(20, 25)}, {"Size", IdRange(50, 52)}, {"Edge", IdRange(30, 41)},
                {"Corner", IdRange(0, 7)}},
            true);
    }
    SECTION("Sphere")
    {
        HandleFixture<CsgSphereBrush> f;
        CheckGroupTable(f, {{"Radius", IdRange(0, 5)}}, true);
    }
    SECTION("Ring")
    {
        HandleFixture<CsgRingBrush> f;
        CheckGroupTable(f, {{"Shape", IdRange(0, 3)}}, true);
    }
    SECTION("Arch")
    {
        HandleFixture<CsgArchBrush> f;
        CheckGroupTable(f, {{"Shape", IdRange(0, 4)}}, true);
    }
    SECTION("Wedge")
    {
        HandleFixture<CsgWedgeBrush> f;
        CheckGroupTable(f, {{"Face", IdRange(0, 5)}, {"Size", IdRange(6, 8)}}, true);
    }
    SECTION("Stairs")
    {
        HandleFixture<CsgStairsBrush> f;
        CheckGroupTable(f, {{"Face", IdRange(0, 5)}, {"Size", IdRange(6, 8)}}, true);
    }

    const GroupTable cylinderTable = {{"Ends", {0, 1, 11, 12}}, {"Radius", IdRange(2, 10)}};
    SECTION("Cylinder default")
    {
        HandleFixture<CsgCylinderBrush> f;
        CheckGroupTable(f, cylinderTable, false);
    }
    SECTION("Cylinder with a bulge")
    {
        HandleFixture<CsgCylinderBrush> f;
        f.brush->SetBulge(0.5f);
        CheckGroupTable(f, cylinderTable, true);
        CHECK(f.Handles().size() == 13);
    }
    SECTION("Cylinder with an apex")
    {
        HandleFixture<CsgCylinderBrush> f;
        CsgCylinderParams params;
        params.topRadius_ = Vector2::ZERO;
        f.brush->SetParams(params);
        CheckGroupTable(f, cylinderTable, false);
        CHECK(f.Handles().size() < 13);
    }
}

TEST_CASE("The box size handles move the two faces of an axis", "[shapes][handles][modes]")
{
    HandleFixture<CsgBoxBrush> f;
    f.brush->GetNode()->SetRotation(Quaternion(20.0f, 30.0f, 40.0f));
    f.brush->GetNode()->SetScale(Vector3(1.0f, 2.0f, 0.5f));
    CsgBoxParams params;
    params.size_ = Vector3(2.0f, 1.0f, 3.0f);
    f.brush->SetParams(params);
    // A skew: move one corner. The skew must stay.
    REQUIRE(f.brush->MoveHandles({7}, Vector3(0.2f, 0.1f, -0.1f)));
    const CsgBoxParams start = f.brush->GetParams();
    const auto startHandles = f.Handles();
    const Vector3 axes[3] = {Vector3::RIGHT, Vector3::UP, Vector3::FORWARD};
    const char* const names[3] = {"Size X", "Size Y", "Size Z"};

    for (unsigned a = 0; a < 3; ++a)
    {
        INFO("Axis " << a);
        f.brush->SetParams(start);
        const CsgShapeHandle* handle = FindHandle(startHandles, 50 + a);
        REQUIRE(handle);
        CHECK(handle->name_ == names[a]);
        CHECK(handle->kind_ == CsgShapeHandleKind::Axis);
        CHECK(handle->axis_ == axes[a]);
        CHECK(IsNear(handle->position_, FindHandle(startHandles, 21 + 2 * a)->position_));

        const float d = 0.25f;
        const Vector3 across = axes[a].CrossProduct(Vector3(0.3f, 0.5f, 0.7f));
        REQUIRE(f.brush->MoveHandles({50 + a}, axes[a] * d + across));
        const CsgBoxParams moved = f.brush->GetParams();
        CHECK(moved.size_ == start.size_);
        for (unsigned corner = 0; corner < 8; ++corner)
        {
            const float sign = (corner >> a) & 1u ? 1.0f : -1.0f;
            Vector3 expected = start.corners_[corner];
            expected.MutableData()[a] += sign * d / start.size_.Data()[a];
            CHECK(IsNear(moved.corners_[corner], expected));
        }
        // The centroid of the box stays.
        Vector3 before, after;
        for (unsigned corner = 0; corner < 8; ++corner)
        {
            before += start.size_ * start.corners_[corner];
            after += moved.size_ * moved.corners_[corner];
        }
        CHECK(IsNear(before / 8.0f, after / 8.0f));
        // The size handle follows the cursor on its axis.
        CHECK(IsNear(FindHandle(f.Handles(), 50 + a)->position_, handle->position_ + axes[a] * d, 1e-5f));

        // Stateless: the same delta after a restore gives the same parameters.
        f.brush->SetParams(start);
        REQUIRE(f.brush->MoveHandles({50 + a, 50 + a}, axes[a] * d + across));
        CHECK(f.brush->GetParams().corners_ == moved.corners_);

        // Zero move: false. An inversion is refused and keeps the parameters.
        f.brush->SetParams(start);
        CHECK_FALSE(f.brush->MoveHandles({50 + a}, Vector3::ZERO));
        CHECK_FALSE(f.brush->MoveHandles({50 + a}, across));
        CHECK_FALSE(f.brush->MoveHandles({50 + a}, axes[a] * -10.0f));
        CHECK(f.brush->GetParams().corners_ == start.corners_);
        CHECK_FALSE(f.brush->MoveHandles({50 + a}, axes[a] * std::numeric_limits<float>::infinity()));
    }
    CHECK_FALSE(f.brush->MoveHandles({53}, Vector3(0.1f, 0, 0)));
    CHECK(f.errors.GetCount() == 0);
}

namespace
{

template <class T> void CheckSizeHandlesOfBoxLikeShape(HandleFixture<T>& f)
{
    const auto start = f.brush->GetParams();
    const auto startHandles = f.Handles();
    REQUIRE(startHandles.size() == 9);
    const Vector3 axes[3] = {Vector3::RIGHT, Vector3::UP, Vector3::FORWARD};
    const unsigned faceOfAxis[3] = {1, 4, 3};
    for (unsigned a = 0; a < 3; ++a)
    {
        INFO("Axis " << a);
        f.brush->SetParams(start);
        const CsgShapeHandle& handle = startHandles[6 + a];
        CHECK(handle.id_ == 6 + a);
        CHECK(handle.kind_ == CsgShapeHandleKind::Axis);
        CHECK(handle.axis_ == axes[a]);
        CHECK(handle.group_ == "Size");
        CHECK(IsNear(handle.position_, startHandles[faceOfAxis[a]].position_));

        const float d = 0.25f;
        const Vector3 across = axes[a].CrossProduct(Vector3(0.3f, 0.5f, 0.7f));
        REQUIRE(f.brush->MoveHandles({6 + a}, axes[a] * d + across));
        const auto moved = f.brush->GetParams();
        CHECK(moved.size_.Data()[a] == Catch::Approx(start.size_.Data()[a] + 2.0f * d));
        CHECK(moved.center_ == start.center_);
        for (unsigned other = 0; other < 3; ++other)
        {
            if (other != a)
                CHECK(moved.size_.Data()[other] == start.size_.Data()[other]);
        }

        // Stateless.
        f.brush->SetParams(start);
        REQUIRE(f.brush->MoveHandles({6 + a, 6 + a}, axes[a] * d + across));
        CHECK(f.brush->GetParams().size_ == moved.size_);

        // A shrink clamps. No move is false.
        f.brush->SetParams(start);
        CHECK_FALSE(f.brush->MoveHandles({6 + a}, Vector3::ZERO));
        CHECK_FALSE(f.brush->MoveHandles({6 + a}, across));
        REQUIRE(f.brush->MoveHandles({6 + a}, axes[a] * -100.0f));
        CHECK(f.brush->GetParams().size_.Data()[a] == Catch::Approx(0.001f));
        CHECK(f.brush->GetParams().center_ == start.center_);
    }
    CHECK_FALSE(f.brush->MoveHandles({9}, Vector3(0.1f, 0, 0)));
    CHECK(f.errors.GetCount() == 0);
}

} // namespace

TEST_CASE("The wedge and stairs size handles grow the size around the center", "[shapes][handles][modes]")
{
    SECTION("Wedge")
    {
        CsgWedgeParams wedge;
        wedge.size_ = Vector3(2.0f, 1.0f, 3.0f);
        wedge.center_ = Vector3(0.5f, -0.25f, 1.0f);
        HandleFixture<CsgWedgeBrush> w;
        w.brush->SetParams(wedge);
        CheckSizeHandlesOfBoxLikeShape(w);
    }
    SECTION("Stairs")
    {
        CsgStairsParams stairs;
        stairs.size_ = Vector3(2.0f, 1.0f, 3.0f);
        stairs.center_ = Vector3(0.5f, -0.25f, 1.0f);
        HandleFixture<CsgStairsBrush> s;
        s.brush->SetParams(stairs);
        CheckSizeHandlesOfBoxLikeShape(s);
    }
}

TEST_CASE("The face pick keeps the face handle in the full set and finds the size handle in the size set",
    "[shapes][handles][modes]")
{
    HandleFixture<CsgBoxBrush> f;
    const auto all = f.Handles();
    ea::vector<CsgShapeHandle> sizeOnly;
    for (const CsgShapeHandle& handle : all)
    {
        if (handle.group_ == "Size")
            sizeOnly.push_back(handle);
    }
    REQUIRE(sizeOnly.size() == 3);
    const Vector3 normals[6] = {Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, -1, 0), Vector3(0, 1, 0),
        Vector3(0, 0, -1), Vector3(0, 0, 1)};
    for (unsigned face = 0; face < 6; ++face)
    {
        INFO("Face " << face);
        const Vector3 point = normals[face] * 0.5f;
        // The size handle and the + face handle tie. The lower id, the face, wins.
        REQUIRE(ChooseFaceHandle(all, point, normals[face]));
        CHECK(*ChooseFaceHandle(all, point, normals[face]) == 20 + face);
        REQUIRE(ChooseFaceHandle(sizeOnly, point, normals[face]));
        CHECK(*ChooseFaceHandle(sizeOnly, point, normals[face]) == 50 + face / 2);
    }
}
