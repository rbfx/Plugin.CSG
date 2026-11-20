// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Robustness tests: a procedural box minus a procedural sphere near a corner of the box.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "Urho3D/CSG/CsgTriangulation.h"

#include "TestGeometry.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Model.h>

#include <catch2/catch_amalgamated.hpp>

#include <cmath>
#include <cstdio>

using namespace Urho3D;

namespace
{

/// The result of a boolean, its check as the root does it, and the crossing counts of the diagnostics.
struct BoolResult
{
    SharedPtr<const CsgSolid> solid_;
    CsgModelCheck check_;
    bool closed_{};
    SharedPtr<CsgTriangulatedModel> model_;
    /// Crossings that fold after ConstrainCrossings (non strict rules), summed over all passes of the boolean.
    int fold_{};
    /// Crossings that fold by the rule "inside" after ConstrainCrossings: a position across a triangle edge.
    int foldInside_{};
    /// Crossings that fold only by a position on the line of a triangle edge.
    int foldOnEdge_{};
    /// Crossings that fold and that ConstrainCrossings could not move.
    int unconstrainable_{};
    /// Crossings that the count tested.
    int checked_{};
};
using BoxMinusSphere = BoolResult;

/// Run a boolean with the validity count of the diagnostics on.
BoolResult RunBoolean(const CsgSolid& a, const CsgSolid& b, CsgOperation operation)
{
    CsgInternal::CsgBooleanDiagnostics& diagnostics = CsgInternal::g_csgDiagnostics;
    const bool wasCounting = diagnostics.countInvalid_.load();
    diagnostics.Reset();
    diagnostics.countInvalid_.store(true);
    BoolResult result;
    result.solid_ = CsgBooleanOperation(&a, &b, operation);
    diagnostics.countInvalid_.store(wasCounting);
    result.fold_ = diagnostics.foldCrossings_.load();
    result.foldInside_ = diagnostics.foldInside_.load();
    result.foldOnEdge_ = diagnostics.foldOnEdge_.load();
    result.unconstrainable_ = diagnostics.unconstrainableCrossings_.load();
    result.checked_ = diagnostics.checkedCrossings_.load();
    REQUIRE(result.solid_);
    result.closed_ = result.solid_->IsClosed();
    result.model_ = CsgTriangulateSolid(*result.solid_);
    REQUIRE(result.model_);
    result.check_ = CsgCheckModel(*result.model_);
    return result;
}

/// Make the polygons of a shape mesh as the root does it: through a model and a transform.
ea::vector<CsgPolygon> MeshPolygons(
    Context* context, const CsgShapeMesh& mesh, const Matrix3x4& transform, unsigned meshId)
{
    const SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    return CsgBuildPolygonsFromModel(model, transform, meshId);
}

ea::vector<CsgPolygon> SpherePolygons(Context* context, float radius, const Vector3& center, bool smooth = true)
{
    CsgSphereParams params;
    params.radius_ = Vector3(radius, radius, radius);
    params.smoothNormals_ = smooth;
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(params, mesh));
    return MeshPolygons(context, mesh, Matrix3x4(center, Quaternion::IDENTITY, 1.0f), 2);
}

BoxMinusSphere MakeBoxMinusSphere(Context* context, float radius, const Vector3& center, bool smooth = true)
{
    CsgShapeMesh boxMesh;
    REQUIRE(CsgGenerateBox(CsgBoxParams{}, boxMesh));
    const SharedPtr<CsgSolid> box = CsgSolid::FromOperand(MeshPolygons(context, boxMesh, Matrix3x4::IDENTITY, 1));
    const SharedPtr<CsgSolid> sphere = CsgSolid::FromOperand(SpherePolygons(context, radius, center, smooth));
    return RunBoolean(*box, *sphere, CsgOperation::DifferenceAB);
}

/// The rule of `CsgBuildStats::valid_` in the root build.
bool IsValid(const BoolResult& r)
{
    if (r.solid_->FaceCount() == 0)
        return true;
    return r.closed_ && r.check_.openEdges_ == 0 && r.check_.duplicates_ == 0 && r.check_.degenerates_ == 0
        && r.check_.volume_ > 0.0;
}

/// An empty result: no face, or no positive volume. A sweep case must not give it.
bool IsEmpty(const BoolResult& r)
{
    return r.solid_->FaceCount() == 0 || r.check_.volume_ <= 0.0;
}

/// A sweep case passes when the result is valid and not empty.
bool IsSweepOk(const BoolResult& r)
{
    return IsValid(r) && !IsEmpty(r);
}

/// A point in units of 1/1024 (the grid of the float mode). The position is exact in both modes.
struct GridInts
{
    int x_;
    int y_;
    int z_;
};

Vector3 GridPosition(const GridInts& p)
{
    const float step = 1.0f / 1024.0f;
    return Vector3(static_cast<float>(p.x_) * step, static_cast<float>(p.y_) * step, static_cast<float>(p.z_) * step);
}

CsgPolygon GridTriangle(unsigned shared, const GridInts& p0, const GridInts& p1, const GridInts& p2)
{
    ea::vector<ModelVertex> vertices(3);
    vertices[0].SetPosition(GridPosition(p0));
    vertices[1].SetPosition(GridPosition(p1));
    vertices[2].SetPosition(GridPosition(p2));
    return CsgPolygon(ea::move(vertices), shared);
}

/// A box of 12 triangles in grid units. Each face is the quad (c0, c1, c2, c3), counterclockwise from outside,
/// with the diagonal c0 c2. On the top face (z = max) the diagonal goes from (min x, min y) to (max x, max y).
ea::vector<CsgPolygon> GridBoxTriangles(const GridInts& lo, const GridInts& hi, bool reverseOrder)
{
    const auto at = [&](int i) { return GridInts{i & 1 ? hi.x_ : lo.x_, i & 2 ? hi.y_ : lo.y_, i & 4 ? hi.z_ : lo.z_}; };
    // The corner bits: 1 is +x, 2 is +y, 4 is +z.
    static constexpr int faces[6][4] = {
        {4, 5, 7, 6}, {0, 2, 3, 1}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    ea::vector<CsgPolygon> polygons;
    for (const auto& face : faces)
    {
        polygons.push_back(GridTriangle(1, at(face[0]), at(face[1]), at(face[2])));
        polygons.push_back(GridTriangle(1, at(face[0]), at(face[2]), at(face[3])));
    }
    if (reverseOrder)
        ea::reverse(polygons.begin(), polygons.end());
    return polygons;
}

/// A tetrahedron in grid units with outward triangles.
ea::vector<CsgPolygon> GridTetrahedron(const GridInts (&p)[4], bool reverseOrder)
{
    static constexpr int faces[4][4] = {{0, 1, 2, 3}, {0, 3, 1, 2}, {1, 3, 2, 0}, {0, 2, 3, 1}};
    ea::vector<CsgPolygon> polygons;
    for (const auto& face : faces)
    {
        const Vector3 a = GridPosition(p[face[0]]);
        const Vector3 b = GridPosition(p[face[1]]);
        const Vector3 c = GridPosition(p[face[2]]);
        const Vector3 d = GridPosition(p[face[3]]);
        const bool outward = (b - a).CrossProduct(c - a).DotProduct(d - a) < 0.0f;
        polygons.push_back(outward ? GridTriangle(2, p[face[0]], p[face[1]], p[face[2]])
                                   : GridTriangle(2, p[face[0]], p[face[2]], p[face[1]]));
    }
    if (reverseOrder)
        ea::reverse(polygons.begin(), polygons.end());
    return polygons;
}

/// The counts of a sweep.
struct SweepCounts
{
    unsigned failed_{};
    unsigned empty_{};
    int fold_{};
    int foldInside_{};
    int foldOnEdge_{};
    int unconstrainable_{};
    int checked_{};

    void Add(const BoolResult& r)
    {
        failed_ += IsSweepOk(r) ? 0 : 1;
        empty_ += IsEmpty(r) ? 1 : 0;
        fold_ += r.fold_;
        foldInside_ += r.foldInside_;
        foldOnEdge_ += r.foldOnEdge_;
        unconstrainable_ += r.unconstrainable_;
        checked_ += r.checked_;
    }
};

/// A small generator with a fixed sequence, so the sweep is the same on all platforms.
struct SweepRandom
{
    unsigned state_;
    unsigned Next()
    {
        state_ = state_ * 1664525u + 1013904223u;
        return state_ >> 8;
    }
    /// A value in [-0.2, 0.2] on a grid of 1/4096 units.
    float Offset() { return static_cast<float>(static_cast<int>(Next() % 1639u) - 819) / 4096.0f; }
};

/// Call `function(radius, center, smooth)` for `count` random cases near the corner (0.5, 0.5, 0.5).
/// A third of the cases put the center on a diagonal plane of the box (two equal offsets).
template <class Function> void ForEachRandomCase(unsigned seed, unsigned count, const Function& function)
{
    static constexpr float radii[] = {0.25f, 0.3f, 0.5f, 0.75f};
    SweepRandom random{seed};
    for (unsigned i = 0; i < count; ++i)
    {
        const float radius = radii[random.Next() % 4u];
        float a = random.Offset();
        float b = random.Offset();
        float c = random.Offset();
        switch (random.Next() % 9u)
        {
        case 0: c = a; break;
        case 1: b = a; break;
        case 2: c = b; break;
        default: break;
        }
        const bool smooth = (random.Next() & 1u) != 0;
        function(radius, Vector3(0.5f + a, 0.5f + b, 0.5f + c), smooth);
    }
}

/// Call `function(radius, center, smooth)` for a grid of centers near the corner (0.5, 0.5, 0.5).
/// The offsets are in [-0.2, 0.2] with the step 0.05, and small offsets near zero, on each axis.
template <class Function> void ForEachGridCase(const Function& function)
{
    static constexpr float radii[] = {0.25f, 0.3f, 0.5f, 0.75f};
    float offsets[13];
    for (int i = -4; i <= 4; ++i)
        offsets[i + 4] = static_cast<float>(i) * 0.05f;
    offsets[9] = 1e-4f;
    offsets[10] = 1.0f / 1024.0f;
    offsets[11] = 1.0f / 2048.0f;
    offsets[12] = -1.0f / 1024.0f;
    for (const float radius : radii)
        for (const float a : offsets)
            for (const float b : offsets)
                for (const float c : offsets)
                    function(radius, Vector3(0.5f + a, 0.5f + b, 0.5f + c), true);
}

/// Run `count` random cases of a box minus a sphere and return the counts.
SweepCounts SweepCorner(Context* context, unsigned seed, unsigned count)
{
    SweepCounts counts;
    ForEachRandomCase(seed, count,
        [&](float radius, const Vector3& center, bool smooth)
        {
            const BoolResult result = MakeBoxMinusSphere(context, radius, center, smooth);
            counts.Add(result);
            if (!IsSweepOk(result))
                UNSCOPED_INFO("radius " << radius << " center (" << center.x_ << ", " << center.y_ << ", "
                                        << center.z_ << ") smooth " << smooth);
        });
    return counts;
}

/// A box of size 1 at the origin minus a sphere (the first cut), then minus the same sphere again (the second cut).
struct Recut
{
    BoolResult first_;
    BoolResult second_;
};

Recut MakeRecut(Context* context, float radius, const Vector3& center, bool smooth)
{
    CsgShapeMesh boxMesh;
    REQUIRE(CsgGenerateBox(CsgBoxParams{}, boxMesh));
    const SharedPtr<CsgSolid> box = CsgSolid::FromOperand(MeshPolygons(context, boxMesh, Matrix3x4::IDENTITY, 1));
    const SharedPtr<CsgSolid> sphere = CsgSolid::FromOperand(SpherePolygons(context, radius, center, smooth));
    Recut result;
    result.first_ = RunBoolean(*box, *sphere, CsgOperation::DifferenceAB);
    result.second_ = RunBoolean(*result.first_.solid_, *sphere, CsgOperation::DifferenceAB);
    return result;
}

/// The counts of a sweep of second identical cuts.
struct RecutCounts
{
    unsigned cases_{};
    unsigned firstFailed_{};
    unsigned failed_{};
    unsigned duplicates_{};
    unsigned open_{};
    unsigned empty_{};
    /// Second cuts with a volume difference above 1e-3 and above 0.1.
    unsigned volumeAbove1em3_{};
    unsigned volumeAbove0p1_{};
    long long addedFaces_{};
    double maxVolumeDifference_{};

    void Add(float radius, const Vector3& center, bool smooth, const Recut& recut)
    {
        const BoolResult& first = recut.first_;
        const BoolResult& second = recut.second_;
        ++cases_;
        firstFailed_ += IsSweepOk(first) ? 0 : 1;
        empty_ += (IsEmpty(first) ? 1 : 0) + (IsEmpty(second) ? 1 : 0);
        if (!IsSweepOk(second))
        {
            ++failed_;
            duplicates_ += second.check_.duplicates_ != 0 ? 1 : 0;
            open_ += second.check_.openEdges_ != 0 || !second.closed_ ? 1 : 0;
            printf("re-cut not valid: radius %.9g center (%.9g, %.9g, %.9g) smooth %d duplicates %u open %u "
                   "volume %.9g -> %.9g\n",
                radius, center.x_, center.y_, center.z_, static_cast<int>(smooth), second.check_.duplicates_,
                second.check_.openEdges_, first.check_.volume_, second.check_.volume_);
        }
        addedFaces_ += static_cast<long long>(second.solid_->FaceCount()) - first.solid_->FaceCount();
        const double volumeDifference = std::abs(second.check_.volume_ - first.check_.volume_);
        volumeAbove1em3_ += volumeDifference > 1e-3 ? 1 : 0;
        volumeAbove0p1_ += volumeDifference > 0.1 ? 1 : 0;
        maxVolumeDifference_ = ea::max(maxVolumeDifference_, volumeDifference);
    }
};

} // namespace

TEST_CASE("Box minus a sphere: a crossing on the twin of a coplanar edge keeps the grid point of the twin",
    "[boolean][regression]")
{
    // The sphere has its 45 degree meridian in the plane x = z, and the edge x = z = 0.5 of the box is in this
    // plane. The box edge crosses a sphere triangle at the point where the meridian edge crosses a box face.
    // The rounding of the two twin crossings is one grid point, outside the projected sphere triangle.
    // The fold rule moved one twin a grid step away. The sphere piece between the two points then lay flat on the
    // box face, and its triangulation gave a duplicate of a triangle of the box face.
    auto context = MakeShared<Context>();
    for (const bool smooth : {true, false})
    {
        const BoxMinusSphere result = MakeBoxMinusSphere(context, 0.3f, Vector3(0.3f, 0.5f, 0.3f), smooth);
        CHECK(result.closed_);
        CHECK(result.check_.duplicates_ == 0);
        CHECK(result.check_.openEdges_ == 0);
        CHECK(result.check_.degenerates_ == 0);
        CHECK(result.solid_->FaceCount() > 0);
        CHECK(IsValid(result));
        CHECK(result.foldOnEdge_ == 0);
    }
}

TEST_CASE("Box minus a sphere: a crossing inside its triangle does not round onto a triangle edge",
    "[boolean][regression]")
{
    // A sphere edge crosses a box face triangle a third of a grid step from the diagonal of the face.
    // The nearest rounding is on the diagonal. The region between the curve and the diagonal got zero width,
    // and the sphere piece at the diagonal lay flat on the box face. This gave a duplicate triangle.
    auto context = MakeShared<Context>();
    for (const bool smooth : {true, false})
    {
        const BoxMinusSphere result = MakeBoxMinusSphere(context, 0.25f, Vector3(0.45f, 0.4f, 0.7f), smooth);
        CHECK(result.closed_);
        CHECK(result.check_.duplicates_ == 0);
        CHECK(result.check_.openEdges_ == 0);
        CHECK(result.check_.degenerates_ == 0);
        CHECK(result.solid_->FaceCount() > 0);
        CHECK(IsValid(result));
        CHECK(result.foldOnEdge_ == 0);
    }
}

TEST_CASE("Box minus a sphere near a corner gives a valid result over a random sweep", "[boolean][regression]")
{
    auto context = MakeShared<Context>();
    // The case count is small on purpose. It keeps the total test time below one second in Debug.
    const SweepCounts counts = SweepCorner(context, 20261119u, 60);
    CHECK(counts.failed_ == 0);
    CHECK(counts.empty_ == 0);
    // No crossing folds by the rule "inside" after ConstrainCrossings. A twin that moves later would show here.
    CHECK(counts.foldInside_ == 0);
    CHECK(counts.foldOnEdge_ == 0);
}

TEST_CASE("Box minus a sphere near a corner over a long random sweep", "[.][boolean][sweep]")
{
    auto context = MakeShared<Context>();
    const SweepCounts counts = SweepCorner(context, 1u, 5000);
    printf("random sweep: failed %u, empty %u, checked %d, fold %d, fold inside %d, unconstrainable %d\n", counts.failed_,
        counts.empty_, counts.checked_, counts.fold_, counts.foldInside_, counts.unconstrainable_);
    CHECK(counts.failed_ == 0);
}

TEST_CASE("Box minus a sphere near a corner over a grid of centers", "[.][boolean][sweep]")
{
    auto context = MakeShared<Context>();
    SweepCounts counts;
    ForEachGridCase(
        [&](float radius, const Vector3& center, bool smooth)
        {
            const BoolResult result = MakeBoxMinusSphere(context, radius, center, smooth);
            counts.Add(result);
            if (!IsSweepOk(result))
                UNSCOPED_INFO("radius " << radius << " center (" << center.x_ << ", " << center.y_ << ", "
                                        << center.z_ << ")");
        });
    printf("grid sweep: failed %u, empty %u, checked %d, fold %d, fold inside %d, unconstrainable %d\n", counts.failed_,
        counts.empty_, counts.checked_, counts.fold_, counts.foldInside_, counts.unconstrainable_);
    CHECK(counts.failed_ == 0);
}

TEST_CASE("Box minus a sphere near a corner and a second cut with the same sphere", "[.][boolean][sweep]")
{
    // A second identical cut can add faces, and some second cuts are not valid (see the known limits in the README).
    // The test reports the counts of the second cut. It asserts only that each first cut is valid.
    auto context = MakeShared<Context>();
    RecutCounts counts;
    const auto recut = [&](float radius, const Vector3& center, bool smooth)
    { counts.Add(radius, center, smooth, MakeRecut(context, radius, center, smooth)); };
    ForEachGridCase(recut);
    ForEachRandomCase(20261119u, 80, recut);
    printf("re-cut sweep: cases %u, first cut not valid %u, second cut not valid %u (duplicates %u, open %u), "
           "empty results %u, volume difference above 1e-3 %u, above 0.1 %u, "
           "added faces %lld, max volume difference %.9g\n",
        counts.cases_, counts.firstFailed_, counts.failed_, counts.duplicates_, counts.open_, counts.empty_,
        counts.volumeAbove1em3_, counts.volumeAbove0p1_, counts.addedFaces_, counts.maxVolumeDifference_);
    CHECK(counts.firstFailed_ == 0);
}

TEST_CASE("Box minus a sphere: a second identical cut at a vertex of the first cut stays valid", "[boolean][regression]")
{
    // The nearest rounding of a crossing in the second cut is often a vertex that the first cut made.
    // The crossing welds to that vertex and does not move. A move gave a duplicate sheet.
    auto context = MakeShared<Context>();
    const Vector3 centers[] = {Vector3(0.300000012f, 0.349999994f, 0.699999988f),
        Vector3(0.300000012f, 0.499023438f, 0.449999988f)};
    for (const Vector3& center : centers)
    {
        INFO("center (" << center.x_ << ", " << center.y_ << ", " << center.z_ << ")");
        const Recut recut = MakeRecut(context, 0.25f, center, true);
        CHECK(IsValid(recut.first_));
        CHECK(IsValid(recut.second_));
        CHECK(recut.second_.check_.duplicates_ == 0);
    }
}

TEST_CASE("Box minus a sphere: a known second cut with a duplicate sheet", "[.][boolean][known]")
{
    // An open case. The first cut is valid, but the second identical cut has one duplicate triangle.
    // A vertex of the first cut is on the edge line of a sphere triangle. The weld rule covers only the vertices of
    // the chain of that edge, so it does not cover this vertex. The crossing moves, and a duplicate sheet stays.
    auto context = MakeShared<Context>();
    const Recut recut = MakeRecut(context, 0.25f, Vector3(0.300000012f, 0.400000006f, 0.499023438f), true);
    CHECK(IsValid(recut.first_));
    // The expected failure. Change the test when the case is fixed.
    CHECK(recut.second_.check_.duplicates_ == 1);
    CHECK(!IsValid(recut.second_));
}

namespace
{

/// The box [0, 1024] x [0, 512] x [0, 1024] in grid units. The diagonal of the top face is y = x / 2.
ea::vector<CsgPolygon> ReviewBox(bool reverseOrder)
{
    return GridBoxTriangles(GridInts{0, 0, 0}, GridInts{1024, 512, 1024}, reverseOrder);
}

void CheckFixture(const BoolResult& result, const char* name)
{
    INFO(name << ": faces " << result.solid_->FaceCount() << " duplicates " << result.check_.duplicates_ << " open "
              << result.check_.openEdges_ << " fold inside " << result.foldInside_ << " unconstrainable "
              << result.unconstrainable_);
    CHECK(IsValid(result));
    CHECK(result.foldInside_ == 0);
    CHECK(result.foldOnEdge_ == 0);
}

} // namespace

TEST_CASE("Boolean: an edge crosses a face diagonal at a point that rounds off the diagonal", "[boolean][regression]")
{
    // The edge P0 P1 crosses the top diagonal of the box at (501, 250.5, 1024). The nearest rounding is (501, 251).
    const GridInts tetra[4] = {{301, 100, 512}, {701, 401, 1536}, {1400, 300, 700}, {-300, 400, 1400}};
    for (const bool reverseA : {false, true})
    {
        for (const bool reverseB : {false, true})
        {
            if (reverseA && reverseB)
                continue;
            const SharedPtr<CsgSolid> a = CsgSolid::FromOperand(ReviewBox(reverseA));
            const SharedPtr<CsgSolid> b = CsgSolid::FromOperand(GridTetrahedron(tetra, reverseB));
            REQUIRE(a->Volume() > 0.0);
            REQUIRE(b->Volume() > 0.0);
            INFO("reverse A " << reverseA << " reverse B " << reverseB);
            for (const CsgOperation operation :
                {CsgOperation::DifferenceAB, CsgOperation::Union, CsgOperation::Intersection})
            {
                INFO("operation " << static_cast<int>(operation));
                const BoolResult result = RunBoolean(*a, *b, operation);
                CHECK(result.solid_->FaceCount() > 0);
                CheckFixture(result, "tetrahedron");
            }
        }
    }
}

TEST_CASE("Boolean: a sphere at the top diagonal of a box, and a second identical cut", "[boolean][regression]")
{
    auto context = MakeShared<Context>();
    const SharedPtr<CsgSolid> a = CsgSolid::FromOperand(ReviewBox(false));
    for (const bool smooth : {true, false})
    {
        INFO("smooth " << smooth);
        const SharedPtr<CsgSolid> b = CsgSolid::FromOperand(
            SpherePolygons(context, 0.3f, Vector3(0.5f + 1.0f / 2048.0f, 0.25f, 1.0f), smooth));
        const BoolResult united = RunBoolean(*a, *b, CsgOperation::Union);
        CheckFixture(united, "union");
        const BoolResult cut = RunBoolean(*a, *b, CsgOperation::DifferenceAB);
        CheckFixture(cut, "cut");
        // A second identical cut is not idempotent: it can add sliver faces at grid points of the first cut, and
        // a crossing with no valid cell corner can stay on the line of a triangle edge. The result stays valid.
        // So the test does not check the face count, and the volume changes only by a small amount.
        const BoolResult again = RunBoolean(*cut.solid_, *b, CsgOperation::DifferenceAB);
        INFO("faces of the first cut " << cut.solid_->FaceCount() << ", of the second cut "
                                       << again.solid_->FaceCount() << ", fold inside " << again.foldInside_
                                       << ", fold on edge " << again.foldOnEdge_);
        CHECK(IsValid(again));
        CHECK(again.check_.volume_ == Catch::Approx(cut.check_.volume_).margin(2e-5));
    }
}

TEST_CASE("Boolean: a tilted box top near the box top, and a cylinder near the box axis", "[boolean][regression]")
{
    auto context = MakeShared<Context>();
    const SharedPtr<CsgSolid> unit = CsgSolid::FromOperand(MakeBoxPolygons(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f), 1));
    SECTION("tilted box")
    {
        // The box top is at z = 1 + 1/4096. The box turns about the center of its top face.
        const Vector3 pivot(0.5f, 0.5f, 1.0f + 1.0f / 4096.0f);
        const Matrix3x4 transform = Matrix3x4(pivot, Quaternion::IDENTITY, 1.0f)
            * Matrix3x4(Vector3::ZERO, Quaternion(5e-4f * 180.0f / 3.14159265f, Vector3::RIGHT)
                    * Quaternion(1e-3f * 180.0f / 3.14159265f, Vector3::FORWARD), 1.0f);
        const SharedPtr<CsgSolid> b = CsgSolid::FromOperand(
            MakeBoxPolygons(Vector3(0.0f, 0.0f, -0.25f), Vector3(0.25f, 0.25f, 0.25f), 2, transform));
        for (const CsgOperation operation : {CsgOperation::DifferenceAB, CsgOperation::Union})
        {
            const BoolResult result = RunBoolean(*unit, *b, operation);
            INFO("tilted box op " << static_cast<int>(operation) << ": fold " << result.fold_ << ", fold inside "
                                  << result.foldInside_ << ", unconstrainable " << result.unconstrainable_);
            CheckFixture(result, "tilted box");
        }
    }
    SECTION("cylinder")
    {
        CsgCylinderParams params;
        params.bottomCenter_ = Vector3(0.0f, -1.0f, 0.0f);
        params.topCenter_ = Vector3(0.0f, 1.0f, 0.0f);
        params.bottomRadius_ = Vector2(0.3f, 0.3f);
        params.topRadius_ = Vector2(0.3f, 0.3f);
        CsgShapeMesh mesh;
        REQUIRE(CsgGenerateCylinder(params, mesh));
        // The local Y axis of the cylinder goes along Z.
        const Matrix3x4 transform(Vector3(0.5f + 1.0f / 2048.0f, 0.5f + 1.0f / 1024.0f, 0.5f),
            Quaternion(90.0f, Vector3::RIGHT), 1.0f);
        const SharedPtr<CsgSolid> b = CsgSolid::FromOperand(MeshPolygons(context, mesh, transform, 2));
        for (const CsgOperation operation : {CsgOperation::DifferenceAB, CsgOperation::Union})
        {
            const BoolResult result = RunBoolean(*unit, *b, operation);
            INFO("cylinder op " << static_cast<int>(operation) << ": fold " << result.fold_ << ", fold inside "
                                << result.foldInside_ << ", unconstrainable " << result.unconstrainable_);
            CheckFixture(result, "cylinder");
        }
    }
}

