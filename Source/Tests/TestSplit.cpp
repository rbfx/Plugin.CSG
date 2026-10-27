// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"

#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <EASTL/algorithm.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

/// Twice the vector area of a loop, from the Newell sum.
Vector3 LoopVectorArea(const ea::vector<Vector3>& positions, const ea::vector<int>& ids)
{
    Vector3 sum = Vector3::ZERO;
    for (unsigned i = 0; i < ids.size(); ++i)
        sum += positions[ids[i]].CrossProduct(positions[ids[(i + 1) % ids.size()]]);
    return sum;
}

Vector3 TriangleVectorArea(const CsgSolid& solid, int triangle)
{
    const auto positions = solid.GetTrianglePositions();
    const Vector3& p0 = positions[triangle * 3];
    return (positions[triangle * 3 + 1] - p0).CrossProduct(positions[triangle * 3 + 2] - p0);
}

double TriangleArea(const CsgSolid& solid, int triangle)
{
    return 0.5 * TriangleVectorArea(solid, triangle).Length();
}

int TriangleCount(const CsgSolid& solid)
{
    const auto start = solid.GetTriangleStart();
    return static_cast<int>(start.back());
}

double SurfaceArea(const CsgSolid& solid)
{
    double area = 0.0;
    for (int t = 0; t < TriangleCount(solid); ++t)
        area += TriangleArea(solid, t);
    return area;
}

uint64_t EdgeKey(int from, int to)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(from)) << 32) | static_cast<uint32_t>(to);
}

int CountKey(const std::vector<uint64_t>& sorted, uint64_t key)
{
    const auto range = std::equal_range(sorted.begin(), sorted.end(), key);
    return static_cast<int>(range.second - range.first);
}

int TotalHoles(const CsgSplitTestOperand& operand)
{
    int total = 0;
    for (int count : operand.holeCount_)
        total += count;
    return total;
}

int CountState(const CsgSplitTestOperand& operand, uint8_t state)
{
    int total = 0;
    for (uint8_t value : operand.triangleState_)
        total += value == state ? 1 : 0;
    return total;
}

/// Area of the surface of the operand after the split: whole triangles and the regions of split triangles.
double SplitArea(const CsgSolid& solid, const CsgSplitTestOperand& operand, const ea::vector<Vector3>& positions)
{
    double area = 0.0;
    for (int t = 0; t < TriangleCount(solid); ++t)
    {
        if (operand.triangleState_[t] != 1)
            area += TriangleArea(solid, t);
    }
    for (const CsgSplitTestRegion& region : operand.regions_)
        area += 0.5 * LoopVectorArea(positions, region.ids_).Length();
    return area;
}

/// Check the structure of the regions of one operand. The curve edges must be the ones of the pair.
void CheckOperand(const CsgSolid& solid, const CsgSplitTestOperand& operand, const CsgSplitTestResult& result)
{
    const int idCount = static_cast<int>(result.positions_.size());
    const int triangleCount = TriangleCount(solid);
    REQUIRE(static_cast<int>(operand.triangleState_.size()) == triangleCount);
    REQUIRE(static_cast<int>(operand.holeCount_.size()) == triangleCount);

    // Regions are grouped by triangle, in the order of the triangles.
    int previousTriangle = -1;
    std::vector<std::vector<uint64_t>> edgesOfTriangle(triangleCount);
    std::vector<uint64_t> allEdges;
    for (const CsgSplitTestRegion& region : operand.regions_)
    {
        REQUIRE(region.triangle_ >= 0);
        REQUIRE(region.triangle_ < triangleCount);
        CHECK(region.triangle_ >= previousTriangle);
        previousTriangle = region.triangle_;
        CHECK(operand.triangleState_[region.triangle_] == 1);

        const int size = static_cast<int>(region.ids_.size());
        REQUIRE(size >= 3);
        // The loop is closed: the last id links to the first id, and every link has two different valid ids.
        for (int i = 0; i < size; ++i)
        {
            const int from = region.ids_[i];
            const int to = region.ids_[(i + 1) % size];
            REQUIRE(from >= 0);
            REQUIRE(from < idCount);
            CHECK(from != to);
            edgesOfTriangle[region.triangle_].push_back(EdgeKey(from, to));
            allEdges.push_back(EdgeKey(from, to));
        }
        // The polygon has one vertex for each id, at the position of the id.
        REQUIRE(static_cast<int>(region.polygonPositions_.size()) == size);
        for (int i = 0; i < size; ++i)
        {
            const Vector3& expected = result.positions_[region.ids_[i]];
            CHECK(std::fabs(region.polygonPositions_[i].x_ - expected.x_) <= 1e-5f);
            CHECK(std::fabs(region.polygonPositions_[i].y_ - expected.y_) <= 1e-5f);
            CHECK(std::fabs(region.polygonPositions_[i].z_ - expected.z_) <= 1e-5f);
        }
        // The region keeps the orientation of the face.
        const Vector3 normal = LoopVectorArea(result.positions_, region.ids_);
        CHECK(normal.DotProduct(TriangleVectorArea(solid, region.triangle_)) >= -1e-6f);
    }

    // In one triangle, no directed edge appears twice. An edge with a reverse is a curve segment or a bridge.
    // The edges without a reverse are the boundary of the triangle: one cycle through distinct ids.
    for (int t = 0; t < triangleCount; ++t)
    {
        std::vector<uint64_t>& edges = edgesOfTriangle[t];
        std::sort(edges.begin(), edges.end());
        CHECK(std::adjacent_find(edges.begin(), edges.end()) == edges.end());
        std::vector<int> outgoing;
        std::vector<int> incoming;
        for (uint64_t key : edges)
        {
            const int from = static_cast<int>(key >> 32);
            const int to = static_cast<int>(key & 0xffffffffu);
            if (std::binary_search(edges.begin(), edges.end(), EdgeKey(to, from)))
                continue;
            outgoing.push_back(from);
            incoming.push_back(to);
        }
        std::sort(outgoing.begin(), outgoing.end());
        std::sort(incoming.begin(), incoming.end());
        CHECK(std::adjacent_find(outgoing.begin(), outgoing.end()) == outgoing.end());
        CHECK(outgoing == incoming);
    }

    // Each curve segment is a directed edge in both directions: once for the region on each side.
    std::sort(allEdges.begin(), allEdges.end());
    for (const auto& edge : result.curveEdges_)
    {
        CHECK(CountKey(allEdges, EdgeKey(edge.first, edge.second)) == 1);
        CHECK(CountKey(allEdges, EdgeKey(edge.second, edge.first)) == 1);
    }
}

void CheckSplit(const CsgSolid& a, const CsgSolid& b, double relativeTolerance = 1e-4)
{
    const CsgSplitTestResult result = CsgCorefineSplitForTest(a, b);
    REQUIRE(!result.positions_.empty());
    CHECK(result.vertexCountA_ + result.vertexCountB_ == result.crossingBase_);

    const double areaA = SurfaceArea(a);
    const double areaB = SurfaceArea(b);
    CHECK(std::fabs(SplitArea(a, result.a_, result.positions_) - areaA) <= relativeTolerance * areaA);
    CHECK(std::fabs(SplitArea(b, result.b_, result.positions_) - areaB) <= relativeTolerance * areaB);
    CHECK(CountState(result.a_, 2) == 0);
    CHECK(CountState(result.b_, 2) == 0);
    CheckOperand(a, result.a_, result);
    CheckOperand(b, result.b_, result);
}

/// Compare the bits, so a NaN equals itself.
bool SameBits(const Vector3& x, const Vector3& y)
{
    return std::memcmp(&x, &y, sizeof(Vector3)) == 0;
}

bool SameBits(const ea::vector<Vector3>& x, const ea::vector<Vector3>& y)
{
    if (x.size() != y.size())
        return false;
    for (unsigned i = 0; i < x.size(); ++i)
    {
        if (!SameBits(x[i], y[i]))
            return false;
    }
    return true;
}

bool SameOperand(const CsgSplitTestOperand& x, const CsgSplitTestOperand& y)
{
    if (x.triangleState_ != y.triangleState_ || x.holeCount_ != y.holeCount_ || x.regions_.size() != y.regions_.size())
        return false;
    for (unsigned i = 0; i < x.regions_.size(); ++i)
    {
        if (x.regions_[i].triangle_ != y.regions_[i].triangle_ || x.regions_[i].ids_ != y.regions_[i].ids_
            || x.regions_[i].bridged_ != y.regions_[i].bridged_
            || !SameBits(x.regions_[i].polygonPositions_, y.regions_[i].polygonPositions_))
            return false;
    }
    return true;
}

bool SameSplit(const CsgSplitTestResult& x, const CsgSplitTestResult& y)
{
    return x.crossingBase_ == y.crossingBase_ && SameBits(x.positions_, y.positions_) && x.curveEdges_ == y.curveEdges_
        && SameOperand(x.a_, y.a_) && SameOperand(x.b_, y.b_);
}

bool AllFinite(const CsgSplitTestResult& result)
{
    for (const Vector3& p : result.positions_)
    {
        if (!std::isfinite(p.x_) || !std::isfinite(p.y_) || !std::isfinite(p.z_))
            return false;
    }
    for (const CsgSplitTestOperand* operand : {&result.a_, &result.b_})
    {
        for (const CsgSplitTestRegion& region : operand->regions_)
        {
            for (const Vector3& p : region.polygonPositions_)
            {
                if (!std::isfinite(p.x_) || !std::isfinite(p.y_) || !std::isfinite(p.z_))
                    return false;
            }
        }
    }
    return true;
}

/// A texture coordinate that is a linear function of the position. Interpolation on a flat face reproduces it.
Vector4 LinearUv(const Vector3& p)
{
    return Vector4(0.37f * p.x_ + 0.11f * p.y_ - 0.2f * p.z_ + 1.0f, -0.5f * p.x_ + 0.3f * p.y_ + 0.23f * p.z_, 0.0f, 0.0f);
}

void SetLinearUv(ea::vector<CsgPolygon>& polygons)
{
    for (CsgPolygon& polygon : polygons)
    {
        for (ModelVertex& vertex : polygon.vertices_)
            vertex.uv_[0] = LinearUv(vertex.GetPosition());
    }
}

void Append(ea::vector<CsgPolygon>& target, ea::vector<CsgPolygon>&& source)
{
    for (CsgPolygon& polygon : source)
        target.push_back(ea::move(polygon));
}

/// Make a closed box shell: the outer box and an inner box turned inside out.
ea::vector<CsgPolygon> MakeShellPolygons(const Vector3& center, const Vector3& outerHalf, const Vector3& innerHalf, unsigned shared)
{
    ea::vector<CsgPolygon> polygons = MakeBoxPolygons(center, outerHalf, shared);
    ea::vector<CsgPolygon> inner = MakeBoxPolygons(center, innerHalf, shared);
    for (CsgPolygon& polygon : inner)
        polygon.Flip();
    Append(polygons, ea::move(inner));
    return polygons;
}

/// Check the interpolated attributes of all the region polygons against the linear function of the position.
int CheckInterpolation(const CsgSplitTestOperand& operand)
{
    int checked = 0;
    for (const CsgSplitTestRegion& region : operand.regions_)
    {
        REQUIRE(region.polygon_.vertices_.size() == region.ids_.size());
        for (const ModelVertex& vertex : region.polygon_.vertices_)
        {
            const Vector4 expected = LinearUv(vertex.GetPosition());
            CHECK(std::fabs(vertex.uv_[0].x_ - expected.x_) <= 5e-4f);
            CHECK(std::fabs(vertex.uv_[0].y_ - expected.y_) <= 5e-4f);
            ++checked;
        }
    }
    return checked;
}

} // namespace

TEST_CASE("Split conserves the area of offset boxes", "[split][corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));
    const CsgSolid b(MakeBoxPolygons(Vector3(1.0f, 0.0f, 0.0f), half, 2));

    CheckSplit(a, b);
    CheckSplit(b, a);

    const CsgSplitTestResult result = CsgCorefineSplitForTest(a, b);
    // The boxes cut each other, so both operands have split triangles.
    CHECK(CountState(result.a_, 1) > 0);
    CHECK(CountState(result.b_, 1) > 0);
    CHECK(result.a_.regions_.size() > static_cast<size_t>(CountState(result.a_, 1)));
    // The curve is a closed loop on the surface of both operands. There is no hole in a triangle.
    CHECK(TotalHoles(result.a_) == 0);
    CHECK(TotalHoles(result.b_) == 0);
}

TEST_CASE("Split conserves the area of a box and a rotated box", "[split][corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const Quaternion rotation(35.0f, Vector3(1.0f, 2.0f, 0.5f).Normalized());
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));
    const CsgSolid b(MakeBoxPolygons(Vector3(0.3f, 0.2f, 0.1f), half, 2, Matrix3x4(Vector3::ZERO, rotation, 1.0f)));
    CheckSplit(a, b);
    CheckSplit(b, a);
}

TEST_CASE("Split conserves the area of a sphere and a box", "[split][corefine]")
{
    const CsgSolid sphere(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.3f), 1.3f, 8, 12, 1));
    const CsgSolid box(MakeBoxPolygons(Vector3::ZERO, Vector3(1.0f, 1.0f, 1.0f), 2));
    CheckSplit(sphere, box);
    CheckSplit(box, sphere);
}

TEST_CASE("Split makes a hole for a closed curve inside a triangle", "[split][corefine]")
{
    // A small box goes through the plate. Its section is a closed curve inside one triangle of each plate face.
    const CsgSolid plate(MakeBoxPolygons(Vector3::ZERO, Vector3(5.0f, 0.25f, 5.0f), 1));
    const CsgSolid post(MakeBoxPolygons(Vector3(3.5f, 0.0f, -1.0f), Vector3(0.5f, 1.0f, 0.5f), 2));

    CheckSplit(plate, post);
    CheckSplit(post, plate);

    const CsgSplitTestResult result = CsgCorefineSplitForTest(plate, post);
    // One hole in the top face and one in the bottom face of the plate. The post has none.
    CHECK(TotalHoles(result.a_) == 2);
    CHECK(TotalHoles(result.b_) == 0);

    // Each hole joins the region around it with two bridges, so it makes two bridged regions.
    int bridged = 0;
    for (const CsgSplitTestRegion& region : result.a_.regions_)
        bridged += region.bridged_ ? 1 : 0;
    CHECK(bridged == 4);
    for (const CsgSplitTestRegion& region : result.b_.regions_)
        CHECK(!region.bridged_);

    // The swapped operands have the holes in the second operand.
    const CsgSplitTestResult swapped = CsgCorefineSplitForTest(post, plate);
    CHECK(TotalHoles(swapped.a_) == 0);
    CHECK(TotalHoles(swapped.b_) == 2);
}

TEST_CASE("Split is deterministic", "[split][corefine]")
{
    const Quaternion rotation(20.0f, Vector3(0.0f, 1.0f, 1.0f).Normalized());
    const CsgSolid sphere(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.3f), 1.3f, 8, 12, 1));
    const CsgSolid box(MakeBoxPolygons(Vector3(0.2f, 0.0f, 0.1f), Vector3(1.0f, 1.0f, 1.0f), 2,
        Matrix3x4(Vector3::ZERO, rotation, 1.0f)));
    const CsgSolid plate(MakeBoxPolygons(Vector3::ZERO, Vector3(5.0f, 0.25f, 5.0f), 1));
    const CsgSolid post(MakeBoxPolygons(Vector3(3.5f, 0.0f, -1.0f), Vector3(0.5f, 1.0f, 0.5f), 2));

    CHECK(SameSplit(CsgCorefineSplitForTest(sphere, box), CsgCorefineSplitForTest(sphere, box)));
    CHECK(SameSplit(CsgCorefineSplitForTest(plate, post), CsgCorefineSplitForTest(plate, post)));
}

TEST_CASE("Split of disjoint solids has no regions", "[split][corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));
    const CsgSolid b(MakeBoxPolygons(Vector3(5.0f, 0.0f, 0.0f), half, 2));
    const CsgSplitTestResult result = CsgCorefineSplitForTest(a, b);
    CHECK(result.a_.regions_.empty());
    CHECK(result.b_.regions_.empty());
    CHECK(CountState(result.a_, 0) == TriangleCount(a));
    CHECK(CountState(result.b_, 0) == TriangleCount(b));
    CHECK(TotalHoles(result.a_) == 0);
}

TEST_CASE("Split of coplanar touching boxes is finite and deterministic", "[split][corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));
    for (float x : {2.0f, -2.0f})
    {
        const CsgSolid b(MakeBoxPolygons(Vector3(x, 0.0f, 0.0f), half, 2));
        const CsgSplitTestResult first = CsgCorefineSplitForTest(a, b);
        const CsgSplitTestResult second = CsgCorefineSplitForTest(a, b);
        CHECK(SameSplit(first, second));
        CHECK(AllFinite(first));
        CHECK(first.a_.regions_.size() < 10000);
        for (const CsgSplitTestRegion& region : first.a_.regions_)
        {
            for (int id : region.ids_)
            {
                CHECK(id >= 0);
                CHECK(id < static_cast<int>(first.positions_.size()));
            }
        }
        // The area of the surface stays the same wherever the perturbation puts the contact.
        const double areaA = SurfaceArea(a);
        CHECK(std::fabs(SplitArea(a, first.a_, first.positions_) - areaA) <= 1e-4 * areaA);
    }
}

TEST_CASE("Split interpolates the attributes of the crossings", "[split][corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const Quaternion rotation(35.0f, Vector3(1.0f, 2.0f, 0.5f).Normalized());
    ea::vector<CsgPolygon> polygonsA = MakeBoxPolygons(Vector3::ZERO, half, 1);
    ea::vector<CsgPolygon> polygonsB = MakeBoxPolygons(Vector3(0.3f, 0.2f, 0.1f), Vector3(0.9f, 1.1f, 1.3f), 2,
        Matrix3x4(Vector3::ZERO, rotation, 1.0f));
    SetLinearUv(polygonsA);
    SetLinearUv(polygonsB);
    const CsgSolid a(polygonsA);
    const CsgSolid b(polygonsB);

    // The crossings on an edge of the operand use the edge lerp. The crossings of the other operand use the
    // barycentric blend. Both reproduce the linear function exactly.
    const CsgSplitTestResult result = CsgCorefineSplitForTest(a, b);
    int crossingVertices = 0;
    for (const CsgSplitTestOperand* operand : {&result.a_, &result.b_})
    {
        const int checked = CheckInterpolation(*operand);
        CHECK(checked > 0);
        for (const CsgSplitTestRegion& region : operand->regions_)
        {
            for (int id : region.ids_)
                crossingVertices += id >= result.crossingBase_ ? 1 : 0;
        }
    }
    CHECK(crossingVertices > 20);
}

TEST_CASE("Split joins two holes of one triangle", "[split][corefine]")
{
    // Two posts go through the plate inside one triangle of each plate face (the wedge x > |z|).
    const CsgSolid plate(MakeBoxPolygons(Vector3::ZERO, Vector3(5.0f, 0.25f, 5.0f), 1));
    ea::vector<CsgPolygon> posts = MakeBoxPolygons(Vector3(3.5f, 0.0f, -1.0f), Vector3(0.5f, 1.0f, 0.5f), 2);
    Append(posts, MakeBoxPolygons(Vector3(3.5f, 0.0f, 1.5f), Vector3(0.5f, 1.0f, 0.5f), 3));
    const CsgSolid twoPosts(posts);

    CheckSplit(plate, twoPosts);
    CheckSplit(twoPosts, plate);
    const CsgSplitTestResult result = CsgCorefineSplitForTest(plate, twoPosts);
    CHECK(TotalHoles(result.a_) == 4);
    int mostHoles = 0;
    for (int count : result.a_.holeCount_)
        mostHoles = std::max(mostHoles, count);
    CHECK(mostHoles == 2);
    CHECK(TotalHoles(result.b_) == 0);
}

TEST_CASE("Split places a hole inside an island", "[split][corefine]")
{
    // A box shell goes through the plate. Its two surfaces cut two nested closed curves in a triangle.
    const CsgSolid plate(MakeBoxPolygons(Vector3::ZERO, Vector3(5.0f, 0.25f, 5.0f), 1));
    const CsgSolid shell(MakeShellPolygons(
        Vector3(3.5f, 0.0f, -1.0f), Vector3(1.2f, 1.0f, 1.2f), Vector3(0.6f, 0.8f, 0.6f), 2));

    CheckSplit(plate, shell);
    CheckSplit(shell, plate);
    const CsgSplitTestResult result = CsgCorefineSplitForTest(plate, shell);
    CHECK(TotalHoles(result.a_) == 4);
    int mostHoles = 0;
    for (int count : result.a_.holeCount_)
        mostHoles = std::max(mostHoles, count);
    CHECK(mostHoles == 2);
    // Each curve has an island: boundary region, two islands and the bridged pieces, in each of the two triangles.
    CHECK(result.a_.regions_.size() >= 2 * 5);
}

namespace
{

CsgPolygon MakeFlatPolygon(std::initializer_list<Vector3> corners)
{
    ea::vector<ModelVertex> vertices;
    for (const Vector3& corner : corners)
    {
        ModelVertex vertex;
        vertex.SetPosition(corner);
        vertices.push_back(vertex);
    }
    return CsgPolygon(ea::move(vertices), 2);
}

/// Result of a split of one triangle with curve segments made by hand.
struct HandSplit
{
    CsgSplitResult::State state_{};
    /// Area of each region in the plane z = 0. It is positive for the counterclockwise order.
    ea::vector<double> areas_;
};

/// Split the one triangle of the operand with the curve segments between the points.
/// The triangle and the points lie in the plane z = 0. The segments do not come from an intersection.
HandSplit SplitWithSegments(const ea::vector<CsgPolygon>& aPolygons, const ea::vector<CsgPolygon>& bPolygons,
    bool splitSecond, const ea::vector<ea::pair<Vector3, Vector3>>& segments)
{
    const CsgSolid a(aPolygons);
    const CsgSolid b(bPolygons);
    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    const auto idOf = [&core](const Vector3& point)
    {
        for (const Mesh* mesh : {&core.a_, &core.b_})
        {
            for (int i = 0; i < mesh->VertexCount(); ++i)
            {
                if (mesh->loops_.positions_[i] == point)
                    return core.Global(*mesh, i);
            }
        }
        FAIL("The point is not a vertex of the operands.");
        return -1;
    };
    const Mesh& mesh = splitSecond ? core.b_ : core.a_;
    REQUIRE(mesh.triangleFace_.size() == 1);
    CsgSegmentLists lists;
    lists.Reset(1);
    for (const auto& segment : segments)
        lists.Append(0, idOf(segment.first), idOf(segment.second));

    CsgScratch scratch;
    CsgSplitResult result;
    core.SplitOperand(mesh, lists, scratch, result);
    HandSplit split;
    split.state_ = result.GetState(0);
    for (int region = 0; region < result.RegionCount(); ++region)
    {
        const int* ids = result.RegionIds(region);
        const int count = result.RegionSize(region);
        double area = 0.0;
        for (int i = 0; i < count; ++i)
        {
            const int p = ids[i];
            const int q = ids[(i + 1) % count];
            const Vector3& pp = p < core.b_.offset_ ? core.a_.loops_.positions_[p] : core.b_.loops_.positions_[p - core.b_.offset_];
            const Vector3& qq = q < core.b_.offset_ ? core.a_.loops_.positions_[q] : core.b_.loops_.positions_[q - core.b_.offset_];
            area += 0.5 * (static_cast<double>(pp.x_) * qq.y_ - static_cast<double>(qq.x_) * pp.y_);
        }
        split.areas_.push_back(area);
    }
    return split;
}

void CheckHandSplit(const HandSplit& split, double triangleArea, unsigned regionCount)
{
    CHECK(split.state_ == CsgSplitResult::State::Split);
    CHECK(split.areas_.size() == regionCount);
    double sum = 0.0;
    for (const double area : split.areas_)
    {
        CHECK(area > 0.0);
        sum += area;
    }
    CHECK(sum == Catch::Approx(triangleArea));
}

} // namespace

TEST_CASE("Split orders a curve that has the direction of a boundary edge", "[split][corefine]")
{
    // At the corner (0, 0) the curve goes to (2, 0) along the next boundary edge and to (0, 2) along the previous
    // boundary edge. The incoming boundary edge must come first in the tie and the outgoing boundary edge last.
    const Vector3 p0(0, 0, 0), p1(4, 0, 0), p2(0, 4, 0);
    const Vector3 c(2, 0, 0), d(1, 1, 0), e(0, 2, 0);
    const ea::vector<ea::pair<Vector3, Vector3>> segments = {{p0, c}, {c, d}, {d, p0}, {p0, e}, {e, d}};
    ea::vector<CsgPolygon> triangle;
    triangle.push_back(MakeFlatPolygon({p0, p1, p2}));
    ea::vector<CsgPolygon> points;
    points.push_back(MakeFlatPolygon({c, d, e}));
    // The curve points have higher ids than the corners in the first split and lower ids in the second split.
    CheckHandSplit(SplitWithSegments(triangle, points, false, segments), 8.0, 3);
    CheckHandSplit(SplitWithSegments(points, triangle, true, segments), 8.0, 3);
}

TEST_CASE("Split places the hole of a curve with two inner faces in the region around it", "[split][corefine]")
{
    const Vector3 p0(0, 0, 0), p1(8, 0, 0), p2(0, 8, 0);
    ea::vector<CsgPolygon> triangle;
    triangle.push_back(MakeFlatPolygon({p0, p1, p2}));
    SECTION("theta")
    {
        // A square with a diagonal. The two inner faces touch every vertex of the hole.
        const Vector3 s0(1, 1, 0), s1(3, 1, 0), s2(3, 3, 0), s3(1, 3, 0);
        ea::vector<CsgPolygon> points;
        points.push_back(MakeFlatPolygon({s0, s1, s2, s3}));
        const ea::vector<ea::pair<Vector3, Vector3>> segments = {{s0, s1}, {s1, s2}, {s2, s3}, {s3, s0}, {s0, s2}};
        CheckHandSplit(SplitWithSegments(triangle, points, false, segments), 32.0, 4);
    }
    SECTION("figure 8")
    {
        // Two triangles that touch at one vertex. The hole visits that vertex twice.
        const Vector3 x0(2, 2, 0), a1(1, 1, 0), a2(3, 1, 0), b1(3, 3, 0), b2(1, 3, 0);
        ea::vector<CsgPolygon> points;
        points.push_back(MakeFlatPolygon({x0, a1, a2}));
        points.push_back(MakeFlatPolygon({x0, b1, b2}));
        const ea::vector<ea::pair<Vector3, Vector3>> segments = {
            {x0, a1}, {a1, a2}, {a2, x0}, {x0, b1}, {b1, b2}, {b2, x0}};
        CheckHandSplit(SplitWithSegments(triangle, points, false, segments), 32.0, 4);
    }
}

namespace
{

CsgGridPoint2 Point2(CsgGridInt u, CsgGridInt v)
{
    CsgGridPoint2 p;
    p.u_ = u;
    p.v_ = v;
    return p;
}

/// Trace a plane graph and check the cycles. Every trace closes. Each boundary edge of the chain is in one cycle.
/// Each curve edge is in one cycle in each direction. A cycle has no walk x -> y -> x.
CsgTraceTestResult CheckTrace(const ea::vector<CsgGridPoint2>& points, const ea::vector<int>& chain,
    const ea::vector<ea::pair<int, int>>& curves)
{
    const CsgTraceTestResult result = CsgTraceRegionsForTest(points, chain, curves);
    CHECK(result.closed_);
    std::vector<uint64_t> edges;
    for (const ea::vector<int>& cycle : result.cycles_)
    {
        const int size = static_cast<int>(cycle.size());
        CHECK(size >= 3);
        for (int i = 0; i < size; ++i)
        {
            edges.push_back(EdgeKey(cycle[i], cycle[(i + 1) % size]));
            CHECK(cycle[i] != cycle[(i + 2) % size]);
        }
    }
    std::sort(edges.begin(), edges.end());
    CHECK(std::adjacent_find(edges.begin(), edges.end()) == edges.end());
    // A walk x -> y -> x of the chain has no area. Its two edges cancel, so they are in no cycle.
    const int count = static_cast<int>(chain.size());
    for (int i = 0; i < count; ++i)
    {
        const int from = chain[i];
        const int to = chain[(i + 1) % count];
        const bool spur = chain[(i + 2) % count] == from || chain[(i + count - 1) % count] == to;
        CHECK(CountKey(edges, EdgeKey(from, to)) == (spur ? 0 : 1));
    }
    for (const auto& curve : curves)
    {
        CHECK(CountKey(edges, EdgeKey(curve.first, curve.second)) == 1);
        CHECK(CountKey(edges, EdgeKey(curve.second, curve.first)) == 1);
    }
    return result;
}

/// Points of the triangle (0, 0), (100, 0), (0, 100) with the ids 0, 1 and 2, and more points after them.
ea::vector<CsgGridPoint2> TracePoints(std::initializer_list<CsgGridPoint2> more)
{
    ea::vector<CsgGridPoint2> points = {Point2(0, 0), Point2(100, 0), Point2(0, 100)};
    points.insert(points.end(), more.begin(), more.end());
    return points;
}

} // namespace

TEST_CASE("Trace closes with a curve at a corner beyond the incoming edge", "[split]")
{
    // The curve 1 - 3 - 4 starts at the corner 1. The point 3 is below the edge 0 - 1, outside the triangle.
    // The curve 1 - 5 - 4 starts at the same corner and goes into the triangle. So the sector rule runs.
    const auto points = TracePoints({Point2(60, -1), Point2(0, 50), Point2(50, 20)});
    const auto result = CheckTrace(points, {0, 1, 2, 4}, {{1, 3}, {3, 4}, {1, 5}, {5, 4}});
    CHECK(result.cycles_.size() == 3);
}

TEST_CASE("Trace closes with a curve at a corner beyond the outgoing edge", "[split]")
{
    // The point 3 is beyond the edge 1 - 2. The curve 1 - 5 - 4 goes into the triangle.
    const auto points = TracePoints({Point2(70, 40), Point2(0, 50), Point2(50, 20)});
    const auto result = CheckTrace(points, {0, 1, 2, 4}, {{1, 3}, {3, 4}, {1, 5}, {5, 4}});
    CHECK(result.cycles_.size() == 3);
}

TEST_CASE("Trace closes with a curve at a corner behind the corner", "[split]")
{
    // The point 3 is beyond both edges at the corner 1. The curve 1 - 5 - 4 goes into the triangle.
    const auto points = TracePoints({Point2(120, -10), Point2(0, 50), Point2(50, 20)});
    const auto result = CheckTrace(points, {0, 1, 2, 4}, {{1, 3}, {3, 4}, {1, 5}, {5, 4}});
    CHECK(result.cycles_.size() == 3);
}

TEST_CASE("Trace closes with a crossing id on two edges of the triangle", "[split]")
{
    // The id 3 is on the edge 0 - 1 and on the edge 1 - 2. It is at the point of the corner 1.
    const auto points = TracePoints({Point2(100, 0), Point2(50, 20), Point2(0, 50)});
    CheckTrace(points, {0, 3, 1, 3, 2, 5}, {{3, 4}, {4, 5}});
}

TEST_CASE("Trace closes with two curves at one corner and one of them outside", "[split]")
{
    // At the corner 1, the curve to 3 is inside the triangle and the curve to 4 is below the edge 0 - 1.
    const auto points = TracePoints({Point2(50, 20), Point2(60, -1), Point2(0, 50)});
    const auto result = CheckTrace(points, {0, 1, 2, 5}, {{1, 3}, {3, 5}, {1, 4}, {4, 5}});
    CHECK(result.cycles_.size() == 3);
}

TEST_CASE("Trace closes with curves at a crossing beyond its edge", "[split]")
{
    // The crossing 3 is on the edge 0 - 1. The curves go to 4 below the edge and to 5 inside the triangle.
    const auto points = TracePoints({Point2(50, 0), Point2(40, -2), Point2(30, 30), Point2(0, 50)});
    const auto result = CheckTrace(points, {0, 3, 1, 2, 6}, {{3, 4}, {4, 6}, {3, 5}, {5, 6}});
    CHECK(result.cycles_.size() == 3);
}

namespace
{

/// Sorted ids of the cycle that holds the directed edge from - to. It is empty if no cycle holds that edge.
std::vector<int> CycleWithEdge(const CsgTraceTestResult& result, int from, int to)
{
    for (const ea::vector<int>& cycle : result.cycles_)
    {
        const int size = static_cast<int>(cycle.size());
        for (int i = 0; i < size; ++i)
        {
            if (cycle[i] == from && cycle[(i + 1) % size] == to)
            {
                std::vector<int> ids(cycle.begin(), cycle.end());
                std::sort(ids.begin(), ids.end());
                return ids;
            }
        }
    }
    return {};
}

} // namespace

TEST_CASE("Trace of a chord that crosses a boundary edge on the grid keeps two regions", "[split]")
{
    // The chord 3 - 5 - 4 goes from the edge 0 - 1 to the edge 2 - 0. The rounded point 5 is below the edge 0 - 1,
    // so the piece 5 - 4 crosses the boundary edge 0 - 3 on the grid. The rounded region 0 - 3 - 5 - 4 is a bow tie.
    // The trace does not use the points at a chain vertex with one curve or at a curve vertex with two curves.
    // So the regions are the regions of the exact map.
    const auto points = TracePoints({Point2(50, 0), Point2(0, 50), Point2(60, -5)});
    const auto result = CheckTrace(points, {0, 3, 1, 2, 4}, {{3, 5}, {5, 4}});
    REQUIRE(result.cycles_.size() == 2);
    CHECK(CycleWithEdge(result, 0, 3) == std::vector<int>{0, 3, 4, 5});
    CHECK(CycleWithEdge(result, 3, 1) == std::vector<int>{1, 2, 3, 4, 5});
}

TEST_CASE("Trace of a chord with folded curve vertices keeps two regions", "[split]")
{
    // The chord 3 - 5 - 6 - 4 has two curve vertices of degree two. The point 5 is beyond the edge 1 - 2 and the
    // point 6 is near the edge 0 - 1, so the rounded chord folds over itself. Each region keeps its exact ids.
    const auto points = TracePoints({Point2(50, 0), Point2(0, 50), Point2(90, 40), Point2(20, 2)});
    const auto result = CheckTrace(points, {0, 3, 1, 2, 4}, {{3, 5}, {5, 6}, {6, 4}});
    REQUIRE(result.cycles_.size() == 2);
    CHECK(CycleWithEdge(result, 0, 3) == std::vector<int>{0, 3, 4, 5, 6});
    CHECK(CycleWithEdge(result, 3, 1) == std::vector<int>{1, 2, 3, 4, 5, 6});
}

TEST_CASE("Trace of a folded closed curve gives one cycle in each direction", "[split]")
{
    // The closed curve 3 - 4 - 5 - 6 is inside the triangle. Its rounded points make a bow tie. The trace gives the
    // boundary cycle and the closed curve once in each direction. The hole and its island have the same ids.
    const auto points = TracePoints({Point2(10, 10), Point2(40, 10), Point2(10, 40), Point2(40, 40)});
    const auto result = CheckTrace(points, {0, 1, 2}, {{3, 4}, {4, 5}, {5, 6}, {6, 3}});
    REQUIRE(result.cycles_.size() == 3);
    CHECK(CycleWithEdge(result, 0, 1) == std::vector<int>{0, 1, 2});
    CHECK(CycleWithEdge(result, 3, 4) == std::vector<int>{3, 4, 5, 6});
    CHECK(CycleWithEdge(result, 4, 3) == std::vector<int>{3, 4, 5, 6});
}
