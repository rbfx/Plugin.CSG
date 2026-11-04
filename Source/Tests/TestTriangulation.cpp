// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoolean.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "Urho3D/CSG/CsgTriangulation.h"

#include "TestGeometry.h"

#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

using Point = std::pair<float, float>;

/// Make a polygon in the plane z = 0 from 2D points. The plane normal is +Z, or -Z if the winding is clockwise.
CsgPolygon MakePolygon2D(const std::vector<Point>& points, bool clockwise = false)
{
    CsgPolygon polygon;
    for (const Point& point : points)
    {
        ModelVertex vertex;
        vertex.SetPosition(Vector3(point.first, point.second, 0.0f));
        polygon.vertices_.push_back(vertex);
    }
    polygon.plane_ = Plane(Vector3(0.0f, 0.0f, clockwise ? -1.0f : 1.0f), Vector3::ZERO);
    return polygon;
}

/// Make a polygon from 3D positions with the given plane normal.
CsgPolygon MakePolygon3D(const std::vector<Vector3>& positions, const Vector3& normal)
{
    CsgPolygon polygon;
    for (const Vector3& position : positions)
    {
        ModelVertex vertex;
        vertex.SetPosition(position);
        polygon.vertices_.push_back(vertex);
    }
    polygon.plane_ = Plane(normal, positions[0]);
    return polygon;
}

double SignedArea(const CsgPolygon& polygon, unsigned a, unsigned b, unsigned c)
{
    const Vector3 pa = polygon.vertices_[a].GetPosition();
    const Vector3 pb = polygon.vertices_[b].GetPosition();
    const Vector3 pc = polygon.vertices_[c].GetPosition();
    return 0.5 * ((double(pb.x_) - pa.x_) * (double(pc.y_) - pa.y_) - (double(pb.y_) - pa.y_) * (double(pc.x_) - pa.x_));
}

double PolygonArea(const std::vector<Point>& points)
{
    double area = 0.0;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Point& p = points[i];
        const Point& q = points[(i + 1) % points.size()];
        area += double(p.first) * q.second - double(q.first) * p.second;
    }
    return 0.5 * area;
}

double TriangulatedArea(const CsgPolygon& polygon, const ea::vector<unsigned>& indices)
{
    double area = 0.0;
    for (unsigned t = 0; t + 2 < indices.size(); t += 3)
        area += SignedArea(polygon, indices[t], indices[t + 1], indices[t + 2]);
    return area;
}

bool IndicesValid(const CsgPolygon& polygon, const ea::vector<unsigned>& indices)
{
    if (indices.size() != (polygon.vertices_.size() - 2) * 3)
        return false;
    for (const unsigned index : indices)
    {
        if (index >= polygon.vertices_.size())
            return false;
    }
    return true;
}

/// Check the triangles of a clean polygon: the count, the area and the orientation.
void CheckProperTriangulation(const std::vector<Point>& points)
{
    for (const bool reversed : {false, true})
    {
        std::vector<Point> ordered = points;
        if (reversed)
            std::reverse(ordered.begin(), ordered.end());
        const CsgPolygon polygon = MakePolygon2D(ordered, reversed);
        const double expected = PolygonArea(ordered);

        ea::vector<unsigned> indices;
        REQUIRE(CsgTriangulatePolygon(polygon, indices));
        CHECK(indices.size() == (ordered.size() - 2) * 3);
        CHECK(IndicesValid(polygon, indices));
        CHECK(TriangulatedArea(polygon, indices) == Catch::Approx(expected).margin(1e-9));
        for (unsigned t = 0; t + 2 < indices.size(); t += 3)
        {
            // Every triangle has the winding of the polygon and a real area.
            const double area = SignedArea(polygon, indices[t], indices[t + 1], indices[t + 2]);
            CHECK(area * expected > 1e-9);
        }
    }
}

bool InsideConvex(const std::vector<Point>& quad, double x, double y)
{
    // The quad is clockwise, so the point is inside if it is on the right of each edge.
    for (size_t i = 0; i < quad.size(); ++i)
    {
        const Point& p = quad[i];
        const Point& q = quad[(i + 1) % quad.size()];
        const double cross = (double(q.first) - p.first) * (y - p.second) - (double(q.second) - p.second) * (x - p.first);
        if (cross > 1e-9)
            return false;
    }
    return true;
}

// The face of a union of boxes that touch: a bridge between two loops makes a zero-width spike.
const std::vector<Point> SPIKE_POLYGON = {
    {0, 1}, {2, 1}, {2, -1}, {1, 0}, {1, 1}, {1, 1}, {1, 1}, {0, 1}};
// The area of the spike polygon is the area of this quad. It is clockwise.
const std::vector<Point> SPIKE_REGION = {{1, 1}, {2, 1}, {2, -1}, {1, 0}};

using SolidPtr = SharedPtr<const CsgSolid>;

SolidPtr MakeKeptSolid(const std::vector<CsgPolygon>& polygons)
{
    ea::vector<CsgSharedPolygon> faces;
    for (const CsgPolygon& polygon : polygons)
        faces.push_back(ea::make_shared<CsgPolygon>(polygon));
    return SolidPtr(CsgSolid::FromKeptFaces(ea::move(faces), CSG_DEFAULT_EPSILON, {}, {}, 0));
}

SolidPtr MakeBoxSolid(const Vector3& center, unsigned shared)
{
    return SolidPtr(CsgSolid::FromOperand(MakeBoxPolygons(center, Vector3::ONE, shared)));
}

/// Return the edges of the triangles of a face that lie on the x axis, as sorted pairs of x.
std::vector<std::pair<float, float>> AxisEdges(const CsgSolid& solid, unsigned face)
{
    std::vector<std::pair<float, float>> result;
    const auto positions = solid.GetTrianglePositions();
    const auto start = solid.GetTriangleStart();
    for (unsigned t = start[face]; t < start[face + 1]; ++t)
    {
        for (unsigned corner = 0; corner < 3; ++corner)
        {
            const Vector3& p = positions[t * 3 + corner];
            const Vector3& q = positions[t * 3 + (corner + 1) % 3];
            if (p.y_ == 0.0f && p.z_ == 0.0f && q.y_ == 0.0f && q.z_ == 0.0f && p.x_ != q.x_)
                result.emplace_back(std::min(p.x_, q.x_), std::max(p.x_, q.x_));
        }
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

struct TriangleMesh
{
    std::vector<Vector3> positions_;
    double area_{};
    double volume_{};
    std::vector<unsigned> geometryTriangles_;
};

/// Read the triangles of all geometries of a triangulated solid.
TriangleMesh ReadModel(const CsgTriangulatedModel& model)
{
    TriangleMesh mesh;
    for (const auto& geometry : model.geometries_)
    {
        mesh.geometryTriangles_.push_back(geometry.indexData_.size() / 3);
        for (unsigned t = 0; t + 2 < geometry.indexData_.size(); t += 3)
        {
            const Vector3 a = geometry.vertices_[geometry.indexData_[t]].GetPosition();
            const Vector3 b = geometry.vertices_[geometry.indexData_[t + 1]].GetPosition();
            const Vector3 c = geometry.vertices_[geometry.indexData_[t + 2]].GetPosition();
            mesh.positions_.push_back(a);
            mesh.positions_.push_back(b);
            mesh.positions_.push_back(c);
            mesh.area_ += 0.5 * (b - a).CrossProduct(c - a).Length();
            mesh.volume_ += double(a.DotProduct(b.CrossProduct(c))) / 6.0;
        }
    }
    return mesh;
}

/// Return true if each directed edge of the triangles has its opposite edge. Zero-length edges are ignored.
bool IsMeshClosed(const TriangleMesh& mesh)
{
    std::vector<std::pair<IntVector3, IntVector3>> edges;
    const float scale = 1.0f / CSG_DEFAULT_EPSILON;
    for (size_t i = 0; i + 2 < mesh.positions_.size(); i += 3)
    {
        const IntVector3 q[3] = {QuantizePosition(mesh.positions_[i], scale),
            QuantizePosition(mesh.positions_[i + 1], scale), QuantizePosition(mesh.positions_[i + 2], scale)};
        for (int k = 0; k < 3; ++k)
        {
            if (q[k] != q[(k + 1) % 3])
                edges.emplace_back(q[k], q[(k + 1) % 3]);
        }
    }
    const auto less = [](const IntVector3& x, const IntVector3& y)
    { return std::make_tuple(x.x_, x.y_, x.z_) < std::make_tuple(y.x_, y.y_, y.z_); };
    std::vector<std::pair<IntVector3, IntVector3>> forward = edges;
    std::vector<std::pair<IntVector3, IntVector3>> backward;
    for (const auto& e : edges)
        backward.emplace_back(e.second, e.first);
    const auto pairLess = [&](const std::pair<IntVector3, IntVector3>& x, const std::pair<IntVector3, IntVector3>& y)
    {
        if (less(x.first, y.first) || less(y.first, x.first))
            return less(x.first, y.first);
        return less(x.second, y.second);
    };
    std::sort(forward.begin(), forward.end(), pairLess);
    std::sort(backward.begin(), backward.end(), pairLess);
    return forward.size() == backward.size()
        && std::equal(forward.begin(), forward.end(), backward.begin(),
            [](const auto& x, const auto& y) { return x.first == y.first && x.second == y.second; });
}

/// Count the triangles of a solid with a normal against the normal of their face. Faces whose polygon has no area
/// are skipped if `onlyFacesWithArea` is set.
unsigned CountInvertedTriangles(const CsgSolid& solid, bool onlyFacesWithArea)
{
    unsigned inverted = 0;
    const auto positions = solid.GetTrianglePositions();
    for (unsigned f = 0; f < solid.FaceCount(); ++f)
    {
        const CsgPolygon& polygon = *solid.GetFace(f);
        const Vector3 normal = polygon.plane_.normal_;
        if (onlyFacesWithArea)
        {
            double shoelace = 0.0;
            for (size_t i = 0; i < polygon.vertices_.size(); ++i)
            {
                const Vector3 p = polygon.vertices_[i].GetPosition();
                const Vector3 q = polygon.vertices_[(i + 1) % polygon.vertices_.size()].GetPosition();
                shoelace += 0.5 * double(p.CrossProduct(q).DotProduct(normal));
            }
            if (std::abs(shoelace) < 1e-6)
                continue;
        }
        for (unsigned t = solid.GetTriangleStart()[f]; t < solid.GetTriangleStart()[f + 1]; ++t)
        {
            const Vector3 ab = positions[t * 3 + 1] - positions[t * 3];
            const Vector3 ac = positions[t * 3 + 2] - positions[t * 3];
            if (double(ab.CrossProduct(ac).DotProduct(normal)) < -1e-6)
                ++inverted;
        }
    }
    return inverted;
}

/// Return the next number of a deterministic generator.
unsigned NextRandom(unsigned& state)
{
    state = state * 1664525u + 1013904223u;
    return state >> 8;
}

} // namespace

TEST_CASE("Triangulate a convex polygon", "[triangulate]")
{
    CheckProperTriangulation({{0, 0}, {3, 0}, {4, 2}, {2, 4}, {-1, 2}});
}

TEST_CASE("Triangulate a concave polygon", "[triangulate]")
{
    // L shape and an arrow head.
    CheckProperTriangulation({{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}});
    CheckProperTriangulation({{0, 0}, {4, 1}, {0, 2}, {1, 1}});
}

TEST_CASE("Triangulate a polygon with a collinear run", "[triangulate]")
{
    // The vertices on the bottom and right edges are on straight runs. They never make a flat triangle.
    CheckProperTriangulation({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 1}, {3, 2}, {0, 2}});
}

TEST_CASE("Triangulate the unit square in the ear order of the reference", "[triangulate]")
{
    // The first ear is the tip at index 0 and then the remaining three vertices.
    const CsgPolygon square = MakePolygon2D({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    ea::vector<unsigned> indices;
    REQUIRE(CsgTriangulatePolygon(square, indices));
    const unsigned expected[] = {3, 0, 1, 1, 2, 3};
    REQUIRE(indices.size() == 6);
    CHECK(std::equal(indices.begin(), indices.end(), expected));

    const CsgPolygon triangle = MakePolygon2D({{0, 0}, {1, 0}, {0, 1}});
    REQUIRE(CsgTriangulatePolygon(triangle, indices));
    CHECK(indices.size() == 3);

    CsgPolygon tiny = MakePolygon2D({{0, 0}, {1, 0}});
    CHECK_FALSE(CsgTriangulatePolygon(tiny, indices));
}

TEST_CASE("Triangulate a polygon with a zero-width spike", "[triangulate]")
{
    const CsgPolygon polygon = MakePolygon2D(SPIKE_POLYGON, true);
    ea::vector<unsigned> indices;
    bool clean = false;
    CsgTriangulationScratch scratch;
    REQUIRE(CsgTriangulatePolygon(polygon, indices, {}, clean, scratch));
    CHECK(clean);
    REQUIRE(IndicesValid(polygon, indices));

    // The spike has no area, so the triangles cover the area of the true region.
    CHECK(PolygonArea(SPIKE_POLYGON) == Catch::Approx(-1.5));
    CHECK(TriangulatedArea(polygon, indices) == Catch::Approx(-1.5).margin(1e-9));
    for (unsigned t = 0; t + 2 < indices.size(); t += 3)
    {
        // No triangle has the opposite winding, and a triangle with an area lies in the true region.
        const double area = SignedArea(polygon, indices[t], indices[t + 1], indices[t + 2]);
        CHECK(area < 1e-9);
        if (area < -1e-9)
        {
            double x = 0.0;
            double y = 0.0;
            for (unsigned k = 0; k < 3; ++k)
            {
                x += polygon.vertices_[indices[t + k]].GetPosition().x_ / 3.0;
                y += polygon.vertices_[indices[t + k]].GetPosition().y_ / 3.0;
            }
            CHECK(InsideConvex(SPIKE_REGION, x, y));
        }
    }
}

TEST_CASE("Triangulate a polygon with a spike that crosses the first ear", "[triangulate]")
{
    // After the needle at index 0 is clipped, the ear (0,1) (2,1) (2,-1) must not be accepted: the spike from
    // (1,1) to (0,1) crosses it.
    const std::vector<Point> points = {{0, 1}, {2, 1}, {2, -1}, {1, -1}, {1, 1}, {0, 1}};
    const std::vector<Point> region = {{1, 1}, {2, 1}, {2, -1}, {1, -1}};
    const CsgPolygon polygon = MakePolygon2D(points, true);
    ea::vector<unsigned> indices;
    bool clean = false;
    CsgTriangulationScratch scratch;
    REQUIRE(CsgTriangulatePolygon(polygon, indices, {}, clean, scratch));
    CHECK(clean);
    REQUIRE(IndicesValid(polygon, indices));
    CHECK(TriangulatedArea(polygon, indices) == Catch::Approx(-2.0).margin(1e-9));
    for (unsigned t = 0; t + 2 < indices.size(); t += 3)
    {
        const double area = SignedArea(polygon, indices[t], indices[t + 1], indices[t + 2]);
        CHECK(area < 1e-9);
        if (area < -1e-9)
        {
            double x = 0.0;
            double y = 0.0;
            for (unsigned k = 0; k < 3; ++k)
            {
                x += polygon.vertices_[indices[t + k]].GetPosition().x_ / 3.0;
                y += polygon.vertices_[indices[t + k]].GetPosition().y_ / 3.0;
            }
            CHECK(InsideConvex(region, x, y));
        }
    }
}

TEST_CASE("Triangulate a polygon with duplicate positions", "[triangulate]")
{
    const std::vector<Point> doubled = {{0, 0}, {0, 0}, {2, 0}, {2, 0}, {2, 2}, {2, 2}, {0, 2}, {0, 2}};
    const CsgPolygon polygon = MakePolygon2D(doubled);
    ea::vector<unsigned> indices;
    bool clean = false;
    CsgTriangulationScratch scratch;
    REQUIRE(CsgTriangulatePolygon(polygon, indices, {}, clean, scratch));
    CHECK(clean);
    CHECK(IndicesValid(polygon, indices));
    CHECK(TriangulatedArea(polygon, indices) == Catch::Approx(4.0).margin(1e-9));
    for (unsigned t = 0; t + 2 < indices.size(); t += 3)
        CHECK(SignedArea(polygon, indices[t], indices[t + 1], indices[t + 2]) > -1e-9);
}

TEST_CASE("Triangulate with shared runs gives a matching split of the run", "[triangulate]")
{
    // The vertex (1.9995, 0) is within an end of the diagonal. Without the mark, the ear at the apex has the whole
    // run as its edge and skips the vertex. With the mark, every triangle uses the vertex.
    const CsgPolygon polygon = MakePolygon2D({{1, 0.5f}, {2, 0}, {1.9995f, 0}, {0, 0}});
    ea::vector<unsigned> plain;
    ea::vector<unsigned> marked;
    bool cleanPlain = false;
    bool cleanMarked = false;
    CsgTriangulationScratch scratch;
    REQUIRE(CsgTriangulatePolygon(polygon, plain, {}, cleanPlain, scratch));
    const bool marks[4] = {false, false, true, false};
    REQUIRE(CsgTriangulatePolygon(polygon, marked, marks, cleanMarked, scratch));
    CHECK(cleanPlain);
    CHECK(cleanMarked);
    CHECK(plain != marked);
    REQUIRE(marked.size() == 6);
    // Both triangles contain the marked vertex 2.
    CHECK((marked[0] == 2 || marked[1] == 2 || marked[2] == 2));
    CHECK((marked[3] == 2 || marked[4] == 2 || marked[5] == 2));

    // A mark list of the wrong size is ignored.
    ea::vector<unsigned> wrongSize;
    REQUIRE(CsgTriangulatePolygon(polygon, wrongSize, ea::span<const bool>(marks, 3), cleanMarked, scratch));
    CHECK(wrongSize == plain);
}

TEST_CASE("Solid faces that share a run split it the same way", "[triangulate][solid]")
{
    // Two faces meet along the x axis from x = 0 to x = 2. Both have the vertex (1.9995, 0, 0) on the run.
    const Vector3 apex(1.0f, 0.0f, 0.5f);
    const CsgPolygon floor = MakePolygon3D({{0, 0, 0}, {1.9995f, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}}, Vector3(0, 0, 1));
    const CsgPolygon wall = MakePolygon3D({apex, {2, 0, 0}, {1.9995f, 0, 0}, {0, 0, 0}}, Vector3(0, 1, 0));
    const SolidPtr solid = MakeKeptSolid({floor, wall});
    REQUIRE(solid->FaceCount() == 2);

    // The later face of the pair keeps the run split.
    const std::vector<std::pair<float, float>> expected = {{0.0f, 1.9995f}, {1.9995f, 2.0f}};
    CHECK(AxisEdges(*solid, 0) == expected);
    CHECK(AxisEdges(*solid, 1) == expected);
    CHECK_FALSE(solid->IsFallbackFace(0));
    CHECK_FALSE(solid->IsFallbackFace(1));
    CHECK(solid->GetTriangleStart()[1] - solid->GetTriangleStart()[0] == 3);
    CHECK(solid->GetTriangleStart()[2] - solid->GetTriangleStart()[1] == 2);
}

TEST_CASE("Triangulate reports an unclean result for a degenerate polygon", "[triangulate]")
{
    // All vertices are on one line. No proper ear exists, so the polygon gets flat triangles.
    const CsgPolygon line = MakePolygon2D({{0, 0}, {1, 0}, {2, 0}, {3, 0}});
    ea::vector<unsigned> indices;
    bool clean = true;
    CsgTriangulationScratch scratch;
    REQUIRE(CsgTriangulatePolygon(line, indices, {}, clean, scratch));
    CHECK_FALSE(clean);
    CHECK(IndicesValid(line, indices));

    // The two-argument function gives the same triangles.
    ea::vector<unsigned> same;
    REQUIRE(CsgTriangulatePolygon(line, same));
    CHECK(same == indices);

    // A proper polygon is clean, and the flag is set again on each call.
    const CsgPolygon square = MakePolygon2D({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    REQUIRE(CsgTriangulatePolygon(square, indices, {}, clean, scratch));
    CHECK(clean);
}

TEST_CASE("Solid lists a degenerate face as a fallback face", "[triangulate][solid]")
{
    const CsgPolygon line = MakePolygon2D({{0, 0}, {1, 0}, {2, 0}, {3, 0}});
    const CsgPolygon square = MakePolygon2D({{0, 1}, {1, 1}, {1, 2}, {0, 2}});
    const CsgPolygon triangle = MakePolygon2D({{5, 0}, {6, 0}, {5, 1}});
    const SolidPtr solid = MakeKeptSolid({square, line, triangle});
    REQUIRE(solid->FaceCount() == 3);
    CHECK_FALSE(solid->IsFallbackFace(0));
    CHECK(solid->IsFallbackFace(1));
    CHECK_FALSE(solid->IsFallbackFace(2));

    // The face still has its triangles, and CopyTriangles triangulates it again.
    CHECK(solid->GetTriangleStart()[2] - solid->GetTriangleStart()[1] == 2);
    ea::vector<unsigned> copy;
    REQUIRE(solid->CopyTriangles(1, copy));
    CHECK(copy.size() == 6);
    REQUIRE(solid->CopyTriangles(0, copy));
    CHECK(copy.size() == 6);
}

TEST_CASE("Triangulate is deterministic", "[triangulate]")
{
    const CsgPolygon spike = MakePolygon2D(SPIKE_POLYGON, true);
    const CsgPolygon concave = MakePolygon2D({{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}});
    ea::vector<unsigned> first;
    ea::vector<unsigned> second;
    ea::vector<unsigned> other;
    CsgTriangulationScratch shared;
    bool clean = true;
    REQUIRE(CsgTriangulatePolygon(spike, first));
    // A scratch that held another polygon gives the same result as a new one.
    REQUIRE(CsgTriangulatePolygon(concave, other, {}, clean, shared));
    REQUIRE(CsgTriangulatePolygon(spike, second, {}, clean, shared));
    CHECK(first == second);

    // Two solids from the same input have the same triangles.
    const SolidPtr a = MakeBoxSolid(Vector3::ZERO, 1);
    const SolidPtr b = MakeBoxSolid(Vector3(1, 0, 0), 2);
    const SolidPtr x = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON);
    const SolidPtr y = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON);
    REQUIRE(x->FaceCount() == y->FaceCount());
    CHECK(x->GetTriangleVertices().size() == y->GetTriangleVertices().size());
    CHECK(std::equal(x->GetTriangleVertices().begin(), x->GetTriangleVertices().end(), y->GetTriangleVertices().begin()));
    CHECK(std::equal(x->GetTriangleStart().begin(), x->GetTriangleStart().end(), y->GetTriangleStart().begin()));
}

TEST_CASE("Triangulate does not fail on a vertex that is not a number", "[triangulate]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const std::vector<std::vector<Point>> cases = {
        {{0, 0}, {1, 0}, {nan, 1}, {0, 1}},
        {{0, 0}, {1, 0}, {inf, 1}, {0, 1}, {-1, 0.5f}},
        {{nan, nan}, {nan, nan}, {nan, nan}, {nan, nan}, {nan, nan}},
        {{0, 0}, {inf, 0}, {inf, inf}, {0, inf}},
    };
    for (const auto& points : cases)
    {
        const CsgPolygon polygon = MakePolygon2D(points);
        ea::vector<unsigned> indices;
        bool clean = true;
        CsgTriangulationScratch scratch;
        CHECK(CsgTriangulatePolygon(polygon, indices, {}, clean, scratch));
        CHECK(IndicesValid(polygon, indices));
    }

    // A normal that is not a number gives the default axes.
    CsgPolygon badNormal = MakePolygon2D({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    badNormal.plane_.normal_ = Vector3(nan, nan, nan);
    ea::vector<unsigned> indices;
    CHECK(CsgTriangulatePolygon(badNormal, indices));
    CHECK(IndicesValid(badNormal, indices));
}

TEST_CASE("Triangulated union of boxes has no hole and no inverted triangle", "[triangulate][boolean]")
{
    const SolidPtr a = MakeBoxSolid(Vector3::ZERO, 1);
    const SolidPtr b = MakeBoxSolid(Vector3(1, 0, 0), 2);
    const SolidPtr result = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON);
    REQUIRE(result->FaceCount() > 0);

    // Each face has triangles. The union has faces with spikes.
    bool anySpike = false;
    for (unsigned f = 0; f < result->FaceCount(); ++f)
    {
        CHECK(result->GetTriangleStart()[f + 1] > result->GetTriangleStart()[f]);
        anySpike = anySpike || result->GetFace(f)->vertices_.size() > 4;
    }
    CHECK(anySpike);

    // The union is the prism [-1, 2] x [-1, 1] x [-1, 1]. Its surface is 2 * (3 * 2 + 3 * 2 + 2 * 2) = 32.
    const SharedPtr<CsgTriangulatedModel> model = CsgTriangulateSolid(*result);
    REQUIRE(model);
    REQUIRE(model->hasAnyVertex_);
    const TriangleMesh mesh = ReadModel(*model);
    // The area along the face normals is 32. A face without area, like a zero-width ribbon, can have triangles
    // that cancel. So the area of all triangles is not a measure, but the signed area is.
    double signedArea = 0.0;
    const auto positions = result->GetTrianglePositions();
    for (unsigned f = 0; f < result->FaceCount(); ++f)
    {
        const Vector3 normal = result->GetFace(f)->plane_.normal_;
        for (unsigned t = result->GetTriangleStart()[f]; t < result->GetTriangleStart()[f + 1]; ++t)
        {
            const Vector3 ab = positions[t * 3 + 1] - positions[t * 3];
            const Vector3 ac = positions[t * 3 + 2] - positions[t * 3];
            signedArea += 0.5 * double(ab.CrossProduct(ac).DotProduct(normal));
        }
    }
    // Each triangle of a face with an area has the winding of its face.
    CHECK(CountInvertedTriangles(*result, true) == 0);
    CHECK(signedArea == Catch::Approx(32.0).margin(1e-3));
    CHECK(mesh.area_ >= 32.0 - 1e-3);
    CHECK(mesh.volume_ == Catch::Approx(12.0).margin(1e-3));
    CHECK(IsMeshClosed(mesh));
    CHECK(model->boundingBox_.min_.x_ == Catch::Approx(-1.0f));
    CHECK(model->boundingBox_.max_.x_ == Catch::Approx(2.0f));

    // One geometry for each shared key, in ascending order.
    REQUIRE(model->geometries_.size() == 2);
    CHECK(model->geometries_[0].shared_ == 1);
    CHECK(model->geometries_[1].shared_ == 2);
}

TEST_CASE("Triangulate a solid shares the vertices of the corners", "[triangulate][solid]")
{
    const SolidPtr box = MakeBoxSolid(Vector3::ZERO, 7);
    const SharedPtr<CsgTriangulatedModel> model = CsgTriangulateSolid(*box);
    REQUIRE(model);
    REQUIRE(model->geometries_.size() == 1);
    // The corners of the faces have no normals, so the faces share the 8 corners of the box.
    CHECK(model->geometries_[0].vertices_.size() == 8);
    CHECK(model->geometries_[0].indexData_.size() == 36);
    CHECK(ReadModel(*model).area_ == Catch::Approx(24.0).margin(1e-4));
}

// The union of boxes that touch in a plane keeps no zero-width region. Such a region has triangles of both windings.
TEST_CASE("Triangulated union of boxes has the winding of the face on all triangles", "[triangulate][boolean]")
{
    const SolidPtr a = MakeBoxSolid(Vector3::ZERO, 1);
    const SolidPtr b = MakeBoxSolid(Vector3(1, 0, 0), 2);
    const SolidPtr result = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON);
    REQUIRE(result->FaceCount() > 0);
    CHECK(CountInvertedTriangles(*result, false) == 0);

    // No face is a ribbon: each polygon has an area.
    for (unsigned f = 0; f < result->FaceCount(); ++f)
    {
        const CsgPolygon& polygon = *result->GetFace(f);
        Vector3 sum = Vector3::ZERO;
        for (size_t i = 0; i < polygon.vertices_.size(); ++i)
            sum += polygon.vertices_[i].GetPosition().CrossProduct(
                polygon.vertices_[(i + 1) % polygon.vertices_.size()].GetPosition());
        CAPTURE(f);
        CHECK(double(sum.Length()) * 0.5 >= 1e-5);
    }

    // The area of all triangles, without sign, is the surface of the prism.
    const SharedPtr<CsgTriangulatedModel> model = CsgTriangulateSolid(*result);
    REQUIRE(model);
    const TriangleMesh mesh = ReadModel(*model);
    CHECK(mesh.area_ == Catch::Approx(32.0).margin(1e-3));
    CHECK(mesh.volume_ == Catch::Approx(12.0).margin(1e-3));
    CHECK(IsMeshClosed(mesh));
}

// The test for a zero-width region must not depend on the distance from the origin.
// The boxes are rotated, so the strips are not aligned with an axis and the cross products do not cancel.
// The grid step is 2^-10 units in float mode and 2^-16 units in double mode.
// The snapped corners do not stay coplanar. The result can have a ribbon face, an inverted triangle or an area
// that is 0.01 off. The test fails in both modes (float: 5 checks, double: 26 checks), so it keeps the [!mayfail] tag.
TEST_CASE("known limit: triangulated union of boxes far from the origin has no ribbon",
    "[triangulate][boolean][!mayfail]")
{
    const Vector3 offsets[] = {Vector3(100.3f, 0, 0), Vector3(0, -323.7f, 0), Vector3(0, 0, 291.9f),
        Vector3(277.1f, 188.2f, -199.9f)};
    const Quaternion rotations[] = {Quaternion(), Quaternion(37.0f, Vector3::RIGHT), Quaternion(12.5f, Vector3::UP)};
    for (const Vector3& offset : offsets)
    {
        for (const Quaternion& rotation : rotations)
        {
            CAPTURE(offset.x_, offset.y_, offset.z_, rotation.Angle());
            const Matrix3x4 transform(offset, rotation, 1.0f);
            const SolidPtr a(CsgSolid::FromOperand(MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1, transform)));
            const SolidPtr b(CsgSolid::FromOperand(MakeBoxPolygons(Vector3(1, 0, 0), Vector3::ONE, 2, transform)));
            const SolidPtr result = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON);
            REQUIRE(result->FaceCount() > 0);
            CHECK(CountInvertedTriangles(*result, false) == 0);

            // No face is a ribbon. The area of a polygon is computed in double, relative to its first vertex.
            for (unsigned f = 0; f < result->FaceCount(); ++f)
            {
                const CsgPolygon& polygon = *result->GetFace(f);
                const size_t count = polygon.vertices_.size();
                const Vector3 origin = polygon.vertices_[0].GetPosition();
                double normal[3] = {};
                for (size_t i = 0; i < count; ++i)
                {
                    const Vector3 p = polygon.vertices_[i].GetPosition() - origin;
                    const Vector3 q = polygon.vertices_[(i + 1) % count].GetPosition() - origin;
                    normal[0] += double(p.y_) * q.z_ - double(p.z_) * q.y_;
                    normal[1] += double(p.z_) * q.x_ - double(p.x_) * q.z_;
                    normal[2] += double(p.x_) * q.y_ - double(p.y_) * q.x_;
                }
                CAPTURE(f);
                CHECK(0.5 * std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]) >= 1e-5);
            }

            // The area and the volume are computed in double, relative to the offset.
            double area = 0.0;
            double volume = 0.0;
            const auto positions = result->GetTrianglePositions();
            for (unsigned f = 0; f < result->FaceCount(); ++f)
            {
                for (unsigned tri = result->GetTriangleStart()[f]; tri < result->GetTriangleStart()[f + 1]; ++tri)
                {
                    const Vector3 p = positions[tri * 3] - offset;
                    const Vector3 q = positions[tri * 3 + 1] - offset;
                    const Vector3 r = positions[tri * 3 + 2] - offset;
                    area += 0.5 * double((q - p).CrossProduct(r - p).Length());
                    volume += double(p.DotProduct(q.CrossProduct(r))) / 6.0;
                }
            }
            CHECK(area == Catch::Approx(32.0).margin(1e-2));
            CHECK(volume == Catch::Approx(12.0).margin(1e-2));
        }
    }
}

TEST_CASE("Triangulate random polygons with spikes and duplicate vertices", "[triangulate]")
{
    struct Shape
    {
        std::vector<Point> boundary_;
        Point target_;
    };
    // Both shapes are counter-clockwise, and every boundary point sees the target.
    const Shape shapes[] = {
        {{{1, 0}, {3, 0}, {4, 1}, {4, 3}, {3, 4}, {1, 4}, {0, 3}, {0, 1}}, {2, 2}},
        {{{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}}, {1, 1}},
    };
    const float fractions[] = {0.25f, 0.5f, 0.75f};

    unsigned state = 12345u;
    unsigned failures = 0;
    for (unsigned iteration = 0; iteration < 600; ++iteration)
    {
        const Shape& shape = shapes[iteration % 2];
        std::vector<Point> points;
        const auto addPoint = [&](const Point& point)
        {
            points.push_back(point);
            if (NextRandom(state) % 4 == 0)
                points.push_back(point);
        };
        const auto addSpike = [&](const Point& from)
        {
            const float t = fractions[NextRandom(state) % 3];
            const Point tip(from.first + t * (shape.target_.first - from.first),
                from.second + t * (shape.target_.second - from.second));
            addPoint(tip);
            addPoint(from);
        };
        for (size_t i = 0; i < shape.boundary_.size(); ++i)
        {
            const Point& p = shape.boundary_[i];
            const Point& q = shape.boundary_[(i + 1) % shape.boundary_.size()];
            addPoint(p);
            if (NextRandom(state) % 3 == 0)
                addSpike(p);
            if (NextRandom(state) % 3 == 0)
            {
                const Point middle((p.first + q.first) * 0.5f, (p.second + q.second) * 0.5f);
                addPoint(middle);
                if (NextRandom(state) % 2 == 0)
                    addSpike(middle);
            }
        }

        for (const bool reversed : {false, true})
        {
            std::vector<Point> ordered = points;
            if (reversed)
                std::reverse(ordered.begin(), ordered.end());
            const CsgPolygon polygon = MakePolygon2D(ordered, reversed);
            const double expected = PolygonArea(ordered);
            ea::vector<unsigned> indices;
            bool clean = false;
            CsgTriangulationScratch scratch;
            REQUIRE(CsgTriangulatePolygon(polygon, indices, {}, clean, scratch));
            REQUIRE(IndicesValid(polygon, indices));
            double unsignedArea = 0.0;
            bool inverted = false;
            for (unsigned t = 0; t + 2 < indices.size(); t += 3)
            {
                const double area = SignedArea(polygon, indices[t], indices[t + 1], indices[t + 2]);
                unsignedArea += std::abs(area);
                inverted = inverted || area * expected < -1e-9;
            }
            const bool ok = clean && !inverted && std::abs(unsignedArea - std::abs(expected)) < 1e-6;
            if (!ok)
            {
                ++failures;
                if (failures <= 3)
                {
                    std::string text;
                    for (const Point& point : ordered)
                        text += "(" + std::to_string(point.first) + "," + std::to_string(point.second) + ") ";
                    UNSCOPED_INFO("iteration " << iteration << " clean " << clean << " inverted " << inverted
                        << " area " << unsignedArea << " expected " << expected << " polygon " << text);
                }
            }
        }
    }
    CHECK(failures == 0);
}

namespace
{

/// Make the model of a box with the corners shared. The model has one geometry with 8 vertices and 12 triangles.
SharedPtr<CsgTriangulatedModel> MakeCheckedBox()
{
    const SolidPtr box = MakeBoxSolid(Vector3::ZERO, 7);
    SharedPtr<CsgTriangulatedModel> model = CsgTriangulateSolid(*box);
    REQUIRE(model);
    REQUIRE(model->geometries_.size() == 1);
    REQUIRE(model->geometries_[0].indexData_.size() == 36);
    return model;
}

} // namespace

TEST_CASE("Check model: a box is valid", "[triangulate][check]")
{
    const SharedPtr<CsgTriangulatedModel> model = MakeCheckedBox();
    const CsgModelCheck check = CsgCheckModel(*model);
    CHECK(check.openEdges_ == 0);
    CHECK(check.duplicates_ == 0);
    CHECK(check.degenerates_ == 0);
    // The box has the size 2, so the volume is 8.
    CHECK(check.volume_ == Catch::Approx(8.0).margin(1e-4));

    CHECK(CsgCheckModel(CsgTriangulatedModel()).volume_ == 0.0);
}

TEST_CASE("Check model: a box with a flipped winding has a negative volume", "[triangulate][check]")
{
    const SharedPtr<CsgTriangulatedModel> model = MakeCheckedBox();
    ea::vector<unsigned>& indices = model->geometries_[0].indexData_;
    for (unsigned i = 0; i < indices.size(); i += 3)
        ea::swap(indices[i + 1], indices[i + 2]);
    const CsgModelCheck check = CsgCheckModel(*model);
    // The surface is still closed: each edge has its opposite edge.
    CHECK(check.openEdges_ == 0);
    CHECK(check.volume_ == Catch::Approx(-8.0).margin(1e-4));
}

TEST_CASE("Check model: a missing triangle opens the edges", "[triangulate][check]")
{
    const SharedPtr<CsgTriangulatedModel> model = MakeCheckedBox();
    ea::vector<unsigned>& indices = model->geometries_[0].indexData_;
    indices.resize(indices.size() - 3);
    const CsgModelCheck check = CsgCheckModel(*model);
    // The missing triangle has 3 edges. Their opposite edges stay without a partner.
    CHECK(check.openEdges_ == 3);
    CHECK(check.duplicates_ == 0);
    CHECK(check.degenerates_ == 0);
}

TEST_CASE("Check model: a repeated triangle is a duplicate", "[triangulate][check]")
{
    const SharedPtr<CsgTriangulatedModel> model = MakeCheckedBox();
    ea::vector<unsigned>& indices = model->geometries_[0].indexData_;
    const unsigned a = indices[0], b = indices[1], c = indices[2];
    SECTION("same winding")
    {
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }
    SECTION("rotated corners")
    {
        indices.push_back(b);
        indices.push_back(c);
        indices.push_back(a);
    }
    const CsgModelCheck check = CsgCheckModel(*model);
    CHECK(check.duplicates_ == 1);
    CHECK(check.degenerates_ == 0);
    // The repeated edges have one opposite edge for two edges.
    CHECK(check.openEdges_ == 3);
}

TEST_CASE("Check model: a triangle that has no area is degenerate", "[triangulate][check]")
{
    const SharedPtr<CsgTriangulatedModel> model = MakeCheckedBox();
    CsgTriangulatedModel::Geometry& geometry = model->geometries_[0];
    const unsigned a = geometry.indexData_[0], b = geometry.indexData_[1];

    SECTION("an index twice")
    {
        geometry.indexData_.push_back(a);
        geometry.indexData_.push_back(a);
        geometry.indexData_.push_back(b);
    }
    SECTION("two corners on one grid point")
    {
        const ModelVertex vertex = geometry.vertices_[a];
        // A new vertex at the same position: the corners share one grid point.
        geometry.vertices_.push_back(vertex);
        geometry.indexData_.push_back(a);
        geometry.indexData_.push_back(geometry.vertices_.size() - 1);
        geometry.indexData_.push_back(b);
    }
    SECTION("three corners on one line")
    {
        ModelVertex middle = geometry.vertices_[a];
        middle.SetPosition(Vector3::ZERO);
        ModelVertex end = middle;
        end.SetPosition(Vector3(0.5f, 0.0f, 0.0f));
        ModelVertex start = middle;
        start.SetPosition(Vector3(-0.5f, 0.0f, 0.0f));
        const unsigned first = geometry.vertices_.size();
        geometry.vertices_.push_back(start);
        geometry.vertices_.push_back(middle);
        geometry.vertices_.push_back(end);
        geometry.indexData_.push_back(first);
        geometry.indexData_.push_back(first + 1);
        geometry.indexData_.push_back(first + 2);
    }
    SECTION("a position that is not finite")
    {
        ModelVertex vertex = geometry.vertices_[a];
        vertex.SetPosition(Vector3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f));
        geometry.vertices_.push_back(vertex);
        geometry.indexData_.push_back(a);
        geometry.indexData_.push_back(b);
        geometry.indexData_.push_back(geometry.vertices_.size() - 1);
    }
    const CsgModelCheck check = CsgCheckModel(*model);
    CHECK(check.degenerates_ == 1);
    CHECK(check.duplicates_ == 0);
}

namespace
{

/// Make a model with one triangle.
SharedPtr<CsgTriangulatedModel> MakeTriangleModel(const Vector3& a, const Vector3& b, const Vector3& c)
{
    auto model = MakeShared<CsgTriangulatedModel>();
    CsgTriangulatedModel::Geometry geometry;
    for (const Vector3& position : {a, b, c})
    {
        ModelVertex vertex{};
        vertex.SetPosition(position);
        geometry.vertices_.push_back(vertex);
    }
    geometry.indexData_ = {0, 1, 2};
    model->geometries_.push_back(ea::move(geometry));
    return model;
}

} // namespace

TEST_CASE("Check model: a thin triangle is degenerate only below the grid step of 2^-24", "[triangulate][check]")
{
    const Vector3 a(0.0f, 0.0f, 0.0f);
    const Vector3 b(1.0f, 0.0f, 0.0f);

    // The height is 2^-20: 16 grid steps.
    CHECK(CsgCheckModel(*MakeTriangleModel(a, b, Vector3(0.0f, std::ldexp(1.0f, -20), 0.0f))).degenerates_ == 0);
    // The corners are on one line.
    CHECK(CsgCheckModel(*MakeTriangleModel(a, b, Vector3(0.5f, 0.0f, 0.0f))).degenerates_ == 1);
    // The height is 2^-26: the corner rounds to the grid point of the first corner.
    CHECK(CsgCheckModel(*MakeTriangleModel(a, b, Vector3(0.0f, std::ldexp(1.0f, -26), 0.0f))).degenerates_ == 1);
    // The height is 2^-24: one grid step. The triangle has an area on the grid.
    CHECK(CsgCheckModel(*MakeTriangleModel(a, b, Vector3(0.0f, std::ldexp(1.0f, -24), 0.0f))).degenerates_ == 0);
    // The sides are 256 units long: a product on the grid is 2^64, which needs more than 64 bits.
    CHECK(CsgCheckModel(*MakeTriangleModel(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 256.0f, 0.0f),
        Vector3(0.0f, 0.0f, 256.0f))).degenerates_ == 0);
    // A tie rounds to the even grid value, as in C#. 5 * 2^-25 is the grid value 2.5, so it becomes 2.
    // The first two corners are then one grid point. Rounding up would make them differ.
    CHECK(CsgCheckModel(*MakeTriangleModel(Vector3(std::ldexp(1.0f, -23), 0.0f, 0.0f),
        Vector3(5.0f * std::ldexp(1.0f, -25), 0.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f))).degenerates_ == 1);
    // A far position is counted as degenerate, as a position that is not finite.
    CHECK(CsgCheckModel(*MakeTriangleModel(a, b, Vector3(0.0f, 1e9f, 0.0f))).degenerates_ == 1);
}
