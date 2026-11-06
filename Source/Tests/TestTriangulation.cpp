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

TEST_CASE("Triangulate a ring with a collinear middle vertex without a flat triangle", "[triangulate]")
{
    // The vertex 2 lies on the edge from vertex 3 to vertex 1, close to vertex 1. The ear at vertex 0 does not
    // block on vertex 2, but it would leave the three collinear vertices as the last triangle.
    // The triangulator must clip another ear first.
    const CsgPolygon polygon = MakePolygon2D({{1, 1}, {0, 0}, {0.0005f, 0}, {2, 0}});
    ea::vector<unsigned> indices;
    bool clean = false;
    CsgTriangulationScratch scratch;
    REQUIRE(CsgTriangulatePolygon(polygon, indices, {}, clean, scratch));
    CHECK(clean);
    REQUIRE(indices.size() == 6);
    CHECK(IndicesValid(polygon, indices));
    CHECK(TriangulatedArea(polygon, indices) == Catch::Approx(1.0).margin(1e-9));
    for (unsigned t = 0; t < 6; t += 3)
    {
        CHECK(std::abs(SignedArea(polygon, indices[t], indices[t + 1], indices[t + 2])) > 1e-9);
        CHECK((indices[t] == 2 || indices[t + 1] == 2 || indices[t + 2] == 2));
    }
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
    // The vertex (1.9995, 0) is within an end of the diagonal. An ear at the apex would have the whole run as its
    // edge and leave a flat last triangle. The triangulator avoids that ear, with the mark or without it.
    // Then every triangle uses the vertex.
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
    REQUIRE(plain.size() == 6);
    for (unsigned t = 0; t < 6; t += 3)
        CHECK(std::abs(SignedArea(polygon, plain[t], plain[t + 1], plain[t + 2])) > 1e-9);
    CHECK((plain[0] == 2 || plain[1] == 2 || plain[2] == 2));
    CHECK((plain[3] == 2 || plain[4] == 2 || plain[5] == 2));
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
// The output positions are float. A float step at a distance of 256 to 512 units is 2^-15 units.
// In double mode this step is larger than the grid step, so the rounding to float can break the coplanarity.
// The tolerances for the area and the volume are the larger of the grid step and the float step times the
// number of triangles. A face with a zero area after the rounding to float is a legitimate result.
// A face with an area below the area of the smallest grid triangle is a ribbon, unless the rounding explains it.
namespace
{

/// Union of two rotated boxes at some distances from the origin. Check the result.
/// A single inverted triangle is accepted at `limitOffset`, if the float step is larger than the grid step.
void CheckUnionFarFromOrigin(const Quaternion& rotation, const Vector3* limitOffset = nullptr)
{
    const Vector3 offsets[] = {Vector3(100.3f, 0, 0), Vector3(0, -323.7f, 0), Vector3(0, 0, 291.9f),
        Vector3(277.1f, 188.2f, -199.9f)};
    const double gridStep = 1.0 / CsgGridScale;
    for (const Vector3& offset : offsets)
    {
        {
            CAPTURE(offset.x_, offset.y_, offset.z_, rotation.Angle());
            const Matrix3x4 transform(offset, rotation, 1.0f);
            const SolidPtr a(CsgSolid::FromOperand(MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1, transform)));
            const SolidPtr b(CsgSolid::FromOperand(MakeBoxPolygons(Vector3(1, 0, 0), Vector3::ONE, 2, transform)));
            const SolidPtr result = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON);
            REQUIRE(result->FaceCount() > 0);

            // The float step at the largest coordinate. The rounding to float moves a position by half of it.
            float largest = 0.0f;
            const auto positions = result->GetTrianglePositions();
            for (const Vector3& position : positions)
            {
                largest = ea::max(largest,
                    ea::max(std::abs(position.x_), ea::max(std::abs(position.y_), std::abs(position.z_))));
            }
            const double floatStep = double(std::nextafter(largest, std::numeric_limits<float>::infinity())) - largest;
            const double tolerance = ea::max(gridStep, floatStep);
            // The rounding to float adds an error to an area only when the float step is larger than the grid step.
            const double roundingError = floatStep > gridStep ? floatStep : 0.0;

            // No triangle is inverted, except at the one proven limit of the float output.
            // There the rounding to float turns a convex corner of less than one float step into a reflex corner.
            // Then no triangulation of the float polygon has only proper triangles.
            if (roundingError == 0.0 || !limitOffset || !(offset == *limitOffset))
                CHECK(CountInvertedTriangles(*result, false) == 0);
            else
            {
                // Measured in the double build: 1 inverted triangle with an area of -1.4e-6 at this offset.
                // The bound is about twice the measured area. A spike with an area of 1.5e-5 fails it.
                unsigned inverted = 0;
                for (unsigned f = 0; f < result->FaceCount(); ++f)
                {
                    const Vector3 normal = result->GetFace(f)->plane_.normal_;
                    for (unsigned tri = result->GetTriangleStart()[f]; tri < result->GetTriangleStart()[f + 1]; ++tri)
                    {
                        const Vector3& p = positions[tri * 3];
                        const Vector3& q = positions[tri * 3 + 1];
                        const Vector3& r = positions[tri * 3 + 2];
                        const double signedArea = 0.5 * double((q - p).CrossProduct(r - p).DotProduct(normal));
                        CAPTURE(f, tri, signedArea);
                        CHECK(signedArea >= -3e-6);
                        // The same threshold as CountInvertedTriangles.
                        inverted += signedArea < -0.5e-6 ? 1 : 0;
                    }
                }
                CHECK(inverted <= 1);
            }

            // The smallest triangle on the grid has half of a grid cell as area.
            const double minimumArea = 0.25 * gridStep * gridStep;

            // No face is a ribbon. The area of a polygon is computed in double, relative to its first vertex.
            for (unsigned f = 0; f < result->FaceCount(); ++f)
            {
                const CsgPolygon& polygon = *result->GetFace(f);
                const size_t count = polygon.vertices_.size();
                const Vector3 origin = polygon.vertices_[0].GetPosition();
                double normal[3] = {};
                double perimeter = 0.0;
                for (size_t i = 0; i < count; ++i)
                {
                    const Vector3 p = polygon.vertices_[i].GetPosition() - origin;
                    const Vector3 q = polygon.vertices_[(i + 1) % count].GetPosition() - origin;
                    normal[0] += double(p.y_) * q.z_ - double(p.z_) * q.y_;
                    normal[1] += double(p.z_) * q.x_ - double(p.x_) * q.z_;
                    normal[2] += double(p.x_) * q.y_ - double(p.y_) * q.x_;
                    perimeter += double((q - p).Length());
                }
                const double faceArea = 0.5 * std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
                CAPTURE(f, faceArea, perimeter);
                CHECK((faceArea >= minimumArea || faceArea <= roundingError * perimeter));
            }

            // The area and the volume are computed in double, relative to the offset.
            double area = 0.0;
            double volume = 0.0;
            unsigned triangles = 0;
            for (unsigned f = 0; f < result->FaceCount(); ++f)
            {
                for (unsigned tri = result->GetTriangleStart()[f]; tri < result->GetTriangleStart()[f + 1]; ++tri)
                {
                    const Vector3 p = positions[tri * 3] - offset;
                    const Vector3 q = positions[tri * 3 + 1] - offset;
                    const Vector3 r = positions[tri * 3 + 2] - offset;
                    area += 0.5 * double((q - p).CrossProduct(r - p).Length());
                    volume += double(p.DotProduct(q.CrossProduct(r))) / 6.0;
                    ++triangles;
                }
            }
            // A position moves by the tolerance. An edge has a length of at most 2 units, so each triangle adds
            // at most twice the tolerance to the area and to the volume.
            const double margin = 2.0 * tolerance * triangles;
            CAPTURE(margin, triangles);
            CHECK(area == Catch::Approx(32.0).margin(margin));
            CHECK(volume == Catch::Approx(12.0).margin(margin));
        }
    }
}

}

TEST_CASE("Triangulated union of boxes far from the origin has no ribbon", "[triangulate][boolean]")
{
    CheckUnionFarFromOrigin(Quaternion());
    CheckUnionFarFromOrigin(Quaternion(37.0f, Vector3::RIGHT));
    // The double build keeps one tiny inverted triangle at this offset, a limit of the float output.
    const Vector3 limitOffset(0, 0, 291.9f);
    CheckUnionFarFromOrigin(Quaternion(12.5f, Vector3::UP), &limitOffset);
}

namespace
{

/// Area of a triangle of a 3D polygon along the plane normal, computed in double relative to the first vertex.
double NormalArea(const CsgPolygon& polygon, const std::vector<unsigned>& corners)
{
    const Vector3 origin = polygon.vertices_[corners[0]].GetPosition();
    double normal[3] = {};
    for (size_t i = 0; i < corners.size(); ++i)
    {
        const Vector3 p = polygon.vertices_[corners[i]].GetPosition() - origin;
        const Vector3 q = polygon.vertices_[corners[(i + 1) % corners.size()]].GetPosition() - origin;
        normal[0] += double(p.y_) * q.z_ - double(p.z_) * q.y_;
        normal[1] += double(p.z_) * q.x_ - double(p.x_) * q.z_;
        normal[2] += double(p.x_) * q.y_ - double(p.y_) * q.x_;
    }
    const Vector3 n = polygon.plane_.normal_;
    return 0.5 * (normal[0] * n.x_ + normal[1] * n.y_ + normal[2] * n.z_);
}

/// Triangulate a polygon with float positions far from the origin. Check that no triangle is inverted and that the
/// triangles cover the area of the polygon.
void CheckFarPolygon(const std::vector<Vector3>& positions, const Vector3& normal)
{
    const CsgPolygon polygon = MakePolygon3D(positions, normal);
    std::vector<unsigned> all(positions.size());
    for (unsigned i = 0; i < all.size(); ++i)
        all[i] = i;
    const double expected = NormalArea(polygon, all);
    REQUIRE(expected > 0.0);

    ea::vector<unsigned> indices;
    REQUIRE(CsgTriangulatePolygon(polygon, indices));
    REQUIRE(IndicesValid(polygon, indices));
    double signedSum = 0.0;
    double absoluteSum = 0.0;
    for (unsigned t = 0; t < indices.size(); t += 3)
    {
        const double area = NormalArea(polygon, {indices[t], indices[t + 1], indices[t + 2]});
        CAPTURE(t, area);
        // The smallest triangle on the grid has a quarter of a grid cell or more as area.
        CHECK(area >= -1e-9);
        signedSum += area;
        absoluteSum += std::abs(area);
    }
    CHECK(signedSum == Catch::Approx(expected).margin(1e-9));
    CHECK(absoluteSum == Catch::Approx(expected).margin(1e-9));
}

}

// Reduced from a face of the union of two rotated boxes at the offset (100.3, 0, 0). The positions are on the
// float grid at 100 units. Two vertices differ by one grid step in z. They form a needle with a vertex inside it.
// A clip of the needle past the inside vertex inverts a later triangle, so the first pass has one inverted triangle.
// The triangulator then makes the result again without needle clips that pass an inside vertex.
TEST_CASE("Triangulate a needle with a vertex inside it without an inverted triangle", "[triangulate]")
{
    CheckFarPolygon({Vector3(0x1.982508p+6f, -1.0f, -0x1.68cp+0f), Vector3(0x1.99e04cp+6f, -1.0f, 0x1.163ap-1f),
                        Vector3(0x1.9210d8p+6f, -1.0f, 0x1.f3dep-1f), Vector3(0x1.9210d8p+6f, -1.0f, 0x1.f3dcp-1f),
                        Vector3(0x1.95f89p+6f, -1.0f, 0x1.850cp-1f), Vector3(0x1.951afp+6f, -1.0f, -0x1.bb4p-3f)},
        Vector3(0.0f, -1.0f, 0.0f));
}

// Reduced from a face of the same union at the offset (0, -323.7, 0). The polygon is a strip with a width of one
// grid step along its flat top edge, and a spike to the origin. No ear is left in the ring. The most convex vertex
// has a vertex inside its triangle. The triangulator clips the tip of the spike first.
TEST_CASE("Triangulate a spike on a one-step strip without an inverted triangle", "[triangulate]")
{
    CheckFarPolygon({Vector3(0x1.bb48p-3f, -0x1.44b334p+8f, 0x1.f3dep-1f),
                        Vector3(0x1.bb48p-3f, -0x1.42b334p+8f, 0x1.f3dep-1f), Vector3(0.0f, -0x1.42b334p+8f, 0.0f),
                        Vector3(0x1.bb48p-3f, -0x1.42b334p+8f, 0x1.f3dcp-1f),
                        Vector3(0x1.bb48p-3f, -0x1.43b334p+8f, 0x1.f3dcp-1f),
                        Vector3(0x1.bb48p-3f, -0x1.44b334p+8f, 0x1.f3dcp-1f)},
        Vector3(-0x1.f3dcf2p-1f, 0.0f, 0x1.bb4712p-3f));
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

namespace
{

/// Add the triangle a, b, c as one polygon. The winding gives the normal.
void AddTriangle(ea::vector<CsgPolygon>& polygons, const Vector3& a, const Vector3& b, const Vector3& c)
{
    const Vector3 normal = (b - a).CrossProduct(c - a).Normalized();
    polygons.push_back(MakePolygon3D({a, b, c}, normal));
}

/// A box with the size `size` at the origin. Each side is a grid of cells with two triangles each.
/// The cells are the same as in BrushShapes::GridBox of BiteSizedWar.
ea::vector<CsgPolygon> MakeGridBoxPolygons(const Vector3& size, const int (&counts)[3])
{
    ea::vector<CsgPolygon> polygons;
    for (int axis = 0; axis < 3; ++axis)
    {
        for (const float sign : {-1.0f, 1.0f})
        {
            const int u = (axis + 1) % 3;
            const int v = (axis + 2) % 3;
            const auto at = [&](int i, int j)
            {
                float values[3] = {};
                values[axis] = sign * 0.5f;
                values[u] = -0.5f + static_cast<float>(i) / static_cast<float>(counts[u]);
                values[v] = -0.5f + static_cast<float>(j) / static_cast<float>(counts[v]);
                return Vector3(values[0] * size.x_, values[1] * size.y_, values[2] * size.z_);
            };
            for (int j = 0; j < counts[v]; ++j)
            {
                for (int i = 0; i < counts[u]; ++i)
                {
                    const Vector3 p00 = at(i, j), p10 = at(i + 1, j), p01 = at(i, j + 1), p11 = at(i + 1, j + 1);
                    if (sign > 0.0f)
                    {
                        AddTriangle(polygons, p00, p10, p11);
                        AddTriangle(polygons, p00, p11, p01);
                    }
                    else
                    {
                        AddTriangle(polygons, p00, p11, p10);
                        AddTriangle(polygons, p00, p01, p11);
                    }
                }
            }
        }
    }
    return polygons;
}

/// A closed prism along Z with a regular cross-section of `sides` corners on a circle of `radius`.
/// The first corner is at the angle 0, so it lies at center + (radius, 0).
ea::vector<CsgPolygon> MakePrismPolygons(const Vector3& center, float radius, float halfLength, int sides)
{
    std::vector<Vector3> bottom, top;
    for (int i = 0; i < sides; ++i)
    {
        const double angle = 2.0 * M_PI * i / sides;
        const Vector3 offset(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)), 0.0f);
        bottom.push_back(center + offset - Vector3(0.0f, 0.0f, halfLength));
        top.push_back(center + offset + Vector3(0.0f, 0.0f, halfLength));
    }
    ea::vector<CsgPolygon> polygons;
    for (int i = 1; i + 1 < sides; ++i)
    {
        AddTriangle(polygons, top[0], top[i], top[i + 1]);
        AddTriangle(polygons, bottom[0], bottom[i + 1], bottom[i]);
    }
    for (int i = 0; i < sides; ++i)
    {
        const int n = (i + 1) % sides;
        AddTriangle(polygons, bottom[i], bottom[n], top[n]);
        AddTriangle(polygons, bottom[i], top[n], top[i]);
    }
    return polygons;
}

/// Return true if the point (x, y) is strictly inside the projection of the triangle onto the XY plane.
bool InsideTriangleXY(const Vector3& a, const Vector3& b, const Vector3& c, double x, double y)
{
    const auto side = [&](const Vector3& p, const Vector3& q)
    { return (double(q.x_) - p.x_) * (y - p.y_) - (double(q.y_) - p.y_) * (x - p.x_); };
    const double d0 = side(a, b), d1 = side(b, c), d2 = side(c, a);
    return (d0 > 0.0 && d1 > 0.0 && d2 > 0.0) || (d0 < 0.0 && d1 < 0.0 && d2 < 0.0);
}

} // namespace

TEST_CASE("Triangulated face with a hole that touches a cell edge at one corner leaves the hole open",
    "[triangulate][boolean]")
{
    // A slab of 12.5 x 5 x 1 with 10 x 4 x 1 cells. The cell lines on X are at -6.25 + 1.25 k.
    // An octagon hole of radius 0.2 at x = -5.2 has its first corner at x = -5, exactly on a cell edge.
    // The hole touches the outer loop of its cell triangle at this one corner.
    const int counts[3] = {10, 4, 1};
    const SolidPtr slab(CsgSolid::FromOperand(MakeGridBoxPolygons(Vector3(12.5f, 5.0f, 1.0f), counts)));
    const float radius = 0.2f;
    const double octagonArea = 2.0 * std::sqrt(2.0) * radius * radius;
    for (const float y : {-2.0f, 0.5f})
    {
        INFO("y = " << y);
        const Vector3 center(-5.2f, y, 0.0f);
        const SolidPtr hole(CsgSolid::FromOperand(MakePrismPolygons(center, radius, 1.0f, 8)));
        const SolidPtr result = Boolean(slab, hole, CsgOperation::DifferenceAB, CSG_DEFAULT_EPSILON);
        REQUIRE(result->FaceCount() > 0);

        const SharedPtr<CsgTriangulatedModel> model = CsgTriangulateSolid(*result);
        REQUIRE(model);
        const TriangleMesh mesh = ReadModel(*model);
        CHECK(IsMeshClosed(mesh));
        CHECK(mesh.volume_ == Catch::Approx(62.5 - octagonArea).margin(1e-3));

        // The front face (z = 0.5) and the back face (z = -0.5) each cover the slab face less the octagon.
        // A ray along Z through the octagon center hits no triangle.
        double frontArea = 0.0;
        double backArea = 0.0;
        unsigned hits = 0;
        for (size_t t = 0; t + 2 < mesh.positions_.size(); t += 3)
        {
            const Vector3& a = mesh.positions_[t];
            const Vector3& b = mesh.positions_[t + 1];
            const Vector3& c = mesh.positions_[t + 2];
            if (InsideTriangleXY(a, b, c, center.x_, center.y_))
                ++hits;
            const double area = 0.5 * double((b - a).CrossProduct(c - a).Length());
            if (a.z_ == 0.5f && b.z_ == 0.5f && c.z_ == 0.5f)
                frontArea += area;
            else if (a.z_ == -0.5f && b.z_ == -0.5f && c.z_ == -0.5f)
                backArea += area;
        }
        CHECK(hits == 0);
        CHECK(frontArea == Catch::Approx(62.5 - octagonArea).margin(1e-4));
        CHECK(backArea == Catch::Approx(62.5 - octagonArea).margin(1e-4));
    }
}
