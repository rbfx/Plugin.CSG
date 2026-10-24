// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgOptimization.h"

#include <catch2/catch_amalgamated.hpp>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace Urho3D;

namespace
{

constexpr float Epsilon = 1e-4f;

using Point = std::pair<float, float>;

/// Make a polygon in the plane z = 0 with the normal +Z. The points must have counter-clockwise winding.
CsgPolygon MakePolygon(const std::vector<Point>& points)
{
    CsgPolygon polygon;
    for (const Point& point : points)
    {
        ModelVertex vertex;
        vertex.SetPosition(Vector3(point.first, point.second, 0.0f));
        polygon.vertices_.push_back(vertex);
    }
    polygon.plane_ = Plane(Vector3(0.0f, 0.0f, 1.0f), Vector3::ZERO);
    return polygon;
}

CsgPolygon MakeQuad(float x0, float y0, float x1, float y1)
{
    return MakePolygon({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
}

double PolygonArea(const CsgPolygon& polygon)
{
    double area = 0.0;
    const size_t count = polygon.vertices_.size();
    for (size_t i = 0; i < count; ++i)
    {
        const Vector3 a = polygon.vertices_[i].GetPosition();
        const Vector3 b = polygon.vertices_[(i + 1) % count].GetPosition();
        area += static_cast<double>(a.x_) * b.y_ - static_cast<double>(b.x_) * a.y_;
    }
    return 0.5 * area;
}

double TotalArea(const ea::vector<CsgPolygon>& polygons)
{
    double area = 0.0;
    for (const CsgPolygon& polygon : polygons)
        area += PolygonArea(polygon);
    return area;
}

bool IsIdentical(const ea::vector<CsgPolygon>& lhs, const ea::vector<CsgPolygon>& rhs)
{
    if (lhs.size() != rhs.size())
        return false;
    for (size_t i = 0; i < lhs.size(); ++i)
    {
        if (lhs[i].vertices_.size() != rhs[i].vertices_.size())
            return false;
        for (size_t j = 0; j < lhs[i].vertices_.size(); ++j)
        {
            if (lhs[i].vertices_[j].GetPosition() != rhs[i].vertices_[j].GetPosition())
                return false;
        }
    }
    return true;
}

/// Make a grid of cell quads. The order of the cells is a fixed permutation, so the input is not in row order.
ea::vector<CsgPolygon> MakeGrid(unsigned size, unsigned stride)
{
    ea::vector<CsgPolygon> cells;
    const unsigned count = size * size;
    for (unsigned i = 0; i < count; ++i)
    {
        const unsigned index = (i * stride) % count;
        const float x = static_cast<float>(index % size);
        const float y = static_cast<float>(index / size);
        cells.push_back(MakeQuad(x, y, x + 1.0f, y + 1.0f));
    }
    return cells;
}

} // namespace

TEST_CASE("Optimization merges two adjacent coplanar quads", "[optimize]")
{
    ea::vector<CsgPolygon> polygons{MakeQuad(0, 0, 1, 1), MakeQuad(1, 0, 2, 1)};

    CHECK(CsgSimplifyPolygons(polygons, Epsilon));
    REQUIRE(polygons.size() == 1);
    CHECK(PolygonArea(polygons[0]) == Catch::Approx(2.0).margin(1e-5));
}

TEST_CASE("Optimization resolves a T-junction vertex on an edge", "[optimize]")
{
    // The bottom quad has the long top edge. The vertex (1, 1) of the two upper quads lies on this edge.
    ea::vector<CsgPolygon> polygons{MakeQuad(0, 0, 2, 1), MakeQuad(0, 1, 1, 2), MakeQuad(1, 1, 2, 2)};

    CHECK(CsgSimplifyPolygons(polygons, Epsilon));
    CHECK(TotalArea(polygons) == Catch::Approx(4.0).margin(1e-5));
    REQUIRE(polygons.size() == 1);
    CHECK(polygons[0].vertices_.size() >= 4);
}

TEST_CASE("Optimization is deterministic", "[optimize]")
{
    ea::vector<CsgPolygon> first = MakeGrid(4, 7);
    ea::vector<CsgPolygon> second = MakeGrid(4, 7);

    CsgSimplifyPolygons(first, Epsilon);
    CsgSimplifyPolygons(second, Epsilon);

    CHECK(TotalArea(first) == Catch::Approx(16.0).margin(1e-4));
    CHECK(IsIdentical(first, second));
}

TEST_CASE("Optimization output follows the input order, not the hash order", "[optimize]")
{
    // Two different input orders of the same cells. Each order gives the same result in each run.
    for (const unsigned stride : {3u, 5u, 7u})
    {
        ea::vector<CsgPolygon> reference = MakeGrid(5, stride);
        CsgSimplifyPolygons(reference, Epsilon);
        CHECK(TotalArea(reference) == Catch::Approx(25.0).margin(1e-4));

        for (unsigned run = 0; run < 3; ++run)
        {
            ea::vector<CsgPolygon> repeated = MakeGrid(5, stride);
            CsgSimplifyPolygons(repeated, Epsilon);
            CHECK(IsIdentical(reference, repeated));
        }
    }
}

TEST_CASE("Optimization handles a NaN vertex", "[optimize]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    ea::vector<CsgPolygon> polygons{MakeQuad(0, 0, 1, 1), MakeQuad(1, 0, 2, 1), MakePolygon({{2, 0}, {3, 0}, {nan, 1}})};

    // The call must return. The result is not checked.
    CsgSimplifyPolygons(polygons, Epsilon);
    SUCCEED();
}
