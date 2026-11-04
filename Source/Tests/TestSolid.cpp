// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "Urho3D/CSG/CsgTriangulation.h"

#include "TestGeometry.h"

#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <cmath>
#include <limits>
#include <thread>

using namespace Urho3D;

namespace
{

CsgPolygon MakeTestPolygon(ea::span<const Vector3> positions)
{
    ea::vector<ModelVertex> vertices;
    for (const Vector3& position : positions)
    {
        ModelVertex vertex;
        vertex.SetPosition(position);
        vertices.push_back(vertex);
    }
    return CsgPolygon(ea::move(vertices), 1);
}

void Append(ea::vector<CsgPolygon>& target, const ea::vector<CsgPolygon>& source)
{
    target.insert(target.end(), source.begin(), source.end());
}

} // namespace

TEST_CASE("Solid from a box", "[solid]")
{
    const auto polygons = MakeBoxPolygons(Vector3(1.0f, 2.0f, 3.0f), Vector3(0.5f, 1.0f, 1.5f), 1);
    const CsgSolid solid(polygons);

    CHECK(solid.FaceCount() == 6);
    CHECK(solid.ComponentCount() == 1);
    CHECK(solid.GetBoundsMin() == Vector3(0.5f, 1.0f, 1.5f));
    CHECK(solid.GetBoundsMax() == Vector3(1.5f, 3.0f, 4.5f));
    CHECK(solid.GetComponentMin(0) == solid.GetBoundsMin());
    CHECK(solid.GetComponentMax(0) == solid.GetBoundsMax());

    // Each quad gives two triangles.
    REQUIRE(solid.GetTriangleStart().size() == 7);
    CHECK(solid.GetTriangleStart()[6] == 12);
    CHECK(solid.GetTrianglePositions().size() == 36);
    CHECK(solid.GetTriangleVertices().size() == 36);
    for (unsigned face = 0; face < 6; ++face)
    {
        CHECK(solid.GetTriangleStart()[face + 1] - solid.GetTriangleStart()[face] == 2);
        CHECK(solid.GetFaceMin()[face].x_ >= 0.5f);
        CHECK(solid.GetFaceMax()[face].z_ <= 4.5f);
        CHECK(solid.GetFaceComponents()[face] == 0);
    }
    CHECK(solid.GetComponentStart().size() == 2);
    CHECK(solid.GetComponentFaces().size() == 6);
}

TEST_CASE("Solid volume of a unit cube is one", "[solid]")
{
    const CsgSolid cube(MakeBoxPolygons(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f), 1));
    REQUIRE(cube.IsClosed());
    CHECK(cube.Volume() == Catch::Approx(1.0).margin(1e-9));

    // The volume does not depend on the position.
    const CsgSolid moved(MakeBoxPolygons(Vector3(10.0f, -3.0f, 7.0f), Vector3(0.5f, 0.5f, 0.5f), 1));
    CHECK(moved.Volume() == Catch::Approx(1.0).margin(1e-6));

    const CsgSolid box(MakeBoxPolygons(Vector3(1.0f, 2.0f, 3.0f), Vector3(0.5f, 1.0f, 1.5f), 1));
    CHECK(box.Volume() == Catch::Approx(6.0).margin(1e-6));

    const CsgSolid nothing(ea::span<const CsgPolygon>{});
    CHECK(nothing.Volume() == 0.0);
}

TEST_CASE("Solid triangles match the face positions", "[solid]")
{
    const auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    const CsgSolid solid(polygons);
    for (unsigned face = 0; face < solid.FaceCount(); ++face)
    {
        const auto& vertices = solid.GetFace(face)->vertices_;
        for (unsigned i = solid.GetTriangleStart()[face] * 3; i < solid.GetTriangleStart()[face + 1] * 3; ++i)
        {
            const unsigned local = solid.GetTriangleVertices()[i];
            REQUIRE(local < vertices.size());
            CHECK(solid.GetTrianglePositions()[i] == vertices[local].GetPosition());
        }
    }
}

TEST_CASE("Solid of two disjoint boxes has two components", "[solid]")
{
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    Append(polygons, MakeBoxPolygons(Vector3(10.0f, 0.0f, 0.0f), Vector3::ONE, 1));
    const CsgSolid solid(polygons);

    CHECK(solid.FaceCount() == 12);
    REQUIRE(solid.ComponentCount() == 2);
    // The components follow the order of their first face.
    for (unsigned face = 0; face < 12; ++face)
        CHECK(solid.GetFaceComponents()[face] == face / 6);
    CHECK(solid.GetComponentStart()[0] == 0);
    CHECK(solid.GetComponentStart()[1] == 6);
    CHECK(solid.GetComponentStart()[2] == 12);
    CHECK(solid.GetComponentMin(0) == Vector3(-1.0f, -1.0f, -1.0f));
    CHECK(solid.GetComponentMax(0) == Vector3(1.0f, 1.0f, 1.0f));
    CHECK(solid.GetComponentMin(1) == Vector3(9.0f, -1.0f, -1.0f));
    CHECK(solid.GetComponentMax(1) == Vector3(11.0f, 1.0f, 1.0f));
    CHECK(solid.GetBoundsMin() == Vector3(-1.0f, -1.0f, -1.0f));
    CHECK(solid.GetBoundsMax() == Vector3(11.0f, 1.0f, 1.0f));
}

TEST_CASE("Component order follows the first face, not the face order of the inputs", "[solid]")
{
    // Faces of the two boxes alternate. Component 0 holds face 0.
    const auto first = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    const auto second = MakeBoxPolygons(Vector3(10.0f, 0.0f, 0.0f), Vector3::ONE, 1);
    ea::vector<CsgPolygon> polygons;
    for (unsigned i = 0; i < 6; ++i)
    {
        polygons.push_back(second[i]);
        polygons.push_back(first[i]);
    }
    const CsgSolid solid(polygons);
    REQUIRE(solid.ComponentCount() == 2);
    for (unsigned face = 0; face < 12; ++face)
        CHECK(solid.GetFaceComponents()[face] == face % 2);
    // Faces of a component stay in the order of the faces.
    for (unsigned c = 0; c < 6; ++c)
    {
        CHECK(solid.GetComponentFaces()[c] == c * 2);
        CHECK(solid.GetComponentFaces()[6 + c] == c * 2 + 1);
    }
}

TEST_CASE("Solid of two boxes that share a face has one component", "[solid]")
{
    // Edges that two boxes share have four faces. The faces of both boxes are joined.
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    Append(polygons, MakeBoxPolygons(Vector3(2.0f, 0.0f, 0.0f), Vector3::ONE, 1));
    const CsgSolid solid(polygons);
    CHECK(solid.FaceCount() == 12);
    CHECK(solid.ComponentCount() == 1);
}

TEST_CASE("Solid of two boxes that touch in the middle of a face has two components", "[solid]")
{
    // The small face lies in the large face. No edge is shared.
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    Append(polygons, MakeBoxPolygons(Vector3(1.5f, 0.0f, 0.0f), Vector3(0.5f, 0.25f, 0.25f), 1));
    const CsgSolid solid(polygons);
    CHECK(solid.ComponentCount() == 2);
}

TEST_CASE("Solid welds noisy corners within epsilon", "[solid]")
{
    // The noise is far below epsilon, but the corners are not bit-identical.
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    unsigned counter = 0;
    for (CsgPolygon& polygon : polygons)
    {
        for (ModelVertex& vertex : polygon.vertices_)
        {
            const float noise = 1e-6f * static_cast<float>((counter++ % 7) + 1);
            vertex.SetPosition(vertex.GetPosition() + Vector3(noise, -noise, noise));
        }
    }
    const CsgSolid solid(polygons, 1e-4f);
    CHECK(solid.FaceCount() == 6);
    CHECK(solid.ComponentCount() == 1);

    // The noise is below half a grid step in both modes, so the one snap of the operand puts each noisy corner
    // on the grid point of its box corner. The faces stay together also without a welding tolerance.
    const CsgSolid tight(polygons, 1e-8f);
    CHECK(tight.ComponentCount() == 1);
}

TEST_CASE("Solid joins faces through a T-junction", "[solid]")
{
    // A long edge of the first face meets two short edges of two other faces.
    // The two short faces share only one vertex.
    const Vector3 longFace[] = {Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(2, 1, 0), Vector3(0, 1, 0)};
    const Vector3 leftFace[] = {Vector3(1, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 1)};
    const Vector3 rightFace[] = {Vector3(2, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 1)};
    ea::vector<CsgPolygon> polygons;
    polygons.push_back(MakeTestPolygon(longFace));
    polygons.push_back(MakeTestPolygon(leftFace));
    polygons.push_back(MakeTestPolygon(rightFace));

    const CsgSolid solid(polygons, 1e-4f);
    CHECK(solid.FaceCount() == 3);
    CHECK(solid.ComponentCount() == 1);
}

TEST_CASE("Solid of a sphere has one component", "[solid]")
{
    const auto polygons = MakeSpherePolygons(Vector3(1.0f, 2.0f, 3.0f), 2.0f, 8, 12, 1);
    const CsgSolid solid(polygons);
    CHECK(solid.FaceCount() == 8 * 12);
    CHECK(solid.ComponentCount() == 1);
    CHECK(solid.GetBoundsMin().x_ >= -1.0f - 1e-4f);
    CHECK(solid.GetBoundsMax().y_ <= 4.0f + 1e-4f);
    CHECK(solid.GetBoundsMax().y_ >= 4.0f - 1e-4f);
    // Poles and rings are triangles and quads: 2 * 12 triangles, 6 * 12 * 2 triangles.
    CHECK(solid.GetTriangleStart()[8 * 12] == 2 * 12 + 6 * 12 * 2);
}

TEST_CASE("Solid of nothing is empty", "[solid]")
{
    const CsgSolid solid(ea::span<const CsgPolygon>{});
    CHECK(solid.FaceCount() == 0);
    CHECK(solid.ComponentCount() == 0);
    CHECK(solid.ToPolygons().empty());
}

TEST_CASE("Solid ToPolygons returns deep copies", "[solid]")
{
    const auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 5);
    const CsgSolid solid(polygons);

    auto copy = solid.ToPolygons();
    REQUIRE(copy.size() == 6);
    const Vector3 original = solid.GetFace(0)->vertices_[0].GetPosition();
    CHECK(copy[0].vertices_[0].GetPosition() == original);
    CHECK(copy[0].shared_ == 5);
    CHECK(solid.GetFace(0).get() != &copy[0]);

    copy[0].vertices_[0].SetPosition(Vector3(100.0f, 100.0f, 100.0f));
    copy[0].vertices_.pop_back();
    copy[1].Flip();
    CHECK(solid.GetFace(0)->vertices_[0].GetPosition() == original);
    CHECK(solid.GetFace(0)->vertices_.size() == 4);

    // A second copy is not affected by the first.
    const auto again = solid.ToPolygons();
    CHECK(again[0].vertices_[0].GetPosition() == original);
    CHECK(again[1].plane_.normal_ == polygons[1].plane_.normal_);

    // The solid does not use the input polygons after construction.
    ea::vector<CsgPolygon> input = polygons;
    const CsgSolid second(input);
    input[0].vertices_[0].SetPosition(Vector3(50.0f, 50.0f, 50.0f));
    CHECK(second.GetFace(0)->vertices_[0].GetPosition() == original);
}

TEST_CASE("Point welder merges points within epsilon", "[solid][welder]")
{
    const float eps = 0.01f;
    CsgPointWelder welder(eps, 16);

    CHECK(welder.Add(Vector3(1.0f, 2.0f, 3.0f)) == 0);
    CHECK(welder.Add(Vector3(1.005f, 2.0f, 3.0f)) == 0);
    CHECK(welder.Add(Vector3(1.0f, 2.004f, 2.996f)) == 0);
    // Distinct points stay apart.
    CHECK(welder.Add(Vector3(1.0f, 2.0f, 3.1f)) == 1);
    CHECK(welder.Add(Vector3(1.0f, 2.0f, 3.1f)) == 1);
    CHECK(welder.Add(Vector3(-5.0f, 0.0f, 0.0f)) == 2);
    CHECK(welder.Count() == 3);
    CHECK(welder.GetPosition(0) == Vector3(1.0f, 2.0f, 3.0f));
    CHECK(welder.GetPositions().size() == 3);

    CHECK(welder.Find(Vector3(1.0f, 2.0f, 3.009f)) == 0);
    CHECK(welder.Find(Vector3(1.0f, 2.0f, 3.5f)) == -1);
    CHECK(welder.Count() == 3);
}

TEST_CASE("Point welder finds neighbors across a cell border", "[solid][welder]")
{
    // The cells are four epsilons wide. The points lie on both sides of the border at 4 * eps.
    const float eps = 0.25f;
    CsgPointWelder welder(eps, 4);
    CHECK(welder.Add(Vector3(0.99f, 0.0f, 0.0f)) == 0);
    CHECK(welder.Add(Vector3(1.1f, 0.0f, 0.0f)) == 0);
    CHECK(welder.Add(Vector3(1.3f, 0.0f, 0.0f)) == 1);

    // The border is also at zero, with negative coordinates on the other side.
    CsgPointWelder negative(eps, 4);
    CHECK(negative.Add(Vector3(-0.1f, -0.1f, -0.1f)) == 0);
    CHECK(negative.Add(Vector3(0.1f, 0.1f, 0.1f)) == 1);
    CHECK(negative.Add(Vector3(0.1f, 0.1f, -0.1f)) == 1);
    CHECK(negative.Add(Vector3(-0.1f, 0.1f, 0.1f)) == 1);
    CHECK(negative.Add(Vector3(-0.1f, -0.2f, -0.1f)) == 0);
}

TEST_CASE("Point welder takes the earliest point", "[solid][welder]")
{
    const float eps = 1.0f;
    CsgPointWelder welder(eps, 4);
    CHECK(welder.Add(Vector3(0.0f, 0.0f, 0.0f)) == 0);
    CHECK(welder.Add(Vector3(1.5f, 0.0f, 0.0f)) == 1);
    // The point is within epsilon of both. The earlier one wins.
    CHECK(welder.Add(Vector3(0.75f, 0.0f, 0.0f)) == 0);
    CHECK(welder.Find(Vector3(0.75f, 0.0f, 0.0f)) == 0);
    CHECK(welder.Count() == 2);
}

TEST_CASE("Point welder with zero epsilon welds identical points only", "[solid][welder]")
{
    CsgPointWelder welder(0.0f, 4);
    CHECK(welder.Add(Vector3(1.0f, 2.0f, 3.0f)) == 0);
    CHECK(welder.Add(Vector3(1.0f, 2.0f, 3.0f)) == 0);
    CHECK(welder.Add(Vector3(1.0f, 2.0f, 3.0000002f)) == 1);
    CHECK(welder.Add(Vector3(0.0f, 0.0f, 0.0f)) == 2);
    CHECK(welder.Add(Vector3(-0.0f, -0.0f, 0.0f)) == 2);
    CHECK(welder.Find(Vector3(1.0f, 2.0f, 3.0f)) == 0);
    CHECK(welder.Find(Vector3(9.0f, 2.0f, 3.0f)) == -1);
    CHECK(welder.Count() == 3);
}

TEST_CASE("Point welder handles many points and far coordinates", "[solid][welder]")
{
    const float eps = 0.01f;
    CsgPointWelder welder(eps, 0);
    constexpr int count = 20000;
    for (int i = 0; i < count; ++i)
    {
        const Vector3 point(static_cast<float>(i) * 0.5f, static_cast<float>(i % 17), -1.0e6f + static_cast<float>(i));
        REQUIRE(welder.Add(point) == i);
    }
    for (int i = 0; i < count; ++i)
    {
        const Vector3 point(static_cast<float>(i) * 0.5f, static_cast<float>(i % 17), -1.0e6f + static_cast<float>(i));
        REQUIRE(welder.Add(point + Vector3(eps * 0.25f, 0.0f, 0.0f)) == i);
    }
    CHECK(welder.Count() == count);
    // Huge and invalid coordinates do not crash.
    CHECK(welder.Add(Vector3(1.0e30f, -1.0e30f, 0.0f)) == count);
    CHECK(welder.Add(Vector3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f)) == count + 1);
}

namespace
{

/// Make an open mesh: a box without its last face, with one corner of the first face replaced.
ea::vector<CsgPolygon> MakeOpenBoxWithCorner(const Vector3& corner)
{
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    polygons.pop_back();
    polygons[0].vertices_[0].SetPosition(corner);
    return polygons;
}

} // namespace

TEST_CASE("Solid of an open mesh with a NaN corner is stable", "[solid]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const auto polygons = MakeOpenBoxWithCorner(Vector3(nan, nan, nan));
    const CsgSolid first(polygons);
    const CsgSolid second(polygons);
    CHECK(first.FaceCount() == 5);
    CHECK(first.ComponentCount() >= 1);
    CHECK(first.ComponentCount() <= 5);
    CHECK(first.ComponentCount() == second.ComponentCount());
    for (unsigned face = 0; face < 5; ++face)
        CHECK(first.GetFaceComponents()[face] == second.GetFaceComponents()[face]);

    // Only one coordinate is NaN.
    const auto partial = MakeOpenBoxWithCorner(Vector3(0.0f, nan, 0.0f));
    const CsgSolid third(partial);
    CHECK(third.FaceCount() == 5);
    CHECK(third.ComponentCount() >= 1);
}

TEST_CASE("Solid of an open mesh with an infinite corner is stable", "[solid]")
{
    const float inf = std::numeric_limits<float>::infinity();
    for (const Vector3& corner : {Vector3(inf, inf, inf), Vector3(-inf, 0.0f, 1.0f), Vector3(0.0f, inf, -inf)})
    {
        const auto polygons = MakeOpenBoxWithCorner(corner);
        const CsgSolid first(polygons);
        const CsgSolid second(polygons);
        CHECK(first.FaceCount() == 5);
        CHECK(first.ComponentCount() >= 1);
        CHECK(first.ComponentCount() <= 5);
        CHECK(first.ComponentCount() == second.ComponentCount());
        for (unsigned face = 0; face < 5; ++face)
            CHECK(first.GetFaceComponents()[face] == second.GetFaceComponents()[face]);
    }
}

namespace
{

/// Return the box faces without the faces in the mask. The bit i of the mask removes the face i.
ea::vector<CsgPolygon> MakeBoxWithoutFaces(const Vector3& center, const Vector3& halfSize, unsigned mask)
{
    const auto box = MakeBoxPolygons(center, halfSize, 1);
    ea::vector<CsgPolygon> result;
    for (unsigned i = 0; i < box.size(); ++i)
    {
        if ((mask & (1u << i)) == 0)
            result.push_back(box[i]);
    }
    return result;
}

/// Make a box of half size one. The +x face is a frame around a square hole. The hole has the side holeSize.
/// Return the open edge length of the pinhole after the snap of the operand to the grid.
/// The corners of the hole are at plus and minus half the side. Each one snaps to its own grid point.
float SnappedPinholeEdgeLength(float holeSize)
{
    const float h = holeSize * 0.5f;
    const double side = (CsgGridPoint::Round(h) - CsgGridPoint::Round(-h)) / CsgGridScale;
    return static_cast<float>(4.0 * side);
}

ea::vector<CsgPolygon> MakeBoxWithPinhole(float holeSize)
{
    ea::vector<CsgPolygon> result = MakeBoxWithoutFaces(Vector3::ZERO, Vector3::ONE, 1u);
    const float h = holeSize * 0.5f;
    const Vector3 outer[4] = {Vector3(1, -1, -1), Vector3(1, 1, -1), Vector3(1, 1, 1), Vector3(1, -1, 1)};
    const Vector3 inner[4] = {Vector3(1, -h, -h), Vector3(1, h, -h), Vector3(1, h, h), Vector3(1, -h, h)};
    for (unsigned i = 0; i < 4; ++i)
    {
        const unsigned j = (i + 1) % 4;
        const Vector3 trapezoid[4] = {outer[i], outer[j], inner[j], inner[i]};
        result.push_back(MakeTestPolygon(trapezoid));
    }
    return result;
}

/// Make a box of which the +x face is two quads. The split points are on the edges of the two neighbor faces.
ea::vector<CsgPolygon> MakeBoxWithSplitFace()
{
    ea::vector<CsgPolygon> result = MakeBoxWithoutFaces(Vector3::ZERO, Vector3::ONE, 1u);
    const Vector3 lower[4] = {Vector3(1, -1, -1), Vector3(1, 1, -1), Vector3(1, 1, 0), Vector3(1, -1, 0)};
    const Vector3 upper[4] = {Vector3(1, -1, 0), Vector3(1, 1, 0), Vector3(1, 1, 1), Vector3(1, -1, 1)};
    result.push_back(MakeTestPolygon(lower));
    result.push_back(MakeTestPolygon(upper));
    return result;
}

/// Make an L shape of three unit cubes. All walls between the cubes are left out.
ea::vector<CsgPolygon> MakeLShape()
{
    // Cube A1 touches A2 at +x and touches B at +y. The faces are +x, +y, +z, -x, -y, -z.
    ea::vector<CsgPolygon> result = MakeBoxWithoutFaces(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f), 0b000011);
    Append(result, MakeBoxWithoutFaces(Vector3(1.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f), 0b001000));
    Append(result, MakeBoxWithoutFaces(Vector3(0.5f, 1.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f), 0b010000));
    return result;
}

/// Return the distance from the box surface with a sign. The value is negative inside the box.
float BoxDistance(const Vector3& point, const Vector3& center, const Vector3& halfSize)
{
    const Vector3 d = point - center;
    return ea::max(ea::max(Abs(d.x_) - halfSize.x_, Abs(d.y_) - halfSize.y_), Abs(d.z_) - halfSize.z_);
}

} // namespace

TEST_CASE("Solid of a box is closed", "[solid]")
{
    const CsgSolid solid(MakeBoxPolygons(Vector3(1.0f, 2.0f, 3.0f), Vector3(0.5f, 1.0f, 1.5f), 1));
    CHECK(solid.IsClosed());
    CHECK_FALSE(solid.HasOpenEdges());
    CHECK(solid.OpenEdgeLength() == 0.0f);
}

TEST_CASE("Solid of a sphere is closed", "[solid]")
{
    const CsgSolid solid(MakeSpherePolygons(Vector3(1.0f, 2.0f, 3.0f), 2.0f, 8, 12, 1));
    CHECK(solid.IsClosed());
    CHECK_FALSE(solid.HasOpenEdges());
}

TEST_CASE("Solid of nothing is closed and contains nothing", "[solid]")
{
    const CsgSolid solid(ea::span<const CsgPolygon>{});
    CHECK(solid.IsClosed());
    CHECK_FALSE(solid.HasOpenEdges());
    CHECK_FALSE(solid.ContainsPoint(Vector3::ZERO));
}

TEST_CASE("Solid of a box without a face is open", "[solid]")
{
    const CsgSolid solid(MakeBoxWithoutFaces(Vector3::ZERO, Vector3::ONE, 1u << 2));
    CHECK_FALSE(solid.IsClosed());
    CHECK(solid.HasOpenEdges());
    // The hole has four edges of length two.
    CHECK(solid.OpenEdgeLength() == Catch::Approx(8.0f).margin(1e-3f));
}

TEST_CASE("Solid of a box with a face of two quads is closed", "[solid]")
{
    // The split points lie on the long edges of the two neighbor faces. These are T-junctions.
    const auto polygons = MakeBoxWithSplitFace();
    REQUIRE(polygons.size() == 7);
    CHECK_FALSE(IsEdgeClosed(polygons, 1e-4f));

    const CsgSolid solid(polygons);
    CHECK(solid.FaceCount() == 7);
    CHECK(solid.ComponentCount() == 1);
    CHECK(solid.IsClosed());
    CHECK_FALSE(solid.HasOpenEdges());
    CHECK(solid.ContainsPoint(Vector3(0.2f, 0.3f, 0.4f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(1.5f, 0.3f, 0.4f)));
}

TEST_CASE("Solid crack budget", "[solid]")
{
    // A hole of the side 0.004 has the edge length 0.016. That is below 200 epsilons, which is 0.02.
    const CsgSolid small(MakeBoxWithPinhole(0.004f), 1e-4f);
    CHECK(small.IsClosed());
    CHECK(small.HasOpenEdges());
    // The operand snaps to the grid once, so the open edge length is a multiple of the grid step.
    CHECK(small.OpenEdgeLength() == Catch::Approx(SnappedPinholeEdgeLength(0.004f)).margin(1e-5f));
    // The parity tolerates the crack.
    CHECK(small.ContainsPoint(Vector3(0.1f, 0.2f, 0.3f)));
    CHECK_FALSE(small.ContainsPoint(Vector3(2.0f, 0.2f, 0.3f)));

    // A hole of the side 0.006 has the edge length 0.024. That is above 200 epsilons (0.02).
    // The total edge length is about 29.66, so only 0.001 of it (about 0.0297) keeps the solid closed.
    const CsgSolid fraction(MakeBoxWithPinhole(0.006f), 1e-4f);
    CHECK(fraction.IsClosed());
    CHECK(fraction.HasOpenEdges());
    CHECK(fraction.OpenEdgeLength() == Catch::Approx(SnappedPinholeEdgeLength(0.006f)).margin(1e-5f));

    // A hole of the side 0.02 has the edge length 0.08. That is more than the budget.
    const CsgSolid large(MakeBoxWithPinhole(0.02f), 1e-4f);
    CHECK_FALSE(large.IsClosed());
    CHECK(large.HasOpenEdges());
    CHECK(large.OpenEdgeLength() == Catch::Approx(SnappedPinholeEdgeLength(0.02f)).margin(1e-4f));

    // A larger epsilon gives a larger budget.
    const CsgSolid wide(MakeBoxWithPinhole(0.02f), 1e-3f);
    CHECK(wide.IsClosed());
}

TEST_CASE("Solid constructor repairs the T-junctions", "[solid]")
{
    const auto polygons = MakeBoxWithSplitFace();
    const CsgSolid solid(polygons);
    CHECK(solid.IsClosed());
    CHECK_FALSE(solid.HasOpenEdges());
    CHECK(solid.FaceCount() == 7);
    // The two neighbors of the split gain one vertex each.
    unsigned vertexCount = 0;
    for (unsigned face = 0; face < solid.FaceCount(); ++face)
        vertexCount += solid.GetFace(face)->vertices_.size();
    CHECK(vertexCount == 5 * 4 + 2 * 4 + 2);
    CHECK(IsEdgeClosed(solid.ToPolygons(), 1e-6f));
    CHECK(solid.ContainsPoint(Vector3(0.9f, 0.0f, 0.1f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(1.1f, 0.0f, 0.1f)));
}

TEST_CASE("Solid of an open mesh with a NaN corner is not closed", "[solid]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (const Vector3& corner : {Vector3(nan, nan, nan), Vector3(0.0f, nan, 0.0f), Vector3(inf, inf, inf),
             Vector3(-inf, 0.0f, 1.0f)})
    {
        const auto polygons = MakeOpenBoxWithCorner(corner);
        const CsgSolid solid(polygons);
        CHECK_FALSE(solid.IsClosed());
        // The query must finish and must not read outside of the arrays.
        (void)solid.ContainsPoint(Vector3(0.5f, 0.5f, 0.5f));
        (void)solid.ContainsPoint(Vector3(5.0f, 5.0f, 5.0f));
        CHECK_FALSE(solid.ContainsPoint(Vector3(nan, 0.0f, 0.0f)));
    }
}

TEST_CASE("Solid ContainsPoint for a box", "[solid][contains]")
{
    const Vector3 center(1.0f, 2.0f, 3.0f);
    const Vector3 halfSize(0.5f, 1.0f, 1.5f);
    const CsgSolid solid(MakeBoxPolygons(center, halfSize, 1));

    CHECK(solid.ContainsPoint(center));
    CHECK(solid.ContainsPoint(center + Vector3(0.4f, 0.9f, -1.4f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(10.0f, 2.0f, 3.0f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(-10.0f, -10.0f, -10.0f)));
    CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.0f, 2.0f, 0.0f)));

    // Near a face, on each side of it. The offsets are more than one grid step, so the snap keeps the side.
    for (const float offset : {0.01f, 0.003f})
    {
        CHECK(solid.ContainsPoint(center + Vector3(0.5f - offset, 0.0f, 0.0f)));
        CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.5f + offset, 0.0f, 0.0f)));
        CHECK(solid.ContainsPoint(center + Vector3(0.1f, -1.0f + offset, 0.2f)));
        CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.1f, -1.0f - offset, 0.2f)));
        CHECK(solid.ContainsPoint(center + Vector3(0.1f, 0.2f, 1.5f - offset)));
        CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.1f, 0.2f, 1.5f + offset)));
    }

    // On a face, on an edge and on a corner the symbolic perturbation decides. The answer is deterministic.
    const Vector3 surface[] = {center + Vector3(0.5f, 0.0f, 0.0f), center + Vector3(0.5f, 1.0f, 0.0f),
        center + Vector3(0.5f, 1.0f, 1.5f), center + Vector3(-0.5f, -1.0f, -1.5f), center + Vector3(0.0f, 1.0f, 1.5f)};
    for (const Vector3& point : surface)
    {
        const bool first = solid.ContainsPoint(point);
        CHECK(solid.ContainsPoint(point) == first);
        CHECK(CsgInternal::CsgClassifyGridPointForTest(solid, CsgGridPoint::From(point), 0b0111) == first);
    }
    // The triangles move along +x, so the ray from a point on the face at +x crosses that face only.
    CHECK(solid.ContainsPoint(center + Vector3(0.5f, 0.0f, 0.0f)));
    CHECK_FALSE(solid.ContainsPoint(center + Vector3(-0.5f, 0.0f, 0.0f)));

    // Points that are not finite or out of the range of the grid are outside.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CHECK_FALSE(solid.ContainsPoint(Vector3(nan, nan, nan)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(center.x_, nan, center.z_)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(inf, 0.0f, 0.0f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(-inf, -inf, -inf)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(1.0e6f, 0.0f, 0.0f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(0.0f, -1.0e30f, 0.0f)));
}

TEST_CASE("Solid ContainsPoint for a sphere", "[solid][contains]")
{
    const Vector3 center(1.0f, 2.0f, 3.0f);
    const CsgSolid solid(MakeSpherePolygons(center, 2.0f, 16, 24, 1));
    REQUIRE(solid.IsClosed());

    CHECK(solid.ContainsPoint(center));
    CHECK(solid.ContainsPoint(center + Vector3(1.8f, 0.0f, 0.0f)));
    CHECK(solid.ContainsPoint(center + Vector3(0.0f, -1.8f, 0.0f)));
    CHECK(solid.ContainsPoint(center + Vector3(0.0f, 0.0f, 1.8f)));
    CHECK(solid.ContainsPoint(center + Vector3(1.0f, 1.0f, 1.0f)));
    CHECK_FALSE(solid.ContainsPoint(center + Vector3(2.1f, 0.0f, 0.0f)));
    CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.0f, 2.1f, 0.0f)));
    CHECK_FALSE(solid.ContainsPoint(center + Vector3(1.3f, 1.3f, 1.3f)));
    CHECK_FALSE(solid.ContainsPoint(center + Vector3(10.0f, 0.0f, 0.0f)));
}

TEST_CASE("Solid ContainsPoint for a concave shape", "[solid][contains]")
{
    // Three unit cubes: (0..2, 0..1) and (0..1, 1..2) in the xy plane. The notch is x 1..2, y 1..2.
    const auto polygons = MakeLShape();
    REQUIRE(polygons.size() == 14);
    const CsgSolid solid(polygons);
    CHECK(solid.ComponentCount() == 1);
    CHECK(solid.IsClosed());
    CHECK(SignedVolume(polygons) == Catch::Approx(3.0).margin(1e-5));

    CHECK(solid.ContainsPoint(Vector3(0.5f, 0.5f, 0.5f)));
    CHECK(solid.ContainsPoint(Vector3(1.5f, 0.5f, 0.5f)));
    CHECK(solid.ContainsPoint(Vector3(0.5f, 1.5f, 0.5f)));
    CHECK(solid.ContainsPoint(Vector3(0.2f, 1.8f, 0.7f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(1.5f, 1.5f, 0.5f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(1.9f, 1.1f, 0.3f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(2.5f, 0.5f, 0.5f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(0.5f, 2.5f, 0.5f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(0.5f, 0.5f, 1.5f)));
}

TEST_CASE("Solid ContainsPoint for a cavity", "[solid][contains]")
{
    // The inner box has reversed faces, so it is a hole in the outer box.
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3(2.0f, 2.0f, 2.0f), 1);
    auto inner = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    for (CsgPolygon& polygon : inner)
        polygon.Flip();
    Append(polygons, inner);

    const CsgSolid solid(polygons);
    CHECK(solid.ComponentCount() == 2);
    CHECK(solid.IsClosed());

    CHECK_FALSE(solid.ContainsPoint(Vector3::ZERO));
    CHECK_FALSE(solid.ContainsPoint(Vector3(0.5f, -0.5f, 0.3f)));
    CHECK(solid.ContainsPoint(Vector3(1.5f, 0.0f, 0.0f)));
    CHECK(solid.ContainsPoint(Vector3(0.0f, -1.5f, 1.9f)));
    CHECK(solid.ContainsPoint(Vector3(-1.9f, 1.9f, -1.9f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(3.0f, 0.0f, 0.0f)));
}

TEST_CASE("Solid ContainsPoint agrees with an analytic box", "[solid][contains]")
{
    const Vector3 center(1.0f, 2.0f, 3.0f);
    const Vector3 halfSize(0.5f, 1.0f, 1.5f);
    const CsgSolid solid(MakeBoxPolygons(center, halfSize, 1));
    // Two grid steps. A point nearer to the surface can snap onto it.
    const float margin = 2.0f / static_cast<float>(CsgGridScale);

    // A fixed generator, so the points are the same on every platform.
    uint32_t state = 12345u;
    const auto next = [&]()
    {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
    };

    unsigned inside = 0;
    unsigned checked = 0;
    for (unsigned i = 0; i < 1000; ++i)
    {
        const Vector3 point(-1.0f + next() * 4.0f, -0.5f + next() * 5.0f, 0.0f + next() * 6.0f);
        const float distance = BoxDistance(point, center, halfSize);
        // A point within a few grid steps of the surface can snap onto it.
        if (Abs(distance) < margin)
            continue;
        ++checked;
        const bool expected = distance < 0.0f;
        inside += expected ? 1 : 0;
        CHECK(solid.ContainsPoint(point) == expected);
    }
    // The sample must have points on both sides.
    CHECK(checked > 900);
    CHECK(inside > 20);
    CHECK(inside < checked);
}

TEST_CASE("Solid ContainsPoint equals the exact classifier on random points", "[solid][contains]")
{
    const CsgSolid box(MakeBoxPolygons(Vector3(0.3f, -0.2f, 0.1f), Vector3(0.5f, 1.0f, 1.5f), 1));
    const CsgSolid sphere(MakeSpherePolygons(Vector3(0.1f, 0.0f, 0.2f), 1.2f, 12, 16, 2));
    const auto cutter = CsgSolid::FromOperand(MakeBoxPolygons(Vector3(0.9f, 0.0f, 0.0f), Vector3::ONE * 0.8f, 1));
    const auto base = CsgSolid::FromOperand(MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1));
    REQUIRE((cutter && base));
    const auto difference = CsgBooleanOperation(base.Get(), cutter.Get(), CsgOperation::DifferenceAB);
    REQUIRE(difference);
    REQUIRE(difference->FaceCount() > 0);

    const CsgSolid* solids[] = {&box, &sphere, difference.Get()};

    // A fixed generator, so the points are the same on every platform.
    uint32_t state = 987654321u;
    const auto next = [&]()
    {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
    };

    for (const CsgSolid* solid : solids)
    {
        // The third reference visits all triangles of the exact loops. It has no BVH and no box filter.
        const CsgExactLoops loops = solid->GetExactLoops();
        const ea::span<const unsigned> triangleStart = solid->GetTriangleStart();
        const ea::span<const unsigned> triangleVertices = solid->GetTriangleVertices();
        const auto bruteForce = [&](const CsgGridPoint& p)
        {
            bool parity = false;
            for (unsigned face = 0; face < solid->FaceCount(); ++face)
            {
                for (unsigned t = triangleStart[face]; t < triangleStart[face + 1]; ++t)
                {
                    const auto corner = [&](unsigned k) -> const CsgGridPoint&
                    { return loops.grid_[loops.ids_[loops.vertexStart_[face] + triangleVertices[t * 3 + k]]]; };
                    parity ^= CsgExact::AxisRayCrossesTriangle(p, corner(0), corner(1), corner(2), 0b0111);
                }
            }
            return parity;
        };

        unsigned inside = 0;
        constexpr unsigned count = 600;
        for (unsigned i = 0; i < count; ++i)
        {
            const Vector3 point(-2.0f + next() * 4.0f, -2.0f + next() * 4.0f, -2.0f + next() * 4.0f);
            const bool contains = solid->ContainsPoint(point);
            CHECK(contains == CsgInternal::CsgClassifyGridPointForTest(*solid, CsgGridPoint::From(point), 0b0111));
            CHECK(contains == bruteForce(CsgGridPoint::From(point)));
            inside += contains ? 1 : 0;
        }
        CHECK(inside > 10);
        CHECK(inside < count);
    }
}

TEST_CASE("Solid FromOperand keeps a clean box", "[solid]")
{
    const auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    const auto solid = CsgSolid::FromOperand(ea::vector<CsgPolygon>(polygons), CSG_DEFAULT_EPSILON);
    REQUIRE(solid);
    CHECK(solid->FaceCount() == 6);
    CHECK(solid->IsClosed());
    // The faces are the faces of the input. Nothing changed.
    for (unsigned face = 0; face < 6; ++face)
    {
        REQUIRE(solid->GetFace(face)->vertices_.size() == 4);
        for (unsigned i = 0; i < 4; ++i)
            CHECK(solid->GetFace(face)->vertices_[i].GetPosition() == polygons[face].vertices_[i].GetPosition());
    }
}

TEST_CASE("Solid FromOperand adds the T-junction vertices", "[solid]")
{
    auto polygons = MakeBoxWithSplitFace();
    REQUIRE_FALSE(IsEdgeClosed(polygons, 1e-6f));
    const auto solid = CsgSolid::FromOperand(ea::move(polygons), CSG_DEFAULT_EPSILON);
    REQUIRE(solid);
    CHECK(solid->FaceCount() == 7);
    CHECK(solid->IsClosed());
    CHECK_FALSE(solid->HasOpenEdges());

    // The two neighbors of the split gain one vertex each. The other faces keep four.
    unsigned vertexCount = 0;
    for (unsigned face = 0; face < solid->FaceCount(); ++face)
        vertexCount += solid->GetFace(face)->vertices_.size();
    CHECK(vertexCount == 5 * 4 + 2 * 4 + 2);

    const auto repaired = solid->ToPolygons();
    CHECK(IsEdgeClosed(repaired, 1e-6f));
    CHECK(SignedVolume(repaired) == Catch::Approx(8.0).margin(1e-4));
    CHECK(solid->ContainsPoint(Vector3(0.9f, 0.0f, 0.1f)));
    CHECK_FALSE(solid->ContainsPoint(Vector3(1.1f, 0.0f, 0.1f)));
}

TEST_CASE("Solid FromOperand moves noisy corners to one position", "[solid]")
{
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    unsigned counter = 0;
    for (CsgPolygon& polygon : polygons)
    {
        for (ModelVertex& vertex : polygon.vertices_)
        {
            const float noise = 1e-6f * static_cast<float>((counter++ % 7) + 1);
            vertex.SetPosition(vertex.GetPosition() + Vector3(noise, -noise, noise));
        }
    }
    REQUIRE_FALSE(IsEdgeClosed(polygons, 1e-8f));
    const auto solid = CsgSolid::FromOperand(ea::move(polygons), 1e-4f);
    REQUIRE(solid);
    CHECK(solid->FaceCount() == 6);
    CHECK(solid->IsClosed());
    CHECK(IsEdgeClosed(solid->ToPolygons(), 1e-8f));
}

namespace
{

SharedPtr<CsgSolid> MakeBoxSolid(const Vector3& center, float halfSize)
{
    return CsgSolid::FromOperand(MakeBoxPolygons(center, Vector3::ONE * halfSize, 1));
}

/// The kept faces of a boolean test: all faces of A, and all faces of B flipped. The ids are taken from the solids.
struct KeptFaces
{
    ea::vector<CsgSharedPolygon> faces_;
    ea::vector<CsgFaceOrigin> origins_;
    ea::vector<int> ids_;
    int idSpace_{};
};

KeptFaces KeepBoxes(const SharedPtr<CsgSolid>& a, const SharedPtr<CsgSolid>& b)
{
    KeptFaces kept;
    kept.idSpace_ = static_cast<int>(a->VertexKeySpace() + b->VertexKeySpace());
    for (unsigned face = 0; face < a->FaceCount(); ++face)
    {
        kept.faces_.push_back(a->GetFace(face));
        kept.origins_.push_back({SharedPtr<const CsgSolid>(a), face, false});
        const unsigned count = a->GetFace(face)->vertices_.size();
        for (unsigned k = 0; k < count; ++k)
            kept.ids_.push_back(a->VertexKey(face, k));
    }
    for (unsigned face = 0; face < b->FaceCount(); ++face)
    {
        kept.faces_.push_back(CsgSolid::Flipped(*b->GetFace(face)));
        kept.origins_.push_back({SharedPtr<const CsgSolid>(b), face, true});
        const unsigned count = b->GetFace(face)->vertices_.size();
        for (unsigned k = 0; k < count; ++k)
            kept.ids_.push_back(static_cast<int>(a->VertexKeySpace()) + b->VertexKey(face, count - 1 - k));
    }
    return kept;
}

SharedPtr<CsgSolid> BuildKept(KeptFaces& kept)
{
    return CsgSolid::FromKeptFaces(ea::move(kept.faces_), CSG_DEFAULT_EPSILON,
        {kept.origins_.data(), kept.origins_.size()}, {kept.ids_.data(), kept.ids_.size()}, kept.idSpace_);
}

Vector3 TriangleNormal(const Vector3& a, const Vector3& b, const Vector3& c)
{
    return (b - a).CrossProduct(c - a);
}

/// Check that the triangles of every face index the face vertices at the triangle positions.
void CheckTrianglesMatchFaces(const CsgSolid& solid)
{
    for (unsigned face = 0; face < solid.FaceCount(); ++face)
    {
        const auto& vertices = solid.GetFace(face)->vertices_;
        for (unsigned t = solid.GetTriangleStart()[face]; t < solid.GetTriangleStart()[face + 1]; ++t)
        {
            for (unsigned corner = 0; corner < 3; ++corner)
            {
                const unsigned local = solid.GetTriangleVertices()[t * 3 + corner];
                REQUIRE(local < vertices.size());
                CHECK(vertices[local].GetPosition() == solid.GetTrianglePositions()[t * 3 + corner]);
            }
        }
    }
}

} // namespace

TEST_CASE("Solid from kept faces shares faces and copies triangles", "[solid][kept]")
{
    const auto a = MakeBoxSolid(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    const auto b = MakeBoxSolid(Vector3(5.0f, 0.0f, 0.0f), 1.0f);
    KeptFaces kept = KeepBoxes(a, b);
    const auto solid = BuildKept(kept);

    REQUIRE(solid->FaceCount() == 12);
    CHECK(solid->ComponentCount() == 2);
    CHECK(solid->IsClosed());
    CheckTrianglesMatchFaces(*solid);

    ea::vector<unsigned> expected;
    ea::vector<unsigned> actual;
    for (unsigned face = 0; face < 6; ++face)
    {
        // An untouched face is the same object, and its triangles are the source triangles.
        CHECK(solid->GetFace(face).get() == a->GetFace(face).get());
        REQUIRE(a->CopyTriangles(face, expected));
        REQUIRE(solid->CopyTriangles(face, actual));
        CHECK(actual == expected);
        CHECK(solid->GetFaceMin()[face] == a->GetFaceMin()[face]);
        CHECK(solid->GetFaceMax()[face] == a->GetFaceMax()[face]);
    }

    for (unsigned face = 0; face < 6; ++face)
    {
        const CsgPolygon& source = *b->GetFace(face);
        const CsgPolygon& flipped = *solid->GetFace(6 + face);
        CHECK(solid->GetFace(6 + face).get() != b->GetFace(face).get());
        REQUIRE(flipped.vertices_.size() == source.vertices_.size());
        const unsigned count = source.vertices_.size();
        for (unsigned k = 0; k < count; ++k)
            CHECK(flipped.vertices_[k].GetPosition() == source.vertices_[count - 1 - k].GetPosition());
        CHECK(flipped.plane_.normal_ == -source.plane_.normal_);
        CHECK(flipped.plane_.d_ == -source.plane_.d_);
        CHECK(flipped.shared_ == source.shared_);
        // The source stays as it was.
        CHECK(b->GetFace(face)->plane_.normal_ == source.plane_.normal_);

        // Each copied triangle winds with the flipped plane and has the corners of a source triangle.
        const unsigned sourceFirst = b->GetTriangleStart()[face];
        const unsigned first = solid->GetTriangleStart()[6 + face];
        REQUIRE(solid->GetTriangleStart()[7 + face] - first == b->GetTriangleStart()[face + 1] - sourceFirst);
        for (unsigned t = 0; t < b->GetTriangleStart()[face + 1] - sourceFirst; ++t)
        {
            const Vector3* p = &solid->GetTrianglePositions()[(first + t) * 3];
            const Vector3* q = &b->GetTrianglePositions()[(sourceFirst + t) * 3];
            CHECK(p[0] == q[2]);
            CHECK(p[1] == q[1]);
            CHECK(p[2] == q[0]);
            CHECK(TriangleNormal(p[0], p[1], p[2]).DotProduct(flipped.plane_.normal_) > 0.0f);
            CHECK(TriangleNormal(q[0], q[1], q[2]).DotProduct(source.plane_.normal_) > 0.0f);
        }
    }
}

TEST_CASE("Solid vertex keys are stable and exact", "[solid][kept]")
{
    const auto a = MakeBoxSolid(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    const auto b = MakeBoxSolid(Vector3(5.0f, 0.0f, 0.0f), 1.0f);
    CHECK(a->VertexKeySpace() == 8);

    KeptFaces kept = KeepBoxes(a, b);
    const auto solid = BuildKept(kept);
    CHECK(solid->VertexKeySpace() == 16);

    // The keys of the first solid keep their numbers. Two corners have one key if and only if their positions are equal.
    for (unsigned face = 0; face < 6; ++face)
    {
        for (unsigned k = 0; k < 4; ++k)
            CHECK(solid->VertexKey(face, k) == a->VertexKey(face, k));
    }
    for (unsigned f0 = 0; f0 < solid->FaceCount(); ++f0)
    {
        for (unsigned k0 = 0; k0 < 4; ++k0)
        {
            const int key = solid->VertexKey(f0, k0);
            CHECK(key >= 0);
            CHECK(key < static_cast<int>(solid->VertexKeySpace()));
            for (unsigned f1 = 0; f1 < solid->FaceCount(); ++f1)
            {
                for (unsigned k1 = 0; k1 < 4; ++k1)
                {
                    const bool samePosition = solid->GetFace(f0)->vertices_[k0].GetPosition()
                        == solid->GetFace(f1)->vertices_[k1].GetPosition();
                    CHECK((key == solid->VertexKey(f1, k1)) == samePosition);
                }
            }
        }
    }

    // Empty ids are legal. The positions give the keys.
    KeptFaces missing = KeepBoxes(a, b);
    missing.ids_.clear();
    const auto welded = BuildKept(missing);
    CHECK(welded->VertexKeySpace() == 16);
    CHECK(welded->ComponentCount() == 2);
}

TEST_CASE("Solid exact weld merges bit-identical points only", "[solid][kept]")
{
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    // A point within epsilon of the corner, but not identical.
    const Vector3 corner = polygons[0].vertices_[0].GetPosition();
    polygons[0].vertices_[0].SetPosition(corner + Vector3(1e-6f, 0.0f, 0.0f));
    ea::vector<CsgSharedPolygon> faces;
    for (const CsgPolygon& polygon : polygons)
        faces.push_back(ea::make_shared<CsgPolygon>(polygon));
    const auto solid = CsgSolid::FromKeptFaces(ea::move(faces), CSG_DEFAULT_EPSILON, {}, {}, 0);
    CHECK(solid->VertexKeySpace() == 9);
    const CsgExactLoops loops = solid->GetExactLoops();
    CHECK(loops.positions_.size() == 9);
    CHECK(loops.vertexStart_.size() == 7);
    CHECK(loops.ids_.size() == 24);
    for (unsigned face = 0; face < 6; ++face)
        CHECK(loops.loopLength_[face] == 4);
    for (unsigned face = 0; face < 6; ++face)
    {
        for (unsigned k = 0; k < 4; ++k)
        {
            const int id = solid->VertexKey(face, k);
            CHECK(loops.ids_[loops.vertexStart_[face] + k] == id);
            CHECK(loops.positions_[id] == solid->GetFace(face)->vertices_[k].GetPosition());
        }
    }
}

TEST_CASE("Solid builder copies components and faces", "[solid][kept]")
{
    const auto a = MakeBoxSolid(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    const auto b = MakeBoxSolid(Vector3(5.0f, 0.0f, 0.0f), 2.0f);

    SECTION("Components")
    {
        CsgSolid::Builder builder(12, 72);
        builder.AppendComponent(*a, 0, false);
        builder.AppendComponent(*b, 0, true);
        const auto solid = builder.Finish();
        REQUIRE(solid->FaceCount() == 12);
        CHECK(solid->ComponentCount() == 2);
        CHECK(solid->IsClosed());
        CHECK(solid->VertexKeySpace() == 16);
        CheckTrianglesMatchFaces(*solid);
        for (unsigned face = 0; face < 6; ++face)
        {
            CHECK(solid->GetFace(face).get() == a->GetFace(face).get());
            CHECK(solid->GetFace(6 + face).get() != b->GetFace(face).get());
            CHECK(solid->GetFace(6 + face)->plane_.normal_ == -b->GetFace(face)->plane_.normal_);
            CHECK(solid->GetFaceComponents()[face] == 0);
            CHECK(solid->GetFaceComponents()[6 + face] == 1);
        }
        CHECK(solid->GetComponentMin(1) == Vector3(3.0f, -2.0f, -2.0f));
        CHECK(solid->GetComponentMax(1) == Vector3(7.0f, 2.0f, 2.0f));
        CHECK(solid->GetBoundsMin() == Vector3(-1.0f, -2.0f, -2.0f));
        CHECK(solid->ContainsPoint(Vector3(0.0f, 0.0f, 0.0f)));
        CHECK_FALSE(solid->ContainsPoint(Vector3(2.5f, 0.0f, 0.0f)));
        // The exact ids of the sources do not meet: one solid is the first, and the other starts after it.
        for (unsigned face = 0; face < 6; ++face)
        {
            for (unsigned k = 0; k < 4; ++k)
            {
                CHECK(solid->VertexKey(face, k) == a->VertexKey(face, k));
                CHECK(solid->VertexKey(6 + face, k) >= 8);
            }
        }
    }

    SECTION("Solids")
    {
        CsgSolid::Builder builder(12, 72);
        builder.AppendSolid(*a);
        builder.AppendSolid(*b);
        const auto solid = builder.Finish();
        REQUIRE(solid->FaceCount() == 12);
        CHECK(solid->ComponentCount() == 2);
        CHECK(solid->IsClosed());
        CHECK(solid->VertexKeySpace() == 16);
        CheckTrianglesMatchFaces(*solid);
        for (unsigned face = 0; face < 6; ++face)
        {
            CHECK(solid->GetFace(face).get() == a->GetFace(face).get());
            CHECK(solid->GetFace(6 + face).get() == b->GetFace(face).get());
        }
        CHECK(solid->ContainsPoint(Vector3(5.0f, 1.0f, 1.0f)));
        CHECK_FALSE(solid->ContainsPoint(Vector3(2.5f, 0.0f, 0.0f)));
    }

    SECTION("An open source makes an open result")
    {
        auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
        polygons.pop_back();
        const auto open = CsgSolid::FromOperand(ea::move(polygons));
        REQUIRE_FALSE(open->IsClosed());
        CsgSolid::Builder builder(12, 72);
        builder.AppendSolid(*a);
        builder.AppendSolid(*open);
        const auto solid = builder.Finish();
        CHECK_FALSE(solid->IsClosed());
        CHECK(solid->FaceCount() == 11);
    }
}

TEST_CASE("Solid Keep follows the operation table", "[solid][kept]")
{
    for (const bool inside : {false, true})
    {
        CHECK(CsgSolid::Keep(inside, CsgOperation::Union, false) == !inside);
        CHECK(CsgSolid::Keep(inside, CsgOperation::Union, true) == !inside);
        CHECK(CsgSolid::Keep(inside, CsgOperation::Intersection, false) == inside);
        CHECK(CsgSolid::Keep(inside, CsgOperation::Intersection, true) == inside);
        CHECK(CsgSolid::Keep(inside, CsgOperation::DifferenceAB, false) == !inside);
        CHECK(CsgSolid::Keep(inside, CsgOperation::DifferenceAB, true) == inside);
    }
}

TEST_CASE("Solid copies no triangles of a face without triangles", "[solid][kept]")
{
    // A face without triangles gives no triangles.
    ModelVertex vertex;
    vertex.SetPosition(Vector3(4.0f, 5.0f, 6.0f));
    ea::vector<ModelVertex> vertices{vertex, vertex};
    ea::vector<CsgSharedPolygon> faces{ea::make_shared<CsgPolygon>(ea::move(vertices), 1)};
    const auto degenerate = CsgSolid::FromKeptFaces(ea::move(faces), CSG_DEFAULT_EPSILON, {}, {}, 0);
    ea::vector<unsigned> triangles{1, 2, 3};
    CHECK_FALSE(degenerate->CopyTriangles(0, triangles));
    CHECK(triangles.empty());
}

TEST_CASE("Solid from kept faces survives non-finite positions", "[solid][kept]")
{
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 1);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    polygons[1].vertices_[2].SetPosition(Vector3(nan, 0.0f, 0.0f));
    polygons[2].vertices_[0].SetPosition(Vector3(std::numeric_limits<float>::infinity(), 0.0f, 0.0f));
    ea::vector<CsgSharedPolygon> faces;
    ea::vector<int> ids;
    int next = 0;
    for (const CsgPolygon& polygon : polygons)
    {
        faces.push_back(ea::make_shared<CsgPolygon>(polygon));
        for (unsigned k = 0; k < polygon.vertices_.size(); ++k)
            ids.push_back(next++);
    }
    const auto solid = CsgSolid::FromKeptFaces(
        ea::move(faces), CSG_DEFAULT_EPSILON, {}, {ids.data(), ids.size()}, next);
    CHECK(solid->FaceCount() == 6);
    // The query must finish on a solid with non-finite corners.
    (void)solid->ContainsPoint(Vector3(0.1f, 0.1f, 0.1f));
    CHECK_FALSE(solid->IsClosed());
}

TEST_CASE("Solid exact ids count ids and not positions", "[solid][kept]")
{
    // The boxes touch at the plane x = 1. Four corner positions are equal.
    const auto a = MakeBoxSolid(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    const auto b = MakeBoxSolid(Vector3(2.0f, 0.0f, 0.0f), 1.0f);
    REQUIRE(a->VertexKeySpace() == 8);
    REQUIRE(b->VertexKeySpace() == 8);

    SECTION("Builder")
    {
        CsgSolid::Builder builder(12, 72);
        builder.AppendComponent(*a, 0, false);
        builder.AppendComponent(*b, 0, false);
        const auto solid = builder.Finish();
        REQUIRE(solid->FaceCount() == 12);
        // Sixteen exact ids, but only twelve distinct grid points. The builder has the grid points of its sources,
        // and corners with one grid point weld to one id. So the solid has twelve ids.
        CHECK(solid->VertexKeySpace() == 12);
        CHECK(solid->GetExactLoops().positions_.size() == 12);
    }

    SECTION("Kept faces")
    {
        KeptFaces kept = KeepBoxes(a, b);
        const auto solid = BuildKept(kept);
        REQUIRE(solid->FaceCount() == 12);
        CHECK(solid->VertexKeySpace() == 16);
        CHECK(solid->IsClosed());

        // Find the corners of both boxes at one position. They have different keys.
        unsigned shared = 0;
        for (unsigned f0 = 0; f0 < 6; ++f0)
        {
            for (unsigned k0 = 0; k0 < 4; ++k0)
            {
                for (unsigned f1 = 6; f1 < 12; ++f1)
                {
                    for (unsigned k1 = 0; k1 < 4; ++k1)
                    {
                        if (solid->GetFace(f0)->vertices_[k0].GetPosition()
                            != solid->GetFace(f1)->vertices_[k1].GetPosition())
                            continue;
                        ++shared;
                        CHECK(solid->VertexKey(f0, k0) != solid->VertexKey(f1, k1));
                    }
                }
            }
        }
        CHECK(shared > 0);
    }
}

TEST_CASE("Builder region groups the kept faces like a solid of the kept faces", "[solid][kept][region]")
{
    const auto a = MakeBoxSolid(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    const auto b = MakeBoxSolid(Vector3(5.0f, 0.0f, 0.0f), 2.0f);

    // The solid of the kept faces is the reference. Its topology build gives the components and the closed flag.
    // The builder appends the same kept faces after a whole component and must give the same grouping.
    // The builder cancels the triangles of a face with no area. The faces that it drops are the last ones.
    const auto check = [&](KeptFaces& kept, bool expectClosed, unsigned expectComponents, unsigned dropped = 0)
    {
        ea::vector<CsgGridPoint> grid;
        for (const CsgSharedPolygon& face : kept.faces_)
        {
            for (const ModelVertex& vertex : face->vertices_)
                grid.push_back(CsgGridPoint::From(vertex.GetPosition()));
        }
        const auto far = MakeBoxSolid(Vector3(40.0f, 0.0f, 0.0f), 1.0f);
        const unsigned farFaces = far->FaceCount();
        ea::vector<CsgSharedPolygon> reference = kept.faces_;
        const auto expected = CsgSolid::FromKeptFaces(ea::move(reference), CSG_DEFAULT_EPSILON,
            {kept.origins_.data(), kept.origins_.size()}, {kept.ids_.data(), kept.ids_.size()}, kept.idSpace_,
            {grid.data(), grid.size()});

        CsgSolid::Builder builder(kept.faces_.size() + farFaces, 400);
        builder.AppendComponent(*far, 0, false);
        builder.AppendRegion(ea::move(kept.faces_), {kept.origins_.data(), kept.origins_.size()},
            {kept.ids_.data(), kept.ids_.size()}, kept.idSpace_, {grid.data(), grid.size()});
        const auto solid = builder.Finish();

        REQUIRE(solid->FaceCount() == expected->FaceCount() - dropped + farFaces);
        CHECK(solid->IsClosed() == expectClosed);
        CHECK(expected->IsClosed() == expectClosed);
        // The far box is component 0. The region components follow in the order of their first face.
        REQUIRE(solid->ComponentCount() == expected->ComponentCount() + 1);
        CHECK(expected->ComponentCount() == expectComponents);
        for (unsigned face = 0; face < expected->FaceCount() - dropped; ++face)
        {
            CHECK(solid->GetFaceComponents()[farFaces + face] == expected->GetFaceComponents()[face] + 1);
            const unsigned begin = expected->GetTriangleStart()[face];
            const unsigned end = expected->GetTriangleStart()[face + 1];
            const unsigned solidBegin = solid->GetTriangleStart()[farFaces + face];
            REQUIRE(solid->GetTriangleStart()[farFaces + 1 + face] - solidBegin == end - begin);
            for (unsigned i = 0; i < (end - begin) * 3; ++i)
            {
                CHECK(solid->GetTriangleVertices()[solidBegin * 3 + i] == expected->GetTriangleVertices()[begin * 3 + i]);
                CHECK(solid->GetTrianglePositions()[solidBegin * 3 + i]
                    == expected->GetTrianglePositions()[begin * 3 + i]);
            }
            CHECK(solid->GetFace(farFaces + face).get() == expected->GetFace(face).get());
        }
        for (unsigned component = 0; component < expected->ComponentCount(); ++component)
        {
            CHECK(solid->GetComponentMin(component + 1) == expected->GetComponentMin(component));
            CHECK(solid->GetComponentMax(component + 1) == expected->GetComponentMax(component));
        }
    };

    SECTION("Balanced loops use the edge match")
    {
        KeptFaces kept = KeepBoxes(a, b);
        check(kept, true, 2);
    }

    SECTION("A missing face is a crack and uses the topology build")
    {
        KeptFaces kept = KeepBoxes(a, b);
        // Drop the fifth face of the first box: a quad with four ids.
        kept.faces_.erase(kept.faces_.begin() + 4);
        kept.origins_.erase(kept.origins_.begin() + 4);
        kept.ids_.erase(kept.ids_.begin() + 16, kept.ids_.begin() + 20);
        check(kept, false, 2);
    }

    SECTION("A collapsed face joins a component and keeps the solid closed, and the builder drops it")
    {
        KeptFaces kept = KeepBoxes(a, b);
        // A triangle with three corners at one position and one id. It has a triangle but no loop.
        ea::vector<ModelVertex> corners(3, a->GetFace(0)->vertices_[0]);
        kept.faces_.push_back(ea::make_shared<CsgPolygon>(ea::move(corners), 1));
        // The faces have no origins, so the solids triangulate them.
        kept.origins_.clear();
        for (int k = 0; k < 3; ++k)
            kept.ids_.push_back(a->VertexKey(0, 0));
        check(kept, true, 2, 1);
    }
}

TEST_CASE("Solid answers queries from many threads on a fresh solid", "[solid][kept]")
{
    const auto polygons = MakeSpherePolygons(Vector3::ZERO, 1.0f, 12, 16, 1);
    ea::vector<Vector3> points;
    for (int i = 0; i < 400; ++i)
        points.push_back(Vector3(-1.3f + 0.0067f * i, 1.2f - 0.0059f * (i % 97), -1.1f + 0.0053f * (i % 211)));

    const auto query = [&points](const CsgSolid& solid, ea::vector<unsigned char>& result)
    {
        for (const Vector3& point : points)
            result.push_back(solid.ContainsPoint(point) ? 1 : 0);
    };

    // The reference comes from a second solid in one thread.
    const CsgSolid reference(polygons);
    ea::vector<unsigned char> expected;
    query(reference, expected);

    // The solid is fresh: no face tree exists when the threads start.
    const CsgSolid shared(polygons);
    ea::vector<unsigned char> results[4];
    std::thread threads[4];
    for (int i = 0; i < 4; ++i)
        threads[i] = std::thread([&, i] { query(shared, results[i]); });
    for (std::thread& thread : threads)
        thread.join();
    for (int i = 0; i < 4; ++i)
        CHECK(results[i] == expected);
}

namespace
{

/// Tell if each face corner is at the position of its id, and if that position is grid / scale in the range.
bool CornersOnGrid(const CsgSolid& solid)
{
    const CsgExactLoops loops = solid.GetExactLoops();
    for (unsigned face = 0; face < solid.FaceCount(); ++face)
    {
        const auto& vertices = solid.GetFace(face)->vertices_;
        for (unsigned i = 0; i < vertices.size(); ++i)
        {
            const int id = loops.ids_[loops.vertexStart_[face] + i];
            const Vector3& position = vertices[i].GetPosition();
            if (position != loops.positions_[id])
                return false;
            const Vector3 onGrid = loops.grid_[id].ToPosition();
            for (int axis = 0; axis < 3; ++axis)
            {
                if (std::fabs(position.Data()[axis]) < 8000.0f && position.Data()[axis] != onGrid.Data()[axis])
                    return false;
            }
        }
    }
    return true;
}

} // namespace

TEST_CASE("Solid snaps each corner, also two corners with one grid point", "[solid][grid]")
{
    // The corner (1, 1, 1) of the top face moves by 4e-4. This is more than epsilon and less than half a grid step.
    // So the two corners do not weld within epsilon, but they have one grid point.
    // The neighbor box on the face x = 1 is smaller, so its corners make T-junctions and the solid needs a repair.
    ea::vector<CsgPolygon> polygons = MakeBoxPolygons(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f), 1);
    unsigned moved = 0;
    for (CsgPolygon& polygon : polygons)
    {
        if (polygon.plane_.normal_.y_ < 0.5f)
            continue;
        for (ModelVertex& vertex : polygon.vertices_)
        {
            if (vertex.GetPosition() == Vector3::ONE)
            {
                vertex.SetPosition(Vector3(1.0004f, 1.0f, 1.0f));
                ++moved;
            }
        }
    }
    REQUIRE(moved == 1);
    Append(polygons, MakeBoxPolygons(Vector3(1.5f, 0.5f, 0.25f), Vector3(0.5f, 0.5f, 0.25f), 2));

    const CsgSolid solid(polygons);
    CHECK(CornersOnGrid(solid));
    const auto operand = CsgSolid::FromOperand(ea::vector<CsgPolygon>(polygons));
    CHECK(CornersOnGrid(*operand));
}

TEST_CASE("Solid out of the grid range does not weld corners with one clamped grid point", "[solid][grid]")
{
    // All corners have x out of the range of the grid, so their x grid coordinates are clamped to one value.
    // The rotation about the x axis keeps y and z of the two ends of an x edge equal: they get one grid point.
    const Matrix3x4 transform(Vector3::ZERO, Quaternion(3.0f, Vector3::RIGHT), 1.0f);
    const auto polygons = MakeBoxPolygons(Vector3(10001.0f, 0.0f, 0.0f), Vector3::ONE, 1, transform);
    const auto operand = CsgSolid::FromOperand(ea::vector<CsgPolygon>(polygons));
    CHECK(operand->GetExactLoops().positions_.size() == 8);
    CHECK(operand->ComponentCount() == 1);
    CHECK(operand->IsClosed());
    const CsgSolid solid(polygons);
    CHECK(solid.GetExactLoops().positions_.size() == 8);
    CHECK(solid.ComponentCount() == 1);

    // A NaN coordinate has the grid point zero. It must not weld with the corner at the origin.
    ea::vector<CsgPolygon> withNaN = MakeBoxPolygons(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f), 1);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Vector3 triangle[] = {Vector3(nan, 0.0f, 0.0f), Vector3(5.0f, 0.0f, 0.0f), Vector3(5.0f, 1.0f, 0.0f)};
    withNaN.push_back(MakeTestPolygon(triangle));
    const CsgSolid nanSolid(withNaN);
    CHECK(nanSolid.GetExactLoops().positions_.size() == 11);
    const auto nanOperand = CsgSolid::FromOperand(ea::move(withNaN));
    CHECK(nanOperand->GetExactLoops().positions_.size() == 11);
}

namespace
{

/// Return true if the triangle has three corners on one line, exactly on the grid.
bool HasZeroExactNormal(const Vector3& a, const Vector3& b, const Vector3& c)
{
    const CsgNormal normal =
        CsgExact::Normal(CsgGridPoint::From(a), CsgGridPoint::From(b), CsgGridPoint::From(c));
    return normal.x_.Sign() == 0 && normal.y_.Sign() == 0 && normal.z_.Sign() == 0;
}

/// A polygon with the vertices at the given multiples of 1/1024.
CsgPolygon MakeGridPolygon(ea::span<const IntVector3> points)
{
    ea::vector<Vector3> positions;
    for (const IntVector3& point : points)
        positions.push_back(Vector3(point.x_, point.y_, point.z_) / 1024.0f);
    return MakeTestPolygon(positions);
}

} // namespace

TEST_CASE("Kept face with a vertex on a straight run has no triangle without area", "[solid][kept][flat]")
{
    // The vertices 3, 4 and 5 are within rounding of one line. The vertices 8 and 9 are on the straight run at the
    // left edge. The ear clipper closes the short edges with flat triangles. The flat triangles turn.
    const IntVector3 points[] = {{0, 0, 0}, {1829, 0, 0}, {1829, 1210, 0}, {326, 1210, -2}, {346, 1210, -2},
        {347, 1210, -2}, {0, 1210, 0}, {0, 1072, 2}, {0, 360, 0}, {0, 361, 0}};
    const CsgPolygon polygon = MakeGridPolygon(points);

    // The triangulator alone leaves a flat triangle here, so the test covers the turn.
    ea::vector<unsigned> raw;
    REQUIRE(CsgTriangulatePolygon(polygon, raw));
    unsigned rawFlat = 0;
    for (unsigned t = 0; t + 2 < raw.size(); t += 3)
    {
        rawFlat += HasZeroExactNormal(polygon.vertices_[raw[t]].GetPosition(),
            polygon.vertices_[raw[t + 1]].GetPosition(), polygon.vertices_[raw[t + 2]].GetPosition());
    }
    CHECK(rawFlat > 0);

    ea::vector<CsgSharedPolygon> faces;
    faces.push_back(ea::make_shared<CsgPolygon>(polygon));
    const auto solid = CsgSolid::FromKeptFaces(ea::move(faces), CSG_DEFAULT_EPSILON, {}, {}, 0);
    REQUIRE(solid->FaceCount() == 1);
    const auto positions = solid->GetTrianglePositions();
    REQUIRE(solid->GetTriangleStart()[1] == positions.size() / 3);
    // A polygon of n vertices gives n - 2 triangles, flat or not.
    CHECK(positions.size() / 3 == 8);
    for (unsigned t = 0; t < positions.size() / 3; ++t)
    {
        CAPTURE(t);
        CHECK_FALSE(HasZeroExactNormal(positions[t * 3], positions[t * 3 + 1], positions[t * 3 + 2]));
    }
    // Each triangle indexes a vertex of the face.
    for (const unsigned index : solid->GetTriangleVertices())
        CHECK(index < polygon.vertices_.size());
}

TEST_CASE("Builder region cancels opposite triangles on the same points and stays closed",
    "[solid][kept][region][cancel]")
{
    const auto far = MakeBoxSolid(Vector3(40.0f, 0.0f, 0.0f), 1.0f);
    const unsigned farFaces = far->FaceCount();
    const int farIds = static_cast<int>(far->VertexKeySpace());

    const Vector3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0), d(0, 0, 1);
    const Vector3 e(5, 0, 0), f(6, 0, 0), g(5, 1, 0);
    // A tetrahedron with outward faces, ids 0 to 3.
    struct Item
    {
        ea::vector<Vector3> positions_;
        ea::vector<int> ids_;
    };
    const Item tetrahedron[] = {
        {{a, c, b}, {0, 2, 1}},
        {{a, b, d}, {0, 1, 3}},
        {{a, d, c}, {0, 3, 2}},
        {{b, c, d}, {1, 2, 3}},
    };
    // A sheet of one triangle on the points e, f, g (ids 4 to 6) and the same triangle in the opposite direction.
    const Item sheet[] = {{{e, f, g}, {4, 5, 6}}, {{g, f, e}, {6, 5, 4}}};

    ea::vector<Item> items;
    items.push_back(sheet[0]);
    for (const Item& item : tetrahedron)
        items.push_back(item);
    items.push_back(sheet[1]);

    ea::vector<CsgSharedPolygon> faces;
    ea::vector<int> ids;
    ea::vector<CsgGridPoint> grid;
    for (const Item& item : items)
    {
        faces.push_back(ea::make_shared<CsgPolygon>(MakeTestPolygon({item.positions_.data(), item.positions_.size()})));
        for (unsigned k = 0; k < item.positions_.size(); ++k)
        {
            ids.push_back(item.ids_[k]);
            grid.push_back(CsgGridPoint::From(item.positions_[k]));
        }
    }
    const CsgSharedPolygon tetrahedronFace = faces[1];

    CsgSolid::Builder builder(farFaces + faces.size(), 400);
    builder.AppendComponent(*far, 0, false);
    builder.AppendRegion(ea::move(faces), {}, {ids.data(), ids.size()}, 7, {grid.data(), grid.size()});
    const auto solid = builder.Finish();

    // The sheet is gone. The four faces of the tetrahedron stay, in order, with their ids and triangles.
    REQUIRE(solid->FaceCount() == farFaces + 4);
    CHECK(solid->GetFace(farFaces).get() == tetrahedronFace.get());
    CHECK(solid->IsClosed());
    CHECK_FALSE(solid->HasOpenEdges());
    CHECK(solid->OpenEdgeCount() == 0);
    CHECK(solid->ComponentCount() == 2);
    REQUIRE(solid->GetTriangleStart().size() == farFaces + 5);
    CHECK(solid->GetTriangleStart()[farFaces] == 12);
    CHECK(solid->GetTriangleStart()[farFaces + 4] == 16);
    REQUIRE(solid->GetTriangleVertices().size() == 16 * 3);
    REQUIRE(solid->GetTrianglePositions().size() == 16 * 3);
    for (unsigned face = 0; face < 4; ++face)
    {
        CHECK(solid->GetTriangleStart()[farFaces + face + 1] == 12 + face + 1);
        for (unsigned k = 0; k < 3; ++k)
        {
            // The ids are numbers of the exact loops: a key equals another one if the input ids are equal.
            for (unsigned other = 0; other < 4; ++other)
            {
                for (unsigned j = 0; j < 3; ++j)
                {
                    CHECK((solid->VertexKey(farFaces + face, k) == solid->VertexKey(farFaces + other, j))
                        == (tetrahedron[face].ids_[k] == tetrahedron[other].ids_[j]));
                    CHECK(solid->VertexKey(farFaces + face, k) >= farIds);
                }
            }
            CHECK(solid->GetFace(farFaces + face)->vertices_[k].GetPosition() == tetrahedron[face].positions_[k]);
            const unsigned corner = (12 + face) * 3 + k;
            CHECK(solid->GetTriangleVertices()[corner] < 3);
            CHECK(solid->GetTrianglePositions()[corner]
                == solid->GetFace(farFaces + face)->vertices_[solid->GetTriangleVertices()[corner]].GetPosition());
        }
        Vector3 low = tetrahedron[face].positions_[0];
        Vector3 high = low;
        for (const Vector3& position : tetrahedron[face].positions_)
        {
            low = Vector3(ea::min(low.x_, position.x_), ea::min(low.y_, position.y_), ea::min(low.z_, position.z_));
            high = Vector3(ea::max(high.x_, position.x_), ea::max(high.y_, position.y_), ea::max(high.z_, position.z_));
        }
        CHECK(solid->GetFaceMin()[farFaces + face] == low);
        CHECK(solid->GetFaceMax()[farFaces + face] == high);
    }
    CHECK(solid->ContainsPoint(Vector3(0.1f, 0.1f, 0.1f)));
    CHECK_FALSE(solid->ContainsPoint(Vector3(5.2f, 0.2f, 0.0f)));
}

TEST_CASE("Builder region removes a closed sheet with no volume", "[solid][kept][region][empty]")
{
    const auto far = MakeBoxSolid(Vector3(40.0f, 0.0f, 0.0f), 1.0f);
    const unsigned farFaces = far->FaceCount();

    const Vector3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0), d(0, 0, 1);
    const Vector3 e(5, 0, 0), f(6, 0, 0), g(6, 1, 0), h(5, 1, 0);
    struct Item
    {
        ea::vector<Vector3> positions_;
        ea::vector<int> ids_;
    };
    // A tetrahedron with outward faces, ids 0 to 3.
    const Item tetrahedron[] = {
        {{a, c, b}, {0, 2, 1}},
        {{a, b, d}, {0, 1, 3}},
        {{a, d, c}, {0, 3, 2}},
        {{b, c, d}, {1, 2, 3}},
    };
    // A quad e, f, g, h in two directions, ids 4 to 7. The first copy has the diagonal e-g, the second one has the
    // diagonal f-h. No triangle of one copy has the points of a triangle of the other copy. So no triangle
    // cancels, and the four faces make a closed sheet that has no volume.
    const Item sheet[] = {
        {{e, f, g}, {4, 5, 6}},
        {{e, g, h}, {4, 6, 7}},
        {{f, e, h}, {5, 4, 7}},
        {{f, h, g}, {5, 7, 6}},
    };

    const auto build = [&](bool withSheet)
    {
        ea::vector<const Item*> items;
        if (withSheet)
        {
            items.push_back(&sheet[0]);
            items.push_back(&sheet[2]);
        }
        for (const Item& item : tetrahedron)
            items.push_back(&item);
        if (withSheet)
        {
            items.push_back(&sheet[1]);
            items.push_back(&sheet[3]);
        }
        ea::vector<CsgSharedPolygon> faces;
        ea::vector<int> ids;
        ea::vector<CsgGridPoint> grid;
        for (const Item* item : items)
        {
            faces.push_back(
                ea::make_shared<CsgPolygon>(MakeTestPolygon({item->positions_.data(), item->positions_.size()})));
            for (unsigned k = 0; k < item->positions_.size(); ++k)
            {
                ids.push_back(item->ids_[k]);
                grid.push_back(CsgGridPoint::From(item->positions_[k]));
            }
        }
        CsgSolid::Builder builder(farFaces + faces.size(), 400);
        builder.AppendComponent(*far, 0, false);
        builder.AppendRegion(ea::move(faces), {}, {ids.data(), ids.size()}, 8, {grid.data(), grid.size()});
        return builder.Finish();
    };

    const auto withSheet = build(true);
    const auto without = build(false);

    // The sheet is gone. The result is the same as the one of the tetrahedron alone.
    REQUIRE(withSheet->FaceCount() == farFaces + 4);
    CHECK(withSheet->FaceCount() == without->FaceCount());
    CHECK(withSheet->IsClosed());
    CHECK_FALSE(withSheet->HasOpenEdges());
    CHECK(withSheet->ComponentCount() == 2);
    CHECK(withSheet->GetTriangleStart().size() == without->GetTriangleStart().size());
    CHECK(withSheet->GetTriangleStart().back() == without->GetTriangleStart().back());
    CHECK(withSheet->Volume() == Catch::Approx(without->Volume()));
    for (unsigned face = 0; face < 4; ++face)
    {
        for (unsigned k = 0; k < 3; ++k)
        {
            CHECK(withSheet->GetFace(farFaces + face)->vertices_[k].GetPosition() == tetrahedron[face].positions_[k]);
            for (unsigned other = 0; other < 4; ++other)
            {
                for (unsigned j = 0; j < 3; ++j)
                {
                    CHECK((withSheet->VertexKey(farFaces + face, k) == withSheet->VertexKey(farFaces + other, j))
                        == (tetrahedron[face].ids_[k] == tetrahedron[other].ids_[j]));
                }
            }
        }
    }
    CHECK(withSheet->ContainsPoint(Vector3(0.1f, 0.1f, 0.1f)));
    CHECK_FALSE(withSheet->ContainsPoint(Vector3(5.5f, 0.5f, 0.0f)));
}

TEST_CASE("Builder region keeps a closed group that has volume", "[solid][kept][region][empty]")
{
    // The tetrahedron alone is a closed group. Its volume is not zero, so it stays, also when it is the whole region.
    const Vector3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0), d(0, 0, 1);
    const Vector3 corners[4][3] = {{a, c, b}, {a, b, d}, {a, d, c}, {b, c, d}};
    const int ids[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    ea::vector<CsgSharedPolygon> faces;
    ea::vector<int> faceIds;
    ea::vector<CsgGridPoint> grid;
    for (unsigned face = 0; face < 4; ++face)
    {
        faces.push_back(ea::make_shared<CsgPolygon>(MakeTestPolygon({corners[face], 3})));
        for (unsigned k = 0; k < 3; ++k)
        {
            faceIds.push_back(ids[face][k]);
            grid.push_back(CsgGridPoint::From(corners[face][k]));
        }
    }
    CsgSolid::Builder builder(4, 12);
    builder.AppendRegion(ea::move(faces), {}, {faceIds.data(), faceIds.size()}, 4, {grid.data(), grid.size()});
    const auto solid = builder.Finish();
    CHECK(solid->FaceCount() == 4);
    CHECK(solid->IsClosed());
    CHECK(solid->ComponentCount() == 1);
    CHECK(solid->Volume() == Catch::Approx(1.0 / 6.0));
}
