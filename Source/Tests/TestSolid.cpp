// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"

#include <catch2/catch_amalgamated.hpp>

#include <limits>

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

    // Without welding tolerance the faces fall apart.
    const CsgSolid tight(polygons, 1e-8f);
    CHECK(tight.ComponentCount() > 1);
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
    CHECK(small.OpenEdgeLength() == Catch::Approx(0.016f).margin(1e-5f));
    // The parity tolerates the crack.
    CHECK(small.ContainsPoint(Vector3(0.1f, 0.2f, 0.3f)));
    CHECK_FALSE(small.ContainsPoint(Vector3(2.0f, 0.2f, 0.3f)));

    // A hole of the side 0.006 has the edge length 0.024. That is above 200 epsilons (0.02).
    // The total edge length is about 29.66, so only 0.001 of it (about 0.0297) keeps the solid closed.
    const CsgSolid fraction(MakeBoxWithPinhole(0.006f), 1e-4f);
    CHECK(fraction.IsClosed());
    CHECK(fraction.HasOpenEdges());
    CHECK(fraction.OpenEdgeLength() == Catch::Approx(0.024f).margin(1e-5f));

    // A hole of the side 0.02 has the edge length 0.08. That is more than the budget.
    const CsgSolid large(MakeBoxWithPinhole(0.02f), 1e-4f);
    CHECK_FALSE(large.IsClosed());
    CHECK(large.HasOpenEdges());
    CHECK(large.OpenEdgeLength() == Catch::Approx(0.08f).margin(1e-4f));

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

    // Near a face, on each side of it.
    for (const float offset : {0.01f, 1e-3f, 1e-5f})
    {
        CHECK(solid.ContainsPoint(center + Vector3(0.5f - offset, 0.0f, 0.0f)));
        CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.5f + offset, 0.0f, 0.0f)));
        CHECK(solid.ContainsPoint(center + Vector3(0.1f, -1.0f + offset, 0.2f)));
        CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.1f, -1.0f - offset, 0.2f)));
        CHECK(solid.ContainsPoint(center + Vector3(0.1f, 0.2f, 1.5f - offset)));
        CHECK_FALSE(solid.ContainsPoint(center + Vector3(0.1f, 0.2f, 1.5f + offset)));
    }

    // On a face, on an edge and on a corner the result is not defined, but it is stable.
    const Vector3 surface[] = {center + Vector3(0.5f, 0.0f, 0.0f), center + Vector3(0.5f, 1.0f, 0.0f),
        center + Vector3(0.5f, 1.0f, 1.5f), center + Vector3(-0.5f, -1.0f, -1.5f), center + Vector3(0.0f, 1.0f, 1.5f)};
    for (const Vector3& point : surface)
        CHECK(solid.ContainsPoint(point) == solid.ContainsPoint(point));

    // Points that are not finite are outside. A bad epsilon gives the default.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CHECK_FALSE(solid.ContainsPoint(Vector3(nan, nan, nan)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(center.x_, nan, center.z_)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(inf, 0.0f, 0.0f)));
    CHECK_FALSE(solid.ContainsPoint(Vector3(-inf, -inf, -inf)));
    CHECK(solid.ContainsPoint(center, 0.0f));
    CHECK(solid.ContainsPoint(center, nan));
    CHECK(solid.ContainsPoint(center, -1.0f));
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
    const float epsilon = 1e-4f;
    const CsgSolid solid(MakeBoxPolygons(center, halfSize, 1), epsilon);

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
        // A point within a few epsilons of the surface can go either way.
        if (Abs(distance) < 10.0f * epsilon)
            continue;
        ++checked;
        const bool expected = distance < 0.0f;
        inside += expected ? 1 : 0;
        CHECK(solid.ContainsPoint(point, epsilon) == expected);
    }
    // The sample must have points on both sides.
    CHECK(checked > 900);
    CHECK(inside > 20);
    CHECK(inside < checked);
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
