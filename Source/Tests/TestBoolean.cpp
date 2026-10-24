// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoolean.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"

#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <cmath>
#include <cstring>
#include <limits>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

constexpr double VOLUME_TOLERANCE = 1e-3;

using SolidPtr = SharedPtr<const CsgSolid>;

SolidPtr MakeSolid(ea::vector<CsgPolygon>&& polygons)
{
    return SolidPtr(CsgSolid::FromOperand(ea::move(polygons)));
}

SolidPtr MakeBox(const Vector3& center, unsigned shared, const Matrix3x4& transform = Matrix3x4::IDENTITY)
{
    return MakeSolid(MakeBoxPolygons(center, Vector3(1.0f, 1.0f, 1.0f), shared, transform));
}

SolidPtr Run(const SolidPtr& a, const SolidPtr& b, CsgOperation operation)
{
    return Boolean(a, b, operation, CSG_DEFAULT_EPSILON);
}

double Volume(const SolidPtr& solid)
{
    const auto polygons = solid->ToPolygons();
    return SignedVolume(polygons);
}

bool IsClosed(const SolidPtr& solid)
{
    const auto polygons = solid->ToPolygons();
    return IsEdgeClosed(polygons, CSG_DEFAULT_EPSILON);
}

/// Check that the result is a closed mesh without open edges.
void CheckClosed(const SolidPtr& solid)
{
    CHECK(IsClosed(solid));
    CHECK_FALSE(solid->HasOpenEdges());
}

template <class T> bool SpansEqual(ea::span<const T> x, ea::span<const T> y)
{
    return x.size() == y.size() && (x.empty() || std::memcmp(x.data(), y.data(), x.size() * sizeof(T)) == 0);
}

bool BitEqual(float x, float y)
{
    return std::memcmp(&x, &y, sizeof(float)) == 0;
}

bool BitEqual(const Vector3& x, const Vector3& y)
{
    return BitEqual(x.x_, y.x_) && BitEqual(x.y_, y.y_) && BitEqual(x.z_, y.z_);
}

/// Return true if the vertices, the triangles and the face data of the solids are equal bit by bit.
bool SolidsIdentical(const CsgSolid& x, const CsgSolid& y)
{
    if (x.FaceCount() != y.FaceCount() || x.ComponentCount() != y.ComponentCount())
        return false;
    for (unsigned f = 0; f < x.FaceCount(); ++f)
    {
        const CsgPolygon& px = *x.GetFace(f);
        const CsgPolygon& py = *y.GetFace(f);
        if (px.vertices_.size() != py.vertices_.size() || px.shared_ != py.shared_)
            return false;
        for (unsigned v = 0; v < px.vertices_.size(); ++v)
        {
            if (!BitEqual(px.vertices_[v].GetPosition(), py.vertices_[v].GetPosition())
                || std::memcmp(&px.vertices_[v].normal_, &py.vertices_[v].normal_, sizeof(px.vertices_[v].normal_)) != 0)
                return false;
        }
    }
    return SpansEqual(x.GetTriangleStart(), y.GetTriangleStart())
        && SpansEqual(x.GetTriangleVertices(), y.GetTriangleVertices())
        && SpansEqual(x.GetTrianglePositions(), y.GetTrianglePositions())
        && SpansEqual(x.GetComponentStart(), y.GetComponentStart())
        && SpansEqual(x.GetComponentFaces(), y.GetComponentFaces());
}

SolidPtr MakeRotatedBox(const Vector3& center, unsigned shared)
{
    const Quaternion rotation(37.0f, Vector3(1.0f, 1.0f, 0.0f).Normalized());
    return MakeBox(center, shared, Matrix3x4(Vector3::ZERO, rotation, 1.0f));
}

} // namespace

TEST_CASE("Boolean of overlapping boxes gives the expected volumes", "[boolean]")
{
    const auto a = MakeBox(Vector3::ZERO, 1);
    const auto b = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2);

    const auto unionAB = Run(a, b, CsgOperation::Union);
    const auto differenceAB = Run(a, b, CsgOperation::DifferenceAB);
    const auto intersectionAB = Run(a, b, CsgOperation::Intersection);
    CHECK(Volume(unionAB) == Catch::Approx(12.0).margin(VOLUME_TOLERANCE));
    CHECK(Volume(differenceAB) == Catch::Approx(4.0).margin(VOLUME_TOLERANCE));
    CHECK(Volume(intersectionAB) == Catch::Approx(4.0).margin(VOLUME_TOLERANCE));
    CheckClosed(unionAB);
    CheckClosed(differenceAB);
    CheckClosed(intersectionAB);

    // The other order.
    CHECK(Volume(Run(b, a, CsgOperation::Union)) == Catch::Approx(12.0).margin(VOLUME_TOLERANCE));
    CHECK(Volume(Run(b, a, CsgOperation::DifferenceAB)) == Catch::Approx(4.0).margin(VOLUME_TOLERANCE));
    CHECK(Volume(Run(b, a, CsgOperation::Intersection)) == Catch::Approx(4.0).margin(VOLUME_TOLERANCE));
}

TEST_CASE("Boolean of boxes that share a face", "[boolean]")
{
    const auto a = MakeBox(Vector3::ZERO, 1);
    const auto b = MakeBox(Vector3(2.0f, 0.0f, 0.0f), 2);

    const auto unionAB = Run(a, b, CsgOperation::Union);
    const auto differenceAB = Run(a, b, CsgOperation::DifferenceAB);
    const auto intersectionAB = Run(a, b, CsgOperation::Intersection);
    CHECK(Volume(unionAB) == Catch::Approx(16.0).margin(VOLUME_TOLERANCE));
    CHECK(Volume(differenceAB) == Catch::Approx(8.0).margin(VOLUME_TOLERANCE));
    CHECK(intersectionAB->FaceCount() == 0);
    CHECK(Volume(intersectionAB) == Catch::Approx(0.0).margin(VOLUME_TOLERANCE));
    CheckClosed(unionAB);
    CheckClosed(differenceAB);
    CheckClosed(intersectionAB);
}

TEST_CASE("Boolean of a box and a rotated box keeps the volume identities", "[boolean]")
{
    const auto a = MakeBox(Vector3::ZERO, 1);
    const auto b = MakeRotatedBox(Vector3(0.3f, 0.2f, 0.1f), 2);
    const double volumeA = Volume(a);
    const double volumeB = Volume(b);

    const auto unionAB = Run(a, b, CsgOperation::Union);
    const auto differenceAB = Run(a, b, CsgOperation::DifferenceAB);
    const auto intersectionAB = Run(a, b, CsgOperation::Intersection);
    const double volumeUnion = Volume(unionAB);
    const double volumeIntersection = Volume(intersectionAB);
    const double volumeDifference = Volume(differenceAB);
    CHECK(volumeIntersection > 1.0);
    CHECK(volumeUnion + volumeIntersection == Catch::Approx(volumeA + volumeB).epsilon(1e-3));
    CHECK(volumeDifference == Catch::Approx(volumeA - volumeIntersection).epsilon(1e-3));
    CheckClosed(unionAB);
    CheckClosed(differenceAB);
    CheckClosed(intersectionAB);
}

TEST_CASE("Boolean of a sphere and a box keeps the volume identities", "[boolean]")
{
    const auto sphere = MakeSolid(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.3f), 1.3f, 12, 16, 1));
    const auto box = MakeBox(Vector3::ZERO, 2);
    const double volumeSphere = Volume(sphere);
    const double volumeBox = Volume(box);

    const auto unionAB = Run(sphere, box, CsgOperation::Union);
    const auto differenceAB = Run(sphere, box, CsgOperation::DifferenceAB);
    const auto intersectionAB = Run(sphere, box, CsgOperation::Intersection);
    const double volumeIntersection = Volume(intersectionAB);
    CHECK(volumeIntersection > 1.0);
    CHECK(Volume(unionAB) + volumeIntersection == Catch::Approx(volumeSphere + volumeBox).epsilon(1e-3));
    CHECK(Volume(differenceAB) == Catch::Approx(volumeSphere - volumeIntersection).epsilon(1e-3));
    CheckClosed(unionAB);
    CheckClosed(differenceAB);
    CheckClosed(intersectionAB);
}

TEST_CASE("Boolean of a sphere inside a box", "[boolean]")
{
    const auto box = MakeBox(Vector3::ZERO, 1);
    const auto sphere = MakeSolid(MakeSpherePolygons(Vector3(0.1f, 0.0f, 0.0f), 0.5f, 8, 12, 2));
    const double volumeBox = Volume(box);
    const double volumeSphere = Volume(sphere);
    REQUIRE(volumeSphere > 0.3);

    const auto unionAB = Run(box, sphere, CsgOperation::Union);
    const auto differenceAB = Run(box, sphere, CsgOperation::DifferenceAB);
    const auto intersectionAB = Run(box, sphere, CsgOperation::Intersection);
    CHECK(Volume(unionAB) == Catch::Approx(volumeBox).margin(VOLUME_TOLERANCE));
    CHECK(Volume(intersectionAB) == Catch::Approx(volumeSphere).margin(VOLUME_TOLERANCE));
    CHECK(Volume(differenceAB) == Catch::Approx(volumeBox - volumeSphere).margin(VOLUME_TOLERANCE));
    CheckClosed(unionAB);
    CheckClosed(differenceAB);
    CheckClosed(intersectionAB);

    // The sphere is the first operand.
    CHECK(Volume(Run(sphere, box, CsgOperation::Union)) == Catch::Approx(volumeBox).margin(VOLUME_TOLERANCE));
    CHECK(Volume(Run(sphere, box, CsgOperation::Intersection)) == Catch::Approx(volumeSphere).margin(VOLUME_TOLERANCE));
    CHECK(Run(sphere, box, CsgOperation::DifferenceAB)->FaceCount() == 0);
}

TEST_CASE("Boolean results are operands again and the union is associative", "[boolean]")
{
    const auto a = MakeBox(Vector3::ZERO, 1);
    const auto b = MakeBox(Vector3(0.8f, 0.3f, 0.2f), 2);
    const auto c = MakeRotatedBox(Vector3(0.5f, 1.0f, 0.3f), 3);

    const auto left = Run(Run(a, b, CsgOperation::Union), c, CsgOperation::Union);
    const auto right = Run(a, Run(b, c, CsgOperation::Union), CsgOperation::Union);
    CHECK(Volume(left) > 12.0);
    CHECK(Volume(left) == Catch::Approx(Volume(right)).margin(VOLUME_TOLERANCE));
    CheckClosed(left);
    CheckClosed(right);

    // A difference chain: (A - B) - C and A - (B + C) are the same shape.
    const auto chain = Run(Run(a, b, CsgOperation::DifferenceAB), c, CsgOperation::DifferenceAB);
    const auto together = Run(a, Run(b, c, CsgOperation::Union), CsgOperation::DifferenceAB);
    CHECK(Volume(chain) == Catch::Approx(Volume(together)).margin(VOLUME_TOLERANCE));
    CheckClosed(chain);
    CheckClosed(together);
}

TEST_CASE("Boolean is deterministic", "[boolean]")
{
    for (const CsgOperation operation :
        {CsgOperation::Union, CsgOperation::DifferenceAB, CsgOperation::Intersection})
    {
        // Two runs over operands that are built twice.
        const auto first = Run(MakeBox(Vector3::ZERO, 1), MakeRotatedBox(Vector3(0.3f, 0.2f, 0.1f), 2), operation);
        const auto second = Run(MakeBox(Vector3::ZERO, 1), MakeRotatedBox(Vector3(0.3f, 0.2f, 0.1f), 2), operation);
        CHECK(first->FaceCount() > 0);
        CHECK(SolidsIdentical(*first, *second));

        // The same call repeated over the same operands.
        const auto a = MakeSolid(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.3f), 1.3f, 8, 12, 1));
        const auto b = MakeBox(Vector3::ZERO, 2);
        const auto third = Run(a, b, operation);
        const auto fourth = Run(a, b, operation);
        CHECK(third->FaceCount() > 0);
        CHECK(SolidsIdentical(*third, *fourth));
    }
}

TEST_CASE("Boolean follows the rules for an empty operand", "[boolean]")
{
    const auto empty = MakeSolid({});
    const auto box = MakeBox(Vector3::ZERO, 1);
    REQUIRE(empty->FaceCount() == 0);

    // An empty first operand: a union gives the second operand, the other operations give the first one.
    CHECK(Run(empty, box, CsgOperation::Union).Get() == box.Get());
    CHECK(Run(empty, box, CsgOperation::DifferenceAB).Get() == empty.Get());
    CHECK(Run(empty, box, CsgOperation::Intersection).Get() == empty.Get());
    // An empty second operand: an intersection gives the second operand, the other operations give the first one.
    CHECK(Run(box, empty, CsgOperation::Union).Get() == box.Get());
    CHECK(Run(box, empty, CsgOperation::DifferenceAB).Get() == box.Get());
    CHECK(Run(box, empty, CsgOperation::Intersection).Get() == empty.Get());
    // Both empty.
    CHECK(Run(empty, empty, CsgOperation::Union)->FaceCount() == 0);
    CHECK(Run(empty, empty, CsgOperation::Intersection)->FaceCount() == 0);
    CHECK(Run(empty, empty, CsgOperation::DifferenceAB)->FaceCount() == 0);
}

TEST_CASE("Boolean of disjoint boxes", "[boolean]")
{
    const auto a = MakeBox(Vector3::ZERO, 1);
    const auto b = MakeBox(Vector3(5.0f, 0.5f, 0.0f), 2);

    const auto unionAB = Run(a, b, CsgOperation::Union);
    const auto differenceAB = Run(a, b, CsgOperation::DifferenceAB);
    const auto intersectionAB = Run(a, b, CsgOperation::Intersection);
    CHECK(Volume(unionAB) == Catch::Approx(16.0).margin(VOLUME_TOLERANCE));
    CHECK(unionAB->ComponentCount() == 2);
    CHECK(Volume(differenceAB) == Catch::Approx(8.0).margin(VOLUME_TOLERANCE));
    CHECK(intersectionAB->FaceCount() == 0);
    CheckClosed(unionAB);
    CheckClosed(differenceAB);
    CheckClosed(intersectionAB);
}

TEST_CASE("Boolean with a corner that is not finite does not crash", "[boolean]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (const float bad : {nan, inf})
    {
        // The vertex is on the side of the box that the other box crosses.
        auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3(1.0f, 1.0f, 1.0f), 1);
        for (CsgPolygon& polygon : polygons)
        {
            if (polygon.plane_.normal_.x_ > 0.5f)
                polygon.vertices_[0].SetPosition(Vector3(1.0f, bad, 1.0f));
        }
        const auto broken = MakeSolid(ea::move(polygons));
        const auto box = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2);
        const auto small = MakeBox(Vector3(0.0f, 0.0f, 0.0f), 3);
        for (const CsgOperation operation :
            {CsgOperation::Union, CsgOperation::DifferenceAB, CsgOperation::Intersection})
        {
            CHECK_NOTHROW(Run(broken, box, operation));
            CHECK_NOTHROW(Run(box, broken, operation));
            CHECK_NOTHROW(Run(broken, small, operation));
            CHECK_NOTHROW(Run(small, broken, operation));
            CHECK_NOTHROW(Run(broken, broken, operation));
        }
    }
}

TEST_CASE("Boolean takes the default epsilon for an epsilon that is not positive", "[boolean]")
{
    const auto a = MakeBox(Vector3::ZERO, 1);
    const auto b = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2);
    for (const float epsilon : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()})
    {
        const auto result = Boolean(a, b, CsgOperation::Union, epsilon);
        CHECK(Volume(result) == Catch::Approx(12.0).margin(VOLUME_TOLERANCE));
    }
}


// The union of boxes with coplanar faces has faces with a zero-width spike. The old triangulator could not triangulate
// such a face. The triangulator gives it triangles, so the next corefinement sees it.
TEST_CASE("Boolean of a result with coplanar faces is associative", "[boolean]")
{
    const auto a = MakeBox(Vector3::ZERO, 1);
    const auto b = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2);
    const auto c = MakeRotatedBox(Vector3(0.5f, 1.0f, 0.3f), 3);

    const auto left = Run(Run(a, b, CsgOperation::Union), c, CsgOperation::Union);
    const auto right = Run(a, Run(b, c, CsgOperation::Union), CsgOperation::Union);
    CHECK(Volume(right) == Catch::Approx(16.051433).margin(VOLUME_TOLERANCE));
    CHECK(Volume(left) == Catch::Approx(Volume(right)).margin(VOLUME_TOLERANCE));
    CheckClosed(left);
    CheckClosed(right);
}

// Rotation rounds the positions, so the vertices of a coplanar face are on a shared edge only within rounding.
TEST_CASE("Boolean of a rotated result with coplanar faces is associative", "[boolean]")
{
    for (const float angle : {30.0f, 12.5f, 77.0f, 4.5f, 9.4f, 14.3f})
    {
        CAPTURE(angle);
        const Quaternion world(angle, Vector3::ONE.Normalized());
        const Quaternion own(37.0f, Vector3(1.0f, 1.0f, 0.0f).Normalized());
        const auto a = MakeBox(world * Vector3::ZERO, 1, Matrix3x4(Vector3::ZERO, world, 1.0f));
        const auto b = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2, Matrix3x4(Vector3::ZERO, world, 1.0f));
        const auto c = MakeBox(Vector3(0.5f, 1.0f, 0.3f), 3, Matrix3x4(Vector3::ZERO, world * own, 1.0f));

        const auto left = Run(Run(a, b, CsgOperation::Union), c, CsgOperation::Union);
        const auto right = Run(a, Run(b, c, CsgOperation::Union), CsgOperation::Union);
        CHECK(Volume(right) == Catch::Approx(16.051433).margin(VOLUME_TOLERANCE));
        CHECK(Volume(left) == Catch::Approx(Volume(right)).margin(VOLUME_TOLERANCE));
        CheckClosed(left);
        CheckClosed(right);
    }
}
