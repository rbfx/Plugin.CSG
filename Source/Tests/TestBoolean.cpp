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
#include <sstream>
#include <string>
#include <vector>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

constexpr double VOLUME_TOLERANCE = 1e-3;

/// A snapped crossing position moves up to 2^-11 units. The closed mesh check needs a larger tolerance.
constexpr float GRID_TOLERANCE = 2e-3f;

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
    return IsEdgeClosed(polygons, GRID_TOLERANCE);
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
    // The grid step is 1/1024 unit. With z = 0.3 the pole of the sphere snaps onto the plane of the box.
    // The value 0.35 keeps the pole off the plane.
    const auto sphere = MakeSolid(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.35f), 1.3f, 12, 16, 1));
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

namespace
{

/// Check the associativity of the union for the rotation about the axis (1, 1, 1) with the angle.
void CheckRotatedAssociativity(float angle)
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

} // namespace

// Rotation rounds the positions, so the vertices of a coplanar face are on a shared edge only within rounding.
// The grid step is 2^-10 units. The exact tests snap the corners of rotated brushes one by one.
// The snapped corners do not stay coplanar. This gives wrong unions, for example at the angle 4.5.
// With CSG_EXACT_USE_DOUBLE the grid step is 2^-26 units and the test passes. The float mode still fails.
TEST_CASE("known limit: rotated result with coplanar faces is associative", "[boolean][!mayfail]")
{
    for (const float angle : {30.0f, 12.5f, 4.5f, 14.3f})
        CheckRotatedAssociativity(angle);
}

// The union of A and B keeps zero-width spikes in some faces. In the union at 9.4 degrees, the face f3 has the
// vertices (8, 5, 2), and vertex 8 lies on the segment 2-5. The neighbour face f1 has the edge 2-5 without
// vertex 8. The next corefinement with C leaves a hole near the crossing on that edge. The spike removal is not done.
TEST_CASE("known limit: zero-width spikes in kept faces", "[boolean][!mayfail]")
{
    for (const float angle : {77.0f, 9.4f})
        CheckRotatedAssociativity(angle);
}

namespace
{

struct SweepStats
{
    unsigned samples_{};
    unsigned failures_{};
    unsigned unionWrong_{};
    std::vector<std::string> list_;
};

/// Run the union of A, B and C for one rotation. A is the cube [-1, 1]^3 and B is [0, 2] x [-1, 1]^2.
/// B shares the plane x = 1 with A, so the union has coplanar contact. Count a failed sample in the statistics.
void SweepSample(const Vector3& axis, float angle, SweepStats& stats)
{
    const Quaternion world(angle, axis.Normalized());
    const Quaternion own(37.0f, Vector3(1.0f, 1.0f, 0.0f).Normalized());
    const Matrix3x4 worldTransform(Vector3::ZERO, world, 1.0f);
    const auto a = MakeBox(Vector3::ZERO, 1, worldTransform);
    const auto b = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2, worldTransform);
    const auto c = MakeBox(Vector3(0.5f, 1.0f, 0.3f), 3, Matrix3x4(Vector3::ZERO, world * own, 1.0f));

    const auto ab = Run(a, b, CsgOperation::Union);
    const bool unionRight = std::abs(Volume(ab) - 12.0) <= VOLUME_TOLERANCE && IsClosed(ab);
    const auto left = Run(ab, c, CsgOperation::Union);
    const auto right = Run(a, Run(b, c, CsgOperation::Union), CsgOperation::Union);
    const bool associative = std::abs(Volume(left) - Volume(right)) <= VOLUME_TOLERANCE && IsClosed(left)
        && IsClosed(right);

    ++stats.samples_;
    if (unionRight && associative)
        return;
    ++stats.failures_;
    stats.unionWrong_ += unionRight ? 0 : 1;
    std::ostringstream line;
    line << "axis (" << axis.x_ << " " << axis.y_ << " " << axis.z_ << ") angle " << angle << " union "
         << (unionRight ? "right" : "WRONG");
    stats.list_.push_back(line.str());
}

void RunSweep(float step, unsigned maxFailures)
{
    const Vector3 axes[] = {Vector3::RIGHT, Vector3::UP, Vector3::FORWARD, Vector3::ONE};
    SweepStats stats;
    for (const Vector3& axis : axes)
    {
        for (unsigned i = 0;; ++i)
        {
            const float angle = 0.3f + step * static_cast<float>(i);
            if (angle > 180.0f)
                break;
            SweepSample(axis, angle, stats);
        }
    }
    for (const std::string& line : stats.list_)
        UNSCOPED_INFO(line);
    INFO("samples " << stats.samples_ << ", failures " << stats.failures_ << ", wrong unions " << stats.unionWrong_);
    CHECK(stats.failures_ <= maxFailures);
}

} // namespace

// The sweep covers rotations of the coplanar contact: 4 axes and 257 angles from 0.3 to 179.5 degrees.
// The grid step is 2^-10 units. The exact tests snap the corners of rotated boxes one by one.
// The snapped corners do not stay coplanar. The measured count of failed samples is 119 of 1028.
// In all of them the union of A and B is right. The next boolean fails because of zero-width spikes in its faces.
// The failed samples are about the axis (1, 1, 1) (70), the z axis (25), the y axis (17) and the x axis (7).
// The test fails if more than 48 samples fail. It is a known limit, so the failure is expected.
// With CSG_EXACT_USE_DOUBLE the grid step is 2^-16 units. Then 51 samples fail: the axis (1, 1, 1) (45) and the
// z axis (6). The union of A and B is right in all of them. The test also fails in double mode.
TEST_CASE("known limit: union of boxes with coplanar contact is associative over a sweep of rotations",
    "[boolean][sweep][!mayfail]")
{
    RunSweep(0.7f, 48);
}

// A rotation about the axis (1, 1, 1) keeps the corner (1, 1, 1) of A. The corner lies at the middle of an edge of B.
// The snapped corner is often exactly on the snapped edge. The edges of A at the corner then cross a face of B
// at the corner, and the crossings make a closed curve of zero size. The perturbation gives the orientation of
// the curve. The angles are samples of the sweep that gave wrong unions in float mode or in double mode.
TEST_CASE("Union of rotated boxes with a corner on an edge of the other box is right", "[boolean]")
{
    for (const int step : {9, 24, 37, 50, 109, 116})
    {
        const float angle = 0.3f + 0.7f * static_cast<float>(step);
        CAPTURE(angle);
        const Matrix3x4 transform(Vector3::ZERO, Quaternion(angle, Vector3::ONE.Normalized()), 1.0f);
        const auto ab = Run(MakeBox(Vector3::ZERO, 1, transform), MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2, transform),
            CsgOperation::Union);
        CHECK(Volume(ab) == Catch::Approx(12.0).margin(VOLUME_TOLERANCE));
        CHECK(IsClosed(ab));
    }
}
