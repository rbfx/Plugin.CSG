// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestExactReference.h"
#include "TestGeometry.h"

#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <EASTL/algorithm.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <random>
#include <tuple>
#include <utility>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

// The grid step is 2^-10 units, so a snapped position moves up to 2^-11 units.
// One grid step: 2^-10 units in float mode and 2^-16 units in double mode.
constexpr float Tolerance = static_cast<float>(1.0 / CsgGridScale);

bool Near(const Vector3& a, const Vector3& b, float tolerance = Tolerance)
{
    return std::fabs(a.x_ - b.x_) <= tolerance && std::fabs(a.y_ - b.y_) <= tolerance
        && std::fabs(a.z_ - b.z_) <= tolerance;
}

bool InsideBounds(const Vector3& p, const CsgSolid& solid, float tolerance = Tolerance)
{
    const Vector3& lo = solid.GetBoundsMin();
    const Vector3& hi = solid.GetBoundsMax();
    return p.x_ >= lo.x_ - tolerance && p.y_ >= lo.y_ - tolerance && p.z_ >= lo.z_ - tolerance
        && p.x_ <= hi.x_ + tolerance && p.y_ <= hi.y_ + tolerance && p.z_ <= hi.z_ + tolerance;
}

/// True if the point is on the surface of the axis-aligned box.
bool OnBoxSurface(const Vector3& p, const Vector3& center, const Vector3& half, float tolerance = Tolerance)
{
    const float d[3] = {std::fabs(p.x_ - center.x_), std::fabs(p.y_ - center.y_), std::fabs(p.z_ - center.z_)};
    const float h[3] = {half.x_, half.y_, half.z_};
    bool onFace = false;
    for (int k = 0; k < 3; ++k)
    {
        if (d[k] > h[k] + tolerance)
            return false;
        if (std::fabs(d[k] - h[k]) <= tolerance)
            onFace = true;
    }
    return onFace;
}

/// True if both lists hold the same positions, in any order.
bool SamePositionSets(const ea::vector<Vector3>& a, const ea::vector<Vector3>& b, float tolerance = Tolerance)
{
    if (a.size() != b.size())
        return false;
    ea::vector<bool> used(b.size(), false);
    for (const Vector3& p : a)
    {
        bool found = false;
        for (unsigned i = 0; i < b.size() && !found; ++i)
        {
            if (!used[i] && Near(p, b[i], tolerance))
            {
                used[i] = true;
                found = true;
            }
        }
        if (!found)
            return false;
    }
    return true;
}

bool SameResult(const CsgCorefineTestResult& a, const CsgCorefineTestResult& b)
{
    if (a.positions_.size() != b.positions_.size() || a.segmentCountsA_ != b.segmentCountsA_
        || a.segmentCountsB_ != b.segmentCountsB_ || a.curveEdges_ != b.curveEdges_)
        return false;
    for (unsigned i = 0; i < a.positions_.size(); ++i)
    {
        if (!(a.positions_[i] == b.positions_[i]) || !(a.keys_[i] == b.keys_[i]))
            return false;
    }
    return true;
}

int Sum(const ea::vector<int>& values)
{
    int result = 0;
    for (int value : values)
        result += value;
    return result;
}

/// The curve is a set of closed loops with one direction: each crossing has one outgoing and one incoming segment.
bool CurveIsClosed(const CsgCorefineTestResult& result)
{
    ea::vector<int> outgoing(result.positions_.size(), 0);
    ea::vector<int> incoming(result.positions_.size(), 0);
    for (const auto& edge : result.curveEdges_)
    {
        ++outgoing[edge.first];
        ++incoming[edge.second];
    }
    for (unsigned i = 0; i < outgoing.size(); ++i)
    {
        if (outgoing[i] != 1 || incoming[i] != 1)
            return false;
    }
    return true;
}

Vector3 TriangleNormal(const CsgSolid& solid, int triangle)
{
    const auto positions = solid.GetTrianglePositions();
    const Vector3& p0 = positions[triangle * 3];
    return (positions[triangle * 3 + 1] - p0).CrossProduct(positions[triangle * 3 + 2] - p0);
}

/// Every segment runs along n_A x n_B, from the entering end to the exiting end.
bool SegmentsFollowNormals(const CsgSolid& a, const CsgSolid& b, const CsgCorefineTestResult& result)
{
    int checkedCount = 0;
    for (const CsgCorefineTestSegment& segment : result.segments_)
    {
        const Vector3 direction = TriangleNormal(a, segment.triangleA_).CrossProduct(TriangleNormal(b, segment.triangleB_));
        const Vector3 along = result.positions_[segment.q_] - result.positions_[segment.p_];
        if (along.Length() < 1e-4f || direction.Length() < 1e-6f)
            continue;
        if (direction.DotProduct(along) <= 0.0f)
            return false;
        ++checkedCount;
    }
    return checkedCount > 0;
}

struct AxisBox
{
    double lo_[3];
    double hi_[3];
};

AxisBox MakeAxisBox(const Vector3& center, const Vector3& half, const double (&shift)[3])
{
    AxisBox box;
    const double c[3] = {center.x_, center.y_, center.z_};
    const double h[3] = {half.x_, half.y_, half.z_};
    for (int k = 0; k < 3; ++k)
    {
        box.lo_[k] = c[k] - h[k] + shift[k];
        box.hi_[k] = c[k] + h[k] + shift[k];
    }
    return box;
}

/// Number of times the segment p-q crosses the surface of the box. It clips the segment with the three slabs.
int CountSurfaceCrossings(const double (&p)[3], const double (&q)[3], const AxisBox& box)
{
    double enter = -1e300;
    double exit = 1e300;
    for (int k = 0; k < 3; ++k)
    {
        const double d = q[k] - p[k];
        if (d == 0.0)
        {
            if (p[k] < box.lo_[k] || p[k] > box.hi_[k])
                return 0;
            continue;
        }
        double ta = (box.lo_[k] - p[k]) / d;
        double tb = (box.hi_[k] - p[k]) / d;
        if (ta > tb)
            std::swap(ta, tb);
        enter = std::max(enter, ta);
        exit = std::min(exit, tb);
    }
    if (!(enter < exit))
        return 0;
    return (enter > 0.0 && enter < 1.0 ? 1 : 0) + (exit > 0.0 && exit < 1.0 ? 1 : 0);
}

/// Crossings of the mesh edges of a solid with the surface of a box. The edges move by the shift.
int CountEdgeCrossings(const CsgSolid& solid, const AxisBox& box, const double (&edgeShift)[3])
{
    const auto positions = solid.GetTrianglePositions();
    ea::vector<ea::pair<Vector3, Vector3>> edges;
    for (unsigned t = 0; t < positions.size() / 3; ++t)
    {
        for (unsigned k = 0; k < 3; ++k)
        {
            Vector3 u = positions[t * 3 + k];
            Vector3 v = positions[t * 3 + (k + 1) % 3];
            if (std::tie(v.x_, v.y_, v.z_) < std::tie(u.x_, u.y_, u.z_))
                std::swap(u, v);
            bool known = false;
            for (const auto& edge : edges)
                known = known || (edge.first == u && edge.second == v);
            if (!known)
                edges.push_back(ea::make_pair(u, v));
        }
    }
    int count = 0;
    for (const auto& edge : edges)
    {
        const double p[3] = {edge.first.x_ + edgeShift[0], edge.first.y_ + edgeShift[1], edge.first.z_ + edgeShift[2]};
        const double q[3] = {
            edge.second.x_ + edgeShift[0], edge.second.y_ + edgeShift[1], edge.second.z_ + edgeShift[2]};
        count += CountSurfaceCrossings(p, q, box);
    }
    return count;
}

} // namespace

TEST_CASE("Corefine crossings of two overlapping boxes", "[corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));
    const CsgSolid b(MakeBoxPolygons(Vector3(1.0f, 0.0f, 0.0f), half, 2));
    const CsgCorefineTestResult result = CsgCorefineCrossingsForTest(a, b);

    // The second box moves by (e, e^2, e^3). Then it contains only the corner (1, 1, 1) of the first box, and the
    // first box contains only the corner (e, -1 + e^2, -1 + e^3) of the second box. At each of these corners three
    // box edges cross the other surface. The face diagonals that miss the corner enter and leave the other box,
    // two crossings each. Three faces meet at each corner, and all three diagonals avoid the corner.
    // Each operand adds 3 + 3 * 2 = 9 crossings: 18 in total.
    CHECK(result.positions_.size() == 18u);
    // An independent count with a small numeric displacement gives the same number.
    const double none[3] = {0.0, 0.0, 0.0};
    const double shift[3] = {1e-3, 1e-6, 1e-9};
    const Vector3 shiftedCenter(1.0f, 0.0f, 0.0f);
    CHECK(CountEdgeCrossings(a, MakeAxisBox(shiftedCenter, half, shift), none)
            + CountEdgeCrossings(b, MakeAxisBox(Vector3::ZERO, half, none), shift)
        == 18);
    for (const Vector3& p : result.positions_)
    {
        CHECK(OnBoxSurface(p, Vector3::ZERO, half));
        CHECK(OnBoxSurface(p, Vector3(1.0f, 0.0f, 0.0f), half));
    }
    CHECK(Sum(result.segmentCountsA_) == Sum(result.segmentCountsB_));
    CHECK(Sum(result.segmentCountsA_) == static_cast<int>(result.curveEdges_.size()));
    CHECK(CurveIsClosed(result));
    CHECK(SegmentsFollowNormals(a, b, result));
}

TEST_CASE("Corefine crossings do not depend on the operand order", "[corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const Matrix3x4 rotation(Vector3::ZERO, Quaternion(37.0f, Vector3(1.0f, 1.0f, 0.0f)), 1.0f);
    const CsgSolid box(MakeBoxPolygons(Vector3::ZERO, half, 1));
    const CsgSolid shifted(MakeBoxPolygons(Vector3(0.5f, 0.25f, 0.125f), Vector3(0.75f, 1.2f, 0.9f), 2));
    const CsgSolid rotated(MakeBoxPolygons(Vector3(0.3f, 0.2f, 0.1f), half, 3, rotation));

    SECTION("Offset box")
    {
        const auto ab = CsgCorefineCrossingsForTest(box, shifted);
        const auto ba = CsgCorefineCrossingsForTest(shifted, box);
        CHECK(ab.positions_.size() > 0);
        CHECK(SamePositionSets(ab.positions_, ba.positions_));
        CHECK(CurveIsClosed(ab));
        CHECK(CurveIsClosed(ba));
    }
    SECTION("Rotated box")
    {
        const auto ab = CsgCorefineCrossingsForTest(box, rotated);
        const auto ba = CsgCorefineCrossingsForTest(rotated, box);
        CHECK(ab.positions_.size() > 0);
        CHECK(SamePositionSets(ab.positions_, ba.positions_));
    }
    SECTION("Two runs give the same ids")
    {
        const auto first = CsgCorefineCrossingsForTest(box, rotated);
        const auto second = CsgCorefineCrossingsForTest(box, rotated);
        CHECK(SameResult(first, second));
        const auto third = CsgCorefineCrossingsForTest(box, shifted);
        const auto fourth = CsgCorefineCrossingsForTest(box, shifted);
        CHECK(SameResult(third, fourth));
    }
}

TEST_CASE("Corefine coplanar contact is resolved by the perturbation", "[corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));

    SECTION("Second box moves away from the touching face")
    {
        const CsgSolid b(MakeBoxPolygons(Vector3(2.0f, 0.0f, 0.0f), half, 2));
        const auto first = CsgCorefineCrossingsForTest(a, b);
        const auto second = CsgCorefineCrossingsForTest(a, b);
        CHECK(SameResult(first, second));
        // The displacement (eps, eps^2, eps^3) lifts the second box off the first one.
        CHECK(first.positions_.size() == 0);
        CHECK(Sum(first.segmentCountsA_) == 0);
    }
    SECTION("Second box moves into the touching face")
    {
        const CsgSolid b(MakeBoxPolygons(Vector3(-2.0f, 0.0f, 0.0f), half, 2));
        const auto first = CsgCorefineCrossingsForTest(a, b);
        const auto second = CsgCorefineCrossingsForTest(a, b);
        CHECK(SameResult(first, second));
        CHECK(first.positions_.size() < 1000);
        CHECK(Sum(first.segmentCountsA_) == Sum(first.segmentCountsB_));
        for (const Vector3& p : first.positions_)
            CHECK(InsideBounds(p, a));
    }
}

TEST_CASE("Corefine crossings of disjoint boxes", "[corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));
    const CsgSolid b(MakeBoxPolygons(Vector3(5.0f, 0.0f, 0.0f), half, 2));
    const auto result = CsgCorefineCrossingsForTest(a, b);
    CHECK(result.positions_.size() == 0);
    CHECK(Sum(result.segmentCountsA_) == 0);
    CHECK(Sum(result.segmentCountsB_) == 0);
    CHECK(result.curveEdges_.empty());
}

TEST_CASE("Corefine crossings of a box and a rotated box", "[corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const Matrix3x4 rotation(Vector3::ZERO, Quaternion(37.0f, Vector3(1.0f, 1.0f, 0.0f)), 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1));
    const CsgSolid b(MakeBoxPolygons(Vector3(0.3f, 0.2f, 0.1f), half, 2, rotation));
    const auto result = CsgCorefineCrossingsForTest(a, b);

    CHECK(result.positions_.size() > 0);
    for (const Vector3& p : result.positions_)
    {
        CHECK(InsideBounds(p, a));
        CHECK(InsideBounds(p, b));
    }
    CHECK(Sum(result.segmentCountsA_) == Sum(result.segmentCountsB_));
    CHECK(Sum(result.segmentCountsA_) == static_cast<int>(result.curveEdges_.size()));
    CHECK(CurveIsClosed(result));
    CHECK(SegmentsFollowNormals(a, b, result));
}

TEST_CASE("Corefine crossings of a sphere and a box", "[corefine]")
{
    const Vector3 half(1.0f, 1.0f, 1.0f);
    // The grid step is 1/1024 unit. With z = 0.3 the pole of the sphere snaps onto the plane of the box.
    // The crossings then depend on the operand order. The value 0.35 keeps the pole off the plane.
    const CsgSolid sphere(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.35f), 1.3f, 8, 12, 1));
    const CsgSolid box(MakeBoxPolygons(Vector3::ZERO, half, 2));

    const auto result = CsgCorefineCrossingsForTest(sphere, box);
    CHECK(result.positions_.size() > 0);
    for (const Vector3& p : result.positions_)
    {
        CHECK(InsideBounds(p, sphere));
        CHECK(InsideBounds(p, box));
    }
    CHECK(CurveIsClosed(result));
    CHECK(SegmentsFollowNormals(sphere, box, result));
    CHECK(SameResult(result, CsgCorefineCrossingsForTest(sphere, box)));

    const auto swapped = CsgCorefineCrossingsForTest(box, sphere);
    CHECK(SamePositionSets(result.positions_, swapped.positions_, 2e-3f));
}

namespace
{

/// Convert a 128 bit test number to an expansion that is safe to sum.
/// The number must be below 2^CsgExactVolumeBits for N = 24.
template <int N> CsgExactValue<N> MakeValue(const CsgTest::Int128& value)
{
    const ea::vector<float> terms = CsgTest::FromInt128(value);
    CsgExactValue<N> result;
    REQUIRE(static_cast<int>(terms.size()) <= N);
    for (size_t i = 0; i < terms.size(); ++i)
        result.terms_[i] = static_cast<CsgReal>(terms[i]);
    // The chunks of the conversion are not strongly nonoverlapping. The compression makes them so.
    result.count_ = CsgCompress(static_cast<int>(terms.size()), result.terms_);
    return result;
}

Crossing MakeCrossing(const CsgTest::Int128& low, const CsgTest::Int128& high, bool edgeInSecond,
    const CsgTest::Int128& normalX = CsgTest::Int128(int64_t{0}), const CsgTest::Int128& normalY = CsgTest::Int128(int64_t{0}),
    const CsgTest::Int128& normalZ = CsgTest::Int128(int64_t{0}))
{
    Crossing crossing;
    crossing.lowVolume_ = MakeValue<24>(low);
    crossing.difference_ = MakeValue<24>(low - high);
    crossing.key_.edgeInSecond_ = edgeInSecond;
    crossing.normal_ = {MakeValue<4>(normalX), MakeValue<4>(normalY), MakeValue<4>(normalZ)};
    return crossing;
}

} // namespace

TEST_CASE("Corefine CompareOnEdge orders crossings exactly", "[corefine]")
{
    SECTION("Different parameters")
    {
        // t = L / (L - H): 0.25 and 0.5.
        const Crossing early = MakeCrossing(1, -3, false);
        const Crossing late = MakeCrossing(1, -1, false);
        CHECK(CompareOnEdge(early, late) == -1);
        CHECK(CompareOnEdge(late, early) == 1);
        // Negative differences: the same parameters 0.25 and 0.5.
        const Crossing earlyNegative = MakeCrossing(-1, 3, false);
        const Crossing lateNegative = MakeCrossing(-1, 1, false);
        CHECK(CompareOnEdge(earlyNegative, lateNegative) == -1);
        CHECK(CompareOnEdge(lateNegative, earlyNegative) == 1);
        CHECK(CompareOnEdge(earlyNegative, late) == -1);
        CHECK(CompareOnEdge(late, earlyNegative) == 1);
    }
    SECTION("Equal parameters and equal normals")
    {
        const Crossing x = MakeCrossing(2, -2, false, 2, 4, 6);
        const Crossing y = MakeCrossing(1, -1, false, 1, 2, 3);
        CHECK(CompareOnEdge(x, y) == 0);
    }
    SECTION("Tie in the parameter is broken by the normals")
    {
        // Both parameters are 0.5. The first component with a different Nx / Dx decides.
        const Crossing x = MakeCrossing(2, -2, false, 1, 0, 0);
        const Crossing y = MakeCrossing(1, -1, false, 0, 0, 0);
        CHECK(CompareOnEdge(x, y) == -1);
        CHECK(CompareOnEdge(y, x) == 1);
        // The edge of the second operand reverses the shift.
        const Crossing xSecond = MakeCrossing(2, -2, true, 1, 0, 0);
        const Crossing ySecond = MakeCrossing(1, -1, true, 0, 0, 0);
        CHECK(CompareOnEdge(xSecond, ySecond) == 1);
        CHECK(CompareOnEdge(ySecond, xSecond) == -1);
        // The x components are equal (2 / 4 and 1 / 2). The y component decides: 3 / 4 is more than 1 / 2.
        const Crossing xy = MakeCrossing(2, -2, false, 2, 3, 7);
        const Crossing yy = MakeCrossing(1, -1, false, 1, 1, 7);
        CHECK(CompareOnEdge(xy, yy) == -1);
        // Only the z component differs.
        const Crossing xz = MakeCrossing(2, -2, false, 2, 4, 5);
        const Crossing yz = MakeCrossing(1, -1, false, 1, 2, 2);
        CHECK(CompareOnEdge(xz, yz) == -1);
    }
    SECTION("Negative difference of the volumes")
    {
        // Dx = -4 and Dy = 2. The parameters are 0.5 and 0.5. The denominator sign is negative.
        const Crossing x = MakeCrossing(-2, 2, false, 1, 0, 0);
        const Crossing y = MakeCrossing(1, -1, false, 0, 0, 0);
        CHECK(CompareOnEdge(x, y) == 1);
        CHECK(CompareOnEdge(y, x) == -1);
        const Crossing xSecond = MakeCrossing(-2, 2, true, 1, 0, 0);
        const Crossing ySecond = MakeCrossing(1, -1, true, 0, 0, 0);
        CHECK(CompareOnEdge(xSecond, ySecond) == -1);
        // Both differences are negative.
        const Crossing both = MakeCrossing(-1, 1, false, 0, 0, 0);
        CHECK(CompareOnEdge(x, both) == 1);
    }
    SECTION("Volumes beyond 64 bits")
    {
        const CsgTest::Int128 big = CsgTest::Mul64<CsgTest::Int128>(int64_t{1} << 35, int64_t{1} << 35); // 2^70
        const CsgTest::Int128 zero = CsgTest::Int128(int64_t{0});
        Crossing y = MakeCrossing(1, -1, false);
        // Both parameters are 0.5 exactly.
        CHECK(CompareOnEdge(MakeCrossing(big, zero - big, false), y) == 0);
        // A difference of one part in 2^71 needs the full precision of the products.
        const CsgTest::Int128 more = big + CsgTest::Int128(int64_t{1});
        const Crossing x = MakeCrossing(more, zero - big, false);
        CHECK(CompareOnEdge(x, y) == 1);
        CHECK(CompareOnEdge(y, x) == -1);
    }
}

TEST_CASE("Crossing order matches the int128 oracle", "[corefine]")
{
    using CsgTest::Int128;
    std::mt19937_64 random(2468);
    const auto uniform = [&random](int bits)
    {
        std::uniform_int_distribution<int64_t> distribution(-(int64_t{1} << bits), int64_t{1} << bits);
        return distribution(random);
    };
    // Random value below the volume bound: the product of two random numbers.
    constexpr int VolumeFactorBits = (CsgExactVolumeBits - 1) / 2;
    constexpr int MinorFactorBits = (CsgExactMinorBits - 2) / 2;
    const auto bigValue = [&uniform](int bits) { return CsgTest::Mul64<Int128>(uniform(bits), uniform(bits)); };

    int zeros = 0;
    int nonzeros = 0;
    for (int i = 0; i < 20000; ++i)
    {
        const bool edgeInSecond = (random() & 1) != 0;
        Int128 lowX, highX, lowY, highY;
        Int128 normalX[3], normalY[3];
        switch (i % 4)
        {
        case 0:
            // Random values at full range.
            lowX = bigValue(VolumeFactorBits);
            highX = bigValue(VolumeFactorBits);
            lowY = bigValue(VolumeFactorBits);
            highY = bigValue(VolumeFactorBits);
            for (int k = 0; k < 3; ++k)
            {
                normalX[k] = CsgTest::Mul64<Int128>(uniform(MinorFactorBits), uniform(MinorFactorBits));
                normalY[k] = CsgTest::Mul64<Int128>(uniform(MinorFactorBits), uniform(MinorFactorBits));
            }
            break;
        case 1:
            // Tiny values: many zero volumes, zero differences and equal ratios.
            lowX = Int128(uniform(2));
            highX = Int128(uniform(2));
            lowY = Int128(uniform(2));
            highY = Int128(uniform(2));
            for (int k = 0; k < 3; ++k)
            {
                normalX[k] = Int128(uniform(2));
                normalY[k] = Int128(uniform(2));
            }
            break;
        default:
        {
            // Equal ratios: both crossings are multiples of one base crossing. The normals may tie too.
            const int64_t baseLow = uniform(28);
            const int64_t baseHigh = uniform(28);
            const int64_t factorX = uniform(20);
            const int64_t factorY = uniform(20);
            lowX = CsgTest::Mul64<Int128>(baseLow, factorX);
            highX = CsgTest::Mul64<Int128>(baseHigh, factorX);
            lowY = CsgTest::Mul64<Int128>(baseLow, factorY);
            highY = CsgTest::Mul64<Int128>(baseHigh, factorY);
            for (int k = 0; k < 3; ++k)
            {
                const int64_t baseNormal = i % 4 == 2 ? uniform(3) : uniform(24);
                normalX[k] = CsgTest::Mul64<Int128>(baseNormal, factorX);
                normalY[k] = CsgTest::Mul64<Int128>(baseNormal, factorY);
                // Sometimes one component differs so that the tie-break moves to the next component.
                if ((random() % 8) == 0)
                    normalY[k] = normalY[k] + Int128(int64_t{1});
            }
            break;
        }
        }
        const Crossing x = MakeCrossing(lowX, highX, edgeInSecond, normalX[0], normalX[1], normalX[2]);
        const Crossing y = MakeCrossing(lowY, highY, edgeInSecond, normalY[0], normalY[1], normalY[2]);
        CsgTest::ReferenceCrossing referenceX{lowX, highX, {normalX[0], normalX[1], normalX[2]}, edgeInSecond};
        CsgTest::ReferenceCrossing referenceY{lowY, highY, {normalY[0], normalY[1], normalY[2]}, edgeInSecond};
        const int expected = CsgTest::ReferenceCompareOnEdge(referenceX, referenceY);
        REQUIRE(CompareOnEdge(x, y) == expected);
        REQUIRE(CompareOnEdge(y, x) == CsgTest::ReferenceCompareOnEdge(referenceY, referenceX));
        (expected == 0 ? zeros : nonzeros)++;
    }
    CHECK(zeros > 500);
    CHECK(nonzeros > 5000);
}

TEST_CASE("Corefine sorts more than 16 crossings of one edge", "[corefine]")
{
    // A long thin box passes through 12 plates. Each of its four long edges crosses 24 plate faces.
    const Matrix3x4 identity = Matrix3x4::IDENTITY;
    const CsgSolid rod(MakeBoxPolygons(Vector3::ZERO, Vector3(20.0f, 0.5f, 0.3f), 1));
    ea::vector<CsgPolygon> platePolygons;
    for (unsigned i = 0; i < 12; ++i)
    {
        const auto plate = MakeBoxPolygons(Vector3(-16.5f + 3.0f * static_cast<float>(i), 0.0f, 0.0f),
            Vector3(0.25f, 2.0f, 2.0f), 2 + i, identity);
        platePolygons.insert(platePolygons.end(), plate.begin(), plate.end());
    }
    const CsgSolid plates(platePolygons);

    Corefinement core(rod, plates, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    REQUIRE(core.crossings_.size() > 0);

    // The distinct edges of the rod that have crossings.
    ea::vector<ea::pair<int, int>> edges;
    for (const Crossing& crossing : core.crossings_)
    {
        const auto edge = ea::make_pair(crossing.key_.low_, crossing.key_.high_);
        if (!crossing.key_.edgeInSecond_ && ea::find(edges.begin(), edges.end(), edge) == edges.end())
            edges.push_back(edge);
    }
    unsigned longEdges = 0;
    ea::vector<int> forward;
    ea::vector<int> backward;
    ea::vector<int> again;
    for (const auto& edge : edges)
    {
        core.EdgeCrossings(core.a_, edge.first, edge.second, scratch, forward);
        core.EdgeCrossings(core.a_, edge.second, edge.first, scratch, backward);
        core.EdgeCrossings(core.a_, edge.first, edge.second, scratch, again);
        CHECK(forward == again);
        REQUIRE(forward.size() == backward.size());
        for (unsigned i = 0; i < forward.size(); ++i)
            CHECK(backward[i] == forward[forward.size() - 1 - i]);
        for (unsigned i = 1; i < forward.size(); ++i)
        {
            const Crossing& before = core.CrossingAt(forward[i - 1]);
            const Crossing& after = core.CrossingAt(forward[i]);
            CHECK(before.t_ <= after.t_);
        }
        if (forward.size() > 16)
            ++longEdges;
    }
    // The four long edges have 24 crossings each.
    CHECK(longEdges >= 4);
}

TEST_CASE("Corefine crossings have their rounded grid points", "[corefine]")
{
    const Matrix3x4 transform(Vector3::ZERO, Quaternion(23.0f, Vector3(1.0f, 2.0f, 3.0f).Normalized()), 1.0f);
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid a(MakeBoxPolygons(Vector3::ZERO, half, 1, transform));
    const CsgSolid b(MakeBoxPolygons(Vector3(0.7f, 0.4f, 0.2f), half, 2));
    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    REQUIRE(core.crossings_.size() > 0);
    const double step = 1.0 / CsgGridScale;
    for (const Crossing& crossing : core.crossings_)
    {
        // The grid point lies within one grid step of the float position.
        CHECK(std::fabs(crossing.grid_.x_ * step - crossing.position_.x_) <= step);
        CHECK(std::fabs(crossing.grid_.y_ * step - crossing.position_.y_) <= step);
        CHECK(std::fabs(crossing.grid_.z_ * step - crossing.position_.z_) <= step);
    }
}

TEST_CASE("Corefine handles crossings of one edge that round to one grid point", "[corefine]")
{
    // The wedge B has a flat bottom at z = 0 and a top that rises one step in each half unit along x.
    // At x = -0.75 the top is 1/4 step above the bottom. A vertical edge of A crosses both faces there.
    // Both crossings round to the same grid point.
    const float step = static_cast<float>(1.0 / CsgGridScale);
    const auto polygon = [](std::initializer_list<Vector3> corners)
    {
        ea::vector<ModelVertex> vertices;
        for (const Vector3& corner : corners)
        {
            ModelVertex vertex;
            vertex.SetPosition(corner);
            vertices.push_back(vertex);
        }
        return CsgPolygon(ea::move(vertices), 2);
    };
    ea::vector<CsgPolygon> wedge;
    wedge.push_back(polygon({{-1, -1, 0}, {-1, 1, 0}, {1, 1, 0}, {1, -1, 0}}));
    wedge.push_back(polygon({{1, -1, 0}, {1, 1, 0}, {1, 1, 2 * step}, {1, -1, 2 * step}}));
    wedge.push_back(polygon({{-1, -1, 0}, {1, -1, 2 * step}, {1, 1, 2 * step}, {-1, 1, 0}}));
    wedge.push_back(polygon({{-1, -1, 0}, {1, -1, 0}, {1, -1, 2 * step}}));
    wedge.push_back(polygon({{-1, 1, 0}, {1, 1, 2 * step}, {1, 1, 0}}));
    const CsgSolid b(wedge);
    const CsgSolid a(MakeBoxPolygons(Vector3(-0.125f, 0.0f, 0.0f), Vector3(0.625f, 0.25f, 1.0f), 1));

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);

    // Find the vertical edges of A at x = -0.75. Each one crosses the bottom and the top of the wedge.
    unsigned verticalEdges = 0;
    ea::vector<int> forward;
    ea::vector<int> backward;
    for (int low = 0; low < a.GetTriangleVertices().size() && low < core.a_.VertexCount(); ++low)
    {
        for (int high = low + 1; high < core.a_.VertexCount(); ++high)
        {
            const Vector3& p = core.a_.loops_.positions_[low];
            const Vector3& q = core.a_.loops_.positions_[high];
            if (p.x_ != -0.75f || q.x_ != -0.75f || p.y_ != q.y_ || std::fabs(p.y_) != 0.25f || p.z_ == q.z_)
                continue;
            core.EdgeCrossings(core.a_, low, high, scratch, forward);
            core.EdgeCrossings(core.a_, high, low, scratch, backward);
            // The two crossings of the edge keep two ids, but they have one grid point.
            REQUIRE(forward.size() == 2);
            REQUIRE(backward.size() == 2);
            CHECK(forward[0] != forward[1]);
            CHECK(forward[0] == backward[1]);
            CHECK(forward[1] == backward[0]);
            CHECK(core.CrossingAt(forward[0]).grid_ == core.CrossingAt(forward[1]).grid_);
            ++verticalEdges;
        }
    }
    CHECK(verticalEdges == 2);

    // No curve segment has the same id at both ends.
    const CsgCorefineTestResult result = CsgCorefineCrossingsForTest(a, b);
    for (const CsgCorefineTestSegment& segment : result.segments_)
        CHECK(segment.p_ != segment.q_);
    CHECK(Sum(result.segmentCountsA_) == Sum(result.segmentCountsB_));
}

TEST_CASE("Corefine moves the crossings of one edge with intersecting cells to one grid point", "[corefine]")
{
    // The wedge B has a flat bottom at z = 0 and a top that rises 3 steps in each unit along x.
    // At x = -0.75 the top is 3/4 step above the bottom. A vertical edge of A crosses both faces there.
    // The bottom crossing is on the grid at z = 0. The top crossing rounds to z = 1 step, but its cell holds z = 0.
    // The two cells intersect in the corner z = 0, so ConstrainCrossings puts both crossings at this corner.
    const float step = static_cast<float>(1.0 / CsgGridScale);
    const auto polygon = [](std::initializer_list<Vector3> corners)
    {
        ea::vector<ModelVertex> vertices;
        for (const Vector3& corner : corners)
        {
            ModelVertex vertex;
            vertex.SetPosition(corner);
            vertices.push_back(vertex);
        }
        return CsgPolygon(ea::move(vertices), 2);
    };
    ea::vector<CsgPolygon> wedge;
    wedge.push_back(polygon({{-1, -1, 0}, {-1, 1, 0}, {1, 1, 0}, {1, -1, 0}}));
    wedge.push_back(polygon({{1, -1, 0}, {1, 1, 0}, {1, 1, 6 * step}, {1, -1, 6 * step}}));
    wedge.push_back(polygon({{-1, -1, 0}, {1, -1, 6 * step}, {1, 1, 6 * step}, {-1, 1, 0}}));
    wedge.push_back(polygon({{-1, -1, 0}, {1, -1, 0}, {1, -1, 6 * step}}));
    wedge.push_back(polygon({{-1, 1, 0}, {1, 1, 6 * step}, {1, 1, 0}}));
    const CsgSolid b(wedge);
    const CsgSolid a(MakeBoxPolygons(Vector3(-0.125f, 0.0f, 0.0f), Vector3(0.625f, 0.25f, 1.0f), 1));

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    core.ConstrainCrossings(scratch);

    unsigned verticalEdges = 0;
    ea::vector<int> forward;
    for (int low = 0; low < core.a_.VertexCount(); ++low)
    {
        for (int high = low + 1; high < core.a_.VertexCount(); ++high)
        {
            const Vector3& p = core.a_.loops_.positions_[low];
            const Vector3& q = core.a_.loops_.positions_[high];
            if (p.x_ != -0.75f || q.x_ != -0.75f || p.y_ != q.y_ || std::fabs(p.y_) != 0.25f || p.z_ == q.z_)
                continue;
            core.EdgeCrossings(core.a_, low, high, scratch, forward);
            REQUIRE(forward.size() == 2);
            CHECK(forward[0] != forward[1]);
            CHECK(core.CrossingAt(forward[0]).grid_ == core.CrossingAt(forward[1]).grid_);
            // The common point is the corner at the bottom of the wedge.
            CHECK(core.CrossingAt(forward[0]).grid_.z_ == 0);
            ++verticalEdges;
        }
    }
    CHECK(verticalEdges == 2);

    // No curve segment has the same id at both ends.
    const CsgCorefineTestResult result = CsgCorefineCrossingsForTest(a, b);
    for (const CsgCorefineTestSegment& segment : result.segments_)
        CHECK(segment.p_ != segment.q_);
    CHECK(Sum(result.segmentCountsA_) == Sum(result.segmentCountsB_));
}

namespace
{

/// A point given in grid units. The position is grid / scale. This is exact, so the grid point is the same
/// in the float mode and in the double mode.
struct GridInts
{
    int x_;
    int y_;
    int z_;
};

Vector3 GridPosition(const GridInts& p)
{
    const float step = static_cast<float>(1.0 / CsgGridScale);
    return Vector3(static_cast<float>(p.x_) * step, static_cast<float>(p.y_) * step, static_cast<float>(p.z_) * step);
}

CsgGridPoint GridPointOf(const GridInts& p)
{
    CsgGridPoint result;
    result.x_ = p.x_;
    result.y_ = p.y_;
    result.z_ = p.z_;
    return result;
}

CsgPolygon GridPolygon(unsigned shared, std::initializer_list<GridInts> corners)
{
    ea::vector<ModelVertex> vertices;
    for (const GridInts& corner : corners)
    {
        ModelVertex vertex;
        vertex.SetPosition(GridPosition(corner));
        vertices.push_back(vertex);
    }
    return CsgPolygon(ea::move(vertices), shared);
}

/// A prism with a triangle above and below. The top triangle is counterclockwise seen from above.
ea::vector<CsgPolygon> MakeGridPrism(const GridInts (&top)[3], int zTop, int zBottom, unsigned shared)
{
    GridInts up[3];
    GridInts down[3];
    for (int i = 0; i < 3; ++i)
    {
        up[i] = GridInts{top[i].x_, top[i].y_, zTop};
        down[i] = GridInts{top[i].x_, top[i].y_, zBottom};
    }
    ea::vector<CsgPolygon> polygons;
    polygons.push_back(GridPolygon(shared, {up[0], up[1], up[2]}));
    polygons.push_back(GridPolygon(shared, {down[0], down[2], down[1]}));
    for (int i = 0; i < 3; ++i)
    {
        const int j = (i + 1) % 3;
        polygons.push_back(GridPolygon(shared, {down[i], down[j], up[j], up[i]}));
    }
    return polygons;
}

/// A pyramid with the apex above its base. The base is counterclockwise seen from above.
ea::vector<CsgPolygon> MakeGridPyramid(const ea::vector<GridInts>& base, const GridInts& apex, unsigned shared)
{
    ea::vector<CsgPolygon> polygons;
    for (unsigned i = 0; i < base.size(); ++i)
        polygons.push_back(GridPolygon(shared, {base[i], base[(i + 1) % base.size()], apex}));
    ea::vector<ModelVertex> vertices;
    for (unsigned i = base.size(); i > 0; --i)
    {
        ModelVertex vertex;
        vertex.SetPosition(GridPosition(base[i - 1]));
        vertices.push_back(vertex);
    }
    polygons.push_back(CsgPolygon(ea::move(vertices), shared));
    return polygons;
}

/// Find the crossing of the edge between two grid points of the second operand. The first operand is crossed.
const Crossing* FindEdgeCrossing(const Corefinement& core, const GridInts& end0, const GridInts& end1)
{
    const CsgGridPoint p0 = GridPointOf(end0);
    const CsgGridPoint p1 = GridPointOf(end1);
    for (const Crossing& crossing : core.crossings_)
    {
        if (!crossing.key_.edgeInSecond_)
            continue;
        const CsgGridPoint& low = core.b_.grid_[crossing.key_.low_];
        const CsgGridPoint& high = core.b_.grid_[crossing.key_.high_];
        if ((low == p0 && high == p1) || (low == p1 && high == p0))
            return &crossing;
    }
    return nullptr;
}

} // namespace

TEST_CASE("Mesh finds the triangles of each edge", "[corefine][adjacency]")
{
    // A box is a closed surface. Each edge has two different triangles that both have the edge.
    const CsgSolid box(MakeBoxPolygons(Vector3::ZERO, Vector3(1.0f, 1.0f, 1.0f), 1));
    Mesh mesh(box, false, 0);
    CHECK(!mesh.HasEdgeTriangles());
    mesh.BuildEdgeTriangles();
    CHECK(mesh.HasEdgeTriangles());
    // A second call changes nothing.
    mesh.BuildEdgeTriangles();
    const int triangleCount = static_cast<int>(mesh.triangleFace_.size());
    REQUIRE(triangleCount == 12);
    // 12 box edges and 6 diagonals.
    CHECK(mesh.edgeTriangles_.Size() == 18);
    const auto hasEdge = [&](int triangle, int u, int v)
    {
        for (int corner = 0; corner < 3; ++corner)
        {
            const int a = mesh.triangleIds_[triangle * 3 + corner];
            const int b = mesh.triangleIds_[triangle * 3 + (corner + 1) % 3];
            if ((a == u && b == v) || (a == v && b == u))
                return true;
        }
        return false;
    };
    for (int triangle = 0; triangle < triangleCount; ++triangle)
    {
        for (int corner = 0; corner < 3; ++corner)
        {
            const int u = mesh.triangleIds_[triangle * 3 + corner];
            const int v = mesh.triangleIds_[triangle * 3 + (corner + 1) % 3];
            const CsgEdgeTriangles* pair = mesh.FindEdgeTriangles(u, v);
            REQUIRE(pair != nullptr);
            CHECK(mesh.FindEdgeTriangles(v, u) == pair);
            REQUIRE(pair->first_ >= 0);
            REQUIRE(pair->second_ >= 0);
            CHECK(pair->first_ != pair->second_);
            CHECK(hasEdge(pair->first_, u, v));
            CHECK(hasEdge(pair->second_, u, v));
            CHECK((pair->first_ == triangle || pair->second_ == triangle));
        }
    }
    // Two ids that share no edge have no entry.
    CHECK(mesh.FindEdgeTriangles(0, 0) == nullptr);
}

TEST_CASE("Mesh finds one triangle on a boundary edge", "[corefine][adjacency]")
{
    // An open quad has two triangles. The diagonal has both. The four sides of the quad have one each.
    ea::vector<CsgPolygon> polygons;
    polygons.push_back(GridPolygon(1, {{0, 0, 0}, {100, 0, 0}, {100, 100, 0}, {0, 100, 0}}));
    const CsgSolid quad(polygons, 0.0f);
    Mesh mesh(quad, false, 0);
    mesh.BuildEdgeTriangles();
    REQUIRE(mesh.triangleFace_.size() == 2);
    CHECK(mesh.edgeTriangles_.Size() == 5);
    int shared = 0;
    int boundary = 0;
    for (const auto& entry : mesh.edgeTriangles_)
    {
        REQUIRE(entry.value_.first_ >= 0);
        if (entry.value_.second_ >= 0)
            ++shared;
        else
            ++boundary;
    }
    CHECK(shared == 1);
    CHECK(boundary == 4);
}

TEST_CASE("Crossing validity: a rounded crossing outside its triangle is invalid", "[corefine][validity]")
{
    // A is a prism. Its top triangle has the slanted side 3x + 5y = 3000 (grid units) at the height 100.
    // B is a needle. Its edge from (448, 328, 0) to (450, 336, 300) crosses the top at (448.67, 330.67, 100).
    // The exact point has 3x + 5y = 2999.33, so it is inside. The rounded point (449, 331) has 3002. It is
    // 0.34 grid steps outside of the side.
    const GridInts top[3] = {{0, 0, 0}, {1000, 0, 0}, {0, 600, 0}};
    const CsgSolid a(MakeGridPrism(top, 100, -100, 1), 0.0f);
    const GridInts low{448, 328, 0};
    const GridInts apex{450, 336, 300};
    const CsgSolid b(MakeGridPyramid({low, {430, 330, 0}, {450, 310, 0}}, apex, 2), 0.0f);

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    core.a_.BuildEdgeTriangles();
    core.b_.BuildEdgeTriangles();
    const Crossing* crossing = FindEdgeCrossing(core, low, apex);
    REQUIRE(crossing != nullptr);

    // The rounded crossing is the outside point.
    const CsgGridPoint rounded = GridPointOf({449, 331, 100});
    REQUIRE(crossing->grid_ == rounded);
    CHECK(3 * rounded.x_ + 5 * rounded.y_ == 3002);
    CHECK(!core.CrossingIsValid(*crossing, rounded));
    CHECK(!core.CrossingIsValid(*crossing, rounded, scratch));
    CHECK((core.CrossingInvalidRules(*crossing, rounded, scratch) & CsgRuleInside) != 0);

    // The corners of the grid cell of the exact point, in the plane of the top. The nearest corner that is inside
    // of the side is valid.
    const double exactX = 448.0 + 2.0 / 3.0;
    const double exactY = 328.0 + 8.0 / 3.0;
    double bestDistance = 1e30;
    CsgGridPoint best;
    for (int dx = 0; dx < 2; ++dx)
    {
        for (int dy = 0; dy < 2; ++dy)
        {
            const int x = 448 + dx;
            const int y = 330 + dy;
            const double distance = (x - exactX) * (x - exactX) + (y - exactY) * (y - exactY);
            if (3 * x + 5 * y < 3000 && distance < bestDistance)
            {
                bestDistance = distance;
                best = GridPointOf({x, y, 100});
            }
        }
    }
    REQUIRE(bestDistance < 1e29);
    CHECK(core.CrossingInvalidRules(*crossing, best, scratch) == 0);
    CHECK(core.CrossingIsValid(*crossing, best, scratch));
    CHECK(core.CrossingIsValid(*crossing, best));
    // A point on the side is not strictly inside.
    CHECK((core.CrossingInvalidRules(*crossing, GridPointOf({500, 300, 100}), scratch) & CsgRuleInside) != 0);
    // The corner of the top triangle is not allowed.
    CHECK((core.CrossingInvalidRules(*crossing, GridPointOf({0, 0, 100}), scratch) & CsgRuleCorner) != 0);
}

TEST_CASE("Crossing validity: a valid crossing, and a move that crosses segments", "[corefine][validity]")
{
    // B is a square pyramid with the apex above the top of A. Its four edges to the base cross the top at the four
    // corners of a square, and the four sides of the pyramid make the four segments of the square.
    const GridInts top[3] = {{0, 0, 0}, {1000, 0, 0}, {0, 600, 0}};
    const CsgSolid a(MakeGridPrism(top, 100, -100, 1), 0.0f);
    const GridInts apex{200, 150, 300};
    const ea::vector<GridInts> base = {{170, 120, 0}, {230, 120, 0}, {230, 180, 0}, {170, 180, 0}};
    const CsgSolid b(MakeGridPyramid(base, apex, 2), 0.0f);

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    core.a_.BuildEdgeTriangles();
    core.b_.BuildEdgeTriangles();
    REQUIRE(core.crossings_.size() == 4);
    CHECK(core.segmentsA_.TotalCount() == 4);

    // The crossing of the edge from base[0] to the apex is at (180, 130, 100). It is valid.
    const Crossing* crossing = FindEdgeCrossing(core, base[0], apex);
    REQUIRE(crossing != nullptr);
    REQUIRE(crossing->grid_ == GridPointOf({180, 130, 100}));
    CHECK(core.CrossingInvalidRules(*crossing, crossing->grid_, scratch) == 0);
    CHECK(core.CrossingIsValid(*crossing, crossing->grid_));
    // A near point is valid too.
    CHECK(core.CrossingIsValid(*crossing, GridPointOf({181, 130, 100}), scratch));
    CHECK(core.CrossingIsValid(*crossing, GridPointOf({180, 131, 100}), scratch));
    // The counter agrees: all four crossings are valid.
    int perRule[4];
    CHECK(core.CountInvalidCrossings(scratch, perRule) == 0);

    // Move the crossing to the right of the square, at (230, 150). The two segments at the crossing
    // then cross the opposite side of the square at x = 220.
    const unsigned rules = core.CrossingInvalidRules(*crossing, GridPointOf({230, 150, 100}), scratch);
    CHECK((rules & CsgRuleSegments) != 0);
    CHECK(!core.CrossingIsValid(*crossing, GridPointOf({230, 150, 100}), scratch));
    // The point is inside the top triangle, so the other rules hold.
    CHECK((rules & CsgRuleInside) == 0);
    CHECK((rules & CsgRuleCorner) == 0);
    // The move to a point beyond the square along the edge of the pyramid does not cross a segment.
    CHECK(core.CrossingIsValid(*crossing, GridPointOf({179, 129, 100}), scratch));
}

TEST_CASE("Constrain crossings: a crossing that rounds outside its triangle moves inside", "[corefine][constrain]")
{
    // The same needle and prism as in the validity test. The rounded crossing (449, 331, 100) is outside the side
    // 3x + 5y = 3000 of the top triangle. The exact crossing is (448.67, 330.67, 100).
    const GridInts top[3] = {{0, 0, 0}, {1000, 0, 0}, {0, 600, 0}};
    const CsgSolid a(MakeGridPrism(top, 100, -100, 1), 0.0f);
    const GridInts low{448, 328, 0};
    const GridInts apex{450, 336, 300};
    const CsgSolid b(MakeGridPyramid({low, {430, 330, 0}, {450, 310, 0}}, apex, 2), 0.0f);

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    const Crossing* crossing = FindEdgeCrossing(core, low, apex);
    REQUIRE(crossing != nullptr);
    REQUIRE(crossing->grid_ == GridPointOf({449, 331, 100}));

    // The cell of the exact crossing: x in [448, 449], y in [330, 331], z exactly 100.
    CsgGridPoint lower;
    CsgGridPoint upper;
    const CsgGridPoint lowEnd = core.b_.grid_[crossing->key_.low_];
    const CsgGridPoint highEnd = core.b_.grid_[crossing->key_.high_];
    CsgExact::CrossingCell(lowEnd, highEnd, crossing->lowVolume_, crossing->difference_, crossing->grid_, lower, upper);
    CHECK(lower == GridPointOf({448, 330, 100}));
    CHECK(upper == GridPointOf({449, 331, 100}));

    g_csgDiagnostics.Reset();
    core.ConstrainCrossings(scratch);
    CHECK(g_csgDiagnostics.constrainedCrossings_.load() == 1);
    CHECK(g_csgDiagnostics.unconstrainableCrossings_.load() == 0);

    // The first flipped corner is valid: the x axis flips and the point (448, 331, 100) has 3x + 5y = 2999.
    CHECK(crossing->grid_ == GridPointOf({448, 331, 100}));
    CHECK(crossing->position_ == crossing->grid_.ToPosition());
    CHECK(core.CrossingIsValid(*crossing, crossing->grid_, scratch));
    int perRule[4];
    CHECK(core.CountInvalidCrossings(scratch, perRule, false) == 0);
}

TEST_CASE("Constrain crossings: a valid crossing does not move", "[corefine][constrain]")
{
    // The square pyramid of the validity test: all four crossings are valid.
    const GridInts top[3] = {{0, 0, 0}, {1000, 0, 0}, {0, 600, 0}};
    const CsgSolid a(MakeGridPrism(top, 100, -100, 1), 0.0f);
    const GridInts apex{200, 150, 300};
    const ea::vector<GridInts> base = {{170, 120, 0}, {230, 120, 0}, {230, 180, 0}, {170, 180, 0}};
    const CsgSolid b(MakeGridPyramid(base, apex, 2), 0.0f);

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    REQUIRE(core.crossings_.size() == 4);
    const ea::vector<Crossing> before = core.crossings_;

    g_csgDiagnostics.Reset();
    core.ConstrainCrossings(scratch);
    CHECK(g_csgDiagnostics.constrainedCrossings_.load() == 0);
    CHECK(g_csgDiagnostics.unconstrainableCrossings_.load() == 0);
    for (size_t i = 0; i < before.size(); ++i)
    {
        CHECK(core.crossings_[i].grid_ == before[i].grid_);
        CHECK(core.crossings_[i].position_ == before[i].position_);
    }
}

TEST_CASE("Crossing validity: a closed cycle of segments that turns over is invalid", "[corefine][validity]")
{
    // B is a triangular pyramid with the apex above the top of A. Its three edges to the base cross the top at the
    // grid points (180, 130), (220, 130) and (200, 170). The three segments make a closed cycle inside the top.
    const GridInts top[3] = {{0, 0, 0}, {1000, 0, 0}, {0, 600, 0}};
    const CsgSolid a(MakeGridPrism(top, 100, -100, 1), 0.0f);
    const GridInts apex{200, 150, 300};
    const ea::vector<GridInts> base = {{170, 120, 0}, {230, 120, 0}, {200, 180, 0}};
    const CsgSolid b(MakeGridPyramid(base, apex, 2), 0.0f);

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    core.a_.BuildEdgeTriangles();
    core.b_.BuildEdgeTriangles();
    REQUIRE(core.crossings_.size() == 3);
    const Crossing* crossing = FindEdgeCrossing(core, base[0], apex);
    REQUIRE(crossing != nullptr);
    REQUIRE(crossing->grid_ == GridPointOf({180, 130, 100}));
    CHECK(core.CrossingInvalidRules(*crossing, crossing->grid_, scratch) == 0);

    // Move the crossing to the other side of the line through the two other crossings. The rounded cycle turns
    // over, but no segment crosses another one: the three segments still make a triangle.
    const CsgGridPoint over = GridPointOf({230, 170, 100});
    const unsigned rules = core.CrossingInvalidRules(*crossing, over, scratch);
    CHECK((rules & CsgRuleCycle) != 0);
    CHECK((rules & CsgRuleInside) == 0);
    CHECK((core.CrossingInvalidRules(*crossing, over, scratch, false, false) & CsgRuleCycle) != 0);
    // A move that keeps the orientation does not break the rule.
    CHECK((core.CrossingInvalidRules(*crossing, GridPointOf({181, 131, 100}), scratch) & CsgRuleCycle) == 0);
    // A move onto the line of the two other crossings gives a cycle of no area. It breaks both forms of the rule.
    const CsgGridPoint onLine = GridPointOf({240, 90, 100});
    CHECK((core.CrossingInvalidRules(*crossing, onLine, scratch) & CsgRuleCycle) != 0);
    CHECK((core.CrossingInvalidRules(*crossing, onLine, scratch, false, false) & CsgRuleCycle) != 0);
}

TEST_CASE("Constrain crossings: a small cycle that turns over when rounded keeps its orientation", "[corefine][constrain]")
{
    // B is a flat pyramid. Its apex is 1 grid step above the top of A, so its three edges cross the top at 1/101 of
    // their length from the apex. The exact crossings (299.36, 198.55), (301.76, 201.50) and (298.02, 198.47) make
    // a counterclockwise cycle. The nearest roundings (299, 199), (302, 201) and (298, 198) make a clockwise cycle.
    const GridInts top[3] = {{0, 0, 0}, {2000, 0, 0}, {0, 1200, 0}};
    const CsgSolid a(MakeGridPrism(top, 100, -100, 1), 0.0f);
    const GridInts apex{300, 200, 101};
    const ea::vector<GridInts> base = {{235, 54, 0}, {478, 351, 0}, {100, 45, 0}};
    const CsgSolid b(MakeGridPyramid(base, apex, 2), 0.0f);

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    REQUIRE(core.crossings_.size() == 3);
    const Crossing* c0 = FindEdgeCrossing(core, base[0], apex);
    const Crossing* c1 = FindEdgeCrossing(core, base[1], apex);
    const Crossing* c2 = FindEdgeCrossing(core, base[2], apex);
    REQUIRE((c0 && c1 && c2));
    REQUIRE(c0->grid_ == GridPointOf({299, 199, 100}));
    REQUIRE(c1->grid_ == GridPointOf({302, 201, 100}));
    REQUIRE(c2->grid_ == GridPointOf({298, 198, 100}));
    const auto orientation = [&]()
    {
        const auto project = [](const CsgGridPoint& p) { return CsgExact::Project(p, 2, false); };
        return CsgExact::Orient2(project(c0->grid_), project(c1->grid_), project(c2->grid_));
    };
    REQUIRE(orientation() < 0);
    core.a_.BuildEdgeTriangles();
    core.b_.BuildEdgeTriangles();
    // The nearest rounding has no fold of the other rules. Only the cycle rule finds it.
    int perRule[4];
    CHECK(core.CountInvalidCrossings(scratch, perRule, false) == 3);
    CHECK(perRule[0] + perRule[1] + perRule[2] == 0);

    g_csgDiagnostics.Reset();
    core.ConstrainCrossings(scratch);
    CHECK(g_csgDiagnostics.constrainedCrossings_.load() >= 1);
    CHECK(g_csgDiagnostics.unconstrainableCrossings_.load() == 0);
    CHECK(orientation() > 0);
    CHECK(core.CountInvalidCrossings(scratch, perRule, false) == 0);
    // Each crossing stays in the cell of its exact point.
    for (const Crossing* crossing : {c0, c1, c2})
    {
        CsgGridPoint lower;
        CsgGridPoint upper;
        CsgGridPoint nearest;
        const CsgGridPoint& lowEnd = core.b_.grid_[crossing->key_.low_];
        const CsgGridPoint& highEnd = core.b_.grid_[crossing->key_.high_];
        nearest = CsgExact::RoundCrossing(lowEnd, highEnd, crossing->lowVolume_, crossing->difference_);
        CsgExact::CrossingCell(lowEnd, highEnd, crossing->lowVolume_, crossing->difference_, nearest, lower, upper);
        CHECK((crossing->grid_.x_ >= lower.x_ && crossing->grid_.x_ <= upper.x_));
        CHECK((crossing->grid_.y_ >= lower.y_ && crossing->grid_.y_ <= upper.y_));
        CHECK((crossing->grid_.z_ >= lower.z_ && crossing->grid_.z_ <= upper.z_));
    }
}
