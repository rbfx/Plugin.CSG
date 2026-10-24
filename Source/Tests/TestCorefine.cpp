// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"

#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <EASTL/algorithm.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

constexpr float Tolerance = 1e-5f;

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
        || a.segmentCountsB_ != b.segmentCountsB_ || a.curveEdges_ != b.curveEdges_ || a.outOfRange_ != b.outOfRange_)
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
    CHECK(result.positions_.size() == 18);
    // An independent count with a small numeric displacement gives the same number.
    const double none[3] = {0.0, 0.0, 0.0};
    const double shift[3] = {1e-3, 1e-6, 1e-9};
    const Vector3 shiftedCenter(1.0f, 0.0f, 0.0f);
    CHECK(CountEdgeCrossings(a, MakeAxisBox(shiftedCenter, half, shift), none)
            + CountEdgeCrossings(b, MakeAxisBox(Vector3::ZERO, half, none), shift)
        == 18);
    CHECK_FALSE(result.outOfRange_);
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
    const CsgSolid sphere(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.3f), 1.3f, 8, 12, 1));
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
    CHECK(SamePositionSets(result.positions_, swapped.positions_));
}

TEST_CASE("Corefine flags coordinates outside of the exact range", "[corefine]")
{
    const Vector3 half(2.0f, 2.0f, 2.0f);
    const CsgSolid unit(MakeBoxPolygons(Vector3::ZERO, Vector3(1.0f, 1.0f, 1.0f), 1));

    SECTION("Disjoint")
    {
        const CsgSolid far(MakeBoxPolygons(Vector3(70000.0f, 0.0f, 0.0f), half, 2));
        const auto result = CsgCorefineCrossingsForTest(unit, far);
        CHECK(result.outOfRange_);
        CHECK(result.positions_.size() == 0);
    }
    SECTION("Both operands far away")
    {
        const CsgSolid farA(MakeBoxPolygons(Vector3(70000.0f, 0.0f, 0.0f), half, 1));
        const CsgSolid farB(MakeBoxPolygons(Vector3(70001.0f, 0.5f, 0.25f), half, 2));
        const auto result = CsgCorefineCrossingsForTest(farA, farB);
        CHECK(result.outOfRange_);
        CHECK(result.positions_.size() < 10000);
    }
    SECTION("In range")
    {
        const CsgSolid near(MakeBoxPolygons(Vector3(0.5f, 0.0f, 0.0f), half, 2));
        CHECK_FALSE(CsgCorefineCrossingsForTest(unit, near).outOfRange_);
    }
}

TEST_CASE("Corefine survives a NaN corner", "[corefine]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Vector3 half(1.0f, 1.0f, 1.0f);
    const CsgSolid clean(MakeBoxPolygons(Vector3(0.5f, 0.25f, 0.125f), half, 1));

    auto polygons = MakeBoxPolygons(Vector3::ZERO, half, 2);
    polygons[0].vertices_[0].SetPosition(Vector3(nan, 0.0f, nan));
    const CsgSolid broken(polygons);

    const auto first = CsgCorefineCrossingsForTest(clean, broken);
    const auto second = CsgCorefineCrossingsForTest(broken, clean);
    CHECK(first.positions_.size() < 10000);
    CHECK(second.positions_.size() < 10000);
    CHECK(SameResult(first, CsgCorefineCrossingsForTest(clean, broken)));
}

namespace
{

Crossing MakeCrossing(int low, int high, bool edgeInSecond, const CsgInt128& normalX = 0,
    const CsgInt128& normalY = 0, const CsgInt128& normalZ = 0)
{
    Crossing crossing;
    crossing.lowVolume_ = low;
    crossing.highVolume_ = high;
    crossing.key_.edgeInSecond_ = edgeInSecond;
    crossing.normal_ = {normalX, normalY, normalZ};
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
        const CsgInt128 big = CsgMul64<CsgInt128>(int64_t{1} << 50, int64_t{1} << 50); // 2^100
        Crossing x;
        x.lowVolume_ = big;
        x.highVolume_ = CsgInt128(0) - big;
        Crossing y = MakeCrossing(1, -1, false);
        // Both parameters are 0.5 exactly.
        CHECK(CompareOnEdge(x, y) == 0);
        // A difference of one part in 2^101 needs the full precision of the products.
        x.lowVolume_ = big + CsgInt128(1);
        CHECK(CompareOnEdge(x, y) == 1);
        CHECK(CompareOnEdge(y, x) == -1);
    }
}

TEST_CASE("Corefine converts a portable 128 bit number with one rounding", "[corefine]")
{
    // 2^117 + 2^64 + 1: the part below the rounding position is more than half of one step, so it rounds up.
    // Rounding in three steps would give 2^117.
    const CsgInt128Portable value = CsgInt128Portable::FromParts(1, (int64_t{1} << 53) + 1);
    CHECK(CsgInt128PortableToDouble(value) == std::ldexp(1.0, 117) + std::ldexp(1.0, 65));
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable() - value) == -(std::ldexp(1.0, 117) + std::ldexp(1.0, 65)));
    // The exact half rounds to even.
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable::FromParts(0, (int64_t{1} << 53) + 1)) == std::ldexp(1.0, 117));
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable::FromParts(0, (int64_t{1} << 53) + 3))
        == std::ldexp(1.0, 117) + std::ldexp(1.0, 66));
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable()) == 0.0);
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable(-5)) == -5.0);
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable(int64_t{1} << 62)) == std::ldexp(1.0, 62));
    // The most negative number.
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable::FromParts(0, INT64_MIN)) == -std::ldexp(1.0, 127));
    // The highest bit of the high limb.
    CHECK(CsgInt128PortableToDouble(CsgInt128Portable::FromParts(~uint64_t{0}, INT64_MAX))
        == std::ldexp(1.0, 127));
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
