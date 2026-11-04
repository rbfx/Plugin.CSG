// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include "Urho3D/CSG/CsgExpansion.h"
#include <Urho3D/Core/AssertBase.h>
#include <Urho3D/Math/Vector3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace Urho3D
{

/// Coordinates must stay below 2^CsgGridCoordinateBits units.
constexpr int CsgGridCoordinateBits = 13;

/// One unit has 2^CsgGridScaleBits grid steps.
/// Float mode: the step is 2^-10 units. Grid coordinates stay below 2^23 and differences below 2^24.
/// Double mode: the step is 2^-16 units. Grid coordinates stay below 2^29 and differences below 2^30.
/// In float mode, the grid keeps every float coordinate with an absolute value of 2^-3 or more without a change.
/// In double mode, a float coordinate snaps to the grid of 2^-16 units.
/// In both modes, a difference is exact in CsgReal.
#if defined(CSG_EXACT_USE_DOUBLE)
constexpr int CsgGridScaleBits = 16;
#else
constexpr int CsgGridScaleBits = 10;
#endif

/// Bits of a grid coordinate: the absolute value of a grid coordinate is below 2^CsgGridBits.
constexpr int CsgGridBits = CsgGridCoordinateBits + CsgGridScaleBits;
static_assert(CsgGridBits + 1 <= std::numeric_limits<CsgReal>::digits, "Grid differences must be exact in CsgReal.");
static_assert(CsgGridBits < 31, "A grid coordinate must fit in 32 bits.");

/// A minor of differences has a magnitude below 2^CsgExactMinorBits: 2^50 in float mode, 2^62 in double mode.
constexpr int CsgExactMinorBits = 2 * CsgGridBits + 4;
/// A determinant of differences has a magnitude below 2^CsgExactVolumeBits.
/// It is 2^75 in float mode and 2^93 in double mode.
constexpr int CsgExactVolumeBits = 3 * CsgGridBits + 6;
static_assert(CsgExactVolumeBits <= CsgProductDifferenceInputBits, "Volumes must be valid product difference inputs.");

/// A product of a grid difference, a factor 2 and a volume has a magnitude below 2^CsgExactRoundingBits.
/// The rounding of a point on an edge compares such products. It is 2^100 in float mode and 2^124 in double mode.
constexpr int CsgExactRoundingBits = CsgGridBits + 2 + CsgExactVolumeBits;
/// The sum of two products of grid differences has a magnitude below 2^CsgExactPlanarBits.
constexpr int CsgExactPlanarBits = 2 * CsgGridBits + 3;
/// A sum of up to 2^8 planar values has a magnitude below 2^CsgExactPlanarSumBits.
constexpr int CsgExactPlanarSumBits = CsgExactPlanarBits + 8;
static_assert(CsgGridBits + 2 <= CsgProductDifferenceInputBits, "Doubled differences must be valid inputs.");
static_assert(CsgExactRoundingBits <= 126, "A rounding product must fit in the 128 bit oracle.");
static_assert(CsgExactPlanarSumBits <= CsgProductDifferenceInputBits, "Planar sums must be valid inputs.");

/// Scale of the integer grid for the exact tests: 2^CsgGridScaleBits grid steps in one unit.
constexpr double CsgGridScale = static_cast<double>(int64_t{1} << CsgGridScaleBits);

/// Grid coordinates have an absolute value below this limit.
constexpr int64_t CsgGridIntegerLimit = int64_t{1} << CsgGridBits;

/// Bits that count the fan triangles of a loop in the exact area. A loop has at most 2^CsgLoopAreaPointBits + 2 points.
constexpr int CsgLoopAreaPointBits = 20;
/// Largest number of points of a loop that the exact area accepts. A longer loop gets no exact area.
constexpr int CsgLoopAreaMaxPoints = (1 << CsgLoopAreaPointBits) + 2;
/// Capacity of the expansion of a loop area.
/// Each term of the sum is an integer with a magnitude below 2^CsgExactPlanarBits. The sum has at most
/// CsgLoopAreaMaxPoints - 2 = 2^CsgLoopAreaPointBits terms, so each partial sum is an integer with a magnitude
/// below 2^(CsgExactPlanarBits + CsgLoopAreaPointBits). A compressed expansion is nonoverlapping.
/// Compress keeps the largest component within one ulp of the sum, and the smaller components add up to less than
/// that ulp. All components are integers, so the count is at most the bit count of the sum.
/// The limit is 69 in float mode and 81 in double mode.
constexpr int CsgLoopAreaMaxTerms = CsgExactPlanarBits + CsgLoopAreaPointBits;
static_assert(CsgLoopAreaMaxPoints - 2 == (1 << CsgLoopAreaPointBits), "The bound uses the fan triangle count.");

/// Coordinates with an absolute value of this limit or more are outside the range of the grid.
constexpr float CsgGridCoordinateLimit = static_cast<float>(int64_t{1} << CsgGridCoordinateBits);

/// Integer type of a grid coordinate.
using CsgGridInt = int32_t;

/// Point on the integer grid.
struct CsgGridPoint
{
    CsgGridInt x_{};
    CsgGridInt y_{};
    CsgGridInt z_{};

    /// Snap a point to the grid. The grid coordinate is floor(x * scale + 1/2).
    /// A value exactly between two grid points goes toward plus infinity.
    /// The scaled value and the sum with 1/2 are exact in double for a float input.
    /// Each coordinate must be finite and must have an absolute value less than 2^13.
    static CsgGridPoint From(const Vector3& point)
    {
        URHO3D_ASSERT(std::isfinite(point.x_) && std::isfinite(point.y_) && std::isfinite(point.z_));
        URHO3D_ASSERT(std::fabs(point.x_) < CsgGridCoordinateLimit && std::fabs(point.y_) < CsgGridCoordinateLimit
            && std::fabs(point.z_) < CsgGridCoordinateLimit);
        CsgGridPoint result;
        result.x_ = Round(point.x_);
        result.y_ = Round(point.y_);
        result.z_ = Round(point.z_);
        return result;
    }

    /// The grid coordinate floor(x * scale + 1/2) of a coordinate.
    static CsgGridInt Round(float x)
    {
        return static_cast<CsgGridInt>(std::floor(static_cast<double>(x) * CsgGridScale + 0.5));
    }

    /// Return the float position of the grid point: each coordinate is grid / scale, rounded to float.
    /// The position is exact in float mode. In double mode it is exact below 256 units.
    /// Above 256 units in double mode, the float can differ from the grid point by one float ulp.
    /// Then the grid point is the truth.
    Vector3 ToPosition() const
    {
        constexpr double step = 1.0 / CsgGridScale;
        return Vector3(static_cast<float>(x_ * step), static_cast<float>(y_ * step), static_cast<float>(z_ * step));
    }

    friend bool operator==(const CsgGridPoint& a, const CsgGridPoint& b)
    {
        return a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_;
    }
    friend bool operator!=(const CsgGridPoint& a, const CsgGridPoint& b) { return !(a == b); }
};

/// Point on the plane of the integer grid. The same type holds a difference of two grid points.
/// A coordinate has an absolute value below 2^CsgGridBits. A difference has an absolute value below 2^(CsgGridBits + 1).
struct CsgGridPoint2
{
    CsgGridInt u_{};
    CsgGridInt v_{};

    friend bool operator==(const CsgGridPoint2& a, const CsgGridPoint2& b) { return a.u_ == b.u_ && a.v_ == b.v_; }
    friend bool operator!=(const CsgGridPoint2& a, const CsgGridPoint2& b) { return !(a == b); }
};

/// Differences of two grid coordinates have an absolute value below 2^(CsgGridBits + 1). They fit in CsgGridInt.
static_assert(CsgGridBits + 1 <= 31, "A grid difference must fit in 32 bits.");

/// Unit roundoff of double: half the epsilon, 2^-53.
constexpr double CsgDoubleRoundoff = 0.5 * std::numeric_limits<double>::epsilon();
/// Bound of a product of two grid differences: 2^(2 * CsgGridBits + 2).
static_assert(2 * CsgGridBits + 2 < 63, "A product of grid differences must fit in 64 bits.");
constexpr double CsgPlanarProductBound = static_cast<double>(uint64_t{1} << (2 * CsgGridBits + 2));

/// Whether the double result of a planar determinant of differences is exact.
/// A planar determinant is a cross product or a dot product of two difference vectors.
/// Its magnitude is below 2^CsgExactPlanarBits. Every product and every partial sum is below this bound.
/// Float mode: 2^49 is below 2^53, so the double result is exact.
constexpr bool CsgPlanarExactInDouble = CsgExactPlanarBits <= std::numeric_limits<double>::digits;
/// Error bound of the double result of a planar determinant a * b - c * d, with M = 2^(2 * CsgGridBits + 2).
/// Each product has a relative error of at most u, so an absolute error below u * M.
/// The difference of the two products has a magnitude of at most 2 * M * (1 + u) and one more relative error u.
/// The total error is below (4 + 2u) * u * M. The factor 5 covers it, also with a fused multiply-add.
/// If the magnitude of the double result is above the bound, the sign of the double result is the exact sign.
/// The bound is 0 when the double result is exact.
constexpr double CsgPlanarFilterBound = CsgPlanarExactInDouble ? 0.0 : 5.0 * CsgDoubleRoundoff * CsgPlanarProductBound;
static_assert(CsgPlanarExactInDouble || CsgPlanarFilterBound > 0.0, "An inexact double result needs a filter bound.");
static_assert(CsgPlanarFilterBound < 0.01 * CsgPlanarProductBound, "The filter bound must be small.");

/// Whether the double result of a squared distance difference is exact. Its magnitude is below 2^CsgExactMinorBits.
constexpr bool CsgDistanceExactInDouble = CsgExactMinorBits <= std::numeric_limits<double>::digits;
/// Error bound of the double result of (a * a + b * b) - (c * c + d * d).
/// The four squares add an error below 4 * u * M. The two sums have magnitudes of at most 2 * M * (1 + u).
/// They add an error below 4 * (1 + u) * u * M. The last difference adds an error below 2 * (1 + 2u) * u * M.
/// The total error is below 10.01 * u * M. The factor 12 covers it.
constexpr double CsgDistanceFilterBound =
    CsgDistanceExactInDouble ? 0.0 : 12.0 * CsgDoubleRoundoff * CsgPlanarProductBound;
static_assert(CsgDistanceExactInDouble || CsgDistanceFilterBound > 0.0, "An inexact double result needs a filter bound.");
static_assert(CsgDistanceFilterBound < 0.01 * CsgPlanarProductBound, "The filter bound must be small.");

/// The loop area filter works for loops with up to this number of edges. A bigger loop uses the exact sum.
constexpr int CsgLoopFilterMaxEdges = 1 << (CsgExactPlanarSumBits - CsgExactPlanarBits);
/// Error of one cross term in the double sum of a loop area. It is 0 when the double cross term is exact.
constexpr double CsgLoopTermError = CsgPlanarFilterBound;
/// Error unit of the summation of the cross terms of a loop area, for n terms and a bound T of one term.
/// The error of a recursive sum of n terms is below (n - 1) * u * 1.01 * (sum of the magnitudes).
/// The sum of the magnitudes is below n * T with T = 2 * M. The error is below n * (n - 1) * CsgLoopSumUnit.
constexpr double CsgLoopSumUnit = 1.01 * CsgDoubleRoundoff * 2.0 * CsgPlanarProductBound;

/// Exact vector of three minors. Each component has a magnitude below 2^CsgExactMinorBits.
struct CsgNormal
{
    CsgExactMinor x_;
    CsgExactMinor y_;
    CsgExactMinor z_;
};

/// Exact orientation tests with the symbolic displacement of the second operand.
/// The functions use float expansions.
class PLUGIN_RBFXCSG_API CsgExact
{
public:
    /// Sign of (d - a) . ((b - a) x (c - a)).
    /// It is positive when d lies on the side of the plane (a, b, c) to which the normal (b - a) x (c - a) points.
    /// Bit i of second marks the argument i as a vertex of the second operand.
    /// The result is zero only for a degenerate case that the displacement cannot resolve.
    static int Orient(
        const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d, int second);

    /// Side of the point d of one operand against the plane of the triangle (a, b, c) of the other.
    /// The sign is the one of Orient. If g_csgExactOnly is true, the result is the result of Orient.
    /// Otherwise a point that is not farther from the plane than the thickness of the face lies on it.
    /// The displacement then decides, and a gradient component below the rounding noise does not count.
    /// The faceExtent is the extent of the face of the triangle, so all triangles of one face use one thickness.
    /// The thickness applies only if the arguments come from both operands.
    static int OrientPlane(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d,
        int second, double faceExtent);

    /// Side on which the line (e0, e1) of one operand passes the line (t0, t1) of the other.
    /// The sign is the one of Orient(e0, e1, t0, t1, second).
    /// If g_csgExactOnly is true, the result is the result of Orient.
    /// Otherwise segments that are not farther apart than the thickness meet, and the displacement decides.
    /// The distance and the thickness come from the four points only, so the two triangles on an edge agree.
    static int OrientLines(
        const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1, int second);

    /// Tell whether OrientPlane takes the point d as on the plane of the triangle (a, b, c).
    /// It is always false if g_csgExactOnly is true.
    static bool FlatPlane(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d,
        double faceExtent);

    /// Distance in grid steps within which a point lies on a plane and two lines meet.
    /// The extent is the largest absolute coordinate that is involved. Two steps cover the rounding of a crossing.
    static double Thickness(double extent) { return std::max(2.0, extent * (1.0 / 4194304.0)); }

    /// Largest absolute coordinate of a grid point.
    static double Extent(const CsgGridPoint& p)
    {
        return std::max(std::abs(static_cast<double>(p.x_)),
            std::max(std::abs(static_cast<double>(p.y_)), std::abs(static_cast<double>(p.z_))));
    }

    /// Tell whether the ray from p along +x crosses the triangle (t0, t1, t2).
    /// The configuration is the displaced one of Orient. Only two values of second are valid:
    /// 0b0111 (the triangle is in the second operand and the point is in the first) and
    /// 0b1000 (the point is in the second operand and the triangle is in the first).
    /// The marked points move by delta = (eps, eps^2, eps^3). A point on the plane or on an edge is never a tie.
    /// The bit bounds are those of Orient2 and Orient.
    static bool AxisRayCrossesTriangle(const CsgGridPoint& p, const CsgGridPoint& t0, const CsgGridPoint& t1,
        const CsgGridPoint& t2, int second);

    /// Same as AxisRayCrossesTriangle for the sample point q = (a + b + c) / 3, the centroid of a grid triangle.
    /// The three points a, b, c are all in the same operand, and second has the meaning of AxisRayCrossesTriangle.
    /// The function does not scale q. It uses sums of the exact tests of a, b and c.
    /// The planar sums are below 2^(2 * CsgGridBits + 5) and are computed in 64 bits.
    static bool AxisRayCrossesTriangleCentroid(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c,
        const CsgGridPoint& t0, const CsgGridPoint& t1, const CsgGridPoint& t2, int second);

    /// Project a grid point to a plane. The axis dropAxis (0, 1 or 2) is dropped.
    /// The other two axes keep the cyclic order: drop x gives (y, z), drop y gives (z, x), drop z gives (x, y).
    /// If flip is true, the two coordinates swap. This reverses the orientation of the projection.
    static CsgGridPoint2 Project(const CsgGridPoint& point, int dropAxis, bool flip)
    {
        URHO3D_ASSERT(dropAxis >= 0 && dropAxis < 3);
        CsgGridPoint2 result;
        switch (dropAxis)
        {
        case 0: result.u_ = point.y_; result.v_ = point.z_; break;
        case 1: result.u_ = point.z_; result.v_ = point.x_; break;
        default: result.u_ = point.x_; result.v_ = point.y_; break;
        }
        if (flip)
        {
            const CsgGridInt u = result.u_;
            result.u_ = result.v_;
            result.v_ = u;
        }
        return result;
    }

    /// Difference to - from. Each coordinate of the points must have an absolute value below 2^CsgGridBits.
    /// Then the difference is exact and has an absolute value below 2^(CsgGridBits + 1).
    static CsgGridPoint2 Difference(const CsgGridPoint2& to, const CsgGridPoint2& from)
    {
        CsgGridPoint2 result;
        result.u_ = static_cast<CsgGridInt>(static_cast<int64_t>(to.u_) - from.u_);
        result.v_ = static_cast<CsgGridInt>(static_cast<int64_t>(to.v_) - from.v_);
        return result;
    }

    /// Sign of d0.u * d1.v - d0.v * d1.u. The inputs are difference vectors of grid points.
    /// A coordinate has an absolute value below 2^(CsgGridBits + 1). The result is exact.
    /// It is positive when d1 is counterclockwise from d0, in the (u, v) plane with u to the right and v up.
    /// The function computes in double and uses the filter. The fallback is Cross2Exact.
    static int Cross2(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
    {
        const double det = static_cast<double>(d0.u_) * static_cast<double>(d1.v_)
            - static_cast<double>(d0.v_) * static_cast<double>(d1.u_);
        if (det > CsgPlanarFilterBound)
            return 1;
        if (det < -CsgPlanarFilterBound)
            return -1;
        if constexpr (CsgPlanarExactInDouble)
            return 0;
        return Cross2Exact(d0, d1);
    }

    /// Same as Cross2. Always uses the exact expansion arithmetic.
    static int Cross2Exact(const CsgGridPoint2& d0, const CsgGridPoint2& d1);

    /// Sign of d0.u * d1.u + d0.v * d1.v. The inputs are difference vectors, as in Cross2.
    static int Dot2(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
    {
        const double dot = static_cast<double>(d0.u_) * static_cast<double>(d1.u_)
            + static_cast<double>(d0.v_) * static_cast<double>(d1.v_);
        if (dot > CsgPlanarFilterBound)
            return 1;
        if (dot < -CsgPlanarFilterBound)
            return -1;
        if constexpr (CsgPlanarExactInDouble)
            return 0;
        return Dot2Exact(d0, d1);
    }

    /// Same as Dot2. Always uses the exact expansion arithmetic.
    static int Dot2Exact(const CsgGridPoint2& d0, const CsgGridPoint2& d1);

    /// Sign of (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u).
    /// It is positive when a, b, c turn counterclockwise. Each coordinate has an absolute value below 2^CsgGridBits.
    static int Orient2(const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c)
    {
        return Cross2(Difference(b, a), Difference(c, a));
    }

    /// Compare the directions q0 - p and q1 - p by the angle, clockwise, starting at the direction reference - p.
    /// Return -1 if q0 comes first, 1 if q1 comes first and 0 if the directions are equal.
    /// The reference must differ from p.
    ///
    /// Order. Let r = reference - p, d0 = q0 - p and d1 = q1 - p. Each direction d has a rank and an angle.
    /// The clockwise angle t(d) of d from r is in the range [0, 2 pi). The rank is:
    ///   0 if d is zero (q equal to p),
    ///   1 if t(d) is in [0, pi): Cross2(r, d) < 0, or Cross2(r, d) == 0 and Dot2(r, d) > 0,
    ///   2 if t(d) is in [pi, 2 pi): Cross2(r, d) > 0, or Cross2(r, d) == 0 and Dot2(r, d) < 0.
    /// The order is lexicographic by (rank, t(d)). A zero direction has the rank 0 and compares equal to
    /// any other zero direction. Directions with the same rank 1 or 2 compare by Cross2(d0, d1):
    /// the result is its sign.
    ///
    /// Proof that the order is a strict weak order. Define the key k(d) = (rank(d), t(d)), with t(zero) = 0.
    /// Compare the keys lexicographically. A lexicographic order on pairs of a totally ordered set is a total
    /// preorder, so "less" is a strict weak order, and "equal" means equal keys. It remains to show that the
    /// function returns the comparison of the keys.
    /// 1. Different ranks: the function compares the ranks, as the key does.
    /// 2. Both rank 0: both keys are equal, the function returns 0.
    /// 3. Same rank 1 or 2: both angles are in one half open interval of the length pi.
    ///    So t(d0) - t(d1) is in (-pi, pi). The counterclockwise angle from d0 to d1 is t(d0) - t(d1).
    ///    Cross2(d0, d1) has the sign of sin(t(d0) - t(d1)). In (-pi, pi) the sine has the sign of its argument.
    ///    So Cross2(d0, d1) < 0 if and only if t(d0) < t(d1). Also Cross2(d0, d1) == 0 if and only if
    ///    t(d0) == t(d1), because the angles cannot differ by exactly pi inside the interval.
    /// Directions with equal keys are the same ray, at any length. They compare equal (0).
    /// The caller breaks the tie, for example with a global id.
    /// The rank and the signs are exact, so the order is exact.
    static int DirectionCompare(const CsgGridPoint2& p, const CsgGridPoint2& reference, const CsgGridPoint2& q0,
        const CsgGridPoint2& q1);

    /// Sign of |q0 - p|^2 - |q1 - p|^2. The result is exact.
    static int SquaredDistanceCompare(const CsgGridPoint2& p, const CsgGridPoint2& q0, const CsgGridPoint2& q1)
    {
        const CsgGridPoint2 d0 = Difference(q0, p);
        const CsgGridPoint2 d1 = Difference(q1, p);
        const double n0 = static_cast<double>(d0.u_) * static_cast<double>(d0.u_)
            + static_cast<double>(d0.v_) * static_cast<double>(d0.v_);
        const double n1 = static_cast<double>(d1.u_) * static_cast<double>(d1.u_)
            + static_cast<double>(d1.v_) * static_cast<double>(d1.v_);
        const double diff = n0 - n1;
        if (diff > CsgDistanceFilterBound)
            return 1;
        if (diff < -CsgDistanceFilterBound)
            return -1;
        if constexpr (CsgDistanceExactInDouble)
            return 0;
        return SquaredDistanceCompareExact(d0, d1);
    }

    /// Same as SquaredDistanceCompare for the difference vectors d0 = q0 - p and d1 = q1 - p.
    /// Always uses the exact expansion arithmetic.
    static int SquaredDistanceCompareExact(const CsgGridPoint2& d0, const CsgGridPoint2& d1);

    /// Whether the closed segments ab and cd have a common point.
    /// Touching counts as crossing: an end point of one segment on the other segment, or a common end point.
    /// Collinear segments that overlap, also in one point, count as crossing. Collinear disjoint segments do not.
    /// A segment with equal ends is a point. The function treats it in the same way.
    static bool SegmentsCross(
        const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c, const CsgGridPoint2& d);

    /// Sign of the signed area of the closed loop points[0 .. count - 1]. The sign is positive for the
    /// counterclockwise order. The sum is the shoelace sum of Cross2 terms relative to points[0].
    /// A loop with fewer than 3 points has the area 0. A term has an absolute value below 2^CsgExactPlanarBits.
    /// A sum of up to CsgLoopFilterMaxEdges terms has an absolute value below 2^CsgExactPlanarSumBits.
    /// A longer loop uses the exact sum without the filter.
    static int LoopAreaSign(const CsgGridPoint2* points, int count);

    /// Same as LoopAreaSign. Always uses the exact expansion arithmetic.
    /// A loop with more than CsgLoopAreaMaxPoints points gets the sign of the double sum. This sum can be wrong.
    static int LoopAreaSignExact(const CsgGridPoint2* points, int count);

    /// Write the doubled signed area of the loop as a compressed expansion. This is the sum of LoopAreaSign.
    /// The out buffer needs CsgLoopAreaMaxTerms entries. Return the number of terms. A loop of fewer than 3 points gives 0.
    /// Return -1 and write nothing if the loop has more than CsgLoopAreaMaxPoints points.
    static int LoopAreaExact(const CsgGridPoint2* points, int count, CsgReal* out);

    /// Return the sign of |A| - |B|. A and B are expansions from LoopAreaExact.
    /// Each count is at most CsgLoopAreaMaxTerms. The result is exact for all such inputs.
    /// A count below 0 or above CsgLoopAreaMaxTerms gives 0.
    static int CompareLoopAreaMagnitude(int countA, const CsgReal* a, int countB, const CsgReal* b);

    /// Exact a * b - c * d as an expansion. The inputs must be exact differences of grid coordinates.
    static CsgExactMinor MinorExact(CsgReal a, CsgReal b, CsgReal c, CsgReal d);

    /// Unperturbed (d - a) . ((b - a) x (c - a)), exactly.
    static CsgExactVolume Volume(
        const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d);

    /// (b - a) x (c - a), exactly.
    static CsgNormal Normal(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c);

    /// Exact n . (to - from).
    static CsgExactVolume Dot(const CsgNormal& n, const CsgGridPoint& from, const CsgGridPoint& to);

    /// Whether the edge (e0, e1) crosses the interior of the triangle (t0, t1, t2) of the other operand.
    /// The output e1Above tells whether e1 lies on the side to which the triangle normal points.
    /// A degenerate triangle is never crossed.
    static bool Crosses(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1,
        const CsgGridPoint& t2, double faceExtent, bool edgeInSecond, bool& e1Above);

    /// Whether the line through an edge that crosses the plane of a triangle passes inside that triangle.
    static bool PassesInside(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0,
        const CsgGridPoint& t1, const CsgGridPoint& t2, bool edgeInSecond);

    /// Exact -v.
    static CsgExactVolume Negate(const CsgExactVolume& v);

    /// Round the point x = low + (high - low) * numerator / denominator to the grid.
    /// The result is floor(x + 1/2): a tie goes toward plus infinity.
    /// The denominator must not be zero. The sign of the denominator is free.
    /// The fraction numerator / denominator must be in the range 0 to 1, so that x lies between low and high.
    /// The magnitudes of the numerator and the denominator must be below 2^CsgExactVolumeBits.
    /// Both must be compressed expansions, as Dot returns them.
    static CsgGridInt RoundOnEdge(
        CsgGridInt low, CsgGridInt high, const CsgExactVolume& numerator, const CsgExactVolume& denominator);

    /// Round the crossing of the edge (low, high) with a plane to the grid. Each axis follows RoundOnEdge.
    /// With the plane normal n and a point t0 of the plane, the numerator is n . (low - t0) = lowVolume.
    /// The denominator is n . (low - high) = difference. Both come from Dot, as in the crossing data.
    static CsgGridPoint RoundCrossing(const CsgGridPoint& low, const CsgGridPoint& high,
        const CsgExactVolume& lowVolume, const CsgExactVolume& difference);

    /// Give the cell of the exact crossing x of the edge (low, high) with a plane: lower <= x <= upper on each axis.
    /// The rounded point must be the result of RoundCrossing for the same data. An axis has upper - lower equal to 1,
    /// or equal to 0 if x is exactly on the grid. The rounded point is on both ends of the cell.
    /// The tests use exact signs and no floating point. The difference must not be zero.
    static void CrossingCell(const CsgGridPoint& low, const CsgGridPoint& high, const CsgExactVolume& lowVolume,
        const CsgExactVolume& difference, const CsgGridPoint& rounded, CsgGridPoint& lower, CsgGridPoint& upper);

private:
    /// Sign of (d - a) . ((b - a) x (c - a)) for points of the two operands.
    /// If thick is false, the result is the one of Orient and the other arguments do not count.
    /// If thick is true and the arguments come from both operands, the volume is a distance times a length.
    /// It is the distance of d to the plane (a, b, c) times |u x v|, or the distance between the lines (a, b) and
    /// (c, d) times |u x (w - v)|, for u = b - a, v = c - a, w = d - a. The flag lines selects the second reading.
    /// For lines, the function finds its own thickness.
    static int OrientThick(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d,
        int second, bool thick, bool lines, double thickness);

    /// Thickness of a face of the extent faceExtent against the point d of the other operand.
    static double PlaneThickness(const CsgGridPoint& d, double faceExtent)
    {
        return Thickness(std::max(Extent(d), faceExtent));
    }

    /// Tell whether the volume det is a distance within the thickness: |det| <= thickness * |u x p|.
    static bool Flat(
        double ux, double uy, double uz, double px, double py, double pz, double det, double thickness);

    /// Tell whether the segments (a, a + u) and (a + v, a + v + p) come within the thickness of their closest points.
    /// The distance of the lines is not the measure: lines at a small angle meet far from the segments.
    /// The largest coordinate of the four ends is not the measure of the thickness. A long edge has the error
    /// of its near end where it passes that end. The output is the thickness at the closest points.
    static bool SegmentsMeet(const CsgGridPoint& a, double ux, double uy, double uz, double vx, double vy, double vz,
        double px, double py, double pz, double& thickness);
};

} // namespace Urho3D
