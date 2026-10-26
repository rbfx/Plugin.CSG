// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include "Urho3D/CSG/CsgExpansion.h"
#include <Urho3D/Core/AssertBase.h>
#include <Urho3D/Math/Vector3.h>

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

    friend bool operator==(const CsgGridPoint& a, const CsgGridPoint& b)
    {
        return a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_;
    }
    friend bool operator!=(const CsgGridPoint& a, const CsgGridPoint& b) { return !(a == b); }
};

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
        const CsgGridPoint& t2, bool edgeInSecond, bool& e1Above);

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
};

} // namespace Urho3D
