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
/// Double mode: the step is 2^-26 units. Grid coordinates stay below 2^39 and differences below 2^40.
/// The finer grid keeps every float coordinate with an absolute value of 2^-3 or more without a change.
/// In both modes, a difference is exact in CsgReal.
#if defined(CSG_EXACT_USE_DOUBLE)
constexpr int CsgGridScaleBits = 26;
#else
constexpr int CsgGridScaleBits = 10;
#endif

/// Bits of a grid coordinate: the absolute value of a grid coordinate is below 2^CsgGridBits.
constexpr int CsgGridBits = CsgGridCoordinateBits + CsgGridScaleBits;
static_assert(CsgGridBits + 1 <= std::numeric_limits<CsgReal>::digits, "Grid differences must be exact in CsgReal.");

/// A minor of differences has a magnitude below 2^CsgExactMinorBits: 2^50 in float mode, 2^82 in double mode.
constexpr int CsgExactMinorBits = 2 * CsgGridBits + 4;
/// A determinant of differences has a magnitude below 2^CsgExactVolumeBits.
/// It is 2^75 in float mode and 2^123 in double mode.
constexpr int CsgExactVolumeBits = 3 * CsgGridBits + 6;
static_assert(CsgExactVolumeBits <= CsgProductDifferenceInputBits, "Volumes must be valid product difference inputs.");

/// Scale of the integer grid for the exact tests: 2^CsgGridScaleBits grid steps in one unit.
constexpr double CsgGridScale = static_cast<double>(int64_t{1} << CsgGridScaleBits);

/// Grid coordinates have an absolute value below this limit.
constexpr int64_t CsgGridIntegerLimit = int64_t{1} << CsgGridBits;

/// Coordinates with an absolute value of this limit or more are outside the range of the grid.
constexpr float CsgGridCoordinateLimit = static_cast<float>(int64_t{1} << CsgGridCoordinateBits);

/// Integer type of a grid coordinate.
using CsgGridInt = std::conditional_t<(CsgGridBits < 31), int32_t, int64_t>;

/// Point on the integer grid.
struct CsgGridPoint
{
    CsgGridInt x_{};
    CsgGridInt y_{};
    CsgGridInt z_{};

    /// Snap a point to the grid. A value exactly between two grid points goes to the even grid point.
    /// The rounding assumes the default floating point mode, round to nearest even.
    /// Each coordinate must be finite and must have an absolute value less than 2^13.
    static CsgGridPoint From(const Vector3& point)
    {
        URHO3D_ASSERT(std::isfinite(point.x_) && std::isfinite(point.y_) && std::isfinite(point.z_));
        URHO3D_ASSERT(std::fabs(point.x_) < CsgGridCoordinateLimit && std::fabs(point.y_) < CsgGridCoordinateLimit
            && std::fabs(point.z_) < CsgGridCoordinateLimit);
        CsgGridPoint result;
        result.x_ = static_cast<CsgGridInt>(std::nearbyint(static_cast<double>(point.x_) * CsgGridScale));
        result.y_ = static_cast<CsgGridInt>(std::nearbyint(static_cast<double>(point.y_) * CsgGridScale));
        result.z_ = static_cast<CsgGridInt>(std::nearbyint(static_cast<double>(point.z_) * CsgGridScale));
        return result;
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
};

} // namespace Urho3D
