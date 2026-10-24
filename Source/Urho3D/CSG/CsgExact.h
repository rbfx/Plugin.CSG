// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Core/AssertBase.h>
#include <Urho3D/Math/Vector3.h>

#include <cmath>
#include <cstdint>

#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_ARM64) || defined(_M_ARM64EC))
#include <intrin.h>
#endif

namespace Urho3D
{

namespace CsgDetail
{

/// Multiply two unsigned 64 bit numbers. Return the low half and write the high half to hi.
inline uint64_t UMul64(uint64_t a, uint64_t b, uint64_t& hi)
{
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC)
    return _umul128(a, b, &hi);
#elif defined(_MSC_VER) && (defined(_M_ARM64) || defined(_M_ARM64EC))
    hi = __umulh(a, b);
    return a * b;
#else
    const uint64_t mask = 0xFFFFFFFFull;
    const uint64_t a0 = a & mask;
    const uint64_t a1 = a >> 32;
    const uint64_t b0 = b & mask;
    const uint64_t b1 = b >> 32;
    const uint64_t p00 = a0 * b0;
    const uint64_t p01 = a0 * b1;
    const uint64_t p10 = a1 * b0;
    const uint64_t p11 = a1 * b1;
    const uint64_t middle = (p00 >> 32) + (p01 & mask) + (p10 & mask);
    hi = p11 + (p01 >> 32) + (p10 >> 32) + (middle >> 32);
    return (middle << 32) | (p00 & mask);
#endif
}

/// Add b to a. Return the carry (0 or 1).
inline uint64_t AddCarry(uint64_t& a, uint64_t b)
{
    a += b;
    return a < b ? 1 : 0;
}

} // namespace CsgDetail

/// Signed 128 bit integer made of two 64 bit limbs. The arithmetic wraps around modulo 2^128.
/// The struct is always compiled. It is the 128 bit type on compilers without __int128.
struct CsgInt128Portable
{
    uint64_t lo_{0};
    int64_t hi_{0};

    CsgInt128Portable() = default;
    /// Sign-extending conversion.
    CsgInt128Portable(int64_t value)
        : lo_(static_cast<uint64_t>(value))
        , hi_(value < 0 ? -1 : 0)
    {
    }

    static CsgInt128Portable FromParts(uint64_t lo, int64_t hi)
    {
        CsgInt128Portable result;
        result.lo_ = lo;
        result.hi_ = hi;
        return result;
    }

    /// Exact product of two signed 64 bit numbers.
    static CsgInt128Portable Multiply(int64_t a, int64_t b)
    {
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC)
        int64_t high = 0;
        const uint64_t low = static_cast<uint64_t>(_mul128(a, b, &high));
        return FromParts(low, high);
#else
        uint64_t high = 0;
        const uint64_t low = CsgDetail::UMul64(static_cast<uint64_t>(a), static_cast<uint64_t>(b), high);
        if (a < 0)
            high -= static_cast<uint64_t>(b);
        if (b < 0)
            high -= static_cast<uint64_t>(a);
        return FromParts(low, static_cast<int64_t>(high));
#endif
    }

    /// Product of this number and a signed 64 bit number, modulo 2^128.
    CsgInt128Portable MultiplySmall(int64_t value) const
    {
        const uint64_t small = static_cast<uint64_t>(value);
        uint64_t high = 0;
        const uint64_t low = CsgDetail::UMul64(lo_, small, high);
        high += static_cast<uint64_t>(hi_) * small;
        if (value < 0)
            high -= lo_;
        return FromParts(low, static_cast<int64_t>(high));
    }

    int Sign() const { return hi_ < 0 ? -1 : (hi_ > 0 || lo_ != 0 ? 1 : 0); }

    friend CsgInt128Portable operator+(const CsgInt128Portable& a, const CsgInt128Portable& b)
    {
        const uint64_t low = a.lo_ + b.lo_;
        const uint64_t carry = low < a.lo_ ? 1 : 0;
        return FromParts(low, static_cast<int64_t>(static_cast<uint64_t>(a.hi_) + static_cast<uint64_t>(b.hi_) + carry));
    }

    friend CsgInt128Portable operator-(const CsgInt128Portable& a, const CsgInt128Portable& b)
    {
        const uint64_t borrow = a.lo_ < b.lo_ ? 1 : 0;
        return FromParts(a.lo_ - b.lo_,
            static_cast<int64_t>(static_cast<uint64_t>(a.hi_) - static_cast<uint64_t>(b.hi_) - borrow));
    }

    friend CsgInt128Portable operator-(const CsgInt128Portable& a) { return CsgInt128Portable() - a; }

    /// Product modulo 2^128.
    friend CsgInt128Portable operator*(const CsgInt128Portable& a, const CsgInt128Portable& b)
    {
        uint64_t high = 0;
        const uint64_t low = CsgDetail::UMul64(a.lo_, b.lo_, high);
        high += a.lo_ * static_cast<uint64_t>(b.hi_) + static_cast<uint64_t>(a.hi_) * b.lo_;
        return FromParts(low, static_cast<int64_t>(high));
    }

    CsgInt128Portable& operator+=(const CsgInt128Portable& other) { return *this = *this + other; }
    CsgInt128Portable& operator-=(const CsgInt128Portable& other) { return *this = *this - other; }

    friend bool operator==(const CsgInt128Portable& a, const CsgInt128Portable& b)
    {
        return a.lo_ == b.lo_ && a.hi_ == b.hi_;
    }
    friend bool operator!=(const CsgInt128Portable& a, const CsgInt128Portable& b) { return !(a == b); }
    friend bool operator<(const CsgInt128Portable& a, const CsgInt128Portable& b)
    {
        return a.hi_ != b.hi_ ? a.hi_ < b.hi_ : a.lo_ < b.lo_;
    }
    friend bool operator>(const CsgInt128Portable& a, const CsgInt128Portable& b) { return b < a; }
    friend bool operator<=(const CsgInt128Portable& a, const CsgInt128Portable& b) { return !(b < a); }
    friend bool operator>=(const CsgInt128Portable& a, const CsgInt128Portable& b) { return !(a < b); }
};

#if defined(__SIZEOF_INT128__) && !defined(CSG_FORCE_PORTABLE_INT128)
#define CSG_HAS_NATIVE_INT128 1
__extension__ typedef __int128 CsgInt128;
#else
using CsgInt128 = CsgInt128Portable;
#endif

#if defined(__SIZEOF_INT128__)
#define CSG_HAS_COMPILER_INT128 1
__extension__ typedef __int128 CsgCompilerInt128;
__extension__ typedef unsigned __int128 CsgCompilerUInt128;
#endif

/// Exact product of two signed 64 bit numbers as the 128 bit type Int.
template <typename Int>
inline Int CsgMul64(int64_t a, int64_t b)
{
    return Int(a) * Int(b);
}

template <>
inline CsgInt128Portable CsgMul64<CsgInt128Portable>(int64_t a, int64_t b)
{
    return CsgInt128Portable::Multiply(a, b);
}

/// Product of a 128 bit number and a signed 64 bit number, modulo 2^128.
inline CsgInt128Portable CsgMul(const CsgInt128Portable& a, int64_t b)
{
    return a.MultiplySmall(b);
}

/// Sign of a 128 bit number: -1, 0 or 1.
inline int CsgSign(const CsgInt128Portable& value)
{
    return value.Sign();
}

#if CSG_HAS_COMPILER_INT128
inline CsgCompilerInt128 CsgMul(const CsgCompilerInt128& a, int64_t b)
{
    return a * b;
}

inline int CsgSign(const CsgCompilerInt128& value)
{
    return value > 0 ? 1 : (value < 0 ? -1 : 0);
}
#endif

namespace CsgDetail
{

/// Magnitude of a 128 bit number and its sign.
struct Magnitude128
{
    uint64_t lo_;
    uint64_t hi_;
    int sign_;
};

inline Magnitude128 GetMagnitude(const CsgInt128Portable& value)
{
    const int sign = value.Sign();
    if (sign >= 0)
        return {value.lo_, static_cast<uint64_t>(value.hi_), sign};
    const CsgInt128Portable negated = -value;
    return {negated.lo_, static_cast<uint64_t>(negated.hi_), sign};
}

#if CSG_HAS_COMPILER_INT128
inline Magnitude128 GetMagnitude(const CsgCompilerInt128& value)
{
    const int sign = value > 0 ? 1 : (value < 0 ? -1 : 0);
    const CsgCompilerUInt128 magnitude = value < 0 ? CsgCompilerUInt128(0) - static_cast<CsgCompilerUInt128>(value)
                                                   : static_cast<CsgCompilerUInt128>(value);
    return {static_cast<uint64_t>(magnitude), static_cast<uint64_t>(magnitude >> 64), sign};
}
#endif

/// Product of two 128 bit magnitudes as four limbs. The limb 0 is the least significant.
inline void MultiplyMagnitudes(const Magnitude128& x, const Magnitude128& y, uint64_t (&r)[4])
{
    uint64_t hi00 = 0;
    uint64_t hi01 = 0;
    uint64_t hi10 = 0;
    uint64_t hi11 = 0;
    const uint64_t lo00 = UMul64(x.lo_, y.lo_, hi00);
    const uint64_t lo01 = UMul64(x.lo_, y.hi_, hi01);
    const uint64_t lo10 = UMul64(x.hi_, y.lo_, hi10);
    const uint64_t lo11 = UMul64(x.hi_, y.hi_, hi11);

    r[0] = lo00;
    r[1] = hi00;
    uint64_t carry1 = AddCarry(r[1], lo01);
    carry1 += AddCarry(r[1], lo10);
    r[2] = carry1;
    uint64_t carry2 = AddCarry(r[2], hi01);
    carry2 += AddCarry(r[2], hi10);
    carry2 += AddCarry(r[2], lo11);
    r[3] = hi11 + carry2;
}

/// Compare two 256 bit magnitudes. Return -1, 0 or 1.
inline int CompareLimbs(const uint64_t (&a)[4], const uint64_t (&b)[4])
{
    for (int i = 3; i >= 0; --i)
    {
        if (a[i] != b[i])
            return a[i] < b[i] ? -1 : 1;
    }
    return 0;
}

} // namespace CsgDetail

/// Return the sign of a * b - c * d. The products are exact in 256 bits. It does not overflow.
template <typename Int>
inline int CsgSignOfProductDifference(const Int& a, const Int& b, const Int& c, const Int& d)
{
    using namespace CsgDetail;
    const Magnitude128 ma = GetMagnitude(a);
    const Magnitude128 mb = GetMagnitude(b);
    const Magnitude128 mc = GetMagnitude(c);
    const Magnitude128 md = GetMagnitude(d);
    const int firstSign = ma.sign_ * mb.sign_;
    const int secondSign = mc.sign_ * md.sign_;
    if (firstSign == 0)
        return -secondSign;
    if (secondSign == 0)
        return firstSign;
    if (firstSign != secondSign)
        return firstSign;

    uint64_t first[4];
    uint64_t second[4];
    MultiplyMagnitudes(ma, mb, first);
    MultiplyMagnitudes(mc, md, second);
    return firstSign * CompareLimbs(first, second);
}

/// Scale of the integer grid for the exact tests: 2^-24 units.
/// Coordinates must stay below 2^16 units.
constexpr double CsgGridScale = 1 << 24;

/// Point on the integer grid.
struct CsgGridPoint
{
    int64_t x_{};
    int64_t y_{};
    int64_t z_{};

    /// Snap a point to the grid. A value exactly between two grid points goes to the even grid point.
    /// The rounding assumes the default floating point mode, round to nearest even.
    /// Each coordinate must be finite and must have an absolute value less than 2^16.
    static CsgGridPoint From(const Vector3& point)
    {
        URHO3D_ASSERT(std::isfinite(point.x_) && std::isfinite(point.y_) && std::isfinite(point.z_));
        CsgGridPoint result;
        result.x_ = static_cast<int64_t>(std::nearbyint(static_cast<double>(point.x_) * CsgGridScale));
        result.y_ = static_cast<int64_t>(std::nearbyint(static_cast<double>(point.y_) * CsgGridScale));
        result.z_ = static_cast<int64_t>(std::nearbyint(static_cast<double>(point.z_) * CsgGridScale));
        return result;
    }

    friend bool operator==(const CsgGridPoint& a, const CsgGridPoint& b)
    {
        return a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_;
    }
    friend bool operator!=(const CsgGridPoint& a, const CsgGridPoint& b) { return !(a == b); }
};

/// Exact 128 bit vector.
template <typename Int>
struct CsgNormal
{
    Int x_;
    Int y_;
    Int z_;
};

/// Exact orientation tests with the symbolic displacement of the second operand.
/// Each function with the suffix T works with the 128 bit type Int.
/// The function without the suffix uses CsgInt128.
class PLUGIN_RBFXCSG_API CsgExact
{
public:
    /// Sign of (d - a) . ((b - a) x (c - a)).
    /// It is positive when d lies on the side of the plane (a, b, c) to which the normal (b - a) x (c - a) points.
    /// Bit i of second marks the argument i as a vertex of the second operand.
    /// The result is zero only for a degenerate case that the displacement cannot resolve.
    static int Orient(
        const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d, int second);

    /// Unperturbed (d - a) . ((b - a) x (c - a)), exactly.
    static CsgInt128 Volume(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d);

    /// (b - a) x (c - a), exactly.
    static CsgNormal<CsgInt128> Normal(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c);

    /// Whether the edge (e0, e1) crosses the interior of the triangle (t0, t1, t2) of the other operand.
    /// The output e1Above tells whether e1 lies on the side to which the triangle normal points.
    /// A degenerate triangle is never crossed.
    static bool Crosses(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1,
        const CsgGridPoint& t2, bool edgeInSecond, bool& e1Above);

    /// Whether the line through an edge that crosses the plane of a triangle passes inside that triangle.
    static bool PassesInside(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0,
        const CsgGridPoint& t1, const CsgGridPoint& t2, bool edgeInSecond);

    template <typename Int>
    static int OrientT(
        const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d, int second);

    template <typename Int>
    static Int VolumeT(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d);

    template <typename Int>
    static CsgNormal<Int> NormalT(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c);

    template <typename Int>
    static bool CrossesT(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1,
        const CsgGridPoint& t2, bool edgeInSecond, bool& e1Above);

    template <typename Int>
    static bool PassesInsideT(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0,
        const CsgGridPoint& t1, const CsgGridPoint& t2, bool edgeInSecond);
};

} // namespace Urho3D
