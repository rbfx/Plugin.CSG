// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Test-only copy of the 128 bit integer code of the exact predicates.
// The reference oracle of the tests uses it. The algorithms are the same as in the old library code.

#pragma once

#include <EASTL/vector.h>

#include <cmath>
#include <cstdint>

#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_ARM64) || defined(_M_ARM64EC))
#include <intrin.h>
#endif

namespace CsgTest
{

namespace Detail
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

} // namespace Detail

/// Signed 128 bit integer made of two 64 bit limbs. The arithmetic wraps around modulo 2^128.
/// The struct is always compiled. It is the 128 bit type on compilers without __int128.
struct Int128Portable
{
    uint64_t lo_{0};
    int64_t hi_{0};

    Int128Portable() = default;
    /// Sign-extending conversion.
    Int128Portable(int64_t value)
        : lo_(static_cast<uint64_t>(value))
        , hi_(value < 0 ? -1 : 0)
    {
    }

    static Int128Portable FromParts(uint64_t lo, int64_t hi)
    {
        Int128Portable result;
        result.lo_ = lo;
        result.hi_ = hi;
        return result;
    }

    /// Exact product of two signed 64 bit numbers.
    static Int128Portable Multiply(int64_t a, int64_t b)
    {
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC)
        int64_t high = 0;
        const uint64_t low = static_cast<uint64_t>(_mul128(a, b, &high));
        return FromParts(low, high);
#else
        uint64_t high = 0;
        const uint64_t low = Detail::UMul64(static_cast<uint64_t>(a), static_cast<uint64_t>(b), high);
        if (a < 0)
            high -= static_cast<uint64_t>(b);
        if (b < 0)
            high -= static_cast<uint64_t>(a);
        return FromParts(low, static_cast<int64_t>(high));
#endif
    }

    /// Product of this number and a signed 64 bit number, modulo 2^128.
    Int128Portable MultiplySmall(int64_t value) const
    {
        const uint64_t small = static_cast<uint64_t>(value);
        uint64_t high = 0;
        const uint64_t low = Detail::UMul64(lo_, small, high);
        high += static_cast<uint64_t>(hi_) * small;
        if (value < 0)
            high -= lo_;
        return FromParts(low, static_cast<int64_t>(high));
    }

    int Sign() const { return hi_ < 0 ? -1 : (hi_ > 0 || lo_ != 0 ? 1 : 0); }

    friend Int128Portable operator+(const Int128Portable& a, const Int128Portable& b)
    {
        const uint64_t low = a.lo_ + b.lo_;
        const uint64_t carry = low < a.lo_ ? 1 : 0;
        return FromParts(low, static_cast<int64_t>(static_cast<uint64_t>(a.hi_) + static_cast<uint64_t>(b.hi_) + carry));
    }

    friend Int128Portable operator-(const Int128Portable& a, const Int128Portable& b)
    {
        const uint64_t borrow = a.lo_ < b.lo_ ? 1 : 0;
        return FromParts(a.lo_ - b.lo_,
            static_cast<int64_t>(static_cast<uint64_t>(a.hi_) - static_cast<uint64_t>(b.hi_) - borrow));
    }

    friend Int128Portable operator-(const Int128Portable& a) { return Int128Portable() - a; }

    /// Product modulo 2^128.
    friend Int128Portable operator*(const Int128Portable& a, const Int128Portable& b)
    {
        uint64_t high = 0;
        const uint64_t low = Detail::UMul64(a.lo_, b.lo_, high);
        high += a.lo_ * static_cast<uint64_t>(b.hi_) + static_cast<uint64_t>(a.hi_) * b.lo_;
        return FromParts(low, static_cast<int64_t>(high));
    }

    Int128Portable& operator+=(const Int128Portable& other) { return *this = *this + other; }
    Int128Portable& operator-=(const Int128Portable& other) { return *this = *this - other; }

    friend bool operator==(const Int128Portable& a, const Int128Portable& b)
    {
        return a.lo_ == b.lo_ && a.hi_ == b.hi_;
    }
    friend bool operator!=(const Int128Portable& a, const Int128Portable& b) { return !(a == b); }
    friend bool operator<(const Int128Portable& a, const Int128Portable& b)
    {
        return a.hi_ != b.hi_ ? a.hi_ < b.hi_ : a.lo_ < b.lo_;
    }
    friend bool operator>(const Int128Portable& a, const Int128Portable& b) { return b < a; }
    friend bool operator<=(const Int128Portable& a, const Int128Portable& b) { return !(b < a); }
    friend bool operator>=(const Int128Portable& a, const Int128Portable& b) { return !(a < b); }
};

#if defined(__SIZEOF_INT128__)
#define CSG_TEST_HAS_NATIVE_INT128 1
__extension__ typedef __int128 Int128Native;
__extension__ typedef unsigned __int128 UInt128Native;
/// The 128 bit type of the oracle. It is the compiler type when the compiler has one.
using Int128 = Int128Native;
#else
using Int128 = Int128Portable;
#endif

/// Exact product of two signed 64 bit numbers as the 128 bit type Int.
template <typename Int>
inline Int Mul64(int64_t a, int64_t b)
{
    return Int(a) * Int(b);
}

template <>
inline Int128Portable Mul64<Int128Portable>(int64_t a, int64_t b)
{
    return Int128Portable::Multiply(a, b);
}

/// Product of a 128 bit number and a signed 64 bit number, modulo 2^128.
inline Int128Portable Mul(const Int128Portable& a, int64_t b)
{
    return a.MultiplySmall(b);
}

/// Sign of a 128 bit number: -1, 0 or 1.
inline int Sign(const Int128Portable& value)
{
    return value.Sign();
}

#ifdef CSG_TEST_HAS_NATIVE_INT128
inline Int128Native Mul(const Int128Native& a, int64_t b)
{
    return a * b;
}

inline int Sign(const Int128Native& value)
{
    return value > 0 ? 1 : (value < 0 ? -1 : 0);
}
#endif

namespace Detail
{

/// Magnitude of a 128 bit number and its sign.
struct Magnitude128
{
    uint64_t lo_;
    uint64_t hi_;
    int sign_;
};

inline Magnitude128 GetMagnitude(const Int128Portable& value)
{
    const int sign = value.Sign();
    if (sign >= 0)
        return {value.lo_, static_cast<uint64_t>(value.hi_), sign};
    const Int128Portable negated = -value;
    return {negated.lo_, static_cast<uint64_t>(negated.hi_), sign};
}

#ifdef CSG_TEST_HAS_NATIVE_INT128
inline Magnitude128 GetMagnitude(const Int128Native& value)
{
    const int sign = value > 0 ? 1 : (value < 0 ? -1 : 0);
    const UInt128Native magnitude =
        value < 0 ? UInt128Native(0) - static_cast<UInt128Native>(value) : static_cast<UInt128Native>(value);
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

} // namespace Detail

/// Return the sign of a * b - c * d. The products are exact in 256 bits. It does not overflow.
template <typename Int>
inline int SignOfProductDifference(const Int& a, const Int& b, const Int& c, const Int& d)
{
    using namespace Detail;
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

/// Convert a portable 128 bit number to double with one rounding to nearest even.
inline double ToDouble(const Int128Portable& value)
{
    const Detail::Magnitude128 magnitude = Detail::GetMagnitude(value);
    if (magnitude.sign_ == 0)
        return 0.0;
    const uint64_t hi = magnitude.hi_;
    const uint64_t lo = magnitude.lo_;
    double result;
    if (hi == 0)
    {
        // A 64 bit number converts with one rounding.
        result = static_cast<double>(lo);
    }
    else
    {
        // Keep the top 64 bits of the magnitude. A double has 53 bits, so bit 0 of the top part is far below the
        // rounding position. A sticky bit in bit 0 keeps the round to nearest even correct.
        int shift = 0;
        for (uint64_t rest = hi; rest != 0; rest >>= 1)
            ++shift;
        uint64_t top;
        bool sticky;
        if (shift == 64)
        {
            top = hi;
            sticky = lo != 0;
        }
        else
        {
            top = (hi << (64 - shift)) | (lo >> shift);
            sticky = (lo & ((uint64_t{1} << shift) - 1)) != 0;
        }
        if (sticky)
            top |= 1;
        result = std::ldexp(static_cast<double>(top), shift);
    }
    return magnitude.sign_ < 0 ? -result : result;
}

#ifdef CSG_TEST_HAS_NATIVE_INT128
inline double ToDouble(const Int128Native& value)
{
    return static_cast<double>(value);
}
#endif

/// Multiply a 128 bit number by 2^bits. The bits value must not be negative.
template <typename Int>
inline Int ShiftLeft(Int value, int bits)
{
    while (bits > 0)
    {
        const int step = bits < 40 ? bits : 40;
        value = Mul(value, int64_t(1) << step);
        bits -= step;
    }
    return value;
}

/// Return an exact float expansion of a 128 bit number. Zero gives an empty vector.
/// Each term is a 24 bit chunk times 2^(24 * i). The smallest term is first. The sign is in each term.
inline ea::vector<float> FromInt128(Int128 value)
{
    ea::vector<float> terms;
    const Detail::Magnitude128 magnitude = Detail::GetMagnitude(value);
    if (magnitude.sign_ == 0)
        return terms;

    constexpr uint64_t chunkMask = (uint64_t{1} << 24) - 1;
    for (int i = 0; i * 24 < 128; ++i)
    {
        const int bit = i * 24;
        uint64_t chunk;
        if (bit == 0)
            chunk = magnitude.lo_ & chunkMask;
        else if (bit < 64)
            chunk = ((magnitude.lo_ >> bit) | (magnitude.hi_ << (64 - bit))) & chunkMask;
        else
            chunk = (magnitude.hi_ >> (bit - 64)) & chunkMask;
        if (chunk == 0)
            continue;
        const float signedChunk = magnitude.sign_ < 0 ? -static_cast<float>(chunk) : static_cast<float>(chunk);
        terms.push_back(std::ldexp(signedChunk, bit));
    }
    return terms;
}

/// Sum the float terms to a 128 bit number. Each term is an integer m * 2^e with e >= 0.
inline Int128 ToInt128(const float* terms, int count)
{
    Int128 result = Int128(int64_t(0));
    for (int i = 0; i < count; ++i)
    {
        if (terms[i] == 0.0f)
            continue;
        int exponent = 0;
        const float fraction = std::frexp(terms[i], &exponent);
        // A float has at most 24 significant bits, so the scaled fraction is an integer.
        int64_t mantissa = static_cast<int64_t>(std::ldexp(fraction, 24));
        int shift = exponent - 24;
        while ((mantissa & 1) == 0)
        {
            mantissa /= 2;
            ++shift;
        }
        result = result + ShiftLeft(Int128(mantissa), shift);
    }
    return result;
}

} // namespace CsgTest
