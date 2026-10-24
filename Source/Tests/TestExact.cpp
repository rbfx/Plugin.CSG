// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgExact.h"
#include "Urho3D/CSG/CsgExactFilter.h"

#include <catch2/catch_amalgamated.hpp>

#include <cstdint>
#include <limits>
#include <random>
#include <tuple>

namespace Urho3D
{

namespace
{

// The integer types under test. The portable struct always runs. The default type runs too, when it differs.
#if CSG_HAS_NATIVE_INT128
using CsgTestTypes = std::tuple<CsgInt128Portable, CsgInt128>;
#else
using CsgTestTypes = std::tuple<CsgInt128Portable>;
#endif

CsgInt128Portable MakeInt(const CsgInt128Portable*, int64_t hi, uint64_t lo)
{
    return CsgInt128Portable::FromParts(lo, hi);
}

#if CSG_HAS_COMPILER_INT128
CsgCompilerInt128 MakeInt(const CsgCompilerInt128*, int64_t hi, uint64_t lo)
{
    return static_cast<CsgCompilerInt128>((static_cast<CsgCompilerUInt128>(static_cast<uint64_t>(hi)) << 64) | lo);
}

CsgCompilerInt128 ToNative(const CsgInt128Portable& value)
{
    return MakeInt(static_cast<const CsgCompilerInt128*>(nullptr), value.hi_, value.lo_);
}
#endif

template <typename Int>
Int Make(int64_t hi, uint64_t lo)
{
    return MakeInt(static_cast<const Int*>(nullptr), hi, lo);
}

CsgGridPoint P(int64_t x, int64_t y, int64_t z)
{
    CsgGridPoint point;
    point.x_ = x;
    point.y_ = y;
    point.z_ = z;
    return point;
}

constexpr int64_t Unit = 1 << 24;

} // namespace

TEST_CASE("Portable 128 bit product matches the compiler type", "[exact]")
{
    REQUIRE(CsgInt128Portable::Multiply(3, -4) == CsgInt128Portable(-12));
    REQUIRE(CsgInt128Portable::Multiply(-3, -4) == CsgInt128Portable(12));
    const auto minimum = std::numeric_limits<int64_t>::min();
    // (-2^63) * (-2^63) = 2^126
    REQUIRE(CsgInt128Portable::Multiply(minimum, minimum) == CsgInt128Portable::FromParts(0, int64_t(1) << 62));
    REQUIRE(CsgInt128Portable::Multiply(minimum, 1) == CsgInt128Portable(minimum));

#ifdef __SIZEOF_INT128__
    std::mt19937_64 random(12345);
    const int64_t bound = int64_t(1) << 62;
    std::uniform_int_distribution<int64_t> distribution(-bound + 1, bound - 1);
    for (int i = 0; i < 10000; ++i)
    {
        const int64_t a = distribution(random);
        const int64_t b = distribution(random);
        const CsgCompilerInt128 expected = static_cast<CsgCompilerInt128>(a) * b;
        const CsgInt128Portable product = CsgInt128Portable::Multiply(a, b);
        REQUIRE(ToNative(product) == expected);
        REQUIRE(ToNative(CsgInt128Portable(a) * CsgInt128Portable(b)) == expected);
        REQUIRE(ToNative(CsgMul64<CsgInt128Portable>(a, b)) == expected);
    }

    // Wrapping arithmetic on full 128 bit values.
    std::uniform_int_distribution<uint64_t> bits;
    for (int i = 0; i < 10000; ++i)
    {
        const CsgInt128Portable x = CsgInt128Portable::FromParts(bits(random), static_cast<int64_t>(bits(random)));
        const CsgInt128Portable y = CsgInt128Portable::FromParts(bits(random), static_cast<int64_t>(bits(random)));
        const int64_t small = static_cast<int64_t>(bits(random));
        const CsgCompilerUInt128 ux = static_cast<CsgCompilerUInt128>(ToNative(x));
        const CsgCompilerUInt128 uy = static_cast<CsgCompilerUInt128>(ToNative(y));
        const CsgCompilerUInt128 us = static_cast<CsgCompilerUInt128>(static_cast<CsgCompilerInt128>(small));
        REQUIRE(static_cast<CsgCompilerUInt128>(ToNative(x + y)) == ux + uy);
        REQUIRE(static_cast<CsgCompilerUInt128>(ToNative(x - y)) == ux - uy);
        REQUIRE(static_cast<CsgCompilerUInt128>(ToNative(-x)) == CsgCompilerUInt128(0) - ux);
        REQUIRE(static_cast<CsgCompilerUInt128>(ToNative(x * y)) == ux * uy);
        REQUIRE(static_cast<CsgCompilerUInt128>(ToNative(CsgMul(x, small))) == ux * us);
        REQUIRE((x < y) == (ToNative(x) < ToNative(y)));
        REQUIRE((x >= y) == (ToNative(x) >= ToNative(y)));
        REQUIRE(x.Sign() == CsgSign(ToNative(x)));
    }
#endif
}

TEMPLATE_LIST_TEST_CASE("Sign of product difference", "[exact]", CsgTestTypes)
{
    using Int = TestType;

    SECTION("Random values that fit in the compiler type")
    {
#ifdef __SIZEOF_INT128__
        std::mt19937_64 random(777);
        const int64_t bound = int64_t(1) << 62;
        std::uniform_int_distribution<int64_t> distribution(-bound + 1, bound - 1);
        for (int i = 0; i < 10000; ++i)
        {
            const int64_t a = distribution(random);
            const int64_t b = distribution(random);
            // Make equal products often.
            const bool equal = (i % 4) == 0;
            const int64_t c = equal ? b : distribution(random);
            const int64_t d = equal ? a : distribution(random);
            const CsgCompilerInt128 difference = static_cast<CsgCompilerInt128>(a) * b - static_cast<CsgCompilerInt128>(c) * d;
            const int expected = difference > 0 ? 1 : (difference < 0 ? -1 : 0);
            REQUIRE(CsgSignOfProductDifference<Int>(Int(a), Int(b), Int(c), Int(d)) == expected);
        }
#endif
    }

    SECTION("Hand cases at 2^127 scale")
    {
        const Int pow126 = Make<Int>(int64_t(1) << 62, 0);
        const Int minimum = Make<Int>(std::numeric_limits<int64_t>::min(), 0);
        const Int maximum = Make<Int>(std::numeric_limits<int64_t>::max(), ~uint64_t(0));
        const Int one = Int(1);
        const Int two = Int(2);
        const Int minusOne = Int(-1);

        // 2^127 - 2^127 = 0
        REQUIRE(CsgSignOfProductDifference<Int>(pow126, two, pow126, two) == 0);
        REQUIRE(CsgSignOfProductDifference<Int>(pow126, two, pow126, one) == 1);
        REQUIRE(CsgSignOfProductDifference<Int>(pow126, one, pow126, two) == -1);
        // (-2^127) * 1 - (2^127 - 1) * (-1) = -1
        REQUIRE(CsgSignOfProductDifference<Int>(minimum, one, maximum, minusOne) == -1);
        // (-2^127)^2 = 2^254 is larger than (2^127 - 1)^2
        REQUIRE(CsgSignOfProductDifference<Int>(minimum, minimum, maximum, maximum) == 1);
        REQUIRE(CsgSignOfProductDifference<Int>(maximum, maximum, minimum, minimum) == -1);
        REQUIRE(CsgSignOfProductDifference<Int>(minimum, minimum, minimum, minimum) == 0);
        // Products that differ only in the lowest bit of a 256 bit value.
        REQUIRE(CsgSignOfProductDifference<Int>(maximum, maximum, maximum, Make<Int>(std::numeric_limits<int64_t>::max(), ~uint64_t(0) - 1)) == 1);
        // Opposite signs: -(2^127) - (2^127 - 1) < 0
        REQUIRE(CsgSignOfProductDifference<Int>(minimum, one, maximum, one) == -1);
        REQUIRE(CsgSignOfProductDifference<Int>(maximum, one, minimum, one) == 1);
        // Equal magnitudes with negative products: (-a) * b - (-c) * d where a * b == c * d
        REQUIRE(CsgSignOfProductDifference<Int>(Int(-6), Int(7), Int(-14), Int(3)) == 0);
        REQUIRE(CsgSignOfProductDifference<Int>(Int(-6), Int(7), Int(-14), Int(2)) == -1);
    }

    SECTION("Zero cases")
    {
        const Int zero = Int(0);
        const Int five = Int(5);
        const Int minusFive = Int(-5);
        REQUIRE(CsgSignOfProductDifference<Int>(zero, five, zero, minusFive) == 0);
        REQUIRE(CsgSignOfProductDifference<Int>(zero, five, five, five) == -1);
        REQUIRE(CsgSignOfProductDifference<Int>(zero, five, five, minusFive) == 1);
        REQUIRE(CsgSignOfProductDifference<Int>(five, five, zero, zero) == 1);
        REQUIRE(CsgSignOfProductDifference<Int>(minusFive, five, zero, five) == -1);
        REQUIRE(CsgSignOfProductDifference<Int>(five, minusFive, five, minusFive) == 0);
    }
}

TEMPLATE_LIST_TEST_CASE("Orient on tetrahedra", "[exact]", CsgTestTypes)
{
    using Int = TestType;

    const CsgGridPoint a = P(0, 0, 0);
    const CsgGridPoint b = P(Unit, 0, 0);
    const CsgGridPoint c = P(0, Unit, 0);
    const CsgGridPoint d = P(0, 0, Unit);

    REQUIRE(CsgExact::OrientT<Int>(a, b, c, d, 0) == 1);
    REQUIRE(CsgExact::OrientT<Int>(a, c, b, d, 0) == -1);
    // The perturbation never changes a nonzero determinant.
    for (int second = 0; second < 16; ++second)
    {
        REQUIRE(CsgExact::OrientT<Int>(a, b, c, d, second) == 1);
        REQUIRE(CsgExact::OrientT<Int>(a, c, b, d, second) == -1);
    }

    SECTION("Coplanar points")
    {
        const CsgGridPoint coplanar = P(Unit, Unit, 0);
        REQUIRE(CsgExact::OrientT<Int>(a, b, c, coplanar, 0) == 0);
        REQUIRE(CsgExact::OrientT<Int>(a, b, c, coplanar, 0b1111) == 0);
        // Moving d by (eps, eps^2, eps^3) lifts it by eps^3: u x v = (0, 0, 1) gives plus.
        REQUIRE(CsgExact::OrientT<Int>(a, b, c, coplanar, 0b1000) == 1);
        // Moving a lowers the plane through b and c by the same amount: G = -(v x w + w x u + u x v) = (0, 0, +1).
        REQUIRE(CsgExact::OrientT<Int>(a, b, c, coplanar, 0b0001) == 1);
        // Moving b and c: weights (1, 1, 0), the sum v x w + w x u has z = -2.
        REQUIRE(CsgExact::OrientT<Int>(a, b, c, coplanar, 0b0110) == -1);
        // The same plane with a normal along x: the first component of G decides.
        const CsgGridPoint b2 = P(0, Unit, 0);
        const CsgGridPoint c2 = P(0, 0, Unit);
        REQUIRE(CsgExact::OrientT<Int>(a, b2, c2, P(0, Unit, Unit), 0b1000) == 1);
        REQUIRE(CsgExact::OrientT<Int>(a, c2, b2, P(0, Unit, Unit), 0b1000) == -1);
        // Collinear points have no gradient.
        REQUIRE(CsgExact::OrientT<Int>(a, b, P(2 * Unit, 0, 0), P(3 * Unit, 0, 0), 0b1010) == 0);
    }

    SECTION("Volume and normal")
    {
        const CsgNormal<Int> normal = CsgExact::NormalT<Int>(a, b, c);
        REQUIRE(normal.x_ == Int(0));
        REQUIRE(normal.y_ == Int(0));
        REQUIRE(normal.z_ == CsgMul64<Int>(Unit, Unit));
        REQUIRE(CsgExact::VolumeT<Int>(a, b, c, d) == CsgMul(CsgMul64<Int>(Unit, Unit), Unit));
        REQUIRE(CsgExact::VolumeT<Int>(a, c, b, d) == -CsgMul(CsgMul64<Int>(Unit, Unit), Unit));
        REQUIRE(CsgExact::VolumeT<Int>(a, b, c, P(5, 5, 0)) == Int(0));
    }
}

TEMPLATE_LIST_TEST_CASE("Orient perturbation matches the numeric displacement", "[exact]", CsgTestTypes)
{
    using Int = TestType;

    // The determinant is linear in the displacement, so a small numeric epsilon reproduces the symbolic one.
    std::mt19937 random(99);
    std::uniform_int_distribution<int> coordinate(-4, 4);
    std::uniform_int_distribution<int> factor(-2, 2);
    const long double eps = 1e-4L;
    int nonzero = 0;
    for (int i = 0; i < 3000; ++i)
    {
        const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
        const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
        const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
        const int s = factor(random);
        const int t = factor(random);
        const CsgGridPoint d = P(a.x_ + s * (b.x_ - a.x_) + t * (c.x_ - a.x_),
            a.y_ + s * (b.y_ - a.y_) + t * (c.y_ - a.y_), a.z_ + s * (b.z_ - a.z_) + t * (c.z_ - a.z_));
        REQUIRE(CsgExact::OrientT<Int>(a, b, c, d, 0) == 0);

        for (int second = 1; second < 15; ++second)
        {
            long double p[4][3] = {{(long double)a.x_, (long double)a.y_, (long double)a.z_},
                {(long double)b.x_, (long double)b.y_, (long double)b.z_},
                {(long double)c.x_, (long double)c.y_, (long double)c.z_},
                {(long double)d.x_, (long double)d.y_, (long double)d.z_}};
            for (int k = 0; k < 4; ++k)
            {
                if (((second >> k) & 1) == 0)
                    continue;
                p[k][0] += eps;
                p[k][1] += eps * eps;
                p[k][2] += eps * eps * eps;
            }
            long double u[3], v[3], w[3];
            for (int k = 0; k < 3; ++k)
            {
                u[k] = p[1][k] - p[0][k];
                v[k] = p[2][k] - p[0][k];
                w[k] = p[3][k] - p[0][k];
            }
            const long double det = u[0] * (v[1] * w[2] - v[2] * w[1]) + u[1] * (v[2] * w[0] - v[0] * w[2])
                + u[2] * (v[0] * w[1] - v[1] * w[0]);
            const int result = CsgExact::OrientT<Int>(a, b, c, d, second);
            if (result == 0)
            {
                // A 53 bit long double is too coarse for this bound.
                if (std::numeric_limits<long double>::digits >= 64)
                {
                    REQUIRE(std::abs(det) < 1e-15L);
                }
            }
            else
            {
                ++nonzero;
                REQUIRE(result == (det > 0 ? 1 : -1));
            }
        }
    }
    REQUIRE(nonzero > 1000);
}

TEST_CASE("Orient results are equal for both 128 bit types", "[exact]")
{
#if CSG_HAS_NATIVE_INT128
    std::mt19937_64 random(4242);
    // Cover small values with many degenerate cases and values near the limit of 2^40.
    for (const int64_t range : {int64_t(3), int64_t(1) << 20, (int64_t(1) << 39) - 1})
    {
        std::uniform_int_distribution<int64_t> coordinate(-range, range);
        for (int i = 0; i < 2000; ++i)
        {
            const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint d = P(coordinate(random), coordinate(random), coordinate(random));
            const int second = static_cast<int>(random() % 16);
            REQUIRE(CsgExact::OrientT<CsgInt128Portable>(a, b, c, d, second)
                == CsgExact::OrientT<CsgCompilerInt128>(a, b, c, d, second));
            // The volume has no filter. A nonzero volume gives the sign of Orient for every mask.
            const int volumeSign = CsgSign(CsgExact::VolumeT<CsgCompilerInt128>(a, b, c, d));
            if (volumeSign != 0)
                REQUIRE(CsgExact::OrientT<CsgInt128Portable>(a, b, c, d, second) == volumeSign);
            REQUIRE(CsgSign(CsgExact::VolumeT<CsgInt128Portable>(a, b, c, d)) == volumeSign);
            REQUIRE(ToNative(CsgExact::VolumeT<CsgInt128Portable>(a, b, c, d))
                == CsgExact::VolumeT<CsgCompilerInt128>(a, b, c, d));
            const CsgNormal<CsgInt128Portable> np = CsgExact::NormalT<CsgInt128Portable>(a, b, c);
            const CsgNormal<CsgCompilerInt128> nn = CsgExact::NormalT<CsgCompilerInt128>(a, b, c);
            REQUIRE(ToNative(np.x_) == nn.x_);
            REQUIRE(ToNative(np.y_) == nn.y_);
            REQUIRE(ToNative(np.z_) == nn.z_);
        }
    }
#endif
}

TEST_CASE("Exact filter", "[exact]")
{
    // Use big coordinates so that the products in double round.
    const int64_t big = int64_t(1) << 38;

    const auto runFilter = [](const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d,
                               double& det)
    {
        const CsgDouble4 ux = CsgDouble4::Splat(double(b.x_ - a.x_));
        const CsgDouble4 uy = CsgDouble4::Splat(double(b.y_ - a.y_));
        const CsgDouble4 uz = CsgDouble4::Splat(double(b.z_ - a.z_));
        const CsgDouble4 vx = CsgDouble4::Splat(double(c.x_ - a.x_));
        const CsgDouble4 vy = CsgDouble4::Splat(double(c.y_ - a.y_));
        const CsgDouble4 vz = CsgDouble4::Splat(double(c.z_ - a.z_));
        const CsgDouble4 wx = CsgDouble4::Splat(double(d.x_ - a.x_));
        const CsgDouble4 wy = CsgDouble4::Splat(double(d.y_ - a.y_));
        const CsgDouble4 wz = CsgDouble4::Splat(double(d.z_ - a.z_));
        CsgDouble4 result;
        const int decided = CsgInternal::Filter(ux, uy, uz, vx, vy, vz, wx, wy, wz, result);
        det = result.Get(0);
        return decided;
    };

    SECTION("Four coplanar points force the exact path")
    {
        const CsgGridPoint a = P(3, -5, 7);
        const CsgGridPoint b = P(big + 3, big / 3 - 5, big / 7 + 7);
        const CsgGridPoint c = P(-big / 5 + 3, big - 5, big / 11 + 7);
        // d = a + 2 (b - a) - 3 (c - a)
        const CsgGridPoint d = P(a.x_ + 2 * (b.x_ - a.x_) - 3 * (c.x_ - a.x_),
            a.y_ + 2 * (b.y_ - a.y_) - 3 * (c.y_ - a.y_), a.z_ + 2 * (b.z_ - a.z_) - 3 * (c.z_ - a.z_));
        double det = 0.0;
        REQUIRE((runFilter(a, b, c, d, det) & 1) == 0);
        REQUIRE(CsgExact::Orient(a, b, c, d, 0) == 0);
        REQUIRE(CsgExact::OrientT<CsgInt128Portable>(a, b, c, d, 0) == 0);
        // The volume has no filter. It is exactly zero.
        REQUIRE(CsgSign(CsgExact::VolumeT<CsgInt128Portable>(a, b, c, d)) == 0);
        // One grid unit above the plane: the exact sign is nonzero.
        const CsgGridPoint above = P(d.x_, d.y_, d.z_ + 1);
        const int exactSign = CsgSign(CsgExact::VolumeT<CsgInt128Portable>(a, b, c, above));
        REQUIRE(exactSign != 0);
        REQUIRE(CsgExact::Orient(a, b, c, above, 0) == exactSign);
        REQUIRE(CsgExact::OrientT<CsgInt128Portable>(a, b, c, above, 0) == exactSign);
    }

    SECTION("Clear cases are decided")
    {
        double det = 0.0;
        REQUIRE((runFilter(P(0, 0, 0), P(Unit, 0, 0), P(0, Unit, 0), P(0, 0, Unit), det) & 1) != 0);
        REQUIRE(det > 0.0);
        REQUIRE((runFilter(P(0, 0, 0), P(0, Unit, 0), P(Unit, 0, 0), P(0, 0, Unit), det) & 1) != 0);
        REQUIRE(det < 0.0);
    }

    SECTION("Filter and exact path agree with the exact path alone")
    {
        std::mt19937_64 random(2024);
        std::uniform_int_distribution<int64_t> coordinate(-big, big);
        std::uniform_int_distribution<int64_t> jitter(-2, 2);
        std::uniform_int_distribution<int> factor(-3, 3);
        int undecided = 0;
        int exactDecided = 0;
        for (int i = 0; i < 1000; ++i)
        {
            const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
            CsgGridPoint d = P(coordinate(random), coordinate(random), coordinate(random));
            if (i % 2 == 0)
            {
                // Nearly coplanar: a combination of the edges, rounded to the grid, plus a small jitter.
                const double s = factor(random) * 0.25;
                const double t = factor(random) * 0.5;
                const auto mix = [&](int64_t pa, int64_t pb, int64_t pc)
                { return pa + int64_t(s * double(pb - pa) + t * double(pc - pa)) + jitter(random); };
                d = P(mix(a.x_, b.x_, c.x_), mix(a.y_, b.y_, c.y_), mix(a.z_, b.z_, c.z_));
            }
            double det = 0.0;
            const int decided = runFilter(a, b, c, d, det);
            const int viaFilter = (decided & 1) != 0 ? (det > 0.0 ? 1 : -1) : CsgExact::Orient(a, b, c, d, 0);
            // The volume has no filter. Its sign is the sign of Orient with second = 0.
            const int exactOnly = CsgSign(CsgExact::VolumeT<CsgInt128Portable>(a, b, c, d));
            if ((decided & 1) != 0)
                REQUIRE((det > 0.0 ? 1 : -1) == exactOnly);
            REQUIRE(viaFilter == exactOnly);
            REQUIRE(CsgExact::OrientT<CsgInt128Portable>(a, b, c, d, 0) == exactOnly);
            REQUIRE(CsgExact::Orient(a, b, c, d, 0) == exactOnly);
            if ((decided & 1) == 0)
            {
                ++undecided;
                // The filter does not decide here, so only the exact path gives this nonzero sign.
                if (exactOnly != 0)
                    ++exactDecided;
            }
        }
        // Some of the nearly coplanar cases take the exact path.
        REQUIRE(undecided > 20);
        REQUIRE(exactDecided > 10);
    }

    SECTION("Lanes are independent")
    {
        const CsgDouble4 one = CsgDouble4::Splat(1.0);
        const CsgDouble4 zero = CsgDouble4::Splat(0.0);
        // Lane 0 has det = 1, lane 1 has det = -1, lane 2 has det = 0, lane 3 is ignored.
        const CsgDouble4 ux = CsgDouble4::Create(1.0, -1.0, 0.0, 5.0);
        CsgDouble4 det;
        const int decided = CsgInternal::Filter(ux, zero, zero, zero, one, zero, zero, zero, one, det);
        REQUIRE(decided == 0b011);
        REQUIRE(det.Get(0) == 1.0);
        REQUIRE(det.Get(1) == -1.0);
        REQUIRE(det.Get(2) == 0.0);
    }
}

TEST_CASE("Simd double vector", "[exact]")
{
    const CsgDouble4 a = CsgDouble4::Create(1.0, -2.0, 3.0, -4.0);
    const CsgDouble4 b = CsgDouble4::Create(0.5, 2.0, -3.0, std::numeric_limits<double>::quiet_NaN());
    double out[4];
    (a + b).Store(out);
    REQUIRE(out[0] == 1.5);
    REQUIRE(out[1] == 0.0);
    REQUIRE(out[2] == 0.0);
    REQUIRE(std::isnan(out[3]));
    (a - b).Store(out);
    REQUIRE(out[0] == 0.5);
    REQUIRE(out[1] == -4.0);
    REQUIRE(out[2] == 6.0);
    (a * b).Store(out);
    REQUIRE(out[0] == 0.5);
    REQUIRE(out[1] == -4.0);
    REQUIRE(out[2] == -9.0);
    CsgDouble4::Abs(a).Store(out);
    REQUIRE(out[0] == 1.0);
    REQUIRE(out[1] == 2.0);
    REQUIRE(out[2] == 3.0);
    REQUIRE(out[3] == 4.0);
    // Lane 3 compares with NaN and gives a clear bit.
    REQUIRE(CsgDouble4::GreaterThanMask(a, b) == 0b0101);
    REQUIRE(CsgDouble4::GreaterThanMask(b, a) == 0b0010);
    REQUIRE(a.Get(2) == 3.0);
}

TEST_CASE("Grid point snapping", "[exact]")
{
    const auto from = [](float x, float y, float z) { return CsgGridPoint::From(Vector3(x, y, z)); };

    REQUIRE(from(0.5f, -0.5f, 0.0f) == P(Unit / 2, -Unit / 2, 0));
    REQUIRE(from(1.0f, -1.0f, 2.0f) == P(Unit, -Unit, 2 * Unit));
    // 1e-8 * 2^24 = 0.168 rounds to zero.
    REQUIRE(from(1e-8f, -1e-8f, 0.0f) == P(0, 0, 0));
    // 65535.99f is the float 65535.98828125 = 16777213 / 256, so the grid value is exact.
    REQUIRE(from(65535.99f, -65535.99f, 0.0f) == P(int64_t(16777213) << 16, -(int64_t(16777213) << 16), 0));

    // A value between two grid points goes to the even one: 0.5, 1.5, 2.5 and -0.5, -1.5 grid units.
    const float step = 1.0f / float(Unit);
    REQUIRE(from(0.5f * step, 1.5f * step, 2.5f * step) == P(0, 2, 2));
    REQUIRE(from(-0.5f * step, -1.5f * step, -2.5f * step) == P(0, -2, -2));
    // Values off the tie go to the nearest grid point.
    REQUIRE(from(0.75f * step, -0.75f * step, 3.25f * step) == P(1, -1, 3));
}

TEST_CASE("Exact crossing tests", "[exact]")
{
    const CsgGridPoint t0 = P(0, 0, 0);
    const CsgGridPoint t1 = P(4 * Unit, 0, 0);
    const CsgGridPoint t2 = P(0, 4 * Unit, 0);

    SECTION("An edge through the triangle")
    {
        const CsgGridPoint e0 = P(Unit, Unit, -Unit);
        const CsgGridPoint e1 = P(Unit, Unit, Unit);
        bool above = false;
        REQUIRE(CsgExact::Crosses(e0, e1, t0, t1, t2, false, above));
        REQUIRE(above);
        REQUIRE(CsgExact::Crosses(e0, e1, t0, t1, t2, true, above));
        REQUIRE(CsgExact::Crosses(e1, e0, t0, t1, t2, false, above));
        REQUIRE_FALSE(above);
        REQUIRE(CsgExact::CrossesT<CsgInt128Portable>(e0, e1, t0, t1, t2, false, above));
        REQUIRE(CsgExact::PassesInside(e0, e1, t0, t1, t2, false));
        // The orientation of the triangle does not matter for the inside test.
        REQUIRE(CsgExact::PassesInside(e0, e1, t0, t2, t1, false));
    }

    SECTION("An edge beside the triangle or short of the plane")
    {
        bool above = false;
        REQUIRE_FALSE(CsgExact::Crosses(P(3 * Unit, 3 * Unit, -Unit), P(3 * Unit, 3 * Unit, Unit), t0, t1, t2, false, above));
        REQUIRE_FALSE(CsgExact::Crosses(P(Unit, Unit, Unit), P(Unit, Unit, 2 * Unit), t0, t1, t2, false, above));
        REQUIRE(above);
        REQUIRE_FALSE(CsgExact::PassesInside(P(3 * Unit, 3 * Unit, -Unit), P(3 * Unit, 3 * Unit, Unit), t0, t1, t2, false));
        REQUIRE_FALSE(CsgExact::PassesInsideT<CsgInt128Portable>(
            P(3 * Unit, 3 * Unit, -Unit), P(3 * Unit, 3 * Unit, Unit), t0, t1, t2, false));
    }

    SECTION("An edge through a vertex is claimed by the displacement")
    {
        // The edge passes the vertex t0. The displaced operand decides which side. The result is the same for both types.
        const CsgGridPoint e0 = P(0, 0, -Unit);
        const CsgGridPoint e1 = P(0, 0, Unit);
        for (const bool edgeInSecond : {false, true})
        {
            bool aboveA = false;
            bool aboveB = false;
            REQUIRE(CsgExact::Crosses(e0, e1, t0, t1, t2, edgeInSecond, aboveA)
                == CsgExact::CrossesT<CsgInt128Portable>(e0, e1, t0, t1, t2, edgeInSecond, aboveB));
            REQUIRE(aboveA == aboveB);
        }
    }
}

} // namespace Urho3D
