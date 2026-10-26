// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgExact.h"
#include "Urho3D/CSG/CsgExactFilter.h"
#include "Urho3D/CSG/CsgExpansion.h"

#include "TestExactReference.h"

#include <catch2/catch_amalgamated.hpp>

#include <algorithm>
#include <cmath>
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

constexpr int64_t Unit = 1 << 10;

/// Crosses with the oracle orientation. The structure is the same as in the library.
bool ReferencePassesInside(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0,
    const CsgGridPoint& t1, const CsgGridPoint& t2, bool edgeInSecond)
{
    const int linesMask = edgeInSecond ? 0b0011 : 0b1100;
    const int s0 = CsgTest::ReferenceOrient(e0, e1, t0, t1, linesMask);
    const int s1 = CsgTest::ReferenceOrient(e0, e1, t1, t2, linesMask);
    const int s2 = CsgTest::ReferenceOrient(e0, e1, t2, t0, linesMask);
    return s0 != 0 && s1 == s0 && s2 == s0;
}

bool ReferenceCrosses(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1,
    const CsgGridPoint& t2, bool edgeInSecond, bool& e1Above)
{
    const int sidesMask = edgeInSecond ? 0b1000 : 0b0111;
    const int s0 = CsgTest::ReferenceOrient(t0, t1, t2, e0, sidesMask);
    const int s1 = CsgTest::ReferenceOrient(t0, t1, t2, e1, sidesMask);
    e1Above = s1 > 0;
    if (s0 == 0 || s1 == 0 || s0 == s1)
        return false;
    return ReferencePassesInside(e0, e1, t0, t1, t2, edgeInSecond);
}

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

TEST_CASE("Orient on tetrahedra", "[exact]")
{
    const CsgGridPoint a = P(0, 0, 0);
    const CsgGridPoint b = P(Unit, 0, 0);
    const CsgGridPoint c = P(0, Unit, 0);
    const CsgGridPoint d = P(0, 0, Unit);

    REQUIRE(CsgExact::Orient(a, b, c, d, 0) == 1);
    REQUIRE(CsgExact::Orient(a, c, b, d, 0) == -1);
    // The perturbation never changes a nonzero determinant.
    for (int second = 0; second < 16; ++second)
    {
        REQUIRE(CsgExact::Orient(a, b, c, d, second) == 1);
        REQUIRE(CsgExact::Orient(a, c, b, d, second) == -1);
    }

    SECTION("Coplanar points")
    {
        const CsgGridPoint coplanar = P(Unit, Unit, 0);
        REQUIRE(CsgExact::Orient(a, b, c, coplanar, 0) == 0);
        REQUIRE(CsgExact::Orient(a, b, c, coplanar, 0b1111) == 0);
        // Moving d by (eps, eps^2, eps^3) lifts it by eps^3: u x v = (0, 0, 1) gives plus.
        REQUIRE(CsgExact::Orient(a, b, c, coplanar, 0b1000) == 1);
        // Moving a lowers the plane through b and c by the same amount: G = -(v x w + w x u + u x v) = (0, 0, +1).
        REQUIRE(CsgExact::Orient(a, b, c, coplanar, 0b0001) == 1);
        // Moving b and c: weights (1, 1, 0), the sum v x w + w x u has z = -2.
        REQUIRE(CsgExact::Orient(a, b, c, coplanar, 0b0110) == -1);
        // The same plane with a normal along x: the first component of G decides.
        const CsgGridPoint b2 = P(0, Unit, 0);
        const CsgGridPoint c2 = P(0, 0, Unit);
        REQUIRE(CsgExact::Orient(a, b2, c2, P(0, Unit, Unit), 0b1000) == 1);
        REQUIRE(CsgExact::Orient(a, c2, b2, P(0, Unit, Unit), 0b1000) == -1);
        // Collinear points have no gradient.
        REQUIRE(CsgExact::Orient(a, b, P(2 * Unit, 0, 0), P(3 * Unit, 0, 0), 0b1010) == 0);
    }
}

TEMPLATE_LIST_TEST_CASE("Volume and normal on tetrahedra", "[exact]", CsgTestTypes)
{
    using Int = TestType;

    const CsgGridPoint a = P(0, 0, 0);
    const CsgGridPoint b = P(Unit, 0, 0);
    const CsgGridPoint c = P(0, Unit, 0);
    const CsgGridPoint d = P(0, 0, Unit);

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

TEST_CASE("Orient perturbation matches the numeric displacement", "[exact]")
{
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
        REQUIRE(CsgExact::Orient(a, b, c, d, 0) == 0);

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
            const int result = CsgExact::Orient(a, b, c, d, second);
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

TEST_CASE("Volume and normal results are equal for both 128 bit types", "[exact]")
{
#if CSG_HAS_NATIVE_INT128
    std::mt19937_64 random(4242);
    // Cover small values with many degenerate cases and values near the limit of 2^23.
    for (const int64_t range : {int64_t(3), int64_t(1) << 20, (int64_t(1) << 23) - 1})
    {
        std::uniform_int_distribution<int64_t> coordinate(-range, range);
        for (int i = 0; i < 2000; ++i)
        {
            const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint d = P(coordinate(random), coordinate(random), coordinate(random));
            const int second = static_cast<int>(random() % 16);
            // The volume has no filter. A nonzero volume gives the sign of Orient for every mask.
            const int volumeSign = CsgSign(CsgExact::VolumeT<CsgCompilerInt128>(a, b, c, d));
            if (volumeSign != 0)
                REQUIRE(CsgExact::Orient(a, b, c, d, second) == volumeSign);
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

TEST_CASE("Reference oracle matches the library Orient", "[exact]")
{
    std::mt19937_64 random(8086);
    // Small values give many degenerate cases. Large values stay inside the grid range of 2^23.
    for (const int64_t range : {int64_t(3), int64_t(1) << 10, (int64_t(1) << 23) - 1})
    {
        std::uniform_int_distribution<int64_t> coordinate(-range, range);
        for (int i = 0; i < 3000; ++i)
        {
            const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint d = P(coordinate(random), coordinate(random), coordinate(random));
            for (int second = 0; second < 16; ++second)
                REQUIRE(CsgExact::Orient(a, b, c, d, second) == CsgTest::ReferenceOrient(a, b, c, d, second));
            REQUIRE(CsgSign(CsgExact::Volume(a, b, c, d)) == CsgTest::Sign(CsgTest::ReferenceVolume(a, b, c, d)));
            // The perturbation never changes a nonzero volume.
            const int volumeSign = CsgTest::Sign(CsgTest::ReferenceVolume(a, b, c, d));
            if (volumeSign != 0)
                REQUIRE(CsgTest::ReferenceOrient(a, b, c, d, 0) == volumeSign);
        }
    }
}

TEST_CASE("Orient matches the int128 oracle", "[exact]")
{
    std::mt19937_64 random(20240601);
    const int64_t limit = (int64_t(1) << 23) - 1;
    for (const int64_t range : {int64_t(3), int64_t(1) << 10, limit})
    {
        std::uniform_int_distribution<int64_t> coordinate(-range, range);
        std::uniform_int_distribution<int> small(-1, 1);
        std::uniform_int_distribution<int> axisPick(0, 2);
        std::uniform_int_distribution<int> extremePick(0, 5);
        const auto inRange = [&](const CsgGridPoint& p)
        { return std::abs(p.x_) <= range && std::abs(p.y_) <= range && std::abs(p.z_) <= range; };
        const auto randomPoint = [&]() { return P(coordinate(random), coordinate(random), coordinate(random)); };
        // Coordinates at the limits of the range and next to them.
        const auto extremeValue = [&]()
        {
            const int64_t values[6] = {range, -range, range - 1, -range + 1, 0, 1};
            return values[extremePick(random)];
        };
        const auto extremePoint = [&]() { return P(extremeValue(), extremeValue(), extremeValue()); };
        const auto combination = [](const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, int i, int j)
        {
            return P(a.x_ + i * (b.x_ - a.x_) + j * (c.x_ - a.x_), a.y_ + i * (b.y_ - a.y_) + j * (c.y_ - a.y_),
                a.z_ + i * (b.z_ - a.z_) + j * (c.z_ - a.z_));
        };

        int nonzeroPerturbed = 0;
        for (int n = 0; n < 20000; ++n)
        {
            CsgGridPoint a = randomPoint();
            CsgGridPoint b = randomPoint();
            CsgGridPoint c = randomPoint();
            CsgGridPoint d = randomPoint();
            switch (n % 10)
            {
            case 0: // Random points.
                break;
            case 1: // Coplanar.
            case 2:
            {
                const CsgGridPoint moved = combination(a, b, c, small(random), small(random));
                if (inRange(moved))
                    d = moved;
                break;
            }
            case 3: // Collinear.
            {
                const CsgGridPoint onLine = combination(a, b, b, small(random), 0);
                if (inRange(onLine))
                    c = onLine;
                const CsgGridPoint second = combination(a, b, b, small(random), 0);
                if (inRange(second))
                    d = second;
                break;
            }
            case 4: // Duplicate points.
            {
                const CsgGridPoint* sources[3] = {&a, &b, &c};
                CsgGridPoint* targets[3] = {&b, &c, &d};
                *targets[n / 10 % 3] = *sources[(n / 30) % (n / 10 % 3 + 1)];
                break;
            }
            case 5: // Coplanar points moved by one unit on one axis.
            case 6:
            {
                CsgGridPoint moved = combination(a, b, c, small(random), small(random));
                const int axis = axisPick(random);
                const int64_t step = (random() & 1) != 0 ? 1 : -1;
                (axis == 0 ? moved.x_ : axis == 1 ? moved.y_ : moved.z_) += step;
                if (inRange(moved))
                    d = moved;
                break;
            }
            case 7: // Extreme coordinates.
                a = extremePoint();
                b = extremePoint();
                c = extremePoint();
                d = extremePoint();
                break;
            case 8: // Extreme coordinates with a coplanar point.
            {
                a = extremePoint();
                b = extremePoint();
                c = extremePoint();
                const CsgGridPoint moved = combination(a, b, c, small(random), small(random));
                d = inRange(moved) ? moved : extremePoint();
                break;
            }
            default: // All points equal, or three equal.
                b = a;
                if (n % 20 < 10)
                {
                    c = a;
                    d = a;
                }
                break;
            }

            for (int second = 0; second < 16; ++second)
            {
                const int expected = CsgTest::ReferenceOrient(a, b, c, d, second);
                REQUIRE(CsgExact::Orient(a, b, c, d, second) == expected);
                if (second != 0 && second != 15 && CsgTest::ReferenceOrient(a, b, c, d, 0) == 0 && expected != 0)
                    ++nonzeroPerturbed;
            }
        }
        // The perturbation resolves many degenerate cases.
        REQUIRE(nonzeroPerturbed > 1000);
    }
}

TEST_CASE("Crosses and PassesInside match the int128 oracle", "[exact]")
{
    std::mt19937_64 random(777);
    const int64_t limit = (int64_t(1) << 23) - 1;
    int crossings = 0;
    for (const int64_t range : {int64_t(3), int64_t(1) << 10, limit})
    {
        std::uniform_int_distribution<int64_t> coordinate(-range, range);
        std::uniform_int_distribution<int> small(-1, 1);
        const auto randomPoint = [&]() { return P(coordinate(random), coordinate(random), coordinate(random)); };
        for (int n = 0; n < 20000; ++n)
        {
            const CsgGridPoint t0 = randomPoint();
            const CsgGridPoint t1 = randomPoint();
            const CsgGridPoint t2 = randomPoint();
            CsgGridPoint e0 = randomPoint();
            CsgGridPoint e1 = randomPoint();
            if (n % 4 == 1)
            {
                // An end point in the plane of the triangle.
                const int i = small(random);
                const int j = small(random);
                const CsgGridPoint moved = P(t0.x_ + i * (t1.x_ - t0.x_) + j * (t2.x_ - t0.x_),
                    t0.y_ + i * (t1.y_ - t0.y_) + j * (t2.y_ - t0.y_), t0.z_ + i * (t1.z_ - t0.z_) + j * (t2.z_ - t0.z_));
                if (std::abs(moved.x_) <= range && std::abs(moved.y_) <= range && std::abs(moved.z_) <= range)
                    e1 = moved;
            }
            else if (n % 4 == 2)
            {
                // An edge that ends at a vertex of the triangle.
                e1 = (n % 3 == 0) ? t0 : (n % 3 == 1) ? t1 : t2;
            }
            else if (n % 4 == 3)
            {
                // The edge starts and ends at the same point.
                e1 = e0;
            }

            for (const bool edgeInSecond : {false, true})
            {
                bool above = false;
                bool expectedAbove = false;
                const bool expected = ReferenceCrosses(e0, e1, t0, t1, t2, edgeInSecond, expectedAbove);
                REQUIRE(CsgExact::Crosses(e0, e1, t0, t1, t2, edgeInSecond, above) == expected);
                REQUIRE(above == expectedAbove);
                REQUIRE(CsgExact::PassesInside(e0, e1, t0, t1, t2, edgeInSecond)
                    == ReferencePassesInside(e0, e1, t0, t1, t2, edgeInSecond));
                if (expected)
                    ++crossings;
            }
        }
    }
    REQUIRE(crossings > 1000);
}

TEST_CASE("Float expansion of a 128 bit number is exact", "[exact]")
{
    std::mt19937_64 random(31337);
    std::uniform_int_distribution<uint64_t> bits;
    for (int i = 0; i < 2000; ++i)
    {
        const int64_t hi = static_cast<int64_t>(bits(random)) >> (random() % 64);
        const CsgTest::Int128 value = (CsgTest::Int128(hi) * CsgTest::Int128(int64_t(1) << 62)) * CsgTest::Int128(4)
            + CsgTest::Int128(static_cast<int64_t>(bits(random) >> 1));
        const ea::vector<float> terms = CsgTest::FromInt128(value);
        REQUIRE(CsgTest::ToInt128(terms.data(), static_cast<int>(terms.size())) == value);
    }
    REQUIRE(CsgTest::FromInt128(CsgTest::Int128(int64_t(0))).empty());
}

namespace
{

using Int128 = CsgTest::Int128;

constexpr int RealDigits = std::numeric_limits<CsgReal>::digits;

Int128 MakeInt128(int64_t value)
{
    return Int128(value);
}

/// Convert a number with an integer value to the oracle type. The conversion is exact.
Int128 RealToInt128(CsgReal value)
{
    if (value == CsgReal(0))
        return MakeInt128(0);
    int exponent = 0;
    const CsgReal fraction = std::frexp(value, &exponent);
    const int64_t mantissa = static_cast<int64_t>(std::ldexp(fraction, RealDigits));
    const int shift = exponent - RealDigits;
    if (shift >= 0)
        return CsgTest::ShiftLeft(MakeInt128(mantissa), shift);
    REQUIRE(shift > -63);
    REQUIRE(mantissa % (int64_t(1) << -shift) == 0);
    return MakeInt128(mantissa / (int64_t(1) << -shift));
}

/// Exact value of an expansion with integer terms.
Int128 ExpansionValue(const CsgReal* terms, int count)
{
    Int128 result = MakeInt128(0);
    for (int i = 0; i < count; ++i)
        result = result + RealToInt128(terms[i]);
    return result;
}

ea::vector<CsgReal> ToExpansion(const Int128& value)
{
    const ea::vector<float> terms = CsgTest::FromInt128(value);
    ea::vector<CsgReal> result;
    for (const float term : terms)
        result.push_back(static_cast<CsgReal>(term));
    return result;
}

/// Position of the highest set bit of a nonzero number.
int HighBit(CsgReal value)
{
    int exponent = 0;
    std::frexp(value, &exponent);
    return exponent - 1;
}

/// Position of the lowest set bit of a nonzero number.
int LowBit(CsgReal value)
{
    int exponent = 0;
    const CsgReal fraction = std::frexp(value, &exponent);
    int64_t mantissa = static_cast<int64_t>(std::ldexp(std::fabs(fraction), RealDigits));
    int position = exponent - RealDigits;
    while ((mantissa & 1) == 0)
    {
        mantissa /= 2;
        ++position;
    }
    return position;
}

/// Check that the terms do not overlap, have no zero and have increasing magnitudes.
bool IsNonOverlapping(const CsgReal* terms, int count)
{
    for (int i = 0; i < count; ++i)
    {
        if (terms[i] == CsgReal(0))
            return false;
        if (i + 1 < count && !(std::fabs(terms[i]) < std::fabs(terms[i + 1])))
            return false;
        if (i + 1 < count && HighBit(terms[i]) >= LowBit(terms[i + 1]))
            return false;
    }
    return true;
}

/// Random 128 bit number with a random size up to maxBits bits and a random sign.
Int128 RandomInt128(std::mt19937_64& random, int maxBits)
{
    const int bits = static_cast<int>(random() % static_cast<uint64_t>(maxBits + 1));
    Int128 value = MakeInt128(0);
    for (int done = 0; done < bits; done += 32)
    {
        const int chunkBits = bits - done < 32 ? bits - done : 32;
        const int64_t chunk = static_cast<int64_t>(random() & ((uint64_t{1} << chunkBits) - 1));
        value = CsgTest::ShiftLeft(value, chunkBits) + MakeInt128(chunk);
    }
    return (random() & 1) ? MakeInt128(0) - value : value;
}

Int128 Power2(int bits)
{
    return CsgTest::ShiftLeft(MakeInt128(1), bits);
}

} // namespace

TEST_CASE("Two product is exact", "[exact]")
{
    using TwoProductFunction = CsgReal (*)(CsgReal, CsgReal, CsgReal&);
    ea::vector<TwoProductFunction> functions{&CsgTwoProduct, &CsgTwoProductDekker};
#if defined(CSG_HAS_HARDWARE_FMA)
    functions.push_back(&CsgTwoProductFma);
#endif

    std::mt19937_64 random(2024);
    for (const TwoProductFunction function : functions)
    {
        for (int i = 0; i < 10000; ++i)
        {
            // Integers up to 2^digits. The exact product has up to 2 * digits bits.
            const int64_t limit = int64_t(1) << RealDigits;
            const auto makeValue = [&]
            {
                const int bits = static_cast<int>(random() % (RealDigits + 1));
                const int64_t magnitude = static_cast<int64_t>(random() % (uint64_t{1} << bits));
                return (random() & 1) ? -magnitude : magnitude;
            };
            const int64_t a = i % 100 == 0 ? limit : makeValue();
            const int64_t b = i % 100 == 1 ? -limit : makeValue();
            const Int128 expected = MakeInt128(a) * MakeInt128(b);

            CsgReal error = 0;
            const CsgReal product = function(static_cast<CsgReal>(a), static_cast<CsgReal>(b), error);
            REQUIRE(product == static_cast<CsgReal>(a) * static_cast<CsgReal>(b));
            REQUIRE(RealToInt128(product) + RealToInt128(error) == expected);

            // Scale the second factor by 2^-80. The exact result is scaled by 2^-80 too.
            const CsgReal scaledB = static_cast<CsgReal>(b) * CsgReal(0x1p-80);
            CsgReal scaledError = 0;
            const CsgReal scaledProduct = function(static_cast<CsgReal>(a), scaledB, scaledError);
            REQUIRE(
                RealToInt128(std::ldexp(scaledProduct, 80)) + RealToInt128(std::ldexp(scaledError, 80)) == expected);
        }
    }
}

TEST_CASE("Fixed size expansion values", "[exact]")
{
    CsgExactMinor minor;
    minor.count_ = 0;
    REQUIRE(minor.Sign() == 0);
    REQUIRE(minor.Approximate() == 0.0);

    const ea::vector<CsgReal> terms = ToExpansion(MakeInt128(-1234567) * Power2(30) + MakeInt128(77));
    CsgExactVolume volume;
    volume.count_ = static_cast<int>(terms.size());
    for (int i = 0; i < volume.count_; ++i)
        volume.terms_[i] = terms[i];
    REQUIRE(volume.Sign() == -1);
    REQUIRE(volume.Approximate() == -1234567.0 * 1073741824.0 + 77.0);
}

TEST_CASE("Expansion scale, sum and compress are exact", "[exact]")
{
    std::mt19937_64 random(777);
    for (int i = 0; i < 5000; ++i)
    {
        const Int128 x = RandomInt128(random, 100);
        const Int128 y = RandomInt128(random, 100);
        const ea::vector<CsgReal> ex = ToExpansion(x);
        const ea::vector<CsgReal> ey = ToExpansion(y);
        const int cx = static_cast<int>(ex.size());
        const int cy = static_cast<int>(ey.size());
        REQUIRE(ExpansionValue(ex.data(), cx) == x);

        // Scale by an integer with up to 24 bits.
        const int64_t factor = static_cast<int64_t>(random() % (uint64_t{1} << 24)) * ((random() & 1) ? -1 : 1);
        ea::vector<CsgReal> scaled(2 * cx + 1);
        const int scaledCount = CsgScaleExpansion(cx, ex.data(), static_cast<CsgReal>(factor), scaled.data());
        REQUIRE(scaledCount >= 0);
        REQUIRE(scaledCount <= 2 * cx);
        REQUIRE(ExpansionValue(scaled.data(), scaledCount) == CsgTest::Mul(x, factor));
        REQUIRE(IsNonOverlapping(scaled.data(), scaledCount));
        if (cx == 0 || factor == 0)
            REQUIRE(scaledCount == 0);

        // Sum of two expansions. The expansions can overlap each other.
        ea::vector<CsgReal> sum(cx + cy + 1);
        const int sumCount = CsgExpansionSum(cx, ex.data(), cy, ey.data(), sum.data());
        REQUIRE(sumCount >= 0);
        REQUIRE(sumCount <= cx + cy);
        REQUIRE(ExpansionValue(sum.data(), sumCount) == x + y);
        REQUIRE(IsNonOverlapping(sum.data(), sumCount));

        // The sum of a number and its negation is empty.
        ea::vector<CsgReal> negated(ex);
        for (CsgReal& term : negated)
            term = -term;
        ea::vector<CsgReal> zero(2 * cx + 1);
        REQUIRE(CsgExpansionSum(cx, ex.data(), cx, negated.data(), zero.data()) == 0);

        // Compress a sum of three terms. The value stays the same.
        ea::vector<CsgReal> chain(2 * cx + cy + 1);
        const int chainCount = CsgExpansionSum(sumCount, sum.data(), scaledCount, scaled.data(), chain.data());
        REQUIRE(IsNonOverlapping(chain.data(), chainCount));
        const int compressedCount = CsgCompress(chainCount, chain.data());
        REQUIRE(compressedCount >= 0);
        REQUIRE(compressedCount <= chainCount);
        REQUIRE(ExpansionValue(chain.data(), compressedCount) == x + y + CsgTest::Mul(x, factor));
        REQUIRE(IsNonOverlapping(chain.data(), compressedCount));
    }

    SECTION("Empty inputs")
    {
        CsgReal out[4];
        const CsgReal one[1] = {1};
        REQUIRE(CsgScaleExpansion(0, one, 3, out) == 0);
        REQUIRE(CsgExpansionSum(0, one, 0, one, out) == 0);
        REQUIRE(CsgExpansionSum(1, one, 0, one, out) == 1);
        REQUIRE(out[0] == 1);
        REQUIRE(CsgExpansionSum(0, one, 1, one, out) == 1);
        REQUIRE(CsgCompress(0, out) == 0);
        REQUIRE(CsgExpansionSign(0, out) == 0);
    }
}

TEST_CASE("Expansion product difference sign matches the oracle", "[exact]")
{
    const Int128 limit = Power2(75);
    const Int128 zero = MakeInt128(0);
    const Int128 one = MakeInt128(1);

    const auto check = [](const Int128& a, const Int128& b, const Int128& c, const Int128& d)
    {
        const ea::vector<CsgReal> ta = ToExpansion(a);
        const ea::vector<CsgReal> tb = ToExpansion(b);
        const ea::vector<CsgReal> tc = ToExpansion(c);
        const ea::vector<CsgReal> td = ToExpansion(d);
        const int expected = CsgTest::SignOfProductDifference(a, b, c, d);
        const int actual = CsgExpansionProductDifferenceSign(static_cast<int>(ta.size()), ta.data(),
            static_cast<int>(tb.size()), tb.data(), static_cast<int>(tc.size()), tc.data(),
            static_cast<int>(td.size()), td.data());
        REQUIRE(actual == expected);
    };

    std::mt19937_64 random(4242);

    SECTION("Random values")
    {
        for (int i = 0; i < 20000; ++i)
            check(RandomInt128(random, 75), RandomInt128(random, 75), RandomInt128(random, 75),
                RandomInt128(random, 75));
    }

    SECTION("Equal products")
    {
        for (int i = 0; i < 5000; ++i)
        {
            const Int128 a = RandomInt128(random, 25);
            const Int128 b = RandomInt128(random, 25);
            const Int128 k = RandomInt128(random, 25);
            check(a, b, b, a);
            check(a, b, zero - a, zero - b);
            check(a * k, b, a, b * k);
            check(a * k, zero - b, zero - a, b * k);
        }
    }

    SECTION("Products that differ by one")
    {
        for (int i = 0; i < 5000; ++i)
        {
            // n * n - (n - 1) * (n + 1) = 1
            Int128 n = RandomInt128(random, 74);
            if (n < zero)
                n = zero - n;
            check(n, n, n - one, n + one);
            check(n - one, n + one, n, n);
            check(zero - n, n, n - one, zero - n - one);
            check(n, n - one, n - one, n);
        }
        const Int128 n = limit - MakeInt128(2);
        check(n, n, n - one, n + one);
        check(n - one, n + one, n, n);
    }

    SECTION("Zeros")
    {
        for (int i = 0; i < 2000; ++i)
        {
            const Int128 a = RandomInt128(random, 75);
            const Int128 b = RandomInt128(random, 75);
            check(zero, a, b, zero);
            check(zero, a, zero, b);
            check(a, b, zero, a);
            check(a, zero, a, b);
            check(zero, zero, zero, zero);
        }
    }

    SECTION("Values near 2^75")
    {
        const Int128 values[] = {limit - one, zero - (limit - one), limit - MakeInt128(2),
            zero - (limit - MakeInt128(2)), Power2(74), zero - Power2(74), Power2(74) + one, Power2(50), one,
            zero - one};
        for (const Int128& a : values)
            for (const Int128& b : values)
                for (const Int128& c : values)
                    for (const Int128& d : values)
                        check(a, b, c, d);
    }
}

TEST_CASE("Exact filter", "[exact]")
{
    // Use big coordinates so that the products in float round.
    const int64_t big = int64_t(1) << 20;

    const auto runFilter = [](const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d,
                               float& det)
    {
        const CsgFloat4 ux = CsgFloat4::Splat(static_cast<float>(b.x_ - a.x_));
        const CsgFloat4 uy = CsgFloat4::Splat(static_cast<float>(b.y_ - a.y_));
        const CsgFloat4 uz = CsgFloat4::Splat(static_cast<float>(b.z_ - a.z_));
        const CsgFloat4 vx = CsgFloat4::Splat(static_cast<float>(c.x_ - a.x_));
        const CsgFloat4 vy = CsgFloat4::Splat(static_cast<float>(c.y_ - a.y_));
        const CsgFloat4 vz = CsgFloat4::Splat(static_cast<float>(c.z_ - a.z_));
        const CsgFloat4 wx = CsgFloat4::Splat(static_cast<float>(d.x_ - a.x_));
        const CsgFloat4 wy = CsgFloat4::Splat(static_cast<float>(d.y_ - a.y_));
        const CsgFloat4 wz = CsgFloat4::Splat(static_cast<float>(d.z_ - a.z_));
        CsgFloat4 result;
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
        float det = 0.0f;
        REQUIRE((runFilter(a, b, c, d, det) & 1) == 0);
        REQUIRE(CsgExact::Orient(a, b, c, d, 0) == 0);
        // The volume has no filter. It is exactly zero.
        REQUIRE(CsgSign(CsgExact::VolumeT<CsgInt128Portable>(a, b, c, d)) == 0);
        // One grid unit above the plane: the exact sign is nonzero.
        const CsgGridPoint above = P(d.x_, d.y_, d.z_ + 1);
        const int exactSign = CsgSign(CsgExact::VolumeT<CsgInt128Portable>(a, b, c, above));
        REQUIRE(exactSign != 0);
        REQUIRE(CsgExact::Orient(a, b, c, above, 0) == exactSign);
    }

    SECTION("Clear cases are decided")
    {
        float det = 0.0f;
        REQUIRE((runFilter(P(0, 0, 0), P(Unit, 0, 0), P(0, Unit, 0), P(0, 0, Unit), det) & 1) != 0);
        REQUIRE(det > 0.0f);
        REQUIRE((runFilter(P(0, 0, 0), P(0, Unit, 0), P(Unit, 0, 0), P(0, 0, Unit), det) & 1) != 0);
        REQUIRE(det < 0.0f);
    }

    SECTION("Decided lanes have the sign of the reference")
    {
        std::mt19937_64 random(2024);
        const int64_t limit = (int64_t(1) << 23) - 1;
        std::uniform_int_distribution<int64_t> coordinate(-limit, limit);
        std::uniform_int_distribution<int> factor(-1, 1);
        std::uniform_int_distribution<int> axisPick(0, 2);
        std::uniform_int_distribution<int> signPick(0, 1);
        const auto clamp = [limit](int64_t value) { return std::max(-limit, std::min(limit, value)); };
        int decidedCount = 0;
        int undecidedCount = 0;
        for (int i = 0; i < 4000; ++i)
        {
            const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
            CsgGridPoint d = P(coordinate(random), coordinate(random), coordinate(random));
            if (i % 4 != 0)
            {
                // Coplanar point: d = a + s (b - a) + t (c - a), clamped into the range.
                const int s = factor(random);
                const int t = factor(random);
                const auto mix = [&](int64_t pa, int64_t pb, int64_t pc)
                { return clamp(pa + s * (pb - pa) + t * (pc - pa)); };
                d = P(mix(a.x_, b.x_, c.x_), mix(a.y_, b.y_, c.y_), mix(a.z_, b.z_, c.z_));
                if (i % 4 == 1)
                {
                    // Near coplanar point: move by one unit on one axis.
                    const int64_t step = signPick(random) == 0 ? -1 : 1;
                    const int axis = axisPick(random);
                    d = P(clamp(d.x_ + (axis == 0 ? step : 0)), clamp(d.y_ + (axis == 1 ? step : 0)),
                        clamp(d.z_ + (axis == 2 ? step : 0)));
                }
            }
            float det = 0.0f;
            const int decided = runFilter(a, b, c, d, det);
            const int reference = CsgTest::ReferenceOrient(a, b, c, d, 0);
            if ((decided & 1) != 0)
            {
                ++decidedCount;
                REQUIRE((det > 0.0f ? 1 : -1) == reference);
            }
            else
            {
                ++undecidedCount;
            }
            REQUIRE(CsgExact::Orient(a, b, c, d, 0) == reference);
        }
        // The filter decides some lanes and leaves some lanes to the exact path.
        REQUIRE(decidedCount > 100);
        REQUIRE(undecidedCount > 100);
    }

    SECTION("Lanes are independent")
    {
        const CsgFloat4 one = CsgFloat4::Splat(1.0f);
        const CsgFloat4 zero = CsgFloat4::Splat(0.0f);
        // Lane 0 has det = 1, lane 1 has det = -1, lane 2 has det = 0, lane 3 is ignored.
        const CsgFloat4 ux = CsgFloat4::Create(1.0f, -1.0f, 0.0f, 5.0f);
        CsgFloat4 det;
        const int decided = CsgInternal::Filter(ux, zero, zero, zero, one, zero, zero, zero, one, det);
        REQUIRE(decided == 0b011);
        REQUIRE(det.Get(0) == 1.0f);
        REQUIRE(det.Get(1) == -1.0f);
        REQUIRE(det.Get(2) == 0.0f);
    }
}

TEST_CASE("Simd float vector", "[exact]")
{
    const CsgFloat4 a = CsgFloat4::Create(1.0f, -2.0f, 3.0f, -4.0f);
    const CsgFloat4 b = CsgFloat4::Create(0.5f, 2.0f, -3.0f, std::numeric_limits<float>::quiet_NaN());
    float out[4];
    (a + b).Store(out);
    REQUIRE(out[0] == 1.5f);
    REQUIRE(out[1] == 0.0f);
    REQUIRE(out[2] == 0.0f);
    REQUIRE(std::isnan(out[3]));
    (a - b).Store(out);
    REQUIRE(out[0] == 0.5f);
    REQUIRE(out[1] == -4.0f);
    REQUIRE(out[2] == 6.0f);
    (a * b).Store(out);
    REQUIRE(out[0] == 0.5f);
    REQUIRE(out[1] == -4.0f);
    REQUIRE(out[2] == -9.0f);
    CsgFloat4::Abs(a).Store(out);
    REQUIRE(out[0] == 1.0f);
    REQUIRE(out[1] == 2.0f);
    REQUIRE(out[2] == 3.0f);
    REQUIRE(out[3] == 4.0f);
    // Lane 3 compares with NaN and gives a clear bit.
    REQUIRE(CsgFloat4::GreaterThanMask(a, b) == 0b0101);
    REQUIRE(CsgFloat4::GreaterThanMask(b, a) == 0b0010);
    REQUIRE(a.Get(2) == 3.0f);
}

TEST_CASE("Grid point snapping", "[exact]")
{
    const auto from = [](float x, float y, float z) { return CsgGridPoint::From(Vector3(x, y, z)); };

    REQUIRE(from(0.5f, -0.5f, 0.0f) == P(Unit / 2, -Unit / 2, 0));
    REQUIRE(from(1.0f, -1.0f, 2.0f) == P(Unit, -Unit, 2 * Unit));
    // 1e-4 * 2^10 = 0.1024 rounds to zero.
    REQUIRE(from(1e-4f, -1e-4f, 0.0f) == P(0, 0, 0));
    // 8191.99f is the float 8191.990234375 = 8388598 / 1024, so the grid value is exact.
    REQUIRE(from(8191.99f, -8191.99f, 0.0f) == P(8388598, -8388598, 0));

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
    }

    SECTION("An edge through a vertex is claimed by the displacement")
    {
        // The edge passes the vertex t0. The displaced operand decides which side. The result must match the oracle.
        const CsgGridPoint e0 = P(0, 0, -Unit);
        const CsgGridPoint e1 = P(0, 0, Unit);
        for (const bool edgeInSecond : {false, true})
        {
            bool aboveA = false;
            bool aboveB = false;
            REQUIRE(CsgExact::Crosses(e0, e1, t0, t1, t2, edgeInSecond, aboveA)
                == ReferenceCrosses(e0, e1, t0, t1, t2, edgeInSecond, aboveB));
            REQUIRE(aboveA == aboveB);
        }
    }
}

} // namespace Urho3D
