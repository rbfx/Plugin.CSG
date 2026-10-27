// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgExact.h"
#include "Urho3D/CSG/CsgExactFilter.h"
#include "Urho3D/CSG/CsgExpansion.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestExactReference.h"
#include "TestGeometry.h"

#include <catch2/catch_amalgamated.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

namespace Urho3D
{

namespace
{

// The integer types under test. The portable struct always runs. The compiler type runs too, when it exists.
#ifdef CSG_TEST_HAS_NATIVE_INT128
using CsgTestTypes = std::tuple<CsgTest::Int128Portable, CsgTest::Int128Native>;
#else
using CsgTestTypes = std::tuple<CsgTest::Int128Portable>;
#endif

CsgTest::Int128Portable MakeInt(const CsgTest::Int128Portable*, int64_t hi, uint64_t lo)
{
    return CsgTest::Int128Portable::FromParts(lo, hi);
}

#ifdef CSG_TEST_HAS_NATIVE_INT128
CsgTest::Int128Native MakeInt(const CsgTest::Int128Native*, int64_t hi, uint64_t lo)
{
    return static_cast<CsgTest::Int128Native>(
        (static_cast<CsgTest::UInt128Native>(static_cast<uint64_t>(hi)) << 64) | lo);
}

CsgTest::Int128Native ToNative(const CsgTest::Int128Portable& value)
{
    return MakeInt(static_cast<const CsgTest::Int128Native*>(nullptr), value.hi_, value.lo_);
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

constexpr int64_t Unit = static_cast<int64_t>(CsgGridScale);

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
    REQUIRE(CsgTest::Int128Portable::Multiply(3, -4) == CsgTest::Int128Portable(-12));
    REQUIRE(CsgTest::Int128Portable::Multiply(-3, -4) == CsgTest::Int128Portable(12));
    const auto minimum = std::numeric_limits<int64_t>::min();
    // (-2^63) * (-2^63) = 2^126
    REQUIRE(CsgTest::Int128Portable::Multiply(minimum, minimum) == CsgTest::Int128Portable::FromParts(0, int64_t(1) << 62));
    REQUIRE(CsgTest::Int128Portable::Multiply(minimum, 1) == CsgTest::Int128Portable(minimum));

#ifdef CSG_TEST_HAS_NATIVE_INT128
    std::mt19937_64 random(12345);
    const int64_t bound = int64_t(1) << 62;
    std::uniform_int_distribution<int64_t> distribution(-bound + 1, bound - 1);
    for (int i = 0; i < 10000; ++i)
    {
        const int64_t a = distribution(random);
        const int64_t b = distribution(random);
        const CsgTest::Int128Native expected = static_cast<CsgTest::Int128Native>(a) * b;
        const CsgTest::Int128Portable product = CsgTest::Int128Portable::Multiply(a, b);
        REQUIRE(ToNative(product) == expected);
        REQUIRE(ToNative(CsgTest::Int128Portable(a) * CsgTest::Int128Portable(b)) == expected);
        REQUIRE(ToNative(CsgTest::Mul64<CsgTest::Int128Portable>(a, b)) == expected);
    }

    // Wrapping arithmetic on full 128 bit values.
    std::uniform_int_distribution<uint64_t> bits;
    for (int i = 0; i < 10000; ++i)
    {
        const CsgTest::Int128Portable x = CsgTest::Int128Portable::FromParts(bits(random), static_cast<int64_t>(bits(random)));
        const CsgTest::Int128Portable y = CsgTest::Int128Portable::FromParts(bits(random), static_cast<int64_t>(bits(random)));
        const int64_t small = static_cast<int64_t>(bits(random));
        const CsgTest::UInt128Native ux = static_cast<CsgTest::UInt128Native>(ToNative(x));
        const CsgTest::UInt128Native uy = static_cast<CsgTest::UInt128Native>(ToNative(y));
        const CsgTest::UInt128Native us = static_cast<CsgTest::UInt128Native>(static_cast<CsgTest::Int128Native>(small));
        REQUIRE(static_cast<CsgTest::UInt128Native>(ToNative(x + y)) == ux + uy);
        REQUIRE(static_cast<CsgTest::UInt128Native>(ToNative(x - y)) == ux - uy);
        REQUIRE(static_cast<CsgTest::UInt128Native>(ToNative(-x)) == CsgTest::UInt128Native(0) - ux);
        REQUIRE(static_cast<CsgTest::UInt128Native>(ToNative(x * y)) == ux * uy);
        REQUIRE(static_cast<CsgTest::UInt128Native>(ToNative(CsgTest::Mul(x, small))) == ux * us);
        REQUIRE((x < y) == (ToNative(x) < ToNative(y)));
        REQUIRE((x >= y) == (ToNative(x) >= ToNative(y)));
        REQUIRE(x.Sign() == CsgTest::Sign(ToNative(x)));
    }
#endif
}

TEMPLATE_LIST_TEST_CASE("Sign of product difference", "[exact]", CsgTestTypes)
{
    using Int = TestType;

    SECTION("Random values that fit in the compiler type")
    {
#ifdef CSG_TEST_HAS_NATIVE_INT128
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
            const CsgTest::Int128Native difference = static_cast<CsgTest::Int128Native>(a) * b - static_cast<CsgTest::Int128Native>(c) * d;
            const int expected = difference > 0 ? 1 : (difference < 0 ? -1 : 0);
            REQUIRE(CsgTest::SignOfProductDifference<Int>(Int(a), Int(b), Int(c), Int(d)) == expected);
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
        REQUIRE(CsgTest::SignOfProductDifference<Int>(pow126, two, pow126, two) == 0);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(pow126, two, pow126, one) == 1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(pow126, one, pow126, two) == -1);
        // (-2^127) * 1 - (2^127 - 1) * (-1) = -1
        REQUIRE(CsgTest::SignOfProductDifference<Int>(minimum, one, maximum, minusOne) == -1);
        // (-2^127)^2 = 2^254 is larger than (2^127 - 1)^2
        REQUIRE(CsgTest::SignOfProductDifference<Int>(minimum, minimum, maximum, maximum) == 1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(maximum, maximum, minimum, minimum) == -1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(minimum, minimum, minimum, minimum) == 0);
        // Products that differ only in the lowest bit of a 256 bit value.
        REQUIRE(CsgTest::SignOfProductDifference<Int>(maximum, maximum, maximum, Make<Int>(std::numeric_limits<int64_t>::max(), ~uint64_t(0) - 1)) == 1);
        // Opposite signs: -(2^127) - (2^127 - 1) < 0
        REQUIRE(CsgTest::SignOfProductDifference<Int>(minimum, one, maximum, one) == -1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(maximum, one, minimum, one) == 1);
        // Equal magnitudes with negative products: (-a) * b - (-c) * d where a * b == c * d
        REQUIRE(CsgTest::SignOfProductDifference<Int>(Int(-6), Int(7), Int(-14), Int(3)) == 0);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(Int(-6), Int(7), Int(-14), Int(2)) == -1);
    }

    SECTION("Zero cases")
    {
        const Int zero = Int(0);
        const Int five = Int(5);
        const Int minusFive = Int(-5);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(zero, five, zero, minusFive) == 0);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(zero, five, five, five) == -1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(zero, five, five, minusFive) == 1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(five, five, zero, zero) == 1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(minusFive, five, zero, five) == -1);
        REQUIRE(CsgTest::SignOfProductDifference<Int>(five, minusFive, five, minusFive) == 0);
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

TEST_CASE("Volume and normal on tetrahedra", "[exact]")
{
    const CsgGridPoint a = P(0, 0, 0);
    const CsgGridPoint b = P(Unit, 0, 0);
    const CsgGridPoint c = P(0, Unit, 0);
    const CsgGridPoint d = P(0, 0, Unit);

    const CsgNormal normal = CsgExact::Normal(a, b, c);
    REQUIRE(normal.x_.Sign() == 0);
    REQUIRE(normal.y_.Sign() == 0);
    REQUIRE(normal.z_.Approximate() == static_cast<double>(Unit) * Unit);
    REQUIRE(CsgExact::Volume(a, b, c, d).Approximate() == static_cast<double>(Unit) * Unit * Unit);
    REQUIRE(CsgExact::Volume(a, c, b, d).Approximate() == -static_cast<double>(Unit) * Unit * Unit);
    REQUIRE(CsgExact::Volume(a, b, c, P(5, 5, 0)).Sign() == 0);
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

TEST_CASE("Reference oracle matches the library Orient", "[exact]")
{
    std::mt19937_64 random(8086);
    // Small values give many degenerate cases. Large values stay inside the range of the grid.
    for (const int64_t range : {int64_t(3), int64_t(1) << 10, CsgGridIntegerLimit - 1})
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
            REQUIRE(CsgExact::Volume(a, b, c, d).Sign() == CsgTest::Sign(CsgTest::ReferenceVolume(a, b, c, d)));
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
    const int64_t limit = CsgGridIntegerLimit - 1;
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

TEST_CASE("Orient stress test against the int128 oracle", "[exact][stress]")
{
    std::mt19937_64 random(987654321);
    const int64_t range = CsgGridIntegerLimit - 1;
    std::uniform_int_distribution<int64_t> coordinate(-range, range);
    std::uniform_int_distribution<int> mask(0, 15);
    std::uniform_int_distribution<int> factor(-2, 3);
    std::uniform_int_distribution<int> kind(0, 3);
    std::uniform_int_distribution<int> axisPick(0, 2);
    std::uniform_int_distribution<int> offsetPick(-1, 1);
    const auto inRange = [&](const CsgGridPoint& p)
    { return std::abs(p.x_) <= range && std::abs(p.y_) <= range && std::abs(p.z_) <= range; };
    const auto randomPoint = [&]() { return P(coordinate(random), coordinate(random), coordinate(random)); };

    constexpr int caseCount = 60000;
    int zeroCount = 0;
    int nearZeroCount = 0;
    for (int n = 0; n < caseCount; ++n)
    {
        const CsgGridPoint a = randomPoint();
        const CsgGridPoint b = randomPoint();
        const CsgGridPoint c = randomPoint();
        CsgGridPoint d = randomPoint();
        const int selected = kind(random);
        if (selected != 0)
        {
            // Try to place d on the plane of a, b and c.
            for (int attempt = 0; attempt < 16; ++attempt)
            {
                const int i = factor(random);
                const int j = factor(random);
                const CsgGridPoint candidate = P(a.x_ + i * (b.x_ - a.x_) + j * (c.x_ - a.x_),
                    a.y_ + i * (b.y_ - a.y_) + j * (c.y_ - a.y_), a.z_ + i * (b.z_ - a.z_) + j * (c.z_ - a.z_));
                if (inRange(candidate))
                {
                    d = candidate;
                    break;
                }
            }
            if (selected == 2)
            {
                // Move d by one grid unit on one axis. The point is nearly coplanar.
                const int offset = offsetPick(random);
                const int axis = axisPick(random);
                if (axis == 0 && std::abs(d.x_ + offset) <= range)
                    d.x_ += offset;
                else if (axis == 1 && std::abs(d.y_ + offset) <= range)
                    d.y_ += offset;
                else if (axis == 2 && std::abs(d.z_ + offset) <= range)
                    d.z_ += offset;
            }
            else if (selected == 3)
            {
                // Collinear points a, b and d with a coplanar fourth point.
                const int i = factor(random);
                const CsgGridPoint candidate = P(
                    a.x_ + i * (b.x_ - a.x_), a.y_ + i * (b.y_ - a.y_), a.z_ + i * (b.z_ - a.z_));
                if (inRange(candidate))
                    d = candidate;
            }
        }

        const int second = mask(random);
        const int expected = CsgTest::ReferenceOrient(a, b, c, d, second);
        REQUIRE(CsgExact::Orient(a, b, c, d, second) == expected);
        const int volumeSign = CsgTest::ReferenceOrient(a, b, c, d, 0);
        REQUIRE(CsgExact::Orient(a, b, c, d, 0) == volumeSign);
        if (volumeSign == 0)
            ++zeroCount;
        else if (selected == 2)
            ++nearZeroCount;
    }
    // The test must reach the degenerate and the nearly degenerate cases.
    REQUIRE(zeroCount > caseCount / 10);
    REQUIRE(nearZeroCount > caseCount / 20);
}

TEST_CASE("Crosses and PassesInside match the int128 oracle", "[exact]")
{
    std::mt19937_64 random(777);
    const int64_t limit = CsgGridIntegerLimit - 1;
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

TEST_CASE("Approximate of an expansion with large values", "[exact]")
{
    // Build the compressed expansion of a number.
    const auto build = [](const CsgTest::Int128& value, CsgExactVolume& out)
    {
        const ea::vector<float> terms = CsgTest::FromInt128(value);
        REQUIRE(terms.size() <= 24);
        for (size_t i = 0; i < terms.size(); ++i)
            out.terms_[i] = terms[i];
        out.count_ = CsgCompress(static_cast<int>(terms.size()), out.terms_);
        REQUIRE(out.count_ >= 0);
    };
    // The result is correct when it differs from the expected value by at most one unit in the last place.
    const auto withinOneUlp = [](double result, double expected)
    {
        const double ulp = std::nextafter(std::fabs(expected), std::numeric_limits<double>::infinity()) - std::fabs(expected);
        return std::fabs(result - expected) <= ulp;
    };

    const CsgTest::Int128 pow70 = CsgTest::ShiftLeft(CsgTest::Int128(int64_t(1)), 70);
    const CsgTest::Int128 pow74 = CsgTest::ShiftLeft(CsgTest::Int128(int64_t(1)), 74);

    SECTION("Plus and minus 2^70 + 1")
    {
        CsgExactVolume positive;
        build(pow70 + CsgTest::Int128(int64_t(1)), positive);
        CsgExactVolume negative;
        build(CsgTest::Int128(int64_t(0)) - (pow70 + CsgTest::Int128(int64_t(1))), negative);
        const double expected = std::ldexp(1.0, 70);
        REQUIRE(withinOneUlp(positive.Approximate(), expected));
        REQUIRE(withinOneUlp(negative.Approximate(), -expected));
        REQUIRE(withinOneUlp(CsgExpansionApproximate(positive.count_, positive.terms_), expected));
        REQUIRE(withinOneUlp(CsgExpansionApproximate(negative.count_, negative.terms_), -expected));
        REQUIRE(positive.Sign() == 1);
        REQUIRE(negative.Sign() == -1);
    }

    SECTION("2^74 + 12345")
    {
        CsgExactVolume value;
        build(pow74 + CsgTest::Int128(int64_t(12345)), value);
        const double expected = std::ldexp(1.0, 74) + 12345.0;
        REQUIRE(withinOneUlp(value.Approximate(), expected));
        REQUIRE(withinOneUlp(CsgExpansionApproximate(value.count_, value.terms_), expected));
    }

    SECTION("Low terms change the rounding")
    {
        // The double ulp of 2^70 is 2^18. The value 2^70 + 2^17 is a tie and rounds to even.
        // The extra low term 1 breaks the tie and the result rounds up.
        const double tieDown = std::ldexp(1.0, 70);
        const double roundedUp = std::ldexp(1.0, 70) + std::ldexp(1.0, 18);
        const auto check = [&](const CsgTest::Int128& value, double expected)
        {
            CsgExactVolume volume;
            build(value, volume);
            REQUIRE(volume.Approximate() == expected);
            REQUIRE(CsgExpansionApproximate(volume.count_, volume.terms_) == expected);
            CsgExactVolume negated;
            build(CsgTest::Int128(int64_t(0)) - value, negated);
            REQUIRE(negated.Approximate() == -expected);
            REQUIRE(CsgExpansionApproximate(negated.count_, negated.terms_) == -expected);
        };
        const CsgTest::Int128 half = CsgTest::ShiftLeft(CsgTest::Int128(int64_t(1)), 17);
        check(pow70 + CsgTest::ShiftLeft(CsgTest::Int128(int64_t(1)), 20), tieDown + std::ldexp(1.0, 20));
        check(pow70 + half, tieDown);
        check(pow70 + half + CsgTest::Int128(int64_t(1)), roundedUp);
        check(pow74 + CsgTest::ShiftLeft(CsgTest::Int128(int64_t(3)), 21) + CsgTest::Int128(int64_t(1)),
            std::ldexp(1.0, 74) + 3.0 * std::ldexp(1.0, 21) + 1.0);
        // The double ulp of 2^74 is 2^22. The value 2^74 + 2^21 + 1 rounds up.
        check(pow74 + CsgTest::ShiftLeft(CsgTest::Int128(int64_t(1)), 21) + CsgTest::Int128(int64_t(1)),
            std::ldexp(1.0, 74) + std::ldexp(1.0, 22));
    }

    SECTION("Zero")
    {
        CsgExactVolume zero;
        build(CsgTest::Int128(int64_t(0)), zero);
        REQUIRE(zero.count_ == 0);
        REQUIRE(zero.Approximate() == 0.0);
        REQUIRE(CsgExpansionApproximate(0, zero.terms_) == 0.0);
    }
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

#if defined(CSG_HAS_HARDWARE_FMA)
TEST_CASE("Dekker two product equals the FMA two product", "[exact]")
{
    std::mt19937_64 random(424242);
    std::uniform_int_distribution<int> exponentPick(-20, 20);
    std::uniform_real_distribution<double> mantissa(0.5, 1.0);
    const auto makeValue = [&]
    {
        const CsgReal value = static_cast<CsgReal>(std::ldexp(mantissa(random), exponentPick(random)));
        return (random() & 1) ? -value : value;
    };
    for (int i = 0; i < 100000; ++i)
    {
        const CsgReal a = makeValue();
        const CsgReal b = makeValue();
        CsgReal dekkerError = 0;
        CsgReal fmaError = 0;
        const CsgReal dekkerProduct = CsgTwoProductDekker(a, b, dekkerError);
        const CsgReal fmaProduct = CsgTwoProductFma(a, b, fmaError);
        REQUIRE(dekkerProduct == fmaProduct);
        REQUIRE(dekkerError == fmaError);
    }
}
#endif

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

TEST_CASE("Volume and normal match the int128 oracle", "[exact]")
{
    std::mt19937_64 random(8087);
    // Small values give many degenerate cases. Large values stay inside the range of the grid.
    for (const int64_t range : {int64_t(3), int64_t(1) << 10, CsgGridIntegerLimit - 1})
    {
        std::uniform_int_distribution<int64_t> coordinate(-range, range);
        for (int i = 0; i < 3000; ++i)
        {
            const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgGridPoint d = P(coordinate(random), coordinate(random), coordinate(random));
            const CsgExactVolume volume = CsgExact::Volume(a, b, c, d);
            REQUIRE(ExpansionValue(volume.terms_, volume.count_) == CsgTest::ReferenceVolume(a, b, c, d));
            const CsgNormal normal = CsgExact::Normal(a, b, c);
            const CsgTest::ReferenceNormal expected = CsgTest::ReferenceNormalOf(a, b, c);
            REQUIRE(ExpansionValue(normal.x_.terms_, normal.x_.count_) == expected.x_);
            REQUIRE(ExpansionValue(normal.y_.terms_, normal.y_.count_) == expected.y_);
            REQUIRE(ExpansionValue(normal.z_.terms_, normal.z_.count_) == expected.z_);
        }
    }
}

TEST_CASE("Expansion product difference sign matches the oracle", "[exact]")
{
    // The inputs use the full range of the function: 2^75 in float mode and 2^123 in double mode.
    constexpr int InputBits = CsgProductDifferenceInputBits;
    const Int128 limit = Power2(InputBits);
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
            check(RandomInt128(random, InputBits), RandomInt128(random, InputBits), RandomInt128(random, InputBits),
                RandomInt128(random, InputBits));
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
            Int128 n = RandomInt128(random, InputBits - 1);
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
            const Int128 a = RandomInt128(random, InputBits);
            const Int128 b = RandomInt128(random, InputBits);
            check(zero, a, b, zero);
            check(zero, a, zero, b);
            check(a, b, zero, a);
            check(a, zero, a, b);
            check(zero, zero, zero, zero);
        }
    }

    SECTION("Values near the input limit")
    {
        const Int128 values[] = {limit - one, zero - (limit - one), limit - MakeInt128(2),
            zero - (limit - MakeInt128(2)), Power2(InputBits - 1), zero - Power2(InputBits - 1),
            Power2(InputBits - 1) + one, Power2(50), one,
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
        REQUIRE(CsgExact::Volume(a, b, c, d).Sign() == 0);
        // One grid unit above the plane: the exact sign is nonzero.
        const CsgGridPoint above = P(d.x_, d.y_, d.z_ + 1);
        const int exactSign = CsgExact::Volume(a, b, c, above).Sign();
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
        const int64_t limit = CsgGridIntegerLimit - 1;
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

TEST_CASE("Filter and Orient decide the sign of near coplanar points with large differences", "[exact][filter]")
{
    std::mt19937_64 random(777001);
    const int64_t limit = CsgGridIntegerLimit - 1;
    std::uniform_int_distribution<int64_t> coordinate(-limit, limit);
    std::uniform_int_distribution<int> factor(-1, 2);
    std::uniform_int_distribution<int> axisPick(0, 2);
    std::uniform_int_distribution<int> signPick(0, 1);
    std::uniform_int_distribution<int> bitsPick(0, CsgGridBits - 1);
    std::uniform_int_distribution<int> maskPick(0, 15);
    const int64_t differenceLimit = int64_t{1} << (CsgGridBits + 1);

    int64_t largestDifference = 0;
    int decidedCount = 0;
    int undecidedCount = 0;
    int zeroCount = 0;
    int tested = 0;
    // The position is rejected when it is outside the range of the grid.
    const auto inRange = [limit](int64_t x, int64_t y, int64_t z)
    { return std::abs(x) <= limit && std::abs(y) <= limit && std::abs(z) <= limit; };
    while (tested < 100000)
    {
        const CsgGridPoint a = P(coordinate(random), coordinate(random), coordinate(random));
        const CsgGridPoint b = P(coordinate(random), coordinate(random), coordinate(random));
        const CsgGridPoint c = P(coordinate(random), coordinate(random), coordinate(random));
        // Coplanar point d = a + s (b - a) + t (c - a).
        const int s = factor(random);
        const int t = factor(random);
        int64_t dx = a.x_ + s * (int64_t{b.x_} - a.x_) + t * (int64_t{c.x_} - a.x_);
        int64_t dy = a.y_ + s * (int64_t{b.y_} - a.y_) + t * (int64_t{c.y_} - a.y_);
        int64_t dz = a.z_ + s * (int64_t{b.z_} - a.z_) + t * (int64_t{c.z_} - a.z_);
        if (!inRange(dx, dy, dz))
            continue;
        // Move d off the plane by a random amount: one unit on one axis or up to 2^bits on all axes.
        if (tested % 3 != 0)
        {
            const int64_t amount = (int64_t{1} << bitsPick(random)) * (signPick(random) == 0 ? -1 : 1);
            if (tested % 3 == 1)
            {
                const int axis = axisPick(random);
                (axis == 0 ? dx : (axis == 1 ? dy : dz)) += signPick(random) == 0 ? -1 : 1;
            }
            else
            {
                dx += amount;
                dy -= amount / 2;
                dz += amount / 4;
            }
            if (!inRange(dx, dy, dz))
                continue;
        }
        const CsgGridPoint d = P(dx, dy, dz);
        ++tested;

        const int64_t differences[9] = {int64_t{b.x_} - a.x_, int64_t{b.y_} - a.y_, int64_t{b.z_} - a.z_,
            int64_t{c.x_} - a.x_, int64_t{c.y_} - a.y_, int64_t{c.z_} - a.z_, int64_t{d.x_} - a.x_,
            int64_t{d.y_} - a.y_, int64_t{d.z_} - a.z_};
        for (const int64_t difference : differences)
        {
            REQUIRE(std::abs(difference) < differenceLimit);
            largestDifference = std::max(largestDifference, std::abs(difference));
        }

        const int oracle = CsgTest::Sign(CsgTest::ReferenceVolume(a, b, c, d));
        zeroCount += oracle == 0 ? 1 : 0;

        // The filter path: a decided lane has the sign of the oracle.
        const auto lane = [&](int i) { return CsgFloat4::Splat(static_cast<float>(differences[i])); };
        CsgFloat4 det;
        const int decided = CsgInternal::Filter(lane(0), lane(1), lane(2), lane(3), lane(4), lane(5), lane(6), lane(7),
            lane(8), det);
        if ((decided & 1) != 0)
        {
            ++decidedCount;
            REQUIRE(oracle != 0);
            REQUIRE((det.Get(0) > 0.0f ? 1 : -1) == oracle);
        }
        else
        {
            ++undecidedCount;
        }

        // The public path with and without the symbolic displacement.
        REQUIRE(CsgExact::Orient(a, b, c, d, 0) == CsgTest::ReferenceOrient(a, b, c, d, 0));
        if (oracle != 0)
            REQUIRE(CsgExact::Orient(a, b, c, d, 0) == oracle);
        const int mask = maskPick(random);
        REQUIRE(CsgExact::Orient(a, b, c, d, mask) == CsgTest::ReferenceOrient(a, b, c, d, mask));
    }
    // The test reaches differences above the exact range of a float, and all three outcomes happen.
    REQUIRE(largestDifference > (int64_t{1} << (CsgGridBits - 1)));
    REQUIRE(decidedCount > 1000);
    REQUIRE(undecidedCount > 1000);
    REQUIRE(zeroCount > 1000);
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
    // 0.1 grid steps round to zero. In float mode this is 1e-4 * 2^10 = 0.1024 steps.
    const float tenthStep = static_cast<float>(0.1 / CsgGridScale);
    REQUIRE(from(tenthStep, -tenthStep, 0.0f) == P(0, 0, 0));
    // 8191.99f is the float 8191.990234375 = 8388598 / 1024, so the grid value is exact.
    REQUIRE(from(8191.99f, -8191.99f, 0.0f) == P(8388598 * (Unit / 1024), -8388598 * (Unit / 1024), 0));

    // A value between two grid points goes toward plus infinity: 0.5, 1.5, 2.5 and -0.5, -1.5, -2.5 grid units.
    const float step = 1.0f / float(Unit);
    REQUIRE(from(0.5f * step, 1.5f * step, 2.5f * step) == P(1, 2, 3));
    REQUIRE(from(-0.5f * step, -1.5f * step, -2.5f * step) == P(0, -1, -2));
    // Values off the tie go to the nearest grid point.
    REQUIRE(from(0.75f * step, -0.75f * step, 3.25f * step) == P(1, -1, 3));
    REQUIRE(from(0.25f * step, -0.25f * step, -3.25f * step) == P(0, 0, -3));
    REQUIRE(from(-0.75f * step, 0.4999f * step, -0.5001f * step) == P(-1, 0, -1));
    // Negative values and values near the tie.
    REQUIRE(from(-1.0f, -0.5f, -8191.0f) == P(-Unit, -Unit / 2, -8191 * Unit));
    REQUIRE(from(-1.0f - 0.5f * step, 1.0f + 0.5f * step, -1.0f + 0.5f * step) == P(-Unit, Unit + 1, -Unit + 1));
    REQUIRE(from(std::nextafter(0.5f * step, 1.0f), std::nextafter(-0.5f * step, -1.0f), 0.0f) == P(1, -1, 0));
    REQUIRE(from(std::nextafter(0.5f * step, 0.0f), std::nextafter(-0.5f * step, 0.0f), 0.0f) == P(0, 0, 0));
    // Tiny values and a negative zero snap to zero.
    REQUIRE(from(-0.0f, 1e-30f, -1e-30f) == P(0, 0, 0));
}

TEST_CASE("Grid point snapping matches the floor rule on random values", "[exact]")
{
    std::mt19937_64 random(7);
    std::uniform_real_distribution<float> distribution(-8191.0f, 8191.0f);
    for (int i = 0; i < 20000; ++i)
    {
        const float value = (i & 1) ? distribution(random) : std::ldexp(distribution(random), -static_cast<int>(random() % 20));
        const long double scaled = static_cast<long double>(value) * static_cast<long double>(CsgGridScale);
        const long double expected = std::floor(scaled + 0.5L);
        REQUIRE(static_cast<long double>(CsgGridPoint::From(Vector3(value, 0.0f, 0.0f)).x_) == expected);
    }
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

namespace
{

/// Convert a 128 bit number to a volume expansion.
CsgExactVolume MakeVolume(const Int128& value)
{
    const ea::vector<CsgReal> terms = ToExpansion(value);
    CsgExactVolume result;
    REQUIRE(terms.size() <= 24);
    for (size_t i = 0; i < terms.size(); ++i)
        result.terms_[i] = terms[i];
    result.count_ = CsgCompress(static_cast<int>(terms.size()), result.terms_);
    return result;
}

Int128 Negated(const Int128& value)
{
    return MakeInt128(0) - value;
}

Int128 Absolute(const Int128& value)
{
    return CsgTest::Sign(value) < 0 ? Negated(value) : value;
}

/// Reference for floor(low + (high - low) * n / d + 1/2).
/// It looks for the smallest g with x < g + 1/2 by bisection. The test is exact in 128 bit integers.
int64_t ReferenceRound(int64_t low, int64_t high, Int128 n, Int128 d)
{
    if (CsgTest::Sign(d) < 0)
    {
        n = Negated(n);
        d = Negated(d);
    }
    const int64_t span = high - low;
    int64_t lo = std::min(low, high);
    int64_t hi = std::max(low, high);
    while (lo < hi)
    {
        const int64_t mid = lo + (hi - lo) / 2;
        // x < mid + 1/2 is the same as 2 * span * n < (2 * (mid - low) + 1) * d.
        const Int128 difference = CsgTest::Mul(n, 2 * span) - CsgTest::Mul(d, 2 * (mid - low) + 1);
        if (CsgTest::Sign(difference) < 0)
            hi = mid;
        else
            lo = mid + 1;
    }
    return lo;
}

int64_t FloorDivide(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        --q;
    return q;
}

/// One case of a point on an edge. The fraction n / d is in the range 0 to 1 before the random negation.
struct EdgeCase
{
    int64_t low_;
    int64_t high_;
    Int128 n_;
    Int128 d_;
};

int64_t RandomGrid(std::mt19937_64& random, int64_t range)
{
    return static_cast<int64_t>(random() % static_cast<uint64_t>(2 * range + 1)) - range;
}

/// Pick the ends of an edge. Some edges are short, some are long, some run from high to low.
void RandomEnds(std::mt19937_64& random, EdgeCase& edge)
{
    const int64_t limit = CsgGridIntegerLimit - 1;
    const int64_t ranges[4] = {limit, 1000, 40, limit};
    const int64_t range = ranges[random() % 4];
    edge.low_ = RandomGrid(random, range);
    edge.high_ = RandomGrid(random, range);
    if (random() % 8 == 0)
        edge.high_ = edge.low_ + (random() & 1 ? 1 : -1);
    if (random() % 16 == 0)
        edge.high_ = edge.low_;
    if (random() % 16 == 0)
    {
        edge.low_ = (random() & 1) ? limit : -limit;
        edge.high_ = (random() & 1) ? limit : -limit;
    }
}

/// A fraction n / d with d in [2^(b-1), 2^b) and n in the range 0 to d.
void RandomFraction(std::mt19937_64& random, EdgeCase& edge)
{
    const int bits = 1 + static_cast<int>(random() % (CsgExactVolumeBits - 1));
    edge.d_ = Power2(bits - 1) + Absolute(RandomInt128(random, bits - 1));
    switch (random() % 6)
    {
    case 0: edge.n_ = MakeInt128(0); break;
    case 1: edge.n_ = edge.d_; break;
    case 2: edge.n_ = edge.d_ - Absolute(RandomInt128(random, std::min(bits - 1, 8))); break;
    default: edge.n_ = Absolute(RandomInt128(random, bits - 1)); break;
    }
}

/// A fraction that puts x exactly on a tie, or one unit of n or d away from it.
void TieFraction(std::mt19937_64& random, EdgeCase& edge)
{
    if (edge.low_ == edge.high_)
        edge.high_ = edge.low_ + (edge.low_ > 0 ? -1 : 1);
    const int64_t span = std::abs(edge.high_ - edge.low_);
    // d = 2 * span * q is below 2^CsgExactVolumeBits.
    const int qBits = CsgExactVolumeBits - CsgGridBits - 3;
    const int64_t q = 1 + static_cast<int64_t>(random() % (uint64_t{1} << (random() % qBits + 1)));
    const int64_t c = static_cast<int64_t>(random() % static_cast<uint64_t>(span));
    edge.d_ = CsgTest::Mul(MakeInt128(2 * span), q);
    edge.n_ = CsgTest::Mul(MakeInt128(2 * c + 1), q);
    switch (random() % 4)
    {
    case 0: edge.n_ = edge.n_ + MakeInt128(1); break;
    case 1: edge.n_ = edge.n_ - MakeInt128(1); break;
    case 2: edge.d_ = edge.d_ + MakeInt128(1); break;
    default: break;
    }
}

} // namespace

TEST_CASE("RoundOnEdge on simple edges", "[exact]")
{
    const auto round = [](CsgGridInt low, CsgGridInt high, int64_t n, int64_t d)
    { return CsgExact::RoundOnEdge(low, high, MakeVolume(MakeInt128(n)), MakeVolume(MakeInt128(d))); };

    // x = 0 + 10 * 1/2 = 5. The sign of the denominator does not matter.
    REQUIRE(round(0, 10, 1, 2) == 5);
    REQUIRE(round(0, 10, -1, -2) == 5);
    // Ties go toward plus infinity, also when the edge runs downward.
    REQUIRE(round(0, 1, 1, 2) == 1);
    REQUIRE(round(1, 0, 1, 2) == 1);
    REQUIRE(round(-1, 0, 1, 2) == 0);
    REQUIRE(round(0, -1, 1, 2) == 0);
    REQUIRE(round(0, -1, -1, -2) == 0);
    REQUIRE(round(-3, -2, 1, 2) == -2);
    REQUIRE(round(-2, -3, 1, 2) == -2);
    // Both ends of the fraction.
    REQUIRE(round(-7, 9, 0, 5) == -7);
    REQUIRE(round(-7, 9, 5, 5) == 9);
    REQUIRE(round(-7, 9, -5, -5) == 9);
    REQUIRE(round(4, 4, 3, 7) == 4);
    // x = 3 * 7 / 14 = 1.5 is a tie. The fractions 6/14 and 8/14 give x of about 1.29 and 1.71.
    REQUIRE(round(0, 3, 7, 14) == 2);
    REQUIRE(round(0, 3, 6, 14) == 1);
    REQUIRE(round(0, 3, 8, 14) == 2);

    const CsgGridPoint low = P(0, 10, -10);
    const CsgGridPoint high = P(4, -10, -11);
    const CsgGridPoint point = CsgExact::RoundCrossing(low, high, MakeVolume(MakeInt128(1)), MakeVolume(MakeInt128(2)));
    REQUIRE(point == P(2, 0, -10));
    const CsgGridPoint negated =
        CsgExact::RoundCrossing(low, high, MakeVolume(MakeInt128(-1)), MakeVolume(MakeInt128(-2)));
    REQUIRE(negated == point);
}

TEST_CASE("Negate flips the sign of a volume", "[exact]")
{
    std::mt19937_64 random(11);
    for (int i = 0; i < 200; ++i)
    {
        const Int128 value = RandomInt128(random, CsgExactVolumeBits);
        const CsgExactVolume volume = MakeVolume(value);
        const CsgExactVolume negated = CsgExact::Negate(volume);
        REQUIRE(negated.Sign() == -volume.Sign());
        REQUIRE(ExpansionValue(negated.terms_, negated.count_) == Negated(value));
    }
}

TEST_CASE("RoundOnEdge matches the integer references", "[exact][stress]")
{
    std::mt19937_64 random(2024);
    int tieCount = 0;
    constexpr int CaseCount = 30000;
    for (int i = 0; i < CaseCount; ++i)
    {
        EdgeCase edge;
        RandomEnds(random, edge);
        const bool tie = i % 3 == 1;
        if (tie)
            TieFraction(random, edge);
        else
            RandomFraction(random, edge);
        // A random negation of both numerator and denominator.
        if (random() & 1)
        {
            edge.n_ = Negated(edge.n_);
            edge.d_ = Negated(edge.d_);
        }
        // The near tie may leave the range 0 to 1 only by the tiny unit. Keep the fraction in range.
        const Int128 absoluteN = Absolute(edge.n_);
        const Int128 absoluteD = Absolute(edge.d_);
        if (absoluteN > absoluteD || CsgTest::Sign(absoluteD) == 0)
            continue;
        if (CsgTest::Sign(edge.n_) * CsgTest::Sign(edge.d_) < 0)
            continue;

        const int64_t expected = ReferenceRound(edge.low_, edge.high_, edge.n_, edge.d_);
        const CsgGridInt actual = CsgExact::RoundOnEdge(static_cast<CsgGridInt>(edge.low_),
            static_cast<CsgGridInt>(edge.high_), MakeVolume(edge.n_), MakeVolume(edge.d_));
        REQUIRE(actual == expected);
        tieCount += tie ? 1 : 0;
    }
    REQUIRE(tieCount > 5000);
}

TEST_CASE("RoundOnEdge matches the floor division of small numbers", "[exact]")
{
    std::mt19937_64 random(99);
    for (int i = 0; i < 20000; ++i)
    {
        const int64_t low = RandomGrid(random, i % 2 ? 50 : CsgGridIntegerLimit - 1);
        const int64_t high = RandomGrid(random, i % 2 ? 50 : CsgGridIntegerLimit - 1);
        const int64_t d = 1 + static_cast<int64_t>(random() % (i % 3 == 0 ? 6 : 1000000));
        const int64_t n = static_cast<int64_t>(random() % static_cast<uint64_t>(d + 1));
        const int64_t span = high - low;
        const int64_t expected = FloorDivide(2 * low * d + 2 * span * n + d, 2 * d);
        const bool negate = (random() & 1) != 0;
        const int64_t sn = negate ? -n : n;
        const int64_t sd = negate ? -d : d;
        REQUIRE(CsgExact::RoundOnEdge(static_cast<CsgGridInt>(low), static_cast<CsgGridInt>(high),
                    MakeVolume(MakeInt128(sn)), MakeVolume(MakeInt128(sd)))
            == expected);
    }
}

TEST_CASE("RoundCrossing matches the int128 oracle", "[exact][stress]")
{
    std::mt19937_64 random(5150);
    int tested = 0;
    int degenerate = 0;
    constexpr int CaseCount = 30000;
    for (int i = 0; i < CaseCount; ++i)
    {
        const int64_t range = i % 3 == 0 ? 30 : (i % 3 == 1 ? 5000 : CsgGridIntegerLimit - 1);
        const auto point = [&]() { return P(RandomGrid(random, range), RandomGrid(random, range), RandomGrid(random, range)); };
        const CsgGridPoint t0 = point();
        const CsgGridPoint t1 = point();
        const CsgGridPoint t2 = point();
        const CsgGridPoint low = point();
        const CsgGridPoint high = point();

        // The oracle values. The numerator is n . (low - t0). The denominator is n . (low - high).
        const Int128 lowVolume = CsgTest::ReferenceVolume(t0, t1, t2, low);
        const Int128 highVolume = CsgTest::ReferenceVolume(t0, t1, t2, high);
        const Int128 difference = lowVolume - highVolume;
        if (CsgTest::Sign(difference) == 0)
        {
            ++degenerate;
            continue;
        }
        // The edge must reach the plane: the volumes of both ends have opposite signs or zero.
        if (CsgTest::Sign(lowVolume) * CsgTest::Sign(highVolume) > 0)
            continue;

        const CsgNormal normal = CsgExact::Normal(t0, t1, t2);
        const CsgExactVolume libraryLow = CsgExact::Dot(normal, t0, low);
        const CsgExactVolume libraryDifference = CsgExact::Dot(normal, high, low);
        REQUIRE(ExpansionValue(libraryLow.terms_, libraryLow.count_) == lowVolume);
        REQUIRE(ExpansionValue(libraryDifference.terms_, libraryDifference.count_) == difference);

        const CsgGridPoint actual = CsgExact::RoundCrossing(low, high, libraryLow, libraryDifference);
        REQUIRE(actual.x_ == ReferenceRound(low.x_, high.x_, lowVolume, difference));
        REQUIRE(actual.y_ == ReferenceRound(low.y_, high.y_, lowVolume, difference));
        REQUIRE(actual.z_ == ReferenceRound(low.z_, high.z_, lowVolume, difference));
        ++tested;
    }
    REQUIRE(tested > 10000);
    REQUIRE(degenerate < CaseCount / 2);
}

TEST_CASE("RoundCrossing with a plane through the edge center", "[exact]")
{
    // The plane z = 0 cuts every edge at its center. The center is an exact tie in x, y and z.
    const CsgGridPoint t0 = P(0, 0, 0);
    const CsgGridPoint t1 = P(Unit, 0, 0);
    const CsgGridPoint t2 = P(0, Unit, 0);
    const CsgNormal normal = CsgExact::Normal(t0, t1, t2);
    const auto round = [&](const CsgGridPoint& low, const CsgGridPoint& high)
    {
        return CsgExact::RoundCrossing(
            low, high, CsgExact::Dot(normal, t0, low), CsgExact::Dot(normal, high, low));
    };
    REQUIRE(round(P(0, 0, -1), P(1, -1, 1)) == P(1, 0, 0));
    REQUIRE(round(P(1, -1, 1), P(0, 0, -1)) == P(1, 0, 0));
    REQUIRE(round(P(-3, 5, -1), P(-4, 6, 1)) == P(-3, 6, 0));
    REQUIRE(round(P(-4, 6, 1), P(-3, 5, -1)) == P(-3, 6, 0));
}

// Planar predicates on the integer grid.

namespace
{

using RefInt = CsgTest::Int128;

constexpr int64_t GridLimit = int64_t{1} << CsgGridBits;
/// The largest absolute value of a grid difference.
constexpr int64_t MaxDifference = 2 * GridLimit - 2;

CsgGridPoint2 Pt(int64_t u, int64_t v)
{
    CsgGridPoint2 point;
    point.u_ = static_cast<CsgGridInt>(u);
    point.v_ = static_cast<CsgGridInt>(v);
    return point;
}

int RefSign(const RefInt& value)
{
    return CsgTest::Sign(value);
}

/// Sign of a * b - c * d in the oracle.
int RefMinor(int64_t a, int64_t b, int64_t c, int64_t d)
{
    return RefSign(CsgTest::Mul64<RefInt>(a, b) - CsgTest::Mul64<RefInt>(c, d));
}

int RefCross(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
{
    return RefMinor(d0.u_, d1.v_, d0.v_, d1.u_);
}

int RefDot(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
{
    return RefMinor(d0.u_, d1.u_, -static_cast<int64_t>(d0.v_), d1.v_);
}

int RefOrient(const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c)
{
    const int64_t ux = static_cast<int64_t>(b.u_) - a.u_, uy = static_cast<int64_t>(b.v_) - a.v_;
    const int64_t vx = static_cast<int64_t>(c.u_) - a.u_, vy = static_cast<int64_t>(c.v_) - a.v_;
    return RefMinor(ux, vy, uy, vx);
}

int RefDistance(const CsgGridPoint2& p, const CsgGridPoint2& q0, const CsgGridPoint2& q1)
{
    const auto norm = [&](const CsgGridPoint2& q)
    {
        const int64_t du = static_cast<int64_t>(q.u_) - p.u_, dv = static_cast<int64_t>(q.v_) - p.v_;
        return CsgTest::Mul64<RefInt>(du, du) + CsgTest::Mul64<RefInt>(dv, dv);
    };
    return RefSign(norm(q0) - norm(q1));
}

CsgGridPoint2 Diff(const CsgGridPoint2& to, const CsgGridPoint2& from)
{
    return Pt(static_cast<int64_t>(to.u_) - from.u_, static_cast<int64_t>(to.v_) - from.v_);
}

/// Rank of a direction in the clockwise order from r: 0 for zero, 1 for [0, pi), 2 for [pi, 2 pi).
int RefRank(const CsgGridPoint2& r, const CsgGridPoint2& d)
{
    if (d.u_ == 0 && d.v_ == 0)
        return 0;
    const int cross = RefCross(r, d);
    if (cross != 0)
        return cross < 0 ? 1 : 2;
    return RefDot(r, d) > 0 ? 1 : 2;
}

int RefDirectionCompare(const CsgGridPoint2& p, const CsgGridPoint2& reference, const CsgGridPoint2& q0,
    const CsgGridPoint2& q1)
{
    const CsgGridPoint2 r = Diff(reference, p), d0 = Diff(q0, p), d1 = Diff(q1, p);
    const int rank0 = RefRank(r, d0), rank1 = RefRank(r, d1);
    if (rank0 != rank1)
        return rank0 < rank1 ? -1 : 1;
    if (rank0 == 0)
        return 0;
    return RefCross(d0, d1);
}

bool RefSegmentsCross(const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c, const CsgGridPoint2& d)
{
    const int o1 = RefOrient(a, b, c), o2 = RefOrient(a, b, d), o3 = RefOrient(c, d, a), o4 = RefOrient(c, d, b);
    if (o1 * o2 > 0 || o3 * o4 > 0)
        return false;
    if (o1 != 0 || o2 != 0 || o3 != 0 || o4 != 0)
        return true;
    return std::max(std::min(a.u_, b.u_), std::min(c.u_, d.u_)) <= std::min(std::max(a.u_, b.u_), std::max(c.u_, d.u_))
        && std::max(std::min(a.v_, b.v_), std::min(c.v_, d.v_)) <= std::min(std::max(a.v_, b.v_), std::max(c.v_, d.v_));
}

RefInt RefLoopArea(const CsgGridPoint2* points, int count)
{
    RefInt sum = RefInt(0);
    for (int i = 1; i + 1 < count; ++i)
    {
        const CsgGridPoint2 d0 = Diff(points[i], points[0]), d1 = Diff(points[i + 1], points[0]);
        sum = sum + (CsgTest::Mul64<RefInt>(d0.u_, d1.v_) - CsgTest::Mul64<RefInt>(d0.v_, d1.u_));
    }
    return sum;
}

int RefLoopAreaSign(const CsgGridPoint2* points, int count)
{
    return RefSign(RefLoopArea(points, count));
}

/// Sign of |area of a| - |area of b| in the oracle.
int RefCompareLoopAreaMagnitude(const std::vector<CsgGridPoint2>& a, const std::vector<CsgGridPoint2>& b)
{
    const RefInt areaA = RefLoopArea(a.data(), static_cast<int>(a.size()));
    const RefInt areaB = RefLoopArea(b.data(), static_cast<int>(b.size()));
    return RefSign((RefSign(areaA) < 0 ? -areaA : areaA) - (RefSign(areaB) < 0 ? -areaB : areaB));
}

/// The sum of the doubles that the hole sort used before the exact comparison.
double DoubleLoopAreaMagnitude(const std::vector<CsgGridPoint2>& loop)
{
    double area = 0.0;
    for (size_t i = 1; i + 1 < loop.size(); ++i)
    {
        const CsgGridPoint2 p = Diff(loop[i], loop[0]);
        const CsgGridPoint2 q = Diff(loop[i + 1], loop[0]);
        area += static_cast<double>(p.u_) * q.v_ - static_cast<double>(p.v_) * q.u_;
    }
    return std::fabs(area);
}

int CompareLoopAreaExact(const std::vector<CsgGridPoint2>& a, const std::vector<CsgGridPoint2>& b)
{
    CsgReal termsA[CsgLoopAreaMaxTerms];
    CsgReal termsB[CsgLoopAreaMaxTerms];
    const int countA = CsgExact::LoopAreaExact(a.data(), static_cast<int>(a.size()), termsA);
    const int countB = CsgExact::LoopAreaExact(b.data(), static_cast<int>(b.size()), termsB);
    return CsgExact::CompareLoopAreaMagnitude(countA, termsA, countB, termsB);
}

/// Random grid coordinate. The distributions cover the full range, the extremes, and small values.
int64_t RandomCoordinate(std::mt19937_64& random)
{
    switch (random() % 6)
    {
    case 0: return static_cast<int64_t>(random() % (2 * GridLimit - 1)) - (GridLimit - 1);
    case 1: return (random() & 1) ? GridLimit - 1 : -(GridLimit - 1);
    case 2: return static_cast<int64_t>(random() % 17) - 8;
    case 3: return static_cast<int64_t>(random() % (2 * (GridLimit >> 8) + 1)) - (GridLimit >> 8);
    case 4: return ((random() & 1) ? 1 : -1) * (GridLimit - 1 - static_cast<int64_t>(random() % 64));
    default: return static_cast<int64_t>(random() % (2 * GridLimit - 1)) - (GridLimit - 1);
    }
}

CsgGridPoint2 RandomPoint(std::mt19937_64& random)
{
    const int64_t u = RandomCoordinate(random);
    return Pt(u, RandomCoordinate(random));
}

/// Random difference vector with components in the full range of differences.
CsgGridPoint2 RandomDifference(std::mt19937_64& random)
{
    const auto component = [&]() -> int64_t
    {
        switch (random() % 4)
        {
        case 0: return static_cast<int64_t>(random() % (2 * MaxDifference + 1)) - MaxDifference;
        case 1: return (random() & 1) ? MaxDifference : -MaxDifference;
        case 2: return static_cast<int64_t>(random() % 9) - 4;
        default: return static_cast<int64_t>(random() % (2 * MaxDifference + 1)) - MaxDifference;
        }
    };
    const int64_t u = component();
    return Pt(u, component());
}

bool InverseModulo(int64_t value, int64_t modulus, int64_t& inverse)
{
    int64_t r0 = modulus, r1 = value % modulus, t0 = 0, t1 = 1;
    while (r1 != 0)
    {
        const int64_t quotient = r0 / r1;
        const int64_t r2 = r0 - quotient * r1;
        const int64_t t2 = t0 - quotient * t1;
        r0 = r1;
        r1 = r2;
        t0 = t1;
        t1 = t2;
    }
    if (r0 != 1)
        return false;
    inverse = t0 < 0 ? t0 + modulus : t0;
    return true;
}

/// Make two vectors with nonnegative components up to maxValue, so that d0.u * d1.v - d0.v * d1.u = k.
/// The vectors are large, so the products are near the maximum. The value k is in the range 0 to 2^20.
void MakeCrossPair(std::mt19937_64& random, int64_t maxValue, int64_t k, CsgGridPoint2& d0, CsgGridPoint2& d1)
{
    for (;;)
    {
        const int64_t p = maxValue / 2 + static_cast<int64_t>(random() % (maxValue / 2));
        const int64_t q = maxValue / 2 + static_cast<int64_t>(random() % (maxValue / 2));
        int64_t inverse = 0;
        if (!InverseModulo(q, p, inverse))
            continue;
        // Solve q * x = -k modulo p. Then y = (k + q * x) / p and p * y - q * x = k.
        const int64_t x = (((p - k % p) % p) * inverse) % p;
        const int64_t numerator = k + q * x;
        if (numerator % p != 0)
            continue;
        d0 = Pt(p, q);
        d1 = Pt(x, numerator / p);
        return;
    }
}

/// Place nonnegative difference vectors in the grid. The point p and the points q[i] = p + d[i] are in range.
/// Random sign flips and a swap of the axes cover all quadrants.
void Embed(std::mt19937_64& random, const CsgGridPoint2* d, int count, CsgGridPoint2& p, CsgGridPoint2* q)
{
    int64_t maxU = 0, maxV = 0;
    for (int i = 0; i < count; ++i)
    {
        maxU = std::max<int64_t>(maxU, d[i].u_);
        maxV = std::max<int64_t>(maxV, d[i].v_);
    }
    const auto slack = [&](int64_t maxValue) -> int64_t
    {
        const int64_t room = MaxDifference - maxValue;
        return (random() & 1) ? 0 : static_cast<int64_t>(random() % (room + 1));
    };
    int64_t pu = -(GridLimit - 1) + slack(maxU);
    int64_t pv = -(GridLimit - 1) + slack(maxV);
    int64_t qu[8], qv[8];
    for (int i = 0; i < count; ++i)
    {
        qu[i] = pu + d[i].u_;
        qv[i] = pv + d[i].v_;
    }
    const bool flipU = random() & 1, flipV = random() & 1, swap = random() & 1;
    const auto place = [&](int64_t u, int64_t v)
    {
        if (flipU)
            u = -u;
        if (flipV)
            v = -v;
        return swap ? Pt(v, u) : Pt(u, v);
    };
    p = place(pu, pv);
    for (int i = 0; i < count; ++i)
        q[i] = place(qu[i], qv[i]);
}

/// Make three points a, b, c with Orient2 = k (before the random flips) near the extremes of the grid.
void MakeNearOrient(std::mt19937_64& random, int64_t k, CsgGridPoint2& a, CsgGridPoint2& b, CsgGridPoint2& c)
{
    CsgGridPoint2 d[2];
    MakeCrossPair(random, MaxDifference, k, d[0], d[1]);
    CsgGridPoint2 q[2];
    Embed(random, d, 2, a, q);
    b = q[0];
    c = q[1];
}

bool CheckCross(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
{
    const int expected = RefCross(d0, d1);
    return CsgExact::Cross2(d0, d1) == expected && CsgExact::Cross2Exact(d0, d1) == expected
        && CsgExact::Cross2(d1, d0) == -expected;
}

bool CheckDot(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
{
    const int expected = RefDot(d0, d1);
    return CsgExact::Dot2(d0, d1) == expected && CsgExact::Dot2Exact(d0, d1) == expected
        && CsgExact::Dot2(d1, d0) == expected;
}

bool CheckOrient(const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c)
{
    const int expected = RefOrient(a, b, c);
    return CsgExact::Orient2(a, b, c) == expected && CsgExact::Orient2(b, c, a) == expected
        && CsgExact::Orient2(a, c, b) == -expected;
}

bool CheckDistance(const CsgGridPoint2& p, const CsgGridPoint2& q0, const CsgGridPoint2& q1)
{
    const int expected = RefDistance(p, q0, q1);
    return CsgExact::SquaredDistanceCompare(p, q0, q1) == expected
        && CsgExact::SquaredDistanceCompare(p, q1, q0) == -expected
        && CsgExact::SquaredDistanceCompareExact(Diff(q0, p), Diff(q1, p)) == expected;
}

/// Collinear points: a, b and c on one line through the common direction e, at integer multiples.
void MakeCollinear(std::mt19937_64& random, CsgGridPoint2& a, CsgGridPoint2& b, CsgGridPoint2& c)
{
    const int64_t m = 1 + static_cast<int64_t>(random() % 1000);
    const int64_t reach = (GridLimit / 2) / m;
    const int64_t eu = static_cast<int64_t>(random() % (2 * reach + 1)) - reach;
    const int64_t ev = static_cast<int64_t>(random() % (2 * reach + 1)) - reach;
    const int64_t s = static_cast<int64_t>(random() % (2 * m + 1)) - m;
    const int64_t t = static_cast<int64_t>(random() % (2 * m + 1)) - m;
    const int64_t half = GridLimit / 2;
    a = Pt(static_cast<int64_t>(random() % (2 * half + 1)) - half, static_cast<int64_t>(random() % (2 * half + 1)) - half);
    b = Pt(a.u_ + s * eu, a.v_ + s * ev);
    c = Pt(a.u_ + t * eu, a.v_ + t * ev);
}

constexpr int StressCount = 20000;

} // namespace

TEST_CASE("Project drops an axis and keeps the cyclic order", "[exact][planar]")
{
    const CsgGridPoint point = P(1, 2, 3);
    REQUIRE(CsgExact::Project(point, 0, false) == Pt(2, 3));
    REQUIRE(CsgExact::Project(point, 1, false) == Pt(3, 1));
    REQUIRE(CsgExact::Project(point, 2, false) == Pt(1, 2));
    REQUIRE(CsgExact::Project(point, 0, true) == Pt(3, 2));
    REQUIRE(CsgExact::Project(point, 1, true) == Pt(1, 3));
    REQUIRE(CsgExact::Project(point, 2, true) == Pt(2, 1));

    // The sign of Orient2 of the projection is the sign of the normal component of the dropped axis.
    std::mt19937_64 random(777);
    for (int i = 0; i < 2000; ++i)
    {
        CsgGridPoint t[3];
        for (CsgGridPoint& corner : t)
            corner = P(RandomCoordinate(random), RandomCoordinate(random), RandomCoordinate(random));
        const CsgTest::ReferenceNormal normal = CsgTest::ReferenceNormalOf(t[0], t[1], t[2]);
        const CsgTest::Int128 components[3] = {normal.x_, normal.y_, normal.z_};
        for (int axis = 0; axis < 3; ++axis)
        {
            const int expected = RefSign(components[axis]);
            for (const bool flip : {false, true})
            {
                const int actual = CsgExact::Orient2(
                    CsgExact::Project(t[0], axis, flip), CsgExact::Project(t[1], axis, flip), CsgExact::Project(t[2], axis, flip));
                REQUIRE(actual == (flip ? -expected : expected));
            }
        }
    }
}

TEST_CASE("Planar filter constants", "[exact][planar]")
{
    INFO("planar bound " << CsgPlanarFilterBound << " distance bound " << CsgDistanceFilterBound);
    // The bound is zero exactly when the double result is exact.
    REQUIRE((CsgPlanarFilterBound == 0.0) == CsgPlanarExactInDouble);
    REQUIRE((CsgDistanceFilterBound == 0.0) == CsgDistanceExactInDouble);
#if defined(CSG_EXACT_USE_DOUBLE)
    REQUIRE(CsgPlanarFilterBound == 640.0);
    REQUIRE(CsgDistanceFilterBound == 1536.0);
#else
    REQUIRE(CsgPlanarFilterBound == 0.0);
    REQUIRE(CsgDistanceFilterBound == 0.0);
#endif
}

TEST_CASE("Cross2 and Dot2 match the int128 oracle", "[exact][planar]")
{
    std::mt19937_64 random(1001);
    for (int i = 0; i < StressCount; ++i)
    {
        const CsgGridPoint2 d0 = RandomDifference(random), d1 = RandomDifference(random);
        REQUIRE(CheckCross(d0, d1));
        REQUIRE(CheckDot(d0, d1));
    }
    // Parallel, equal, zero and extreme vectors.
    const int64_t m = MaxDifference;
    const CsgGridPoint2 extremes[] = {Pt(0, 0), Pt(m, m), Pt(-m, m), Pt(m, -m), Pt(-m, -m), Pt(m, 0), Pt(0, m), Pt(-m, 0),
        Pt(0, -m), Pt(1, 0), Pt(0, 1), Pt(1, 1), Pt(-1, 1), Pt(m, 1), Pt(1, m), Pt(m, m - 1), Pt(m - 1, m)};
    for (const CsgGridPoint2& x : extremes)
    {
        for (const CsgGridPoint2& y : extremes)
        {
            REQUIRE(CheckCross(x, y));
            REQUIRE(CheckDot(x, y));
        }
    }
    REQUIRE(CsgExact::Cross2(Pt(m, m), Pt(m, m)) == 0);
    REQUIRE(CsgExact::Cross2(Pt(m, -m), Pt(-m, m)) == 0);
    REQUIRE(CsgExact::Dot2(Pt(m, m), Pt(m, -m)) == 0);
    REQUIRE(CsgExact::Dot2(Pt(m, m), Pt(m, m)) == 1);
}

TEST_CASE("Cross2 and Dot2 near the filter bound", "[exact][planar]")
{
    std::mt19937_64 random(1002);
    for (int i = 0; i < StressCount; ++i)
    {
        // Large vectors with a small determinant: the products are near 2^(2 * GridBits + 2).
        const int64_t k = static_cast<int64_t>(random() % 2049) - 1024;
        CsgGridPoint2 d0, d1;
        MakeCrossPair(random, MaxDifference, k < 0 ? -k : k, d0, d1);
        if (k < 0)
            std::swap(d0, d1);
        if (random() & 1)
            d0 = Pt(-d0.u_, -d0.v_);
        REQUIRE(std::abs(RefCross(d0, d1)) == (k == 0 ? 0 : 1));
        REQUIRE(CheckCross(d0, d1));
        // A quarter turn of d1 turns the cross product into a dot product.
        REQUIRE(CheckDot(d0, Pt(d1.v_, -d1.u_)));
        REQUIRE(CheckDot(d0, Pt(-d1.v_, d1.u_)));
    }
    // The exact value of the determinant is k.
    for (int64_t k = 0; k <= 1024; ++k)
    {
        CsgGridPoint2 d0, d1;
        MakeCrossPair(random, MaxDifference, k, d0, d1);
        const int64_t exact = static_cast<int64_t>(d0.u_) * d1.v_ - static_cast<int64_t>(d0.v_) * d1.u_;
        REQUIRE(exact == k);
        REQUIRE(CsgExact::Cross2(d0, d1) == (k == 0 ? 0 : 1));
        REQUIRE(CsgExact::Cross2(d1, d0) == (k == 0 ? 0 : -1));
        REQUIRE(CsgExact::Dot2(d0, Pt(d1.v_, -d1.u_)) == (k == 0 ? 0 : 1));
    }
}

TEST_CASE("Orient2 matches the int128 oracle", "[exact][planar]")
{
    std::mt19937_64 random(1003);
    for (int i = 0; i < StressCount; ++i)
    {
        const CsgGridPoint2 a = RandomPoint(random), b = RandomPoint(random), c = RandomPoint(random);
        REQUIRE(CheckOrient(a, b, c));
        REQUIRE(CheckOrient(a, a, c));
        REQUIRE(CheckOrient(a, b, a));
        REQUIRE(CheckOrient(a, b, b));
    }
    for (int i = 0; i < StressCount; ++i)
    {
        CsgGridPoint2 a, b, c;
        MakeCollinear(random, a, b, c);
        REQUIRE(RefOrient(a, b, c) == 0);
        REQUIRE(CsgExact::Orient2(a, b, c) == 0);
        REQUIRE(CsgExact::Orient2(c, b, a) == 0);
        // Move one point by one step: the determinant is no longer zero, in general.
        c.u_ += (c.u_ < GridLimit - 1) ? 1 : -1;
        REQUIRE(CheckOrient(a, b, c));
    }
    // The corners of the grid.
    const int64_t e = GridLimit - 1;
    const CsgGridPoint2 corners[] = {Pt(e, e), Pt(-e, e), Pt(e, -e), Pt(-e, -e), Pt(0, 0), Pt(e, 0), Pt(0, -e), Pt(e, e - 1)};
    for (const CsgGridPoint2& a : corners)
        for (const CsgGridPoint2& b : corners)
            for (const CsgGridPoint2& c : corners)
                REQUIRE(CheckOrient(a, b, c));
    REQUIRE(CsgExact::Orient2(Pt(0, 0), Pt(1, 0), Pt(0, 1)) == 1);
    REQUIRE(CsgExact::Orient2(Pt(0, 0), Pt(0, 1), Pt(1, 0)) == -1);
    REQUIRE(CsgExact::Orient2(Pt(-e, -e), Pt(e, e), Pt(0, 0)) == 0);
    REQUIRE(CsgExact::Orient2(Pt(-e, -e), Pt(e, e), Pt(1, 0)) == -1);
}

TEST_CASE("Orient2 with a small determinant and large coordinates", "[exact][planar]")
{
    std::mt19937_64 random(1004);
    for (int i = 0; i < StressCount; ++i)
    {
        const int64_t k = static_cast<int64_t>(random() % 2049) - 1024;
        CsgGridPoint2 a, b, c;
        MakeNearOrient(random, k < 0 ? -k : k, a, b, c);
        if (k < 0)
            std::swap(b, c);
        REQUIRE(CheckOrient(a, b, c));
        const int64_t u1 = static_cast<int64_t>(b.u_) - a.u_, v1 = static_cast<int64_t>(b.v_) - a.v_;
        const int64_t u2 = static_cast<int64_t>(c.u_) - a.u_, v2 = static_cast<int64_t>(c.v_) - a.v_;
        const int64_t exact = u1 * v2 - v1 * u2;
        // The embedding can swap the axes. It then changes the sign.
        REQUIRE(std::abs(exact) == std::abs(k));
        REQUIRE(CsgExact::Orient2(a, b, c) == (exact > 0) - (exact < 0));
    }
}

TEST_CASE("SquaredDistanceCompare matches the int128 oracle", "[exact][planar]")
{
    std::mt19937_64 random(1005);
    for (int i = 0; i < StressCount; ++i)
    {
        const CsgGridPoint2 p = RandomPoint(random), q0 = RandomPoint(random), q1 = RandomPoint(random);
        REQUIRE(CheckDistance(p, q0, q1));
        REQUIRE(CheckDistance(p, q0, q0));
        REQUIRE(CheckDistance(p, p, q1));
        // Same distance: swapped and negated difference components.
        const CsgGridPoint2 d = Diff(q0, p);
        const CsgGridPoint2 turned = Pt(-static_cast<int64_t>(d.v_), d.u_);
        const int64_t u = static_cast<int64_t>(p.u_) + turned.u_, v = static_cast<int64_t>(p.v_) + turned.v_;
        if (std::abs(u) < GridLimit && std::abs(v) < GridLimit)
        {
            REQUIRE(CheckDistance(p, q0, Pt(u, v)));
            REQUIRE(CsgExact::SquaredDistanceCompare(p, q0, Pt(u, v)) == 0);
        }
    }
    // Large vectors with a squared distance difference of 0 to 1024 in absolute value.
    // (c + s)^2 + (e - 1)^2 versus c^2 + e^2 with e fixed by the target k.
    for (int i = 0; i < StressCount; ++i)
    {
        const int64_t k = static_cast<int64_t>(random() % 2049) - 1024;
        CsgGridPoint2 d0, d1;
        if ((k & 1) == 0)
        {
            // s = 1, t = -1: the difference is 2 * (c - e) + 2 = k.
            const int64_t e = MaxDifference / 2 + static_cast<int64_t>(random() % (MaxDifference / 2 - 2000));
            const int64_t c = e - 1 + k / 2;
            d0 = Pt(c + 1, e - 1);
            d1 = Pt(c, e);
        }
        else
        {
            // s = 2, t = -1: the difference is 4 * c - 2 * e + 5 = k.
            const int64_t c = MaxDifference / 4 + static_cast<int64_t>(random() % (MaxDifference / 4 - 2000));
            const int64_t e = 2 * c + (5 - k) / 2;
            d0 = Pt(c + 2, e - 1);
            d1 = Pt(c, e);
        }
        REQUIRE(d0.u_ >= 0);
        REQUIRE(d0.v_ >= 0);
        REQUIRE(d1.u_ >= 0);
        REQUIRE(d1.v_ >= 0);
        REQUIRE(std::max<int64_t>(std::max<int64_t>(d0.u_, d0.v_), std::max<int64_t>(d1.u_, d1.v_)) <= MaxDifference);
        const int64_t n0 = static_cast<int64_t>(d0.u_) * d0.u_ + static_cast<int64_t>(d0.v_) * d0.v_;
        const int64_t n1 = static_cast<int64_t>(d1.u_) * d1.u_ + static_cast<int64_t>(d1.v_) * d1.v_;
        REQUIRE(n0 - n1 == k);

        CsgGridPoint2 p;
        CsgGridPoint2 q[2];
        const CsgGridPoint2 pair[2] = {d0, d1};
        Embed(random, pair, 2, p, q);
        REQUIRE(CheckDistance(p, q[0], q[1]));
        REQUIRE(CsgExact::SquaredDistanceCompare(p, q[0], q[1]) == (k > 0) - (k < 0));
    }
}

TEST_CASE("DirectionCompare is a strict weak order", "[exact][planar]")
{
    // 16 distinct rays, in the counterclockwise order.
    const CsgGridPoint2 rays[16] = {Pt(1, 0), Pt(2, 1), Pt(1, 1), Pt(1, 2), Pt(0, 1), Pt(-1, 2), Pt(-1, 1), Pt(-2, 1),
        Pt(-1, 0), Pt(-2, -1), Pt(-1, -1), Pt(-1, -2), Pt(0, -1), Pt(1, -2), Pt(1, -1), Pt(2, -1)};
    const int64_t scales[] = {1, 3, 1000};
    std::vector<CsgGridPoint2> directions;
    for (const CsgGridPoint2& ray : rays)
        for (const int64_t scale : scales)
            directions.push_back(Pt(ray.u_ * scale, ray.v_ * scale));
    directions.push_back(Pt(0, 0));
    const int count = static_cast<int>(directions.size());
    const CsgGridPoint2 p = Pt(100, -50);
    const auto at = [&](int i) { return Pt(p.u_ + directions[i].u_, p.v_ + directions[i].v_); };

    for (int refIndex = 0; refIndex < 16; ++refIndex)
    {
        const CsgGridPoint2 reference = at(refIndex * 3);
        // The expected order: rank, then the clockwise angle from the reference.
        const double referenceAngle = std::atan2(double(rays[refIndex].v_), double(rays[refIndex].u_));
        const auto key = [&](int i) -> std::pair<int, double>
        {
            if (i == count - 1)
                return {0, 0.0};
            const CsgGridPoint2& ray = rays[i / 3];
            double angle = referenceAngle - std::atan2(double(ray.v_), double(ray.u_));
            if (angle < -1e-9)
                angle += 2.0 * 3.14159265358979323846;
            return {1, angle < 1e-9 ? 0.0 : angle};
        };

        std::vector<int> less(count * count), equal(count * count);
        for (int i = 0; i < count; ++i)
        {
            for (int j = 0; j < count; ++j)
            {
                const int result = CsgExact::DirectionCompare(p, reference, at(i), at(j));
                REQUIRE(result == RefDirectionCompare(p, reference, at(i), at(j)));
                // Antisymmetry.
                REQUIRE(result == -CsgExact::DirectionCompare(p, reference, at(j), at(i)));
                less[i * count + j] = result < 0;
                equal[i * count + j] = result == 0;
                // Equal exactly for the same ray or two zero directions.
                const bool sameRay = (i / 3 == j / 3) || (i == count - 1 && j == count - 1);
                REQUIRE((result == 0) == (sameRay && ((i == count - 1) == (j == count - 1))));
                // The order follows the angle.
                const auto ki = key(i), kj = key(j);
                const bool angleLess = ki.first != kj.first ? ki.first < kj.first : ki.second < kj.second - 1e-6;
                const bool angleEqual = ki.first == kj.first && std::fabs(ki.second - kj.second) <= 1e-6;
                REQUIRE((result < 0) == angleLess);
                REQUIRE((result == 0) == angleEqual);
            }
        }
        // Transitivity of the order and of the equality, over all triples.
        for (int i = 0; i < count; ++i)
        {
            REQUIRE(!less[i * count + i]);
            REQUIRE(equal[i * count + i]);
            for (int j = 0; j < count; ++j)
            {
                for (int k = 0; k < count; ++k)
                {
                    if (less[i * count + j] && less[j * count + k])
                        REQUIRE(less[i * count + k]);
                    if (equal[i * count + j] && equal[j * count + k])
                        REQUIRE(equal[i * count + k]);
                    // Transitivity of the incomparability.
                    if (equal[i * count + j] && less[j * count + k])
                        REQUIRE(less[i * count + k]);
                    if (less[i * count + j] && equal[j * count + k])
                        REQUIRE(less[i * count + k]);
                }
            }
        }
    }

    // The reference direction itself comes first, the opposite direction is the first of the second half.
    REQUIRE(CsgExact::DirectionCompare(Pt(0, 0), Pt(1, 0), Pt(5, 0), Pt(0, -5)) < 0);
    REQUIRE(CsgExact::DirectionCompare(Pt(0, 0), Pt(1, 0), Pt(0, -5), Pt(0, 5)) < 0);
    REQUIRE(CsgExact::DirectionCompare(Pt(0, 0), Pt(1, 0), Pt(-5, 0), Pt(0, 5)) < 0);
    REQUIRE(CsgExact::DirectionCompare(Pt(0, 0), Pt(1, 0), Pt(0, 5), Pt(5, 0)) > 0);
    REQUIRE(CsgExact::DirectionCompare(Pt(0, 0), Pt(1, 0), Pt(0, 0), Pt(5, 0)) < 0);
}

TEST_CASE("DirectionCompare matches the int128 oracle", "[exact][planar]")
{
    std::mt19937_64 random(1006);
    for (int i = 0; i < StressCount; ++i)
    {
        const CsgGridPoint2 p = RandomPoint(random), q0 = RandomPoint(random), q1 = RandomPoint(random);
        CsgGridPoint2 reference = RandomPoint(random);
        if (reference == p)
            reference.u_ += (reference.u_ < GridLimit - 1) ? 1 : -1;
        const int expected = RefDirectionCompare(p, reference, q0, q1);
        REQUIRE(CsgExact::DirectionCompare(p, reference, q0, q1) == expected);
        REQUIRE(CsgExact::DirectionCompare(p, reference, q1, q0) == -expected);
        REQUIRE(CsgExact::DirectionCompare(p, reference, q0, q0) == 0);
    }
    // Collinear points: equal and opposite directions, and the reference on the same line.
    for (int i = 0; i < StressCount; ++i)
    {
        CsgGridPoint2 a, b, c;
        MakeCollinear(random, a, b, c);
        const CsgGridPoint2 reference = (c == a) ? b : c;
        if (reference == a)
            continue;
        const int expected = RefDirectionCompare(a, reference, b, c);
        REQUIRE(CsgExact::DirectionCompare(a, reference, b, c) == expected);
        REQUIRE(CsgExact::DirectionCompare(a, reference, c, b) == -expected);
    }
    // Near collinear with a large coordinate: the cross product is k in 0 to 1024.
    for (int i = 0; i < StressCount; ++i)
    {
        const int64_t k = static_cast<int64_t>(random() % 2049) - 1024;
        CsgGridPoint2 a, b, c;
        MakeNearOrient(random, k < 0 ? -k : k, a, b, c);
        if (k < 0)
            std::swap(b, c);
        const CsgGridPoint2 reference = b;
        REQUIRE(CsgExact::DirectionCompare(a, reference, b, c) == RefDirectionCompare(a, reference, b, c));
        REQUIRE(CsgExact::DirectionCompare(a, reference, c, b) == RefDirectionCompare(a, reference, c, b));
        if (c != a)
            REQUIRE(CsgExact::DirectionCompare(a, c, b, c) == RefDirectionCompare(a, c, b, c));
    }
}

TEST_CASE("SegmentsCross cases", "[exact][planar]")
{
    // Proper crossing.
    REQUIRE(CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 10), Pt(0, 10), Pt(10, 0)));
    // The end of one segment touches the inside of the other.
    REQUIRE(CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 0), Pt(5, 0), Pt(5, 7)));
    REQUIRE(CsgExact::SegmentsCross(Pt(5, 7), Pt(5, 0), Pt(0, 0), Pt(10, 0)));
    // A common end point.
    REQUIRE(CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 0), Pt(10, 0), Pt(10, 7)));
    REQUIRE(CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 0), Pt(20, 5), Pt(10, 0)));
    // Disjoint.
    REQUIRE(!CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 0), Pt(5, 1), Pt(5, 7)));
    REQUIRE(!CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 0), Pt(11, -3), Pt(11, 3)));
    // The line through one segment cuts the other, but the segments do not meet.
    REQUIRE(!CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 0), Pt(20, -5), Pt(20, 5)));
    // Collinear overlap.
    REQUIRE(CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 10), Pt(5, 5), Pt(20, 20)));
    REQUIRE(CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 10), Pt(3, 3), Pt(4, 4)));
    REQUIRE(CsgExact::SegmentsCross(Pt(10, 10), Pt(0, 0), Pt(20, 20), Pt(5, 5)));
    // Collinear, touching in one point.
    REQUIRE(CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 10), Pt(10, 10), Pt(20, 20)));
    // Collinear, disjoint.
    REQUIRE(!CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 10), Pt(11, 11), Pt(20, 20)));
    REQUIRE(!CsgExact::SegmentsCross(Pt(0, 0), Pt(10, 0), Pt(-5, 0), Pt(-1, 0)));
    REQUIRE(!CsgExact::SegmentsCross(Pt(0, 5), Pt(0, 9), Pt(0, 1), Pt(0, 4)));
    // Points as segments.
    REQUIRE(CsgExact::SegmentsCross(Pt(3, 3), Pt(3, 3), Pt(0, 0), Pt(10, 10)));
    REQUIRE(!CsgExact::SegmentsCross(Pt(3, 4), Pt(3, 4), Pt(0, 0), Pt(10, 10)));
    REQUIRE(CsgExact::SegmentsCross(Pt(3, 3), Pt(3, 3), Pt(3, 3), Pt(3, 3)));
    REQUIRE(!CsgExact::SegmentsCross(Pt(3, 3), Pt(3, 3), Pt(3, 4), Pt(3, 4)));
    // Extreme coordinates: a diagonal and the other diagonal of the grid, and a segment just next to the center.
    const int64_t e = GridLimit - 1;
    REQUIRE(CsgExact::SegmentsCross(Pt(-e, -e), Pt(e, e), Pt(-e, e), Pt(e, -e)));
    REQUIRE(CsgExact::SegmentsCross(Pt(-e, -e), Pt(e, e), Pt(0, 0), Pt(0, 1)));
    REQUIRE(!CsgExact::SegmentsCross(Pt(-e, -e), Pt(e, e), Pt(1, 0), Pt(e, 1)));
    REQUIRE(CsgExact::SegmentsCross(Pt(-e, -e), Pt(e - 1, e), Pt(-e, e), Pt(e, -e)));
}

TEST_CASE("SegmentsCross matches an independent oracle", "[exact][planar]")
{
    // The oracle intersects the parametric lines. It uses 64 bit integers: the values are below 2^62.
    const auto parametric = [](const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c,
                                const CsgGridPoint2& d)
    {
        const int64_t rU = int64_t(b.u_) - a.u_, rV = int64_t(b.v_) - a.v_;
        const int64_t sU = int64_t(d.u_) - c.u_, sV = int64_t(d.v_) - c.v_;
        const int64_t qU = int64_t(c.u_) - a.u_, qV = int64_t(c.v_) - a.v_;
        int64_t denominator = rU * sV - rV * sU;
        int64_t tNumerator = qU * sV - qV * sU;
        int64_t uNumerator = qU * rV - qV * rU;
        if (denominator == 0)
        {
            if (uNumerator != 0)
                return false;
            // Collinear: the intervals of the parameter t = (x - a) . r / (r . r) overlap.
            const int64_t t0 = qU * rU + qV * rV;
            const int64_t t1 = t0 + sU * rU + sV * rV;
            const int64_t length = rU * rU + rV * rV;
            return std::max(std::min(t0, t1), int64_t{0}) <= std::min(std::max(t0, t1), length);
        }
        if (denominator < 0)
        {
            denominator = -denominator;
            tNumerator = -tNumerator;
            uNumerator = -uNumerator;
        }
        return tNumerator >= 0 && tNumerator <= denominator && uNumerator >= 0 && uNumerator <= denominator;
    };

    std::mt19937_64 random(1007);
    int crossing = 0;
    for (int i = 0; i < StressCount; ++i)
    {
        CsgGridPoint2 a, b, c, d;
        if (i % 2 == 0)
        {
            // Small grid: many touching and collinear cases.
            const auto small = [&]() { return Pt(int64_t(random() % 7) - 3, int64_t(random() % 7) - 3); };
            a = small();
            b = small();
            c = small();
            d = small();
        }
        else
        {
            a = RandomPoint(random);
            b = RandomPoint(random);
            c = RandomPoint(random);
            d = RandomPoint(random);
        }
        if (a == b || c == d)
            continue;
        const bool expected = parametric(a, b, c, d);
        REQUIRE(CsgExact::SegmentsCross(a, b, c, d) == expected);
        REQUIRE(CsgExact::SegmentsCross(c, d, a, b) == expected);
        REQUIRE(CsgExact::SegmentsCross(b, a, d, c) == expected);
        crossing += expected;
    }
    REQUIRE(crossing > 1000);
}

TEST_CASE("LoopAreaSign matches the int128 oracle", "[exact][planar]")
{
    std::mt19937_64 random(1008);
    std::vector<CsgGridPoint2> loop;
    for (int i = 0; i < 4000; ++i)
    {
        const int count = (i % 5 == 0) ? 250 + static_cast<int>(random() % 60) : 3 + static_cast<int>(random() % 12);
        loop.clear();
        for (int j = 0; j < count; ++j)
            loop.push_back(RandomPoint(random));
        const int expected = RefLoopAreaSign(loop.data(), count);
        REQUIRE(CsgExact::LoopAreaSign(loop.data(), count) == expected);
        REQUIRE(CsgExact::LoopAreaSignExact(loop.data(), count) == expected);
        // The reversed loop has the opposite area.
        std::reverse(loop.begin(), loop.end());
        REQUIRE(CsgExact::LoopAreaSign(loop.data(), count) == -expected);
        REQUIRE(RefLoopAreaSign(loop.data(), count) == -expected);
    }
    // Triangles agree with Orient2.
    for (int i = 0; i < StressCount; ++i)
    {
        const CsgGridPoint2 triangle[3] = {RandomPoint(random), RandomPoint(random), RandomPoint(random)};
        REQUIRE(CsgExact::LoopAreaSign(triangle, 3) == RefOrient(triangle[0], triangle[1], triangle[2]));
        REQUIRE(CsgExact::LoopAreaSign(triangle, 3) == CsgExact::Orient2(triangle[0], triangle[1], triangle[2]));
    }
    // A square, counterclockwise and clockwise. Short loops have the area 0.
    const CsgGridPoint2 square[4] = {Pt(0, 0), Pt(4, 0), Pt(4, 4), Pt(0, 4)};
    REQUIRE(CsgExact::LoopAreaSign(square, 4) == 1);
    const CsgGridPoint2 reversed[4] = {Pt(0, 4), Pt(4, 4), Pt(4, 0), Pt(0, 0)};
    REQUIRE(CsgExact::LoopAreaSign(reversed, 4) == -1);
    REQUIRE(CsgExact::LoopAreaSign(square, 2) == 0);
    REQUIRE(CsgExact::LoopAreaSign(square, 0) == 0);
    REQUIRE(CsgExact::LoopAreaSign(nullptr, 0) == 0);
}

TEST_CASE("LoopAreaSign with 256 edges and alternating signs", "[exact][planar]")
{
    std::mt19937_64 random(1009);
    // Points A, B, A, B, ... around the origin. Each pair of terms is cross(A, B) and cross(B, A).
    // The terms alternate in sign and cancel. A large cross(A, B) makes the partial sums inexact in the filter.
    for (int i = 0; i < 400; ++i)
    {
        const int64_t k = static_cast<int64_t>(random() % 1025);
        CsgGridPoint2 d0, d1;
        MakeCrossPair(random, GridLimit - 1, k, d0, d1);
        if (random() & 1)
            std::swap(d0, d1);
        const int64_t cross = static_cast<int64_t>(d0.u_) * d1.v_ - static_cast<int64_t>(d0.v_) * d1.u_;

        for (const int count : {256, 255, 254, 257, 300})
        {
            std::vector<CsgGridPoint2> loop(count);
            loop[0] = Pt(0, 0);
            for (int j = 1; j < count; ++j)
                loop[j] = (j & 1) ? d0 : d1;
            // The terms are cross(A, B), cross(B, A), ... : term t has the sign (-1)^t times cross(A, B).
            const int terms = count - 2;
            const int64_t sum = (terms & 1) ? cross : 0;
            const int expected = (sum > 0) - (sum < 0);
            REQUIRE(RefLoopAreaSign(loop.data(), count) == expected);
            REQUIRE(CsgExact::LoopAreaSign(loop.data(), count) == expected);
            REQUIRE(CsgExact::LoopAreaSignExact(loop.data(), count) == expected);
        }
    }

    // Large terms of random sign, and a loop that goes out and back on the same points.
    std::vector<CsgGridPoint2> loop(256);
    int zeroSums = 0;
    for (int i = 0; i < 2000; ++i)
    {
        for (CsgGridPoint2& point : loop)
            point = RandomPoint(random);
        // Make the second half of the loop the mirror of the first half: the area is 0.
        for (int j = 1; j < 128; ++j)
            loop[256 - j] = loop[j];
        const int expected = RefLoopAreaSign(loop.data(), 256);
        zeroSums += expected == 0;
        REQUIRE(CsgExact::LoopAreaSign(loop.data(), 256) == expected);
        REQUIRE(CsgExact::LoopAreaSignExact(loop.data(), 256) == expected);
    }
    // The mirrored loop goes out and comes back on the same points: the area is always 0.
    REQUIRE(zeroSums == 2000);
}

TEST_CASE("LoopAreaSign of a collapsed loop is zero", "[exact][planar]")
{
    std::mt19937_64 random(1010);
    // Out and back on the same points.
    for (int i = 0; i < 2000; ++i)
    {
        const int half = 2 + static_cast<int>(random() % 100);
        std::vector<CsgGridPoint2> loop;
        for (int j = 0; j < half; ++j)
            loop.push_back(RandomPoint(random));
        for (int j = half - 2; j >= 1; --j)
            loop.push_back(loop[j]);
        const int count = static_cast<int>(loop.size());
        REQUIRE(RefLoopAreaSign(loop.data(), count) == 0);
        REQUIRE(CsgExact::LoopAreaSign(loop.data(), count) == 0);
        REQUIRE(CsgExact::LoopAreaSignExact(loop.data(), count) == 0);
    }
    // All points on one line and all points equal.
    for (int i = 0; i < 2000; ++i)
    {
        CsgGridPoint2 a, b, c;
        MakeCollinear(random, a, b, c);
        const CsgGridPoint2 line[6] = {a, b, c, b, a, c};
        REQUIRE(CsgExact::LoopAreaSign(line, 6) == 0);
        REQUIRE(CsgExact::LoopAreaSignExact(line, 6) == 0);
        const CsgGridPoint2 same[5] = {a, a, a, a, a};
        REQUIRE(CsgExact::LoopAreaSign(same, 5) == 0);
    }
    // A thin sliver of area 1/2 unit square stays nonzero.
    const CsgGridPoint2 sliver[3] = {Pt(-(GridLimit - 1), -(GridLimit - 1)), Pt(GridLimit - 1, GridLimit - 1), Pt(GridLimit - 1, GridLimit - 2)};
    REQUIRE(CsgExact::LoopAreaSign(sliver, 3) == -1);
}

TEST_CASE("LoopAreaExact has the sign of the int128 oracle", "[exact][planar]")
{
    std::mt19937_64 random(1020);
    std::vector<CsgGridPoint2> loop;
    for (int i = 0; i < 2000; ++i)
    {
        const int count = (i % 5 == 0) ? 250 + static_cast<int>(random() % 60) : 3 + static_cast<int>(random() % 12);
        loop.clear();
        for (int j = 0; j < count; ++j)
            loop.push_back(RandomPoint(random));
        CsgReal terms[CsgLoopAreaMaxTerms];
        const int termCount = CsgExact::LoopAreaExact(loop.data(), count, terms);
        REQUIRE(CsgExpansionSign(termCount, terms) == RefLoopAreaSign(loop.data(), count));
    }
    CsgReal terms[CsgLoopAreaMaxTerms];
    REQUIRE(CsgExact::LoopAreaExact(loop.data(), 2, terms) == 0);
    REQUIRE(CsgExact::LoopAreaExact(nullptr, 0, terms) == 0);
}

TEST_CASE("LoopAreaExact stays in the term limit and rejects long loops", "[exact][planar]")
{
    static_assert(CsgLoopAreaMaxTerms >= CsgExactPlanarBits + CsgLoopAreaPointBits, "The term limit covers the bound.");
    const CsgGridInt extreme = static_cast<CsgGridInt>(CsgGridIntegerLimit - 1);
    std::mt19937_64 random(1040);
    std::vector<CsgGridPoint2> loop;
    CsgReal terms[CsgLoopAreaMaxTerms];
    for (int trial = 0; trial < 4; ++trial)
    {
        // The longest allowed loop with the largest coordinates and many random corner signs.
        loop.clear();
        for (int j = 0; j < CsgLoopAreaMaxPoints; ++j)
        {
            const uint64_t bits = random();
            CsgGridPoint2 p;
            p.u_ = (bits & 1) ? extreme : -extreme;
            p.v_ = (bits & 2) ? extreme : -extreme;
            if (trial >= 2 && (bits & 4))
                p.u_ = static_cast<CsgGridInt>(random() % (2 * extreme + 1)) - extreme;
            loop.push_back(p);
        }
        const int count = static_cast<int>(loop.size());
        const int termCount = CsgExact::LoopAreaExact(loop.data(), count, terms);
        REQUIRE(termCount >= 0);
        REQUIRE(termCount <= CsgLoopAreaMaxTerms);
        REQUIRE(CsgExpansionSign(termCount, terms) == RefLoopAreaSign(loop.data(), count));
        REQUIRE(CsgExact::LoopAreaSignExact(loop.data(), count) == RefLoopAreaSign(loop.data(), count));

        // One more point is over the limit. There is no exact area, and the sign still has a value.
        loop.push_back(loop.front());
        const int longCount = static_cast<int>(loop.size());
        REQUIRE(CsgExact::LoopAreaExact(loop.data(), longCount, terms) == -1);
        const int sign = CsgExact::LoopAreaSignExact(loop.data(), longCount);
        REQUIRE(sign >= -1);
        REQUIRE(sign <= 1);
        REQUIRE(CsgExact::CompareLoopAreaMagnitude(-1, terms, termCount, terms) == 0);
    }
}

TEST_CASE("CompareLoopAreaMagnitude matches the int128 oracle", "[exact][planar]")
{
    std::mt19937_64 random(1021);
    std::vector<CsgGridPoint2> a;
    std::vector<CsgGridPoint2> b;
    int counts[3] = {};
    for (int i = 0; i < 3000; ++i)
    {
        a.clear();
        const int count = 3 + static_cast<int>(random() % 20);
        for (int j = 0; j < count; ++j)
            a.push_back(RandomPoint(random));
        b.clear();
        switch (i % 4)
        {
        case 0: // A random loop.
            for (int j = 0, n = 3 + static_cast<int>(random() % 20); j < n; ++j)
                b.push_back(RandomPoint(random));
            break;
        case 1: // The reverse loop has the same magnitude.
            b.assign(a.rbegin(), a.rend());
            break;
        case 2: // One vertex moves by at most 2 grid units: the areas are close.
            b = a;
            b[1 + random() % (count - 1)].v_ += (random() & 1) ? 1 : -1;
            break;
        default: // A loop with a repeated start and a zero area.
            b.assign(a.size(), a[0]);
            break;
        }
        const int expected = RefCompareLoopAreaMagnitude(a, b);
        REQUIRE(CompareLoopAreaExact(a, b) == expected);
        REQUIRE(CompareLoopAreaExact(b, a) == -expected);
        REQUIRE(CompareLoopAreaExact(a, a) == 0);
        ++counts[expected + 1];
    }
    REQUIRE(counts[0] > 100);
    REQUIRE(counts[1] > 100);
    REQUIRE(counts[2] > 100);
}

TEST_CASE("CompareLoopAreaMagnitude separates areas that differ by 1", "[exact][planar]")
{
    // A star polygon winds around its center many times. The doubled area and the partial sums of the fan are large.
    // The larger loop has the exact area D + 1 and the smaller one has D. D needs many terms of the double sum.
    // Float mode needs a star with many vertices: the terms are below 2^49 and the partial sums reach 2^53 slowly.
    // Double mode has terms of 2^58 and more, so 20 vertices are enough.
    constexpr int VertexCount = CsgGridBits <= 24 ? 301 : 20;
    constexpr int Step = CsgGridBits <= 24 ? 75 : 3;
    constexpr double Pi = 3.14159265358979323846;
    std::mt19937_64 random(1022);
    int doubleWrong = 0;
    int maxTerms = 0;
    constexpr int Trials = 200;
    for (int trial = 0; trial < Trials; ++trial)
    {
        const double radius = static_cast<double>(GridLimit - 16 - static_cast<int64_t>(random() % 8));
        const double phase = static_cast<double>(random() % 1000) * 0.001;
        std::vector<CsgGridPoint2> smaller;
        for (int i = 0; i < VertexCount; ++i)
        {
            const double angle = phase + 2.0 * Pi * (static_cast<double>(i) * Step / VertexCount);
            smaller.push_back(Pt(std::llround(radius * std::cos(angle)), std::llround(radius * std::sin(angle))));
        }
        // The neighbors of the vertex j have the u coordinates a and a - 1. A step of 1 in v of the vertex j
        // changes the doubled area by exactly 1.
        const int j = 2 + static_cast<int>(random() % (VertexCount - 4));
        smaller[j + 1].u_ = smaller[j - 1].u_ - 1;
        std::vector<CsgGridPoint2> larger = smaller;
        const RefInt before = RefLoopArea(smaller.data(), VertexCount);
        for (const int delta : {1, -1})
        {
            larger[j].v_ = smaller[j].v_ + delta;
            const RefInt after = RefLoopArea(larger.data(), VertexCount);
            REQUIRE(RefSign(after - before) != 0);
            // The loops have one orientation. The order of the magnitudes is the order of the signed areas.
            REQUIRE(RefSign(RefLoopArea(smaller.data(), VertexCount)) != 0);
            const int expected = RefCompareLoopAreaMagnitude(larger, smaller);
            REQUIRE(expected != 0);
            REQUIRE(CompareLoopAreaExact(larger, smaller) == expected);
            REQUIRE(CompareLoopAreaExact(smaller, larger) == -expected);

            const double doubleLarger = DoubleLoopAreaMagnitude(larger);
            const double doubleSmaller = DoubleLoopAreaMagnitude(smaller);
            const int doubleSign = doubleLarger > doubleSmaller ? 1 : (doubleLarger < doubleSmaller ? -1 : 0);
            doubleWrong += doubleSign != expected;

            CsgReal terms[CsgLoopAreaMaxTerms];
            maxTerms = std::max(maxTerms, CsgExact::LoopAreaExact(larger.data(), VertexCount, terms));
        }
    }
    WARN("The double sum gave the wrong order in " << doubleWrong << " of " << 2 * Trials << " pairs. Maximum terms: "
                                                   << maxTerms);
    // The old double method must fail on these loops, so the test shows the value of the exact comparison.
    REQUIRE(doubleWrong > 0);
}

TEST_CASE("Orient2 cost", "[.orient2cost][exact]")
{
    std::mt19937_64 random(1011);
    std::vector<CsgGridPoint2> points(4096);
    for (CsgGridPoint2& point : points)
        point = RandomPoint(random);
    const int repeats = 20000;
    int total = 0;
    const auto begin = std::chrono::steady_clock::now();
    for (int r = 0; r < repeats; ++r)
        for (size_t i = 0; i + 2 < points.size(); ++i)
            total += CsgExact::Orient2(points[i], points[i + 1], points[i + 2]);
    const auto end = std::chrono::steady_clock::now();
    const double nanoseconds = std::chrono::duration<double, std::nano>(end - begin).count() / (double(repeats) * 4094.0);
    WARN("Orient2: " << nanoseconds << " ns per call (checksum " << total << ")");
    REQUIRE(total != 12345678);
}

namespace
{

CsgGridPoint GridP(int x, int y, int z)
{
    CsgGridPoint result;
    result.x_ = x;
    result.y_ = y;
    result.z_ = z;
    return result;
}

constexpr int RayMasks[2] = {0b0111, 0b1000};

/// Count the triangles that the ray from p along +x crosses.
int CountRayCrossings(const std::vector<CsgGridPoint>& corners, const std::vector<int>& triangles,
    const CsgGridPoint& p, int second)
{
    int count = 0;
    for (size_t i = 0; i + 2 < triangles.size(); i += 3)
    {
        count += CsgExact::AxisRayCrossesTriangle(
            p, corners[triangles[i]], corners[triangles[i + 1]], corners[triangles[i + 2]], second);
    }
    return count;
}

/// Cube with the corner (0, 0, 0) and the side 10. Its 12 triangles have the diagonals through the corner 0 or 7.
void MakeGridCube(std::vector<CsgGridPoint>& corners, std::vector<int>& triangles)
{
    corners.clear();
    for (int i = 0; i < 8; ++i)
        corners.push_back(GridP((i & 1) * 10, ((i >> 1) & 1) * 10, ((i >> 2) & 1) * 10));
    triangles = {// x = 0
        0, 2, 6, 0, 6, 4,
        // x = 10
        1, 5, 7, 1, 7, 3,
        // y = 0
        0, 4, 5, 0, 5, 1,
        // y = 10
        2, 3, 7, 2, 7, 6,
        // z = 0
        0, 1, 3, 0, 3, 2,
        // z = 10
        4, 6, 7, 4, 7, 5};
}

} // namespace

TEST_CASE("AxisRayCrossesTriangle of a cube", "[exact][axisray]")
{
    std::vector<CsgGridPoint> corners;
    std::vector<int> triangles;
    MakeGridCube(corners, triangles);
    for (const int second : RayMasks)
    {
        // Inner points. The point (5, 5, 5) lies exactly on the projected diagonals of the faces x = 0 and x = 10.
        const CsgGridPoint inner[] = {GridP(5, 5, 5), GridP(5, 3, 7), GridP(1, 1, 1), GridP(9, 9, 9), GridP(5, 5, 3),
            GridP(3, 3, 3)};
        for (const CsgGridPoint& p : inner)
        {
            INFO("mask " << second << " point " << p.x_ << " " << p.y_ << " " << p.z_);
            CHECK(CountRayCrossings(corners, triangles, p, second) == 1);
        }
        // Outer points. The ray crosses the cube twice or not at all.
        CHECK(CountRayCrossings(corners, triangles, GridP(-5, 3, 4), second) == 2);
        CHECK(CountRayCrossings(corners, triangles, GridP(-5, 5, 5), second) == 2);
        CHECK(CountRayCrossings(corners, triangles, GridP(-5, 20, 4), second) == 0);
        CHECK(CountRayCrossings(corners, triangles, GridP(-5, 5, -1), second) == 0);
        CHECK(CountRayCrossings(corners, triangles, GridP(20, 5, 5), second) == 0);
        CHECK(CountRayCrossings(corners, triangles, GridP(20, 3, 4), second) == 0);
        // Points on the boundary of the projection. The point moves by r * (eps^2, eps^3) relative to the cube.
        // It is inside the projected cube if both displaced coordinates stay in the open range (0, 10).
        // The coordinate 0 needs r = 1, the coordinate 10 needs r = -1 and the coordinate 5 always works.
        const int r = second == 0b1000 ? 1 : -1;
        const auto inRange = [r](int c) { return c == 5 || (c == 0 && r > 0) || (c == 10 && r < 0); };
        const int boundary[][2] = {{0, 5}, {10, 5}, {5, 0}, {5, 10}, {0, 0}, {0, 10}, {10, 0}, {10, 10}};
        for (const auto& yz : boundary)
        {
            INFO("mask " << second << " y " << yz[0] << " z " << yz[1]);
            CHECK(CountRayCrossings(corners, triangles, GridP(-5, yz[0], yz[1]), second)
                == (inRange(yz[0]) && inRange(yz[1]) ? 2 : 0));
        }
        // A point on the face x = 0. The cube moves by eps in x for the mask 0b0111 and the point for the mask 0b1000.
        CHECK(CountRayCrossings(corners, triangles, GridP(0, 5, 5), second) == (second == 0b1000 ? 1 : 2));
        CHECK(CountRayCrossings(corners, triangles, GridP(10, 5, 5), second) == (second == 0b1000 ? 0 : 1));
    }
}

TEST_CASE("AxisRayCrossesTriangle with a point on a shared projected edge", "[exact][axisray]")
{
    // Two triangles share the edge (0, 0) - (10, 10) of the (y, z) projection, with a different x on the two sides.
    {
        const std::vector<CsgGridPoint> corners = {
            GridP(5, 0, 0), GridP(5, 10, 10), GridP(5, 10, 0), GridP(6, 0, 0), GridP(4, 10, 10), GridP(5, 0, 10)};
        const std::vector<int> triangles = {0, 1, 2, 3, 4, 5};
        // The projected triangles are on the two sides of the edge. The second has the opposite orientation.
        for (const int second : RayMasks)
            CHECK(CountRayCrossings(corners, triangles, GridP(0, 5, 5), second) == 1);
    }
    // The shared edge is parallel to the u axis.
    {
        const std::vector<CsgGridPoint> corners = {
            GridP(5, 0, 5), GridP(5, 10, 5), GridP(5, 5, 10), GridP(5, 10, 5), GridP(5, 0, 5), GridP(5, 5, 0)};
        const std::vector<int> triangles = {0, 1, 2, 3, 4, 5};
        for (const int second : RayMasks)
            CHECK(CountRayCrossings(corners, triangles, GridP(0, 5, 5), second) == 1);
    }
    // The shared edge is parallel to the v axis, and the triangles have the same orientation as the cube faces.
    {
        const std::vector<CsgGridPoint> corners = {
            GridP(5, 5, 0), GridP(5, 5, 10), GridP(5, 10, 5), GridP(5, 5, 10), GridP(5, 5, 0), GridP(5, 0, 5)};
        const std::vector<int> triangles = {0, 1, 2, 3, 4, 5};
        for (const int second : RayMasks)
            CHECK(CountRayCrossings(corners, triangles, GridP(0, 5, 5), second) == 1);
    }
}

TEST_CASE("AxisRayCrossesTriangle with a point on the projected vertex of a fan", "[exact][axisray]")
{
    // The apex is (5, 5, 5). The six ring points surround the projected apex.
    const int ring[6][2] = {{15, 5}, {10, 15}, {0, 15}, {-5, 5}, {0, -5}, {10, -5}};
    for (const bool reversed : {false, true})
    {
        std::vector<CsgGridPoint> corners = {GridP(5, 5, 5)};
        for (int i = 0; i < 6; ++i)
            corners.push_back(GridP(3 + (i * 3) % 5, ring[i][0], ring[i][1]));
        std::vector<int> triangles;
        for (int i = 0; i < 6; ++i)
        {
            triangles.push_back(0);
            triangles.push_back(reversed ? 1 + (i + 1) % 6 : 1 + i);
            triangles.push_back(reversed ? 1 + i : 1 + (i + 1) % 6);
        }
        for (const int second : RayMasks)
        {
            INFO("mask " << second << " reversed " << reversed);
            CHECK(CountRayCrossings(corners, triangles, GridP(0, 5, 5), second) == 1);
            CHECK(CountRayCrossings(corners, triangles, GridP(-7, 5, 5), second) == 1);
        }
    }
}

TEST_CASE("AxisRayCrossesTriangle with a point in the plane of the triangle", "[exact][axisray]")
{
    // The triangle lies in the plane x = 5 and the point is in the plane, inside the projection.
    const CsgGridPoint t0 = GridP(5, 0, 0), t1 = GridP(5, 10, 0), t2 = GridP(5, 0, 10);
    const CsgGridPoint p = GridP(5, 1, 1);
    // The triangle moves to x = 5 + eps, so the point is behind the plane and the ray crosses it.
    CHECK(CsgExact::AxisRayCrossesTriangle(p, t0, t1, t2, 0b0111));
    CHECK(CsgExact::AxisRayCrossesTriangle(p, t0, t2, t1, 0b0111));
    // The point moves to x = 5 + eps, so the point is ahead of the plane and the ray does not cross it.
    CHECK_FALSE(CsgExact::AxisRayCrossesTriangle(p, t0, t1, t2, 0b1000));
    CHECK_FALSE(CsgExact::AxisRayCrossesTriangle(p, t0, t2, t1, 0b1000));
    // A triangle parallel to the ray never crosses.
    CHECK_FALSE(CsgExact::AxisRayCrossesTriangle(GridP(0, 1, 1), GridP(3, 0, 0), GridP(4, 10, 0), GridP(3, 0, 0), 0b0111));
    CHECK_FALSE(CsgExact::AxisRayCrossesTriangle(GridP(0, 1, 1), GridP(3, 0, 0), GridP(4, 10, 0), GridP(5, 20, 0), 0b1000));
}

TEST_CASE("AxisRayCrossesTriangle parity equals ContainsPoint on a sphere", "[exact][axisray]")
{
    const CsgSolid sphere(MakeSpherePolygons(Vector3(0.1f, -0.2f, 0.05f), 1.0f, 16, 24, 1));
    const CsgInternal::Mesh mesh(sphere, false, 0);
    const std::vector<CsgGridPoint> corners(mesh.grid_.begin(), mesh.grid_.end());
    std::vector<int> triangles(mesh.triangleIds_.begin(), mesh.triangleIds_.end());
    REQUIRE(!triangles.empty());

    std::mt19937_64 random(777);
    std::uniform_int_distribution<int> coordinate(static_cast<int>(-1.5 * CsgGridScale), static_cast<int>(1.5 * CsgGridScale));
    int inside = 0;
    int tested = 0;
    for (int i = 0; i < 2000; ++i)
    {
        CsgGridPoint p = GridP(coordinate(random), coordinate(random), coordinate(random));
        const Vector3 position = p.ToPosition();
        // Skip the points near the faceted surface, where a float ray is not reliable.
        const float distance = (position - Vector3(0.1f, -0.2f, 0.05f)).Length();
        if (std::abs(distance - 1.0f) < 0.06f)
            continue;
        const bool expected = sphere.ContainsPoint(position);
        for (const int second : RayMasks)
        {
            const bool parity = (CountRayCrossings(corners, triangles, p, second) & 1) != 0;
            REQUIRE(parity == expected);
        }
        ++tested;
        inside += expected ? 1 : 0;
    }
    CHECK(tested > 1000);
    CHECK(inside > 100);
}

} // namespace Urho3D
