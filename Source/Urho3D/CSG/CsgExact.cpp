// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgExact.h"
#include "Urho3D/CSG/CsgExactFilter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Urho3D
{

namespace
{

/// Relative error bound of the floating point filter, with u as the float unit roundoff (half the float epsilon).
/// The published error bound is (7 + 56u) * u times the permanent, for exact inputs.
/// In float mode the differences are exact in float. In double mode a difference can be above 2^24.
/// Then the conversion to float rounds it once, with a relative error of at most u.
/// The determinant of the rounded inputs then differs by at most about 3u times the permanent.
/// The sum of both errors is about 10u. The value 8e = 16u covers it.
/// The float range holds the permanent: it is below 2^123 in double mode.
constexpr float FilterBound = 8.0f * std::numeric_limits<float>::epsilon();

int SignOfFloat(float value)
{
    return value > 0.0f ? 1 : -1;
}

/// Add a minor multiplied by the weight -1 or 1 to an accumulator with at most 12 terms.
void AddWeighted(CsgReal* accumulator, int& count, const CsgExactMinor& minor, int weight)
{
    CsgReal signedTerms[4];
    for (int i = 0; i < minor.count_; ++i)
        signedTerms[i] = weight < 0 ? -minor.terms_[i] : minor.terms_[i];
    CsgReal sum[12];
    const int sumCount = CsgExpansionSum(count, accumulator, minor.count_, signedTerms, sum);
    for (int i = 0; i < sumCount; ++i)
        accumulator[i] = sum[i];
    count = CsgCompress(sumCount, accumulator);
}

/// Largest distance of the double estimate from an integer for which the exact test runs.
/// The estimate has an error of about 2^-20 at most. A distance above this margin decides the rounding.
constexpr double RoundingMargin = 1.0 / 1024.0;

/// Terms of the odd factor 2 * offset + delta with delta equal to -1 or 1. Return the number of terms.
/// The factor can need more bits than one CsgReal holds, so it has two terms.
int OddFactor(int64_t offset, int delta, CsgReal (&terms)[2])
{
    terms[0] = static_cast<CsgReal>(delta);
    if (offset == 0)
        return 1;
    terms[1] = static_cast<CsgReal>(2 * offset);
    return 2;
}

/// Sign of 2 * span * N - (2 * offset + delta) * D, where span is the difference high - low.
int RoundingSign(CsgReal doubleSpan, int64_t offset, int delta, const CsgExactVolume& n, const CsgExactVolume& d)
{
    CsgReal factor[2];
    const int factorCount = OddFactor(offset, delta, factor);
    return CsgExpansionProductDifferenceSign(1, &doubleSpan, n.count_, n.terms_, factorCount, factor, d.count_, d.terms_);
}

/// Round with the positive denominator d and the estimate t of the fraction.
CsgGridInt RoundWithEstimate(CsgGridInt low, CsgGridInt high, double t, const CsgExactVolume& n, const CsgExactVolume& d)
{
    const int64_t span = static_cast<int64_t>(high) - low;
    if (span == 0)
        return low;
    const double estimate = static_cast<double>(low) + static_cast<double>(span) * t + 0.5;
    double integer = std::floor(estimate);
    const bool decided = std::abs(estimate - integer - 0.5) < 0.5 - RoundingMargin;
    int64_t g = static_cast<int64_t>(integer);
    const int64_t lowest = low < high ? low : high;
    const int64_t highest = low < high ? high : low;
    g = g < lowest ? lowest : (g > highest ? highest : g);
    if (decided)
        return static_cast<CsgGridInt>(g);

    // Correct the guess with exact signs. Both tests together are x >= g - 1/2 and x < g + 1/2.
    const CsgReal doubleSpan = static_cast<CsgReal>(2 * span);
    for (int step = 0; step < 4; ++step)
    {
        if (RoundingSign(doubleSpan, g - low, -1, n, d) < 0)
        {
            --g;
            continue;
        }
        if (RoundingSign(doubleSpan, g - low, 1, n, d) >= 0)
        {
            ++g;
            continue;
        }
        return static_cast<CsgGridInt>(g);
    }
    URHO3D_ASSERT(false, "The rounding correction must end within four steps");
    return static_cast<CsgGridInt>(g);
}

} // namespace

int CsgInternal::Filter(const CsgFloat4& ux, const CsgFloat4& uy, const CsgFloat4& uz, const CsgFloat4& vx,
    const CsgFloat4& vy, const CsgFloat4& vz, const CsgFloat4& wx, const CsgFloat4& wy, const CsgFloat4& wz,
    CsgFloat4& det)
{
    const CsgFloat4 m1 = vy * wz - vz * wy;
    const CsgFloat4 m2 = vz * wx - vx * wz;
    const CsgFloat4 m3 = vx * wy - vy * wx;
    det = ux * m1 + uy * m2 + uz * m3;
    const CsgFloat4 permanent = CsgFloat4::Abs(ux) * (CsgFloat4::Abs(vy * wz) + CsgFloat4::Abs(vz * wy))
        + CsgFloat4::Abs(uy) * (CsgFloat4::Abs(vz * wx) + CsgFloat4::Abs(vx * wz))
        + CsgFloat4::Abs(uz) * (CsgFloat4::Abs(vx * wy) + CsgFloat4::Abs(vy * wx));
    return CsgFloat4::GreaterThanMask(CsgFloat4::Abs(det), permanent * CsgFloat4::Splat(FilterBound)) & 0b111;
}

CsgExactMinor CsgExact::MinorExact(CsgReal a, CsgReal b, CsgReal c, CsgReal d)
{
    // Two_Two_Diff after Shewchuk: the product a * b minus the product c * d. The result has four terms.
    CsgReal abLow;
    const CsgReal abHigh = CsgTwoProduct(a, b, abLow);
    CsgReal cdLow;
    const CsgReal cdHigh = CsgTwoProduct(c, d, cdLow);

    CsgExactMinor result;
    CsgReal t0;
    const CsgReal i0 = CsgTwoDiff(abLow, cdLow, result.terms_[0]);
    const CsgReal j = CsgTwoSum(abHigh, i0, t0);
    const CsgReal i1 = CsgTwoDiff(t0, cdHigh, result.terms_[1]);
    result.terms_[3] = CsgTwoSum(j, i1, result.terms_[2]);
    result.count_ = CsgCompress(4, result.terms_);
    return result;
}

int CsgExact::Orient(
    const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d, int second)
{
    const int64_t ux = static_cast<int64_t>(b.x_) - a.x_, uy = static_cast<int64_t>(b.y_) - a.y_,
                  uz = static_cast<int64_t>(b.z_) - a.z_;
    const int64_t vx = static_cast<int64_t>(c.x_) - a.x_, vy = static_cast<int64_t>(c.y_) - a.y_,
                  vz = static_cast<int64_t>(c.z_) - a.z_;
    const int64_t wx = static_cast<int64_t>(d.x_) - a.x_, wy = static_cast<int64_t>(d.y_) - a.y_,
                  wz = static_cast<int64_t>(d.z_) - a.z_;

    // In float mode every difference is below 2^24 and exact in float. In double mode the conversion can round.
    // The filter bound covers this rounding and the rounding of the products and the sums.
    const float fux = static_cast<float>(ux), fuy = static_cast<float>(uy), fuz = static_cast<float>(uz);
    const float fvx = static_cast<float>(vx), fvy = static_cast<float>(vy), fvz = static_cast<float>(vz);
    const float fwx = static_cast<float>(wx), fwy = static_cast<float>(wy), fwz = static_cast<float>(wz);
    const float m1 = fvy * fwz - fvz * fwy;
    const float m2 = fvz * fwx - fvx * fwz;
    const float m3 = fvx * fwy - fvy * fwx;
    const float det = fux * m1 + fuy * m2 + fuz * m3;
    const float permanent = std::abs(fux) * (std::abs(fvy * fwz) + std::abs(fvz * fwy))
        + std::abs(fuy) * (std::abs(fvz * fwx) + std::abs(fvx * fwz))
        + std::abs(fuz) * (std::abs(fvx * fwy) + std::abs(fvy * fwx));
    if (std::abs(det) > permanent * FilterBound)
        return SignOfFloat(det);

    const CsgReal rux = static_cast<CsgReal>(ux), ruy = static_cast<CsgReal>(uy), ruz = static_cast<CsgReal>(uz);
    const CsgReal rvx = static_cast<CsgReal>(vx), rvy = static_cast<CsgReal>(vy), rvz = static_cast<CsgReal>(vz);
    const CsgReal rwx = static_cast<CsgReal>(wx), rwy = static_cast<CsgReal>(wy), rwz = static_cast<CsgReal>(wz);

    // Each minor has at most 4 terms. Each scaled minor has at most 8 terms. The sums have 16 and 24 terms.
    const CsgExactMinor minors[3] = {
        MinorExact(rvy, rwz, rvz, rwy), MinorExact(rvz, rwx, rvx, rwz), MinorExact(rvx, rwy, rvy, rwx)};
    const CsgReal scales[3] = {rux, ruy, ruz};
    CsgReal scaled[3][8];
    int scaledCount[3];
    for (int i = 0; i < 3; ++i)
    {
        scaledCount[i] = CsgScaleExpansion(minors[i].count_, minors[i].terms_, scales[i], scaled[i]);
        scaledCount[i] = CsgCompress(scaledCount[i], scaled[i]);
    }
    CsgReal partial[16];
    int partialCount = CsgExpansionSum(scaledCount[0], scaled[0], scaledCount[1], scaled[1], partial);
    partialCount = CsgCompress(partialCount, partial);
    CsgReal total[24];
    int totalCount = CsgExpansionSum(partialCount, partial, scaledCount[2], scaled[2], total);
    totalCount = CsgCompress(totalCount, total);
    const int exactSign = CsgExpansionSign(totalCount, total);
    if (exactSign != 0)
        return exactSign;
    if (second == 0 || second == 0b1111)
        return 0;

    // Displacing the marked points by delta adds delta . G to first order, where G sums their gradients.
    // With delta = (eps, eps^2, eps^3) the first nonzero component of G decides.
    // The higher orders vanish for a translation.
    // Gradients: d/db = v x w, d/dc = w x u, d/dd = u x v. The gradient d/da is minus their sum.
    CsgReal gradient[3][12];
    int gradientCount[3] = {0, 0, 0};
    const CsgReal pairs[3][3] = {{rvx, rvy, rvz}, {rwx, rwy, rwz}, {rux, ruy, ruz}};
    for (int i = 1; i <= 3; ++i)
    {
        const CsgReal* p = pairs[i - 1];
        const CsgReal* q = pairs[i % 3];
        const int weight = ((second >> i) & 1) - (second & 1);
        if (weight == 0)
            continue;
        const CsgExactMinor cross[3] = {
            MinorExact(p[1], q[2], p[2], q[1]), MinorExact(p[2], q[0], p[0], q[2]), MinorExact(p[0], q[1], p[1], q[0])};
        for (int k = 0; k < 3; ++k)
            AddWeighted(gradient[k], gradientCount[k], cross[k], weight);
    }
    for (int k = 0; k < 3; ++k)
    {
        const int sign = CsgExpansionSign(gradientCount[k], gradient[k]);
        if (sign != 0)
            return sign;
    }
    return 0;
}

bool CsgExact::AxisRayCrossesTriangle(const CsgGridPoint& p, const CsgGridPoint& t0, const CsgGridPoint& t1,
    const CsgGridPoint& t2, int second)
{
    URHO3D_ASSERT(second == 0b0111 || second == 0b1000, "The mask must move the triangle or the point");

    // The ray is p + t * (1, 0, 0) with t > 0. Project along x: u = y, v = z.
    const CsgGridPoint2 q = Project(p, 0, false);
    const CsgGridPoint2 corners[3] = {Project(t0, 0, false), Project(t1, 0, false), Project(t2, 0, false)};

    // The sign of the projected area is the sign of the x component of the normal.
    const int s = Orient2(corners[0], corners[1], corners[2]);
    if (s == 0)
        return false;

    // Only the relative displacement of the point and the triangle matters. Orient adds delta to each marked point.
    // Mask 0b1000: the point moves by +delta. Mask 0b0111: the triangle moves, so the point moves by -delta.
    // Let r be the sign of the displacement of the point relative to the triangle: r = 1 for 0b1000, r = -1 for 0b0111.
    // The projected displacement of the point is r * (eps^2, eps^3). The x part r * eps only moves the plane test.
    const int r = second == 0b1000 ? 1 : -1;

    // The point is inside the projected triangle if it is on the inner side of all three edges.
    // The inner side has the sign s. A tie (o == 0) is resolved by the displacement:
    // Orient2(e0, e1, q + r * (eps^2, eps^3)) = Orient2(e0, e1, q) + r * (d.u * eps^3 - d.v * eps^2).
    // Here d = e1 - e0. The term eps^2 is the larger one. If d.v is zero, the term eps^3 decides.
    for (int i = 0; i < 3; ++i)
    {
        const CsgGridPoint2& e0 = corners[i];
        const CsgGridPoint2& e1 = corners[(i + 1) % 3];
        int o = Orient2(e0, e1, q);
        if (o == 0)
        {
            const CsgGridPoint2 d = Difference(e1, e0);
            if (d.v_ != 0)
                o = d.v_ > 0 ? -r : r;
            else
                o = d.u_ > 0 ? r : -r;
        }
        if (o * s < 0)
            return false;
    }

    // The sign of o3 is the sign of N . (p - t0) with N = (t1 - t0) x (t2 - t0). N.x has the sign s.
    // The plane meets the ray at t = -N . (p - t0) / N.x. The hit is ahead (t > 0) if the signs differ.
    const int o3 = Orient(t0, t1, t2, p, second);
    URHO3D_ASSERT(o3 != 0, "The displacement must resolve the plane test for a triangle that is not parallel to x");
    return o3 * s < 0;
}

bool CsgExact::AxisRayCrossesTriangleCentroid(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c,
    const CsgGridPoint& t0, const CsgGridPoint& t1, const CsgGridPoint& t2, int second)
{
    URHO3D_ASSERT(second == 0b0111 || second == 0b1000, "The mask must move the triangle or the point");

    // The sample is q = (a + b + c) / 3. Multiplying by 3 does not change a sign, so q needs no scaling:
    // the sign of d x (q - e0) is the sign of d x ((a - e0) + (b - e0) + (c - e0)).
    // Each cross is below 2^(2 * CsgGridBits + 3). The sum of three is below 2^(2 * CsgGridBits + 5).
    static_assert(2 * CsgGridBits + 5 <= 63, "The sum of three planar crosses must fit in 64 bits.");
    const CsgGridPoint2 samples[3] = {Project(a, 0, false), Project(b, 0, false), Project(c, 0, false)};
    const CsgGridPoint2 corners[3] = {Project(t0, 0, false), Project(t1, 0, false), Project(t2, 0, false)};

    const int s = Orient2(corners[0], corners[1], corners[2]);
    if (s == 0)
        return false;

    // The displacement and the tie rule are the same as in AxisRayCrossesTriangle. All three points move together.
    const int r = second == 0b1000 ? 1 : -1;
    for (int i = 0; i < 3; ++i)
    {
        const CsgGridPoint2& e0 = corners[i];
        const CsgGridPoint2& e1 = corners[(i + 1) % 3];
        const CsgGridPoint2 d = Difference(e1, e0);
        int64_t sum = 0;
        for (const CsgGridPoint2& sample : samples)
        {
            const CsgGridPoint2 w = Difference(sample, e0);
            sum += static_cast<int64_t>(d.u_) * w.v_ - static_cast<int64_t>(d.v_) * w.u_;
        }
        int o = sum > 0 ? 1 : (sum < 0 ? -1 : 0);
        if (o == 0)
        {
            if (d.v_ != 0)
                o = d.v_ > 0 ? -r : r;
            else
                o = d.u_ > 0 ? r : -r;
        }
        if (o * s < 0)
            return false;
    }

    // The sign of N . (q - t0) is the sign of the sum of N . (a - t0), N . (b - t0) and N . (c - t0).
    // Each term has up to 24 terms. The sums have up to 48 and 72 terms.
    const CsgNormal n = Normal(t0, t1, t2);
    const CsgGridPoint* points[3] = {&a, &b, &c};
    CsgExactVolume dots[3];
    for (int i = 0; i < 3; ++i)
        dots[i] = Dot(n, t0, *points[i]);
    CsgReal partial[48];
    int partialCount = CsgExpansionSum(dots[0].count_, dots[0].terms_, dots[1].count_, dots[1].terms_, partial);
    partialCount = CsgCompress(partialCount, partial);
    CsgReal total[72];
    int totalCount = CsgExpansionSum(partialCount, partial, dots[2].count_, dots[2].terms_, total);
    totalCount = CsgCompress(totalCount, total);
    int o3 = CsgExpansionSign(totalCount, total);

    // On the plane the displacement decides. The first nonzero component of N is N.x, and its sign is s.
    // The point moves by r * delta relative to the triangle, so the sign is r * s.
    if (o3 == 0)
        o3 = r * s;
    return o3 * s < 0;
}

CsgNormal CsgExact::Normal(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c)
{
    const CsgReal ux = static_cast<CsgReal>(static_cast<int64_t>(b.x_) - a.x_);
    const CsgReal uy = static_cast<CsgReal>(static_cast<int64_t>(b.y_) - a.y_);
    const CsgReal uz = static_cast<CsgReal>(static_cast<int64_t>(b.z_) - a.z_);
    const CsgReal vx = static_cast<CsgReal>(static_cast<int64_t>(c.x_) - a.x_);
    const CsgReal vy = static_cast<CsgReal>(static_cast<int64_t>(c.y_) - a.y_);
    const CsgReal vz = static_cast<CsgReal>(static_cast<int64_t>(c.z_) - a.z_);
    return {MinorExact(uy, vz, uz, vy), MinorExact(uz, vx, ux, vz), MinorExact(ux, vy, uy, vx)};
}

CsgExactVolume CsgExact::Dot(const CsgNormal& n, const CsgGridPoint& from, const CsgGridPoint& to)
{
    const CsgReal scales[3] = {static_cast<CsgReal>(static_cast<int64_t>(to.x_) - from.x_),
        static_cast<CsgReal>(static_cast<int64_t>(to.y_) - from.y_),
        static_cast<CsgReal>(static_cast<int64_t>(to.z_) - from.z_)};
    const CsgExactMinor* components[3] = {&n.x_, &n.y_, &n.z_};

    // Each minor has at most 4 terms. Each scaled minor has at most 8 terms. The sums have 16 and 24 terms.
    CsgReal scaled[3][8];
    int scaledCount[3];
    for (int i = 0; i < 3; ++i)
    {
        scaledCount[i] = CsgScaleExpansion(components[i]->count_, components[i]->terms_, scales[i], scaled[i]);
        scaledCount[i] = CsgCompress(scaledCount[i], scaled[i]);
    }
    CsgReal partial[16];
    int partialCount = CsgExpansionSum(scaledCount[0], scaled[0], scaledCount[1], scaled[1], partial);
    partialCount = CsgCompress(partialCount, partial);
    CsgExactVolume result;
    result.count_ = CsgExpansionSum(partialCount, partial, scaledCount[2], scaled[2], result.terms_);
    result.count_ = CsgCompress(result.count_, result.terms_);
    return result;
}

CsgExactVolume CsgExact::Volume(
    const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d)
{
    return Dot(Normal(a, b, c), a, d);
}

bool CsgExact::Crosses(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1,
    const CsgGridPoint& t2, bool edgeInSecond, bool& e1Above)
{
    const int sidesMask = edgeInSecond ? 0b1000 : 0b0111;
    const int s0 = Orient(t0, t1, t2, e0, sidesMask);
    const int s1 = Orient(t0, t1, t2, e1, sidesMask);
    e1Above = s1 > 0;
    if (s0 == 0 || s1 == 0 || s0 == s1)
        return false;
    return PassesInside(e0, e1, t0, t1, t2, edgeInSecond);
}

bool CsgExact::PassesInside(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0,
    const CsgGridPoint& t1, const CsgGridPoint& t2, bool edgeInSecond)
{
    const int linesMask = edgeInSecond ? 0b0011 : 0b1100;

    // Lane k holds Orient(e0, e1, t_k, t_k+1). Undecided lanes take the exact path when needed.
    const auto diff = [](int64_t t, int64_t e) { return static_cast<float>(t - e); };
    const CsgFloat4 vx = CsgFloat4::Create(diff(t0.x_, e0.x_), diff(t1.x_, e0.x_), diff(t2.x_, e0.x_), 0.0f);
    const CsgFloat4 vy = CsgFloat4::Create(diff(t0.y_, e0.y_), diff(t1.y_, e0.y_), diff(t2.y_, e0.y_), 0.0f);
    const CsgFloat4 vz = CsgFloat4::Create(diff(t0.z_, e0.z_), diff(t1.z_, e0.z_), diff(t2.z_, e0.z_), 0.0f);
    const CsgFloat4 wx = CsgFloat4::Create(diff(t1.x_, e0.x_), diff(t2.x_, e0.x_), diff(t0.x_, e0.x_), 0.0f);
    const CsgFloat4 wy = CsgFloat4::Create(diff(t1.y_, e0.y_), diff(t2.y_, e0.y_), diff(t0.y_, e0.y_), 0.0f);
    const CsgFloat4 wz = CsgFloat4::Create(diff(t1.z_, e0.z_), diff(t2.z_, e0.z_), diff(t0.z_, e0.z_), 0.0f);
    CsgFloat4 detVector;
    const int decided = CsgInternal::Filter(CsgFloat4::Splat(diff(e1.x_, e0.x_)), CsgFloat4::Splat(diff(e1.y_, e0.y_)),
        CsgFloat4::Splat(diff(e1.z_, e0.z_)), vx, vy, vz, wx, wy, wz, detVector);
    float det[4];
    detVector.Store(det);

    const int s0 = (decided & 1) != 0 ? SignOfFloat(det[0]) : Orient(e0, e1, t0, t1, linesMask);
    if (s0 == 0)
        return false;
    const int s1 = (decided & 2) != 0 ? SignOfFloat(det[1]) : Orient(e0, e1, t1, t2, linesMask);
    if (s1 != s0)
        return false;
    const int s2 = (decided & 4) != 0 ? SignOfFloat(det[2]) : Orient(e0, e1, t2, t0, linesMask);
    return s2 == s0;
}

CsgExactVolume CsgExact::Negate(const CsgExactVolume& v)
{
    CsgExactVolume result;
    result.count_ = v.count_;
    for (int i = 0; i < v.count_; ++i)
        result.terms_[i] = -v.terms_[i];
    return result;
}

CsgGridInt CsgExact::RoundOnEdge(
    CsgGridInt low, CsgGridInt high, const CsgExactVolume& numerator, const CsgExactVolume& denominator)
{
    const int denominatorSign = denominator.Sign();
    URHO3D_ASSERT(denominatorSign != 0);
    if (denominatorSign < 0)
    {
        const CsgExactVolume n = Negate(numerator);
        const CsgExactVolume d = Negate(denominator);
        return RoundWithEstimate(low, high, n.Approximate() / d.Approximate(), n, d);
    }
    return RoundWithEstimate(low, high, numerator.Approximate() / denominator.Approximate(), numerator, denominator);
}

CsgGridPoint CsgExact::RoundCrossing(const CsgGridPoint& low, const CsgGridPoint& high, const CsgExactVolume& lowVolume,
    const CsgExactVolume& difference)
{
    const int differenceSign = difference.Sign();
    URHO3D_ASSERT(differenceSign != 0);
    const bool flip = differenceSign < 0;
    const CsgExactVolume n = flip ? Negate(lowVolume) : lowVolume;
    const CsgExactVolume d = flip ? Negate(difference) : difference;
    const double t = n.Approximate() / d.Approximate();
    CsgGridPoint result;
    result.x_ = RoundWithEstimate(low.x_, high.x_, t, n, d);
    result.y_ = RoundWithEstimate(low.y_, high.y_, t, n, d);
    result.z_ = RoundWithEstimate(low.z_, high.z_, t, n, d);
    return result;
}

namespace
{

/// Sign of x - g for x = low + (high - low) * n / d with d > 0, in exact arithmetic.
int SignAgainstGrid(CsgGridInt low, CsgGridInt high, CsgGridInt g, const CsgExactVolume& n, const CsgExactVolume& d)
{
    const int64_t span = static_cast<int64_t>(high) - low;
    const int64_t offset = static_cast<int64_t>(g) - low;
    if (span == 0)
        return offset == 0 ? 0 : (offset > 0 ? -1 : 1);
    const int numeratorSign = n.Sign();
    if (offset == 0)
        return (span > 0 ? 1 : -1) * numeratorSign;
    if (numeratorSign == 0)
        return offset > 0 ? -1 : 1;
    // d * (x - g) = span * n - offset * d
    const CsgReal spanReal = static_cast<CsgReal>(span);
    const CsgReal offsetReal = static_cast<CsgReal>(offset);
    return CsgExpansionProductDifferenceSign(1, &spanReal, n.count_, n.terms_, 1, &offsetReal, d.count_, d.terms_);
}

} // namespace

void CsgExact::CrossingCell(const CsgGridPoint& low, const CsgGridPoint& high, const CsgExactVolume& lowVolume,
    const CsgExactVolume& difference, const CsgGridPoint& rounded, CsgGridPoint& lower, CsgGridPoint& upper)
{
    const int differenceSign = difference.Sign();
    URHO3D_ASSERT(differenceSign != 0);
    const bool flip = differenceSign < 0;
    const CsgExactVolume n = flip ? Negate(lowVolume) : lowVolume;
    const CsgExactVolume d = flip ? Negate(difference) : difference;
    lower = rounded;
    upper = rounded;
    // The rounding is floor(x + 1/2), so x is in [rounded - 1/2, rounded + 1/2).
    const auto cell = [&](CsgGridInt CsgGridPoint::*axis)
    {
        const int sign = SignAgainstGrid(low.*axis, high.*axis, rounded.*axis, n, d);
        if (sign < 0)
            --(lower.*axis);
        else if (sign > 0)
            ++(upper.*axis);
    };
    cell(&CsgGridPoint::x_);
    cell(&CsgGridPoint::y_);
    cell(&CsgGridPoint::z_);
}

namespace
{

/// Sign of a compressed expansion of a difference or a sum.
int ExpansionSign(const CsgExactMinor& value)
{
    return CsgExpansionSign(value.count_, value.terms_);
}

CsgReal ToReal(CsgGridInt value)
{
    // A difference has an absolute value below 2^(CsgGridBits + 1). CsgReal holds it exactly.
    return static_cast<CsgReal>(value);
}

} // namespace

int CsgExact::Cross2Exact(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
{
    return ExpansionSign(MinorExact(ToReal(d0.u_), ToReal(d1.v_), ToReal(d0.v_), ToReal(d1.u_)));
}

int CsgExact::Dot2Exact(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
{
    // a * c + b * d = a * c - (-b) * d. The negation is exact.
    return ExpansionSign(MinorExact(ToReal(d0.u_), ToReal(d1.u_), -ToReal(d0.v_), ToReal(d1.v_)));
}

int CsgExact::SquaredDistanceCompareExact(const CsgGridPoint2& d0, const CsgGridPoint2& d1)
{
    // (u0^2 - u1^2) + (v0^2 - v1^2): two minors and one exact sum.
    const CsgExactMinor first = MinorExact(ToReal(d0.u_), ToReal(d0.u_), ToReal(d1.u_), ToReal(d1.u_));
    const CsgExactMinor second = MinorExact(ToReal(d0.v_), ToReal(d0.v_), ToReal(d1.v_), ToReal(d1.v_));
    CsgReal sum[8];
    const int count = CsgExpansionSum(first.count_, first.terms_, second.count_, second.terms_, sum);
    return CsgExpansionSign(count, sum);
}

int CsgExact::DirectionCompare(const CsgGridPoint2& p, const CsgGridPoint2& reference, const CsgGridPoint2& q0,
    const CsgGridPoint2& q1)
{
    const CsgGridPoint2 r = Difference(reference, p);
    const CsgGridPoint2 d0 = Difference(q0, p);
    const CsgGridPoint2 d1 = Difference(q1, p);
    URHO3D_ASSERT(r.u_ != 0 || r.v_ != 0);

    // Rank: 0 for a zero direction, 1 for the clockwise angles [0, pi), 2 for [pi, 2 pi).
    const auto rank = [&](const CsgGridPoint2& d)
    {
        if (d.u_ == 0 && d.v_ == 0)
            return 0;
        const int cross = Cross2(r, d);
        if (cross != 0)
            return cross < 0 ? 1 : 2;
        return Dot2(r, d) > 0 ? 1 : 2;
    };
    const int rank0 = rank(d0);
    const int rank1 = rank(d1);
    if (rank0 != rank1)
        return rank0 < rank1 ? -1 : 1;
    if (rank0 == 0)
        return 0;
    return Cross2(d0, d1);
}

bool CsgExact::SegmentsCross(
    const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c, const CsgGridPoint2& d)
{
    const int o1 = Orient2(a, b, c);
    const int o2 = Orient2(a, b, d);
    if (o1 * o2 > 0)
        return false;
    const int o3 = Orient2(c, d, a);
    const int o4 = Orient2(c, d, b);
    if (o3 * o4 > 0)
        return false;
    if (o1 != 0 || o2 != 0 || o3 != 0 || o4 != 0)
        return true;

    // Collinear segments, or points: the segments meet if the bounding intervals overlap on both axes.
    return std::max(std::min(a.u_, b.u_), std::min(c.u_, d.u_)) <= std::min(std::max(a.u_, b.u_), std::max(c.u_, d.u_))
        && std::max(std::min(a.v_, b.v_), std::min(c.v_, d.v_)) <= std::min(std::max(a.v_, b.v_), std::max(c.v_, d.v_));
}

int CsgExact::LoopAreaSign(const CsgGridPoint2* points, int count)
{
    if (count < 3)
        return 0;
    const int terms = count - 2;
    if (terms <= CsgLoopFilterMaxEdges)
    {
        double sum = 0.0;
        double magnitude = 0.0;
        for (int i = 1; i < count - 1; ++i)
        {
            const CsgGridPoint2 d0 = Difference(points[i], points[0]);
            const CsgGridPoint2 d1 = Difference(points[i + 1], points[0]);
            const double term = static_cast<double>(d0.u_) * static_cast<double>(d1.v_)
                - static_cast<double>(d0.v_) * static_cast<double>(d1.u_);
            sum += term;
            magnitude += std::fabs(term);
        }
        // Exact terms and a sum of magnitudes below 2^53: every partial sum is an exact integer.
        if (CsgPlanarExactInDouble && magnitude < 9007199254740992.0)
            return sum > 0.0 ? 1 : (sum < 0.0 ? -1 : 0);

        const double n = static_cast<double>(terms);
        const double bound = n * CsgLoopTermError + n * (n - 1.0) * CsgLoopSumUnit;
        if (sum > bound)
            return 1;
        if (sum < -bound)
            return -1;
    }
    return LoopAreaSignExact(points, count);
}

int CsgExact::LoopAreaSignExact(const CsgGridPoint2* points, int count)
{
    CsgReal area[CsgLoopAreaMaxTerms];
    const int areaCount = LoopAreaExact(points, count, area);
    if (areaCount < 0)
    {
        // The loop is too long for the expansion. Use the sign of the double sum.
        double sum = 0.0;
        for (int i = 1; i < count - 1; ++i)
        {
            const CsgGridPoint2 d0 = Difference(points[i], points[0]);
            const CsgGridPoint2 d1 = Difference(points[i + 1], points[0]);
            sum += static_cast<double>(d0.u_) * static_cast<double>(d1.v_)
                - static_cast<double>(d0.v_) * static_cast<double>(d1.u_);
        }
        return sum > 0.0 ? 1 : (sum < 0.0 ? -1 : 0);
    }
    return CsgExpansionSign(areaCount, area);
}

int CsgExact::LoopAreaExact(const CsgGridPoint2* points, int count, CsgReal* out)
{
    if (count < 3)
        return 0;
    if (count > CsgLoopAreaMaxPoints)
        return -1;
    constexpr int Capacity = CsgLoopAreaMaxTerms;
    CsgReal bufferA[Capacity + 4];
    CsgReal bufferB[Capacity + 4];
    CsgReal* current = bufferA;
    CsgReal* next = bufferB;
    int currentCount = 0;
    for (int i = 1; i < count - 1; ++i)
    {
        const CsgGridPoint2 d0 = Difference(points[i], points[0]);
        const CsgGridPoint2 d1 = Difference(points[i + 1], points[0]);
        const CsgExactMinor term = MinorExact(ToReal(d0.u_), ToReal(d1.v_), ToReal(d0.v_), ToReal(d1.u_));
        int nextCount = CsgExpansionSum(currentCount, current, term.count_, term.terms_, next);
        nextCount = CsgCompress(nextCount, next);
        URHO3D_ASSERT(nextCount <= Capacity);
        // The bound on the terms makes this impossible. The check keeps the buffers safe in a release build.
        if (nextCount > Capacity)
            return -1;
        std::swap(current, next);
        currentCount = nextCount;
    }
    for (int i = 0; i < currentCount; ++i)
        out[i] = current[i];
    return currentCount;
}

int CsgExact::CompareLoopAreaMagnitude(int countA, const CsgReal* a, int countB, const CsgReal* b)
{
    // An input over the limit is not a valid area. The compare then gives equal and does not overflow the buffers.
    if (countA < 0 || countB < 0 || countA > CsgLoopAreaMaxTerms || countB > CsgLoopAreaMaxTerms)
        return 0;
    const int signA = CsgExpansionSign(countA, a);
    const int signB = CsgExpansionSign(countB, b);
    if (signA == 0 || signB == 0)
        return (signA != 0) - (signB != 0);

    // |A| - |B| = sA * A - sB * B. A negation is exact and keeps the order of the terms.
    // The exact sum has no limit on the size of the inputs, so it needs no bit or term limit.
    CsgReal positiveA[CsgLoopAreaMaxTerms];
    CsgReal negativeB[CsgLoopAreaMaxTerms];
    for (int i = 0; i < countA; ++i)
        positiveA[i] = signA < 0 ? -a[i] : a[i];
    for (int i = 0; i < countB; ++i)
        negativeB[i] = signB < 0 ? b[i] : -b[i];
    CsgReal difference[2 * CsgLoopAreaMaxTerms];
    const int count = CsgExpansionSum(countA, positiveA, countB, negativeB, difference);
    return CsgExpansionSign(count, difference);
}

} // namespace Urho3D
