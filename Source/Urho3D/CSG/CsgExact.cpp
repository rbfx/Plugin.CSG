// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgExact.h"
#include "Urho3D/CSG/CsgExactFilter.h"

#include <cmath>
#include <limits>

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

} // namespace Urho3D
