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

/// Relative error bound of the floating point filter, with e as the float epsilon.
/// The published error bound is (7 + 56e) * e times the permanent. The value 8e covers it.
constexpr float FilterBound = 8.0f * std::numeric_limits<float>::epsilon();

int SignOfFloat(float value)
{
    return value > 0.0f ? 1 : -1;
}

} // namespace

template <typename Int>
int CsgExact::OrientT(
    const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d, int second)
{
    const int64_t ux = b.x_ - a.x_, uy = b.y_ - a.y_, uz = b.z_ - a.z_;
    const int64_t vx = c.x_ - a.x_, vy = c.y_ - a.y_, vz = c.z_ - a.z_;
    const int64_t wx = d.x_ - a.x_, wy = d.y_ - a.y_, wz = d.z_ - a.z_;

    // Coordinates up to 2^23 make every difference at most 2^24. Such a difference is exact in float.
    // Only the products and the sums round. The filter bound covers these errors.
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

    const Int exact = CsgMul(CsgMul64<Int>(vy, wz) - CsgMul64<Int>(vz, wy), ux)
        + CsgMul(CsgMul64<Int>(vz, wx) - CsgMul64<Int>(vx, wz), uy)
        + CsgMul(CsgMul64<Int>(vx, wy) - CsgMul64<Int>(vy, wx), uz);
    const int exactSign = CsgSign(exact);
    if (exactSign != 0)
        return exactSign;
    if (second == 0 || second == 0b1111)
        return 0;

    // Displacing the marked points by delta adds delta . G to first order, where G sums their gradients.
    // With delta = (eps, eps^2, eps^3) the first nonzero component of G decides.
    // The higher orders vanish for a translation.
    // Gradients: d/db = v x w, d/dc = w x u, d/dd = u x v. The gradient d/da is minus their sum.
    Int gx = Int(0);
    Int gy = Int(0);
    Int gz = Int(0);
    const int64_t pairs[3][3] = {{vx, vy, vz}, {wx, wy, wz}, {ux, uy, uz}};
    for (int i = 1; i <= 3; ++i)
    {
        const int64_t* p = pairs[i - 1];
        const int64_t* q = pairs[i % 3];
        const Int cx = CsgMul64<Int>(p[1], q[2]) - CsgMul64<Int>(p[2], q[1]);
        const Int cy = CsgMul64<Int>(p[2], q[0]) - CsgMul64<Int>(p[0], q[2]);
        const Int cz = CsgMul64<Int>(p[0], q[1]) - CsgMul64<Int>(p[1], q[0]);
        const int64_t weight = ((second >> i) & 1) - (second & 1);
        gx = gx + CsgMul(cx, weight);
        gy = gy + CsgMul(cy, weight);
        gz = gz + CsgMul(cz, weight);
    }
    if (CsgSign(gx) != 0)
        return CsgSign(gx);
    if (CsgSign(gy) != 0)
        return CsgSign(gy);
    return CsgSign(gz);
}

template <typename Int>
CsgNormal<Int> CsgExact::NormalT(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c)
{
    const int64_t ux = b.x_ - a.x_, uy = b.y_ - a.y_, uz = b.z_ - a.z_;
    const int64_t vx = c.x_ - a.x_, vy = c.y_ - a.y_, vz = c.z_ - a.z_;
    return {CsgMul64<Int>(uy, vz) - CsgMul64<Int>(uz, vy), CsgMul64<Int>(uz, vx) - CsgMul64<Int>(ux, vz),
        CsgMul64<Int>(ux, vy) - CsgMul64<Int>(uy, vx)};
}

template <typename Int>
Int CsgExact::VolumeT(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d)
{
    const CsgNormal<Int> n = NormalT<Int>(a, b, c);
    return CsgMul(n.x_, d.x_ - a.x_) + CsgMul(n.y_, d.y_ - a.y_) + CsgMul(n.z_, d.z_ - a.z_);
}

template <typename Int>
bool CsgExact::CrossesT(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1,
    const CsgGridPoint& t2, bool edgeInSecond, bool& e1Above)
{
    const int sidesMask = edgeInSecond ? 0b1000 : 0b0111;
    const int s0 = OrientT<Int>(t0, t1, t2, e0, sidesMask);
    const int s1 = OrientT<Int>(t0, t1, t2, e1, sidesMask);
    e1Above = s1 > 0;
    if (s0 == 0 || s1 == 0 || s0 == s1)
        return false;
    return PassesInsideT<Int>(e0, e1, t0, t1, t2, edgeInSecond);
}

template <typename Int>
bool CsgExact::PassesInsideT(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0,
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

    const int s0 = (decided & 1) != 0 ? SignOfFloat(det[0]) : OrientT<Int>(e0, e1, t0, t1, linesMask);
    if (s0 == 0)
        return false;
    const int s1 = (decided & 2) != 0 ? SignOfFloat(det[1]) : OrientT<Int>(e0, e1, t1, t2, linesMask);
    if (s1 != s0)
        return false;
    const int s2 = (decided & 4) != 0 ? SignOfFloat(det[2]) : OrientT<Int>(e0, e1, t2, t0, linesMask);
    return s2 == s0;
}

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

int CsgExact::Orient(
    const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d, int second)
{
    return OrientT<CsgInt128>(a, b, c, d, second);
}

CsgInt128 CsgExact::Volume(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, const CsgGridPoint& d)
{
    return VolumeT<CsgInt128>(a, b, c, d);
}

CsgNormal<CsgInt128> CsgExact::Normal(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c)
{
    return NormalT<CsgInt128>(a, b, c);
}

bool CsgExact::Crosses(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0, const CsgGridPoint& t1,
    const CsgGridPoint& t2, bool edgeInSecond, bool& e1Above)
{
    return CrossesT<CsgInt128>(e0, e1, t0, t1, t2, edgeInSecond, e1Above);
}

bool CsgExact::PassesInside(const CsgGridPoint& e0, const CsgGridPoint& e1, const CsgGridPoint& t0,
    const CsgGridPoint& t1, const CsgGridPoint& t2, bool edgeInSecond)
{
    return PassesInsideT<CsgInt128>(e0, e1, t0, t1, t2, edgeInSecond);
}

#define CSG_INSTANTIATE_EXACT(INT) \
    template int CsgExact::OrientT<INT>( \
        const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&, int); \
    template INT CsgExact::VolumeT<INT>( \
        const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&); \
    template CsgNormal<INT> CsgExact::NormalT<INT>(const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&); \
    template bool CsgExact::CrossesT<INT>(const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&, \
        const CsgGridPoint&, const CsgGridPoint&, bool, bool&); \
    template bool CsgExact::PassesInsideT<INT>( \
        const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&, const CsgGridPoint&, bool)

CSG_INSTANTIATE_EXACT(CsgInt128Portable);
#if CSG_HAS_NATIVE_INT128
CSG_INSTANTIATE_EXACT(CsgInt128);
#endif

} // namespace Urho3D
