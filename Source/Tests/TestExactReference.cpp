// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "TestExactReference.h"

namespace CsgTest
{

namespace
{

struct Point64
{
    int64_t x_;
    int64_t y_;
    int64_t z_;
};

Point64 Read(const Urho3D::CsgGridPoint& point)
{
    return {static_cast<int64_t>(point.x_), static_cast<int64_t>(point.y_), static_cast<int64_t>(point.z_)};
}

} // namespace

int ReferenceOrient(const Urho3D::CsgGridPoint& a, const Urho3D::CsgGridPoint& b, const Urho3D::CsgGridPoint& c,
    const Urho3D::CsgGridPoint& d, int second)
{
    const Point64 pa = Read(a);
    const Point64 pb = Read(b);
    const Point64 pc = Read(c);
    const Point64 pd = Read(d);
    const int64_t ux = pb.x_ - pa.x_, uy = pb.y_ - pa.y_, uz = pb.z_ - pa.z_;
    const int64_t vx = pc.x_ - pa.x_, vy = pc.y_ - pa.y_, vz = pc.z_ - pa.z_;
    const int64_t wx = pd.x_ - pa.x_, wy = pd.y_ - pa.y_, wz = pd.z_ - pa.z_;

    const Int128 exact = Mul(Mul64<Int128>(vy, wz) - Mul64<Int128>(vz, wy), ux)
        + Mul(Mul64<Int128>(vz, wx) - Mul64<Int128>(vx, wz), uy)
        + Mul(Mul64<Int128>(vx, wy) - Mul64<Int128>(vy, wx), uz);
    const int exactSign = Sign(exact);
    if (exactSign != 0)
        return exactSign;
    if (second == 0 || second == 0b1111)
        return 0;

    // Displacing the marked points by delta adds delta . G to first order, where G sums their gradients.
    // With delta = (eps, eps^2, eps^3) the first nonzero component of G decides.
    // The higher orders vanish for a translation.
    // Gradients: d/db = v x w, d/dc = w x u, d/dd = u x v. The gradient d/da is minus their sum.
    Int128 gx = Int128(int64_t(0));
    Int128 gy = Int128(int64_t(0));
    Int128 gz = Int128(int64_t(0));
    const int64_t pairs[3][3] = {{vx, vy, vz}, {wx, wy, wz}, {ux, uy, uz}};
    for (int i = 1; i <= 3; ++i)
    {
        const int64_t* p = pairs[i - 1];
        const int64_t* q = pairs[i % 3];
        const Int128 cx = Mul64<Int128>(p[1], q[2]) - Mul64<Int128>(p[2], q[1]);
        const Int128 cy = Mul64<Int128>(p[2], q[0]) - Mul64<Int128>(p[0], q[2]);
        const Int128 cz = Mul64<Int128>(p[0], q[1]) - Mul64<Int128>(p[1], q[0]);
        const int64_t weight = ((second >> i) & 1) - (second & 1);
        gx = gx + Mul(cx, weight);
        gy = gy + Mul(cy, weight);
        gz = gz + Mul(cz, weight);
    }
    if (Sign(gx) != 0)
        return Sign(gx);
    if (Sign(gy) != 0)
        return Sign(gy);
    return Sign(gz);
}

ReferenceNormal ReferenceNormalOf(
    const Urho3D::CsgGridPoint& a, const Urho3D::CsgGridPoint& b, const Urho3D::CsgGridPoint& c)
{
    const Point64 pa = Read(a);
    const Point64 pb = Read(b);
    const Point64 pc = Read(c);
    const int64_t ux = pb.x_ - pa.x_, uy = pb.y_ - pa.y_, uz = pb.z_ - pa.z_;
    const int64_t vx = pc.x_ - pa.x_, vy = pc.y_ - pa.y_, vz = pc.z_ - pa.z_;
    return {Mul64<Int128>(uy, vz) - Mul64<Int128>(uz, vy), Mul64<Int128>(uz, vx) - Mul64<Int128>(ux, vz),
        Mul64<Int128>(ux, vy) - Mul64<Int128>(uy, vx)};
}

Int128 ReferenceVolume(const Urho3D::CsgGridPoint& a, const Urho3D::CsgGridPoint& b, const Urho3D::CsgGridPoint& c,
    const Urho3D::CsgGridPoint& d)
{
    const ReferenceNormal n = ReferenceNormalOf(a, b, c);
    const Point64 pa = Read(a);
    const Point64 pd = Read(d);
    return Mul(n.x_, pd.x_ - pa.x_) + Mul(n.y_, pd.y_ - pa.y_) + Mul(n.z_, pd.z_ - pa.z_);
}

int ReferenceCompareOnEdge(const ReferenceCrossing& x, const ReferenceCrossing& y)
{
    const Int128 dx = x.lowVolume_ - x.highVolume_;
    const Int128 dy = y.lowVolume_ - y.highVolume_;
    // t_x - t_y = (Lx * Dy - Ly * Dx) / (Dx * Dy)
    const int numeratorSign = SignOfProductDifference<Int128>(x.lowVolume_, dy, y.lowVolume_, dx);
    const int denominatorSign = Sign(dx) * Sign(dy);
    if (numeratorSign != 0)
        return numeratorSign * denominatorSign;

    // Tie: compare delta * (Nx / Dx - Ny / Dy), component by component.
    const int shift = x.edgeInSecond_ ? 1 : -1;
    const Int128 components[3][2] = {
        {x.normal_.x_, y.normal_.x_}, {x.normal_.y_, y.normal_.y_}, {x.normal_.z_, y.normal_.z_}};
    for (const auto& component : components)
    {
        const int differenceSign = SignOfProductDifference<Int128>(component[0], dy, component[1], dx);
        if (differenceSign != 0)
            return differenceSign * denominatorSign * shift;
    }
    return 0;
}

} // namespace CsgTest
