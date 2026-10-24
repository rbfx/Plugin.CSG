// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgPolygon.h"

#include <Urho3D/Math/MathDefs.h>

#include <EASTL/algorithm.h>

namespace Urho3D
{

namespace
{

void FlipModelVertex(ModelVertex& v)
{
    v.normal_ = -v.normal_;
    v.tangent_ = Vector4(-v.tangent_.x_, -v.tangent_.y_, -v.tangent_.z_, v.tangent_.w_);
    if (v.HasBinormal())
        v.binormal_ = -v.binormal_;
}

} // namespace

CsgPolygon::CsgPolygon() = default;

CsgPolygon::CsgPolygon(ea::vector<ModelVertex>&& verts, unsigned sharedKey)
    : vertices_(ea::move(verts))
    , shared_(sharedKey)
{
    if (vertices_.size() >= 3)
        plane_ = Plane(vertices_[0].GetPosition(), vertices_[1].GetPosition(), vertices_[2].GetPosition());
}

CsgPolygon::CsgPolygon(CsgPolygon&&) = default;
CsgPolygon& CsgPolygon::operator=(CsgPolygon&&) = default;

CsgPolygon::CsgPolygon(const CsgPolygon&) = default;
CsgPolygon& CsgPolygon::operator=(const CsgPolygon&) = default;

void CsgPolygon::Flip()
{
    ea::reverse(vertices_.begin(), vertices_.end());
    for (auto& v : vertices_)
        FlipModelVertex(v);
    plane_.normal_ = -plane_.normal_;
    plane_.d_ = -plane_.d_;
}

bool CsgPolygon::IsConvex() const
{
    if (vertices_.size() < 3)
        return false;

    const unsigned count = vertices_.size();
    for (unsigned i = 0; i < count; ++i)
    {
        const Vector3& p0 = vertices_[i].GetPosition();
        const Vector3& p1 = vertices_[(i + 1) % count].GetPosition();
        const Vector3& p2 = vertices_[(i + 2) % count].GetPosition();

        const Vector3 edge1 = p1 - p0;
        const Vector3 edge2 = p2 - p1;
        const Vector3 cross = edge1.CrossProduct(edge2);

        if (cross.DotProduct(plane_.normal_) < -M_EPSILON)
            return false;
    }
    return true;
}

} // namespace Urho3D
