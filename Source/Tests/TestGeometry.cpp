// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "TestGeometry.h"

#include "Urho3D/CSG/CsgCommon.h"

#include <EASTL/hash_map.h>

#include <cmath>

namespace Urho3D
{

namespace
{

CsgPolygon MakePolygon(ea::span<const Vector3> positions, unsigned shared)
{
    ea::vector<ModelVertex> vertices;
    vertices.reserve(positions.size());
    for (const Vector3& position : positions)
    {
        ModelVertex vertex;
        vertex.SetPosition(position);
        vertices.push_back(vertex);
    }
    return CsgPolygon(ea::move(vertices), shared);
}

float& AxisComponent(Vector3& v, int index)
{
    return index == 0 ? v.x_ : (index == 1 ? v.y_ : v.z_);
}

float AxisComponent(const Vector3& v, int index)
{
    return index == 0 ? v.x_ : (index == 1 ? v.y_ : v.z_);
}

struct IntVector3Hash
{
    size_t operator()(const IntVector3& v) const noexcept
    {
        return CsgPackedKeyHash()(static_cast<uint64_t>(static_cast<uint32_t>(v.x_)) * 73856093u
            ^ static_cast<uint64_t>(static_cast<uint32_t>(v.y_)) * 19349663u
            ^ static_cast<uint64_t>(static_cast<uint32_t>(v.z_)) * 83492791u);
    }
};

} // namespace

ea::vector<CsgPolygon> MakeBoxPolygons(
    const Vector3& center, const Vector3& halfSize, unsigned shared, const Matrix3x4& transform)
{
    // Each face has a normal axis and two tangent axes. The order (u, v) gives u x v = normal.
    static const int faces[6][3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}, {0, 2, 1}, {1, 0, 2}, {2, 1, 0}};

    ea::vector<CsgPolygon> result;
    for (unsigned i = 0; i < 6; ++i)
    {
        const int n = faces[i][0];
        const int u = faces[i][1];
        const int v = faces[i][2];
        const float sign = i < 3 ? 1.0f : -1.0f;
        // Faces 3 to 5 use the negative side of the axis. Their (u, v) order keeps the normal outward.
        Vector3 nAxis, uAxis, vAxis;
        AxisComponent(nAxis, n) = sign * AxisComponent(halfSize, n);
        AxisComponent(uAxis, u) = AxisComponent(halfSize, u);
        AxisComponent(vAxis, v) = AxisComponent(halfSize, v);

        const Vector3 corners[4] = {
            center + nAxis - uAxis - vAxis,
            center + nAxis + uAxis - vAxis,
            center + nAxis + uAxis + vAxis,
            center + nAxis - uAxis + vAxis,
        };
        Vector3 positions[4];
        for (unsigned j = 0; j < 4; ++j)
            positions[j] = transform * corners[j];
        result.push_back(MakePolygon(positions, shared));
    }
    return result;
}

ea::vector<CsgPolygon> MakeSpherePolygons(
    const Vector3& center, float radius, unsigned rings, unsigned segments, unsigned shared)
{
    const auto point = [&](unsigned ring, unsigned segment)
    {
        const float theta = M_PI * static_cast<float>(ring) / static_cast<float>(rings);
        const float phi = 2.0f * M_PI * static_cast<float>(segment % segments) / static_cast<float>(segments);
        return center
            + radius * Vector3(std::sin(theta) * std::cos(phi), std::cos(theta), std::sin(theta) * std::sin(phi));
    };

    ea::vector<CsgPolygon> result;
    for (unsigned r = 0; r < rings; ++r)
    {
        for (unsigned s = 0; s < segments; ++s)
        {
            // Ring 0 is the top pole. Winding is clockwise in (x, z) seen from +y, so normals point out.
            if (r == 0)
            {
                const Vector3 positions[3] = {point(0, 0), point(1, s + 1), point(1, s)};
                result.push_back(MakePolygon(positions, shared));
            }
            else if (r + 1 == rings)
            {
                const Vector3 positions[3] = {point(r, s), point(r, s + 1), point(rings, 0)};
                result.push_back(MakePolygon(positions, shared));
            }
            else
            {
                const Vector3 positions[4] = {point(r, s), point(r, s + 1), point(r + 1, s + 1), point(r + 1, s)};
                result.push_back(MakePolygon(positions, shared));
            }
        }
    }
    return result;
}

double SignedVolume(ea::span<const CsgPolygon> polygons)
{
    double volume = 0.0;
    for (const CsgPolygon& polygon : polygons)
    {
        const auto& vertices = polygon.vertices_;
        for (unsigned i = 2; i < vertices.size(); ++i)
        {
            const Vector3 a = vertices[0].GetPosition();
            const Vector3 b = vertices[i - 1].GetPosition();
            const Vector3 c = vertices[i].GetPosition();
            volume += static_cast<double>(a.DotProduct(b.CrossProduct(c))) / 6.0;
        }
    }
    return volume;
}

bool IsEdgeClosed(ea::span<const CsgPolygon> polygons, float epsilon)
{
    const float invEps = 1.0f / epsilon;
    ea::hash_map<IntVector3, unsigned, IntVector3Hash> ids;
    const auto getId = [&](const Vector3& position)
    {
        const IntVector3 key = QuantizePosition(position, invEps);
        const auto it = ids.find(key);
        if (it != ids.end())
            return it->second;
        const unsigned id = ids.size();
        ids[key] = id;
        return id;
    };

    ea::hash_map<uint64_t, int, CsgPackedKeyHash> edges;
    for (const CsgPolygon& polygon : polygons)
    {
        const unsigned count = polygon.vertices_.size();
        for (unsigned i = 0; i < count; ++i)
        {
            const unsigned a = getId(polygon.vertices_[i].GetPosition());
            const unsigned b = getId(polygon.vertices_[(i + 1) % count].GetPosition());
            if (a == b)
                continue;
            ++edges[CsgDirectedKey(a, b)];
        }
    }

    for (const auto& item : edges)
    {
        const unsigned a = static_cast<unsigned>(item.first >> 32);
        const unsigned b = static_cast<unsigned>(item.first & 0xffffffffu);
        const auto opposite = edges.find(CsgDirectedKey(b, a));
        if (opposite == edges.end() || opposite->second != item.second)
            return false;
    }
    return true;
}

} // namespace Urho3D
