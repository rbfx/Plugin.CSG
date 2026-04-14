// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgTriangulation.h"

#include "Urho3D/CSG/CsgOptimization.h"
#include <Urho3D/Container/Ptr.h>
#include "Urho3D/CSG/CsgCommon.h"
#include <Urho3D/Math/MathDefs.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/hash_map.h>
#include <EASTL/sort.h>

namespace Urho3D
{

namespace
{

double SignedArea2D(const ea::vector<Vector2>& vertices)
{
    const unsigned count = vertices.size();
    if (count < 3)
        return 0.0;

    double area2 = 0.0;
    for (unsigned i = 0; i < count; ++i)
    {
        const Vector2& a = vertices[i];
        const Vector2& b = vertices[(i + 1u) % count];
        area2 += static_cast<double>(a.x_) * static_cast<double>(b.y_) - static_cast<double>(b.x_) * static_cast<double>(a.y_);
    }
    return 0.5 * area2;
}

double Orient2D(const Vector2& a, const Vector2& b, const Vector2& c)
{
    return static_cast<double>(b.x_ - a.x_) * static_cast<double>(c.y_ - a.y_)
        - static_cast<double>(b.y_ - a.y_) * static_cast<double>(c.x_ - a.x_);
}

double ComputeOrientationEpsilon(const ea::vector<Vector2>& vertices)
{
    if (vertices.empty())
        return 1e-12;

    Vector2 minPoint = vertices.front();
    Vector2 maxPoint = vertices.front();
    for (const Vector2& point : vertices)
    {
        minPoint.x_ = ea::min(minPoint.x_, point.x_);
        minPoint.y_ = ea::min(minPoint.y_, point.y_);
        maxPoint.x_ = ea::max(maxPoint.x_, point.x_);
        maxPoint.y_ = ea::max(maxPoint.y_, point.y_);
    }

    const double scaleX = static_cast<double>(maxPoint.x_ - minPoint.x_);
    const double scaleY = static_cast<double>(maxPoint.y_ - minPoint.y_);
    const double scale = ea::max(1.0, ea::max(scaleX, scaleY));
    return scale * scale * 1e-12;
}

bool IsPointInTriangle(const Vector2& point, const Vector2& a, const Vector2& b, const Vector2& c, double windingSign, double orientEps)
{
    const double ab = Orient2D(a, b, point) * windingSign;
    const double bc = Orient2D(b, c, point) * windingSign;
    const double ca = Orient2D(c, a, point) * windingSign;
    return ab >= -orientEps && bc >= -orientEps && ca >= -orientEps;
}

} // namespace

bool CsgTriangulatePolygon(const CsgPolygon& poly, ea::vector<unsigned>& outIndices)
{
    const unsigned numVertices = poly.vertices_.size();
    if (numVertices < 3)
        return false;

    outIndices.clear();
    // Project to 2D for robust ear clipping in polygon plane.
    Vector3 axisU, axisV;
    MakeAxesFromNormal(poly.plane_.normal_, axisU, axisV);

    ea::vector<Vector2> verts2D(numVertices);
    for (unsigned i = 0; i < numVertices; ++i)
    {
        const Vector3 p = poly.vertices_[i].GetPosition();
        verts2D[i] = {p.DotProduct(axisU), p.DotProduct(axisV)};
    }

    const double orientEps = ComputeOrientationEpsilon(verts2D);
    const double signedArea = SignedArea2D(verts2D);
    if (Abs(signedArea) <= orientEps)
        return false;
    const double windingSign = signedArea > 0.0 ? 1.0 : -1.0;

    ea::vector<unsigned> indices(numVertices);
    for (unsigned i = 0; i < numVertices; ++i)
        indices[i] = i;

    unsigned count = numVertices;
    unsigned maxIterations = count * count;

    while (count > 3)
    {
        if (maxIterations-- == 0)
            return false;

        bool earFound = false;
        for (unsigned i = 0; i < count; ++i)
        {
            const unsigned prev = (i + count - 1u) % count;
            const unsigned next = (i + 1u) % count;

            const unsigned u = indices[prev];
            const unsigned v = indices[i];
            const unsigned w = indices[next];

            const Vector2& p0 = verts2D[u];
            const Vector2& p1 = verts2D[v];
            const Vector2& p2 = verts2D[w];
            const double earOrient = Orient2D(p0, p1, p2) * windingSign;
            if (earOrient <= orientEps)
                continue;

            bool containsVertex = false;
            for (unsigned j = 0; j < count; ++j)
            {
                if (j == prev || j == i || j == next)
                    continue;

                const Vector2& point = verts2D[indices[j]];
                if (IsPointInTriangle(point, p0, p1, p2, windingSign, orientEps))
                {
                    containsVertex = true;
                    break;
                }
            }

            if (containsVertex)
                continue;

            outIndices.push_back(indices[prev]);
            outIndices.push_back(indices[i]);
            outIndices.push_back(indices[next]);
            indices.erase(indices.begin() + i);
            --count;
            earFound = true;
            break;
        }

        if (!earFound)
            return false;
    }

    if (count != 3)
        return false;

    const double finalOrient = Orient2D(verts2D[indices[0]], verts2D[indices[1]], verts2D[indices[2]]) * windingSign;
    if (finalOrient <= orientEps)
        return false;

    outIndices.push_back(indices[0]);
    outIndices.push_back(indices[1]);
    outIndices.push_back(indices[2]);
    return outIndices.size() == (numVertices - 2u) * 3u;
}

bool CsgTriangulateGeometry(
    const ea::vector<CsgPolygon>& polys, float epsilon,
    ea::vector<ModelVertex>& outVertices, ea::vector<unsigned>& outIndexData)
{
    outVertices.clear();
    outIndexData.clear();

    if (polys.empty())
        return false;

    const float eps = ea::max(epsilon, 1e-6f);
    const float invEps = 1.0f / eps;

    unsigned estimatedVertexCount = 0;
    unsigned estimatedIndexCount = 0;
    for (const auto& poly : polys)
    {
        estimatedVertexCount += poly.vertices_.size();
        if (poly.vertices_.size() >= 3)
            estimatedIndexCount += (poly.vertices_.size() - 2u) * 3u;
    }

    outVertices.reserve(estimatedVertexCount);
    outIndexData.reserve(estimatedIndexCount);

    ea::hash_map<IntVector3, ea::vector<unsigned>> buckets;
    buckets.reserve(estimatedVertexCount * 2u);

    const auto getOrCreateVertexIndex = [&](const ModelVertex& vertex) -> unsigned
    {
        const IntVector3 key = QuantizePosition(vertex.GetPosition(), invEps);
        auto& bucket = buckets[key];

        for (const unsigned existingIndex : bucket)
        {
            const ModelVertex& existing = outVertices[existingIndex];

            // Fast reject by position with CSG epsilon.
            if (!existing.position_.Equals(vertex.position_, eps))
                continue;

            // Other attributes are expected to match very closely if the vertex is truly shared.
            if (existing == vertex)
                return existingIndex;
        }

        const unsigned newIndex = outVertices.size();
        outVertices.push_back(vertex);
        bucket.push_back(newIndex);
        return newIndex;
    };

    ea::vector<unsigned> triIndices;
    for (const auto& poly : polys)
    {
        if (poly.vertices_.size() < 3)
            continue;

        if (!CsgTriangulatePolygon(poly, triIndices))
        {
            outVertices.clear();
            outIndexData.clear();
            return false;
        }

        for (const unsigned polyVertexIndex : triIndices)
            outIndexData.push_back(getOrCreateVertexIndex(poly.vertices_[polyVertexIndex]));
    }

    return !outIndexData.empty();
}

SharedPtr<CsgTriangulatedModel> CsgTriangulatePolygons(ea::vector<CsgPolygon> polygons, float epsilon)
{
    auto result = MakeShared<CsgTriangulatedModel>();

    if (polygons.empty())
        return result;

    ea::hash_map<unsigned, ea::vector<CsgPolygon>> groups;
    for (auto& p : polygons)
        groups[p.shared_].push_back(ea::move(p));

    ea::vector<unsigned> groupKeys;
    groupKeys.reserve(groups.size());
    for (const auto& pair : groups)
        groupKeys.push_back(pair.first);
    ea::sort(groupKeys.begin(), groupKeys.end());

    result->geometries_.reserve(groupKeys.size());

    for (const unsigned sharedKey : groupKeys)
    {
        auto& polys = groups[sharedKey];

        CsgSimplifyPolygons(polys, epsilon);
        if (polys.empty())
            continue;

        CsgTriangulatedModel::Geometry geometry;
        geometry.shared_ = sharedKey;

        if (!CsgTriangulateGeometry(polys, epsilon, geometry.vertices_, geometry.indexData_))
            return {};

        if (geometry.vertices_.empty() || geometry.indexData_.empty())
            return {};

        for (const auto& v : geometry.vertices_)
        {
            const Vector3 pos = v.GetPosition();
            if (!result->hasAnyVertex_)
            {
                result->boundingBox_ = BoundingBox(pos, pos);
                result->hasAnyVertex_ = true;
            }
            else
                result->boundingBox_.Merge(pos);
        }

        result->geometries_.push_back(ea::move(geometry));
    }

    return result;
}

} // namespace Urho3D
