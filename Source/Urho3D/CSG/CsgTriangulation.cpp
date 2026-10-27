// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgTriangulation.h"

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/CsgSolid.h"
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Math/MathDefs.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/algorithm.h>
#include <EASTL/hash_map.h>
#include <EASTL/sort.h>

#include <cmath>
#include <limits>

namespace Urho3D
{

namespace
{

/// Distance from the line through its neighbors, relative to the largest coordinate of the polygon, within
/// which a vertex counts as part of a straight run: a few float roundings. Faces that share a run then agree
/// that it is straight, so neither claims the sliver between the run and its chord.
constexpr double STRAIGHT_DISTANCE = 1e-6;

} // namespace

bool CsgTriangulatePolygon(const CsgPolygon& poly, ea::vector<unsigned>& outIndices)
{
    bool clean = true;
    CsgTriangulationScratch scratch;
    return CsgTriangulatePolygon(poly, outIndices, {}, clean, scratch);
}

bool CsgTriangulatePolygon(const CsgPolygon& poly, ea::vector<unsigned>& outIndices, ea::span<const bool> straightMarks,
    bool& clean, CsgTriangulationScratch& scratch)
{
    clean = true;
    const unsigned numVertices = poly.vertices_.size();
    if (numVertices < 3)
        return false;

    outIndices.clear();
    if (numVertices == 3)
    {
        outIndices.push_back(0);
        outIndices.push_back(1);
        outIndices.push_back(2);
        return true;
    }

    // A mark list of the wrong size is ignored.
    const bool* straight = straightMarks.size() == numVertices ? straightMarks.data() : nullptr;

    Vector3 axisU, axisV;
    MakeAxesFromNormal(poly.plane_.normal_, axisU, axisV);
    ea::vector<double>& xs = scratch.xs_;
    ea::vector<double>& ys = scratch.ys_;
    ea::vector<unsigned>& ring = scratch.ring_;
    xs.resize(numVertices);
    ys.resize(numVertices);
    double extent = 0.0;
    for (unsigned i = 0; i < numVertices; ++i)
    {
        const Vector3 p = poly.vertices_[i].GetPosition();
        extent = ea::max(extent, ea::max(static_cast<double>(std::abs(p.x_)),
            ea::max(static_cast<double>(std::abs(p.y_)), static_cast<double>(std::abs(p.z_)))));
        xs[i] = static_cast<double>(p.x_) * axisU.x_ + static_cast<double>(p.y_) * axisU.y_
            + static_cast<double>(p.z_) * axisU.z_;
        ys[i] = static_cast<double>(p.x_) * axisV.x_ + static_cast<double>(p.y_) * axisV.y_
            + static_cast<double>(p.z_) * axisV.z_;
    }

    // Orient tests by the winding of the projected polygon, whatever the handedness of the projection.
    double area = 0.0;
    for (unsigned i = 0; i < numVertices; ++i)
    {
        const unsigned j = (i + 1) % numVertices;
        area += xs[i] * ys[j] - xs[j] * ys[i];
    }
    const double winding = area < 0.0 ? -1.0 : 1.0;

    ring.resize(numVertices);
    for (unsigned i = 0; i < numVertices; ++i)
        ring[i] = i;

    const auto orient = [&](unsigned a, unsigned b, unsigned c)
    { return winding * ((xs[b] - xs[a]) * (ys[c] - ys[a]) - (ys[b] - ys[a]) * (xs[c] - xs[a])); };

    const auto length = [&](unsigned a, unsigned b)
    {
        const double dx = xs[b] - xs[a];
        const double dy = ys[b] - ys[a];
        return std::sqrt(dx * dx + dy * dy);
    };

    const auto same = [&](unsigned a, unsigned b) { return xs[a] == xs[b] && ys[a] == ys[b]; };

    // A vertex on a shared run within rounding. With four vertices left, clipping the vertex opposite one
    // would leave the run itself as the last triangle.
    const auto isStraight = [&](unsigned vertex)
    {
        if (!straight || !straight[vertex])
            return false;
        const unsigned at = static_cast<unsigned>(ea::find(ring.begin(), ring.end(), vertex) - ring.begin());
        const unsigned previous = ring[(at + ring.size() - 1) % ring.size()];
        const unsigned next = ring[(at + 1) % ring.size()];
        const double chord = length(previous, next);
        return chord <= 0.0 || std::abs(orient(previous, vertex, next)) / chord < extent * STRAIGHT_DISTANCE;
    };

    // Return true if the vertex lies on the open segment between two vertices, within rounding.
    const auto touchesEdge = [&](unsigned from, unsigned to, unsigned vertex)
    {
        const double chord = length(from, to);
        if (std::abs(orient(from, to, vertex)) > chord * extent * STRAIGHT_DISTANCE)
            return false;
        const double along = (xs[vertex] - xs[from]) * (xs[to] - xs[from]) + (ys[vertex] - ys[from]) * (ys[to] - ys[from]);
        return along > 0.0 && along < chord * chord;
    };

    // A vertex inside the ear blocks it, and so does one within rounding of the new diagonal: clipping past a
    // vertex on the diagonal would strand it on a straight run with nothing left to connect to. Vertices on
    // the ear's own boundary edges, or their extensions, do not block it.
    const auto containsOther = [&](unsigned a, unsigned b, unsigned c)
    {
        const double diagonal = length(c, a) * extent * STRAIGHT_DISTANCE;
        const unsigned ringSize = ring.size();
        for (unsigned r = 0; r < ringSize; ++r)
        {
            const unsigned j = ring[r];
            if (j == a || j == b || j == c || same(j, a) || same(j, b) || same(j, c))
                continue;
            // A vertex on an ear edge does not block the ear by itself. But a boundary edge from it into the
            // ear side of that edge crosses the ear, and then the diagonal is not inside the polygon.
            const bool onAB = touchesEdge(a, b, j);
            if (onAB || touchesEdge(b, c, j))
            {
                for (const unsigned n : {ring[(r + ringSize - 1) % ringSize], ring[(r + 1) % ringSize]})
                {
                    if (!same(n, j) && (onAB ? orient(a, b, n) : orient(b, c, n)) > 0.0)
                        return true;
                }
            }
            if (orient(a, b, j) <= 0.0 || orient(b, c, j) <= 0.0)
                continue;
            const double side = orient(c, a, j);
            if (side > 0.0)
                return true;
            // Within rounding of the diagonal counts only alongside its interior: a vertex next to an end of it
            // closes a tiny edge rather than lying on a straight run.
            if (side >= -diagonal)
            {
                const double along = (xs[j] - xs[c]) * (xs[a] - xs[c]) + (ys[j] - ys[c]) * (ys[a] - ys[c]);
                const double lengthSquared = (xs[a] - xs[c]) * (xs[a] - xs[c]) + (ys[a] - ys[c]) * (ys[a] - ys[c]);
                if (along > lengthSquared * 1e-3 && along < lengthSquared * (1.0 - 1e-3))
                    return true;
            }
        }
        return false;
    };

    outIndices.reserve((numVertices - 2) * 3);
    while (ring.size() > 3)
    {
        const unsigned count = ring.size();
        int clip = -1;
        int fallback = -1;
        double fallbackTurn = -std::numeric_limits<double>::infinity();
        for (unsigned i = 0; i < count; ++i)
        {
            const unsigned a = ring[(i + count - 1) % count];
            const unsigned b = ring[i];
            const unsigned c = ring[(i + 1) % count];
            const double turn = orient(a, b, c);
            const double scale = length(a, b) * length(b, c);
            const double normalized = scale > 0.0 ? turn / scale : 0.0;
            if (normalized > fallbackTurn)
            {
                fallbackTurn = normalized;
                fallback = static_cast<int>(i);
            }
            // A tip within rounding of a neighbor closes a zero-length edge: the needle has no area to invert, and
            // leaving it would block every ear around it.
            const double needle = extent * STRAIGHT_DISTANCE;
            if ((!straight || !straight[b]) && turn >= 0.0 && (length(a, b) <= needle || length(b, c) <= needle))
            {
                clip = static_cast<int>(i);
                break;
            }
            // Distance of b from the chord a-c is twice the ear's area over the chord length.
            const double chord = length(a, c);
            if (turn <= 0.0 || turn <= chord * extent * STRAIGHT_DISTANCE || isStraight(b)
                || (count == 4 && isStraight(ring[(i + 2) % count])) || containsOther(a, b, c))
                continue;
            clip = static_cast<int>(i);
            break;
        }
        if (clip < 0)
        {
            clean = false;
            // The shared runs left no ear. An ear that ignores the runs still lies inside the polygon, so it is
            // better than the most convex vertex, which can lie outside.
            for (unsigned i = 0; i < count && clip < 0; ++i)
            {
                const unsigned a = ring[(i + count - 1) % count];
                const unsigned b = ring[i];
                const unsigned c = ring[(i + 1) % count];
                const double turn = orient(a, b, c);
                if (turn > length(a, c) * extent * STRAIGHT_DISTANCE && !containsOther(a, b, c))
                    clip = static_cast<int>(i);
            }
            // Not a number turns compare false, so the fallback can stay unset. Then the first vertex is clipped.
            if (clip < 0)
                clip = fallback >= 0 ? fallback : 0;
        }
        outIndices.push_back(ring[(clip + count - 1) % count]);
        outIndices.push_back(ring[clip]);
        outIndices.push_back(ring[(clip + 1) % count]);
        ring.erase(ring.begin() + clip);
    }
    outIndices.push_back(ring[0]);
    outIndices.push_back(ring[1]);
    outIndices.push_back(ring[2]);
    return true;
}

namespace
{

/// Triangulate polygons with the triangles from `triangulate`, given the index of each polygon.
/// Vertices are shared when they coincide and their attributes agree. If `useKeys` is set, `vertexKey` names the
/// position of each polygon vertex by an id below `keySpace`, so the function needs no hashing.
template <class GetPolygon, class Triangulate, class VertexKey>
bool TriangulateGeometryImpl(unsigned polygonCount, GetPolygon&& getPolygon, Triangulate&& triangulate,
    bool useKeys, VertexKey&& vertexKey, unsigned keySpace, float epsilon, ea::vector<ModelVertex>& outVertices,
    ea::vector<unsigned>& outIndexData)
{
    outVertices.clear();
    outIndexData.clear();

    if (polygonCount == 0)
        return false;

    const float eps = ea::max(epsilon, 1e-6f);
    const float invEps = 1.0f / eps;

    unsigned estimatedVertexCount = 0;
    unsigned estimatedIndexCount = 0;
    for (unsigned i = 0; i < polygonCount; ++i)
    {
        const unsigned size = getPolygon(i).vertices_.size();
        estimatedVertexCount += size;
        if (size >= 3)
            estimatedIndexCount += (size - 2u) * 3u;
    }

    outVertices.reserve(estimatedVertexCount);
    outIndexData.reserve(estimatedIndexCount);

    // Vertices with one key form a chain through `next`. Keys are the ids given, else cells of quantized positions.
    ea::vector<int> next;
    next.reserve(estimatedVertexCount);
    ea::vector<int> headsById;
    if (useKeys)
        headsById.resize(keySpace, -1);
    ea::hash_map<IntVector3, int> headsByCell;
    if (!useKeys)
        headsByCell.reserve(estimatedVertexCount);

    unsigned polyIndex = 0;
    const auto getOrCreateVertexIndex = [&](const ModelVertex& vertex, unsigned localIndex) -> unsigned
    {
        int key = 0;
        IntVector3 cell;
        int head = -1;
        if (useKeys)
        {
            key = vertexKey(polyIndex, localIndex);
            head = key >= 0 && static_cast<unsigned>(key) < keySpace ? headsById[key] : -1;
        }
        else
        {
            cell = QuantizePosition(vertex.GetPosition(), invEps);
            const auto found = headsByCell.find(cell);
            head = found != headsByCell.end() ? found->second : -1;
        }

        for (int existingIndex = head; existingIndex >= 0; existingIndex = next[existingIndex])
        {
            const ModelVertex& existing = outVertices[existingIndex];

            // Faces that share a vertex hold the same position exactly. Merging a vertex into one merely
            // nearby would move it for some faces and not for others, opening the mesh along their edges.
            if (existing.position_ != vertex.position_)
                continue;

            // Other attributes are expected to match very closely if the vertex is truly shared.
            if (existing == vertex)
                return static_cast<unsigned>(existingIndex);
        }

        const int newIndex = static_cast<int>(outVertices.size());
        outVertices.push_back(vertex);
        next.push_back(head);
        if (useKeys)
        {
            if (key >= 0 && static_cast<unsigned>(key) < keySpace)
                headsById[key] = newIndex;
        }
        else
            headsByCell[cell] = newIndex;
        return static_cast<unsigned>(newIndex);
    };

    ea::vector<unsigned> triIndices;
    for (polyIndex = 0; polyIndex < polygonCount; ++polyIndex)
    {
        const CsgPolygon& poly = getPolygon(polyIndex);
        if (poly.vertices_.size() < 3)
            continue;

        if (!triangulate(poly, polyIndex, triIndices))
            continue;

        for (const unsigned polyVertexIndex : triIndices)
        {
            if (polyVertexIndex < poly.vertices_.size())
                outIndexData.push_back(getOrCreateVertexIndex(poly.vertices_[polyVertexIndex], polyVertexIndex));
        }
    }

    return !outIndexData.empty();
}

/// Add the geometry to the model and grow the bounding box.
void AddGeometry(CsgTriangulatedModel& result, CsgTriangulatedModel::Geometry&& geometry)
{
    for (const auto& v : geometry.vertices_)
    {
        const Vector3 pos = v.GetPosition();
        if (!result.hasAnyVertex_)
        {
            result.boundingBox_ = BoundingBox(pos, pos);
            result.hasAnyVertex_ = true;
        }
        else
            result.boundingBox_.Merge(pos);
    }

    result.geometries_.push_back(ea::move(geometry));
}

} // namespace

bool CsgTriangulateGeometry(
    const ea::vector<CsgPolygon>& polys, float epsilon,
    ea::vector<ModelVertex>& outVertices, ea::vector<unsigned>& outIndexData)
{
    CsgTriangulationScratch scratch;
    return TriangulateGeometryImpl(
        polys.size(), [&](unsigned i) -> const CsgPolygon& { return polys[i]; },
        [&](const CsgPolygon& poly, unsigned, ea::vector<unsigned>& indices)
        {
            bool clean = true;
            return CsgTriangulatePolygon(poly, indices, {}, clean, scratch);
        },
        false, [](unsigned, unsigned) { return 0; }, 0, epsilon, outVertices, outIndexData);
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

        if (polys.empty())
            continue;

        CsgTriangulatedModel::Geometry geometry;
        geometry.shared_ = sharedKey;

        if (!CsgTriangulateGeometry(polys, epsilon, geometry.vertices_, geometry.indexData_))
            continue;

        if (geometry.vertices_.empty() || geometry.indexData_.empty())
            continue;

        AddGeometry(*result, ea::move(geometry));
    }

    return result;
}

SharedPtr<CsgTriangulatedModel> CsgTriangulateSolid(const CsgSolid& solid, float epsilon)
{
    auto result = MakeShared<CsgTriangulatedModel>();

    const unsigned faceCount = solid.FaceCount();
    if (faceCount == 0)
        return result;

    // Faces of one group in face order, groups in ascending order of the shared key.
    ea::vector<ea::pair<unsigned, unsigned>> order;
    order.reserve(faceCount);
    for (unsigned i = 0; i < faceCount; ++i)
        order.emplace_back(solid.GetFace(i)->shared_, i);
    ea::sort(order.begin(), order.end());

    const unsigned keySpace = solid.VertexKeySpace();
    ea::vector<unsigned> members;
    for (unsigned begin = 0; begin < faceCount;)
    {
        const unsigned sharedKey = order[begin].first;
        members.clear();
        for (; begin < faceCount && order[begin].first == sharedKey; ++begin)
            members.push_back(order[begin].second);

        CsgTriangulatedModel::Geometry geometry;
        geometry.shared_ = sharedKey;

        if (!TriangulateGeometryImpl(
                members.size(), [&](unsigned i) -> const CsgPolygon& { return *solid.GetFace(members[i]); },
                [&](const CsgPolygon&, unsigned i, ea::vector<unsigned>& indices)
                { return solid.CopyTriangles(members[i], indices); },
                true, [&](unsigned i, unsigned local) { return solid.VertexKey(members[i], local); }, keySpace,
                epsilon, geometry.vertices_, geometry.indexData_))
            continue;

        if (geometry.vertices_.empty() || geometry.indexData_.empty())
            continue;

        AddGeometry(*result, ea::move(geometry));
    }

    return result;
}

} // namespace Urho3D
