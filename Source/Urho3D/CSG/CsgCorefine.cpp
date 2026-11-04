// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCorefine.h"

#include "Urho3D/CSG/CsgExactFilter.h"
#include "Urho3D/CSG/CsgSimd.h"
#include <Urho3D/IO/Log.h>

#include <EASTL/algorithm.h>
#include <EASTL/sort.h>
#include <EASTL/unique_ptr.h>

#ifdef CSG_CHECK_SUSPECTS
#include <cstdio>
#endif
#include <cmath>
#include <limits>

namespace Urho3D
{

namespace CsgInternal
{


namespace
{

/// Tolerance of the first comparison of two crossings on an edge.
constexpr double EDGE_ORDER_TOLERANCE = 1e-9;

/// Number of ids that the sort of the crossings of an edge sorts by insertion.
constexpr int INSERTION_SORT_LIMIT = 16;

int SignOfFloat(float value)
{
    return value > 0.0f ? 1 : -1;
}

/// Return true if each coordinate of the grid point is inside the range of the grid.
[[maybe_unused]] bool InGridRange(const CsgGridPoint& point)
{
    const auto inRange = [](CsgGridInt value) { return value > -CsgGridIntegerLimit && value < CsgGridIntegerLimit; };
    return inRange(point.x_) && inRange(point.y_) && inRange(point.z_);
}

/// Sort the ids with a three-way comparison. The sort is stable, and it is safe for an inconsistent comparison.
/// A short list uses insertion sort. A long list uses a bottom-up merge sort.
template <class Compare> void SortIds(ea::vector<int>& ids, ea::vector<int>& merge, const Compare& compare)
{
    const int count = static_cast<int>(ids.size());
    for (int begin = 0; begin < count; begin += INSERTION_SORT_LIMIT)
    {
        const int end = ea::min(begin + INSERTION_SORT_LIMIT, count);
        for (int i = begin + 1; i < end; ++i)
        {
            const int value = ids[i];
            int j = i;
            while (j > begin && compare(ids[j - 1], value) > 0)
            {
                ids[j] = ids[j - 1];
                --j;
            }
            ids[j] = value;
        }
    }
    if (count <= INSERTION_SORT_LIMIT)
        return;

    merge.resize(count);
    int* from = ids.data();
    int* to = merge.data();
    for (int width = INSERTION_SORT_LIMIT; width < count; width *= 2)
    {
        for (int begin = 0; begin < count; begin += 2 * width)
        {
            const int middle = ea::min(begin + width, count);
            const int end = ea::min(begin + 2 * width, count);
            int left = begin;
            int right = middle;
            int out = begin;
            while (left < middle && right < end)
                to[out++] = compare(from[left], from[right]) <= 0 ? from[left++] : from[right++];
            while (left < middle)
                to[out++] = from[left++];
            while (right < end)
                to[out++] = from[right++];
        }
        ea::swap(from, to);
    }
    if (from != ids.data())
    {
        for (int i = 0; i < count; ++i)
            ids[i] = from[i];
    }
}

/// Side of each corner of a triangle against the plane of a triangle of the other operand.
void Sides(const Mesh& corners, int triangle, const Mesh& other, int otherTriangle, int (&sides)[3])
{
    const CsgGridPoint& t0 = other.grid_[other.triangleIds_[otherTriangle * 3]];
    const CsgGridPoint& t1 = other.grid_[other.triangleIds_[otherTriangle * 3 + 1]];
    const CsgGridPoint& t2 = other.grid_[other.triangleIds_[otherTriangle * 3 + 2]];
    const int mask = corners.isSecond_ ? 0b1000 : 0b0111;
    const double faceExtent = other.faceExtent_[other.triangleFace_[otherTriangle]];
    const int* ids = &corners.triangleIds_[triangle * 3];
    const CsgGridPoint& d0 = corners.grid_[ids[0]];
    const CsgGridPoint& d1 = corners.grid_[ids[1]];
    const CsgGridPoint& d2 = corners.grid_[ids[2]];

    // The three corners share the plane, one lane each. Undecided lanes take the exact path.
    const auto diff = [](int64_t a, int64_t b) { return static_cast<float>(a - b); };
    CsgFloat4 detVector;
    const int decided = Filter(CsgFloat4::Splat(diff(t1.x_, t0.x_)), CsgFloat4::Splat(diff(t1.y_, t0.y_)),
        CsgFloat4::Splat(diff(t1.z_, t0.z_)), CsgFloat4::Splat(diff(t2.x_, t0.x_)),
        CsgFloat4::Splat(diff(t2.y_, t0.y_)), CsgFloat4::Splat(diff(t2.z_, t0.z_)),
        CsgFloat4::Create(diff(d0.x_, t0.x_), diff(d1.x_, t0.x_), diff(d2.x_, t0.x_), 0.0f),
        CsgFloat4::Create(diff(d0.y_, t0.y_), diff(d1.y_, t0.y_), diff(d2.y_, t0.y_), 0.0f),
        CsgFloat4::Create(diff(d0.z_, t0.z_), diff(d1.z_, t0.z_), diff(d2.z_, t0.z_), 0.0f), faceExtent,
        detVector);
    float det[4];
    detVector.Store(det);
    sides[0] = (decided & 1) != 0 ? SignOfFloat(det[0]) : CsgExact::OrientPlane(t0, t1, t2, d0, mask, faceExtent);
    sides[1] = (decided & 2) != 0 ? SignOfFloat(det[1]) : CsgExact::OrientPlane(t0, t1, t2, d1, mask, faceExtent);
    sides[2] = (decided & 4) != 0 ? SignOfFloat(det[2]) : CsgExact::OrientPlane(t0, t1, t2, d2, mask, faceExtent);
}

} // namespace

int CompareOnEdge(const Crossing& x, const Crossing& y, bool onePoint)
{
    const CsgExactVolume& dx = x.difference_;
    const CsgExactVolume& dy = y.difference_;
    // t_x - t_y = (Lx * Dy - Ly * Dx) / (Dx * Dy)
    const int numeratorSign = CsgExpansionProductDifferenceSign(
        x.lowVolume_.count_, x.lowVolume_.terms_, dy.count_, dy.terms_, y.lowVolume_.count_, y.lowVolume_.terms_,
        dx.count_, dx.terms_);
    const int denominatorSign = dx.Sign() * dy.Sign();
    if (numeratorSign != 0 && !onePoint)
        return numeratorSign * denominatorSign;

    // Tie: compare delta * (Nx / Dx - Ny / Dy), component by component.
    const int shift = x.key_.edgeInSecond_ ? 1 : -1;
    const CsgExactMinor* components[3][2] = {
        {&x.normal_.x_, &y.normal_.x_}, {&x.normal_.y_, &y.normal_.y_}, {&x.normal_.z_, &y.normal_.z_}};
    for (const auto& component : components)
    {
        const int differenceSign = CsgExpansionProductDifferenceSign(component[0]->count_, component[0]->terms_,
            dy.count_, dy.terms_, component[1]->count_, component[1]->terms_, dx.count_, dx.terms_);
        if (differenceSign != 0)
            return differenceSign * denominatorSign * shift;
    }
    return 0;
}

Mesh::Mesh(const CsgSolid& solid, bool isSecond, int offset)
    : solid_(solid)
    , loops_(solid.GetExactLoops())
    , isSecond_(isSecond)
    , offset_(offset)
{
    const auto triangleStart = solid.GetTriangleStart();
    const auto triangleVertices = solid.GetTriangleVertices();
    const unsigned triangleCount = triangleStart.empty() ? 0 : triangleStart.back();
    triangleFace_.resize(triangleCount);
    triangleIds_.resize(triangleCount * 3);
    faceExtent_.assign(solid.FaceCount(), 0.0);
    for (unsigned face = 0; face < solid.FaceCount(); ++face)
    {
        const unsigned start = loops_.vertexStart_[face];
        for (unsigned t = triangleStart[face]; t < triangleStart[face + 1]; ++t)
        {
            triangleFace_[t] = static_cast<int>(face);
            for (unsigned corner = 0; corner < 3; ++corner)
            {
                const int id = loops_.ids_[start + triangleVertices[t * 3 + corner]];
                triangleIds_[t * 3 + corner] = id;
                faceExtent_[face] = ea::max(faceExtent_[face], CsgExact::Extent(loops_.grid_[id]));
            }
        }
    }

    // The solid stores the grid points. The mesh does not snap the positions again.
    grid_ = loops_.grid_;
}

bool Mesh::IsCollapsed(int triangle) const
{
    const int a = triangleIds_[triangle * 3];
    const int b = triangleIds_[triangle * 3 + 1];
    const int c = triangleIds_[triangle * 3 + 2];
    return a == b || b == c || c == a;
}

void Mesh::BuildEdgeTriangles(const ea::vector<uint8_t>* vertexFilter)
{
    if (edgeTrianglesBuilt_)
        return;
    edgeTrianglesBuilt_ = true;
    const int triangleCount = static_cast<int>(triangleFace_.size());
    if (!vertexFilter)
        edgeTriangles_.Reserve(static_cast<size_t>(triangleCount) * 3 / 2 + 3);
    for (int triangle = 0; triangle < triangleCount; ++triangle)
    {
        // A collapsed triangle has no area. It has no crossing and no part in a triangle map.
        if (IsCollapsed(triangle))
            continue;
        for (int corner = 0; corner < 3; ++corner)
        {
            const int u = triangleIds_[triangle * 3 + corner];
            const int v = triangleIds_[triangle * 3 + (corner + 1) % 3];
            if (vertexFilter && ((*vertexFilter)[u] == 0 || (*vertexFilter)[v] == 0))
                continue;
            bool existed = false;
            CsgEdgeTriangles& entry = edgeTriangles_.FindOrAdd(
                CsgUndirectedKey(static_cast<unsigned>(ea::min(u, v)), static_cast<unsigned>(ea::max(u, v))), existed);
            if (entry.first_ < 0)
                entry.first_ = triangle;
            else if (entry.second_ < 0)
                entry.second_ = triangle;
        }
    }
}

const CsgEdgeTriangles* Mesh::FindEdgeTriangles(int a, int b) const
{
    return edgeTriangles_.Find(
        CsgUndirectedKey(static_cast<unsigned>(ea::min(a, b)), static_cast<unsigned>(ea::max(a, b))));
}

Corefinement::Corefinement(const CsgSolid& a, const CsgSolid& b, float epsilon)
    : a_(a, false, 0)
    , b_(b, true, a_.VertexCount())
    , epsilon_(epsilon)
{
    URHO3D_ASSERT(!a.IsOutOfRange() && !b.IsOutOfRange(), "CsgCorefine: an operand is out of range");
    segmentsA_.Reset(static_cast<unsigned>(a_.triangleFace_.size()));
    segmentsB_.Reset(static_cast<unsigned>(b_.triangleFace_.size()));
    touchedA_.assign(a_.triangleFace_.size(), 0);
    touchedB_.assign(b_.triangleFace_.size(), 0);
    if (!g_csgExactOnly)
        BuildAliases();
}

void Corefinement::BuildAliases()
{
    if (b_.VertexCount() == 0 || !a_.solid_.BoundsOverlap(b_.solid_.GetBoundsMin(), b_.solid_.GetBoundsMax(), epsilon_))
        return;
    // The box of the grid points of the second operand. A vertex of the first operand outside it has no twin.
    CsgGridInt minX = std::numeric_limits<CsgGridInt>::max();
    CsgGridInt minY = minX;
    CsgGridInt minZ = minX;
    CsgGridInt maxX = std::numeric_limits<CsgGridInt>::lowest();
    CsgGridInt maxY = maxX;
    CsgGridInt maxZ = maxX;
    for (const CsgGridPoint& point : b_.grid_)
    {
        minX = ea::min(minX, point.x_);
        minY = ea::min(minY, point.y_);
        minZ = ea::min(minZ, point.z_);
        maxX = ea::max(maxX, point.x_);
        maxY = ea::max(maxY, point.y_);
        maxZ = ea::max(maxZ, point.z_);
    }
    const auto keyOf = [](const CsgGridPoint& point)
    {
        CsgVec3Key key;
        key.x_ = static_cast<uint32_t>(point.x_);
        key.y_ = static_cast<uint32_t>(point.y_);
        key.z_ = static_cast<uint32_t>(point.z_);
        return key;
    };
    CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash> onPoint;
    bool onPointBuilt = false;
    for (int id = 0; id < a_.VertexCount(); ++id)
    {
        const CsgGridPoint& point = a_.grid_[id];
        if (point.x_ < minX || point.x_ > maxX || point.y_ < minY || point.y_ > maxY || point.z_ < minZ
            || point.z_ > maxZ)
            continue;
        if (!onPointBuilt)
        {
            onPointBuilt = true;
            onPoint.Reserve(b_.VertexCount());
            for (int other = 0; other < b_.VertexCount(); ++other)
                onPoint.TryAdd(keyOf(b_.grid_[other]), other);
        }
        const int* twin = onPoint.Find(keyOf(point));
        if (!twin)
            continue;
        if (alias_.empty())
        {
            alias_.assign(b_.VertexCount(), -1);
            aliasBack_.assign(a_.VertexCount(), -1);
        }
        // The first vertex of each operand on a grid point makes the pair.
        if (alias_[*twin] < 0)
        {
            alias_[*twin] = id;
            aliasBack_[id] = *twin;
        }
    }
}

bool Corefinement::SharedLoop(const Mesh& mesh, int face, bool flip, ea::vector<int>* loop) const
{
    if (alias_.empty())
        return false;
    const unsigned start = mesh.loops_.vertexStart_[face];
    const unsigned end = mesh.loops_.vertexStart_[face + 1];
    if (end - start < 3)
        return false;
    for (unsigned i = start; i < end; ++i)
    {
        if (!IsShared(mesh, mesh.loops_.ids_[i]))
            return false;
    }
    if (loop)
    {
        loop->resize(end - start);
        for (unsigned i = start; i < end; ++i)
            (*loop)[flip ? end - 1 - i : i - start] = Global(mesh, mesh.loops_.ids_[i]);
    }
    return true;
}

void Corefinement::CollectCrossings(CsgScratch& scratch, const CsgParallelOptions& parallel)
{
    const CsgSolid& solidA = a_.solid_;
    const CsgSolid& solidB = b_.solid_;
    if (!solidA.BoundsOverlap(solidB.GetBoundsMin(), solidB.GetBoundsMax(), epsilon_))
        return;

    scratch.componentPairs_.clear();
    scratch.facePairs_.clear();
    solidA.GetComponentTree().CollectOverlaps(
        solidB.GetComponentTree(), epsilon_, scratch.componentPairs_, scratch.stack_);
    for (const CsgFaceBvh::ItemPair& pair : scratch.componentPairs_)
    {
        const unsigned componentA = static_cast<unsigned>(pair.first);
        const unsigned componentB = static_cast<unsigned>(pair.second);
        ea::unique_ptr<CsgFaceBvh> temporaryA;
        ea::unique_ptr<CsgFaceBvh> temporaryB;
        const CsgFaceBvh& treeA = solidA.GetNearFaces(componentA, solidB.GetComponentMin(componentB),
            solidB.GetComponentMax(componentB), epsilon_, temporaryA);
        const CsgFaceBvh& treeB = solidB.GetNearFaces(componentB, solidA.GetComponentMin(componentA),
            solidA.GetComponentMax(componentA), epsilon_, temporaryB);
        treeA.CollectOverlaps(treeB, epsilon_, scratch.facePairs_, scratch.stack_, parallel);
    }
    // In thick mode, the result depends on the order of the crossings: Placed snaps a crossing to an earlier one.
    // A parallel search gives the pairs in another order. The sort gives one order for every parallel option.
    if (!g_csgExactOnly)
    {
        ea::sort(scratch.facePairs_.begin(), scratch.facePairs_.end(),
            [](const CsgFaceBvh::ItemPair& x, const CsgFaceBvh::ItemPair& y)
            { return x.first != y.first ? x.first < y.first : x.second < y.second; });
    }
    for (const CsgFaceBvh::ItemPair& pair : scratch.facePairs_)
    {
        if (a_.loops_.loopLength_[pair.first] >= 3 && b_.loops_.loopLength_[pair.second] >= 3)
            IntersectFaces(pair.first, pair.second);
    }
}

void Corefinement::IntersectFaces(int faceA, int faceB)
{
    const auto startA = a_.solid_.GetTriangleStart();
    const auto startB = b_.solid_.GetTriangleStart();
    const auto positionsA = a_.solid_.GetTrianglePositions();
    const auto positionsB = b_.solid_.GetTrianglePositions();
    int found[6] = {};
    bool entering[6] = {};
    int sidesA[3];
    int sidesB[3];
    for (unsigned ta = startA[faceA]; ta < startA[faceA + 1]; ++ta)
    {
        // A triangle whose corners weld together has no area. Its neighbors claim every crossing.
        if (a_.IsCollapsed(static_cast<int>(ta)))
            continue;
        const Vector3& a0 = positionsA[ta * 3];
        const Vector3& a1 = positionsA[ta * 3 + 1];
        const Vector3& a2 = positionsA[ta * 3 + 2];
        const Vector3 minA(ea::min(ea::min(a0.x_, a1.x_), a2.x_), ea::min(ea::min(a0.y_, a1.y_), a2.y_),
            ea::min(ea::min(a0.z_, a1.z_), a2.z_));
        const Vector3 maxA(ea::max(ea::max(a0.x_, a1.x_), a2.x_), ea::max(ea::max(a0.y_, a1.y_), a2.y_),
            ea::max(ea::max(a0.z_, a1.z_), a2.z_));
        for (unsigned tb = startB[faceB]; tb < startB[faceB + 1]; ++tb)
        {
            if (b_.IsCollapsed(static_cast<int>(tb)))
                continue;
            const Vector3& b0 = positionsB[tb * 3];
            const Vector3& b1 = positionsB[tb * 3 + 1];
            const Vector3& b2 = positionsB[tb * 3 + 2];
            const Vector3 minB(ea::min(ea::min(b0.x_, b1.x_), b2.x_), ea::min(ea::min(b0.y_, b1.y_), b2.y_),
                ea::min(ea::min(b0.z_, b1.z_), b2.z_));
            const Vector3 maxB(ea::max(ea::max(b0.x_, b1.x_), b2.x_), ea::max(ea::max(b0.y_, b1.y_), b2.y_),
                ea::max(ea::max(b0.z_, b1.z_), b2.z_));
            // The negated test also skips a NaN box.
            if (!(ea::max(minA.x_, minB.x_) <= ea::min(maxA.x_, maxB.x_) + epsilon_
                    && ea::max(minA.y_, minB.y_) <= ea::min(maxA.y_, maxB.y_) + epsilon_
                    && ea::max(minA.z_, minB.z_) <= ea::min(maxA.z_, maxB.z_) + epsilon_))
                continue;

            // Sides of each triangle's corners against the other's plane. When all corners of either
            // triangle lie on one side, the triangles cannot meet.
            Sides(a_, static_cast<int>(ta), b_, static_cast<int>(tb), sidesA);
            if (sidesA[0] != 0 && sidesA[0] == sidesA[1] && sidesA[1] == sidesA[2])
                continue;
            Sides(b_, static_cast<int>(tb), a_, static_cast<int>(ta), sidesB);
            if (sidesB[0] != 0 && sidesB[0] == sidesB[1] && sidesB[1] == sidesB[2])
                continue;
            int count = 0;
            count = EdgesThrough(a_, static_cast<int>(ta), b_, static_cast<int>(tb), sidesA, found, entering, count);
            count = EdgesThrough(b_, static_cast<int>(tb), a_, static_cast<int>(ta), sidesB, found, entering, count);
            if (count == 0)
                continue;
            // A crossing of an edge of tb is at a point inside ta. Both triangles are split, also without a segment.
            // Else a triangle next to the crossing is whole and misses the crossing on its edge.
            touchedA_[ta] = 1;
            touchedB_[tb] = 1;
            // Exact decisions give two ends. Anything else comes from a degenerate triangle. Two ends at one grid
            // point have one id, so the segment has no length.
            if (count != 2 || found[0] == found[1])
                continue;
            ea::pair<int, int> segment = entering[1] && !entering[0] ? ea::make_pair(found[1], found[0])
                                                                      : ea::make_pair(found[0], found[1]);
            segmentsA_.Append(static_cast<int>(ta), segment.first, segment.second);
            segmentsB_.Append(static_cast<int>(tb), segment.first, segment.second);
            bool existed = false;
            curveEdges_.FindOrAdd(CsgUndirectedKey(found[0], found[1]), existed) = segment;
        }
    }
}

int Corefinement::EdgesThrough(const Mesh& edges, int triangle, const Mesh& other, int otherTriangle,
    const int (&sides)[3], int (&found)[6], bool (&entering)[6], int count)
{
    const CsgGridPoint& t0 = other.grid_[other.triangleIds_[otherTriangle * 3]];
    const CsgGridPoint& t1 = other.grid_[other.triangleIds_[otherTriangle * 3 + 1]];
    const CsgGridPoint& t2 = other.grid_[other.triangleIds_[otherTriangle * 3 + 2]];
    for (int corner = 0; corner < 3; ++corner)
    {
        const int next = (corner + 1) % 3;
        // An edge crosses the plane only between corners on opposite sides of it.
        if (sides[corner] == 0 || sides[next] == 0 || sides[corner] == sides[next])
            continue;
        const int u = edges.triangleIds_[triangle * 3 + corner];
        const int v = edges.triangleIds_[triangle * 3 + next];
        const int low = ea::min(u, v);
        const int high = ea::max(u, v);
        if (!CsgExact::PassesInside(edges.grid_[low], edges.grid_[high], t0, t1, t2, edges.isSecond_))
            continue;
        const int id = CrossingId(CsgCrossingKey{low, high, otherTriangle, edges.isSecond_}, edges, t0, t1, t2,
            other.faceExtent_[other.triangleFace_[otherTriangle]]);
        if (count < 6)
        {
            const bool headAbove = sides[next] > 0;
            found[count] = id;
            entering[count] = edges.isSecond_ ? !headAbove : headAbove;
        }
        ++count;
    }
    return count;
}

CsgGridPoint Corefinement::Placed(const CsgGridPoint& grid, bool isVertex, int record, bool& inGroup)
{
    if (g_csgExactOnly)
        return grid;
    const int64_t reach = static_cast<int64_t>(
        ea::min(PlacedReach * CsgExact::Thickness(CsgExact::Extent(grid)), static_cast<double>(1 << (PlacedShift - 1))));
    int64_t best = -1;
    CsgGridPoint found = grid;
    int foundRecord = -1;
    if (!isVertex && placed_.Size() > 0)
    {
        for (int64_t x = (grid.x_ - reach) >> PlacedShift; x <= (grid.x_ + reach) >> PlacedShift; ++x)
            for (int64_t y = (grid.y_ - reach) >> PlacedShift; y <= (grid.y_ + reach) >> PlacedShift; ++y)
                for (int64_t z = (grid.z_ - reach) >> PlacedShift; z <= (grid.z_ + reach) >> PlacedShift; ++z)
                {
                    const CsgGridCellKey cell{static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z)};
                    const ea::vector<PlacedPoint>* near = placed_.Find(cell);
                    if (!near)
                        continue;
                    for (const PlacedPoint& point : *near)
                    {
                        const CsgGridPoint& other = point.grid_;
                        const int64_t distance = ea::max(std::abs(static_cast<int64_t>(other.x_) - grid.x_),
                            ea::max(std::abs(static_cast<int64_t>(other.y_) - grid.y_),
                                std::abs(static_cast<int64_t>(other.z_) - grid.z_)));
                        if (distance <= reach && (best < 0 || distance < best))
                        {
                            best = distance;
                            found = other;
                            foundRecord = point.record_;
                        }
                    }
                }
    }
    if (best >= 0)
    {
        crossings_[foundRecord].placedGroup_ = true;
        inGroup = true;
        return found;
    }
    bool existed = false;
    placed_.FindOrAdd(CsgGridCellKey{grid.x_ >> PlacedShift, grid.y_ >> PlacedShift, grid.z_ >> PlacedShift}, existed)
        .push_back(PlacedPoint{grid, record});
    return grid;
}

int Corefinement::CrossingId(const CsgCrossingKey& key, const Mesh& edges, const CsgGridPoint& t0,
    const CsgGridPoint& t1, const CsgGridPoint& t2, double faceExtent)
{
    bool existed = false;
    int& idRef = crossingIds_.FindOrAdd(key, existed);
    if (existed)
        return idRef;

    const CsgNormal normal = CsgExact::Normal(t0, t1, t2);
    const CsgGridPoint& lowGrid = edges.grid_[key.low_];
    const CsgGridPoint& highGrid = edges.grid_[key.high_];
    CsgExactVolume lowVolume = CsgExact::Dot(normal, t0, lowGrid);
    CsgExactVolume difference = CsgExact::Dot(normal, highGrid, lowGrid);
    // In thick mode, an end that the orientation takes as on the plane has zero volume. The order on the edge
    // and the position then agree with the orientation. That end is the crossing.
    bool lowFlat = false;
    bool highFlat = false;
    if (!g_csgExactOnly)
    {
        lowFlat = CsgExact::FlatPlane(t0, t1, t2, lowGrid, faceExtent);
        highFlat = CsgExact::FlatPlane(t0, t1, t2, highGrid, faceExtent);
        if (lowFlat || highFlat)
        {
            // The difference is lowVolume - highVolume. Compute it before a volume changes.
            const CsgExactVolume highVolume = highFlat ? CsgExactVolume{} : CsgExact::Dot(normal, t0, highGrid);
            difference = lowFlat ? (highFlat ? CsgExactVolume{} : CsgExact::Negate(highVolume)) : lowVolume;
            if (lowFlat)
                lowVolume = CsgExactVolume{};
        }
    }
    double t = 0.5;
    CsgGridPoint grid;
    const int record = static_cast<int>(crossings_.size());
    bool inGroup = false;
    if (lowFlat)
        grid = Placed(lowGrid, true, record, inGroup);
    else if (highFlat)
        grid = Placed(highGrid, true, record, inGroup);
    else if (difference.Sign() != 0)
    {
        // The rounding is exact and it ignores the sign of the difference.
        grid = CsgExact::RoundCrossing(lowGrid, highGrid, lowVolume, difference);
    }
    else
    {
        // An edge in the plane has no crossing point. Use the middle, as for t.
        grid.x_ = static_cast<CsgGridInt>((static_cast<int64_t>(lowGrid.x_) + highGrid.x_ + 1) >> 1);
        grid.y_ = static_cast<CsgGridInt>((static_cast<int64_t>(lowGrid.y_) + highGrid.y_ + 1) >> 1);
        grid.z_ = static_cast<CsgGridInt>((static_cast<int64_t>(lowGrid.z_) + highGrid.z_ + 1) >> 1);
    }
    if (difference.Sign() != 0)
    {
        t = lowVolume.Approximate() / difference.Approximate();
        t = std::isnan(t) ? 0.5 : (t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t));
    }
    // An end on the plane is the crossing itself and it does not move. Other crossings snap to an earlier one.
    if (!g_csgExactOnly && !lowFlat && !highFlat)
        grid = Placed(grid, false, record, inGroup);

    Crossing data;
    data.placedGroup_ = inGroup;
    data.grid_ = grid;
    // The crossing lies between the ends of the edge, so it is in the range of the grid.
    URHO3D_ASSERT(InGridRange(grid), "CsgCorefine crossing is outside the range of the grid");
    // The position is grid / scale, so the result vertex is on the grid and the next boolean keeps it.
    data.position_ = grid.ToPosition();
    data.t_ = t;
    data.key_ = key;
    data.lowVolume_ = lowVolume;
    data.difference_ = difference;
    data.normal_ = normal;

    int id = CrossingBase() + record;
    crossings_.push_back(data);
    idRef = id;

    bool listExisted = false;
    EdgeList& list = edgeCrossings_[key.edgeInSecond_ ? 1 : 0].FindOrAdd(
        CsgDirectedKey(static_cast<unsigned>(key.low_), static_cast<unsigned>(key.high_)), listExisted);
    const int node = static_cast<int>(edgeNodes_.size());
    edgeNodes_.push_back(EdgeNode{id, record, -1});
    if (list.tail_ < 0)
        list.head_ = node;
    else
        edgeNodes_[list.tail_].next_ = node;
    list.tail_ = node;
    return id;
}

int Corefinement::CompareRecords(int x, int y) const
{
    const Crossing& cx = Record(x);
    const Crossing& cy = Record(y);
    // Doubles separate all but nearly coincident crossings. Those are ordered exactly.
    // The value t_ is the quotient of two Approximate() sums of compressed expansions. Its relative error
    // is about 2^-47, far below the tolerance of 1e-9. A larger gap of t_ then has the sign of the exact gap.
    // A gap of 1e-9 or less goes to CompareOnEdge. Then the exact tie rule and the ids decide.
    if (std::fabs(cx.t_ - cy.t_) > EDGE_ORDER_TOLERANCE)
        return cx.t_ < cy.t_ ? -1 : 1;
    const int order = CompareOnEdge(cx, cy);
    if (order != 0)
        return order;
    const int idX = RecordId(x);
    const int idY = RecordId(y);
    return idX < idY ? -1 : (idX > idY ? 1 : 0);
}

void Corefinement::EdgeCrossings(
    const Mesh& mesh, int from, int to, CsgScratch& scratch, ea::vector<int>& output) const
{
    output.clear();
    const EdgeList* list = edgeCrossings_[mesh.isSecond_ ? 1 : 0].Find(
        CsgDirectedKey(static_cast<unsigned>(ea::min(from, to)), static_cast<unsigned>(ea::max(from, to))));
    if (!list)
        return;
    // The output holds the records while it is sorted.
    for (int node = list->head_; node >= 0; node = edgeNodes_[node].next_)
        output.push_back(edgeNodes_[node].record_);

    SortIds(output, scratch.merge_, [this](int x, int y) { return CompareRecords(x, y); });
    // In thick mode, neighbors at one grid point are one point: an edge that meets an edge of the other operand
    // crosses the two faces on that edge there. The displacement decides on which side the edges pass, so it also
    // orders these crossings. Their exact order can be the opposite one.
    if (!g_csgExactOnly)
    {
        const int count = static_cast<int>(output.size());
        for (int first = 0; first < count;)
        {
            int end = first + 1;
            while (end < count && Record(output[end]).grid_ == Record(output[first]).grid_)
                ++end;
            // A run is short, so an insertion sort is enough. It is stable and it is safe for an inconsistent order.
            for (int i = first + 1; i < end; ++i)
            {
                const int value = output[i];
                int j = i;
                while (j > first)
                {
                    int order = CompareOnEdge(Record(output[j - 1]), Record(value), true);
                    if (order == 0)
                        order = output[j - 1] < value ? -1 : (output[j - 1] > value ? 1 : 0);
                    if (order <= 0)
                        break;
                    output[j] = output[j - 1];
                    --j;
                }
                output[j] = value;
            }
            first = end;
        }
    }
    // Crossings of one edge that round to one grid point have one id. They are next to each other.
    int kept = 0;
    for (int i = 0; i < static_cast<int>(output.size()); ++i)
    {
        const int id = RecordId(output[i]);
        if (kept == 0 || output[kept - 1] != id)
            output[kept++] = id;
    }
    output.resize(kept);
    if (from > to)
        ea::reverse(output.begin(), output.end());
}

namespace
{

/// Compare the magnitudes of two exact minors: the sign of |a| - |b|.
/// The double values select the result when they differ clearly. Otherwise the expansions decide.
int CompareMagnitude(const CsgExactMinor& a, const CsgExactMinor& b)
{
    // The approximations have a relative error far below 2^-40, so the factor separates only safe cases.
    const double fa = std::fabs(a.Approximate());
    const double fb = std::fabs(b.Approximate());
    if (fa > fb * (1.0 + 1e-9))
        return 1;
    if (fb > fa * (1.0 + 1e-9))
        return -1;
    CsgReal ta[4];
    CsgReal tb[4];
    const CsgReal sa = a.Sign() < 0 ? CsgReal(-1) : CsgReal(1);
    const CsgReal sb = b.Sign() < 0 ? CsgReal(-1) : CsgReal(1);
    for (int i = 0; i < a.count_; ++i)
        ta[i] = a.terms_[i] * sa;
    for (int i = 0; i < b.count_; ++i)
        tb[i] = b.terms_[i] * sb;
    const CsgReal one = 1;
    return CsgExpansionProductDifferenceSign(a.count_, ta, 1, &one, b.count_, tb, 1, &one);
}

/// Whether the bridge from-to is blocked by the edge pq.
/// A contact only at an end of the bridge does not block it. A collinear contact blocks it, also at an end.
/// The callers skip the edges at the end ids. An edge that touches an end then has another id at the grid point
/// of the end. The position of an id is grid / scale of its grid point, so such ids have one position.
/// The edge between them has zero length. The exact tests treat the ids as one point.
bool BridgeBlockedBy(const CsgGridPoint2& from, const CsgGridPoint2& to, const CsgGridPoint2& p, const CsgGridPoint2& q)
{
    if (!CsgExact::SegmentsCross(from, to, p, q))
        return false;
    const int o1 = CsgExact::Orient2(from, to, p);
    const int o2 = CsgExact::Orient2(from, to, q);
    if (o1 == 0 && o2 == 0)
        return true;
    // The lines are not the same, so the segments have one common point.
    if (p == from || p == to || q == from || q == to)
        return false;
    return CsgExact::Orient2(p, q, from) != 0 && CsgExact::Orient2(p, q, to) != 0;
}

uint64_t DirectedEdgeKey(int from, int to)
{
    return CsgDirectedKey(static_cast<unsigned>(from), static_cast<unsigned>(to));
}

/// Hash of a sorted id list. Equal lists give equal hashes.
uint64_t HashSortedIds(const int* ids, int count)
{
    const CsgPackedKeyHash mix;
    uint64_t hash = static_cast<uint64_t>(count) * 0x9E3779B97F4A7C15ULL;
    for (int i = 0; i < count; ++i)
        hash = mix(hash + static_cast<uint32_t>(ids[i]) + 0x9E3779B97F4A7C15ULL);
    return hash;
}

float ClampFloat(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

/// Set the projection from the exact normal of the grid corners of a triangle. Return false if the normal is zero.
/// Then the outputs stay unchanged.
bool SetTriangleProjection(const Mesh& mesh, int triangle, int& dropAxis, bool& flip)
{
    const CsgNormal normal = CsgExact::Normal(mesh.grid_[mesh.triangleIds_[triangle * 3]],
        mesh.grid_[mesh.triangleIds_[triangle * 3 + 1]], mesh.grid_[mesh.triangleIds_[triangle * 3 + 2]]);
    const CsgExactMinor* components[3] = {&normal.x_, &normal.y_, &normal.z_};
    int best = 0;
    for (int axis = 1; axis < 3; ++axis)
    {
        if (CompareMagnitude(*components[axis], *components[best]) > 0)
            best = axis;
    }
    const int sign = components[best]->Sign();
    if (sign == 0)
        return false;
    dropAxis = best;
    flip = sign < 0;
    return true;
}

/// Choose the projection of the triangle to a coordinate plane. Drop the axis of the largest exact normal
/// component. Flip the projection if that component is negative, so the face order is counterclockwise.
/// If the corners of the triangle are collinear on the grid, use the first triangle of the face with a normal
/// that is not zero. If all the triangles of the face are collinear on the grid, drop the z axis.
void ChooseTriangleProjection(const Mesh& mesh, int face, int triangle, int& dropAxis, bool& flip)
{
    dropAxis = 2;
    flip = false;
    if (SetTriangleProjection(mesh, triangle, dropAxis, flip))
        return;
    const auto starts = mesh.solid_.GetTriangleStart();
    for (unsigned other = starts[face]; other < starts[face + 1]; ++other)
    {
        if (static_cast<int>(other) != triangle && SetTriangleProjection(mesh, static_cast<int>(other), dropAxis, flip))
            return;
    }
}

/// Divides one triangle along its curve segments into regions of global ids.
///
/// The regions are traced over directed edges. The boundary pieces run in the face direction. The curve segments run
/// in both directions. Each directed edge belongs to one region. Each closed curve inside the triangle gives an island
/// and a hole. Two bridges join a hole to the region around it. All the memory comes from the scratch.
class TriangleSplitter
{
public:
    TriangleSplitter(const Corefinement& core, const Mesh& mesh, CsgScratch& scratch)
        : core_(core)
        , mesh_(mesh)
        , scratch_(scratch)
        , s_(scratch.split_)
    {
    }

    /// Split the triangle. Return false if the trace of the regions does not close.
    /// The regions are in the scratch: regions_ holds the loops of the pool and regionBridged_ the flags.
    bool Split(int face, int triangle, const CsgSegmentLists& segments)
    {
        Clear();
        triangle_ = triangle;
        for (int i = 0; i < 3; ++i)
        {
            const int from = mesh_.triangleIds_[triangle * 3 + i];
            const int to = mesh_.triangleIds_[triangle * 3 + (i + 1) % 3];
            AddChainId(core_.Global(mesh_, from));
            core_.EdgeCrossings(mesh_, from, to, scratch_, s_.edgePoints_);
            for (const int id : s_.edgePoints_)
                AddChainId(id);
        }
        if (!LinkChain())
        {
            return false;
        }
        SeedWeld();
        for (int node = segments.Head(triangle); node >= 0; node = segments.At(node).next_)
            AddCurve(Weld(segments.At(node).p_), Weld(segments.At(node).q_));
        ChooseProjection(face, triangle);
        if (!TraceAll())
        {
            return false;
        }

        ClassifyClosedCycles();
        s_.regions_ = s_.cycles_;
        s_.regionBridged_.assign(s_.regions_.size(), 0);
        if (!s_.holes_.empty())
        {
            holeCount_ = static_cast<int>(s_.holes_.size());
            if (!PlaceHoles())
            {
                return false;
            }
        }
        return true;
    }

    /// Trace the cycles of a plane graph for the tests. The ids are indices of the points.
    /// The chain is the boundary in the counterclockwise order. Return false if a trace does not close.
    /// The cycles are in the scratch: cycles_ touch the boundary and closedCycles_ do not.
    /// If grid is not null, the ids at one grid point weld as in Split.
    bool TraceForTest(const CsgGridPoint2* points, const CsgGridPoint* grid, const int* chain, int chainCount,
        const int* curveEnds, int curveCount)
    {
        Clear();
        testPoints_ = points;
        testGrid_ = grid;
        for (int i = 0; i < chainCount; ++i)
            AddChainId(chain[i]);
        if (!LinkChain())
            return false;
        if (grid)
            SeedWeld();
        for (int i = 0; i < curveCount; ++i)
        {
            const int p = curveEnds[2 * i];
            const int q = curveEnds[2 * i + 1];
            AddCurve(grid ? Weld(p) : p, grid ? Weld(q) : q);
        }
        return TraceAll();
    }

    /// Number of holes joined by bridges in the last split.
    int HoleCount() const { return holeCount_; }

private:
    using List = CsgAdjacency::List;

    void Clear()
    {
        s_.boundary_.clear();
        s_.boundaryOut_.Clear();
        s_.passes_.Clear();
        s_.curve_.Clear();
        s_.boundaryEdges_.Clear();
        s_.visited_.Clear();
        s_.weld_.Clear();
        s_.pool_.Clear();
        s_.loopAreas_.clear();
        s_.areaTerms_.clear();
        s_.cycles_.clear();
        s_.closedCycles_.clear();
        s_.holes_.clear();
        s_.regions_.clear();
        s_.regionBridged_.clear();
        s_.curveKeys_.Clear();
        s_.curveEnds_.clear();
        failed_ = false;
        holeCount_ = 0;
    }

    /// Give each grid point of the chain the slot of its first chain id. A curve id at a chain point then welds to
    /// the chain id. Without this, a segment from a chain id to a curve id at one grid point stays in the plane map.
    void SeedWeld()
    {
        for (const int id : s_.boundary_)
        {
            const CsgGridPoint& point = GridOf(id);
            CsgVec3Key key;
            key.x_ = static_cast<uint32_t>(point.x_);
            key.y_ = static_cast<uint32_t>(point.y_);
            key.z_ = static_cast<uint32_t>(point.z_);
            bool existed = false;
            int& slot = s_.weld_.FindOrAdd(key, existed);
            if (!existed)
                slot = id;
        }
    }

    /// Return the node of the plane map for an id of a curve segment end. A chain id stays as it is.
    /// Two ids at one grid point with other edges give no consistent order of the edges around the point.
    /// The tie rules of Direction and Turn then trace wrong cycles. One id removes the tie.
    /// A curve id welds to the chain id at its grid point, or to the first curve id seen there.
    /// Two chain ids at one grid point do not weld: the chain keeps its structure.
    int Weld(int id)
    {
        if (ea::find(s_.boundary_.begin(), s_.boundary_.end(), id) != s_.boundary_.end())
            return id;
        const CsgGridPoint& point = GridOf(id);
        CsgVec3Key key;
        key.x_ = static_cast<uint32_t>(point.x_);
        key.y_ = static_cast<uint32_t>(point.y_);
        key.z_ = static_cast<uint32_t>(point.z_);
        bool existed = false;
        int& slot = s_.weld_.FindOrAdd(key, existed);
        if (!existed)
            slot = id;
        // The key keeps 32 bits of each coordinate. A hit with another point keeps its own id.
        else if (!(GridOf(slot) == point))
            return id;
        return slot;
    }

    /// Append an id to the boundary chain. The chain has no two equal ids next to each other.
    void AddChainId(int id)
    {
        if (s_.boundary_.empty() || s_.boundary_.back() != id)
            s_.boundary_.push_back(id);
    }

    /// Link the boundary chain: its directed edges and the passes of the chain through each id.
    /// Return false if the chain has fewer than three ids or the same directed edge twice.
    bool LinkChain()
    {
        while (s_.boundary_.size() > 1 && s_.boundary_.back() == s_.boundary_.front())
            s_.boundary_.pop_back();
        const int boundaryCount = static_cast<int>(s_.boundary_.size());
        if (boundaryCount < 3)
            return false;
        // The chain can visit an id more than once. Each visit is a pass of the chain through the id.
        // A pass has its own sector of the triangle at the id, from the previous id clockwise to the next id.
        for (int i = 0; i < boundaryCount; ++i)
        {
            const int from = s_.boundary_[i];
            const int to = s_.boundary_[(i + 1) % boundaryCount];
            s_.boundaryOut_.Add(from, to);
            s_.passes_.Add(from, i);
            // The same directed edge twice makes two passes with one incoming edge. Such a chain has no plane map.
            if (!s_.boundaryEdges_.Insert(DirectedEdgeKey(from, to)))
                return false;
        }
        return true;
    }

    /// Add a curve segment. The plane map is a simple graph. Two triangle pairs can give the same segment.
    /// A segment can be a piece of the boundary. Such a segment adds no edge. Without this, the trace visits
    /// fewer edges than counted.
    void AddCurve(int p, int q)
    {
        if (p == q || s_.boundaryEdges_.Contains(DirectedEdgeKey(p, q)) || s_.boundaryEdges_.Contains(DirectedEdgeKey(q, p)))
            return;
        if (!s_.curveKeys_.Insert(DirectedEdgeKey(ea::min(p, q), ea::max(p, q))))
            return;
        s_.curve_.Add(p, q);
        s_.curve_.Add(q, p);
        s_.curveEnds_.push_back(p);
        s_.curveEnds_.push_back(q);
    }

    /// Trace the cycles of all the directed edges. Return false if a trace does not close or an edge is left.
    bool TraceAll()
    {
        const int boundaryCount = static_cast<int>(s_.boundary_.size());
        edgeCount_ = static_cast<int>(s_.boundaryEdges_.Size() + s_.curveEnds_.size());
        for (int i = 0; i < boundaryCount; ++i)
            Trace(s_.boundary_[i], s_.boundary_[(i + 1) % boundaryCount]);
        for (unsigned i = 0; i < s_.curveEnds_.size(); i += 2)
        {
            Trace(s_.curveEnds_[i], s_.curveEnds_[i + 1]);
            Trace(s_.curveEnds_[i + 1], s_.curveEnds_[i]);
        }
        return !failed_ && static_cast<int>(s_.visited_.Size()) == edgeCount_;
    }

    /// Grid point of a global id: a vertex of an operand or a crossing.
    const CsgGridPoint& GridOf(int id) const
    {
        if (testGrid_)
            return testGrid_[id];
        if (id >= core_.CrossingBase())
            return core_.CrossingAt(id).grid_;
        return id >= core_.b_.offset_ ? core_.b_.grid_[id - core_.b_.offset_] : core_.a_.grid_[id];
    }

    /// Point of a global id in the plane of the projection.
    CsgGridPoint2 Point(int id) const
    {
        if (testPoints_)
            return testPoints_[id];
        return CsgExact::Project(GridOf(id), dropAxis_, flip_);
    }

    /// Coordinate of a global id on the dropped axis.
    CsgGridInt Dropped(int id) const
    {
        if (testPoints_)
            return 0;
        const CsgGridPoint& p = GridOf(id);
        return dropAxis_ == 0 ? p.x_ : (dropAxis_ == 1 ? p.y_ : p.z_);
    }

    /// Choose the projection of the triangle to a coordinate plane, see ChooseTriangleProjection.
    void ChooseProjection(int face, int triangle) { ChooseTriangleProjection(mesh_, face, triangle, dropAxis_, flip_); }

    /// Direction from one id to another in the plane. It is never zero.
    /// Two ids at one point of the plane get a symbolic direction: the projection is moved by a tiny tilt.
    /// The tilt adds e * w to the first coordinate, with w the coordinate on the dropped axis.
    /// Ids at one grid point get a second, smaller displacement along the second coordinate by the id.
    /// The direction from b to a is the opposite of the direction from a to b, so the turns stay consistent.
    CsgGridPoint2 Direction(int from, int to) const
    {
        const CsgGridPoint2 d = CsgExact::Difference(Point(to), Point(from));
        if (d.u_ != 0 || d.v_ != 0)
            return d;
        const CsgGridInt wFrom = Dropped(from);
        const CsgGridInt wTo = Dropped(to);
        CsgGridPoint2 result;
        if (wFrom != wTo)
            result.u_ = wTo > wFrom ? 1 : -1;
        else
            result.v_ = to > from ? 1 : -1;
        return result;
    }

    /// Copy the points of a loop to the scratch.
    const CsgGridPoint2* LoopPoints(int loop)
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        s_.points_.resize(count);
        for (int i = 0; i < count; ++i)
            s_.points_[i] = Point(ids[i]);
        return s_.points_.data();
    }

    /// Exact sign of the area of a loop. It is positive for the counterclockwise order of the face.
    int AreaSign(int loop) { return CsgExact::LoopAreaSign(LoopPoints(loop), s_.pool_.Count(loop)); }

    /// Return the cache entry of a loop with the magnitude in double. The reference is valid until the next call.
    const CsgLoopArea& ApproximateArea(int loop)
    {
        if (static_cast<int>(s_.loopAreas_.size()) <= loop)
            s_.loopAreas_.resize(s_.pool_.spans_.size());
        if (s_.loopAreas_[loop].approximate_)
            return s_.loopAreas_[loop];

        const CsgGridPoint2* points = LoopPoints(loop);
        const int count = s_.pool_.Count(loop);
        double area = 0.0;
        // The sum of the magnitudes of all the products.
        double magnitude = 0.0;
        for (int i = 1; i + 1 < count; ++i)
        {
            const CsgGridPoint2 p = CsgExact::Difference(points[i], points[0]);
            const CsgGridPoint2 q = CsgExact::Difference(points[i + 1], points[0]);
            const double pq = static_cast<double>(p.u_) * q.v_;
            const double qp = static_cast<double>(p.v_) * q.u_;
            area += pq - qp;
            magnitude += std::fabs(pq) + std::fabs(qp);
        }
        CsgLoopArea& entry = s_.loopAreas_[loop];
        entry.magnitude_ = std::fabs(area);
        // Each product is exact if it has at most 53 bits. If the sum of all magnitudes is below 2^53, each partial
        // sum is an exact integer and the error is zero. Otherwise, the products and the sums round with a relative
        // error of at most 2^-53 of the magnitudes. The terms n + 4 and the unit 2^-52 are a safe upper bound.
        if (CsgPlanarExactInDouble && magnitude < 9007199254740992.0)
            entry.error_ = 0.0;
        else
            entry.error_ = (count + 4) * 2.220446049250313e-16 * magnitude;
        entry.approximate_ = true;
        return entry;
    }

    /// Return the sign of |area of loop a| - |area of loop b|. The result is exact and does not depend on the rounding.
    /// The caches hold the double areas and the exact expansions. They live as long as the pool.
    /// The choice of the smallest region that contains a hole and the order of the holes use it, so it can change
    /// the topology.
    int CompareAreaMagnitude(int a, int b)
    {
        if (a == b)
            return 0;
        // Copy the values. The next call can move the cache.
        const CsgLoopArea entryA = ApproximateArea(a);
        const CsgLoopArea entryB = ApproximateArea(b);
        const double limit = entryA.error_ + entryB.error_;
        // The differences of the values are exact if the errors are zero.
        if (entryA.magnitude_ > entryB.magnitude_ + limit)
            return 1;
        if (entryB.magnitude_ > entryA.magnitude_ + limit)
            return -1;
        if (limit == 0.0)
            return 0;

        const int startA = ExactArea(a);
        const int startB = ExactArea(b);
        // A loop that is too long has no exact area. Use the order of the double magnitudes.
        if (s_.loopAreas_[a].termCount_ < 0 || s_.loopAreas_[b].termCount_ < 0)
            return (entryA.magnitude_ > entryB.magnitude_) - (entryA.magnitude_ < entryB.magnitude_);
        return CsgExact::CompareLoopAreaMagnitude(
            s_.loopAreas_[a].termCount_, s_.areaTerms_.data() + startA, s_.loopAreas_[b].termCount_,
            s_.areaTerms_.data() + startB);
    }

    /// Compute the exact expansion of the area of a loop if the cache has none. Return the start of its terms.
    int ExactArea(int loop)
    {
        if (s_.loopAreas_[loop].termStart_ >= 0)
            return s_.loopAreas_[loop].termStart_;
        const CsgGridPoint2* points = LoopPoints(loop);
        CsgReal terms[CsgLoopAreaMaxTerms];
        const int termCount = CsgExact::LoopAreaExact(points, s_.pool_.Count(loop), terms);
        const int start = static_cast<int>(s_.areaTerms_.size());
        // The count -1 means that the loop is too long for the expansion. The cache keeps the count.
        if (termCount > 0)
            s_.areaTerms_.insert(s_.areaTerms_.end(), terms, terms + termCount);
        s_.loopAreas_[loop].termStart_ = start;
        s_.loopAreas_[loop].termCount_ = termCount;
        return start;
    }

    void Trace(int from, int to)
    {
        if (s_.visited_.Contains(DirectedEdgeKey(from, to)))
            return;
        s_.cycle_.clear();
        bool touchesBoundary = false;
        int u = from;
        int v = to;
        while (s_.visited_.Insert(DirectedEdgeKey(u, v)))
        {
            s_.cycle_.push_back(u);
            touchesBoundary |= s_.boundaryEdges_.Contains(DirectedEdgeKey(u, v));
            const int w = Next(u, v);
            if (w < 0)
            {
                failed_ = true;
                break;
            }
            u = v;
            v = w;
            if (static_cast<int>(s_.cycle_.size()) > edgeCount_)
                break;
        }
        // Next is a bijection, so a trace comes back to its first edge. Anything else is an error in the order.
        if (!failed_ && (u != from || v != to))
        {
            failed_ = true;
        }
        RemoveSpurs(s_.cycle_);
        if (s_.cycle_.size() < 3)
            return;
        AddCycle(s_.cycle_.data(), static_cast<int>(s_.cycle_.size()), touchesBoundary);
    }

    /// Add a traced cycle to the loops. A cycle that passes twice through one id can be two lobes side by side.
    /// Such a cycle splits at the id when no lobe has a negative area. Each lobe can then lie in a different
    /// patch of the classification. A lobe with a negative area is a hole that touches the cycle, so that
    /// cycle stays one loop. The test of the area sign is exact.
    void AddCycle(const int* ids, int count, bool touchesBoundary)
    {
        for (int i = 0; i < count; ++i)
        {
            for (int j = i + 1; j < count; ++j)
            {
                if (ids[i] != ids[j])
                    continue;
                // The lobes are ids [i, j) and ids [j, count) + [0, i).
                ea::vector<int> first(ids + i, ids + j);
                ea::vector<int> second(ids + j, ids + count);
                second.insert(second.end(), ids, ids + i);
                if (first.size() < 3 || second.size() < 3 || AreaSignOf(first.data(), static_cast<int>(first.size())) < 0
                    || AreaSignOf(second.data(), static_cast<int>(second.size())) < 0)
                {
                    CSG_DIAG_COUNT(lobeHoleKept_);
                    continue;
                }
                CSG_DIAG_COUNT(lobeSplits_);
                AddCycle(first.data(), static_cast<int>(first.size()), touchesBoundary);
                AddCycle(second.data(), static_cast<int>(second.size()), touchesBoundary);
                return;
            }
        }
        const int loop = s_.pool_.Add(ids, count);
        (touchesBoundary ? s_.cycles_ : s_.closedCycles_).push_back(loop);
    }

    /// Exact sign of the area of a loop of ids.
    int AreaSignOf(const int* ids, int count)
    {
        s_.points_.resize(count);
        for (int i = 0; i < count; ++i)
            s_.points_[i] = Point(ids[i]);
        return CsgExact::LoopAreaSign(s_.points_.data(), count);
    }

    /// Remove the spurs of a closed cycle. A spur is a walk x -> y -> x. It has no area, and its two directed edges
    /// cancel. A chain that goes to an id and back, or a curve with a free end, gives a spur. The removal is
    /// combinatorial and exact. The cycle can become shorter than three ids. Then it has no area.
    static void RemoveSpurs(ea::vector<int>& cycle)
    {
        int size = 0;
        for (const int id : cycle)
        {
            if (size >= 2 && cycle[size - 2] == id)
                --size;
            else
                cycle[size++] = id;
        }
        // The cycle closes, so a spur can go over the end of the vector.
        int start = 0;
        bool changed = true;
        while (changed && size - start >= 3)
        {
            changed = false;
            if (cycle[size - 1] == cycle[start])
            {
                --size;
                changed = true;
            }
            else if (cycle[size - 2] == cycle[start])
            {
                size -= 2;
                changed = true;
            }
            else if (cycle[size - 1] == cycle[start + 1])
            {
                start += 1;
                --size;
                changed = true;
            }
        }
        if (start > 0)
            ea::copy(cycle.begin() + start, cycle.begin() + size, cycle.begin());
        cycle.resize(size - start);
    }

    /// The edge that follows u -> v around the region on its left. Return -1 if there is none.
    int Next(int u, int v)
    {
        const List* branches = s_.curve_.Find(v);
        const List* passes = s_.passes_.Find(v);
        if (passes)
        {
            if (passes->count_ == 1)
            {
                const int out = ChainNext(s_.passes_.At(passes->head_).to_);
                // One pass and at most one curve: the curve lies between the incoming and the outgoing boundary edge.
                if (!branches)
                    return out;
                if (branches->count_ == 1)
                    return s_.boundaryEdges_.Contains(DirectedEdgeKey(u, v)) ? s_.curve_.At(branches->head_).to_ : out;
            }
            return SectorTurn(u, v, passes, branches);
        }
        if (!branches)
            return -1;
        if (branches->count_ == 2)
        {
            const int first = s_.curve_.At(branches->head_).to_;
            return first == u ? s_.curve_.At(s_.curve_.At(branches->head_).next_).to_ : first;
        }
        return Turn(u, v, branches);
    }

    /// Id before the chain index in the boundary chain.
    int ChainPrevious(int index) const
    {
        const int count = static_cast<int>(s_.boundary_.size());
        return s_.boundary_[(index + count - 1) % count];
    }

    /// Id after the chain index in the boundary chain.
    int ChainNext(int index) const
    {
        const int count = static_cast<int>(s_.boundary_.size());
        return s_.boundary_[(index + 1) % count];
    }

    /// Among the curve directions at an inner vertex v, the first one clockwise from the way back to u.
    ///
    /// The directions around v have a cyclic order: the clockwise angle, then the global id for equal directions.
    /// The result is the next direction after v -> u in this order. The order is exact, so Next is a bijection.
    int Turn(int u, int v, const List* branches)
    {
        const CsgGridPoint2 zero;
        const CsgGridPoint2 back = Direction(v, u);
        int best = -1;
        int bestGroup = 0;
        CsgGridPoint2 bestDirection;
        for (int node = branches->head_; node >= 0; node = s_.curve_.At(node).next_)
        {
            const int w = s_.curve_.At(node).to_;
            if (w == u && branches->count_ > 1)
                continue;
            const CsgGridPoint2 direction = Direction(v, w);
            // Group 0: the direction of v -> u with a higher id. Group 2: the same with a lower id.
            const bool alongBack = CsgExact::Cross2(back, direction) == 0 && CsgExact::Dot2(back, direction) > 0;
            const int group = alongBack ? (w > u ? 0 : 2) : 1;
            bool better = best < 0 || group < bestGroup;
            if (best >= 0 && group == bestGroup)
            {
                const int order = group == 1 ? CsgExact::DirectionCompare(zero, back, direction, bestDirection) : 0;
                better = order < 0 || (order == 0 && w < best);
            }
            if (better)
            {
                best = w;
                bestGroup = group;
                bestDirection = direction;
            }
        }
        return best;
    }

    /// Place of a directed edge from a boundary vertex in the cyclic order of the edges around the vertex.
    ///
    /// The order goes through the passes of the chain in the chain order. The order in a pass k has five parts.
    /// First comes the back edge to the previous id (phase 0), and then the curves on that ray.
    /// Then come the curves inside the sector (phase 1), by the clockwise angle from the back edge.
    /// Then come the curves on the ray of the next id (phase 2). The outgoing edge is last.
    /// A gap curve goes into phase 0 or phase 2 of a pass. Equal places in a phase compare by the rank.
    /// The rank of a curve is its global id.
    struct Slot
    {
        int pass_{};
        int phase_{};
        int64_t rank_{};
        CsgGridPoint2 back_;
        CsgGridPoint2 direction_;
    };

    /// Return -1, 0 or 1 as a comes before b, at the same place or after b.
    /// The cyclic order starts at the first pass.
    static int CompareSlots(const Slot& a, const Slot& b)
    {
        if (a.pass_ != b.pass_)
            return a.pass_ < b.pass_ ? -1 : 1;
        if (a.phase_ != b.phase_)
            return a.phase_ < b.phase_ ? -1 : 1;
        if (a.phase_ == 1)
        {
            const int order = CsgExact::DirectionCompare(CsgGridPoint2{}, a.back_, a.direction_, b.direction_);
            if (order != 0)
                return order;
        }
        return a.rank_ < b.rank_ ? -1 : (a.rank_ > b.rank_ ? 1 : 0);
    }

    /// Back edge and outgoing edge of a pass at v, as directions from v.
    void PassRays(int v, int index, CsgGridPoint2& back, CsgGridPoint2& out) const
    {
        back = Direction(v, ChainPrevious(index));
        out = Direction(v, ChainNext(index));
    }

    /// Place of the curve v -> w at the boundary vertex v.
    ///
    /// The curve goes to the first pass whose sector holds its direction.
    /// A rounded crossing can move the curve out of all the sectors.
    /// The curve is then in a gap after the outgoing edge of a pass k and before the back edge of the next pass j.
    /// The curve goes to the end of pass k if it lies beyond the outgoing edge only.
    /// Else it goes to the start of pass j.
    /// All tests are exact, so each curve has one place, and Next is a bijection.
    Slot CurveSlot(int v, int w, const List* passes) const
    {
        const CsgGridPoint2 zero;
        Slot slot;
        slot.rank_ = w;
        slot.direction_ = Direction(v, w);
        const CsgGridPoint2& d = slot.direction_;
        int pass = 0;
        for (int node = passes->head_; node >= 0; node = s_.passes_.At(node).next_, ++pass)
        {
            const int index = s_.passes_.At(node).to_;
            CsgGridPoint2 back, out;
            PassRays(v, index, back, out);
            slot.pass_ = pass;
            slot.back_ = back;
            // A pass that comes back to the id it came from has the full turn as its sector.
            if (ChainPrevious(index) == ChainNext(index))
            {
                slot.phase_ = 1;
                return slot;
            }
            if (CsgExact::DirectionCompare(zero, back, d, out) <= 0)
            {
                slot.phase_ = OnRay(back, d) ? 0 : (OnRay(out, d) ? 2 : 1);
                return slot;
            }
        }
        // The curve is in a gap. Find the pass k with the gap clockwise from its outgoing edge.
        const int count = passes->count_;
        pass = 0;
        for (int node = passes->head_; node >= 0; node = s_.passes_.At(node).next_, ++pass)
        {
            const int index = s_.passes_.At(node).to_;
            const int nextNode = s_.passes_.At(node).next_ >= 0 ? s_.passes_.At(node).next_ : passes->head_;
            const int nextIndex = s_.passes_.At(nextNode).to_;
            CsgGridPoint2 back, out, nextBack, nextOut;
            PassRays(v, index, back, out);
            PassRays(v, nextIndex, nextBack, nextOut);
            if (count > 1 && CsgExact::DirectionCompare(zero, out, d, nextBack) >= 0)
                continue;
            // Beyond the outgoing edge: w is on the right of v -> next. Beyond the back edge: w is on the right of
            // previous -> v. A curve beyond the outgoing edge only stays with pass k.
            const bool outBeyond = CsgExact::Cross2(out, d) < 0;
            const bool inBeyond = CsgExact::Cross2(nextBack, d) > 0;
            if (outBeyond && !inBeyond)
            {
                slot.pass_ = pass;
                slot.phase_ = 2;
                slot.back_ = back;
            }
            else
            {
                slot.pass_ = (pass + 1) % count;
                slot.phase_ = 0;
                slot.back_ = nextBack;
            }
            return slot;
        }
        // The passes do not split the turn in order. Put the curve at the start of the first pass.
        slot.pass_ = 0;
        slot.phase_ = 0;
        return slot;
    }

    /// Whether the direction d is on the ray of the direction r.
    static bool OnRay(const CsgGridPoint2& r, const CsgGridPoint2& d)
    {
        return CsgExact::Cross2(r, d) == 0 && CsgExact::Dot2(r, d) > 0;
    }

    /// The edge that follows u -> v at a boundary vertex v is the outgoing slot after the slot of v -> u.
    /// The cyclic order gives the slot. The back edges are the only slots that are not outgoing.
    /// No back edge follows a slot of an incoming edge. So each incoming edge gets a different outgoing edge.
    int SectorTurn(int u, int v, const List* passes, const List* branches)
    {
        Slot from;
        if (s_.boundaryEdges_.Contains(DirectedEdgeKey(u, v)))
        {
            int pass = 0;
            for (int node = passes->head_; node >= 0; node = s_.passes_.At(node).next_, ++pass)
            {
                if (ChainPrevious(s_.passes_.At(node).to_) == u)
                    break;
            }
            from.pass_ = pass;
            from.phase_ = 0;
            from.rank_ = std::numeric_limits<int64_t>::min();
        }
        else
            from = CurveSlot(v, u, passes);

        int best = -1;
        int first = -1;
        Slot bestSlot;
        Slot firstSlot;
        const auto consider = [&](int w, const Slot& slot)
        {
            if (first < 0 || CompareSlots(slot, firstSlot) < 0)
            {
                first = w;
                firstSlot = slot;
            }
            if (CompareSlots(from, slot) < 0 && (best < 0 || CompareSlots(slot, bestSlot) < 0))
            {
                best = w;
                bestSlot = slot;
            }
        };
        int pass = 0;
        for (int node = passes->head_; node >= 0; node = s_.passes_.At(node).next_, ++pass)
        {
            Slot slot;
            slot.pass_ = pass;
            slot.phase_ = 2;
            slot.rank_ = std::numeric_limits<int64_t>::max();
            consider(ChainNext(s_.passes_.At(node).to_), slot);
        }
        if (branches)
        {
            for (int node = branches->head_; node >= 0; node = s_.curve_.At(node).next_)
            {
                const int w = s_.curve_.At(node).to_;
                consider(w, CurveSlot(v, w, passes));
            }
        }
        return best >= 0 ? best : first;
    }

    /// A closed curve inside the triangle is traced once in each direction. The turn that encloses more area is the
    /// island and the other is the hole around it, even when rounding gives both the same sign.
    /// Move the closed cycles of the curve network that is connected to the triangle boundary to the regions.
    /// Such a cycle is an inner face of that network, so it is never a hole, even if its area sign is zero or
    /// negative on the grid. Only a curve network apart from the boundary has an outer face, which is a hole.
    /// The faces of one network share vertices, so the connection spreads from cycle to cycle by the shared ids.
    void KeepDetachedClosedCycles()
    {
        s_.attachedIds_.clear();
        for (const int loop : s_.cycles_)
            s_.attachedIds_.insert(s_.attachedIds_.end(), s_.pool_.Data(loop), s_.pool_.Data(loop) + s_.pool_.Count(loop));
        ea::sort(s_.attachedIds_.begin(), s_.attachedIds_.end());
        bool changed = true;
        while (changed)
        {
            changed = false;
            s_.attachedNew_.clear();
            int kept = 0;
            for (const int loop : s_.closedCycles_)
            {
                const int* ids = s_.pool_.Data(loop);
                const int count = s_.pool_.Count(loop);
                bool attached = false;
                for (int i = 0; i < count && !attached; ++i)
                    attached = ea::binary_search(s_.attachedIds_.begin(), s_.attachedIds_.end(), ids[i]);
                if (!attached)
                {
                    s_.closedCycles_[kept++] = loop;
                    continue;
                }
                s_.cycles_.push_back(loop);
                s_.attachedNew_.insert(s_.attachedNew_.end(), ids, ids + count);
                changed = true;
            }
            s_.closedCycles_.resize(kept);
            if (changed)
            {
                s_.attachedIds_.insert(s_.attachedIds_.end(), s_.attachedNew_.begin(), s_.attachedNew_.end());
                ea::sort(s_.attachedIds_.begin(), s_.attachedIds_.end());
            }
        }
    }

    /// The two turns of a curve of zero size have the same exact sign 0. Then the first turn is the outer cycle.
    void ClassifyClosedCycles()
    {
        if (s_.closedCycles_.empty())
            return;
        KeepDetachedClosedCycles();
        const int closedCount = static_cast<int>(s_.closedCycles_.size());
        if (closedCount == 0)
            return;
        s_.closedSorted_.clear();
        s_.closedSortedStart_.clear();
        s_.closedHash_.clear();
        s_.groupHead_.clear();
        s_.groupTail_.clear();
        s_.groupSameHash_.clear();
        s_.groupByHash_.Clear();
        s_.memberNext_.assign(closedCount, -1);
        // The groups keep the order of their first member. A group holds the cycles with the same sorted ids.
        for (int k = 0; k < closedCount; ++k)
        {
            const int loop = s_.closedCycles_[k];
            const int count = s_.pool_.Count(loop);
            const int start = static_cast<int>(s_.closedSorted_.size());
            s_.closedSortedStart_.push_back(start);
            s_.closedSorted_.insert(s_.closedSorted_.end(), s_.pool_.Data(loop), s_.pool_.Data(loop) + count);
            ea::sort(s_.closedSorted_.begin() + start, s_.closedSorted_.end());
            const uint64_t hash = HashSortedIds(s_.closedSorted_.data() + start, count);
            s_.closedHash_.push_back(hash);

            bool existed = false;
            int& chainHead = s_.groupByHash_.FindOrAdd(hash, existed);
            int found = -1;
            int last = -1;
            if (existed)
            {
                for (int g = chainHead; g >= 0 && found < 0; g = s_.groupSameHash_[g])
                {
                    last = g;
                    const int representative = s_.groupHead_[g];
                    const int representativeCount = s_.pool_.Count(s_.closedCycles_[representative]);
                    if (representativeCount == count
                        && ea::equal(s_.closedSorted_.begin() + start, s_.closedSorted_.begin() + start + count,
                            s_.closedSorted_.begin() + s_.closedSortedStart_[representative]))
                        found = g;
                }
            }
            if (found >= 0)
            {
                s_.memberNext_[s_.groupTail_[found]] = k;
                s_.groupTail_[found] = k;
                continue;
            }
            const int group = static_cast<int>(s_.groupHead_.size());
            s_.groupHead_.push_back(k);
            s_.groupTail_.push_back(k);
            s_.groupSameHash_.push_back(-1);
            if (existed)
                s_.groupSameHash_[last] = group;
            else
                chainHead = group;
        }

        for (unsigned g = 0; g < s_.groupHead_.size(); ++g)
        {
            const int head = s_.groupHead_[g];
            const int second = s_.memberNext_[head];
            if (second >= 0 && s_.memberNext_[second] < 0)
            {
                const int m0 = s_.closedCycles_[head];
                const int m1 = s_.closedCycles_[second];
                // The exact signs decide. Equal signs mean a curve of zero size: the first member is the outer cycle.
                const int area0 = AreaSign(m0);
                const int area1 = AreaSign(m1);
                if (area0 >= area1)
                {
                    s_.cycles_.push_back(m0);
                    s_.holes_.push_back(m1);
                }
                else
                {
                    s_.cycles_.push_back(m1);
                    s_.holes_.push_back(m0);
                }
                continue;
            }
            for (int k = head; k >= 0; k = s_.memberNext_[k])
            {
                const int loop = s_.closedCycles_[k];
                (AreaSign(loop) > 0 ? s_.cycles_ : s_.holes_).push_back(loop);
            }
        }
    }

    // ---- Holes. The tests are exact on the grid points of the projection.

    /// Whether the loop contains the point. A point on an edge of the loop counts as inside.
    /// The parity test uses the half open rule on the second coordinate.
    bool Contains(int loop, const CsgGridPoint2& point) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        bool inside = false;
        for (int i = 0, j = count - 1; i < count; j = i++)
        {
            const CsgGridPoint2 p = Point(ids[i]);
            const CsgGridPoint2 q = Point(ids[j]);
            const int side = CsgExact::Orient2(q, p, point);
            if (side == 0 && ea::min(p.u_, q.u_) <= point.u_ && point.u_ <= ea::max(p.u_, q.u_)
                && ea::min(p.v_, q.v_) <= point.v_ && point.v_ <= ea::max(p.v_, q.v_))
                return true;
            if ((p.v_ > point.v_) != (q.v_ > point.v_))
            {
                // The edge goes up from q to p, or down. The ray to the plus side of u crosses it if the point
                // lies to the left of the upward edge.
                if ((p.v_ > q.v_) ? side > 0 : side < 0)
                    inside = !inside;
            }
        }
        return inside;
    }

    /// Whether the loop blocks the bridge from-to. The edges at the end ids do not block it.
    bool Blocked(int loop, const CsgGridPoint2& from, const CsgGridPoint2& to, int endA, int endB) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        for (int i = 0; i < count; ++i)
        {
            const int p = ids[i];
            const int q = ids[(i + 1) % count];
            if (p == endA || p == endB || q == endA || q == endB)
                continue;
            if (BridgeBlockedBy(from, to, Point(p), Point(q)))
                return true;
        }
        return false;
    }

    /// Whether the direction from the vertex index of the loop to the target point lies on the left of the loop.
    /// The left side is the inside of a region and the outside of a hole. The neighbors are the nearest loop
    /// vertices at other points. A zero direction, or a loop without other points, counts as on the left.
    bool OnLeft(int loop, int index, const CsgGridPoint2& target) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        const CsgGridPoint2 v = Point(ids[index]);
        const CsgGridPoint2 d = CsgExact::Difference(target, v);
        if (d.u_ == 0 && d.v_ == 0)
            return true;
        int prev = -1;
        int next = -1;
        for (int k = 1; k < count && prev < 0; ++k)
        {
            if (Point(ids[(index - k + count) % count]) != v)
                prev = (index - k + count) % count;
        }
        for (int k = 1; k < count && next < 0; ++k)
        {
            if (Point(ids[(index + k) % count]) != v)
                next = (index + k) % count;
        }
        if (prev < 0 || next < 0)
            return true;
        const CsgGridPoint2 e0 = CsgExact::Difference(Point(ids[prev]), v);
        const CsgGridPoint2 e1 = CsgExact::Difference(Point(ids[next]), v);
        // The left side runs counterclockwise from e1 to e0.
        if (CsgExact::Cross2(e1, e0) > 0)
            return CsgExact::Cross2(e1, d) > 0 && CsgExact::Cross2(d, e0) > 0;
        return !(CsgExact::Cross2(e0, d) >= 0 && CsgExact::Cross2(d, e1) >= 0);
    }

    /// Index of the nearest vertex of the loop after or before the vertex index at another point. Return -1 if none.
    int OtherPoint(int loop, int index, int step) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        const CsgGridPoint2 v = Point(ids[index]);
        for (int k = 1; k < count; ++k)
        {
            const int other = ((index + step * k) % count + count) % count;
            if (Point(ids[other]) != v)
                return other;
        }
        return -1;
    }

    /// Whether the counterclockwise angle from the reference to a is less than the angle to b. Angles are in [0, 2 pi).
    static bool CounterclockwiseLess(const CsgGridPoint2& reference, const CsgGridPoint2& a, const CsgGridPoint2& b)
    {
        const auto half = [&reference](const CsgGridPoint2& d)
        {
            const int cross = CsgExact::Cross2(reference, d);
            return cross > 0 || (cross == 0 && CsgExact::Dot2(reference, d) > 0) ? 0 : 1;
        };
        const int ha = half(a);
        const int hb = half(b);
        if (ha != hb)
            return ha < hb;
        return CsgExact::Cross2(a, b) > 0;
    }

    /// Whether the point is on the boundary of the loop: at a vertex or on an edge.
    bool OnBoundary(int loop, const CsgGridPoint2& point) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        for (int i = 0; i < count; ++i)
        {
            const CsgGridPoint2 p = Point(ids[i]);
            const CsgGridPoint2 q = Point(ids[(i + 1) % count]);
            if (p == point
                || (CsgExact::Orient2(p, q, point) == 0 && ea::min(p.u_, q.u_) <= point.u_ && point.u_ <= ea::max(p.u_, q.u_)
                    && ea::min(p.v_, q.v_) <= point.v_ && point.v_ <= ea::max(p.v_, q.v_)))
                return true;
        }
        return false;
    }

    /// Whether the hole lies inside the region. A hole vertex off the boundary of the region decides by parity.
    /// A hole can share its vertices and edges with the region, because ids at one grid point are not merged.
    /// If every hole vertex is on the boundary of the region, the side of the region next to the hole decides.
    bool HoleInside(int region, int hole) const
    {
        const int* holeIds = s_.pool_.Data(hole);
        const int holeSize = s_.pool_.Count(hole);
        for (int k = 0; k < holeSize; ++k)
        {
            const CsgGridPoint2 v = Point(holeIds[k]);
            if (!OnBoundary(region, v))
                return Contains(region, v);
        }
        return ContainsBesideHole(region, hole, 0);
    }

    /// Whether the region contains the points just outside the hole at its vertex index.
    /// The test point is the hole vertex v, moved a tiny step into the outside of the hole. The step goes in the
    /// direction d of the next hole edge, turned a tiny angle counterclockwise. That is the left side of the hole.
    /// The test is exact. If v is on the boundary of the region, the side of the region at v decides.
    bool ContainsBesideHole(int region, int hole, int index) const
    {
        const CsgGridPoint2 v = Point(s_.pool_.Data(hole)[index]);
        const int next = OtherPoint(hole, index, 1);
        if (next < 0)
            return Contains(region, v);
        const CsgGridPoint2 d = CsgExact::Difference(Point(s_.pool_.Data(hole)[next]), v);
        const int* ids = s_.pool_.Data(region);
        const int count = s_.pool_.Count(region);
        bool touches = false;
        for (int i = 0; i < count; ++i)
        {
            const CsgGridPoint2 p = Point(ids[i]);
            if (p == v)
            {
                // The inside of the region at this vertex runs counterclockwise from the next edge to the previous
                // edge. Equal edge directions make a full turn.
                touches = true;
                const int before = OtherPoint(region, i, -1);
                const int after = OtherPoint(region, i, 1);
                if (before < 0 || after < 0)
                    continue;
                const CsgGridPoint2 e0 = CsgExact::Difference(Point(ids[before]), v);
                const CsgGridPoint2 e1 = CsgExact::Difference(Point(ids[after]), v);
                const bool full = CsgExact::Cross2(e1, e0) == 0 && CsgExact::Dot2(e1, e0) > 0;
                if (full || CounterclockwiseLess(e1, d, e0))
                    return true;
                continue;
            }
            const CsgGridPoint2 q = Point(ids[(i + 1) % count]);
            if (q == v || CsgExact::Orient2(p, q, v) != 0 || ea::min(p.u_, q.u_) > v.u_ || v.u_ > ea::max(p.u_, q.u_)
                || ea::min(p.v_, q.v_) > v.v_ || v.v_ > ea::max(p.v_, q.v_))
                continue;
            // The point v is inside the edge p-q. The inside of the region is on the left of the edge.
            touches = true;
            const CsgGridPoint2 edge = CsgExact::Difference(q, p);
            const int cross = CsgExact::Cross2(edge, d);
            if (cross > 0 || (cross == 0 && CsgExact::Dot2(edge, d) > 0))
                return true;
        }
        return !touches && Contains(region, v);
    }

    /// Index pair of the closest vertices of the loops.
    ea::pair<int, int> Nearest(int outer, int hole) const
    {
        const int* outerIds = s_.pool_.Data(outer);
        const int* holeIds = s_.pool_.Data(hole);
        const CsgGridPoint2 zero;
        ea::pair<int, int> best(0, 0);
        CsgGridPoint2 bestDifference = CsgExact::Difference(Point(holeIds[0]), Point(outerIds[0]));
        for (int i = 0; i < s_.pool_.Count(outer); ++i)
        {
            for (int j = 0; j < s_.pool_.Count(hole); ++j)
            {
                const CsgGridPoint2 difference = CsgExact::Difference(Point(holeIds[j]), Point(outerIds[i]));
                if (CsgExact::SquaredDistanceCompare(zero, difference, bestDifference) < 0)
                {
                    bestDifference = difference;
                    best = ea::make_pair(i, j);
                }
            }
        }
        return best;
    }

    /// Return the vertex of the outer loop that the vertex from of the hole sees, nearest first. Return -1 if none.
    int Visible(int outer, int hole, int from, int avoidOuter, int avoidHole)
    {
        const int* outerIds = s_.pool_.Data(outer);
        const int outerCount = s_.pool_.Count(outer);
        const int* holeIds = s_.pool_.Data(hole);
        const CsgGridPoint2 start = Point(holeIds[from]);
        s_.order_.resize(outerCount);
        s_.points_.resize(outerCount);
        for (int i = 0; i < outerCount; ++i)
        {
            s_.order_[i] = i;
            s_.points_[i] = Point(outerIds[i]);
        }
        const ea::vector<CsgGridPoint2>& points = s_.points_;
        s_.merge_.resize(outerCount);
        ea::merge_sort_buffer(s_.order_.begin(), s_.order_.end(), s_.merge_.data(),
            [&points, &start](int x, int y) { return CsgExact::SquaredDistanceCompare(start, points[x], points[y]) < 0; });
        for (const int candidate : s_.order_)
        {
            if (candidate == avoidOuter)
                continue;
            const CsgGridPoint2 target = Point(outerIds[candidate]);
            // Holes not yet joined also lie in the region, so a bridge must not cut through them either.
            bool blocked = Blocked(outer, start, target, outerIds[candidate], holeIds[from])
                || Blocked(hole, start, target, outerIds[candidate], holeIds[from]);
            for (int other = pendingFrom_; !blocked && other < static_cast<int>(s_.holes_.size()); ++other)
                blocked = Blocked(s_.holes_[other], start, target, outerIds[candidate], holeIds[from]);
            if (blocked)
                continue;
            if (avoidOuter >= 0
                && BridgeBlockedBy(start, target, Point(holeIds[avoidHole]), Point(outerIds[avoidOuter])))
                continue;
            // The bridge does not cross the loops. It lies inside the region and outside the hole if it leaves
            // both ends on the left side of the loops.
            if (!OnLeft(outer, candidate, start) || !OnLeft(hole, from, target))
                continue;
            return candidate;
        }
        return -1;
    }

    /// Two bridges from the extreme hole vertices along the first axis to visible region vertices.
    /// On success the two new loops are in temp1_ and temp2_.
    bool Bridge(int outer, int hole)
    {
        const int* outerIds = s_.pool_.Data(outer);
        const int outerCount = s_.pool_.Count(outer);
        const int* holeIds = s_.pool_.Data(hole);
        const int holeSize = s_.pool_.Count(hole);
        int low = 0;
        int high = 0;
        for (int i = 1; i < holeSize; ++i)
        {
            const CsgGridInt x = Point(holeIds[i]).u_;
            if (x < Point(holeIds[low]).u_)
                low = i;
            if (x > Point(holeIds[high]).u_)
                high = i;
        }
        if (low == high)
            return false;
        const int lowTarget = Visible(outer, hole, low, -1, -1);
        if (lowTarget < 0)
            return false;
        const int highTarget = Visible(outer, hole, high, lowTarget, low);
        if (highTarget < 0)
            return false;

        // The targets and the indices are valid, so each walk below ends at its target.
        s_.temp1_.clear();
        for (int k = lowTarget;; k = (k + 1) % outerCount)
        {
            s_.temp1_.push_back(outerIds[k]);
            if (k == highTarget)
                break;
        }
        for (int k = high;; k = (k + 1) % holeSize)
        {
            s_.temp1_.push_back(holeIds[k]);
            if (k == low)
                break;
        }
        s_.temp2_.clear();
        for (int k = highTarget;; k = (k + 1) % outerCount)
        {
            s_.temp2_.push_back(outerIds[k]);
            if (k == lowTarget)
                break;
        }
        for (int k = low;; k = (k + 1) % holeSize)
        {
            s_.temp2_.push_back(holeIds[k]);
            if (k == high)
                break;
        }
        return true;
    }

    /// Return true if the sorted unique ids of the loop equal the twin set.
    bool SameSetAsTwin(const int* ids, int count)
    {
        s_.twinTest_.assign(ids, ids + count);
        ea::sort(s_.twinTest_.begin(), s_.twinTest_.end());
        s_.twinTest_.erase(ea::unique(s_.twinTest_.begin(), s_.twinTest_.end()), s_.twinTest_.end());
        return s_.twinTest_ == s_.twin_;
    }

    /// Join each hole to the region around it with two bridges. If no pair of bridges is clear, one bridge makes a
    /// region that visits it twice. Return false if there is no region to join.
    bool PlaceHoles()
    {
        if (s_.regions_.empty())
            return false;

        // Outer holes first, so a hole inside an island is placed after the island exists. The sort is stable.
        // It orders the holes by the exact area. The larger area comes first.
        const int holeTotal = static_cast<int>(s_.holes_.size());
        s_.order_.resize(holeTotal);
        for (int i = 0; i < holeTotal; ++i)
            s_.order_[i] = i;
        {
            const ea::vector<int>& holes = s_.holes_;
            s_.merge_.resize(holeTotal);
            ea::merge_sort_buffer(s_.order_.begin(), s_.order_.end(), s_.merge_.data(),
                [this, &holes](int x, int y) { return CompareAreaMagnitude(holes[y], holes[x]) < 0; });
        }
        s_.temp1_.resize(holeTotal);
        for (int i = 0; i < holeTotal; ++i)
            s_.temp1_[i] = s_.holes_[s_.order_[i]];
        s_.holes_ = s_.temp1_;

        for (int h = 0; h < holeTotal; ++h)
        {
            // The holes after this one are the pending holes of the C# reference. Their order does not matter.
            pendingFrom_ = h + 1;
            const int hole = s_.holes_[h];
            const int* holeIds = s_.pool_.Data(hole);
            const int holeSize = s_.pool_.Count(hole);
            // The island traced from the same curve has the same vertices. The hole never belongs inside it.
            s_.twin_.assign(holeIds, holeIds + holeSize);
            ea::sort(s_.twin_.begin(), s_.twin_.end());
            s_.twin_.erase(ea::unique(s_.twin_.begin(), s_.twin_.end()), s_.twin_.end());
            // The islands inside the hole touch the hole, but they do not contain it. The smallest region that
            // contains the hole is the nearest one around it.
            int index = -1;
            int fallback = -1;
            int smallestRegion = -1;
            for (int r = 0; r < static_cast<int>(s_.regions_.size()); ++r)
            {
                const int region = s_.regions_[r];
                if (s_.pool_.Count(region) == holeSize && SameSetAsTwin(s_.pool_.Data(region), holeSize))
                    continue;
                if (fallback < 0)
                    fallback = r;
                if (!HoleInside(region, hole))
                    continue;
                if (smallestRegion < 0 || CompareAreaMagnitude(region, smallestRegion) < 0)
                {
                    smallestRegion = region;
                    index = r;
                }
            }
            if (index < 0)
                index = ea::max(fallback, 0);
            const int outer = s_.regions_[index];
            if (Bridge(outer, hole))
            {
                const int first = s_.pool_.Add(s_.temp1_.data(), static_cast<int>(s_.temp1_.size()));
                const int second = s_.pool_.Add(s_.temp2_.data(), static_cast<int>(s_.temp2_.size()));
                s_.regions_[index] = first;
                s_.regionBridged_[index] = 1;
                s_.regions_.push_back(second);
                s_.regionBridged_.push_back(1);
            }
            else
            {
                const ea::pair<int, int> nearest = Nearest(outer, hole);
                const int* outerIds = s_.pool_.Data(outer);
                const int outerCount = s_.pool_.Count(outer);
                s_.temp1_.clear();
                for (int k = 0; k <= outerCount; ++k)
                    s_.temp1_.push_back(outerIds[(nearest.first + k) % outerCount]);
                for (int k = 0; k <= holeSize; ++k)
                    s_.temp1_.push_back(holeIds[(nearest.second + k) % holeSize]);
                s_.regions_[index] = s_.pool_.Add(s_.temp1_.data(), static_cast<int>(s_.temp1_.size()));
                s_.regionBridged_[index] = 1;
            }
        }
        return true;
    }

    const Corefinement& core_;
    const Mesh& mesh_;
    CsgScratch& scratch_;
    CsgSplitScratch& s_;
    /// Projection of the current triangle: the dropped axis and the swap of the two other axes.
    int dropAxis_{2};
    bool flip_{};
    int edgeCount_{};
    int triangle_{};
    bool failed_{};
    int holeCount_{};
    /// Points of the ids in a test of the trace, or null. Then the ids index the points.
    const CsgGridPoint2* testPoints_{};
    /// Grid points of the ids in a test of the trace with the weld, or null.
    const CsgGridPoint* testGrid_{};
    int pendingFrom_{};
};

/// Interpolate the attributes at a point of the triangle a, b, c. The weights are clamped into the triangle.
ModelVertex Barycentric(const ModelVertex& a, const ModelVertex& b, const ModelVertex& c, const Vector3& point)
{
    const Vector3 pa = a.GetPosition();
    const Vector3 pb = b.GetPosition();
    const Vector3 pc = c.GetPosition();
    const Vector3 normal = (pb - pa).CrossProduct(pc - pa);
    const float area = normal.LengthSquared();
    if (area <= 0.0f)
        return a;
    const float wb = ClampFloat((point - pa).CrossProduct(pc - pa).DotProduct(normal) / area, 0.0f, 1.0f);
    const float wc = ClampFloat((pb - pa).CrossProduct(point - pa).DotProduct(normal) / area, 0.0f, 1.0f - wb);
    const float wa = 1.0f - wb - wc;
    // Blend b and c first, then blend the pair with a.
    const ModelVertex pair = wb + wc > 0.0f ? LerpModelVertex(b, c, wc / (wb + wc)) : b;
    return LerpModelVertex(a, pair, 1.0f - wa);
}

} // namespace

const CsgGridPoint& PointGrid(const Corefinement& core, int id)
{
    if (id >= core.CrossingBase())
        return core.CrossingAt(id).grid_;
    return id >= core.b_.offset_ ? core.b_.grid_[id - core.b_.offset_] : core.a_.grid_[id];
}

Vector3 PointPosition(const Corefinement& core, int id)
{
    if (id >= core.CrossingBase())
        return core.CrossingAt(id).position_;
    return id >= core.b_.offset_ ? core.b_.loops_.positions_[id - core.b_.offset_] : core.a_.loops_.positions_[id];
}

bool RegionPolygon(const Corefinement& core, const Mesh& mesh, int face, const int* ids, int count, CsgPolygon& output)
{
    const CsgPolygon& source = *mesh.solid_.GetFace(face);
    const unsigned start = mesh.loops_.vertexStart_[face];
    const unsigned end = mesh.loops_.vertexStart_[face + 1];
    const unsigned vertexCount = static_cast<unsigned>(source.vertices_.size());
    if (vertexCount == 0 || end - start != vertexCount)
        return false;

    // The local index of a welded id in the face. The search is bounded by the corners of the face.
    const auto localVertex = [&](int id) -> int
    {
        for (unsigned i = start; i < end; ++i)
        {
            if (mesh.loops_.ids_[i] == id)
                return static_cast<int>(i - start);
        }
        return -1;
    };

    ea::vector<ModelVertex> vertices;
    vertices.reserve(count);
    const auto triangleVertices = mesh.solid_.GetTriangleVertices();
    for (int i = 0; i < count; ++i)
    {
        const int id = ids[i];
        if (id < 0 || id >= core.CrossingBase() + static_cast<int>(core.crossings_.size()))
            return false;
        if (id < core.CrossingBase())
        {
            const int local = localVertex(core.Local(mesh, id));
            if (local >= 0)
            {
                ModelVertex corner = source.vertices_[local];
                // One id has one position: that of the first operand.
                if (core.IsAlias(mesh, id))
                    corner.SetPosition(PointPosition(core, id));
                vertices.push_back(corner);
                continue;
            }
            // A vertex that is not on this face gives no record. The fit fails.
        }
        // Use the crossing only if it belongs to this face.
        const Crossing* found = core.FindRecord(id,
            [&](const Crossing& c)
            {
                if (c.key_.edgeInSecond_ == mesh.isSecond_)
                    return localVertex(c.key_.low_) >= 0 && localVertex(c.key_.high_) >= 0;
                const int t = c.key_.triangle_;
                return t >= 0 && t < static_cast<int>(mesh.triangleFace_.size()) && mesh.triangleFace_[t] == face;
            });
        if (!found)
            return false;
        const Crossing& crossing = *found;
        ModelVertex vertex;
        if (crossing.key_.edgeInSecond_ == mesh.isSecond_)
        {
            // On an edge of this face, or on a diagonal of its triangulation.
            const int low = localVertex(crossing.key_.low_);
            const int high = localVertex(crossing.key_.high_);
            if (low < 0 || high < 0)
                return false;
            vertex = LerpModelVertex(source.vertices_[low], source.vertices_[high], static_cast<float>(crossing.t_));
        }
        else
        {
            const int triangle = crossing.key_.triangle_;
            if (triangle < 0 || triangle >= static_cast<int>(mesh.triangleFace_.size())
                || mesh.triangleFace_[triangle] != face)
                return false;
            const unsigned a = triangleVertices[triangle * 3];
            const unsigned b = triangleVertices[triangle * 3 + 1];
            const unsigned c = triangleVertices[triangle * 3 + 2];
            if (a >= vertexCount || b >= vertexCount || c >= vertexCount)
                return false;
            vertex = Barycentric(source.vertices_[a], source.vertices_[b], source.vertices_[c], crossing.position_);
        }
        vertex.SetPosition(id < core.CrossingBase() ? PointPosition(core, id) : crossing.position_);
        vertices.push_back(vertex);
    }
    output = CsgPolygon(ea::move(vertices), source.shared_);
    output.plane_ = source.plane_;
    return true;
}

namespace
{

/// Whether the point p is strictly between the ends of the segment ab: on its line and not at an end.
bool StrictlyInside(const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& p)
{
    return CsgExact::Orient2(a, b, p) == 0
        && CsgExact::Dot2(CsgExact::Difference(p, a), CsgExact::Difference(p, b)) < 0;
}

/// Whether the segments ab and cd cross. They cross if they have a point in common that is not an end of both,
/// or if they overlap along a line. A common end point is not a crossing. A segment with equal ends never crosses.
/// If strict is false, only a proper crossing counts: the segments cross at a point inside both of them.
bool SegmentsConflict(const CsgGridPoint2& a, const CsgGridPoint2& b, const CsgGridPoint2& c, const CsgGridPoint2& d,
    bool strict)
{
    if (a == b || c == d)
        return false;
    const int o1 = CsgExact::Orient2(a, b, c);
    const int o2 = CsgExact::Orient2(a, b, d);
    if (o1 * o2 > 0)
        return false;
    const int o3 = CsgExact::Orient2(c, d, a);
    const int o4 = CsgExact::Orient2(c, d, b);
    if (o3 * o4 > 0)
        return false;
    if (o1 * o2 < 0 && o3 * o4 < 0)
        return true;
    if (!strict)
        return false;
    if ((o1 == 0 && StrictlyInside(a, b, c)) || (o2 == 0 && StrictlyInside(a, b, d))
        || (o3 == 0 && StrictlyInside(c, d, a)) || (o4 == 0 && StrictlyInside(c, d, b)))
        return true;
    // Equal collinear segments overlap, and no end point is strictly inside the other segment.
    return (a == c && b == d) || (a == d && b == c);
}

/// Same as ChooseTriangleProjection. It uses the cache of the validity scratch if the cache is on.
void CachedTriangleProjection(
    const Mesh& mesh, int triangle, CsgScratch& scratch, int& dropAxis, bool& flip)
{
    ea::vector<uint8_t>& cache = scratch.validity_.projection_[mesh.isSecond_ ? 1 : 0];
    if (cache.empty())
    {
        ChooseTriangleProjection(mesh, mesh.triangleFace_[triangle], triangle, dropAxis, flip);
        return;
    }
    uint8_t& entry = cache[triangle];
    if (entry == 0)
    {
        ChooseTriangleProjection(mesh, mesh.triangleFace_[triangle], triangle, dropAxis, flip);
        entry = static_cast<uint8_t>(1 + 2 * dropAxis + (flip ? 1 : 0));
        return;
    }
    dropAxis = (entry - 1) / 2;
    flip = ((entry - 1) & 1) != 0;
}

/// Fill the items of the map of the triangle in the validity scratch: the chain edges of the boundary first, then the
/// curve segments. The function returns false if the map has no area.
bool BuildMapItems(const Corefinement& core, const Mesh& mesh, int triangle, CsgScratch& scratch)
{
    CsgValidityScratch& v = scratch.validity_;
    v.chain_.clear();
    const auto addChainId = [&v](int id)
    {
        if (v.chain_.empty() || v.chain_.back() != id)
            v.chain_.push_back(id);
    };
    for (int i = 0; i < 3; ++i)
    {
        const int from = mesh.triangleIds_[triangle * 3 + i];
        const int to = mesh.triangleIds_[triangle * 3 + (i + 1) % 3];
        addChainId(core.Global(mesh, from));
        core.EdgeCrossings(mesh, from, to, scratch, v.edgePoints_);
        for (const int id : v.edgePoints_)
            addChainId(id);
    }
    while (v.chain_.size() > 1 && v.chain_.back() == v.chain_.front())
        v.chain_.pop_back();
    const int chainCount = static_cast<int>(v.chain_.size());
    if (chainCount < 3)
        return false;

    v.items_.clear();
    for (int i = 0; i < chainCount; ++i)
    {
        v.items_.push_back(v.chain_[i]);
        v.items_.push_back(v.chain_[(i + 1) % chainCount]);
    }
    const CsgSegmentLists& segments = mesh.isSecond_ ? core.segmentsB_ : core.segmentsA_;
    for (int node = segments.Head(triangle); node >= 0; node = segments.At(node).next_)
    {
        v.items_.push_back(segments.At(node).p_);
        v.items_.push_back(segments.At(node).q_);
    }
    return true;
}

/// Whether the items m and k of the map have a conflict. See SegmentsConflict.
bool ItemsConflict(const ea::vector<int>& items, const ea::vector<CsgGridPoint2>& points, int m, int k, bool strict)
{
    const int a = items[m * 2];
    const int b = items[m * 2 + 1];
    const int c = items[k * 2];
    const int d = items[k * 2 + 1];
    // The same item twice, such as a segment on a chain edge, is not a crossing.
    if (k == m || (a == c && b == d) || (a == d && b == c))
        return false;
    // Two items with a common end point have no proper crossing.
    if (!strict && (a == c || a == d || b == c || b == d))
        return false;
    const CsgGridPoint2 pa = points[m * 2];
    const CsgGridPoint2 pb = points[m * 2 + 1];
    const CsgGridPoint2 pc = points[k * 2];
    const CsgGridPoint2 pd = points[k * 2 + 1];
    // Boxes that do not touch cannot have a common point.
    if (ea::max(pc.u_, pd.u_) < ea::min(pa.u_, pb.u_) || ea::min(pc.u_, pd.u_) > ea::max(pa.u_, pb.u_)
        || ea::max(pc.v_, pd.v_) < ea::min(pa.v_, pb.v_) || ea::min(pc.v_, pd.v_) > ea::max(pa.v_, pb.v_))
        return false;
    return SegmentsConflict(pa, pb, pc, pd, strict);
}

/// Prepare the cache of the map of a triangle for one crossing: the items, the projected end points and the items
/// at the crossing. The function returns false if the map has no area.
bool PrepareMapCache(const Corefinement& core, const Mesh& mesh, int triangle, int crossingId, CsgScratch& scratch,
    CsgMapCache& cache)
{
    if (!BuildMapItems(core, mesh, triangle, scratch))
    {
        cache.items_.clear();
        return false;
    }
    const CsgValidityScratch& v = scratch.validity_;
    CachedTriangleProjection(mesh, triangle, scratch, cache.dropAxis_, cache.flip_);
    cache.items_ = v.items_;
    const int count = static_cast<int>(cache.items_.size());
    cache.points_.resize(count);
    cache.ends_.clear();
    cache.incident_.clear();
    for (int i = 0; i < count; ++i)
    {
        const int id = cache.items_[i];
        cache.points_[i] = CsgExact::Project(PointGrid(core, id), cache.dropAxis_, cache.flip_);
        if (id == crossingId)
        {
            cache.ends_.push_back(i);
            if (cache.incident_.empty() || cache.incident_.back() != i / 2)
                cache.incident_.push_back(i / 2);
        }
    }
    return true;
}

/// Whether the items at the crossing in a prepared map cross another item when the crossing is at the candidate.
bool CachedMapHasConflict(CsgMapCache& cache, const CsgGridPoint& candidate)
{
    if (cache.incident_.empty())
        return false;
    const CsgGridPoint2 moved = CsgExact::Project(candidate, cache.dropAxis_, cache.flip_);
    for (const int end : cache.ends_)
        cache.points_[end] = moved;
    const int count = static_cast<int>(cache.items_.size() / 2);
    for (const int m : cache.incident_)
    {
        for (int k = 0; k < count; ++k)
        {
            if (ItemsConflict(cache.items_, cache.points_, m, k, false))
                return true;
        }
    }
    return false;
}

/// Flag each crossing at an end of two items of a prepared map that conflict (not strict). Only the pairs with an
/// item at the moved crossing can change. See MarkMapConflicts.
void MarkCacheConflicts(const CsgMapCache& cache, int base, ea::vector<uint8_t>& suspects)
{
    const int count = static_cast<int>(cache.items_.size() / 2);
    for (const int m : cache.incident_)
    {
        for (int k = 0; k < count; ++k)
        {
            if (!ItemsConflict(cache.items_, cache.points_, m, k, false))
                continue;
            for (const int item : {m, k})
            {
                for (int end = 0; end < 2; ++end)
                {
                    const int id = cache.items_[item * 2 + end];
                    if (id >= base)
                        suspects[id - base] = 1;
                }
            }
        }
    }
}

/// Whether a segment or a chain edge at the crossing crosses another item of the map of the triangle.
/// The map has the boundary chain of the triangle and the curve segments on it. The crossing moves to the candidate.
bool MapHasConflict(const Corefinement& core, const Mesh& mesh, int triangle, int dropAxis, bool flip, int crossingId,
    const CsgGridPoint& candidate, CsgScratch& scratch, bool strict)
{
    if (!BuildMapItems(core, mesh, triangle, scratch))
        return false;
    CsgValidityScratch& v = scratch.validity_;
    const int count = static_cast<int>(v.items_.size() / 2);
    // The projected end points of the items. The crossing takes the candidate position.
    v.points_.resize(count * 2);
    for (int i = 0; i < count * 2; ++i)
    {
        const int id = v.items_[i];
        v.points_[i] = CsgExact::Project(id == crossingId ? candidate : PointGrid(core, id), dropAxis, flip);
    }
    for (int m = 0; m < count; ++m)
    {
        if (v.items_[m * 2] != crossingId && v.items_[m * 2 + 1] != crossingId)
            continue;
        for (int k = 0; k < count; ++k)
        {
            if (ItemsConflict(v.items_, v.points_, m, k, strict))
                return true;
        }
    }
    return false;
}

/// Flag each crossing at an end of two items of the map that conflict (not strict) at the current positions.
/// The flag is 1 at the index of the crossing. A crossing without a flag has no segment rule failure in this map.
void MarkMapConflicts(const Corefinement& core, const Mesh& mesh, int triangle, CsgScratch& scratch,
    ea::vector<uint8_t>& suspects)
{
    if (!BuildMapItems(core, mesh, triangle, scratch))
        return;
    CsgValidityScratch& v = scratch.validity_;
    int dropAxis = 0;
    bool flip = false;
    CachedTriangleProjection(mesh, triangle, scratch, dropAxis, flip);
    const int count = static_cast<int>(v.items_.size() / 2);
    v.points_.resize(count * 2);
    for (int i = 0; i < count * 2; ++i)
        v.points_[i] = CsgExact::Project(PointGrid(core, v.items_[i]), dropAxis, flip);
    const int base = core.CrossingBase();
    const auto mark = [&](int item)
    {
        for (int end = 0; end < 2; ++end)
        {
            const int id = v.items_[item * 2 + end];
            if (id >= base)
                suspects[id - base] = 1;
        }
    };
    v.boxes_.resize(count * 4);
    for (int i = 0; i < count; ++i)
    {
        const CsgGridPoint2 p = v.points_[i * 2];
        const CsgGridPoint2 q = v.points_[i * 2 + 1];
        v.boxes_[i * 4] = ea::min(p.u_, q.u_);
        v.boxes_[i * 4 + 1] = ea::max(p.u_, q.u_);
        v.boxes_[i * 4 + 2] = ea::min(p.v_, q.v_);
        v.boxes_[i * 4 + 3] = ea::max(p.v_, q.v_);
    }
    for (int m = 0; m < count; ++m)
    {
        const CsgGridInt* box = &v.boxes_[m * 4];
        for (int k = m + 1; k < count; ++k)
        {
            const CsgGridInt* other = &v.boxes_[k * 4];
            // Boxes that do not touch cannot have a common point.
            if (other[1] < box[0] || other[0] > box[1] || other[3] < box[2] || other[2] > box[3])
                continue;
            if (ItemsConflict(v.items_, v.points_, m, k, false))
            {
                mark(m);
                mark(k);
            }
        }
    }
}

/// Trace the closed cycle of directed segments through the crossing in the map of the triangle. Each point of the
/// cycle must be a crossing of an edge of the other mesh with this triangle, so the cycle does not touch the
/// boundary of the triangle. Each point must have one outgoing segment. The output has the ids in the order of the
/// segments, the crossing first. The function returns false if the crossing is not on such a cycle.
bool TraceClosedCycle(const Corefinement& core, const Mesh& mesh, int triangle, int crossingId, ea::vector<int>& cycle)
{
    const CsgSegmentLists& segments = mesh.isSecond_ ? core.segmentsB_ : core.segmentsA_;
    const int count = segments.Count(triangle);
    cycle.clear();
    if (count < 3)
        return false;
    int current = crossingId;
    while (static_cast<int>(cycle.size()) < count)
    {
        cycle.push_back(current);
        int next = -1;
        int outgoing = 0;
        for (int node = segments.Head(triangle); node >= 0; node = segments.At(node).next_)
        {
            if (segments.At(node).p_ == current)
            {
                next = segments.At(node).q_;
                ++outgoing;
            }
        }
        if (outgoing != 1)
            return false;
        if (next == crossingId)
            return cycle.size() >= 3;
        if (next < core.CrossingBase())
            return false;
        const CsgCrossingKey& key = core.CrossingAt(next).key_;
        if (key.triangle_ != triangle || key.edgeInSecond_ == mesh.isSecond_)
            return false;
        current = next;
    }
    return false;
}

/// Return the sign of the area of the exact cycle of crossings in the projection, or 0 if the estimate does not
/// decide. The exact point of a crossing is low + t (high - low). The parameter t is the double quotient of the
/// exact volumes. Its relative error is below 2^-44. The bound covers this error and the rounding of the sums.
/// The output fragile tells whether moves of the crossings in their grid cells can change the sign. A move in the
/// cell is at most 1 step on each axis. It changes twice the area by at most 2 |perimeter| + 2 count.
int CycleAreaSignEstimate(
    const Corefinement& core, const int* cycle, int count, int dropAxis, bool flip, bool* fragile = nullptr)
{
    struct Point
    {
        double u_;
        double v_;
        double error_;
    };
    // The points are relative to the low end of the edge of the first crossing. This keeps the terms small.
    const Crossing& first = core.CrossingAt(cycle[0]);
    const CsgGridPoint& origin = (first.key_.edgeInSecond_ ? core.b_ : core.a_).grid_[first.key_.low_];
    const auto exactPoint = [&](int id)
    {
        const Crossing& crossing = core.CrossingAt(id);
        const Mesh& edges = crossing.key_.edgeInSecond_ ? core.b_ : core.a_;
        const CsgGridPoint& low = edges.grid_[crossing.key_.low_];
        const CsgGridPoint& high = edges.grid_[crossing.key_.high_];
        const CsgGridInt lowAxes[3] = {low.x_, low.y_, low.z_};
        const CsgGridInt highAxes[3] = {high.x_, high.y_, high.z_};
        const CsgGridInt originAxes[3] = {origin.x_, origin.y_, origin.z_};
        double axes[3];
        double error = 0.0;
        for (int axis = 0; axis < 3; ++axis)
        {
            const double offset = static_cast<double>(lowAxes[axis]) - originAxes[axis];
            const double delta = static_cast<double>(highAxes[axis]) - lowAxes[axis];
            axes[axis] = offset + crossing.t_ * delta;
            error = ea::max(error,
                std::ldexp(std::fabs(delta), -44) + std::ldexp(std::fabs(offset) + std::fabs(axes[axis]), -50));
        }
        // The same axes as CsgExact::Project.
        const int uAxis = (dropAxis + 1) % 3;
        const int vAxis = (dropAxis + 2) % 3;
        return flip ? Point{axes[vAxis], axes[uAxis], error} : Point{axes[uAxis], axes[vAxis], error};
    };
    const Point start = exactPoint(cycle[0]);
    Point previous = start;
    double sum = 0.0;
    double bound = 0.0;
    double magnitude = 0.0;
    double perimeter = 0.0;
    for (int i = 1; i <= count; ++i)
    {
        const Point point = i < count ? exactPoint(cycle[i]) : start;
        perimeter += std::fabs(point.u_ - previous.u_) + std::fabs(point.v_ - previous.v_);
        const double a = previous.u_ * point.v_;
        const double b = previous.v_ * point.u_;
        sum += a - b;
        magnitude += std::fabs(a) + std::fabs(b);
        bound += previous.error_ * (std::fabs(point.u_) + std::fabs(point.v_))
            + point.error_ * (std::fabs(previous.u_) + std::fabs(previous.v_)) + 2.0 * previous.error_ * point.error_;
        previous = point;
    }
    // The factor 2 covers the use of the estimated coordinates in the bound.
    bound = 2.0 * bound + std::ldexp(magnitude, -50) * static_cast<double>(count + 2);
    if (fragile)
    {
        // The factor and the term 1 cover the rounding of the perimeter.
        *fragile = std::fabs(sum) <= bound + 1.01 * (2.0 * perimeter + 2.0 * count) + 1.0;
    }
    return sum > bound ? 1 : (sum < -bound ? -1 : 0);
}

/// Fill the cycle cache of the validity scratch with the closed cycles of segments in the triangles of the mesh.
/// A cycle goes to the cache only if moves in the grid cells can turn it over and the estimate gives its sign.
/// See TraceClosedCycle for the cycles.
void FindFragileCycles(const Corefinement& core, const Mesh& mesh, CsgScratch& scratch)
{
    CsgValidityScratch& v = scratch.validity_;
    const CsgSegmentLists& segments = mesh.isSecond_ ? core.segmentsB_ : core.segmentsA_;
    const int base = core.CrossingBase();
    for (unsigned triangle = 0; triangle < segments.ListCount(); ++triangle)
    {
        const int count = segments.Count(triangle);
        if (count < 3)
            continue;
        // The segments sorted by the start point. A point with two outgoing segments is on no cycle.
        v.pairs_.clear();
        for (int node = segments.Head(triangle); node >= 0; node = segments.At(node).next_)
            v.pairs_.push_back({segments.At(node).p_, segments.At(node).q_});
        ea::sort(v.pairs_.begin(), v.pairs_.end());
        v.used_.assign(count, 0);
        const auto interior = [&](int id)
        {
            if (id < base)
                return false;
            const CsgCrossingKey& key = core.CrossingAt(id).key_;
            return key.triangle_ == static_cast<int>(triangle) && key.edgeInSecond_ != mesh.isSecond_;
        };
        for (int first = 0; first < count; ++first)
        {
            if (v.used_[first])
                continue;
            const int start = v.pairs_[first].first;
            v.cycle_.clear();
            int current = start;
            bool closed = false;
            while (static_cast<int>(v.cycle_.size()) < count && interior(current))
            {
                const auto it = ea::lower_bound(v.pairs_.begin(), v.pairs_.end(), ea::make_pair(current, INT_MIN));
                const int at = static_cast<int>(it - v.pairs_.begin());
                if (at >= count || it->first != current || (at + 1 < count && v.pairs_[at + 1].first == current)
                    || v.used_[at])
                    break;
                v.used_[at] = 1;
                v.cycle_.push_back(current);
                current = it->second;
                if (current == start)
                {
                    closed = v.cycle_.size() >= 3;
                    break;
                }
            }
            if (!closed)
                continue;
            int dropAxis = 0;
            bool flip = false;
            CachedTriangleProjection(mesh, static_cast<int>(triangle), scratch, dropAxis, flip);
            bool fragile = false;
            const int cycleCount = static_cast<int>(v.cycle_.size());
            const int sign = CycleAreaSignEstimate(core, v.cycle_.data(), cycleCount, dropAxis, flip, &fragile);
            if (sign == 0 || !fragile)
                continue;
            const int cycle = static_cast<int>(v.cycleSign_.size());
            for (const int id : v.cycle_)
                v.cycleOf_[id - base] = cycle;
            v.cycleIds_.insert(v.cycleIds_.end(), v.cycle_.begin(), v.cycle_.end());
            v.cycleStart_.push_back(static_cast<int>(v.cycleIds_.size()));
            v.cycleSign_.push_back(static_cast<int8_t>(sign));
        }
    }
}

} // namespace

unsigned Corefinement::CrossingInvalidRules(const Crossing& record, const CsgGridPoint& candidate, CsgScratch& scratch,
    bool stopAtFirst, bool strict, bool segmentRule) const
{
    URHO3D_ASSERT(a_.HasEdgeTriangles() && b_.HasEdgeTriangles(), "CsgCorefine: build the edge triangles first");
    // The test supports the crossings of crossings_.
    const ptrdiff_t index = &record - crossings_.data();
    URHO3D_ASSERT(index >= 0 && index < static_cast<ptrdiff_t>(crossings_.size()), "CsgCorefine: unknown crossing");
    const int crossingId = CrossingBase() + static_cast<int>(index);
    const CsgCrossingKey& key = record.key_;
    const Mesh& edges = key.edgeInSecond_ ? b_ : a_;
    const Mesh& other = key.edgeInSecond_ ? a_ : b_;

    // The maps of three triangles hold the crossing: the crossed triangle, then the triangles on the edge.
    struct View
    {
        const Mesh* mesh_;
        int triangle_;
        int dropAxis_;
        bool flip_;
    };
    View views[3];
    int viewCount = 0;
    const auto addView = [&](const Mesh& mesh, int triangle)
    {
        View& view = views[viewCount++];
        view.mesh_ = &mesh;
        view.triangle_ = triangle;
        CachedTriangleProjection(mesh, triangle, scratch, view.dropAxis_, view.flip_);
    };
    addView(other, key.triangle_);
    if (const CsgEdgeTriangles* sides = edges.FindEdgeTriangles(key.low_, key.high_))
    {
        if (sides->first_ >= 0)
            addView(edges, sides->first_);
        if (sides->second_ >= 0)
            addView(edges, sides->second_);
    }

    unsigned rules = 0;
    const auto finished = [&]() { return stopAtFirst && rules != 0; };

    // Rule d: the position is not a corner of the three triangles.
    for (int i = 0; i < viewCount && strict; ++i)
    {
        const View& view = views[i];
        for (int corner = 0; corner < 3; ++corner)
        {
            if (view.mesh_->grid_[view.mesh_->triangleIds_[view.triangle_ * 3 + corner]] == candidate)
                rules |= CsgRuleCorner;
        }
    }
    if (finished())
        return rules;

    // Rule a: the position is strictly inside the projected crossed triangle.
    {
        const View& view = views[0];
        const auto corner = [&](int i)
        {
            return CsgExact::Project(view.mesh_->grid_[view.mesh_->triangleIds_[view.triangle_ * 3 + i]],
                view.dropAxis_, view.flip_);
        };
        const CsgGridPoint2 p0 = corner(0);
        const CsgGridPoint2 p1 = corner(1);
        const CsgGridPoint2 p2 = corner(2);
        const CsgGridPoint2 q = CsgExact::Project(candidate, view.dropAxis_, view.flip_);
        const int side = CsgExact::Orient2(p0, p1, p2);
        // A position on the boundary counts as inside if strict is false.
        const int q0 = CsgExact::Orient2(p0, p1, q);
        const int q1 = CsgExact::Orient2(p1, p2, q);
        const int q2 = CsgExact::Orient2(p2, p0, q);
        const bool outside = strict ? (q0 != side || q1 != side || q2 != side)
                                    : (q0 == -side || q1 == -side || q2 == -side);
        if (side == 0 || outside)
            rules |= CsgRuleInside;
    }
    if (finished())
        return rules;

    // Rule b: the position keeps the strict order along the edge, against the neighbors in the order of the sort.
    if (viewCount > 1)
    {
        const EdgeList* list = edgeCrossings_[key.edgeInSecond_ ? 1 : 0].Find(
            CsgDirectedKey(static_cast<unsigned>(key.low_), static_cast<unsigned>(key.high_)));
        int before = -1;
        int after = -1;
        for (int node = list ? list->head_ : -1; node >= 0; node = edgeNodes_[node].next_)
        {
            const int item = edgeNodes_[node].record_;
            if (item == static_cast<int>(index))
                continue;
            if (CompareRecords(item, static_cast<int>(index)) < 0)
            {
                if (before < 0 || CompareRecords(item, before) > 0)
                    before = item;
            }
            else if (after < 0 || CompareRecords(item, after) < 0)
                after = item;
        }
        const CsgGridPoint& beforeGrid = before >= 0 ? PointGrid(*this, RecordId(before)) : edges.grid_[key.low_];
        const CsgGridPoint& afterGrid = after >= 0 ? PointGrid(*this, RecordId(after)) : edges.grid_[key.high_];
        for (int i = 1; i < viewCount; ++i)
        {
            const View& view = views[i];
            const auto project = [&](const CsgGridPoint& p) { return CsgExact::Project(p, view.dropAxis_, view.flip_); };
            const CsgGridPoint2 direction =
                CsgExact::Difference(project(edges.grid_[key.high_]), project(edges.grid_[key.low_]));
            const CsgGridPoint2 q = project(candidate);
            // The order may keep equal positions if strict is false. Only a reversed order breaks the rule.
            const int limit = strict ? 1 : 0;
            if (CsgExact::Dot2(CsgExact::Difference(q, project(beforeGrid)), direction) < limit
                || CsgExact::Dot2(CsgExact::Difference(project(afterGrid), q), direction) < limit)
                rules |= CsgRuleEdgeOrder;
        }
    }
    if (finished())
        return rules;

    // Rule e: a closed cycle of segments through the crossing inside the crossed triangle keeps the orientation of
    // the exact cycle. A rounded cycle that turns over gives the region inside it the wrong side.
    {
        const View& view = views[0];
        CsgValidityScratch& v = scratch.validity_;
        const int* ids = nullptr;
        int count = 0;
        int exact = 0;
        if (!v.cycleOf_.empty())
        {
            // The cache of ConstrainCrossings has the cycles that can turn over.
            const int cycle = v.cycleOf_[index];
            if (cycle >= 0)
            {
                ids = v.cycleIds_.data() + v.cycleStart_[cycle];
                count = v.cycleStart_[cycle + 1] - v.cycleStart_[cycle];
                exact = v.cycleSign_[cycle];
            }
        }
        else if (TraceClosedCycle(*this, *view.mesh_, view.triangle_, crossingId, v.cycle_))
        {
            ids = v.cycle_.data();
            count = static_cast<int>(v.cycle_.size());
            exact = CycleAreaSignEstimate(*this, ids, count, view.dropAxis_, view.flip_);
        }
        {
            if (exact != 0)
            {
                v.cyclePoints_.clear();
                for (int k = 0; k < count; ++k)
                {
                    const int id = ids[k];
                    v.cyclePoints_.push_back(CsgExact::Project(
                        id == crossingId ? candidate : PointGrid(*this, id), view.dropAxis_, view.flip_));
                }
                const int rounded =
                    CsgExact::LoopAreaSign(v.cyclePoints_.data(), static_cast<int>(v.cyclePoints_.size()));
                // A rounded cycle of no area also breaks the rule: the region inside it has no area.
                if (rounded != exact)
                    rules |= CsgRuleCycle;
            }
        }
    }
    if (finished())
        return rules;

    // Rule c: the items at the crossing cross no other item in the three maps.
    for (int i = 0; i < viewCount && segmentRule; ++i)
    {
        const View& view = views[i];
        if (MapHasConflict(*this, *view.mesh_, view.triangle_, view.dropAxis_, view.flip_, crossingId, candidate, scratch, strict))
        {
            rules |= CsgRuleSegments;
            break;
        }
    }
    return rules;
}

bool Corefinement::CrossingIsValid(const Crossing& record, const CsgGridPoint& candidate) const
{
    CsgScratch scratch;
    return CrossingIsValid(record, candidate, scratch);
}

int Corefinement::CountInvalidCrossings(CsgScratch& scratch, int (&perRule)[4], bool strict) const
{
    for (int& count : perRule)
        count = 0;
    int invalid = 0;
    for (const Crossing& crossing : crossings_)
    {
        const unsigned rules = CrossingInvalidRules(crossing, crossing.grid_, scratch, false, strict);
        if (rules == 0)
            continue;
        ++invalid;
        for (int bit = 0; bit < 4; ++bit)
            perRule[bit] += (rules >> bit) & 1;
    }
    return invalid;
}

void Corefinement::ConstrainCrossings(CsgScratch& scratch)
{
    if constexpr (!ConstrainCrossingsOnGrid)
        return;
    if (crossings_.empty())
        return;
    // The edge triangles are needed for the ends of the crossing edges only.
    {
        ea::vector<uint8_t> filterA(a_.VertexCount(), 0);
        ea::vector<uint8_t> filterB(b_.VertexCount(), 0);
        for (const Crossing& crossing : crossings_)
        {
            ea::vector<uint8_t>& filter = crossing.key_.edgeInSecond_ ? filterB : filterA;
            filter[crossing.key_.low_] = 1;
            filter[crossing.key_.high_] = 1;
        }
        a_.BuildEdgeTriangles(&filterA);
        b_.BuildEdgeTriangles(&filterB);
    }

#ifdef CSG_DIAGNOSTICS
    // 0: not a fold, 1: moved to a valid corner, 2: no valid corner, the nearest rounding stays.
    ea::vector<uint8_t> state;
#endif
    // Masks of the axes that a candidate flips: none, one axis, two axes, three axes.
    static constexpr int FlipMasks[8] = {0, 1, 2, 4, 3, 5, 6, 7};

    CsgValidityScratch& cache = scratch.validity_;
    cache.projection_[0].assign(a_.triangleIds_.size() / 3, 0);
    cache.projection_[1].assign(b_.triangleIds_.size() / 3, 0);
    cache.cycleOf_.assign(crossings_.size(), -1);
    cache.cycleIds_.clear();
    cache.cycleStart_.assign(1, 0);
    cache.cycleSign_.clear();
    FindFragileCycles(*this, a_, scratch);
    FindFragileCycles(*this, b_, scratch);

    // A crossing is a suspect if it is at an end of two conflicting items of a map. Only a suspect gets the segment rule.
    // A cheap pass finds the conflicts once for each map. A move of a crossing marks the conflicts of its maps again.
    ea::vector<uint8_t> suspects(crossings_.size(), 0);
    const auto forEachMap = [&](const Crossing& record, const auto& function)
    {
        const CsgCrossingKey& key = record.key_;
        const Mesh& edges = key.edgeInSecond_ ? b_ : a_;
        const Mesh& other = key.edgeInSecond_ ? a_ : b_;
        function(other, key.triangle_);
        if (const CsgEdgeTriangles* sides = edges.FindEdgeTriangles(key.low_, key.high_))
        {
            if (sides->first_ >= 0)
                function(edges, sides->first_);
            if (sides->second_ >= 0)
                function(edges, sides->second_);
        }
    };
    // A crossing that is exactly on the grid never moves. Only the maps of the other crossings need the cheap pass.
    ea::vector<uint8_t> inexact(crossings_.size(), 0);
    for (size_t index = 0; index < crossings_.size(); ++index)
    {
        const Crossing& crossing = crossings_[index];
        if (crossing.difference_.Sign() == 0)
            continue;
        // A crossing of a group of Placed is on the grid point of the group. A move pulls the group apart.
        // So it counts as exact here: it does not move, and ConstrainClusters keeps it as a fixed point.
        if (crossing.placedGroup_)
            continue;
        const CsgCrossingKey& key = crossing.key_;
        const Mesh& edges = key.edgeInSecond_ ? b_ : a_;
        const CsgGridPoint& low = edges.grid_[key.low_];
        const CsgGridPoint& high = edges.grid_[key.high_];
        // The double estimate of the exact point decides a crossing that is far from its grid point.
        // The error of t_ is about 2^-44. The edge delta is below 2^30 grid units. So the estimate error is
        // about 2^-14 grid units, far below the 1e-3 test. The exact CrossingCell decides the other crossings.
        const double dx = low.x_ + crossing.t_ * (static_cast<double>(high.x_) - low.x_) - crossing.grid_.x_;
        const double dy = low.y_ + crossing.t_ * (static_cast<double>(high.y_) - low.y_) - crossing.grid_.y_;
        const double dz = low.z_ + crossing.t_ * (static_cast<double>(high.z_) - low.z_) - crossing.grid_.z_;
        if (std::abs(dx) > 1e-3 || std::abs(dy) > 1e-3 || std::abs(dz) > 1e-3)
        {
            inexact[index] = 1;
            continue;
        }
        CsgGridPoint lower;
        CsgGridPoint upper;
        CsgExact::CrossingCell(low, high, crossing.lowVolume_, crossing.difference_, crossing.grid_, lower, upper);
        inexact[index] = lower == upper ? 0 : 1;
    }
    {
        ea::vector<uint8_t> doneA(a_.triangleIds_.size() / 3, 0);
        ea::vector<uint8_t> doneB(b_.triangleIds_.size() / 3, 0);
        for (size_t index = 0; index < crossings_.size(); ++index)
        {
            if (!inexact[index])
                continue;
            forEachMap(crossings_[index],
                [&](const Mesh& mesh, int triangle)
                {
                    uint8_t& done = (mesh.isSecond_ ? doneB : doneA)[triangle];
                    if (done)
                        return;
                    done = 1;
                    MarkMapConflicts(*this, mesh, triangle, scratch, suspects);
                });
        }
    }

    for (size_t index = 0; index < crossings_.size(); ++index)
    {
        Crossing& crossing = crossings_[index];
        const CsgCrossingKey& key = crossing.key_;
        const Mesh& edges = key.edgeInSecond_ ? b_ : a_;
        const Mesh& other = key.edgeInSecond_ ? a_ : b_;
        const CsgGridPoint& lowGrid = edges.grid_[key.low_];
        const CsgGridPoint& highGrid = edges.grid_[key.high_];
        if (!inexact[index])
            continue;
        const CsgGridPoint nearest = crossing.grid_;
        const int crossingId = CrossingBase() + static_cast<int>(index);
        // Prepare the maps of the crossing. The tests of the segment rule use them for each candidate.
        int mapCount = 0;
        bool prepared = false;
        const auto prepare = [&]()
        {
            prepared = true;
            mapCount = 0;
            forEachMap(crossing,
                [&](const Mesh& mesh, int triangle)
                {
                    if (PrepareMapCache(*this, mesh, triangle, crossingId, scratch, cache.maps_[mapCount]))
                        ++mapCount;
                });
        };
        const auto segmentsConflict = [&](const CsgGridPoint& candidate)
        {
            if (!prepared)
                prepare();
            for (int i = 0; i < mapCount; ++i)
            {
                if (CachedMapHasConflict(cache.maps_[i], candidate))
                    return true;
            }
            return false;
        };
        // The cheap rules decide for a crossing that is no suspect.
        if (CrossingInvalidRules(crossing, nearest, scratch, true, false, false) == 0
            && (!suspects[index] || !segmentsConflict(nearest)))
        {
#ifdef CSG_CHECK_SUSPECTS
            // Debug check: the full test must not find a fold in a crossing that is no suspect.
            if (!suspects[index] && CrossingInvalidRules(crossing, nearest, scratch, true, false) != 0)
                fprintf(stderr, "SUSPECT MISS crossing %d\n", static_cast<int>(index));
#endif
            continue;
        }
        // A crossing exactly on the grid has no other corner to move to.
        CsgGridPoint lower;
        CsgGridPoint upper;
        CsgExact::CrossingCell(lowGrid, highGrid, crossing.lowVolume_, crossing.difference_, nearest, lower, upper);
        if (lower == upper)
            continue;
#ifdef CSG_DIAGNOSTICS
        if (state.empty())
            state.resize(crossings_.size(), 0);
#endif

        // The grid points of the three triangles of the maps and of the ends of the edge.
        CsgGridPoint corners[11];
        int cornerCount = 0;
        corners[cornerCount++] = lowGrid;
        corners[cornerCount++] = highGrid;
        for (int corner = 0; corner < 3; ++corner)
            corners[cornerCount++] = other.grid_[other.triangleIds_[key.triangle_ * 3 + corner]];
        if (const CsgEdgeTriangles* sides = edges.FindEdgeTriangles(key.low_, key.high_))
        {
            for (const int triangle : {sides->first_, sides->second_})
            {
                if (triangle < 0)
                    continue;
                for (int corner = 0; corner < 3 && cornerCount < 11; ++corner)
                    corners[cornerCount++] = edges.grid_[edges.triangleIds_[triangle * 3 + corner]];
            }
        }

        const CsgGridInt nearestAxis[3] = {nearest.x_, nearest.y_, nearest.z_};
        const CsgGridInt otherAxis[3] = {nearest.x_ == lower.x_ ? upper.x_ : lower.x_,
            nearest.y_ == lower.y_ ? upper.y_ : lower.y_, nearest.z_ == lower.z_ ? upper.z_ : lower.z_};
        bool found = false;
        for (const int mask : FlipMasks)
        {
            if (mask == 0)
                continue;
            CsgGridInt axes[3];
            bool usable = true;
            for (int axis = 0; axis < 3; ++axis)
            {
                const bool flipAxis = ((mask >> axis) & 1) != 0;
                axes[axis] = flipAxis ? otherAxis[axis] : nearestAxis[axis];
                usable = usable && (!flipAxis || otherAxis[axis] != nearestAxis[axis]);
            }
            if (!usable)
                continue;
            CsgGridPoint candidate;
            candidate.x_ = axes[0];
            candidate.y_ = axes[1];
            candidate.z_ = axes[2];
            if (!InGridRange(candidate))
                continue;
            bool atCorner = false;
            for (int k = 0; k < cornerCount; ++k)
                atCorner = atCorner || corners[k] == candidate;
            if (atCorner)
                continue;
            if (CrossingInvalidRules(crossing, candidate, scratch, true, false, false) != 0
                || segmentsConflict(candidate))
                continue;
            crossing.grid_ = candidate;
            crossing.position_ = candidate.ToPosition();
            found = true;
            break;
        }
        if (found)
        {
            // The caches hold the winner as the position of the crossing.
            for (int i = 0; i < mapCount; ++i)
                MarkCacheConflicts(cache.maps_[i], CrossingBase(), suspects);
        }
#ifdef CSG_DIAGNOSTICS
        state[index] = found ? 1 : 2;
#endif
    }

    // The crossings of one edge with intersecting cells move to one common corner.
#ifdef CSG_DIAGNOSTICS
    if (state.empty())
        state.resize(crossings_.size(), 0);
    ConstrainClusters(scratch, inexact, &state);
#else
    ConstrainClusters(scratch, inexact, nullptr);
#endif

    cache.projection_[0].clear();
    cache.projection_[1].clear();
    cache.cycleOf_.clear();

#ifdef CSG_DIAGNOSTICS
    // A later move can fix a crossing that found no corner. Then it is no fold and it does not count.
    for (size_t index = 0; index < state.size(); ++index)
    {
        if (state[index] == 2
            && CrossingInvalidRules(crossings_[index], crossings_[index].grid_, scratch, true, false) == 0)
            state[index] = 0;
    }
    int moved = 0;
    int stuck = 0;
    for (const uint8_t value : state)
    {
        moved += value == 1 ? 1 : 0;
        stuck += value == 2 ? 1 : 0;
    }
    g_csgDiagnostics.constrainedCrossings_.fetch_add(moved, std::memory_order_relaxed);
    g_csgDiagnostics.unconstrainableCrossings_.fetch_add(stuck, std::memory_order_relaxed);
#endif
}

void Corefinement::ConstrainClusters(
    CsgScratch& scratch, const ea::vector<uint8_t>& inexact, ea::vector<uint8_t>* state)
{
    // Masks of the axes that take the upper end of the cell: none, one axis, two axes, three axes.
    static constexpr int CornerMasks[8] = {0, 1, 2, 4, 3, 5, 6, 7};

    CsgValidityScratch& v = scratch.validity_;
    for (int side = 0; side < 2; ++side)
    {
        for (const auto& entry : edgeCrossings_[side])
        {
            // A cluster needs two crossings and at least one inexact crossing.
            int count = 0;
            bool anyInexact = false;
            for (int node = entry.value_.head_; node >= 0; node = edgeNodes_[node].next_)
            {
                ++count;
                anyInexact = anyInexact || inexact[edgeNodes_[node].record_] != 0;
            }
            if (count < 2 || !anyInexact)
                continue;

            ea::vector<int>& records = v.clusterRecords_;
            records.clear();
            for (int node = entry.value_.head_; node >= 0; node = edgeNodes_[node].next_)
                records.push_back(edgeNodes_[node].record_);
            SortIds(records, scratch.merge_, [this](int x, int y) { return CompareRecords(x, y); });

            // The cell of each crossing: the exact position is inside it.
            v.clusterLower_.resize(count);
            v.clusterUpper_.resize(count);
            v.clusterNearest_.resize(count);
            for (int i = 0; i < count; ++i)
            {
                const Crossing& crossing = crossings_[records[i]];
                if (!inexact[records[i]])
                {
                    v.clusterNearest_[i] = v.clusterLower_[i] = v.clusterUpper_[i] = crossing.grid_;
                    continue;
                }
                const Mesh& edges = crossing.key_.edgeInSecond_ ? b_ : a_;
                const CsgGridPoint& low = edges.grid_[crossing.key_.low_];
                const CsgGridPoint& high = edges.grid_[crossing.key_.high_];
                v.clusterNearest_[i] = CsgExact::RoundCrossing(low, high, crossing.lowVolume_, crossing.difference_);
                CsgExact::CrossingCell(low, high, crossing.lowVolume_, crossing.difference_, v.clusterNearest_[i],
                    v.clusterLower_[i], v.clusterUpper_[i]);
            }

            const auto constrain = [&](int begin, int end, const CsgGridPoint& lower, const CsgGridPoint& upper)
            {
                if (end - begin < 2)
                    return;
                // The cluster is fine if all crossings are at one valid position already.
                const CsgGridPoint current = crossings_[records[begin]].grid_;
                bool same = true;
                for (int i = begin + 1; i < end; ++i)
                    same = same && crossings_[records[i]].grid_ == current;
                const auto allValid = [&](const CsgGridPoint& point)
                {
                    for (int i = begin; i < end; ++i)
                    {
                        if (CrossingInvalidRules(crossings_[records[i]], point, scratch, true, false, true) != 0)
                            return false;
                    }
                    return true;
                };
                if (same && allValid(current))
                    return;

                // The corners of the intersection, the nearest to the roundings first. The order of the masks decides a tie.
                CsgGridPoint corners[8];
                int64_t costs[8];
                int cornerCount = 0;
                for (const int mask : CornerMasks)
                {
                    if (((mask & 1) && lower.x_ == upper.x_) || ((mask & 2) && lower.y_ == upper.y_)
                        || ((mask & 4) && lower.z_ == upper.z_))
                        continue;
                    CsgGridPoint corner;
                    corner.x_ = (mask & 1) ? upper.x_ : lower.x_;
                    corner.y_ = (mask & 2) ? upper.y_ : lower.y_;
                    corner.z_ = (mask & 4) ? upper.z_ : lower.z_;
                    int64_t cost = 0;
                    for (int i = begin; i < end; ++i)
                    {
                        const CsgGridPoint& nearest = v.clusterNearest_[i];
                        cost += std::abs(static_cast<int64_t>(corner.x_) - nearest.x_)
                            + std::abs(static_cast<int64_t>(corner.y_) - nearest.y_)
                            + std::abs(static_cast<int64_t>(corner.z_) - nearest.z_);
                    }
                    // An insertion keeps the order of the masks for equal costs.
                    int slot = cornerCount++;
                    while (slot > 0 && costs[slot - 1] > cost)
                    {
                        costs[slot] = costs[slot - 1];
                        corners[slot] = corners[slot - 1];
                        --slot;
                    }
                    costs[slot] = cost;
                    corners[slot] = corner;
                }

                v.clusterSaved_.clear();
                for (int i = begin; i < end; ++i)
                    v.clusterSaved_.push_back(crossings_[records[i]].grid_);
                const auto place = [&](const CsgGridPoint& point)
                {
                    for (int i = begin; i < end; ++i)
                    {
                        crossings_[records[i]].grid_ = point;
                        crossings_[records[i]].position_ = point.ToPosition();
                    }
                };
                for (int k = 0; k < cornerCount; ++k)
                {
                    if (!InGridRange(corners[k]))
                        continue;
                    place(corners[k]);
                    if (!allValid(corners[k]))
                        continue;
                    if (state)
                    {
                        for (int i = begin; i < end; ++i)
                        {
                            if (v.clusterSaved_[i - begin] != corners[k])
                                (*state)[records[i]] = 1;
                        }
                    }
                    return;
                }
                // No corner is valid. The crossings keep their positions.
                for (int i = begin; i < end; ++i)
                {
                    crossings_[records[i]].grid_ = v.clusterSaved_[i - begin];
                    crossings_[records[i]].position_ = v.clusterSaved_[i - begin].ToPosition();
                    if (state && (*state)[records[i]] == 0)
                        (*state)[records[i]] = 2;
                }
            };

            // A cluster is a run of crossings in the order of the sort. The intersection of all cells of a run is not empty.
            int begin = 0;
            CsgGridPoint lower = v.clusterLower_[0];
            CsgGridPoint upper = v.clusterUpper_[0];
            for (int i = 1; i <= count; ++i)
            {
                if (i < count)
                {
                    const CsgGridPoint& cellLower = v.clusterLower_[i];
                    const CsgGridPoint& cellUpper = v.clusterUpper_[i];
                    CsgGridPoint nextLower;
                    CsgGridPoint nextUpper;
                    nextLower.x_ = ea::max(lower.x_, cellLower.x_);
                    nextLower.y_ = ea::max(lower.y_, cellLower.y_);
                    nextLower.z_ = ea::max(lower.z_, cellLower.z_);
                    nextUpper.x_ = ea::min(upper.x_, cellUpper.x_);
                    nextUpper.y_ = ea::min(upper.y_, cellUpper.y_);
                    nextUpper.z_ = ea::min(upper.z_, cellUpper.z_);
                    if (nextLower.x_ <= nextUpper.x_ && nextLower.y_ <= nextUpper.y_ && nextLower.z_ <= nextUpper.z_)
                    {
                        lower = nextLower;
                        upper = nextUpper;
                        continue;
                    }
                }
                constrain(begin, i, lower, upper);
                if (i < count)
                {
                    begin = i;
                    lower = v.clusterLower_[i];
                    upper = v.clusterUpper_[i];
                }
            }
        }
    }
}

void Corefinement::SplitFaces(CsgScratch& scratch)
{
#ifdef CSG_DIAGNOSTICS
    if (g_csgDiagnostics.countInvalid_.load(std::memory_order_relaxed))
    {
        a_.BuildEdgeTriangles();
        b_.BuildEdgeTriangles();
        int perRule[4];
        const int invalid = CountInvalidCrossings(scratch, perRule);
        g_csgDiagnostics.checkedCrossings_.fetch_add(static_cast<int>(crossings_.size()), std::memory_order_relaxed);
        g_csgDiagnostics.invalidCrossings_.fetch_add(invalid, std::memory_order_relaxed);
        g_csgDiagnostics.invalidInside_.fetch_add(perRule[0], std::memory_order_relaxed);
        g_csgDiagnostics.invalidOrder_.fetch_add(perRule[1], std::memory_order_relaxed);
        g_csgDiagnostics.invalidSegments_.fetch_add(perRule[2], std::memory_order_relaxed);
        g_csgDiagnostics.invalidCorner_.fetch_add(perRule[3], std::memory_order_relaxed);
        const int fold = CountInvalidCrossings(scratch, perRule, false);
        g_csgDiagnostics.foldCrossings_.fetch_add(fold, std::memory_order_relaxed);
        g_csgDiagnostics.foldInside_.fetch_add(perRule[0], std::memory_order_relaxed);
        g_csgDiagnostics.foldOrder_.fetch_add(perRule[1], std::memory_order_relaxed);
        g_csgDiagnostics.foldSegments_.fetch_add(perRule[2], std::memory_order_relaxed);
    }
#endif
    SplitOperand(a_, segmentsA_, scratch, splitA_);
    SplitOperand(b_, segmentsB_, scratch, splitB_);
}

void Corefinement::SplitOperand(
    const Mesh& mesh, const CsgSegmentLists& segments, CsgScratch& scratch, CsgSplitResult& result) const
{
    const int triangleCount = static_cast<int>(mesh.triangleFace_.size());
    result.Reset(static_cast<unsigned>(triangleCount));
    TriangleSplitter splitter(*this, mesh, scratch);
    const CsgSplitScratch& s = scratch.split_;
    for (int triangle = 0; triangle < triangleCount; ++triangle)
    {
        result.triangleRegionStart_[triangle] = result.RegionCount();
        // A collapsed triangle has no segments. A triangle without segments and crossings stays whole.
        const ea::vector<uint8_t>& touched = mesh.isSecond_ ? touchedB_ : touchedA_;
        if ((segments.Count(triangle) == 0 && !touched[triangle]) || mesh.IsCollapsed(triangle))
            continue;
        if (!splitter.Split(mesh.triangleFace_[triangle], triangle, segments))
        {
            result.state_[triangle] = static_cast<uint8_t>(CsgSplitResult::State::Failed);
            CSG_DIAG_COUNT(failedSplits_);
            continue;
        }
        CSG_DIAG_COUNT(splitTriangles_);
        result.state_[triangle] = static_cast<uint8_t>(CsgSplitResult::State::Split);
        result.holeCount_[triangle] = splitter.HoleCount();
        for (unsigned r = 0; r < s.regions_.size(); ++r)
        {
            const int loop = s.regions_[r];
            const int count = s.pool_.Count(loop);
            if (count < 3)
                continue;
            result.ids_.insert(result.ids_.end(), s.pool_.Data(loop), s.pool_.Data(loop) + count);
            result.regionStart_.push_back(static_cast<int>(result.ids_.size()));
            result.bridged_.push_back(s.regionBridged_[r]);
        }
    }
    result.triangleRegionStart_[triangleCount] = result.RegionCount();
}

CsgCorefineTestResult CsgCorefineCrossingsForTest(const CsgSolid& a, const CsgSolid& b)
{
    CsgCorefineTestResult result;
    if (a.FaceCount() == 0 || b.FaceCount() == 0)
        return result;

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);

    result.positions_.reserve(core.crossings_.size());
    result.keys_.reserve(core.crossings_.size());
    for (const Crossing& crossing : core.crossings_)
    {
        result.positions_.push_back(crossing.position_);
        result.keys_.push_back(crossing.key_);
    }
    // A crossing at the grid point of a vertex has the vertex id. Such vertices get the indices after the crossings.
    ea::vector<int> vertexIndex(core.CrossingBase(), -1);
    const auto index = [&](int id)
    {
        if (id >= core.CrossingBase())
            return id - core.CrossingBase();
        if (vertexIndex[id] < 0)
        {
            vertexIndex[id] = static_cast<int>(result.positions_.size());
            result.positions_.push_back(PointPosition(core, id));
            const Crossing* record = core.FindRecord(id, [](const Crossing&) { return true; });
            result.keys_.push_back(record ? record->key_ : CsgCrossingKey{});
        }
        return vertexIndex[id];
    };
    result.segmentCountsA_.resize(core.segmentsA_.ListCount());
    for (unsigned i = 0; i < result.segmentCountsA_.size(); ++i)
        result.segmentCountsA_[i] = core.segmentsA_.Count(static_cast<int>(i));
    result.segmentCountsB_.resize(core.segmentsB_.ListCount());
    for (unsigned i = 0; i < result.segmentCountsB_.size(); ++i)
        result.segmentCountsB_[i] = core.segmentsB_.Count(static_cast<int>(i));
    // The two segment lists receive each segment together. Entry k of the pool is the same segment in both.
    result.segments_.resize(core.segmentsA_.TotalCount());
    for (unsigned list = 0; list < core.segmentsA_.ListCount(); ++list)
    {
        for (int i = core.segmentsA_.Head(static_cast<int>(list)); i >= 0; i = core.segmentsA_.At(i).next_)
        {
            result.segments_[i].triangleA_ = static_cast<int>(list);
            result.segments_[i].p_ = index(core.segmentsA_.At(i).p_);
            result.segments_[i].q_ = index(core.segmentsA_.At(i).q_);
        }
    }
    for (unsigned list = 0; list < core.segmentsB_.ListCount(); ++list)
    {
        for (int i = core.segmentsB_.Head(static_cast<int>(list)); i >= 0; i = core.segmentsB_.At(i).next_)
            result.segments_[i].triangleB_ = static_cast<int>(list);
    }
    result.curveEdges_.reserve(core.curveEdges_.Size());
    for (const auto& entry : core.curveEdges_)
        result.curveEdges_.push_back(ea::make_pair(index(entry.value_.first), index(entry.value_.second)));
    return result;
}

namespace
{

void FillSplitOperand(const Corefinement& core, const Mesh& mesh, const CsgSplitResult& split, CsgSplitTestOperand& output)
{
    output.triangleState_ = split.state_;
    output.holeCount_ = split.holeCount_;
    const int triangleCount = static_cast<int>(mesh.triangleFace_.size());
    for (int triangle = 0; triangle < triangleCount; ++triangle)
    {
        for (int r = split.triangleRegionStart_[triangle]; r < split.triangleRegionStart_[triangle + 1]; ++r)
        {
            CsgSplitTestRegion& region = output.regions_.emplace_back();
            region.triangle_ = triangle;
            region.ids_.assign(split.RegionIds(r), split.RegionIds(r) + split.RegionSize(r));
            region.bridged_ = split.bridged_[r] != 0;
            CsgPolygon polygon;
            if (!RegionPolygon(core, mesh, mesh.triangleFace_[triangle], region.ids_.data(),
                    static_cast<int>(region.ids_.size()), polygon))
                continue;
            region.polygon_ = polygon;
            for (const ModelVertex& vertex : polygon.vertices_)
                region.polygonPositions_.push_back(vertex.GetPosition());
        }
    }
}

} // namespace

CsgSplitTestResult CsgCorefineSplitForTest(const CsgSolid& a, const CsgSolid& b)
{
    CsgSplitTestResult result;
    if (a.FaceCount() == 0 || b.FaceCount() == 0)
        return result;

    Corefinement core(a, b, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    core.ConstrainCrossings(scratch);
    core.SplitFaces(scratch);

    result.vertexCountA_ = core.a_.VertexCount();
    result.vertexCountB_ = core.b_.VertexCount();
    result.crossingBase_ = core.CrossingBase();
    const int total = core.CrossingBase() + static_cast<int>(core.crossings_.size());
    result.positions_.reserve(total);
    CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash> firstId;
    for (int id = 0; id < total; ++id)
    {
        result.positions_.push_back(PointPosition(core, id));
        const CsgGridPoint& point = id >= core.CrossingBase() ? core.CrossingAt(id).grid_
            : id >= core.b_.offset_ ? core.b_.grid_[id - core.b_.offset_] : core.a_.grid_[id];
        CsgVec3Key key;
        key.x_ = static_cast<uint32_t>(point.x_);
        key.y_ = static_cast<uint32_t>(point.y_);
        key.z_ = static_cast<uint32_t>(point.z_);
        bool existed = false;
        int& slot = firstId.FindOrAdd(key, existed);
        if (!existed)
            slot = id;
        result.gridFirst_.push_back(slot);
    }
    for (const auto& entry : core.curveEdges_)
        result.curveEdges_.push_back(entry.value_);
    FillSplitOperand(core, core.a_, core.splitA_, result.a_);
    FillSplitOperand(core, core.b_, core.splitB_, result.b_);
    return result;
}

CsgTraceTestResult CsgTraceRegionsForTest(const ea::vector<CsgGridPoint2>& points, const ea::vector<int>& chain,
    const ea::vector<ea::pair<int, int>>& curves, const ea::vector<CsgGridPoint>* grid)
{
    const CsgSolid empty{ea::span<const CsgPolygon>{}};
    const Corefinement core(empty, empty, CSG_DEFAULT_EPSILON);
    CsgScratch scratch;
    TriangleSplitter splitter(core, core.a_, scratch);
    ea::vector<int> ends;
    for (const auto& curve : curves)
    {
        ends.push_back(curve.first);
        ends.push_back(curve.second);
    }
    CsgTraceTestResult result;
    result.closed_ = splitter.TraceForTest(points.data(), grid ? grid->data() : nullptr, chain.data(), static_cast<int>(chain.size()), ends.data(),
        static_cast<int>(curves.size()));
    const CsgSplitScratch& s = scratch.split_;
    for (const ea::vector<int>* list : {&s.cycles_, &s.closedCycles_})
    {
        for (const int loop : *list)
            result.cycles_.emplace_back(s.pool_.Data(loop), s.pool_.Data(loop) + s.pool_.Count(loop));
    }
    return result;
}

} // namespace CsgInternal

} // namespace Urho3D
