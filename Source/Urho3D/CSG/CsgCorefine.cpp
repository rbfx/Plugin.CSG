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
        CsgFloat4::Create(diff(d0.z_, t0.z_), diff(d1.z_, t0.z_), diff(d2.z_, t0.z_), 0.0f), detVector);
    float det[4];
    detVector.Store(det);
    sides[0] = (decided & 1) != 0 ? SignOfFloat(det[0]) : CsgExact::Orient(t0, t1, t2, d0, mask);
    sides[1] = (decided & 2) != 0 ? SignOfFloat(det[1]) : CsgExact::Orient(t0, t1, t2, d1, mask);
    sides[2] = (decided & 4) != 0 ? SignOfFloat(det[2]) : CsgExact::Orient(t0, t1, t2, d2, mask);
}

} // namespace

int CompareOnEdge(const Crossing& x, const Crossing& y)
{
    const CsgExactVolume& dx = x.difference_;
    const CsgExactVolume& dy = y.difference_;
    // t_x - t_y = (Lx * Dy - Ly * Dx) / (Dx * Dy)
    const int numeratorSign = CsgExpansionProductDifferenceSign(
        x.lowVolume_.count_, x.lowVolume_.terms_, dy.count_, dy.terms_, y.lowVolume_.count_, y.lowVolume_.terms_,
        dx.count_, dx.terms_);
    const int denominatorSign = dx.Sign() * dy.Sign();
    if (numeratorSign != 0)
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
    for (unsigned face = 0; face < solid.FaceCount(); ++face)
    {
        const unsigned start = loops_.vertexStart_[face];
        for (unsigned t = triangleStart[face]; t < triangleStart[face + 1]; ++t)
        {
            triangleFace_[t] = static_cast<int>(face);
            for (unsigned corner = 0; corner < 3; ++corner)
                triangleIds_[t * 3 + corner] = loops_.ids_[start + triangleVertices[t * 3 + corner]];
        }
    }

    // The solid stores the grid points. The mesh does not snap the positions again.
    grid_ = loops_.grid_;
    outOfRange_ = solid.IsOutOfRange();
    // Only a solid out of range can have a position that is not finite.
    if (outOfRange_)
    {
        for (unsigned id = 0; id < loops_.positions_.size(); ++id)
        {
            const Vector3& position = loops_.positions_[id];
            if (std::isfinite(position.x_) && std::isfinite(position.y_) && std::isfinite(position.z_))
                continue;
            if (invalid_.empty())
                invalid_.assign(loops_.positions_.size(), 0);
            invalid_[id] = 1;
        }
    }
}

bool Mesh::IsCollapsed(int triangle) const
{
    // This function also returns true for a triangle with a corner that is not finite. The C# reference does not.
    const int a = triangleIds_[triangle * 3];
    const int b = triangleIds_[triangle * 3 + 1];
    const int c = triangleIds_[triangle * 3 + 2];
    if (a == b || b == c || c == a)
        return true;
    return !invalid_.empty() && (invalid_[a] || invalid_[b] || invalid_[c]);
}

Corefinement::Corefinement(const CsgSolid& a, const CsgSolid& b, float epsilon)
    : a_(a, false, 0)
    , b_(b, true, a_.VertexCount())
    , epsilon_(epsilon)
{
    outOfRange_ = a_.outOfRange_ || b_.outOfRange_;
    if (outOfRange_)
    {
        URHO3D_LOGWARNING("CsgCorefine: an operand has a coordinate outside of +-{}. The coordinate is clamped and the result is not valid.",
            static_cast<int>(CSG_EXACT_COORDINATE_LIMIT));
    }
    segmentsA_.Reset(static_cast<unsigned>(a_.triangleFace_.size()));
    segmentsB_.Reset(static_cast<unsigned>(b_.triangleFace_.size()));
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
            // Exact decisions give two ends. Anything else comes from a degenerate triangle.
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
        const int id = CrossingId(CsgCrossingKey{low, high, otherTriangle, edges.isSecond_}, edges, t0, t1, t2);
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

int Corefinement::CrossingId(
    const CsgCrossingKey& key, const Mesh& edges, const CsgGridPoint& t0, const CsgGridPoint& t1, const CsgGridPoint& t2)
{
    bool existed = false;
    int& idRef = crossingIds_.FindOrAdd(key, existed);
    if (existed)
        return idRef;

    const CsgNormal normal = CsgExact::Normal(t0, t1, t2);
    const CsgExactVolume lowVolume = CsgExact::Dot(normal, t0, edges.grid_[key.low_]);
    const CsgExactVolume difference = CsgExact::Dot(normal, edges.grid_[key.high_], edges.grid_[key.low_]);
    const CsgGridPoint& lowGrid = edges.grid_[key.low_];
    const CsgGridPoint& highGrid = edges.grid_[key.high_];
    double t = 0.5;
    CsgGridPoint grid;
    if (difference.Sign() != 0)
    {
        t = lowVolume.Approximate() / difference.Approximate();
        t = std::isnan(t) ? 0.5 : (t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t));
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

    Crossing data;
    data.grid_ = grid;
    // The crossing lies between the ends of the edge, so it is in the range of the grid.
    URHO3D_ASSERT(InGridRange(grid), "CsgCorefine crossing is outside the range of the grid");
    // The position is grid / scale, so the result vertex is on the grid and the next boolean keeps it.
    // An operand out of range has clamped grid points. Then the position is on the float edge.
    if (!outOfRange_)
        data.position_ = grid.ToPosition();
    else
    {
        const Vector3& lowPosition = edges.loops_.positions_[key.low_];
        const Vector3& highPosition = edges.loops_.positions_[key.high_];
        data.position_ = lowPosition + (highPosition - lowPosition) * static_cast<float>(t);
    }
    data.t_ = t;
    data.key_ = key;
    data.lowVolume_ = lowVolume;
    data.difference_ = difference;
    data.normal_ = normal;

    int id = CrossingBase() + static_cast<int>(crossings_.size());
    int record = static_cast<int>(crossings_.size());
    bool merged = false;
    if constexpr (MergeCrossingsOnGrid)
    {
        // A crossing that rounds to an existing grid point is the same vertex.
        // Its order data stays in an alias, because the order is for the edge of this key.
        CsgVec3Key gridKey;
        gridKey.x_ = static_cast<uint32_t>(grid.x_);
        gridKey.y_ = static_cast<uint32_t>(grid.y_);
        gridKey.z_ = static_cast<uint32_t>(grid.z_);
        bool gridExisted = false;
        int& gridIdRef = gridIds_.FindOrAdd(gridKey, gridExisted);
        if (gridExisted)
        {
            id = gridIdRef;
            const int alias = static_cast<int>(aliases_.size());
            record = -alias - 1;
            aliases_.push_back(data);
            aliasIds_.push_back(id);
            Crossing* tail = &crossings_[id - CrossingBase()];
            while (tail->alias_ >= 0)
                tail = &aliases_[tail->alias_];
            tail->alias_ = alias;
            merged = true;
        }
        else
            gridIdRef = id;
    }
    if (!merged)
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

    SortIds(output, scratch.merge_,
        [this](int x, int y)
        {
            const Crossing& cx = Record(x);
            const Crossing& cy = Record(y);
            // Doubles separate all but nearly coincident crossings. Those are ordered exactly.
            if (std::fabs(cx.t_ - cy.t_) > EDGE_ORDER_TOLERANCE)
                return cx.t_ < cy.t_ ? -1 : 1;
            const int order = CompareOnEdge(cx, cy);
            if (order != 0)
                return order;
            const int idX = RecordId(x);
            const int idY = RecordId(y);
            return idX < idY ? -1 : (idX > idY ? 1 : 0);
        });
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

/// Compare as double.CompareTo does in the C# reference: a NaN is smaller than any number and equal to a NaN.
int CompareDouble(double a, double b)
{
    if (a < b)
        return -1;
    if (a > b)
        return 1;
    if (a == b)
        return 0;
    if (std::isnan(a))
        return std::isnan(b) ? 0 : -1;
    return 1;
}

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
/// The callers skip the edges at the end ids, so an edge that touches an end has another id at the same grid
/// point. Such ids are different points below the grid step. The triangulation uses their float positions, and
/// there a bridge along such an edge can cross it.
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
        s_.boundary_.clear();
        s_.boundaryOut_.Clear();
        s_.curve_.Clear();
        s_.boundaryEdges_.Clear();
        s_.visited_.Clear();
        s_.pool_.Clear();
        s_.cycles_.clear();
        s_.closedCycles_.clear();
        s_.holes_.clear();
        s_.regions_.clear();
        s_.regionBridged_.clear();
        failed_ = false;
        holeCount_ = 0;
        triangle_ = triangle;

        for (int i = 0; i < 3; ++i)
        {
            const int from = mesh_.triangleIds_[triangle * 3 + i];
            const int to = mesh_.triangleIds_[triangle * 3 + (i + 1) % 3];
            s_.boundary_.push_back(core_.Global(mesh_, from));
            core_.EdgeCrossings(mesh_, from, to, scratch_, s_.edgePoints_);
            s_.boundary_.insert(s_.boundary_.end(), s_.edgePoints_.begin(), s_.edgePoints_.end());
        }
        const int boundaryCount = static_cast<int>(s_.boundary_.size());
        // A face may visit a vertex twice, so a vertex can have more than one boundary edge leaving it.
        for (int i = 0; i < boundaryCount; ++i)
        {
            const int from = s_.boundary_[i];
            const int to = s_.boundary_[(i + 1) % boundaryCount];
            s_.boundaryOut_.Add(from, to);
            s_.boundaryEdges_.Insert(DirectedEdgeKey(from, to));
        }
        for (int node = segments.Head(triangle); node >= 0; node = segments.At(node).next_)
        {
            const CsgSegment& segment = segments.At(node);
            s_.curve_.Add(segment.p_, segment.q_);
            s_.curve_.Add(segment.q_, segment.p_);
        }

        ChooseProjection(face, triangle);
        edgeCount_ = static_cast<int>(s_.boundaryEdges_.Size()) + segments.Count(triangle) * 2;
        for (int i = 0; i < boundaryCount; ++i)
            Trace(s_.boundary_[i], s_.boundary_[(i + 1) % boundaryCount]);
        for (int node = segments.Head(triangle); node >= 0; node = segments.At(node).next_)
        {
            const CsgSegment& segment = segments.At(node);
            Trace(segment.p_, segment.q_);
            Trace(segment.q_, segment.p_);
        }
        if (failed_ || static_cast<int>(s_.visited_.Size()) != edgeCount_)
            return false;

        ClassifyClosedCycles();
        s_.regions_ = s_.cycles_;
        s_.regionBridged_.assign(s_.regions_.size(), 0);
        if (!s_.holes_.empty())
        {
            holeCount_ = static_cast<int>(s_.holes_.size());
            if (!PlaceHoles())
                return false;
        }
        return true;
    }

    /// Number of holes joined by bridges in the last split.
    int HoleCount() const { return holeCount_; }

private:
    using List = CsgAdjacency::List;

    /// Grid point of a global id: a vertex of an operand or a crossing.
    const CsgGridPoint& GridOf(int id) const
    {
        if (id >= core_.CrossingBase())
            return core_.CrossingAt(id).grid_;
        return id >= core_.b_.offset_ ? core_.b_.grid_[id - core_.b_.offset_] : core_.a_.grid_[id];
    }

    /// Point of a global id in the plane of the projection.
    CsgGridPoint2 Point(int id) const { return CsgExact::Project(GridOf(id), dropAxis_, flip_); }

    /// Coordinate of a global id on the dropped axis.
    CsgGridInt Dropped(int id) const
    {
        const CsgGridPoint& p = GridOf(id);
        return dropAxis_ == 0 ? p.x_ : (dropAxis_ == 1 ? p.y_ : p.z_);
    }

    /// Project the components of a 3D vector as CsgExact::Project does.
    void ProjectVector(double x, double y, double z, double& u, double& v) const
    {
        const double c[3] = {x, y, z};
        u = c[(dropAxis_ + 1) % 3];
        v = c[(dropAxis_ + 2) % 3];
        if (flip_)
            ea::swap(u, v);
    }

    /// Choose the projection of the triangle to a coordinate plane. Drop the axis of the largest exact normal
    /// component. Flip the projection if that component is negative, so the face order is counterclockwise.
    /// If the corners of the triangle are collinear on the grid, use the first triangle of the face with a normal
    /// that is not zero. If all the triangles of the face are collinear on the grid, drop the z axis.
    void ChooseProjection(int face, int triangle)
    {
        dropAxis_ = 2;
        flip_ = false;
        if (ProjectionOf(triangle))
            return;
        const auto starts = mesh_.solid_.GetTriangleStart();
        for (unsigned other = starts[face]; other < starts[face + 1]; ++other)
        {
            if (static_cast<int>(other) != triangle && ProjectionOf(static_cast<int>(other)))
                return;
        }
    }

    /// Set the projection from the exact normal of the grid corners of a triangle. Return false if the normal is zero.
    bool ProjectionOf(int triangle)
    {
        const CsgNormal normal = CsgExact::Normal(mesh_.grid_[mesh_.triangleIds_[triangle * 3]],
            mesh_.grid_[mesh_.triangleIds_[triangle * 3 + 1]], mesh_.grid_[mesh_.triangleIds_[triangle * 3 + 2]]);
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
        dropAxis_ = best;
        flip_ = sign < 0;
        return true;
    }

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

    /// Magnitude of the area of a loop in double. It orders loops by size only. No topology depends on it.
    double AreaMagnitude(int loop)
    {
        const CsgGridPoint2* points = LoopPoints(loop);
        const int count = s_.pool_.Count(loop);
        double area = 0.0;
        for (int i = 1; i + 1 < count; ++i)
        {
            const CsgGridPoint2 p = CsgExact::Difference(points[i], points[0]);
            const CsgGridPoint2 q = CsgExact::Difference(points[i + 1], points[0]);
            area += static_cast<double>(p.u_) * q.v_ - static_cast<double>(p.v_) * q.u_;
        }
        return std::fabs(area);
    }

    /// Twice the signed area of a closed cycle whose points are crossings at one vertex of the other operand.
    /// The vertex lies on the plane of the triangle. The positions of such crossings are equal on the grid.
    /// The perturbation moves the crossing of the edge low-high to the vertex plus k * (high - low) / D.
    /// D is the exact difference of the crossing, and k is one factor for all the crossings of the triangle.
    /// The area of the offsets (high - low) / D has the sign of the perturbed area.
    /// Return false if a point of the cycle is not such a crossing.
    ///
    /// The area is in double, so its sign is not exact. The exact area is S / P, with P the product of all D and
    /// S the sum over the edges i of Cross(g_i, g_i+1) times the product of the D_j with j not i and not i + 1.
    /// The g are the grid differences high - low.
    /// For n crossings, each term multiplies n - 2 volumes of up to 2^93 and a minor of 2^62. The expansion length
    /// grows with n, and CsgExact has no helper for such sums. So the exact sign is not cheap. The double area is
    /// a tie break of a curve of zero size only. The exact AreaSign decides all the other cases.
    bool PerturbedArea(int loop, double& area) const
    {
        const int* cycle = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        const CsgGridPoint& t0 = mesh_.grid_[mesh_.triangleIds_[triangle_ * 3]];
        const Mesh& edges = mesh_.isSecond_ ? core_.a_ : core_.b_;
        int vertex = -1;
        const auto offset = [&](int id, double& u, double& v)
        {
            if (id < core_.CrossingBase())
                return false;
            // A merged crossing has one record for each key. Use the record of an edge through this triangle.
            const Crossing* found = core_.FindRecord(id,
                [this](const Crossing& c) { return c.key_.triangle_ == triangle_ && c.key_.edgeInSecond_ != mesh_.isSecond_; });
            if (!found)
                return false;
            const Crossing& crossing = *found;
            const CsgCrossingKey& key = crossing.key_;
            const double difference = crossing.difference_.Approximate();
            if (difference == 0.0)
                return false;
            const int onPlane = crossing.lowVolume_.Sign() == 0 ? key.low_
                : CsgExact::Dot(crossing.normal_, t0, edges.grid_[key.high_]).Sign() == 0 ? key.high_ : -1;
            if (onPlane < 0 || (vertex >= 0 && onPlane != vertex))
                return false;
            vertex = onPlane;
            const CsgGridPoint& low = edges.grid_[key.low_];
            const CsgGridPoint& high = edges.grid_[key.high_];
            ProjectVector((static_cast<double>(high.x_) - low.x_) / difference,
                (static_cast<double>(high.y_) - low.y_) / difference, (static_cast<double>(high.z_) - low.z_) / difference,
                u, v);
            return true;
        };
        area = 0.0;
        double u0 = 0.0, v0 = 0.0;
        if (!offset(cycle[0], u0, v0))
            return false;
        double pu = u0, pv = v0;
        for (int i = 1; i <= count; ++i)
        {
            double qu = u0, qv = v0;
            if (i < count && !offset(cycle[i], qu, qv))
                return false;
            area += pu * qv - qu * pv;
            pu = qu;
            pv = qv;
        }
        return true;
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
        if (s_.cycle_.size() < 3)
            return;
        const int loop = s_.pool_.Add(s_.cycle_.data(), static_cast<int>(s_.cycle_.size()));
        (touchesBoundary ? s_.cycles_ : s_.closedCycles_).push_back(loop);
    }

    /// The edge that follows u -> v around the region on its left. Return -1 if there is none.
    int Next(int u, int v)
    {
        const List* branches = s_.curve_.Find(v);
        const List* outs = s_.boundaryOut_.Find(v);
        if (outs)
        {
            if (outs->count_ == 1 && !branches)
                return s_.boundaryOut_.At(outs->head_).to_;
            if (outs->count_ == 1 && branches->count_ == 1)
            {
                return s_.boundaryEdges_.Contains(DirectedEdgeKey(u, v)) ? s_.curve_.At(branches->head_).to_
                                                                         : s_.boundaryOut_.At(outs->head_).to_;
            }
            return Turn(u, v, outs, branches);
        }
        if (!branches)
            return -1;
        if (branches->count_ == 2)
        {
            const int first = s_.curve_.At(branches->head_).to_;
            return first == u ? s_.curve_.At(s_.curve_.At(branches->head_).next_).to_ : first;
        }
        return Turn(u, v, nullptr, branches);
    }

    /// Among the outgoing directions at v, the first one clockwise from the way back to u.
    /// The outgoing directions are the boundary targets, then the curve targets.
    ///
    /// The directions around v have a cyclic order: the clockwise angle, then the rank for equal directions.
    /// The result is the next direction after v -> u in this order. So a direction equal to v -> u comes first
    /// if its rank is higher than the rank of u, and last if its rank is lower. The order is exact.
    ///
    /// The rank of a curve neighbor is its global id. Clockwise from the incoming boundary edge, the order is the
    /// inside of the triangle and then the outgoing boundary edge. So in a block of equal directions, the incoming
    /// boundary vertex has the lowest rank and an outgoing boundary target has the highest rank. Then each
    /// incoming edge gets a different outgoing edge, and Next is a bijection.
    int Turn(int u, int v, const List* outs, const List* branches)
    {
        constexpr int64_t boundaryRank = int64_t{1} << 40;
        const CsgGridPoint2 zero;
        const CsgGridPoint2 back = Direction(v, u);
        const int64_t backRank = s_.boundaryEdges_.Contains(DirectedEdgeKey(u, v)) ? u - boundaryRank : u;
        const int total = (outs ? outs->count_ : 0) + (branches ? branches->count_ : 0);
        int best = -1;
        int64_t bestRank = 0;
        int bestGroup = 0;
        CsgGridPoint2 bestDirection;
        int firstOutgoing = -1;
        const auto visit = [&](const CsgAdjacency& adjacency, const List* list, int64_t rankOffset)
        {
            if (!list)
                return;
            for (int node = list->head_; node >= 0; node = adjacency.At(node).next_)
            {
                const int w = adjacency.At(node).to_;
                if (firstOutgoing < 0)
                    firstOutgoing = w;
                if (w == u && total > 1)
                    continue;
                const int64_t rank = w + rankOffset;
                const CsgGridPoint2 direction = Direction(v, w);
                // Group 0: the direction of v -> u with a higher rank. Group 2: the same with a lower rank.
                const bool alongBack = CsgExact::Cross2(back, direction) == 0 && CsgExact::Dot2(back, direction) > 0;
                const int group = alongBack ? (rank > backRank ? 0 : 2) : 1;
                bool better = best < 0 || group < bestGroup;
                if (best >= 0 && group == bestGroup)
                {
                    const int order = group == 1 ? CsgExact::DirectionCompare(zero, back, direction, bestDirection) : 0;
                    better = order < 0 || (order == 0 && rank < bestRank);
                }
                if (better)
                {
                    best = w;
                    bestRank = rank;
                    bestGroup = group;
                    bestDirection = direction;
                }
            }
        };
        visit(s_.boundaryOut_, outs, boundaryRank);
        visit(s_.curve_, branches, 0);
        return best >= 0 ? best : firstOutgoing;
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

    /// The two turns of a curve of zero size have the same exact sign 0. Then PerturbedArea decides in double.
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
                // The exact signs decide. Equal signs mean a curve of zero size: the perturbation decides.
                double area0 = AreaSign(m0);
                double area1 = AreaSign(m1);
                if (area0 == area1)
                {
                    double perturbed0 = 0.0;
                    double perturbed1 = 0.0;
                    if (PerturbedArea(m0, perturbed0) && PerturbedArea(m1, perturbed1))
                    {
                        area0 = perturbed0;
                        area1 = perturbed1;
                    }
                }
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
        const int holeTotal = static_cast<int>(s_.holes_.size());
        s_.keys_.resize(holeTotal);
        s_.order_.resize(holeTotal);
        for (int i = 0; i < holeTotal; ++i)
        {
            s_.keys_[i] = AreaMagnitude(s_.holes_[i]);
            s_.order_[i] = i;
        }
        {
            const ea::vector<double>& keys = s_.keys_;
            s_.merge_.resize(holeTotal);
            ea::merge_sort_buffer(s_.order_.begin(), s_.order_.end(), s_.merge_.data(),
                [&keys](int x, int y) { return CompareDouble(keys[y], keys[x]) < 0; });
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
            double smallest = std::numeric_limits<double>::infinity();
            for (int r = 0; r < static_cast<int>(s_.regions_.size()); ++r)
            {
                const int region = s_.regions_[r];
                if (s_.pool_.Count(region) == holeSize && SameSetAsTwin(s_.pool_.Data(region), holeSize))
                    continue;
                if (fallback < 0)
                    fallback = r;
                if (!HoleInside(region, hole))
                    continue;
                const double area = AreaMagnitude(region);
                if (area < smallest)
                {
                    smallest = area;
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
            const int local = localVertex(id - mesh.offset_);
            if (local < 0)
                return false;
            vertices.push_back(source.vertices_[local]);
            continue;
        }
        // A merged crossing has one record for each key. Use the record that belongs to this face.
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
        vertex.SetPosition(crossing.position_);
        vertices.push_back(vertex);
    }
    output = CsgPolygon(ea::move(vertices), source.shared_);
    output.plane_ = source.plane_;
    return true;
}

void Corefinement::SplitFaces(CsgScratch& scratch)
{
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
        // A collapsed triangle has no segments. A triangle without segments stays whole.
        if (segments.Count(triangle) == 0 || mesh.IsCollapsed(triangle))
            continue;
        if (!splitter.Split(mesh.triangleFace_[triangle], triangle, segments))
        {
            result.state_[triangle] = static_cast<uint8_t>(CsgSplitResult::State::Failed);
            continue;
        }
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

    result.outOfRange_ = core.outOfRange_;
    result.positions_.reserve(core.crossings_.size());
    result.keys_.reserve(core.crossings_.size());
    for (const Crossing& crossing : core.crossings_)
    {
        result.positions_.push_back(crossing.position_);
        result.keys_.push_back(crossing.key_);
    }
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
            result.segments_[i].p_ = core.segmentsA_.At(i).p_ - core.CrossingBase();
            result.segments_[i].q_ = core.segmentsA_.At(i).q_ - core.CrossingBase();
        }
    }
    for (unsigned list = 0; list < core.segmentsB_.ListCount(); ++list)
    {
        for (int i = core.segmentsB_.Head(static_cast<int>(list)); i >= 0; i = core.segmentsB_.At(i).next_)
            result.segments_[i].triangleB_ = static_cast<int>(list);
    }
    result.curveEdges_.reserve(core.curveEdges_.Size());
    for (const auto& entry : core.curveEdges_)
    {
        result.curveEdges_.push_back(ea::make_pair(
            entry.value_.first - core.CrossingBase(), entry.value_.second - core.CrossingBase()));
    }
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
    core.SplitFaces(scratch);

    result.outOfRange_ = core.outOfRange_;
    result.vertexCountA_ = core.a_.VertexCount();
    result.vertexCountB_ = core.b_.VertexCount();
    result.crossingBase_ = core.CrossingBase();
    const int total = core.CrossingBase() + static_cast<int>(core.crossings_.size());
    result.positions_.reserve(total);
    for (int id = 0; id < total; ++id)
        result.positions_.push_back(PointPosition(core, id));
    for (const auto& entry : core.curveEdges_)
        result.curveEdges_.push_back(entry.value_);
    FillSplitOperand(core, core.a_, core.splitA_, result.a_);
    FillSplitOperand(core, core.b_, core.splitB_, result.b_);
    return result;
}

} // namespace CsgInternal

} // namespace Urho3D
