// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCorefine.h"

#include "Urho3D/CSG/CsgExactFilter.h"
#include "Urho3D/CSG/CsgSimd.h"
#include <Urho3D/IO/Log.h>

#include <EASTL/algorithm.h>
#include <EASTL/unique_ptr.h>

#include <cmath>

namespace Urho3D
{

namespace CsgInternal
{

namespace
{

/// Largest grid coordinate after the clamp of an out of range operand.
constexpr float CLAMPED_COORDINATE = 65535.0f;

/// Tolerance of the first comparison of two crossings on an edge.
constexpr double EDGE_ORDER_TOLERANCE = 1e-9;

/// Number of ids that the sort of the crossings of an edge sorts by insertion.
constexpr int INSERTION_SORT_LIMIT = 16;

/// Convert a 128 bit number to double. The function has the same type as CsgInt128.
#if CSG_HAS_NATIVE_INT128
double ToDouble(const CsgInt128& value)
{
    return static_cast<double>(value);
}
#else
double ToDouble(const CsgInt128& value)
{
    return CsgInt128PortableToDouble(value);
}
#endif

int SignOfDouble(double value)
{
    return value > 0.0 ? 1 : -1;
}

/// Snap a position to the grid. A coordinate outside of the range is clamped, a NaN becomes zero.
CsgGridPoint SnapToGrid(const Vector3& position, bool& outOfRange, bool& invalid)
{
    float values[3] = {position.x_, position.y_, position.z_};
    for (float& value : values)
    {
        if (!std::isfinite(value))
        {
            invalid = true;
            outOfRange = true;
            value = 0.0f;
        }
        else if (!(std::fabs(value) < CSG_EXACT_COORDINATE_LIMIT))
        {
            outOfRange = true;
            value = value < 0.0f ? -CLAMPED_COORDINATE : CLAMPED_COORDINATE;
        }
    }
    return CsgGridPoint::From(Vector3(values[0], values[1], values[2]));
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
    const auto diff = [](int64_t a, int64_t b) { return static_cast<double>(a - b); };
    CsgDouble4 detVector;
    const int decided = Filter(CsgDouble4::Splat(diff(t1.x_, t0.x_)), CsgDouble4::Splat(diff(t1.y_, t0.y_)),
        CsgDouble4::Splat(diff(t1.z_, t0.z_)), CsgDouble4::Splat(diff(t2.x_, t0.x_)),
        CsgDouble4::Splat(diff(t2.y_, t0.y_)), CsgDouble4::Splat(diff(t2.z_, t0.z_)),
        CsgDouble4::Create(diff(d0.x_, t0.x_), diff(d1.x_, t0.x_), diff(d2.x_, t0.x_), 0.0),
        CsgDouble4::Create(diff(d0.y_, t0.y_), diff(d1.y_, t0.y_), diff(d2.y_, t0.y_), 0.0),
        CsgDouble4::Create(diff(d0.z_, t0.z_), diff(d1.z_, t0.z_), diff(d2.z_, t0.z_), 0.0), detVector);
    double det[4];
    detVector.Store(det);
    sides[0] = (decided & 1) != 0 ? SignOfDouble(det[0]) : CsgExact::Orient(t0, t1, t2, d0, mask);
    sides[1] = (decided & 2) != 0 ? SignOfDouble(det[1]) : CsgExact::Orient(t0, t1, t2, d1, mask);
    sides[2] = (decided & 4) != 0 ? SignOfDouble(det[2]) : CsgExact::Orient(t0, t1, t2, d2, mask);
}

} // namespace

double CsgInt128PortableToDouble(const CsgInt128Portable& value)
{
    const CsgDetail::Magnitude128 magnitude = CsgDetail::GetMagnitude(value);
    if (magnitude.sign_ == 0)
        return 0.0;
    const uint64_t hi = magnitude.hi_;
    const uint64_t lo = magnitude.lo_;
    double result;
    if (hi == 0)
    {
        // A 64 bit number converts with one rounding.
        result = static_cast<double>(lo);
    }
    else
    {
        // Keep the top 64 bits of the magnitude. A double has 53 bits, so bit 0 of the top part is far below the
        // rounding position. A sticky bit in bit 0 keeps the round to nearest even correct.
        int shift = 0;
        for (uint64_t rest = hi; rest != 0; rest >>= 1)
            ++shift;
        uint64_t top;
        bool sticky;
        if (shift == 64)
        {
            top = hi;
            sticky = lo != 0;
        }
        else
        {
            top = (hi << (64 - shift)) | (lo >> shift);
            sticky = (lo & ((uint64_t{1} << shift) - 1)) != 0;
        }
        if (sticky)
            top |= 1;
        result = std::ldexp(static_cast<double>(top), shift);
    }
    return magnitude.sign_ < 0 ? -result : result;
}

int CompareOnEdge(const Crossing& x, const Crossing& y)
{
    const CsgInt128 dx = x.lowVolume_ - x.highVolume_;
    const CsgInt128 dy = y.lowVolume_ - y.highVolume_;
    // t_x - t_y = (Lx * Dy - Ly * Dx) / (Dx * Dy)
    const int numeratorSign = CsgSignOfProductDifference<CsgInt128>(x.lowVolume_, dy, y.lowVolume_, dx);
    const int denominatorSign = CsgSign(dx) * CsgSign(dy);
    if (numeratorSign != 0)
        return numeratorSign * denominatorSign;

    // Tie: compare delta * (Nx / Dx - Ny / Dy), component by component.
    const int shift = x.key_.edgeInSecond_ ? 1 : -1;
    const CsgInt128 components[3][2] = {
        {x.normal_.x_, y.normal_.x_}, {x.normal_.y_, y.normal_.y_}, {x.normal_.z_, y.normal_.z_}};
    for (const auto& component : components)
    {
        const int differenceSign = CsgSignOfProductDifference<CsgInt128>(component[0], dy, component[1], dx);
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

    grid_.resize(loops_.positions_.size());
    bool anyInvalid = false;
    for (unsigned id = 0; id < grid_.size(); ++id)
    {
        bool invalid = false;
        grid_[id] = SnapToGrid(loops_.positions_[id], outOfRange_, invalid);
        if (invalid && !anyInvalid)
        {
            anyInvalid = true;
            invalid_.assign(grid_.size(), 0);
        }
        if (invalid)
            invalid_[id] = 1;
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
        URHO3D_LOGWARNING("CsgCorefine: an operand has a coordinate outside of +-{}. The exact math is not valid.",
            static_cast<int>(CSG_EXACT_COORDINATE_LIMIT));
    }
    segmentsA_.Reset(static_cast<unsigned>(a_.triangleFace_.size()));
    segmentsB_.Reset(static_cast<unsigned>(b_.triangleFace_.size()));
}

void Corefinement::CollectCrossings(CsgScratch& scratch)
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
        treeA.CollectOverlaps(treeB, epsilon_, scratch.facePairs_, scratch.stack_);
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

    const CsgInt128 lowVolume = CsgExact::Volume(t0, t1, t2, edges.grid_[key.low_]);
    const CsgInt128 highVolume = CsgExact::Volume(t0, t1, t2, edges.grid_[key.high_]);
    double t = 0.5;
    if (lowVolume != highVolume)
    {
        const double low = ToDouble(lowVolume);
        t = low / (low - ToDouble(highVolume));
        t = std::isnan(t) ? 0.5 : (t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t));
    }
    const Vector3& lowPosition = edges.loops_.positions_[key.low_];
    const Vector3& highPosition = edges.loops_.positions_[key.high_];
    const int id = CrossingBase() + static_cast<int>(crossings_.size());
    idRef = id;

    Crossing& crossing = crossings_.emplace_back();
    crossing.position_ = lowPosition + (highPosition - lowPosition) * static_cast<float>(t);
    crossing.t_ = t;
    crossing.key_ = key;
    crossing.lowVolume_ = lowVolume;
    crossing.highVolume_ = highVolume;
    crossing.normal_ = CsgExact::Normal(t0, t1, t2);

    bool listExisted = false;
    EdgeList& list = edgeCrossings_[key.edgeInSecond_ ? 1 : 0].FindOrAdd(
        CsgDirectedKey(static_cast<unsigned>(key.low_), static_cast<unsigned>(key.high_)), listExisted);
    const int node = static_cast<int>(edgeNodes_.size());
    edgeNodes_.push_back(EdgeNode{id, -1});
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
    for (int node = list->head_; node >= 0; node = edgeNodes_[node].next_)
        output.push_back(edgeNodes_[node].id_);

    const int baseId = CrossingBase();
    const ea::vector<Crossing>& crossings = crossings_;
    SortIds(output, scratch.merge_,
        [&crossings, baseId](int x, int y)
        {
            const Crossing& cx = crossings[x - baseId];
            const Crossing& cy = crossings[y - baseId];
            // Doubles separate all but nearly coincident crossings. Those are ordered exactly.
            if (std::fabs(cx.t_ - cy.t_) > EDGE_ORDER_TOLERANCE)
                return cx.t_ < cy.t_ ? -1 : 1;
            const int order = CompareOnEdge(cx, cy);
            return order != 0 ? order : (x < y ? -1 : (x > y ? 1 : 0));
        });
    if (from > to)
        ea::reverse(output.begin(), output.end());
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

} // namespace CsgInternal

} // namespace Urho3D
