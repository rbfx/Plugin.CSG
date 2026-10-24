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

struct Point2
{
    double x_{};
    double y_{};
};

double DistanceSquared(const Point2& a, const Point2& b)
{
    return (a.x_ - b.x_) * (a.x_ - b.x_) + (a.y_ - b.y_) * (a.y_ - b.y_);
}

double Cross2(const Point2& o, const Point2& a, const Point2& b)
{
    return (a.x_ - o.x_) * (b.y_ - o.y_) - (a.y_ - o.y_) * (b.x_ - o.x_);
}

bool SegmentsCross(const Point2& a0, const Point2& a1, const Point2& b0, const Point2& b1)
{
    const double d1 = Cross2(a0, a1, b0);
    const double d2 = Cross2(a0, a1, b1);
    const double d3 = Cross2(b0, b1, a0);
    const double d4 = Cross2(b0, b1, a1);
    return (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0);
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

/// A full turn in double. The macro M_PI is a float constant.
constexpr double TAU = 6.283185307179586;

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

        const CsgPolygon& source = *mesh_.solid_.GetFace(face);
        MakeAxesFromNormal(source.plane_.normal_, axisU_, axisV_);
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

    Vector2 ProjectFloat(int id) const
    {
        const Vector3 p = PointPosition(core_, id);
        return Vector2(p.DotProduct(axisU_), p.DotProduct(axisV_));
    }

    static double Angle(const Vector2& d) { return std::atan2(static_cast<double>(d.y_), static_cast<double>(d.x_)); }

    /// Twice the signed area in the face plane. The math is in double, relative to the first vertex.
    double SignedArea(int loop) const
    {
        const int* cycle = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        const Vector3 origin = PointPosition(core_, cycle[0]);
        const auto along = [&origin](const Vector3& p, const Vector3& axis)
        {
            return (static_cast<double>(p.x_) - origin.x_) * axis.x_ + (static_cast<double>(p.y_) - origin.y_) * axis.y_
                + (static_cast<double>(p.z_) - origin.z_) * axis.z_;
        };
        double area = 0.0;
        for (int i = 0; i < count; ++i)
        {
            const Vector3 p = PointPosition(core_, cycle[i]);
            const Vector3 q = PointPosition(core_, cycle[(i + 1) % count]);
            area += along(p, axisU_) * along(q, axisV_) - along(q, axisU_) * along(p, axisV_);
        }
        return area;
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
    int Turn(int u, int v, const List* outs, const List* branches)
    {
        const Vector2 origin = ProjectFloat(v);
        const double back = Angle(ProjectFloat(u) - origin);
        const int total = (outs ? outs->count_ : 0) + (branches ? branches->count_ : 0);
        int best = -1;
        int firstOutgoing = -1;
        double bestSweep = std::numeric_limits<double>::infinity();
        const auto visit = [&](const CsgAdjacency& adjacency, const List* list)
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
                double sweep = back - Angle(ProjectFloat(w) - origin);
                // The angles are in [-pi, pi], so two additions are enough. The bound keeps a NaN away from a hang.
                for (int k = 0; k < 4 && sweep <= 0.0; ++k)
                    sweep += TAU;
                if (sweep < bestSweep)
                {
                    bestSweep = sweep;
                    best = w;
                }
            }
        };
        visit(s_.boundaryOut_, outs);
        visit(s_.curve_, branches);
        return best >= 0 ? best : firstOutgoing;
    }

    /// A closed curve inside the triangle is traced once in each direction. The turn that encloses more area is the
    /// island and the other is the hole around it, even when rounding gives both the same sign.
    void ClassifyClosedCycles()
    {
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
                if (SignedArea(m0) >= SignedArea(m1))
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
                (SignedArea(loop) > 0.0 ? s_.cycles_ : s_.holes_).push_back(loop);
            }
        }
    }

    // ---- Holes. The math is in double, relative to the first vertex of the first hole.

    Point2 ProjectDouble(int id) const
    {
        const Vector3 p = PointPosition(core_, id);
        const double dx = static_cast<double>(p.x_) - origin_.x_;
        const double dy = static_cast<double>(p.y_) - origin_.y_;
        const double dz = static_cast<double>(p.z_) - origin_.z_;
        return {dx * axisU_.x_ + dy * axisU_.y_ + dz * axisU_.z_, dx * axisV_.x_ + dy * axisV_.y_ + dz * axisV_.z_};
    }

    double Area(int loop) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        double area = 0.0;
        for (int i = 0; i < count; ++i)
        {
            const Point2 p = ProjectDouble(ids[i]);
            const Point2 q = ProjectDouble(ids[(i + 1) % count]);
            area += p.x_ * q.y_ - q.x_ * p.y_;
        }
        return area * 0.5;
    }

    bool Contains(int loop, const Point2& point) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        bool inside = false;
        for (int i = 0, j = count - 1; i < count; j = i++)
        {
            const Point2 p = ProjectDouble(ids[i]);
            const Point2 q = ProjectDouble(ids[j]);
            if ((p.y_ > point.y_) != (q.y_ > point.y_)
                && point.x_ < (q.x_ - p.x_) * (point.y_ - p.y_) / (q.y_ - p.y_) + p.x_)
                inside = !inside;
        }
        return inside;
    }

    bool Blocked(int loop, const Point2& from, const Point2& to, int endA, int endB) const
    {
        const int* ids = s_.pool_.Data(loop);
        const int count = s_.pool_.Count(loop);
        for (int i = 0; i < count; ++i)
        {
            const int p = ids[i];
            const int q = ids[(i + 1) % count];
            if (p == endA || p == endB || q == endA || q == endB)
                continue;
            if (SegmentsCross(from, to, ProjectDouble(p), ProjectDouble(q)))
                return true;
        }
        return false;
    }

    /// Index pair of the closest vertices of the loops.
    ea::pair<int, int> Nearest(int outer, int hole) const
    {
        const int* outerIds = s_.pool_.Data(outer);
        const int* holeIds = s_.pool_.Data(hole);
        ea::pair<int, int> best(0, 0);
        double bestDistance = std::numeric_limits<double>::infinity();
        for (int i = 0; i < s_.pool_.Count(outer); ++i)
        {
            for (int j = 0; j < s_.pool_.Count(hole); ++j)
            {
                const double distance = DistanceSquared(ProjectDouble(outerIds[i]), ProjectDouble(holeIds[j]));
                if (distance < bestDistance)
                {
                    bestDistance = distance;
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
        const Point2 start = ProjectDouble(holeIds[from]);
        s_.order_.resize(outerCount);
        s_.keys_.resize(outerCount);
        for (int i = 0; i < outerCount; ++i)
        {
            s_.order_[i] = i;
            s_.keys_[i] = DistanceSquared(ProjectDouble(outerIds[i]), start);
        }
        const ea::vector<double>& keys = s_.keys_;
        SortIds(s_.order_, s_.merge_, [&keys](int x, int y) { return CompareDouble(keys[x], keys[y]); });
        for (const int candidate : s_.order_)
        {
            if (candidate == avoidOuter)
                continue;
            const Point2 target = ProjectDouble(outerIds[candidate]);
            // Holes not yet joined also lie in the region, so a bridge must not cut through them either.
            bool blocked = Blocked(outer, start, target, outerIds[candidate], holeIds[from])
                || Blocked(hole, start, target, outerIds[candidate], holeIds[from]);
            for (int other = pendingFrom_; !blocked && other < static_cast<int>(s_.holes_.size()); ++other)
                blocked = Blocked(s_.holes_[other], start, target, outerIds[candidate], holeIds[from]);
            if (blocked)
                continue;
            if (avoidOuter >= 0
                && SegmentsCross(start, target, ProjectDouble(holeIds[avoidHole]), ProjectDouble(outerIds[avoidOuter])))
                continue;
            const Point2 midpoint{(start.x_ + target.x_) * 0.5, (start.y_ + target.y_) * 0.5};
            if (!Contains(outer, midpoint) || Contains(hole, midpoint))
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
            const double x = ProjectDouble(holeIds[i]).x_;
            if (x < ProjectDouble(holeIds[low]).x_)
                low = i;
            if (x > ProjectDouble(holeIds[high]).x_)
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
        origin_ = PointPosition(core_, s_.pool_.Data(s_.holes_[0])[0]);

        // Outer holes first, so a hole inside an island is placed after the island exists. The sort is stable.
        const int holeTotal = static_cast<int>(s_.holes_.size());
        s_.keys_.resize(holeTotal);
        s_.order_.resize(holeTotal);
        for (int i = 0; i < holeTotal; ++i)
        {
            s_.keys_[i] = std::fabs(Area(s_.holes_[i]));
            s_.order_[i] = i;
        }
        {
            const ea::vector<double>& keys = s_.keys_;
            SortIds(s_.order_, s_.merge_, [&keys](int x, int y) { return CompareDouble(keys[y], keys[x]); });
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
            // Probe just to the left of the first hole edge, which is the side of the region around it.
            const Point2 a = ProjectDouble(holeIds[0]);
            const Point2 b = ProjectDouble(holeIds[1]);
            const Point2 probe{(a.x_ + b.x_) * 0.5 - (b.y_ - a.y_) * 1e-3, (a.y_ + b.y_) * 0.5 + (b.x_ - a.x_) * 1e-3};
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
                if (!Contains(region, probe))
                    continue;
                const double area = std::fabs(Area(region));
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
    Vector3 axisU_;
    Vector3 axisV_;
    int edgeCount_{};
    bool failed_{};
    int holeCount_{};
    int pendingFrom_{};
    Vector3 origin_;
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
        const Crossing& crossing = core.CrossingAt(id);
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
