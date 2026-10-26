// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

// This header is internal. Do not export these types. Do not use them across a module boundary.

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/CsgExact.h"
#include "Urho3D/CSG/CsgFaceBvh.h"
#include "Urho3D/CSG/CsgFlatMap.h"
#include "Urho3D/CSG/CsgSolid.h"
#include <Urho3D/Math/Vector3.h>

#include <EASTL/span.h>
#include <EASTL/utility.h>
#include <EASTL/vector.h>

#include <cstdint>

namespace Urho3D
{

namespace CsgInternal
{

/// Coordinates with an absolute value of this limit or more are outside the range of the exact math.
constexpr float CSG_EXACT_COORDINATE_LIMIT = CsgGridCoordinateLimit;

/// Whether crossings that round to one grid point share one id. The merge is off now.
/// The merge needs every position on the grid. The solids now store their grid points, and a later step turns it on.
constexpr bool MergeCrossingsOnGrid = false;

/// Crossing of an edge of one operand through a triangle of the other.
struct Crossing
{
    /// The crossing of the edge between the grid points of its ends, rounded to the grid.
    /// position_ is grid / scale, see CsgGridPoint::ToPosition. If an operand is out of range, the grid points
    /// are clamped and position_ is the crossing on the float edge.
    CsgGridPoint grid_;
    Vector3 position_;
    /// Parameter along the edge from its lower to its higher welded id.
    double t_{};
    CsgCrossingKey key_;
    /// Exact plane volume of the lower edge end against the crossed triangle. It orders the crossings on one edge.
    CsgExactVolume lowVolume_{};
    /// The value lowVolume - highVolume, which equals normal . (low - high). It orders the crossings on one edge.
    CsgExactVolume difference_{};
    /// Normal of the crossed triangle. It breaks ties between crossings on one edge.
    CsgNormal normal_{};
    /// Index of the next alias with the same id, or -1. A crossing of a grid point holds the first one.
    /// An alias holds the next one. An alias keeps the order data of a key that merged into the crossing.
    int alias_{-1};
};

/// Order two crossings of one edge by their parameter from its lower to its higher end.
/// The order is exact. The displacement of the second operand breaks the ties. Return -1, 0 or 1.
int CompareOnEdge(const Crossing& x, const Crossing& y);

/// Welded view of an operand: ids for each triangle corner and grid positions of the ids.
/// The grid points are the ones that the solid stores. The mesh does not snap a position.
class Mesh
{
public:
    /// The solid must live as long as the mesh. The offset is the first global id of the mesh vertices.
    Mesh(const CsgSolid& solid, bool isSecond, int offset);

    int VertexCount() const { return static_cast<int>(grid_.size()); }
    /// Return true if the triangle has no area because corners weld together, or if a corner is not finite.
    bool IsCollapsed(int triangle) const;

    const CsgSolid& solid_;
    /// Welded loops of the solid. The spans stay valid as long as the solid lives.
    CsgExactLoops loops_;
    bool isSecond_{};
    int offset_{};
    /// Face of each triangle.
    ea::vector<int> triangleFace_;
    /// Welded id of each triangle corner, three for each triangle.
    ea::vector<int> triangleIds_;
    /// Grid position of each welded id. The span is the grid of the solid.
    ea::span<const CsgGridPoint> grid_;
    /// One byte for each welded id. Not zero if the position is not finite. The vector is empty if all are finite.
    ea::vector<uint8_t> invalid_;
    /// True if a position is not finite or has a coordinate outside the range of the exact math.
    bool outOfRange_{};
};

/// One curve segment in a pool list. The ends are global crossing ids.
struct CsgSegment
{
    int p_{};
    int q_{};
    int next_{-1};
};

/// Lists of segments, one list for each triangle. All the lists share one pool.
/// An append goes to the tail, so the iteration order is the order of the appends.
class CsgSegmentLists
{
public:
    /// Make count empty lists. The memory stays.
    void Reset(unsigned count)
    {
        head_.assign(count, -1);
        tail_.assign(count, -1);
        count_.assign(count, 0);
        pool_.clear();
    }

    void Append(int list, int p, int q)
    {
        const int index = static_cast<int>(pool_.size());
        pool_.push_back(CsgSegment{p, q, -1});
        if (tail_[list] < 0)
            head_[list] = index;
        else
            pool_[tail_[list]].next_ = index;
        tail_[list] = index;
        ++count_[list];
    }

    unsigned ListCount() const { return head_.size(); }
    /// Return the index of the first segment of the list in the pool, or -1.
    int Head(int list) const { return head_[list]; }
    /// Return the number of segments in the list.
    int Count(int list) const { return count_[list]; }
    /// Return a segment of the pool. The field next_ is the next index in the list, or -1.
    const CsgSegment& At(int index) const { return pool_[index]; }
    unsigned TotalCount() const { return pool_.size(); }

private:
    ea::vector<int> head_;
    ea::vector<int> tail_;
    ea::vector<int> count_;
    ea::vector<CsgSegment> pool_;
};

/// Pool of loops of global ids. Each loop is a span of one shared vector.
/// An Add can move the ids. Do not pass a pointer into the pool to Add, and do not keep a Data pointer over an Add.
struct CsgLoopPool
{
    struct Span
    {
        int start_{};
        int count_{};
    };

    void Clear()
    {
        ids_.clear();
        spans_.clear();
    }

    /// Copy the ids to the pool. Return the index of the new loop.
    int Add(const int* data, int count)
    {
        const int index = static_cast<int>(spans_.size());
        spans_.push_back(Span{static_cast<int>(ids_.size()), count});
        ids_.insert(ids_.end(), data, data + count);
        return index;
    }

    int Count(int loop) const { return spans_[loop].count_; }
    const int* Data(int loop) const { return ids_.data() + spans_[loop].start_; }

    ea::vector<int> ids_;
    ea::vector<Span> spans_;
};

/// Directed links between ids. The targets of one id keep the order of the appends.
class CsgAdjacency
{
public:
    struct List
    {
        int head_{-1};
        int tail_{-1};
        int count_{};
    };
    struct Node
    {
        int to_{};
        int next_{-1};
    };

    void Clear()
    {
        lists_.Clear();
        nodes_.clear();
    }

    void Add(int from, int to)
    {
        bool existed = false;
        List& list = lists_.FindOrAdd(static_cast<uint64_t>(static_cast<uint32_t>(from)), existed);
        const int node = static_cast<int>(nodes_.size());
        nodes_.push_back(Node{to, -1});
        if (list.tail_ < 0)
            list.head_ = node;
        else
            nodes_[list.tail_].next_ = node;
        list.tail_ = node;
        ++list.count_;
    }

    /// Return the list of the id or null. An append invalidates the pointer.
    const List* Find(int from) const { return lists_.Find(static_cast<uint64_t>(static_cast<uint32_t>(from))); }
    const Node& At(int node) const { return nodes_[node]; }

private:
    CsgFlatMap<uint64_t, List, CsgPackedKeyHash> lists_;
    ea::vector<Node> nodes_;
};

/// Reusable vectors of the split of a triangle.
struct CsgSplitScratch
{
    /// Boundary of the triangle: corners and the crossings on the edges, as global ids.
    ea::vector<int> boundary_;
    ea::vector<int> edgePoints_;
    CsgAdjacency boundaryOut_;
    CsgAdjacency curve_;
    CsgFlatSet<uint64_t, CsgPackedKeyHash> boundaryEdges_;
    CsgFlatSet<uint64_t, CsgPackedKeyHash> visited_;
    /// All the loops of one triangle. The lists below hold loop indices.
    CsgLoopPool pool_;
    ea::vector<int> cycles_;
    ea::vector<int> closedCycles_;
    ea::vector<int> holes_;
    ea::vector<int> regions_;
    /// One byte for each entry of regions_. Not zero if a bridge to a hole made the region.
    ea::vector<uint8_t> regionBridged_;
    ea::vector<int> cycle_;
    ea::vector<int> temp1_;
    ea::vector<int> temp2_;
    ea::vector<int> twin_;
    ea::vector<int> twinTest_;
    /// Grouping of the closed cycles by their sorted ids.
    ea::vector<int> closedSorted_;
    ea::vector<int> closedSortedStart_;
    ea::vector<uint64_t> closedHash_;
    ea::vector<int> groupHead_;
    ea::vector<int> groupTail_;
    ea::vector<int> groupSameHash_;
    ea::vector<int> memberNext_;
    CsgFlatMap<uint64_t, int, CsgPackedKeyHash> groupByHash_;
    /// Sorted ids of the curve network that is connected to the triangle boundary.
    ea::vector<int> attachedIds_;
    ea::vector<int> attachedNew_;
    /// Sort buffers.
    ea::vector<double> keys_;
    ea::vector<int> order_;
    ea::vector<int> merge_;
    /// Projected grid points of a loop for the exact tests.
    ea::vector<CsgGridPoint2> points_;
};

/// Result of the split of the triangles of one operand.
///
/// A triangle without curve segments is Whole: the boolean keeps it as its three corners.
/// A Failed triangle has segments, but the trace of its regions did not close. The boolean keeps it whole too.
/// A Split triangle has regions. A region is a loop of global ids in the orientation of the face.
/// The loops have at least three ids. A region joined to a hole by a bridge is flagged. Holes are never output alone.
struct CsgSplitResult
{
    enum class State : uint8_t
    {
        Whole,
        Split,
        Failed
    };

    void Reset(unsigned triangleCount)
    {
        state_.assign(triangleCount, static_cast<uint8_t>(State::Whole));
        holeCount_.assign(triangleCount, 0);
        triangleRegionStart_.assign(triangleCount + 1, 0);
        regionStart_.assign(1, 0);
        ids_.clear();
        bridged_.clear();
    }

    int RegionCount() const { return static_cast<int>(bridged_.size()); }
    /// Number of ids of a region.
    int RegionSize(int region) const { return regionStart_[region + 1] - regionStart_[region]; }
    const int* RegionIds(int region) const { return ids_.data() + regionStart_[region]; }
    State GetState(int triangle) const { return static_cast<State>(state_[triangle]); }

    /// State of each triangle.
    ea::vector<uint8_t> state_;
    /// Number of closed curves in a triangle that needed a bridge. It is zero if the triangle is not Split.
    ea::vector<int> holeCount_;
    /// The regions of triangle t are [triangleRegionStart_[t], triangleRegionStart_[t + 1]), in the order of the C# reference.
    ea::vector<int> triangleRegionStart_;
    /// Region r owns [regionStart_[r], regionStart_[r + 1]) in ids_.
    ea::vector<int> regionStart_;
    /// Global ids of all the region loops.
    ea::vector<int> ids_;
    /// One byte for each region. Not zero if a bridge to a hole made the region.
    ea::vector<uint8_t> bridged_;
};

/// A face of the contact region of a boolean before the classification: a region of a split triangle,
/// a whole triangle of a face that the curve reaches, or a whole face.
struct CsgBooleanElement
{
    CsgSharedPolygon polygon_;
    /// First global id of the polygon vertices in the element id pool. Not used if sourceFace_ is not negative.
    int idStart_{};
    int idCount_{};
    /// The face that the element is as a whole, so the result copies its triangles. It is -1 for any other element.
    /// The ids of such an element are the loop of the face in the mesh.
    int sourceFace_{-1};
    /// True for a region of a split triangle. Only these border the curve.
    bool split_{};
};

/// A face that the boolean keeps. A face with a source face reads its ids from the solid of the operand.
struct CsgKeptFace
{
    CsgSharedPolygon polygon_;
    int idStart_{};
    int idCount_{};
    int sourceFace_{-1};
    /// True if the face comes from the second operand.
    bool second_{};
    /// True if the polygon is the flipped copy of the source face.
    bool flip_{};
    /// True for a region of a split triangle. Only these can be zero-width strips.
    bool split_{};
};

/// One entry for the sort in CancelOpposites.
struct CsgOppositeCandidate
{
    int count_{};
    uint64_t hash_{};
    int index_{};
};

/// Reusable vectors of the classification and of the assembly of a boolean.
struct CsgBooleanScratch
{
    ea::vector<uint8_t> touched_;
    ea::vector<uint8_t> frontier_;
    ea::vector<CsgBooleanElement> elements_;
    ea::vector<int> elementIds_;
    ea::vector<uint8_t> inside_;
    ea::vector<CsgKeptFace> kept_;
    ea::vector<int> keptIds_;
    ea::vector<CsgOppositeCandidate> candidates_;
    ea::vector<uint8_t> removed_;
    /// Classification.
    ea::vector<int> edgeStart_;
    ea::vector<int> loops_;
    CsgFlatMap<uint64_t, int, CsgPackedKeyHash> edgeOwner_;
    ea::vector<int> twins_;
    ea::vector<uint8_t> decided_;
    ea::vector<int> queue_;
    ea::vector<int> patch_;
    ea::vector<unsigned> indices_;
};

/// Reusable vectors. A boolean makes one scratch and passes it to every step.
struct CsgScratch
{
    CsgBooleanScratch boolean_;
    ea::vector<CsgFaceBvh::ItemPair> componentPairs_;
    ea::vector<CsgFaceBvh::ItemPair> facePairs_;
    ea::vector<CsgFaceBvh::ItemPair> stack_;
    /// Merge buffer for the sort of the crossings of an edge.
    ea::vector<int> merge_;
    CsgSplitScratch split_;
};

/// The intersection of two operands: crossing vertices and the curve segments on each triangle.
///
/// Global ids: the vertices of mesh A are [0, A.VertexCount), the vertices of mesh B follow, and the crossings
/// follow them. The first crossing has the id CrossingBase().
class Corefinement
{
public:
    Corefinement(const CsgSolid& a, const CsgSolid& b, float epsilon);

    /// Global id of the first crossing. Mesh vertices have the ids below it.
    int CrossingBase() const { return a_.VertexCount() + b_.VertexCount(); }
    int Global(const Mesh& mesh, int id) const { return mesh.offset_ + id; }
    const Crossing& CrossingAt(int globalId) const { return crossings_[globalId - CrossingBase()]; }
    /// Return the first record of the crossing that satisfies the predicate, or null.
    /// The records are the crossing and the aliases of the keys that merged into it. Each one has its own key.
    template <class Predicate> const Crossing* FindRecord(int globalId, const Predicate& predicate) const
    {
        const Crossing* record = &CrossingAt(globalId);
        while (!predicate(*record))
        {
            if (record->alias_ < 0)
                return nullptr;
            record = &aliases_[record->alias_];
        }
        return record;
    }

    /// Find all the face pairs of the operands in the order of the C# reference and intersect them.
    /// The parallel options only change how the face pairs are found. The order of the pairs stays the same.
    /// A caller inside a work queue task must pass options without a work queue.
    void CollectCrossings(CsgScratch& scratch, const CsgParallelOptions& parallel = {});
    /// Intersect the triangles of two faces. Add the crossings and the segments.
    void IntersectFaces(int faceA, int faceB);
    /// Write the crossings on an edge of the mesh to the output, ordered from the vertex from to the vertex to.
    /// The ids are local welded ids of the mesh.
    void EdgeCrossings(const Mesh& mesh, int from, int to, CsgScratch& scratch, ea::vector<int>& output) const;
    /// Split every triangle of both operands that has curve segments into regions. Fill splitA_ and splitB_.
    /// Call this after CollectCrossings.
    void SplitFaces(CsgScratch& scratch);
    /// Split the triangles of one operand with the segments of its triangles.
    void SplitOperand(const Mesh& mesh, const CsgSegmentLists& segments, CsgScratch& scratch, CsgSplitResult& result) const;

    Mesh a_;
    Mesh b_;
    float epsilon_{};
    /// Crossings by global id minus CrossingBase().
    ea::vector<Crossing> crossings_;
    /// Curve segments on each triangle of the first and the second operand. The lists are by triangle index.
    CsgSegmentLists segmentsA_;
    CsgSegmentLists segmentsB_;
    /// Regions of the split triangles of the first and the second operand. SplitFaces fills them.
    CsgSplitResult splitA_;
    CsgSplitResult splitB_;
    /// Curve segments by their undirected ends, oriented from entering to exiting along n_A x n_B.
    CsgFlatMap<uint64_t, ea::pair<int, int>, CsgPackedKeyHash> curveEdges_;
    /// True if a coordinate of an operand is outside the range of the exact math. The results are not exact then.
    bool outOfRange_{};

private:
    /// Entry in the pool list of the crossings of an edge.
    struct EdgeNode
    {
        /// Global id of the crossing.
        int id_{};
        /// Record with the order data of this edge: a crossing index, or -(alias index + 1) for an alias.
        int record_{};
        int next_{-1};
    };
    /// Head and tail of the list of the crossings of an edge, as indices into the pool.
    struct EdgeList
    {
        int head_{-1};
        int tail_{-1};
    };

    int EdgesThrough(const Mesh& edges, int triangle, const Mesh& other, int otherTriangle, const int (&sides)[3],
        int (&found)[6], bool (&entering)[6], int count);
    int CrossingId(const CsgCrossingKey& key, const Mesh& edges, const CsgGridPoint& t0, const CsgGridPoint& t1,
        const CsgGridPoint& t2);

    /// Order data of an edge list record: a crossing or an alias.
    const Crossing& Record(int record) const { return record >= 0 ? crossings_[record] : aliases_[-record - 1]; }
    /// Global id of an edge list record.
    int RecordId(int record) const { return record >= 0 ? CrossingBase() + record : aliasIds_[-record - 1]; }

    CsgFlatMap<CsgCrossingKey, int, CsgCrossingKeyHash> crossingIds_;
    /// Crossing id by grid point. Crossings that round to one grid point share one id.
    CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash> gridIds_;
    /// Order data of the keys that merged into an existing crossing. The data belongs to the key and its edge.
    ea::vector<Crossing> aliases_;
    /// Global id of each alias.
    ea::vector<int> aliasIds_;
    /// Crossings of the edges of the first operand (index 0) and the second operand (index 1), by directed key.
    CsgFlatMap<uint64_t, EdgeList, CsgPackedKeyHash> edgeCrossings_[2];
    ea::vector<EdgeNode> edgeNodes_;
};

/// Return the position of a global id: a vertex of the first operand, a vertex of the second operand or a crossing.
Vector3 PointPosition(const Corefinement& core, int id);
/// Return the grid point of a global id, as PointPosition does for the position.
const CsgGridPoint& PointGrid(const Corefinement& core, int id);

/// Build the polygon of a region with the vertex attributes interpolated inside the source face.
/// The ids are the global ids of a region of a triangle of the face. The polygon keeps the plane and the group of the face.
/// Return false and leave the output unchanged if an id does not belong to the face. The caller then keeps the
/// triangle whole, as for a Failed split.
bool RegionPolygon(const Corefinement& core, const Mesh& mesh, int face, const int* ids, int count, CsgPolygon& output);

/// Curve segment of the crossing step for the tests. The ends are crossing indices (global id minus the base).
struct CsgCorefineTestSegment
{
    int triangleA_{};
    int triangleB_{};
    int p_{};
    int q_{};
};

/// Result of the crossing step for the tests.
struct CsgCorefineTestResult
{
    /// All the segments in the order of their creation. The ends are oriented as in curveEdges_.
    ea::vector<CsgCorefineTestSegment> segments_;
    /// Positions of the crossings in the order of their ids.
    ea::vector<Vector3> positions_;
    /// Keys of the crossings in the order of their ids.
    ea::vector<CsgCrossingKey> keys_;
    /// Number of curve segments on each triangle of the operands.
    ea::vector<int> segmentCountsA_;
    ea::vector<int> segmentCountsB_;
    /// Curve segments in insertion order, as crossing indices (global id minus the first crossing id).
    ea::vector<ea::pair<int, int>> curveEdges_;
    bool outOfRange_{};
};

/// Compute the crossings of two solids and return a summary. Both solids must have faces.
CsgCorefineTestResult CsgCorefineCrossingsForTest(const CsgSolid& a, const CsgSolid& b);

/// One region of the split for the tests.
struct CsgSplitTestRegion
{
    int triangle_{};
    /// Global ids of the loop.
    ea::vector<int> ids_;
    /// True if a bridge to a hole made the region.
    bool bridged_{};
    /// Positions of the vertices of the polygon of the region. The vector is empty if RegionPolygon fails.
    ea::vector<Vector3> polygonPositions_;
    /// The polygon of the region with the interpolated attributes. It is empty if RegionPolygon fails.
    CsgPolygon polygon_;
};

/// Split of one operand for the tests.
struct CsgSplitTestOperand
{
    /// State of each triangle: 0 whole, 1 split, 2 failed.
    ea::vector<uint8_t> triangleState_;
    /// Number of holes joined by bridges in each triangle.
    ea::vector<int> holeCount_;
    /// Regions of the split triangles in the output order.
    ea::vector<CsgSplitTestRegion> regions_;
};

/// Result of the split step for the tests.
struct CsgSplitTestResult
{
    CsgSplitTestOperand a_;
    CsgSplitTestOperand b_;
    int vertexCountA_{};
    int vertexCountB_{};
    /// Global id of the first crossing.
    int crossingBase_{};
    /// Position of each global id.
    ea::vector<Vector3> positions_;
    /// Curve segments oriented as in curveEdges_, as global ids.
    ea::vector<ea::pair<int, int>> curveEdges_;
    bool outOfRange_{};
};

/// Compute the crossings of two solids and split the triangles along the curve. Both solids must have faces.
CsgSplitTestResult CsgCorefineSplitForTest(const CsgSolid& a, const CsgSolid& b);

} // namespace CsgInternal

} // namespace Urho3D
