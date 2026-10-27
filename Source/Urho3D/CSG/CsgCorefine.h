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

#include <atomic>
#include <cstdint>

namespace Urho3D
{

namespace CsgInternal
{

/// Whether Corefinement::ConstrainCrossings moves a rounded crossing that folds the planar map of a triangle.
constexpr bool ConstrainCrossingsOnGrid = true;

/// Crossing of an edge of one operand through a triangle of the other.
struct Crossing
{
    /// The crossing of the edge between the grid points of its ends, rounded to the grid.
    /// position_ is grid / scale, see CsgGridPoint::ToPosition.
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
};

/// Order two crossings of one edge by their parameter from its lower to its higher end.
/// The order is exact. The displacement of the second operand breaks the ties. Return -1, 0 or 1.
int CompareOnEdge(const Crossing& x, const Crossing& y);

/// The triangles on one undirected edge of a mesh. A closed surface has two triangles on an edge.
/// A boundary edge has one. An entry that is not used is -1.
struct CsgEdgeTriangles
{
    int first_{-1};
    int second_{-1};
};

/// Welded view of an operand: ids for each triangle corner and grid positions of the ids.
/// The grid points are the ones that the solid stores. The mesh does not snap a position.
class Mesh
{
public:
    /// The solid must live as long as the mesh. The offset is the first global id of the mesh vertices.
    Mesh(const CsgSolid& solid, bool isSecond, int offset);

    int VertexCount() const { return static_cast<int>(grid_.size()); }
    /// Return true if the triangle has no area because corners weld together.
    bool IsCollapsed(int triangle) const;

    /// Fill edgeTriangles_ in one pass over triangleIds_. The mesh does not build it, because only the check of the
    /// crossings uses it. A second call does nothing. An edge with a repeated welded id is not in the map.
    /// If an edge has more than two triangles, the map keeps the first two.
    /// If vertexFilter is not null, it has one flag for each welded id, and the map keeps only the edges with
    /// two flagged ends. This is faster for a mesh that has few crossings.
    void BuildEdgeTriangles(const ea::vector<uint8_t>* vertexFilter = nullptr);
    bool HasEdgeTriangles() const { return edgeTrianglesBuilt_; }
    /// Return the triangles on the undirected edge between two welded ids, or null. Call BuildEdgeTriangles first.
    const CsgEdgeTriangles* FindEdgeTriangles(int a, int b) const;

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
    /// Triangles of each undirected edge, by CsgUndirectedKey of the welded ids. BuildEdgeTriangles fills it.
    CsgFlatMap<uint64_t, CsgEdgeTriangles, CsgPackedKeyHash> edgeTriangles_;
    bool edgeTrianglesBuilt_{};
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

/// Cached area of one loop of CsgLoopPool. The cache is lazy. A loop never changes after the Add.
struct CsgLoopArea
{
    /// Magnitude of the doubled area in double. The true magnitude differs from it by at most error_.
    double magnitude_{};
    double error_{};
    /// Exact expansion of the doubled area in CsgSplitScratch::areaTerms_. The start is -1 until the first use.
    int termStart_{-1};
    int termCount_{};
    bool approximate_{};
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
    /// Passes of the boundary chain through each id. A pass is the chain index of a visit of the id.
    CsgAdjacency passes_;
    CsgAdjacency curve_;
    CsgFlatSet<uint64_t, CsgPackedKeyHash> boundaryEdges_;
    CsgFlatSet<uint64_t, CsgPackedKeyHash> visited_;
    /// Undirected keys of the curve edges of one triangle. Each edge of the curve graph is in it once.
    CsgFlatSet<uint64_t, CsgPackedKeyHash> curveKeys_;
    /// Ends of the curve edges of one triangle, two entries for each edge.
    ea::vector<int> curveEnds_;
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
    /// Cached areas of the loops of pool_. An entry exists when it is first used. The terms are the exact expansions.
    ea::vector<CsgLoopArea> loopAreas_;
    ea::vector<CsgReal> areaTerms_;
    /// Sort buffers.
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
};

/// One entry for the sort in CancelOpposites.
struct CsgOppositeCandidate
{
    int count_{};
    uint64_t hash_{};
    int index_{};
};

/// The label of an element in a patch with zero exact area, or in a zero sum patch with no exact classification.
constexpr uint8_t CSG_ELEMENT_REMOVED = 2;

/// The elements of one operand of a boolean, collected before the classification.
struct CsgOperandElements
{
    /// One byte for each component of the operand. Not zero if the curve reaches the component.
    ea::vector<uint8_t> touched_;
    ea::vector<CsgBooleanElement> elements_;
    /// The id pool of the elements that have no source face.
    ea::vector<int> elementIds_;
    /// The label of each element after the classification: 0 if outside the other operand, 1 if inside,
    /// CSG_ELEMENT_REMOVED if the element belongs to a patch with zero exact area, or to a zero sum patch with no
    /// exact classification. The boolean keeps no such element.
    ea::vector<uint8_t> inside_;
};

/// Reusable vectors of the classification and of the assembly of a boolean.
struct CsgBooleanScratch
{
    /// The elements of the first operand and of the second operand.
    CsgOperandElements operands_[2];
    ea::vector<uint8_t> frontier_;
    ea::vector<CsgKeptFace> kept_;
    ea::vector<int> keptIds_;
    ea::vector<CsgOppositeCandidate> candidates_;
    ea::vector<uint8_t> removed_;
    /// Classification.
    ea::vector<int> edgeStart_;
    ea::vector<int> loops_;
    CsgFlatMap<uint64_t, int, CsgPackedKeyHash> edgeOwner_;
    /// First id of each grid point, and the curve edges on these canonical ids.
    CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash> firstId_;
    CsgFlatMap<uint64_t, ea::pair<int, int>, CsgPackedKeyHash> curveEdges_;
    ea::vector<int> twins_;
    ea::vector<int> votes_;
    ea::vector<int> parent_;
    ea::vector<int> patchVotes_;
    ea::vector<int> patchBest_;
    ea::vector<double> patchArea_;
    /// Not zero if the best element of the patch has three grid points that are not on one line.
    ea::vector<uint8_t> patchExact_;
    ea::vector<unsigned> indices_;
};

/// A triangle map prepared for the tests of one crossing, see ConstrainCrossings.
struct CsgMapCache
{
    /// Items of the map: two global ids for each item.
    ea::vector<int> items_;
    /// The projected end points of the items, two for each item.
    ea::vector<CsgGridPoint2> points_;
    /// The indices in points_ of the crossing, and the indices of the items that have the crossing as an end.
    ea::vector<int> ends_;
    ea::vector<int> incident_;
    int dropAxis_{0};
    bool flip_{false};
};

/// Reusable vectors of the validity test of a crossing.
struct CsgValidityScratch
{
    /// Boundary chain of one triangle map, as global ids.
    ea::vector<int> chain_;
    ea::vector<int> edgePoints_;
    /// Items of one triangle map: the chain edges first, then the curve segments. Two global ids for each item.
    ea::vector<int> items_;
    /// The projected end points of the items, two for each item.
    ea::vector<CsgGridPoint2> points_;
    /// Cache of the projection of each triangle of the two meshes: 0 is unknown, else 1 + 2 * dropAxis + flip.
    /// ConstrainCrossings fills it. It is empty in the other uses.
    ea::vector<uint8_t> projection_[2];
    /// The boxes of the items of one map: minU, maxU, minV, maxV for each item.
    ea::vector<CsgGridInt> boxes_;
    /// The prepared maps of the three triangles of one crossing.
    CsgMapCache maps_[3];
    /// The global ids of a closed cycle of segments and their projected grid points.
    ea::vector<int> cycle_;
    ea::vector<CsgGridPoint2> cyclePoints_;
    /// The segments of one triangle as pairs of global ids, and a flag for each pair that a cycle uses.
    ea::vector<ea::pair<int, int>> pairs_;
    ea::vector<uint8_t> used_;
    /// Cache of the closed cycles that can turn over. ConstrainCrossings fills it. It is empty in the other uses.
    /// For each crossing the index of its cycle, or -1. For each cycle the start of its ids in cycleIds_ (the
    /// last entry of cycleStart_ is the end) and the exact sign of its area.
    ea::vector<int> cycleOf_;
    ea::vector<int> cycleIds_;
    ea::vector<int> cycleStart_;
    ea::vector<int8_t> cycleSign_;
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
    CsgValidityScratch validity_;
};

/// Rules of the validity test of a crossing position. Corefinement::CrossingInvalidRules returns the failed rules.
enum CsgCrossingRule : unsigned
{
    /// The position is not strictly inside the projected crossed triangle.
    CsgRuleInside = 1,
    /// The position does not keep the strict order on the edge of the crossing.
    CsgRuleEdgeOrder = 2,
    /// A segment or a chain edge that moves with the crossing crosses another segment or chain edge.
    CsgRuleSegments = 4,
    /// The position is equal to a corner of the crossed triangle or of the two triangles on the edge.
    CsgRuleCorner = 8,
    /// The crossing is on a closed cycle of segments inside the crossed triangle, and the rounded cycle turns over.
    CsgRuleCycle = 16
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
    /// Return the crossing of the id if it satisfies the predicate, or null. A vertex id gives null.
    template <class Predicate> const Crossing* FindRecord(int globalId, const Predicate& predicate) const
    {
        if (globalId < CrossingBase())
            return nullptr;
        const Crossing& record = CrossingAt(globalId);
        return predicate(record) ? &record : nullptr;
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
    /// Return the rules that the grid position candidate breaks for the crossing. Zero means that it is valid.
    /// The record must be an element of crossings_. The function uses grid arithmetic only and changes nothing.
    /// The rules are: the position is strictly inside the projected crossed triangle; it is strictly between its
    /// neighbors on the edge of the crossing, in the projection of each triangle on the edge; the segments and
    /// the chain edges that end at the crossing cross no other segment and no other chain edge in the maps of the
    /// three triangles (the crossed one and the two on the edge); it is not a corner of these triangles; a closed
    /// cycle of segments through the crossing inside the crossed triangle keeps the orientation of the exact cycle.
    /// Call BuildEdgeTriangles of both meshes before. If stopAtFirst is true, the function returns at the first
    /// broken rule and the result has that rule only.
    /// If strict is false, the rules only find a fold: the position may be on the boundary of the crossed triangle,
    /// may be equal to a neighbor on the edge or to a corner, and a segment must cross at a point inside both.
    /// The cycle rule is the same in both forms: a rounded cycle of no area also breaks it.
    /// The diagnostic counters use this form to tell a fold from an exact contact.
    /// If segmentRule is false, the function skips the test of the segments (the costly rule).
    unsigned CrossingInvalidRules(const Crossing& record, const CsgGridPoint& candidate, CsgScratch& scratch,
        bool stopAtFirst = false, bool strict = true, bool segmentRule = true) const;
    /// Return true if the candidate position breaks no rule of CrossingInvalidRules.
    bool CrossingIsValid(const Crossing& record, const CsgGridPoint& candidate, CsgScratch& scratch) const
    {
        return CrossingInvalidRules(record, candidate, scratch, true) == 0;
    }
    /// Same as above with a temporary scratch. The first form is faster for many calls.
    bool CrossingIsValid(const Crossing& record, const CsgGridPoint& candidate) const;
    /// Count the crossings whose rounded position breaks a rule. The counts for the first four rules are in perRule,
    /// in the order of CsgCrossingRule. The cycle rule has no count of its own. Build the edge triangles of both
    /// meshes first.
    int CountInvalidCrossings(CsgScratch& scratch, int (&perRule)[4], bool strict = true) const;
    /// Move each rounded crossing that is a fold (the non strict rules of CrossingInvalidRules) to a valid grid
    /// point of the cell of its exact position. The candidates are the corners of the cell: first the nearest
    /// rounding, then the corners with one, two and three axes flipped. The first valid corner wins. The order of
    /// the crossings is the order of the ids and each test sees the current positions of the other crossings.
    /// A corner that equals a corner of the crossed triangle or of the triangles on the edge does not count.
    /// If no corner is valid, the crossing keeps the nearest rounding. Call this after CollectCrossings and
    /// before SplitFaces. It builds the edge triangles of both meshes.
    void ConstrainCrossings(CsgScratch& scratch);
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
    /// One byte for each triangle of the first and the second operand. Not zero if a crossing is on the triangle.
    ea::vector<uint8_t> touchedA_;
    ea::vector<uint8_t> touchedB_;
    /// Regions of the split triangles of the first and the second operand. SplitFaces fills them.
    CsgSplitResult splitA_;
    CsgSplitResult splitB_;
    /// Curve segments by their undirected ends, oriented from entering to exiting along n_A x n_B.
    CsgFlatMap<uint64_t, ea::pair<int, int>, CsgPackedKeyHash> curveEdges_;
private:
    /// Entry in the pool list of the crossings of an edge.
    struct EdgeNode
    {
        /// Global id of the crossing.
        int id_{};
        /// Record with the order data of this edge: a crossing index.
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

    /// Order two records of one edge, as the sort of EdgeCrossings does. Return -1 or 1, or 0 for the same id.
    int CompareRecords(int x, int y) const;

    /// Order data of an edge list record: a crossing.
    const Crossing& Record(int record) const { return crossings_[record]; }
    /// Global id of an edge list record.
    int RecordId(int record) const { return CrossingBase() + record; }

    CsgFlatMap<CsgCrossingKey, int, CsgCrossingKeyHash> crossingIds_;
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
    /// Positions of the crossings in the order of their ids. The vertices that crossings round to follow them.
    /// The segment ends index this vector.
    ea::vector<Vector3> positions_;
    /// Keys of the crossings in the order of their ids. A vertex has the key of its first crossing.
    ea::vector<CsgCrossingKey> keys_;
    /// Number of curve segments on each triangle of the operands.
    ea::vector<int> segmentCountsA_;
    ea::vector<int> segmentCountsB_;
    /// Curve segments in insertion order, as crossing indices (global id minus the first crossing id).
    ea::vector<ea::pair<int, int>> curveEdges_;
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
};

/// Compute the crossings of two solids and split the triangles along the curve. Both solids must have faces.
CsgSplitTestResult CsgCorefineSplitForTest(const CsgSolid& a, const CsgSolid& b);

/// Result of a trace test.
struct CsgTraceTestResult
{
    /// True if every trace closes and the cycles hold every directed edge.
    bool closed_{};
    /// The cycles in the order of the trace. The cycles that touch the boundary come first.
    ea::vector<ea::vector<int>> cycles_;
};

/// Trace the regions of a plane graph as the split of a triangle does. The ids are indices of the points.
/// The chain is the boundary of the triangle in the counterclockwise order. It can visit an id more than once.
/// The curves are undirected edges.
CsgTraceTestResult CsgTraceRegionsForTest(const ea::vector<CsgGridPoint2>& points, const ea::vector<int>& chain,
    const ea::vector<ea::pair<int, int>>& curves);

/// Whether a grid point is inside a solid. The answer is the parity of the exact crossings of a ray along +x.
/// The solid triangles move by the displacement 0b0111 of CsgExact::AxisRayCrossesTriangle, so a point on the surface
/// has a fixed answer. A solid without faces has no inside. CsgSolid::ContainsPoint calls this function.
bool CsgContainsGridPoint(const CsgSolid& solid, const CsgGridPoint& p);

/// Whether a grid point is inside a solid. It runs the exact ray parity of the boolean. The mask second is 0b0111
/// if the solid is the second operand and the point is in the first, or 0b1000 for the reverse.
bool CsgClassifyGridPointForTest(const CsgSolid& other, const CsgGridPoint& p, int second);

/// Whether the centroid (a + b + c) / 3 of a grid triangle is inside a solid. The mask second is as in
/// CsgClassifyGridPointForTest. The three points are in the same operand.
bool CsgClassifyGridCentroidForTest(
    const CsgSolid& other, const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, int second);

#ifdef CSG_DIAGNOSTICS
/// Counters of events inside the boolean. Only the diagnostic tests define CSG_DIAGNOSTICS.
/// The counters are atomic with relaxed order, because a boolean can run on more than one thread.
struct CsgBooleanDiagnostics
{
    std::atomic<int> failedSplits_{};
    std::atomic<int> failedRegionFits_{};
    std::atomic<int> splitTriangles_{};
    std::atomic<int> patches_{};
    std::atomic<int> mixedVotePatches_{};
    std::atomic<int> zeroSumPatches_{};
    /// Zero sum patches in the exact range with no exact classification. The polygon does not match its loop,
    /// or its triangulation has zero exact area. The boolean removes these patches.
    std::atomic<int> zeroSumUnclassifiedPatches_{};
    /// Components that the curve does not reach and that have no face with a loop. The boolean removes them.
    std::atomic<int> componentsRemoved_{};
    /// Zero sum patches in the exact range where every element has zero exact area.
    /// The boolean removes these patches.
    std::atomic<int> zeroSumDegeneratePatches_{};
    std::atomic<int> lobeSplits_{};
    std::atomic<int> lobeHoleKept_{};
    /// Crossings with a rounded position that breaks a rule of the validity test. The counters are for the rules
    /// in the order of CsgCrossingRule. They count only if countInvalid_ is true.
    std::atomic<int> invalidCrossings_{};
    std::atomic<int> invalidInside_{};
    std::atomic<int> invalidOrder_{};
    std::atomic<int> invalidSegments_{};
    std::atomic<int> invalidCorner_{};
    std::atomic<int> checkedCrossings_{};
    /// Crossings that break a rule in the non strict form: a fold. The counters by rule are for inside, order, segments.
    std::atomic<int> foldCrossings_{};
    std::atomic<int> foldInside_{};
    std::atomic<int> foldOrder_{};
    std::atomic<int> foldSegments_{};
    /// Crossings that ConstrainCrossings moved to another grid point, and crossings that it could not move.
    std::atomic<int> constrainedCrossings_{};
    std::atomic<int> unconstrainableCrossings_{};
    /// Switch of the validity count. A test sets it. Reset does not change it.
    std::atomic<bool> countInvalid_{false};

    void Reset()
    {
        for (auto* counter : {&failedSplits_, &failedRegionFits_, &splitTriangles_, &patches_, &mixedVotePatches_,
                 &zeroSumPatches_, &lobeSplits_, &lobeHoleKept_, &invalidCrossings_,
                 &invalidInside_, &invalidOrder_, &invalidSegments_, &invalidCorner_, &checkedCrossings_,
                 &foldCrossings_, &foldInside_, &foldOrder_, &foldSegments_, &constrainedCrossings_,
                 &unconstrainableCrossings_, &zeroSumUnclassifiedPatches_, &zeroSumDegeneratePatches_,
                 &componentsRemoved_})
            counter->store(0, std::memory_order_relaxed);
    }
};

inline CsgBooleanDiagnostics g_csgDiagnostics;
#define CSG_DIAG_COUNT(name) (::Urho3D::CsgInternal::g_csgDiagnostics.name.fetch_add(1, std::memory_order_relaxed))
#else
#define CSG_DIAG_COUNT(name) ((void)0)
#endif

} // namespace CsgInternal

} // namespace Urho3D
