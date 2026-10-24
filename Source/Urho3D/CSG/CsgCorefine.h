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

#include <EASTL/utility.h>
#include <EASTL/vector.h>

#include <cstdint>

namespace Urho3D
{

namespace CsgInternal
{

/// Coordinates with an absolute value of this limit or more are outside the range of the exact math.
constexpr float CSG_EXACT_COORDINATE_LIMIT = 65536.0f;

/// Crossing of an edge of one operand through a triangle of the other.
struct Crossing
{
    Vector3 position_;
    /// Parameter along the edge from its lower to its higher welded id.
    double t_{};
    CsgCrossingKey key_;
    /// Exact plane volumes of the edge ends against the crossed triangle. They order the crossings on one edge.
    CsgInt128 lowVolume_{};
    CsgInt128 highVolume_{};
    /// Normal of the crossed triangle. It breaks ties between crossings on one edge.
    CsgNormal<CsgInt128> normal_{};
};

/// Order two crossings of one edge by their parameter from its lower to its higher end.
/// The order is exact. The displacement of the second operand breaks the ties. Return -1, 0 or 1.
int CompareOnEdge(const Crossing& x, const Crossing& y);

/// Welded view of an operand: ids for each triangle corner and grid positions of the ids.
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
    /// Grid position of each welded id.
    ea::vector<CsgGridPoint> grid_;
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

/// Reusable vectors. A boolean makes one scratch and passes it to every step.
struct CsgScratch
{
    ea::vector<CsgFaceBvh::ItemPair> componentPairs_;
    ea::vector<CsgFaceBvh::ItemPair> facePairs_;
    ea::vector<CsgFaceBvh::ItemPair> stack_;
    /// Merge buffer for the sort of the crossings of an edge.
    ea::vector<int> merge_;
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

    /// Find all the face pairs of the operands in the order of the C# reference and intersect them.
    void CollectCrossings(CsgScratch& scratch);
    /// Intersect the triangles of two faces. Add the crossings and the segments.
    void IntersectFaces(int faceA, int faceB);
    /// Write the crossings on an edge of the mesh to the output, ordered from the vertex from to the vertex to.
    /// The ids are local welded ids of the mesh.
    void EdgeCrossings(const Mesh& mesh, int from, int to, CsgScratch& scratch, ea::vector<int>& output) const;

    Mesh a_;
    Mesh b_;
    float epsilon_{};
    /// Crossings by global id minus CrossingBase().
    ea::vector<Crossing> crossings_;
    /// Curve segments on each triangle of the first and the second operand. The lists are by triangle index.
    CsgSegmentLists segmentsA_;
    CsgSegmentLists segmentsB_;
    /// Curve segments by their undirected ends, oriented from entering to exiting along n_A x n_B.
    CsgFlatMap<uint64_t, ea::pair<int, int>, CsgPackedKeyHash> curveEdges_;
    /// True if a coordinate of an operand is outside the range of the exact math. The results are not exact then.
    bool outOfRange_{};

private:
    /// Entry in the pool list of the crossings of an edge.
    struct EdgeNode
    {
        int id_{};
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

    CsgFlatMap<CsgCrossingKey, int, CsgCrossingKeyHash> crossingIds_;
    /// Crossings of the edges of the first operand (index 0) and the second operand (index 1), by directed key.
    CsgFlatMap<uint64_t, EdgeList, CsgPackedKeyHash> edgeCrossings_[2];
    ea::vector<EdgeNode> edgeNodes_;
};

/// Convert a portable 128 bit number to double with one correct rounding (round to nearest even).
/// The native 128 bit type has its own conversion. This function exists in both modes for the tests.
double CsgInt128PortableToDouble(const CsgInt128Portable& value);

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

} // namespace CsgInternal

} // namespace Urho3D
