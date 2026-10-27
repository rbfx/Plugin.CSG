// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/CsgExact.h"
#include "Urho3D/CSG/CsgFlatMap.h"
#include "Urho3D/CSG/CsgPolygon.h"
#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Container/RefCounted.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/shared_ptr.h>
#include <EASTL/span.h>
#include <EASTL/unique_ptr.h>
#include <EASTL/vector.h>

#include <atomic>
#include <cstdint>
#include <mutex>

namespace Urho3D
{

class CsgFaceBvh;
class CsgSolid;
struct CsgWeldedLoops;
enum class CsgOperation;

/// A face that more than one solid can own. Nobody modifies a face after a solid owns it.
using CsgSharedPolygon = ea::shared_ptr<const CsgPolygon>;

/// Assigns one id to the points within epsilon of each other.
///
/// The earliest point wins, so the result depends only on the order of the input.
/// A grid cell is four epsilons wide. A search looks into the next cell only near a cell border.
/// With an epsilon of zero or less, only points with identical bits share an id.
///
/// The grid key packs 21 bits of each cell coordinate. Far cells can share a key.
/// This only makes a chain longer. Each candidate is checked by distance, so the result stays exact.
class PLUGIN_RBFXCSG_API CsgPointWelder
{
public:
    /// Make a welder. The capacity is the expected number of points.
    CsgPointWelder(float epsilon, unsigned capacity);
    /// Make a welder with an epsilon of zero over positions that are distinct. The ids are the indices.
    /// The welder indexes the positions on the first call of Add or Find. The first of equal positions wins.
    explicit CsgPointWelder(ea::vector<Vector3>&& positions);

    /// Return the id of the earliest point within epsilon. Add the point if there is none.
    int Add(const Vector3& point);
    /// Return the id of the earliest point within epsilon, or -1.
    /// With an epsilon of zero the first call can index the positions. Then one thread at a time can use the welder.
    /// A solid does not expose its welders, so only the build of a solid reaches this case.
    int Find(const Vector3& point) const;

    /// Return the number of ids.
    unsigned Count() const { return positions_.size(); }
    /// Return the epsilon of the welder.
    float GetEpsilon() const { return epsilon_; }
    /// Return the position of the id.
    const Vector3& GetPosition(int id) const { return positions_[id]; }
    /// Return the positions of all ids.
    ea::span<const Vector3> GetPositions() const { return {positions_.data(), positions_.size()}; }

private:
    using CellMap = CsgFlatMap<uint64_t, int, CsgPackedKeyHash>;
    using ExactMap = CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash>;

    float epsilon_{};
    double inverseCell_{};
    ea::vector<Vector3> positions_;
    /// Next id in the same grid cell, or -1. It is empty if epsilon is zero.
    ea::vector<int> next_;
    /// Newest id of each grid cell.
    CellMap cells_;
    /// Id of each exact position. It is used only if epsilon is zero.
    mutable ExactMap exact_;
    /// False if the welder took its positions from the caller and has not indexed them yet.
    mutable bool exactIndexed_{true};

    void IndexExact() const;
};

/// Source of a face that a boolean keeps whole, so the solid can copy the triangles instead of building them.
/// A flipped origin means that the face is the flipped copy of the source face.
/// An origin without a solid means that the face has no source.
struct CsgFaceOrigin
{
    SharedPtr<const CsgSolid> solid_;
    unsigned face_{};
    bool flip_{};
};

/// Face corners of a solid welded by identical position. The spans stay valid as long as the solid lives.
/// A boolean uses the ids to find shared vertices without a search by position.
struct CsgExactLoops
{
    /// Face f owns the corners [vertexStart_[f], vertexStart_[f + 1]) in ids_ and vertexIds_.
    ea::span<const unsigned> vertexStart_;
    /// Id of each face corner.
    ea::span<const int> ids_;
    /// Ids of the loop of each face, with repeated ids dropped. The first loopLength_[f] entries of a face are valid.
    ea::span<const int> vertexIds_;
    /// Length of the loop of each face. The length is zero for a face that bounds no surface.
    ea::span<const int> loopLength_;
    /// Position of each id. It is grid / scale of the grid point of the id, see CsgGridPoint::ToPosition.
    ea::span<const Vector3> positions_;
    /// Grid point of each id. A solid stores it and never snaps it again. It is the truth for the exact math.
    ea::span<const CsgGridPoint> grid_;
};

/// Immutable polygonal CSG operand.
///
/// A solid keeps its faces together with a flat triangulation, face bounds and the face components.
/// Two faces are in one component if they share a welded edge.
/// The solid does not change after construction. Faces are shared and can belong to more than one solid.
///
/// The face trees are BVHs: one over the components and one for each component.
/// The tree over the components exists after the build. The solid builds a face tree on first use.
/// Each face tree has its own once flag. Because of this, many threads can query one shared solid at the same time.
/// The solid keeps the welded loops of the face corners. They give the edges, the open edges and the T-junctions.
/// A solid is closed if the open edges are short. The length limit is the larger of 200 epsilons and 0.001 of the
/// total edge length. Ray parity classifies a point well in such a solid.
/// A boolean builds its result from kept faces. FromKeptFaces and Builder make such a solid.
class PLUGIN_RBFXCSG_API CsgSolid : public RefCounted
{
public:
    /// Build a solid from copies of the polygons. Points within epsilon count as one point.
    /// The solid snaps each welded position to the grid, see FromOperand.
    /// The solid repairs the faces like FromOperand does, so neighboring faces share their edge vertices.
    explicit CsgSolid(ea::span<const CsgPolygon> polygons, float epsilon = CSG_DEFAULT_EPSILON);
    ~CsgSolid() override;

    /// Build a solid for a boolean operand from the polygons. The solid takes the polygons.
    ///
    /// The solid welds and repairs the corners first. Then it snaps each welded position to the grid once:
    /// a coordinate becomes grid / scale. A coordinate that is not finite or is outside the grid range stays as it is,
    /// and IsOutOfRange is then true. Welded positions that snap to one grid point become one vertex.
    /// After this step the solid keeps a grid point for each vertex, and no later step snaps a position again.
    /// The corners within epsilon get one position. A face with an edge that ends at the vertex of another face
    /// gains that vertex. Then neighboring faces share every edge vertex at the same position.
    /// If the polygons need no repair, the result is the first solid. Otherwise it is a second solid over the repaired faces.
    static SharedPtr<CsgSolid> FromOperand(ea::vector<CsgPolygon>&& polygons, float epsilon = CSG_DEFAULT_EPSILON);

    /// Return the solid if its faces share their edge vertices. Otherwise build a solid over the repaired faces.
    /// The new solid has no face origins and no exact ids.
    static SharedPtr<CsgSolid> Repaired(const SharedPtr<CsgSolid>& solid);

    CsgSolid(const CsgSolid&) = delete;
    CsgSolid& operator=(const CsgSolid&) = delete;

    /// Return the number of faces.
    unsigned FaceCount() const { return faces_.size(); }
    /// Return the number of face components. The components are in the order of their first face.
    unsigned ComponentCount() const { return componentStart_.empty() ? 0 : componentStart_.size() - 1; }
    /// Return a face. The caller must not modify it.
    const CsgSharedPolygon& GetFace(unsigned index) const { return faces_[index]; }
    /// Return deep copies of the faces. A change to a copy does not change the solid.
    ea::vector<CsgPolygon> ToPolygons() const;

    /// Return the minimum corner of the solid bounds. An empty solid has an infinite minimum.
    const Vector3& GetBoundsMin() const { return boundsMin_; }
    /// Return the maximum corner of the solid bounds. An empty solid has a negative infinite maximum.
    const Vector3& GetBoundsMax() const { return boundsMax_; }
    /// Return the minimum corner of the bounds of a component.
    const Vector3& GetComponentMin(unsigned component) const { return componentMin_[component]; }
    /// Return the maximum corner of the bounds of a component.
    const Vector3& GetComponentMax(unsigned component) const { return componentMax_[component]; }

    /// Return the bounds of all faces.
    ea::span<const Vector3> GetFaceMin() const { return {faceMin_.data(), faceMin_.size()}; }
    ea::span<const Vector3> GetFaceMax() const { return {faceMax_.data(), faceMax_.size()}; }
    /// Return the first triangle of each face, and one more entry at the end. Face f owns [start[f], start[f + 1]).
    ea::span<const unsigned> GetTriangleStart() const { return {triangleStart_.data(), triangleStart_.size()}; }
    /// Return three positions for each triangle.
    ea::span<const Vector3> GetTrianglePositions() const { return {trianglePositions_.data(), trianglePositions_.size()}; }
    /// Return the three face-local vertex indices of each triangle.
    ea::span<const unsigned> GetTriangleVertices() const { return {triangleVertices_.data(), triangleVertices_.size()}; }
    /// Return the first entry of each component in GetComponentFaces, and one more entry at the end.
    ea::span<const unsigned> GetComponentStart() const { return {componentStart_.data(), componentStart_.size()}; }
    /// Return the faces of all components. Component c owns [start[c], start[c + 1]).
    ea::span<const unsigned> GetComponentFaces() const { return {componentFaces_.data(), componentFaces_.size()}; }
    /// Return the component of each face.
    ea::span<const unsigned> GetFaceComponents() const { return {faceComponents_.data(), faceComponents_.size()}; }

    /// Return true if the faces bound a closed surface, so ray parity is valid.
    /// Open edges up to the crack budget do not make a solid open. An empty solid is closed.
    bool IsClosed() const { return closed_; }
    /// Return true if a position is not finite or has a coordinate outside the range of the grid.
    /// The grid point of such a position is clamped, so a boolean with the solid is not valid.
    bool IsOutOfRange() const { return outOfRange_; }
    /// Return true if any edge has no partner, also a hairline crack that IsClosed tolerates.
    bool HasOpenEdges() const { return !openEdges_.empty(); }
    /// Return the number of edges that have no partner.
    unsigned OpenEdgeCount() const { return static_cast<unsigned>(openEdges_.size()); }
    /// Return the total length of the open edges.
    float OpenEdgeLength() const;
    /// Return true if the point is inside the solid. The solid must be closed.
    ///
    /// Three fixed rays vote by the parity of their crossings. Crossings closer than two epsilons count as one.
    /// A point that is not finite is outside. The result for a point within epsilon of a face is not defined.
    bool ContainsPoint(const Vector3& point, float epsilon = CSG_DEFAULT_EPSILON) const;

    /// Return a copy of the face with the winding, the normals and the plane reversed.
    static CsgSharedPolygon Flipped(const CsgPolygon& face);

    /// Return true if the boolean operation keeps a part of an operand. The flag tells if the part is inside the other operand.
    /// An intersection keeps the inside parts. The second operand of DifferenceAB keeps the inside parts.
    /// Every other case keeps the outside parts.
    static bool Keep(bool inside, CsgOperation operation, bool secondOperand);

    /// Return true if the bounds of the solid overlap the box within epsilon.
    bool BoundsOverlap(const Vector3& min, const Vector3& max, float epsilon) const;
    /// Return the BVH over the component bounds. The solid builds it with the bounds. Many threads can call this.
    const CsgFaceBvh& GetComponentTree() const;
    /// Return the BVH over the faces of a component. It is built on first use and then kept.
    /// Many threads can call this at the same time.
    const CsgFaceBvh& GetFaceTree(unsigned component) const;
    /// Return a BVH that holds every face of the component near the box.
    ///
    /// If the full face tree exists, or if most faces are near, the result is the full tree.
    /// Otherwise the function builds a small tree over the near faces and stores it in the temporary.
    /// The caller keeps the temporary alive while it uses the result.
    /// Many threads can call this at the same time, each with its own temporary.
    const CsgFaceBvh& GetNearFaces(unsigned component, const Vector3& min, const Vector3& max, float epsilon,
        ea::unique_ptr<CsgFaceBvh>& temporary) const;

    // Members for the boolean of the next steps.

    /// Return the corners welded by identical position.
    CsgExactLoops GetExactLoops() const;
    /// Return the id of the position of a face vertex. Vertices with one id coincide exactly.
    int VertexKey(unsigned face, unsigned vertex) const;
    /// Return the number of distinct vertex positions. It is the bound of VertexKey.
    unsigned VertexKeySpace() const;
    /// Copy the triangles of a face as face-local vertex indices. Return false if there is no triangle.
    ///
    /// A face that needed the fallback of the triangulator is triangulated again without the shared runs.
    bool CopyTriangles(unsigned face, ea::vector<unsigned>& output) const;
    /// Return true if the stored triangles of the face needed the fallback of the triangulator.
    /// Then the triangles can be flat or inverted.
    bool IsFallbackFace(unsigned face) const;
    /// Return a point on the first triangle of the face. A face without triangles gives its first corner.
    Vector3 FaceSample(unsigned face) const;
    /// Return a point on the first face of the component.
    Vector3 ComponentSample(unsigned component) const;
    /// Classify a point in a closed solid with three rays. Return true if the point is inside.
    ///
    /// The answer is reliable if the three rays agree and no ray crosses the surface within four epsilons.
    /// A point that is not finite is outside and not reliable.
    bool ClassifyPoint(const Vector3& point, float epsilon, bool& reliable) const;

    /// Build a solid from kept faces of a boolean.
    ///
    /// The origins have one entry for each face or none. A face with an origin copies the triangles and the bounds
    /// from the source face. The origin must fit the face: its face index is valid and the vertex counts are equal.
    /// The exact ids have one entry for each face corner, in face order, or none. Corners with one id
    /// share their position exactly. The ids must be in [0, exactIdSpace).
    /// Without ids, the solid welds the corners by position. A call that breaks these rules is a programming error.
    /// It fails an assert in a debug build. In a release build the solid ignores the ids or the origin.
    /// The exact grid has one grid point for each face corner, in face order, or none. Corners with one id must
    /// have one grid point. The solid keeps the grid points and does not snap the positions.
    /// Without a grid, the solid takes the grid point of each position. Then the positions must be on the grid
    /// already, as in a solid from FromOperand.
    static SharedPtr<CsgSolid> FromKeptFaces(ea::vector<CsgSharedPolygon>&& faces, float epsilon,
        ea::span<const CsgFaceOrigin> origins, ea::span<const int> exactIds, int exactIdSpace,
        ea::span<const CsgGridPoint> exactGrid = {});

    /// Assembles a boolean result from whole operand components and solids of the contact regions.
    ///
    /// The components are closed and disjoint from the rest of the result. The builder copies their topology
    /// with new numbers. Corners of different sources never share an id.
    /// The builder keeps the address of each source. A source must live until Finish returns.
    class Builder
    {
    public:
        /// The capacities are the operand totals. They are an upper bound for results of whole components.
        Builder(unsigned faceCapacity, unsigned cornerCapacity, float epsilon = CSG_DEFAULT_EPSILON);
        /// Append the faces of a component of the source, flipped if the flag is set.
        void AppendComponent(const CsgSolid& source, unsigned component, bool flip);
        /// Append every face of the source in its own order and keep its components.
        void AppendSolid(const CsgSolid& source);
        /// Return the solid. The builder is empty afterwards.
        SharedPtr<CsgSolid> Finish();

    private:
        friend class CsgSolid;
        void AppendFace(const CsgSolid& source, unsigned face, bool flip, unsigned component);
        unsigned IdOffset(const CsgSolid& source);

        ea::vector<CsgSharedPolygon> faces_;
        ea::vector<unsigned> triangleStart_;
        ea::vector<Vector3> trianglePositions_;
        ea::vector<unsigned> triangleVertices_;
        ea::vector<Vector3> faceMin_;
        ea::vector<Vector3> faceMax_;
        ea::vector<unsigned> componentStart_;
        ea::vector<unsigned> componentFaces_;
        ea::vector<unsigned> faceComponents_;
        ea::vector<unsigned> fallbackFaces_;
        /// Exact id of each face corner. The ids of each source start after the ids of the sources before it.
        ea::vector<int> exactIds_;
        /// Grid point of each face corner, copied from the source.
        ea::vector<CsgGridPoint> grid_;
        int exactIdSpace_{};
        /// Id offset of each source. A solid has few sources, so the search is linear.
        ea::vector<ea::pair<const CsgSolid*, int>> idOffsets_;
        float epsilon_{CSG_DEFAULT_EPSILON};
        bool closed_{true};
        bool outOfRange_{};
    };

private:
    /// Build a solid from the parts of a builder. The exact ids give the exact loops, the topology is copied.
    explicit CsgSolid(Builder&& builder);
    /// Build a solid over faces that the caller made. It does not repair the faces.
    /// The grid has one grid point for each face corner, or none. Without a grid, the solid snaps the positions.
    CsgSolid(ea::vector<CsgSharedPolygon>&& faces, float epsilon, ea::span<const CsgGridPoint> grid = {});
    /// Build a solid over kept faces of a boolean. See FromKeptFaces.
    CsgSolid(ea::vector<CsgSharedPolygon>&& faces, float epsilon, ea::span<const CsgFaceOrigin> origins,
        ea::span<const int> exactIds, int exactIdSpace, ea::span<const CsgGridPoint> exactGrid);
    /// Return the welded loops of the topology. They are welded within epsilon if the exact loops are not balanced.
    const CsgWeldedLoops& GetWeld() const;
    /// Move each corner to grid / scale of the grid point of its exact id. Return false if every corner is there.
    /// The grid gets the grid point of each corner, in face order. A face that does not move is the same face.
    bool SnappedFaces(ea::vector<CsgSharedPolygon>& faces, ea::vector<CsgGridPoint>& grid) const;
    /// Return true if the faces do not share their edge vertices exactly, so WatertightFaces changes them.
    bool NeedsRepair() const;
    /// Return true if the solid is closed, has no open edge and needs no repair. Then the fast operand entry is right.
    bool IsCleanOperand() const;
    /// Return the faces with the corners at the welded positions and the T-junction vertices added.
    /// A face that needs no change is the same face. A face that collapses is left out.
    /// The grid gets the grid point of each corner of the result, in face order. A moved corner takes the grid point
    /// of its welded id, so the grid points stay the truth.
    ea::vector<CsgSharedPolygon> WatertightFaces(ea::vector<CsgGridPoint>& grid) const;
    /// Take the faces and build the triangles, the bounds and the components.
    /// The optional origins, exact ids and exact grid are as in FromKeptFaces.
    void Build(ea::vector<CsgSharedPolygon>&& faces, float epsilon, ea::span<const CsgFaceOrigin> origins = {},
        ea::span<const int> exactIds = {}, int exactIdSpace = 0, ea::span<const CsgGridPoint> exactGrid = {});
    /// Copy the triangles and the bounds of the source face of an origin. Return false if the origin does not fit.
    bool CopyOrigin(unsigned index, const CsgFaceOrigin& origin);
    /// Compute the bounds of the solid and of the components from the bounds of the faces. Reset the trees.
    void BuildBounds();
    ea::vector<CsgSharedPolygon> faces_;
    // Face f owns the triangles [triangleStart_[f], triangleStart_[f + 1]). A triangle has three entries in
    // trianglePositions_ and in triangleVertices_. The second array has the face-local vertex indices.
    ea::vector<unsigned> triangleStart_;
    ea::vector<Vector3> trianglePositions_;
    ea::vector<unsigned> triangleVertices_;
    ea::vector<Vector3> faceMin_;
    ea::vector<Vector3> faceMax_;
    // Compressed rows: component c owns componentFaces_[componentStart_[c] .. componentStart_[c + 1]).
    ea::vector<unsigned> componentStart_;
    ea::vector<unsigned> componentFaces_;
    ea::vector<unsigned> faceComponents_;
    // Faces whose triangulation needed the fallback of the triangulator, in ascending order.
    // The build fills the set. The solid keeps a face in the set if a boolean copies the face.
    ea::vector<unsigned> fallbackFaces_;
    // Corners welded by identical position. The booleans of the next steps use them.
    ea::unique_ptr<CsgWeldedLoops> exact_;
    // Corners welded within epsilon. It is empty if the exact loops are balanced. Then GetWeld returns the exact loops.
    ea::unique_ptr<CsgWeldedLoops> welded_;
    // Edges without a partner as welded position pairs.
    ea::vector<ea::pair<Vector3, Vector3>> openEdges_;
    float epsilon_{CSG_DEFAULT_EPSILON};
    bool closed_{true};
    // True if a position is not finite or is outside the range of the grid.
    bool outOfRange_{};
    // True if a face edge ends at a T-junction, so the faces do not share vertices exactly.
    bool hasJunctions_{};
    Vector3 boundsMin_;
    Vector3 boundsMax_;
    ea::vector<Vector3> componentMin_;
    ea::vector<Vector3> componentMax_;
    /// Face tree of a component. The once flag guards the build, and the built flag tells readers that it is done.
    struct FaceTreeSlot
    {
        std::once_flag once_;
        std::atomic<bool> built_{false};
        ea::unique_ptr<CsgFaceBvh> tree_;
    };

    // The tree over the components. The build makes it together with the bounds.
    ea::unique_ptr<CsgFaceBvh> componentTree_;
    // One slot for each component. The slots are made in the build and filled on first use by any thread.
    ea::unique_ptr<FaceTreeSlot[]> faceTrees_;
};

} // namespace Urho3D
