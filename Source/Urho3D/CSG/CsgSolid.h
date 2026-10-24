// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgCommon.h"
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

#include <cstdint>

namespace Urho3D
{

class CsgFaceBvh;
struct CsgWeldedLoops;

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

    /// Return the id of the earliest point within epsilon. Add the point if there is none.
    int Add(const Vector3& point);
    /// Return the id of the earliest point within epsilon, or -1.
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
    ExactMap exact_;
};

/// Immutable polygonal CSG operand.
///
/// A solid keeps its faces together with a flat triangulation, face bounds and the face components.
/// Two faces are in one component if they share a welded edge.
/// The solid does not change after construction. Faces are shared and can belong to more than one solid.
///
/// The face trees are BVHs that the solid builds on first use: one for each component and one over the components.
/// The build needs no lock, because one job at a time uses a solid. A tree changes no visible state of the solid.
/// The solid keeps the welded loops of the face corners. They give the edges, the open edges and the T-junctions.
/// A solid is closed if the open edges are short. The length limit is the larger of 200 epsilons and 0.001 of the
/// total edge length. Ray parity classifies a point well in such a solid.
/// The next steps add the construction from boolean results. They use the private Build function.
class PLUGIN_RBFXCSG_API CsgSolid : public RefCounted
{
public:
    /// Build a solid from copies of the polygons. Points within epsilon count as one point.
    /// The solid repairs the faces like FromOperand does, so neighboring faces share their edge vertices.
    explicit CsgSolid(ea::span<const CsgPolygon> polygons, float epsilon = CSG_DEFAULT_EPSILON);
    ~CsgSolid() override;

    /// Build a solid for a boolean operand from the polygons. The solid takes the polygons.
    ///
    /// The corners within epsilon get one position. A face with an edge that ends at the vertex of another face
    /// gains that vertex. Then neighboring faces share every edge vertex at the same position.
    /// If the polygons need no repair, the result is the first solid. Otherwise it is a second solid over the repaired faces.
    static SharedPtr<CsgSolid> FromOperand(ea::vector<CsgPolygon>&& polygons, float epsilon = CSG_DEFAULT_EPSILON);

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
    /// Return true if any edge has no partner, also a hairline crack that IsClosed tolerates.
    bool HasOpenEdges() const { return !openEdges_.empty(); }
    /// Return the total length of the open edges.
    float OpenEdgeLength() const;
    /// Return true if the point is inside the solid. The solid must be closed.
    ///
    /// Three fixed rays vote by the parity of their crossings. Crossings closer than two epsilons count as one.
    /// A point that is not finite is outside. The result for a point within epsilon of a face is not defined.
    bool ContainsPoint(const Vector3& point, float epsilon = CSG_DEFAULT_EPSILON) const;

    /// Return true if the bounds of the solid overlap the box within epsilon.
    bool BoundsOverlap(const Vector3& min, const Vector3& max, float epsilon) const;
    /// Return the BVH over the component bounds. It is built on first use.
    /// Only one thread at a time can use a solid, because this function needs no lock.
    const CsgFaceBvh& GetComponentTree() const;
    /// Return the BVH over the faces of a component. It is built on first use and then kept.
    /// Only one thread at a time can use a solid, because this function needs no lock.
    const CsgFaceBvh& GetFaceTree(unsigned component) const;
    /// Return a BVH that holds every face of the component near the box.
    ///
    /// If the full face tree exists, or if most faces are near, the result is the full tree.
    /// Otherwise the function builds a small tree over the near faces and stores it in the temporary.
    /// The caller keeps the temporary alive while it uses the result.
    const CsgFaceBvh& GetNearFaces(unsigned component, const Vector3& min, const Vector3& max, float epsilon,
        ea::unique_ptr<CsgFaceBvh>& temporary) const;

private:
    /// Build a solid over faces that the caller made. It does not repair the faces.
    CsgSolid(ea::vector<CsgSharedPolygon>&& faces, float epsilon);
    /// Return the welded loops of the topology. They are welded within epsilon if the exact loops are not balanced.
    const CsgWeldedLoops& GetWeld() const;
    /// Return true if the faces do not share their edge vertices exactly, so WatertightFaces changes them.
    bool NeedsRepair() const;
    /// Return the faces with the corners at the welded positions and the T-junction vertices added.
    /// A face that needs no change is the same face. A face that collapses is left out.
    ea::vector<CsgSharedPolygon> WatertightFaces() const;
    /// Take the faces and build the triangles, the bounds and the components.
    void Build(ea::vector<CsgSharedPolygon>&& faces, float epsilon);

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
    // Corners welded by identical position. The booleans of the next steps use them.
    ea::unique_ptr<CsgWeldedLoops> exact_;
    // Corners welded within epsilon. It is empty if the exact loops are balanced. Then GetWeld returns the exact loops.
    ea::unique_ptr<CsgWeldedLoops> welded_;
    // Edges without a partner as welded position pairs.
    ea::vector<ea::pair<Vector3, Vector3>> openEdges_;
    float epsilon_{CSG_DEFAULT_EPSILON};
    bool closed_{true};
    // True if a face edge ends at a T-junction, so the faces do not share vertices exactly.
    bool hasJunctions_{};
    Vector3 boundsMin_;
    Vector3 boundsMax_;
    ea::vector<Vector3> componentMin_;
    ea::vector<Vector3> componentMax_;
    // Lazy caches. Logically the solid is immutable. Only one job at a time uses a solid, so there is no lock.
    mutable ea::unique_ptr<CsgFaceBvh> componentTree_;
    mutable ea::vector<ea::unique_ptr<CsgFaceBvh>> faceTrees_;
};

} // namespace Urho3D
