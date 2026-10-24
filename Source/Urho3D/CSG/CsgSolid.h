// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/CsgFlatMap.h"
#include "Urho3D/CSG/CsgPolygon.h"
#include "Urho3D/CSG/_Plugin.h"
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
/// The next steps add the welded loops, IsClosed, ContainsPoint and the construction from boolean results.
/// They use the private Build function.
class PLUGIN_RBFXCSG_API CsgSolid : public RefCounted
{
public:
    /// Build a solid from copies of the polygons. Points within epsilon count as one point.
    explicit CsgSolid(ea::span<const CsgPolygon> polygons, float epsilon = CSG_DEFAULT_EPSILON);
    ~CsgSolid() override;

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
    Vector3 boundsMin_;
    Vector3 boundsMax_;
    ea::vector<Vector3> componentMin_;
    ea::vector<Vector3> componentMax_;
    // Lazy caches. Logically the solid is immutable. Only one job at a time uses a solid, so there is no lock.
    mutable ea::unique_ptr<CsgFaceBvh> componentTree_;
    mutable ea::vector<ea::unique_ptr<CsgFaceBvh>> faceTrees_;
};

} // namespace Urho3D
