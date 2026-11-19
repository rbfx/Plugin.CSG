// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/RenderAPI/RenderAPIDefs.h>

#include <EASTL/array.h>
#include <EASTL/vector.h>

namespace Urho3D
{

class Context;
class Model;

/// A face group of a shape mesh: a range of the index list.
/// One face group gives one geometry of the generated model and one material slot.
struct CsgShapeGroup
{
    /// The first index of the group in `CsgShapeMesh::indices_`.
    unsigned indexStart_{};
    /// The number of indices. It is a multiple of 3 and is not zero.
    unsigned indexCount_{};
};

/// The mesh of a procedural shape: one vertex list, one index list (a triangle list) and the face groups.
///
/// Mesh rules for a generator:
/// - A generator builds a table of positions first. Each vertex copies its position from the table.
///   So two vertices at one corner have bit-identical positions.
/// - No triangle has two vertices with the same table position.
/// - The mesh is closed and its signed volume is positive.
/// - Winding: for the triangle (p0, p1, p2), the cross product (p1 - p0) x (p2 - p0) points out of the solid.
///   `CsgPolygon` uses the same rule for its plane.
/// - The group index is the face id. The group order is a file contract: scenes save the materials by index.
///   A generator emits no empty group.
/// An empty `groups_` means one group with all indices.
struct CsgShapeMesh
{
    ea::vector<ModelVertex> vertices_;
    ea::vector<unsigned> indices_;
    ea::vector<CsgShapeGroup> groups_;
};

/// Return the vertex layout of the generated shapes.
/// It is the layout of CoreData/Models/Box.mdl: position, normal, tangent, two UV sets and a color.
/// The same layout lets a generated shape and a CoreData model share one output geometry for one material.
PLUGIN_RBFXCSG_API const ea::vector<VertexElement>& CsgGetShapeVertexElements();

/// Build a model from the mesh. The model has no name and is not in the resource cache.
/// The model has one geometry for each face group. All geometries use one vertex buffer and one index buffer.
/// The buffers keep a CPU copy, so `CsgBuildPolygonsFromModel` can read the model.
/// An empty `elements` gives the layout of `CsgGetShapeVertexElements`.
/// Return null if the mesh is empty, an index is not valid or a group is empty or out of range.
PLUGIN_RBFXCSG_API SharedPtr<Model> CsgBuildModelFromMesh(
    Context* context, const CsgShapeMesh& mesh, const ea::vector<VertexElement>& elements = {});

/// The parameters of a procedural box with 8 movable corners.
///
/// Corner index bits: bit 0 is +X, bit 1 is +Y, bit 2 is +Z. The default corners are (+-0.5, +-0.5, +-0.5).
/// The local position of corner i is `size_ * corners_[i]`, per component.
struct CsgBoxParams
{
    static constexpr float MinSize = 0.001f;
    static constexpr float MaxSize = 16384.0f;
    static constexpr float MaxCorner = 64.0f;

    /// Return the default value of the corner.
    static Vector3 GetDefaultCorner(unsigned index)
    {
        return Vector3(index & 1u ? 0.5f : -0.5f, index & 2u ? 0.5f : -0.5f, index & 4u ? 0.5f : -0.5f);
    }

    Vector3 size_{Vector3::ONE};
    ea::array<Vector3, 8> corners_{GetDefaultCorner(0), GetDefaultCorner(1), GetDefaultCorner(2), GetDefaultCorner(3),
        GetDefaultCorner(4), GetDefaultCorner(5), GetDefaultCorner(6), GetDefaultCorner(7)};
};

/// Clamp each size component to [MinSize, MaxSize] and each corner component to [-MaxCorner, MaxCorner].
/// A component that is not finite gets its default value.
PLUGIN_RBFXCSG_API void CsgSanitizeBoxParams(CsgBoxParams& params);

/// Generate a box with 12 triangles. Each face is two triangles with a fixed diagonal, from face corner 0 to 2.
/// Each face is one face group. The group order is -X, +X, -Y, +Y, -Z, +Z (nominal faces).
///
/// The function uses a sanitized copy of the parameters. See `CsgSanitizeBoxParams`.
/// Moved corners can make a face that is not planar. The two triangles of the face are then a fold.
/// The CSG boolean accepts this, because it uses the triangles and not the quad.
/// Each triangle has a flat normal. UV 0 is a planar projection in local units on the nominal face plane,
/// so the texel density does not change with the size. UV 1 puts each nominal face in one cell of a 3x2 atlas.
///
/// Validity: at each corner, the three edges to the neighbor corners must give a positive triple product
/// (with the sign of the corner octant). The signed volume of the mesh must be positive.
/// A corner that goes through the opposite face makes the box not valid.
/// Return false if the box is not valid. Then `mesh` is empty.
PLUGIN_RBFXCSG_API bool CsgGenerateBox(const CsgBoxParams& params, CsgShapeMesh& mesh);

} // namespace Urho3D
