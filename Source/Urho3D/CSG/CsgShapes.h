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
    /// The material slot, and so the geometry index in the model, of the group.
    /// `SlotFromIndex` gives the index of the group in `CsgShapeMesh::groups_`.
    /// A shape with an optional face (a cone has no cap at its apex) sets the slot,
    /// so the slot of each face stays fixed. The slots must be unique.
    unsigned slot_{SlotFromIndex};

    static constexpr unsigned SlotFromIndex = ~0u;
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
/// - The slot of the group is the face id. The slot order is a file contract: scenes save the materials by slot.
///   A generator emits no empty group. A face that is absent leaves its slot free.
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
/// The model has one geometry for each slot, up to the highest slot of the face groups.
/// A slot without a face group gets a geometry with an empty draw range.
/// All geometries use one vertex buffer and one index buffer.
/// The buffers keep a CPU copy, so `CsgBuildPolygonsFromModel` can read the model.
/// An empty `elements` gives the layout of `CsgGetShapeVertexElements`.
/// Return null if the mesh is empty, an index is not valid, a group is empty or out of range,
/// or two groups use one slot.
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

/// The parameters of a procedural UV sphere (an ellipsoid with three radii).
struct CsgSphereParams
{
    static constexpr float MinRadius = 0.001f;
    static constexpr float MaxRadius = 16384.0f;
    static constexpr int MinSegments = 3;
    static constexpr int MaxSegments = 128;
    static constexpr int MinRings = 2;
    static constexpr int MaxRings = 64;

    /// The radius along each axis.
    Vector3 radius_{0.5f, 0.5f, 0.5f};
    /// The number of longitude divisions.
    int segments_{24};
    /// The number of latitude bands from the top pole to the bottom pole.
    int rings_{12};
    /// True: normals follow the ellipsoid. False: each triangle has a flat normal.
    bool smoothNormals_{true};
};

/// Clamp the radii to [MinRadius, MaxRadius], the segments to [3, 128] and the rings to [2, 64].
/// A radius that is not finite gets its default value.
PLUGIN_RBFXCSG_API void CsgSanitizeSphereParams(CsgSphereParams& params);

/// Generate a UV sphere. The poles are on the +Y and -Y axes.
///
/// The mesh has one position table: the top pole, (rings - 1) rings of `segments` positions, the bottom pole.
/// A pole is one position and has a fan of triangles. The seam duplicates the vertices only for UV.
/// Their positions are bit-identical copies from the table.
/// The mesh has one face group (group 0) for the whole surface.
/// Smooth normals are normalize(p / radius^2). Tangents follow the longitude direction.
/// UV 0 is (longitude / 2pi, latitude / pi). UV 1 is equal to UV 0.
///
/// The function uses a sanitized copy of the parameters. See `CsgSanitizeSphereParams`.
/// Return false if the mesh has no positive volume. This does not happen for sanitized parameters.
PLUGIN_RBFXCSG_API bool CsgGenerateSphere(const CsgSphereParams& params, CsgShapeMesh& mesh);

/// The parameters of a procedural cylinder, cone or frustum with an axis that can be sheared or tilted.
///
/// The axis is the segment from `bottomCenter_` to `topCenter_`, in local space.
/// Each end is an ellipse in the plane perpendicular to the local Y axis through its center.
/// The radius pair of an end is the two radii along the local X and Z axes, not along the tilted axis.
/// The radii of a ring in the body are lerp(bottom, top, t) * (1 + bulge * 4 * t * (1 - t)), per component.
/// The center line of the body stays a straight line between the centers.
struct CsgCylinderParams
{
    static constexpr float MaxCoordinate = 16384.0f;
    static constexpr float MinCenterDistance = 0.001f;
    static constexpr float MinRadius = 0.001f;
    static constexpr float MaxRadius = 16384.0f;
    static constexpr int MinSegments = 3;
    static constexpr int MaxSegments = 128;
    static constexpr int MinHeightSegments = 1;
    static constexpr int MaxHeightSegments = 64;
    static constexpr float MinBulge = -0.95f;
    static constexpr float MaxBulge = 4.0f;

    Vector3 bottomCenter_{0.0f, -0.5f, 0.0f};
    Vector3 topCenter_{0.0f, 0.5f, 0.0f};
    /// The radii of the bottom end along X and Z. (0, 0) makes the end an apex.
    Vector2 bottomRadius_{0.5f, 0.5f};
    /// The radii of the top end along X and Z. (0, 0) makes the end an apex.
    Vector2 topRadius_{0.5f, 0.5f};
    /// The number of divisions around the axis.
    int segments_{24};
    /// The number of rings in the body minus one.
    int heightSegments_{1};
    /// The bulge of the body. 0 is straight.
    float bulge_{0.0f};
    /// True: normals follow the body. False: each triangle has a flat normal.
    bool smoothNormals_{true};
};

/// Clamp the centers to [-16384, 16384], the segments to [3, 128], the height segments to [1, 64]
/// and the bulge to [-0.95, 4]. A value that is not finite gets its default value.
/// Clamp each radius to [0, 16384]. A pair with both components below 0.001 becomes (0, 0): an apex.
/// In any other pair each component is at least 0.001.
PLUGIN_RBFXCSG_API void CsgSanitizeCylinderParams(CsgCylinderParams& params);

/// Generate a cylinder, a cone or a frustum.
///
/// The mesh has one position table: the rows of the body from the bottom to the top (height segments + 1 rows),
/// then the center of each cap. A row has `segments` positions. A row at an apex end has one position.
/// The seam and the cap/body split duplicate the vertices only for UV and normals.
/// Their positions are bit-identical copies from the table.
/// A cap is a fan of `segments` triangles from the end center. An apex end has no cap.
/// The body at an apex end is a fan from the apex. No triangle has two equal positions.
///
/// Face groups, in this order and with these slots: 0 the side, 1 the top cap, 2 the bottom cap.
/// The top cap is the cap at `topCenter_`. A group of an apex end is absent, and its slot stays free.
/// Smooth normals of the body come from the radial direction and the slope of the radius profile.
/// The apex normal is the average of the normals of its neighbors. A cap normal is flat and points away from the body.
/// UV 0 of the side is (angle / 2pi, t). UV 0 of a cap is the planar (x + 0.5, z + 0.5). UV 1 is equal to UV 0.
/// If the top is below the bottom in Y, the winding is flipped, so the signed volume stays positive.
///
/// Return false if the shape is not valid. Then `mesh` is empty. A shape is not valid if:
/// the centers are closer than 0.001, both ends are apexes, the volume is not positive,
/// or the float positions merge in a triangle (a very small shape far from the origin).
PLUGIN_RBFXCSG_API bool CsgGenerateCylinder(const CsgCylinderParams& params, CsgShapeMesh& mesh);

/// The parameters of a procedural ring (a torus with an elliptic tube section).
///
/// The ring lies around the local Y axis: it lies in the XZ plane. The center of the tube moves on a circle
/// of the radius `radius_`. The tube section has a radial radius and an axial radius.
struct CsgRingParams
{
    static constexpr float MinRadius = 0.01f;
    static constexpr float MaxRadius = 16384.0f;
    static constexpr float MinTubeRadius = 0.005f;
    static constexpr float MaxTubeRadius = 16384.0f;
    /// The radial tube radius is at most this fraction of the major radius. The inner edge stays at a positive radius.
    static constexpr float MaxInnerFraction = 0.99f;
    static constexpr int MinSegments = 3;
    static constexpr int MaxSegments = 128;
    static constexpr int MinTubeSegments = 3;
    static constexpr int MaxTubeSegments = 64;
    static constexpr float MinArc = 1.0f;
    static constexpr float MaxArc = 360.0f;
    static constexpr float MinTubeAngle = -180.0f;
    static constexpr float MaxTubeAngle = 180.0f;

    /// The major radius: the distance from the Y axis to the center of the tube.
    float radius_{0.5f};
    /// The tube radii: x is the radial radius, y is the axial radius (along Y).
    Vector2 tubeRadius_{0.15f, 0.15f};
    /// The number of divisions around the axis.
    int segments_{32};
    /// The number of divisions around the tube. 4 gives a square section if `tubeAngle_` is 45 degrees,
    /// and a diamond if it is 0.
    int tubeSegments_{12};
    /// The sweep of the ring around the axis, in degrees. 360 is a closed ring. Less than 360 adds two end caps.
    float arc_{360.0f};
    /// The angle of the first tube vertex in the section, in degrees. It rotates the section.
    /// 0 is the outer side (+radial). The angle grows toward +Y.
    float tubeAngle_{0.0f};
    /// True: normals follow the tube. False: each triangle has a flat normal.
    bool smoothNormals_{true};
};

/// Clamp the radius to [0.01, 16384], each tube radius to [0.005, 16384], the segments to [3, 128],
/// the tube segments to [3, 64], the arc to [1, 360] and the tube angle to [-180, 180].
/// Then clamp the radial tube radius to at most 0.99 * radius.
/// A value that is not finite gets its default value.
PLUGIN_RBFXCSG_API void CsgSanitizeRingParams(CsgRingParams& params);

/// Generate a ring.
///
/// The mesh has one position table: the rings of the tube section around the axis (`segments` rings for a full
/// ring, `segments + 1` for an arc), each with `tubeSegments` positions, then the two section centers of an arc.
/// The last ring of a full ring wraps to the first. The seams duplicate the vertices only for UV.
/// Their positions are bit-identical copies from the table. The end caps of an arc are fans from the
/// section center. The section is convex. The cap rim positions are equal to the tube ring positions.
///
/// Face groups, in this order and with these slots (the order is a file contract):
/// 0 the outer surface, 1 the inner surface, 2 the top (+Y), 3 the bottom (-Y), 4 the start cap, 5 the end cap.
/// The section splits in four quadrants by its axes. A quad of the tube belongs to the quadrant
/// of the angle of its middle, in the section (the angle includes `tubeAngle_`).
/// Outer is around +radial, top around +Y, inner around -radial, bottom around -Y.
/// A middle angle on a quadrant border goes to the next quadrant, in the direction of growing angle.
/// A quadrant without a quad and the caps of a full ring give no group. Their slots stay free.
/// Smooth normals are the analytic normal of the elliptic section: normalize(b cos(t), a sin(t)) in (radial, Y),
/// with the radial radius a and the axial radius b. A cap normal is flat.
/// UV 0 is (angle around the axis / arc, angle around the tube / 2pi). A cap uses the planar section coordinates.
/// UV 1 is equal to UV 0.
///
/// The function uses a sanitized copy of the parameters. See `CsgSanitizeRingParams`.
/// Return false if the shape is not valid. Then `mesh` is empty. A shape is not valid if the volume is not
/// positive or the float positions merge in a triangle (a very small tube far from the origin).
PLUGIN_RBFXCSG_API bool CsgGenerateRing(const CsgRingParams& params, CsgShapeMesh& mesh);

} // namespace Urho3D
