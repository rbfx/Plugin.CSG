// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Urho3D.h>
#include "Urho3D/CSG/CsgPolygon.h"
#include <Urho3D/Container/RefCounted.h>
#include "Urho3D/CSG/CsgCommon.h"
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Math/BoundingBox.h>

#include <EASTL/span.h>
#include <EASTL/vector.h>

namespace Urho3D
{

class CsgSolid;

/// Triangulated CPU representation of CSG output.
class PLUGIN_RBFXCSG_API CsgTriangulatedModel : public RefCounted
{
public:
	struct Geometry
	{
		/// Canonical `(sourceMeshId << 16) | sourceGeometryIndex` representative copied from `CsgPolygon::shared_`.
		/// Used on the main thread to resolve the shared output vertex layout and material.
		unsigned shared_{};

		/// The component of the solid (see `CsgSolid::GetFaceComponents`). The geometries of one component form
		/// one connected closed shape.
		unsigned component_{};

		/// Triangulated vertex stream in unpacked `ModelVertex` format.
		ea::vector<ModelVertex> vertices_;
		ea::vector<unsigned> indexData_;
		/// Source of each triangle: the node ID of the brush that made its polygon (see `CsgPolygon::source_`).
		/// It has one entry for each three indices of `indexData_`. 0 is the base model or an unknown source.
		ea::vector<unsigned> triangleSources_;
	};

	ea::vector<Geometry> geometries_;
	BoundingBox boundingBox_;
	bool hasAnyVertex_{};

	CsgTriangulatedModel() = default;
	~CsgTriangulatedModel() override = default;

	CsgTriangulatedModel(const CsgTriangulatedModel&) = delete;
	CsgTriangulatedModel& operator=(const CsgTriangulatedModel&) = delete;
};

/// Distance from the line through its neighbors, relative to the largest coordinate of the polygon, within
/// which a vertex counts as part of a straight run: a few float roundings. Faces that share a run then agree
/// that it is straight, so neither claims the sliver between the run and its chord.
constexpr double CsgStraightDistance = 1e-6;

/// Buffers of the polygon triangulator. A caller that triangulates many polygons keeps one scratch and reuses it.
struct CsgTriangulationScratch
{
	ea::vector<double> xs_;
	ea::vector<double> ys_;
	ea::vector<unsigned> ring_;
};

/// Triangulate polygon into triangle list indices (triplets).
///
/// The function clips ears in the plane of the polygon with double precision orientation tests.
/// An ear tip must turn strictly toward the inside. A vertex on a straight run of the boundary
/// never becomes a zero area triangle. No other vertex may lie in the ear or on its diagonal.
/// Vertices at the same position as an ear corner do not block the ear. This is the case in a
/// polygon that visits a bridge twice. If no ear exists, the function clips the most convex vertex.
/// The function returns false only if the polygon has fewer than three vertices.
PLUGIN_RBFXCSG_API bool CsgTriangulatePolygon(const CsgPolygon& poly, ea::vector<unsigned>& outIndices);

/// Triangulate a polygon with shared runs.
///
/// A vertex that is marked in `straight` lies on a run that a neighbor face shares. It is not an ear tip
/// while it lies within rounding of the chord of its neighbors. `straight` is empty or has one entry for
/// each vertex. `clean` is false if the function had to clip a vertex that is not a proper ear.
/// Then the result can contain a flat or an inverted triangle.
PLUGIN_RBFXCSG_API bool CsgTriangulatePolygon(const CsgPolygon& poly, ea::vector<unsigned>& outIndices,
	ea::span<const bool> straight, bool& clean, CsgTriangulationScratch& scratch);

/// Triangulate a collection of polygons into one indexed triangle list.
/// Output vertices are returned as full ModelVertex (unpacked).
PLUGIN_RBFXCSG_API bool CsgTriangulateGeometry(
	const ea::vector<CsgPolygon>& polys, float epsilon,
	ea::vector<ModelVertex>& outVertices, ea::vector<unsigned>& outIndexData);

/// Triangulate a solid into a render mesh without cracks.
///
/// A solid shares every edge vertex between neighbor faces at identical positions. The function emits
/// the triangles that the solid already holds as they are.
PLUGIN_RBFXCSG_API SharedPtr<CsgTriangulatedModel> CsgTriangulateSolid(
	const CsgSolid& solid, float epsilon = CSG_DEFAULT_EPSILON);

/// Defects of a triangulated model that must be a closed surface, and its signed volume.
/// See `CsgCheckModel`.
struct CsgModelCheck
{
	/// Number of directed edges between two positions that have no opposite edge.
	unsigned openEdges_{};
	/// Number of triangles that have the same three positions as an earlier triangle.
	unsigned duplicates_{};
	/// Number of triangles with an index twice, two corners on one grid point or three corners on one line.
	unsigned degenerates_{};
	/// Signed volume of the triangles. It is positive if the winding is outward.
	double volume_{};
};

/// Count the defects of a model that must be a closed surface.
///
/// A position is a float triple. A model that has a triangle with a position that is not finite, or that is
/// too far from the origin for the grid, counts the triangle as degenerate.
/// An open edge is a directed edge between two positions that has no opposite edge.
/// A duplicate is a triangle with the same three positions as an earlier one.
/// A degenerate triangle has an index twice, two corners on one grid point or no exact grid normal.
/// The grid has the step 2^-24 units. Call the function before the model goes to node space.
PLUGIN_RBFXCSG_API CsgModelCheck CsgCheckModel(const CsgTriangulatedModel& model);

namespace CsgInternal
{

/// Split the neighbor of each flat triangle at the middle corner and remove the flat triangle.
/// A flat triangle has three distinct grid corners on one line. The new half keeps the source of its neighbor.
/// CsgTriangulateSolid calls the function. It is public for the tests.
PLUGIN_RBFXCSG_API void RepairFlatTriangles(CsgTriangulatedModel& model);

} // namespace CsgInternal

} // namespace Urho3D
