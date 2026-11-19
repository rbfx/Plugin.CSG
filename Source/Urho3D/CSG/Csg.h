// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Math/Matrix3x4.h>
#include <Urho3D/Plugins/PluginApplication.h>

#include <EASTL/utility.h>
#include <EASTL/span.h>
#include <EASTL/vector.h>

namespace Urho3D
{

class Context;
class CsgSolid;
class CsgTriangulatedModel;
class Model;
class WorkQueue;
struct CsgPolygon;
struct ResourceRefList;

enum class CsgOperation
{
    Union,
    DifferenceAB,
    Intersection,
};

static const char* CsgOperationNames[] = {
    "Union",
    "DifferenceAB",
    "Intersection",
    nullptr
};

/// How a brush gets the UV 0 of its polygons.
enum class CsgUvMode
{
    /// Use the UV 0 of the operand mesh. Scale and offset apply to them.
    Local,
    /// Box projection in the frame of the root. The textures of all brushes line up.
    Root,
};

static const char* CsgUvModeNames[] = {
    "Local",
    "Root",
    nullptr
};

/// Compute CSG boolean operation between two solids and return the resulting solid.
///
/// The solids must be closed. The operation uses exact math for the intersection curve.
/// `epsilon` is used for the bounds and for the weld of the result.
/// An epsilon that is not positive gives the default epsilon.
/// An operand with IsOutOfRange true gives an empty solid with IsOutOfRange true.
/// The result can be one of the operands if the other operand is empty. A null operand gives a null result.
/// A work queue lets large operands find the overlapping faces in parallel. The result does not change.
/// Pass the work queue only from a caller outside of all work queue tasks. A caller inside any task must pass null.
/// The code forces a serial run only on worker threads, as a cheap guard. It cannot detect a task on the main thread.
/// A null work queue gives a serial call.
PLUGIN_RBFXCSG_API SharedPtr<const CsgSolid> CsgBooleanOperation(
    const CsgSolid* a, const CsgSolid* b, CsgOperation op, float epsilon = CSG_DEFAULT_EPSILON,
    WorkQueue* workQueue = nullptr);

/// Extract CSG polygons from a `Model` operand.
///
/// `worldTransform` is applied to vertex positions (and normals/tangents) during extraction.
/// `meshId` is encoded into `CsgPolygon::shared_` as (meshId << 16) | geometryIndex.
PLUGIN_RBFXCSG_API ea::vector<CsgPolygon> CsgBuildPolygonsFromModel(
    const Model* model, const Matrix3x4& worldTransform, unsigned meshId = 0);

/// Change the UV 0 and the tangent of polygons. The positions must be in the frame of the root.
///
/// Local: uv0 = uv0 * scale + offset. The tangent stays. Local with scale 1 and offset 0 changes nothing.
/// Root: box projection. The dominant axis of the polygon normal picks the projection plane.
/// The first axis wins a tie, then the second. The choice is for the whole polygon.
///   Axis X: (first, second) = (z, y). Axis Y: (x, z). Axis Z: (x, y).
///   V = -(second coordinate), as in the box generator: V grows down when the second axis is up.
///   U = sign * (first coordinate). The sign keeps the texture not mirrored when seen from outside:
///   with up = +second axis and normal n, the right direction is cross(up, n). It gives
///   sign = -s for X and for Y, and +s for Z, where s is the sign of the normal on the dominant axis.
///   uv = (U, V) * scale + offset.
/// The tangent is the direction of growing U in the polygon plane. Its w follows the shader rule
/// `bitangent = cross(tangent, normal) * w`: the bitangent points along the second axis (up).
/// The normal and UV 1 stay.
PLUGIN_RBFXCSG_API void CsgProjectPolygonUvs(
    ea::vector<CsgPolygon>& polygons, CsgUvMode mode, float scale, const Vector2& offset);

/// Build a solid from a `Model` operand.
///
/// `worldTransform` is applied to vertex positions (and normals/tangents) during extraction.
/// `meshId` is encoded into `CsgPolygon::shared_` as (meshId << 16) | geometryIndex.
/// Returns an empty solid if extraction fails.
PLUGIN_RBFXCSG_API SharedPtr<CsgSolid> CsgBuildSolidFromModel(
    const Model* model, const Matrix3x4& worldTransform, unsigned meshId = 0, float epsilon = CSG_DEFAULT_EPSILON);

/// Triangulate polygons into CPU representation of CSG output.
PLUGIN_RBFXCSG_API SharedPtr<CsgTriangulatedModel> CsgTriangulatePolygons(
    ea::vector<CsgPolygon> polygons, float epsilon = CSG_DEFAULT_EPSILON);

/// Build output `Model` from triangulated CSG result.
///
/// `sourceModels` index corresponds to `sourceMeshId` decoded from `CsgPolygon::shared_`.
/// `sourceMaterials` index corresponds to `sourceMeshId`, and resource list index corresponds
/// to `sourceGeometryIndex` decoded from `CsgPolygon::shared_`.
///
/// If `outComponents` is not null, entry i is the component of model geometry i.
PLUGIN_RBFXCSG_API SharedPtr<Model> CsgBuildModel(
    Context* context,
    const CsgTriangulatedModel& triangulated,
    ea::span<const Model* const> sourceModels,
    ea::span<const ResourceRefList* const> sourceMaterials,
    ResourceRefList* outMaterials = nullptr,
    ea::vector<unsigned>* outComponents = nullptr);

/// Register CSG library objects.
void PLUGIN_RBFXCSG_API RegisterCsgLibrary(Context* context);
void PLUGIN_RBFXCSG_API RegisterCsgLibrary(PluginApplication& context);

} // namespace Urho3D
