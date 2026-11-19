// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/RenderAPI/RenderAPIDefs.h>

#include <EASTL/vector.h>

namespace Urho3D
{

class Context;
class Model;

/// The mesh of a procedural shape. The mesh is one geometry: a triangle list.
///
/// Mesh rules for a generator:
/// - A generator builds a table of positions first. Each vertex copies its position from the table.
///   So two vertices at one corner have bit-identical positions.
/// - No triangle has two vertices with the same table position.
/// - The mesh is closed and its signed volume is positive.
/// - Winding: for the triangle (p0, p1, p2), the cross product (p1 - p0) x (p2 - p0) points out of the solid.
///   `CsgPolygon` uses the same rule for its plane.
struct CsgShapeMesh
{
    ea::vector<ModelVertex> vertices_;
    ea::vector<unsigned> indices_;
};

/// Return the vertex layout of the generated shapes.
/// It is the layout of CoreData/Models/Box.mdl: position, normal, tangent, two UV sets and a color.
/// The same layout lets a generated shape and a CoreData model share one output geometry for one material.
PLUGIN_RBFXCSG_API const ea::vector<VertexElement>& CsgGetShapeVertexElements();

/// Build a model with one geometry from the mesh. The model has no name and is not in the resource cache.
/// The buffers keep a CPU copy, so `CsgBuildPolygonsFromModel` can read the model.
/// An empty `elements` gives the layout of `CsgGetShapeVertexElements`.
/// Return null if the mesh is empty or an index is not valid.
PLUGIN_RBFXCSG_API SharedPtr<Model> CsgBuildModelFromMesh(
    Context* context, const CsgShapeMesh& mesh, const ea::vector<VertexElement>& elements = {});

} // namespace Urho3D
