# rbfx-csg

CSG (Constructive Solid Geometry) module for rbfx/Urho3D.

## Build

This project depends on rbfx/Urho3D headers and libraries.

### Option A: Use an installed rbfx/Urho3D

Configure with a prefix that contains `Urho3DConfig.cmake` (or equivalent):

```bash
cmake --preset linux-gcc-x64-dll -DCMAKE_INSTALL_PREFIX=/path/to/rbfx/sdk-or-source
cmake --build --preset linux-gcc-x64-debug
```

See `CMakePresets.json` for available configure and build presets.

### Option B: Build as part of rbfx/Urho3D

Add this repository as a subdirectory in your rbfx/Urho3D build (or otherwise ensure an `Urho3D` CMake target exists), then build normally.

## Output

- Produces a plugin target: `Plugin.CSG`
- Produces a static library `Plugin.CSG.Editor` (the editor addon) if the target `EditorLibrary` is present. See "Editor addon".

## Solid engine

A `CsgSolid` is an immutable polygonal operand. `CsgBooleanOperation` computes a union, a difference or an intersection of two closed solids.

The engine uses corefinement:

- It finds the intersection curve of the two solids with exact predicates. The predicates use float expansions (Shewchuk) in `CsgExpansion.h`.
- A face BVH (binned SAH) finds the face pairs that overlap.
- A symbolic perturbation (a displacement of the second operand) resolves the degenerate cases, such as a vertex on a face.
- A SIMD filter (`CsgFloat4`: SSE2, NEON, WASM or scalar) decides most tests. A lane that it cannot decide goes to the exact expansion path.
- The result keeps whole faces from the operands and builds new faces only along the curve.

### Integer grid

All exact decisions use an integer grid:

- A solid stores one integer grid point for each vertex id. The float position of a vertex is the grid point divided by the scale. A boolean result keeps the grid points of its ids.
- The grid scale is 2^-10 (1/1024) units in float mode. Grid coordinates are 32 bit integers below 2^23, so differences are below 2^24 and exact in float.
- The CMake option `CSG_EXACT_USE_DOUBLE` (default OFF) makes `CsgReal` a double. The grid step is then 2^-16 units. Grid coordinates are below 2^29, so differences are below 2^30 and exact in double. A float coordinate snaps to this grid.
- The library builds with strict float flags: `-ffp-contract=off -fno-fast-math` (GCC, Clang) or `/fp:strict` (MSVC).
- The coordinate limit is 2^13 (8192) units in both modes. For an operand coordinate outside this limit, or not finite, the solid sets the `outOfRange` flag and `IsOutOfRange()` returns true. A boolean with such an operand returns an empty solid with `IsOutOfRange()` true.
- One rounding rule makes a grid coordinate: floor(x + 1/2). A tie goes toward plus infinity.
- An operand snaps once. `CsgSolid::FromOperand` welds the corners within `epsilon`, repairs the T-junctions, and snaps each welded position to the grid. The fast entry (`PresnappedFaces`) does the weld and the snap in one pass. It is enough if the snapped faces close up exactly. A constructor without grid points (the polygon list entry) also welds within `epsilon`. No later step snaps a position again.
- A crossing of an edge through a triangle rounds once with the same rule. `CsgExact::RoundOnEdge` and `CsgExact::RoundCrossing` apply the rule exactly to a point on an edge.
- A rounded crossing can fold: it leaves its triangle, it crosses a segment or a chain edge of a triangle map, it breaks the order of the crossings on its edge, or it turns over a closed cycle of segments inside its triangle. The exact sign of the cycle area comes from a double estimate with an error bound. If the bound does not decide, the cycle rule does not apply. `Corefinement::ConstrainCrossings` runs after the crossings are collected. It tests each crossing that is not exactly on the grid. If the crossing folds, it tries the other corners of the grid cell of the exact crossing, and keeps the first corner that does not fold. A crossing with no valid corner keeps its nearest rounding. A second pass (`Corefinement::ConstrainClusters`) handles the crossings of one edge whose grid cells intersect. These crossings get one common grid point if a corner of the intersection does not fold. Then the two sides of a thin slab get the same chain of grid points. The switch `ConstrainCrossingsOnGrid` (in `CsgCorefine.h`) turns both passes off.
- The bounds of the exact values follow from the grid (`CsgExact.h`): minors below 2^50 and determinants below 2^75 in float mode, minors below 2^62 and determinants below 2^93 in double mode.
- The SIMD filter stays on float in both modes. Its error bounds are `CsgPlanarFilterBound` (planar determinants and dot products) and `CsgDistanceFilterBound` (distances). In double mode the conversion of a difference to float can round, and the bounds cover this error. A value inside the bound goes to the exact path.

### Splitter, classification and result

- The splitter works on rounded grid points. It uses exact 2D grid predicates in the plane of a face and an exact cyclic order of the edges around a vertex. Before the trace of a triangle, the splitter welds the ids of the curve segments that have the same grid point. A curve id welds to the chain id at its grid point, or to the first curve id there. Two chain ids never weld to each other.
- Classification works per patch. Elements that an edge off the curve joins form a patch. The elements use grid-canonical ids: ids with equal grid points count as one id. The sum of the votes of the curve edges of a patch labels all its elements. A patch with no net vote (a zero-sum patch) gets a point query on one element that has a grid area.
- Every point query in the exact range is exact. A component with no curve edge uses a grid vertex. A zero-sum patch uses the centroid of a triangle, which is a rational point. Both use a ray along +x with the symbolic perturbation of the second operand. The ray reduces to three exact 2D tests and one exact plane test.
- A zero-sum patch where every element has zero grid area has no side. The boolean also removes a zero-sum patch with no exact classification: its polygon does not match its loop, or its triangulation has zero grid area. A component with no loop has no side. The boolean removes it.
  The clean-up would drop the faces of a zero-area patch in any case. A patch with a loop mismatch is removed, and the result can stay open.
- Holes of a region sort by the exact area of their loops. The area is an expansion (`CsgExact::LoopAreaExact`). A loop of more than 2^20 points has no exact area and uses a double estimate.
- The result path has no weld within `epsilon` and no float repair. The kept faces go through an exact clean-up by grid point and id. The clean-up welds ids with equal grid points, removes repeated ids and zero-width spikes, cancels twin faces (two faces that visit the same ids in opposite directions), and splits an edge at the vertices that are exactly on it (T-junctions).
- `CsgSolid::ContainsPoint(point)` is exact and has no epsilon. It snaps the point to the grid like the input of a solid and counts the crossings of an exact ray along +x with the grid triangles. It uses the same predicates as the classification above. A point that is not finite or is out of the grid range is outside. A point on the surface has a fixed answer from the symbolic perturbation. Build a solid from polygons with `CsgSolid::FromOperand` or with the polygon constructor. Both repair the faces. A solid that is not closed can have a crack of one grid step. A ray through the crack gives a wrong parity.
- The result is closed if every directed edge has an opposite partner. This test is exact and has no crack budget. If the surface is not closed, the result reports `IsClosed()` false.
- `epsilon` is used for the weld of operand corners and for the bounds of operands. An operand without exact ids (a solid from `FromOperand` or from polygons) also uses it for the crack budget of `IsClosed()`. An epsilon that is not positive gives the default epsilon.

The pipeline steps were tested by switching them off over 200000 random cases. The segment rule and the edge-order rule of `ConstrainCrossings`, `RemoveSpurs` and the weld in `FromOperand` stay, because the results got worse without them. The suspect pre-pass only makes the work faster, by about 8 to 13 percent. The step `CancelOpposites` had no effect on these cases, so the boolean no longer has it.

Use `CsgBuildPolygonsFromModel` or `CsgBuildSolidFromModel` to get an operand from a `Model`. Use `CsgTriangulatePolygons` and `CsgBuildModel` to get a `Model` from polygons.

## Known limits

- Rounding changes the shape. A vertex moves by up to half a grid step on each axis (1/2048 units in float mode, 1/131072 units in double mode).
- In rare cases the result stays open (`IsClosed()` false). In a sweep of 1028 rotations of a box union with coplanar contact, no sample fails in float mode and 1 sample fails in double mode. This sample is 5.9000 degrees on the axis (1, 1, 1), left union. The union of A and B is right, but the next boolean on the result fails. Three sheets of one operand lie in one grid cell. The sheets give a T-junction between them. In the sweep, no closed result has a wrong volume.
- `ConstrainCrossings` costs time. On the benchmark it adds about 3% (box-box) to 20-25% (rotated coplanar union) to the boolean.
- An operand with a coordinate outside of +-8192 units gives an empty solid with `IsOutOfRange()` true.

## Benchmark

The benchmark is a hidden test (tag `[.benchmark]`). A normal run skips it. Run it with:

```sh
Tests.CSG "[.benchmark]" --benchmark-samples 20
```

The baselines for the float build and the double build are in the header of `Source/Tests/TestBenchmark.cpp`.

## Components

`CsgRoot` builds one `Model` from the `CsgBrush` components in its subtree. A `CsgBrush` has a model and an operation (`Union`, `DifferenceAB` or `Intersection`).
A procedural brush (`CsgBoxBrush`, `CsgSphereBrush`, `CsgCylinderBrush`, `CsgRingBrush`, `CsgWedgeBrush`, `CsgStairsBrush`, `CsgArchBrush`) generates its model from parameters. See "Procedural box", "Procedural sphere", "Procedural cylinder", "Procedural ring", "Procedural wedge", "Procedural stairs" and "Procedural arch".
The root writes the result to a `StaticModel` on the same node.

- The build runs on the `WorkQueue`. The main thread applies the result. A result that is older than a later change still shows, and the root starts a new wait for the newer state. The root drops a stale result of a failed build, of a job with a model that reloaded, or when auto-apply is off.
- `Deferred Delay Sec` is the wait before a rebuild. The wait starts at the first change and a later change does not reset it. A drag gives live updates. `Rebuild()` ignores it.
- Transform changes: a dirty brush node or root node starts one check at the next work queue update. The check compares the local transform of each brush with the last value that the root saw. Only a difference requests a rebuild. A move of the root gives no rebuild. A brush move with a delay of 0 also starts its rebuild at the next work queue update, not in the call.
- Solid cache: the root stores a fingerprint for each brush (brush, model, transform, mesh id and operation). If a rebuild only appends brushes and the base model, the epsilon and the earlier brushes did not change, it starts from the cached solid. It applies only the new brushes.
- `AutoApplyResults` (default true) applies a finished result at once. If it is false, the root queues each result. `HasPendingResult`, `GetPendingResultRevision` and `ApplyPendingResult` give access to the oldest queued result.
- `Bake()` folds the applied brushes into the base operand and removes the brush nodes. The baked geometry exists only at run time and the root does not serialize it. `Bake()` returns false if a rebuild is in flight, a brush is dirty or a result is pending.
- `Generate Collision` (default false) puts a `CollisionShape` with a triangle mesh on the `CsgResult` node. The mesh is the applied result model, not a second build. Each applied result updates the same shape. An empty result removes the shape.
- `Collision Static Body` (default true) adds a static `RigidBody` (mass 0) next to the shape. Clear it if you do not want a body. A shape without a body has no effect in the physics world.
- The collision components are temporary, like the result node. The scene does not save them. The root makes them again after a load. A baked result keeps its collision.
- The engine adds a `PhysicsWorld` to the scene when a `CollisionShape` enters it. The scene saves that world.
- If the engine has no physics, or the context did not register it, the attributes have no effect. The root logs one warning.

### Shape handles (protocol)

A procedural brush offers handles. A handle is a point that the user drags in the viewport.
The editor addon stays independent of the shape types. It uses three events and does not link Plugin.CSG.
The header `Source/Urho3D/CSG/CsgShapeProtocol.h` has the events and the keys. It needs only the Urho3D core types.
The addon sends an event with `component->SendEvent(...)`. The brush answers on itself and writes the result into the event data.
A bad id or a bad delta gives no log message. The answer is `P_ACCEPTED` false.

- `E_CSGSHAPEQUERYHANDLES`. Out: `P_VERSION` (int, 2), `P_VALID` (bool) and `P_HANDLES` (a `VariantVector`).
  Each element is a `VariantMap` with the keys `Id` (int), `Name` (string), `Kind` (int: 0 point, 1 axis),
  `Position` (Vector3) and `Axis` (unit Vector3, zero for a point). The key constants are in `CsgShapeProtocol`.
  The positions and axes are in the local space of the brush. A shape that is not valid gives `P_VALID` false and no handle.
- `E_CSGSHAPEMOVEHANDLES`. In: `P_INDICES` (a `VariantVector` of int ids) and `P_DELTA` (Vector3).
  The delta is the total delta from the drag start, in the local space of the brush.
  Out: `P_ACCEPTED` (bool).
- `E_CSGSHAPERECENTER` (protocol version 2). Out: `P_ACCEPTED` (bool) and `P_OFFSET` (Vector3).
  The offset is the amount, in the local space of the brush, that the shape center was away from the origin.
  The brush moves its parameters by minus the offset. The editor then moves the node by `node->GetTransform() * offset`.
  Then the world shape does not change. Only the local transform of the node changes, so the parent transforms do not matter.
  `P_ACCEPTED` is false if the shape is centered (offset below 1e-6), always centered (sphere, ring) or the new parameters are not valid.
  Then the parameters stay. The handle ids do not change. The editor queries the positions again.
  In C++ the same API is `CsgProceduralBrush::Recenter`. The box center is the mean of the 8 positions `Size * Corner`.
  The cylinder center is the midpoint of the two end centers. The wedge, stairs and arch center is the "Center" attribute. The recenter sets it to zero.

The move is stateless. The brush applies the delta to its current parameters. The editor restores the attributes before each call.
The brush clamps the result with the sanitizer of the shape. It refuses a result that is not valid.
Then `P_ACCEPTED` is false and the parameters stay. A non-finite delta, an unknown id and a move without a change also give false.
An accepted move marks the brush dirty once. In C++ the same API is `CsgProceduralBrush::GetHandles` and `MoveHandles`.
A repeated id counts once. The handles apply in the given order.

The ids are a file contract.

| Shape | Id | Handle |
| --- | --- | --- |
| Box | 0 to 7 | Corners (points). Bit 0 is +X, bit 1 is +Y, bit 2 is +Z. |
| Box | 20 to 25 | Faces (axes) in the order -X, +X, -Y, +Y, -Z, +Z. The face moves its 4 corners along the axis. |
| Sphere | 0 to 5 | Radius at +X, -X, +Y, -Y, +Z, -Z. The + and - handles of one axis edit one component. |
| Cylinder | 0, 1 | Bottom center and top center (points). |
| Cylinder | 2 to 5 | Bottom radius at +X, -X, +Z, -Z. Not offered if the bottom is an apex. |
| Cylinder | 6 to 9 | Top radius at +X, -X, +Z, -Z. Not offered if the top is an apex. |
| Cylinder | 10 | Bulge (axis +X at the middle of the center line). Not offered if the mean X radius is 0.001 or less. |
| Ring | 0 | Major radius (axis +X). |
| Ring | 1 | Radial tube radius (axis +X). |
| Ring | 2 | Axial tube radius (axis +Y). |
| Ring | 3 | Arc end (axis is the tangent at the arc end). The arc is in the range 1 to 360 degrees. |
| Wedge | 0 to 5 | Faces (axes) in the order -X, +X, -Z (front edge), +Z (back), +Y (top edge), -Y (bottom). The size on the axis grows by the move and "Center" moves by half of it, so the opposite face stays. |
| Stairs | 0 to 5 | Faces (axes) in the order -X (side center), +X (side center), -Z (first riser), +Z (back), +Y (top tread), -Y (bottom). The move is as for the wedge. There is no handle for "Steps". |
| Arch | 0 | Outer X radius at the start end of the outer curve (axis +X). The radius grows by the move. |
| Arch | 1 | Outer Y radius at the top of the outer curve (axis +Y). |
| Arch | 2 | Thickness at the top of the inner curve (axis -Y, into the hole). A move into the hole makes the wall thicker. |
| Arch | 3 | Depth at the front (axis +Z). The depth grows by the move and "Center" moves by half of it, so the back stays. |
| Arch | 4 | Arc end (axis is the tangent at the end angle). The arc grows by 2 d / s radians, with s the length of the tangent (-r.x sin a, r.y cos a). Both ends move, so the handle follows the cursor. |

### Pick event (provenance)

Each face of the result knows the brush that made it. The editor uses this to select a brush with a click.
A subtract brush has no surface of its own, but the faces that it cuts point back to it.

- `CsgPolygon::source_` is the node ID of the brush that made the polygon. 0 is the base model or an unknown source.
  Each split, clip, snap and rescale of a polygon keeps it.
- `CsgTriangulatedModel::Geometry::triangleSources_` has one source for each triangle of `indexData_`.
- After each build the worker makes a pick table: three node space corners and one source for each triangle.
  The table belongs to the applied result. A queued or dropped result does not change it.
- The brush fingerprint compares the node ID. A new brush node with the same model and transform folds again.

The header `Source/Urho3D/CSG/CsgRootProtocol.h` has the event. It needs only the Urho3D core types.
The addon sends it with `root->SendEvent(...)`. The root answers on itself, also when it is disabled.

- `E_CSGROOTPICK`. In: `P_ORIGIN` (Vector3, world), `P_DIRECTION` (Vector3, world, any length)
  and `P_MAXDISTANCE` (float, optional, default 1e9).
  Out: `P_VERSION` (int, 1), `P_HIT` (bool), `P_SOURCE` (unsigned node ID), `P_POSITION` (Vector3, world),
  `P_NORMAL` (unit Vector3, world, front face normal) and `P_DISTANCE` (float, world units).

The pick tests all triangles (no BVH). Only front faces count. The nearest hit wins.
The root moves the ray into the space of the result node, so a scaled or rotated root gives world values.
No applied build result, a zero direction or no front face hit gives `P_HIT` false.
In C++ the same function is `CsgRoot::Pick`.

#### Stats event

- `E_CSGROOTQUERYSTATS`. In: none. Out: `P_VERSION` (int, 1), `P_VALID` (bool), `P_BRUSHCOUNT` (unsigned, operands of the build),
  `P_TRIANGLES` and `P_VERTICES` (unsigned, applied result model), `P_REBUILDMS` (float, boolean plus triangulation plus model build),
  `P_REUSEDOPERANDS` (unsigned, operands from the cached prefix), `P_BUILDING` (bool, a job runs) and `P_ERROR` (string).
- The numbers describe the last applied build. A root that built nothing has zeros and `P_VALID` false.
- `P_ERROR` is empty for a valid result. Else it tells the reason: a failed build, an open solid, open edges, duplicate or degenerate triangles, or no volume.
- The root does not know the brush that caused a problem, so the event has no brush ID. The overlay has no "Select offending brush" button.
- A component that is not a root keeps `P_VERSION` unchanged.

## UV projection

Each `CsgBrush` has three UV attributes. They come after "Materials" in the inspector.

- "UV Mode": `Local` or `Root`. `Local` keeps the UV 0 of the operand mesh. `Root` projects UV 0 as a box in the local frame of the `CsgRoot`.
- "UV Scale": the texture repeats for each unit. The default is 1.
- "UV Offset": a `Vector2` that is added after the scale. The default is (0, 0).

Defaults: a `CsgBrush` with a model uses `Local`, scale 1 and offset 0. Old scenes and old results do not change. A procedural brush (`CsgBoxBrush`, `CsgSphereBrush`, `CsgCylinderBrush`, `CsgRingBrush`, `CsgWedgeBrush`, `CsgStairsBrush`, `CsgArchBrush`) uses `Root`.

`Root` mode rules:

- Each polygon picks the dominant axis of its plane normal. A tie goes to X, then Y, then Z. The choice is for the whole polygon.
- The UV is (U, V) for the axis X: (z, y), Y: (x, z), Z: (x, y). V is the negative of the second coordinate, so V grows down. U has a sign that keeps the texture not mirrored when seen from outside.
- uv = (U, V) * scale + offset. Only UV 0 and the tangent change. The normals and UV 1 stay.
- In `Local` mode, uv = uv0 * scale + offset. A mesh without UV elements has no UV to change.
- The projection uses the frame of the root. A move of the root does not change the textures. A scale of the root does.

To line up the textures of many brushes, use `Root` on all of them and the same "UV Scale" and "UV Offset". The UV of a point then depends only on its position in the root. A brush that moves its faces does not make the texture slide. One material for each brush is the normal case. A procedural brush with empty slots uses slot 0.

A change of a UV attribute folds again from that brush, as a change of the operation does.

## Editor addon

`Plugin.CSG.Editor` adds CSG tools to the scene view of the editor. It does not link `Plugin.CSG`. It finds the CSG types by name at run time.

### Setup

1. Link `Plugin.CSG.Editor` to the custom editor executable.
2. In `Setup()`, call `editorPluginManager_->AddPlugin("RbfxCsg.EditorAddon", &RbfxCsg_EditorAddon);` before `EditorApplication::Setup()`.
3. Include `Urho3D/CSGEditor/CsgEditorAddon.h`.
4. Add `Plugin.CSG` to the loaded plugins of the project (`Project.json`, `PluginManager.LoadedPlugins`).

If the type `CsgRoot` is not registered, the addon does nothing and hides its toolbar.

### Hotkeys

The hotkeys work in the scene view. You can bind them again on the "Key Bindings" page of the editor settings.

| Command | Key | Action |
| --- | --- | --- |
| `Csg.AddUnion` | B | Add a union brush at the cursor. |
| `Csg.AddSubtract` | Shift+B | Add a subtract brush (`DifferenceAB`) at the cursor. |
| `Csg.SetUnion` | 1 | Set `Union` on all selected brushes. |
| `Csg.SetDifference` | 2 | Set `DifferenceAB` on all selected brushes. |
| `Csg.SetIntersection` | 3 | Set `Intersection` on all selected brushes. |
| `Csg.MoveEarlier` | [ | Move the active brush one operand earlier. |
| `Csg.MoveLater` | ] | Move the active brush one operand later. |
| `Csg.ToggleBrush` | H | Toggle "Is Enabled" of all selected brushes. |

The toolbar has the add and operation commands. The toolbar "add" buttons put the brush at the center of the view. The operation buttons show the operation of the active brush. The toolbar tooltips show the build status of the active root. The "CSG" menu of the tab context menu has all commands.

### Brush presets

The toolbar combo in front of the "add" buttons selects the shape of a new brush. The hotkeys B and Shift+B and the toolbar "add" buttons use it. The presets are:

- `Box`: a procedural `CsgBoxBrush`. It has no model file.
- `Sphere`: a procedural `CsgSphereBrush`. It has no model file.
- `Cylinder`: a procedural `CsgCylinderBrush`. It has no model file.
- `Ring`: a procedural `CsgRingBrush`. It has no model file.
- `Wedge`: a procedural `CsgWedgeBrush`. It has no model file.
- `Stairs`: a procedural `CsgStairsBrush`. It has no model file.
- `Arch`: a procedural `CsgArchBrush`. It has no model file.
- `Box.mdl`, `Sphere.mdl`, `Cylinder.mdl`, `Cone.mdl`, `Pyramid.mdl` and `Torus.mdl`: a `CsgBrush` with the CoreData model.

The editor settings (`ui.ini`) keep the choice as `CsgEditor.BrushPreset`. A saved name `Box` from an older version now selects the procedural box. An unknown name keeps the current preset. The combo hides a procedural preset if its type is not registered.

Tests.CSG builds a root with one union brush for each preset model. Each build succeeds and gives a closed and valid mesh with a positive volume. If a model file is missing, the test shows a warning and skips the model. To make a missing model a failure, set the CMake option `RBFX_CSG_REQUIRE_CORE_DATA=ON` or the environment variable `CSG_REQUIRE_CORE_DATA=1`.

### Procedural box

`CsgBoxBrush` generates its mesh from parameters. It has no "Model" attribute, so a model drop does not change it.

- "Size": the box size. Each component is in [0.001, 16384]. The default is (1, 1, 1).
- "Corner 0" to "Corner 7": the corners. Index bit 0 is +X, bit 1 is +Y and bit 2 is +Z. Each component is in [-64, 64]. The default is (+-0.5, +-0.5, +-0.5). The local corner position is Size * Corner, per component.
- The scale of the node applies as for a model brush. The operand transform is `rootFrame * brush world transform`, with the root frame equal to the metric scale of the root times the inverse world transform of the root. A box of size 1 on a node with the scale 0.5 under a root with the scale 1 has the volume 0.125. Under a root with the scale 2 it has the world scale 1, so the volume in the fold space is 1. The output model is in the node space of the root, so its volume there is 0.125.
- "Materials": 6 slots, one for each face, in this order: -X, +X, -Y, +Y, -Z, +Z. The order is a file contract. An empty slot uses the material of slot 0. Faces with equal materials share one output geometry.
- Each face is two triangles with a fixed diagonal. A moved corner can make a face that is not planar. The CSG accepts this.
- UV 0 is a planar projection in local units, so the texel density does not change with the size. UV 1 puts each face in one cell of a 3x2 atlas.
- Validity: at each corner, the three edges to the neighbor corners must give a positive triple product, and the volume must be positive. A corner that goes through the opposite face makes the box not valid. A box that is not valid gives no operand and logs one warning.
- A value that is not finite gets its default value.

### Procedural sphere

`CsgSphereBrush` generates a UV sphere (an ellipsoid) from parameters. It has no "Model" attribute. See `CsgGenerateSphere`.

- "Radius": the radius on each axis. Each component is in [0.001, 16384]. The default is (0.5, 0.5, 0.5).
- "Segments": the number of longitude divisions, 3 to 128. The default is 24.
- "Rings": the number of latitude bands from the top pole to the bottom pole, 2 to 64. The default is 12.
- "Smooth Normals": the default is on. Smooth normals follow the ellipsoid. Off gives a flat normal for each triangle.
- "Materials": one slot for the whole surface.
- The poles are on the Y axis. A pole is one position with a fan of triangles. The seam duplicates vertices only for UV. The seam positions are bit-identical copies.
- UV 0 is (longitude / 2pi, latitude / pi). UV 1 is equal to UV 0. The tangent follows the longitude direction.
- The mesh is a polyhedron that is inscribed in the ellipsoid. Its volume is a little less than the ellipsoid volume.
- A value that is not finite gets its default value. The parameters are always valid after clamping.

### Procedural cylinder

`CsgCylinderBrush` generates a cylinder, a cone or a frustum from parameters. It has no "Model" attribute. See `CsgGenerateCylinder`.

- "Bottom Center" and "Top Center": the ends of the axis, in local space. Each component is in [-16384, 16384]. The defaults are (0, -0.5, 0) and (0, 0.5, 0). The axis can be sheared or tilted.
- "Bottom Radius" and "Top Radius": the two radii of an end along the local X and Z axes. The end lies in the plane perpendicular to the Y axis, not to the tilted axis. Each component is in [0, 16384]. The default is (0.5, 0.5).
- A pair with both components below 0.001 is (0, 0): the end is an apex. A cone has one apex position and no cap. A pair with only one zero component gets 0.001 for it.
- "Segments": 3 to 128. The default is 24. "Height Segments": 1 to 64. The default is 1.
- "Bulge": from -0.95 to 4. The default is 0. The radius at the height fraction t is lerp(bottom, top, t) * (1 + bulge * 4 * t * (1 - t)). The center line stays straight.
- "Smooth Normals": the default is on. Off gives a flat normal for each triangle. A cap normal is always flat.
- "Materials": three slots, in this order: 0 the side, 1 the top cap, 2 the bottom cap. An apex end has no cap, and its slot stays free. The other slots keep their numbers.
- UV 0 of the side is (angle / 2pi, height fraction). UV 0 of a cap is the planar (x + 0.5, z + 0.5). UV 1 is equal to UV 0.
- Validity: the centers must be at least 0.001 apart, and also at least 0.001 apart in Y. A horizontal axis is not valid. Both ends must not be apexes. The volume must be positive. A shape that is not valid gives no operand and logs one warning.
- If the top is below the bottom in Y, the generator flips the winding. The volume stays positive and the top cap stays at "Top Center".
- A value that is not finite gets its default value.

### Procedural ring

`CsgRingBrush` generates a ring (a torus with an elliptic tube section) from parameters. It has no "Model" attribute. See `CsgGenerateRing`. The ring lies around the local Y axis: it lies in the XZ plane.

- "Radius": the major radius, from the axis to the center of the tube. It is in [0.01, 16384]. The default is 0.5.
- "Tube Radius": the radial and the axial radius of the tube. Each component is in [0.005, 16384]. The default is (0.15, 0.15). The radial radius is at most 0.99 * "Radius", so the inner edge stays at a positive radius.
- "Segments": 3 to 128, around the axis. The default is 32. "Tube Segments": 3 to 64, around the tube. The default is 12. 4 tube segments give a square section if "Tube Angle" is 45 degrees, and a diamond if it is 0.
- "Arc": the sweep around the axis, in degrees, from 1 to 360. The default is 360. An arc below 360 adds two flat end caps. Each cap is a fan from the center of the section.
- "Tube Angle": from -180 to 180 degrees. The default is 0. It rotates the start of the tube section. 0 is the outer side. The angle grows toward +Y.
- "Smooth Normals": the default is on. The normal is the analytic normal of the elliptic section. Off gives a flat normal for each triangle. A cap normal is always flat.
- "Materials": six slots, in this order: 0 the outer surface, 1 the inner surface, 2 the top (+Y), 3 the bottom (-Y), 4 the start cap, 5 the end cap. The slots of the caps of a full ring stay free. A quadrant without a quad also leaves its slot free. The other slots keep their numbers.
- The section splits in four quadrants by its axes. A quad of the tube belongs to the quadrant of the angle of its middle, including "Tube Angle". A middle angle on a quadrant border goes to the next quadrant, in the direction of growing angle.
- UV 0 is (angle around the axis / arc, angle around the tube / 2pi). A cap uses the planar section coordinates. UV 1 is equal to UV 0.
- Validity: the volume must be positive and no triangle can have two equal positions. The sanitized values always give a valid ring. A shape that is not valid gives no operand and logs one warning.
- A value that is not finite gets its default value.

### Procedural wedge

`CsgWedgeBrush` generates a right-angle ramp from parameters. It has no "Model" attribute. See `CsgGenerateWedge`.

- "Size": the size of the bounding box. Each component is in [0.001, 16384]. The default is (1, 1, 1).
- "Center": the center of the bounding box in the local space of the brush. Each component is in [-16384, 16384]. The default is (0, 0, 0). Each vertex is generated around it.
- The slope rises from the front bottom edge (-Z, -Y) to the back top edge (+Z, +Y). The mesh has 6 positions and 8 triangles.
- "Materials": five slots, in this order: 0 the bottom (-Y), 1 the back (+Z), 2 the slope, 3 the left triangle (-X), 4 the right triangle (+X). The order is a file contract.
- Each triangle has a flat normal. UV 0 is a planar projection of each face in local units, relative to "Center". UV 1 is equal to UV 0.
- Handles: see the table in "Shape handles (protocol)". A face handle changes the size on its axis by the move and moves "Center" by half of the move, so the opposite face stays fixed.
- Validity: the volume must be positive and no triangle can have two equal positions. The sanitized values always give a valid wedge.
- A value that is not finite gets its default value.

### Procedural stairs

`CsgStairsBrush` generates solid stairs from parameters. It has no "Model" attribute. See `CsgGenerateStairs`.

- "Size": the size of the bounding box. Each component is in [0.001, 16384]. The default is (1, 1, 1).
- "Center": the center of the bounding box in the local space of the brush. Each component is in [-16384, 16384]. The default is (0, 0, 0).
- "Steps": 1 to 64. The default is 5. Step i (from 0) has the height Size.y * (i + 1) / Steps and the tread depth Size.z / Steps. The first riser is at the front (-Z) and starts at the bottom. The top tread ends at the back top edge.
- The profile in the YZ plane is extruded along X. Each tread and each riser is one quad. Each side is a fan of triangles from the back bottom corner. The fan uses only the profile points, so the mesh has no T-junction.
- "Materials": five slots, in this order: 0 the treads, 1 the risers, 2 the two sides, 3 the bottom, 4 the back. The order is a file contract.
- Each triangle has a flat normal. UV 0 is a planar projection of each face in local units, relative to "Center". UV 1 is equal to UV 0.
- The volume is Size.x * Size.y * Size.z * (Steps + 1) / (2 * Steps).
- Handles: see the table in "Shape handles (protocol)". The face handles work as for the wedge. The inspector changes "Steps".
- Validity: the volume must be positive and no triangle can have two equal positions. Very small steps far from the origin merge their float positions. Then the shape is not valid, gives no operand and logs one warning.
- A value that is not finite gets its default value.

### Procedural arch

`CsgArchBrush` generates an arch from parameters: an elliptic annular sector in the XY plane, extruded along Z. It has no "Model" attribute. See `CsgGenerateArch`.

- "Radius": the outer radius along X and along Y. Each component is in [0.01, 16384]. The default is (1, 1).
- "Thickness": the wall thickness, in [0.005, max(Radius.x, Radius.y)]. The default is 0.25. The inner radius is Radius - Thickness per axis. If a component of the inner radius is below 0.001, the arch is a filled sector: the front and the back are fans from the center.
- "Depth": the extrusion along Z, in [0.001, 16384]. The default is 1. The arch is from z = -Depth / 2 to Depth / 2 around "Center".
- "Arc": 1 to 180 degrees. The default is 180. The arch is symmetric about +Y: the angles go from 90 - Arc / 2 to 90 + Arc / 2, measured from +X. A point on the outer curve is (Radius.x cos t, Radius.y sin t).
- "Segments": 3 to 128. The default is 16.
- "Center": the center of the curves in the local space of the brush. Each component is in [-16384, 16384]. The default is (0, 0, 0). An arc of 180 degrees stands on the plane y = Center.y, as a door top.
- "Materials": five slots, in this order: 0 the outer curved face, 1 the inner curved face, 2 the front (+Z), 3 the back (-Z), 4 the two ends. A filled sector has no inner face. Its slot stays free. The order is a file contract.
- The curved faces have smooth normals (the analytic normal of the ellipse). The other faces are flat. UV 0 of a curved face is (arc length along the curve, -z). UV 0 of a flat face is a planar projection in local units, relative to "Center". UV 1 is equal to UV 0.
- Each front and back quad between the inner and the outer curve is two triangles. The generator uses the first diagonal that gives two triangles with a positive area.
- Handles: see the table in "Shape handles (protocol)". The top handles are on the mesh if "Segments" is even. Otherwise they are on the ellipse, a little outside the mesh.
- Validity: the volume must be positive, no triangle can have two equal positions and each front quad needs a good diagonal. A very small arch far from the origin is not valid. Then it gives no operand and logs one warning.
- A value that is not finite gets its default value.

### Operation colors and operand overlay

- The debug shape of a brush has the color of its operation: `Union` green, `DifferenceAB` red, `Intersection` blue. The table is in `CsgOperationColor.h`.
- The selection renderer of the editor draws the debug shape of a selected brush, and of all brushes of a selected root. If a brush is active, the addon adds the debug shapes of the other operands of its root. Each shape is drawn one time.
- The addon follows the "Draw Debug Geometry" option of the tab context menu. If the option is off, the addon draws no debug shapes.
- The addon draws at most 64 operand shapes in a frame. It skips the other operands.
- A brush keeps its debug model. A frame does no resource lookup. A missing model is looked up again only after a model change.
- An overlay in the top right corner of the view lists the brushes of the active root in operand order. Each line has the index, a color mark for the operation and the node name. A disabled brush shows "(off)". The build status of the root is at the top.
- A click on a line selects the brush. The line of the active brush is highlighted.
- The overlay height is at most 40 percent of the view height. A longer list scrolls.
- A move with [ or ] is one undo step. The move skips the children that are not brushes. Undo and redo make the root rebuild.
- A held [ or ] repeats. Each repeat is one move and one undo step.
- The [ and ] bindings accept Ctrl and Alt. Some keyboard layouts need AltGr for these keys, and on Windows AltGr is Ctrl+Alt.
- A toggle with H is one undo step. A disabled brush is not an operand. The root rebuilds.

### Workflow

- The active root is the CSG root of the selected node or of its parent.
- "Add brush" adds a child brush with the preset model to the active root at the hit point of the ray.
- If the ray hits nothing, the brush goes to the root position. Without a root, it goes 10 units in front of the camera.
- If there is no active root, "add brush" makes a root node "CSG" next to the selected node or in the scene. The brush is its first operand.
- "Add brush" selects the new brush. One undo step removes the brush, and the new root too.
- An operation change is one undo step. The root rebuilds at once.
- The operation keys do nothing if no brush is selected.
- A new brush has a world scale of about 1. Its local scale is the inverse of the root world scale.

### Drop a model

Drag a model (`.mdl`) from the resource browser into the scene view. The active selected node selects the case:

- A CSG root, or its result node: the drop adds a union brush with the model under the root, at the hit point. Hold Shift at drop time to add a subtract brush (`DifferenceAB`). The new brush is selected. One undo step removes it.
- A brush: the drop sets the model of all selected brushes. The brushes stay at their positions. One undo step restores the old models. A drop of the same model adds no undo step.
- Any other node, or no selection: the stock editor behavior. The editor adds a plain node with a `StaticModel`.

During the drag, the scene shows a preview. The preview brush of the "add" case is disabled, so the CSG result does not change and does not rebuild until the drop. A move out of the view removes the preview.

### Textures and materials

Drag a material (`.material`) or a texture (`.png`, `.jpg`, `.jpeg`, `.tga`, `.dds`, `.bmp`, `.webp`, `.ktx`) from the resource browser onto the scene view. The selected brushes get the material.

- A brush is selected: slot 0 of the attribute "Materials" of each selected brush gets the material. A model brush and a procedural brush work the same way. The other slots stay. Hold Shift to set all slots. One undo step restores all materials.
- A texture: the addon makes a material for the texture, then assigns it. The material file goes next to the texture file. Its name is the full texture file name plus `.material`. Example: `Textures/Brick.png` gives `Textures/Brick.png.material` in the same resource directory. `Brick.png` and `Brick.jpg` keep separate materials. The material uses the technique `Techniques/LitOpaque.xml`, the shader define `PBR` and the texture as the albedo (diffuse) texture. An existing `.material` file is reused and never overwritten.
- A texture that is not in a mounted resource directory (for example in a package): the addon logs one error and does nothing.
- A material drop is taken only when the cursor is over the CSG result of the root of the selection. Elsewhere the stock editor handler assigns the material to the geometry under the cursor, as before. A texture drop has no stock handler, so it works with any cursor position.
- Only a CSG root is selected, with no brush: the drop does nothing. A tooltip shows "Select a brush to assign a material".
- No selected brush and no root: the stock editor handlers take the drop, as before.

During the drag, the tooltip shows "Material: <name>". A material shows as a live preview on the brushes. A texture shows the preview only if its material file exists. The addon makes the file when you release the button.

The "UV Mode", "UV Scale" and "UV Offset" attributes of the brush control the texture projection. See "UV projection".

### Shape handles

The handles change the shape of a procedural brush (`CsgBoxBrush`, `CsgSphereBrush`, `CsgCylinderBrush`, `CsgRingBrush`, `CsgWedgeBrush`, `CsgStairsBrush`, `CsgArchBrush`) in the view. A brush with a model has no handles.

- Select exactly one node with a procedural brush. The handles show on the brush. With two or more selected nodes, no handles show.
- A point handle is a square. It moves freely. An axis handle is a circle with a short line. It moves only along its axis.
- The colors: a white point, an orange axis, a yellow handle under the cursor, a cyan selected handle. A tooltip shows the name of the handle.
- Click a handle to select it. The click does not change the node selection.
- Shift+click a point handle to add it to the selection or to remove it. An axis handle is always alone in the selection.
- A click on no handle clears the handle selection. The click then selects in the scene as usual.
- Escape clears the handle selection.
- Click a face of the result to select its brush and face handle. A subtract brush has no surface of its own: its cut faces select it. Ctrl toggles and Shift adds the brush, as in the stock selector. Only a plain click also selects the face handle.
- The face handle is the axis handle that is in line with the face (|dot(axis, normal)| > 0.9). The nearest one along its axis wins, then the lower id. A sphere gets its radius handle only if the normal is in line with an axis. A face on the base model, or a face of a removed brush, selects the root as usual.
- Alt+click selects the root, as the stock selector does. A click or a drag on a handle or on the gizmo does not pick a face.
- The selected handles get a translate gizmo at their center. The gizmo uses the world axes, or the axes of the node if the transform gizmo is in local space.
- Drag the gizmo to move the handles. A selected axis handle shows a long line along its axis. The handle follows the part of the drag along this line.
- While handles are selected, the transform gizmo of the node does not show and does not react.
- Hold Ctrl during the drag to snap the handle to the grid of the brush space. The first selected handle snaps. An axis handle snaps only along its axis. "Snap step" sets the grid step (default 0.125). 0 turns off the snap.
- Push Escape during the drag to cancel it. The shape goes back to the state before the drag.
- A move to a shape that is not valid keeps the last valid shape of the drag. A move back to the start position gives the start shape again, also with the Ctrl snap.
- Ctrl+Z or Ctrl+Y during the drag cancels the drag first. Then the undo or redo applies as usual.
- Two handles on the same screen point (for example the -Z and +Z faces seen along Z): the click picks the handle nearer to the camera.
- The result of the root updates during the drag. One undo step reverts a full drag.
- Recenter pivot: a drag moves a face or a corner, so the shape goes off the origin of its node. Then the node gizmo and the rotation pivot are in the wrong place. "Recenter pivot" moves the shape parameters so the shape is centered around the origin of the node. It moves the node by the same amount, so the shape in the world does not change.
- The editor recenters at the end of a drag if the check box "Recenter pivot after drag" is set (default on, setting `CsgEditor.AutoRecenter`). It does not recenter during the drag, after a cancelled drag or after a drag that changed nothing. The command "Recenter pivot" in the "Shape" section and in the CSG menu does it for the active brush at any time.
- The drag and the recenter are one undo step. Ctrl+Z reverts the shape and the node position together. The handle selection stays valid after a recenter.
- A sphere and a ring are always centered, so they do not recenter.
- A brush with "UV Mode" `Local` can shift its UV after a recenter, because the UV is in the local coordinates of the brush. The default `Root` is not changed.
- The overlay of the root has a "Shape" section: the brush type, "Valid" or "Invalid", the number of selected handles and the name of the selected handle. It has the "Show handles" and "Recenter pivot after drag" check boxes, the "Recenter pivot" command and the "Snap step" field. The CSG menu of the tab context menu has the same items. The editor settings keep them.

The handles of each shape:

| Shape | Handles |
| --- | --- |
| Box | 8 corners (points) and 6 faces (axes). A face moves its 4 corners. |
| Sphere | 6 radii (axes) at +X, -X, +Y, -Y, +Z and -Z. |
| Cylinder | The bottom and top centers (points), 4 bottom and 4 top radii (axes) and the bulge (axis). A radius at an apex and a bulge on a thin body are not offered. |
| Ring | The major radius, the radial and the axial tube radius, and the arc end (axes). |
| Wedge | 6 faces (axes): the side triangles, the front edge, the back face, the top edge and the bottom. |
| Stairs | 6 faces (axes): the sides, the first riser, the back face, the top tread and the bottom. |
| Arch | The X and Y radius, the thickness, the depth and the arc end (axes). |

### Bake to asset

- `CsgRoot::SaveResultModel(Serializer&)` saves the applied result with `Model::Save`. It returns false if the root has no result. The materials stay outside the file.
- The command "Bake to Asset" is in the CSG menu and in the overlay of the root. It needs a root with a result and a parent node.
- The file is `Models/<root name>_Baked.mdl` in the resource directory of the scene file. If the scene has no resource file name, the data directory of the project is used. Without both, the command logs an error.
- The command never overwrites a file. It appends _1, _2 and so on to the name.
- One undo step adds the sibling node "<root name> Baked" and disables the `CsgRoot` component. The node has the world transform of the root. Its `StaticModel` has the saved model and the materials of the result.
- Undo removes the new node and enables the root. The file stays on the disk.
- This command is not `Bake()`. `Bake()` works at run time and removes the brush nodes.

### Stats and dimensions in the overlay

- The overlay of the root has a collapsible "CSG stats" header. It shows "Brushes N, tris N, last build X ms, reused K/N" and the vertex count. The data comes from `E_CSGROOTQUERYSTATS`.
- An invalid result shows a red "Invalid: <reason>" line. The line stays when the header is closed.
- The "Shape" section of a selected procedural brush shows "Size X x Y x Z". It is the local bounds of the generated mesh times the world scale of the node. The tooltip says so.
- The size comes from `E_CSGSHAPEQUERYBOUNDS` of `CsgShapeProtocol.h` (out: `P_VALID`, `P_MIN`, `P_MAX`, local space). The event does not change the protocol version. The pure function is `ComputeShapeWorldSize`.
- While a handle drag runs, the section shows "Delta: x y z", the total delta in the local space of the brush.

### Manual check list

The editor has no test harness for addons. Do these checks by hand:

1. Start the editor with the project. The log shows "CSG editor addon is registered" and "Added dynamic plugin 'Plugin.CSG'".
2. Open a scene. Make sure that the toolbar shows two "add" buttons and three operation buttons.
3. Clear the selection. Put the cursor on the floor and push B. A node "CSG" with one child "Brush" appears at the cursor. The brush is selected.
4. Push Shift+B. A second brush appears under the same root. The box cuts the first box.
5. Push 3. The result changes to the intersection. The third operation button is active.
6. Select two brushes. Push 1. Both brushes change to union.
7. Push Ctrl+Z three times. The scene goes back to the state of step 4, step 3 and then an empty scene.
8. Click in a text field of the inspector. Type "123b". The text field gets the characters. No brush is added and no operation changes.
9. Select a node without CSG. Push 1. Nothing changes.
10. Select the root. Drag a model into the view. A preview of the model moves with the cursor. Drop it. A new union brush appears at the cursor and is selected.
11. Select the root. Drag a model and hold Shift at the drop. The new brush cuts the result.
12. Select the root. Drag a model, then move the cursor out of the view and release. No brush stays in the scene.
13. Select one brush. Drag a different model into the view. The brush shape changes during the drag. Drop it. Push Ctrl+Z. The old model comes back.
14. Select two brushes. Drop a model. Both brushes change. One Ctrl+Z restores both.
15. Select a brush. Drag a model out of the view again and release. The old model comes back.
16. Select a node without CSG. Drop a model. The editor adds a plain node with the model, as before.
17. Select the root. Drop a material or a prefab. The stock behavior applies.
18. Select a root with the scale (72, 1, 82). Push B. The new brush has about the size of the box model, not 72 units.
19. Remove `Plugin.CSG` from the loaded plugins. Make sure that the toolbar buttons are not visible and that B does nothing.
20. Select "Sphere.mdl" in the preset combo. Push B. The new brush is a sphere. Restart the editor. The combo shows "Sphere.mdl".
21. Select a brush of a root with two brushes. The overlay shows the root name, the build status and two lines. The union brush has a green mark and a green wireframe. The subtract brush has red. Each wireframe has one color layer, not two.
22. Push [ on the second brush. It moves to line 0 and the build status changes. Push Ctrl+Z. The old order comes back and the root rebuilds. Push Ctrl+Y. The move comes back.
23. Push H. The line shows "(off)" and the result loses the brush. Push Ctrl+Z. The brush comes back.
24. Click a line of the overlay. The brush of the line is selected. The node under the overlay is not selected.
25. Hover a CSG toolbar button. The tooltip shows the build status of the active root.
26. Turn off "Draw Debug Geometry" in the tab context menu. No brush wireframes show.
27. Add more brushes than the overlay can show. The overlay stops at 40 percent of the view height and scrolls.
28. Select "Box" in the preset combo. Push B. A `CsgBoxBrush` appears as a wireframe and the root rebuilds. The overlay line shows "CsgBoxBrush".
29. Change "Size" of the box in the inspector. The wireframe and the result change at once.
30. Move "Corner 7" to (-1, 0.5, 0.5). The box is not valid: the wireframe goes, the result loses the brush and the log shows one warning. Set the corner back. The brush comes back.
31. Set a material in slot 3 (+Y) of the box. Only the top face of the box gets the material. Clear slot 3. The top face uses the material of slot 0.
32. Select a box brush and drop a model on it. The box does not change.
33. Select "Sphere" in the preset combo. Push B. A `CsgSphereBrush` appears as a wireframe and the root rebuilds. The overlay line shows "CsgSphereBrush".
34. Change "Radius", "Segments", "Rings" and "Smooth Normals" in the inspector. The wireframe and the result change at once. A value outside the range is clamped.
35. Set a material in the slot of the sphere. The whole sphere gets the material. Drop a model on the sphere. The sphere does not change.
36. Select "Cylinder" in the preset combo. Push B. A `CsgCylinderBrush` appears as a wireframe and the root rebuilds. The overlay line shows "CsgCylinderBrush".
37. Set "Top Radius" to (0, 0). The brush is a cone and the top cap goes. Set a material in slot 1 and in slot 2. Only the bottom cap gets the slot 2 material.
38. Move "Top Center" to (1, 1, 0). The axis tilts and the result stays closed. Swap the two centers in Y. The shape stays solid and is not inside out.
39. Set "Bulge" to 2 and "Height Segments" to 4. The body bulges. Set both centers to the same point. The wireframe goes and the log shows one warning.
40. Select "Ring" in the preset combo. Push B. A `CsgRingBrush` appears as a wireframe and the root rebuilds. The overlay line shows "CsgRingBrush".
41. Set "Arc" to 180. The ring opens and the two end caps appear. Set a material in slot 4 and in slot 5. Each cap gets its own material.
42. Set "Tube Segments" to 4 and "Tube Angle" to 45. The tube section is a square. Set "Tube Angle" to 0. The section is a diamond. Set a material in slot 3. Only the bottom of the ring gets it.
43. Set "Tube Radius" to (5, 0.1). The radial radius goes down to 0.99 times "Radius". The ring stays solid. Subtract the ring from a box. The cut stays closed.
44. Select a brush of a root with a textured material. Set "UV Mode" to `Root`. The texture does not stretch on the faces and is not mirrored on any face seen from outside.
45. Add a second box next to the first one with the same material, "UV Mode" `Root` and the same "UV Scale". The texture continues over the joint with no step.
46. Move the first box along X. The texture stays in place in the world. Set "UV Scale" to 2. The texture is half the size. Change "UV Offset". The texture slides.
47. Add a `CsgBrush` with a model. Its "UV Mode" is `Local` and the texture looks as before. Set `Root`. The texture changes to the world projection.
48. Select a brush. Drag a `.material` from the resource browser to the view. The tooltip shows "Material: <name>" and the brush shows the material. Release. Undo restores the old material. Redo sets it again.
49. Drag a material onto a procedural box with Shift held. All six slots get the material. Without Shift only slot 0 gets it.
50. Select two brushes. Drop a material. Both get it. One undo step restores both.
51. Drag a `.png` that has no `<name>.png.material` file next to it. The tooltip shows the name of the new material. Release. The file `<name>.png.material` appears in the resource browser and the brush shows the texture. Drop the same texture again. The file does not change.
52. Select only a CSG root. Drag a material over the result of the root. The tooltip shows "Select a brush to assign a material". Release. Nothing changes.
53. Select a brush, then drag a material onto another model that is not the CSG result. The stock editor assigns the material to that model, as before. Also select a plain node with a `StaticModel` and drag a material onto it. The stock editor assigns the material, as before.
54. Select "Box" in the preset combo. Push B. The new box shows 8 white squares and 6 orange circles. The overlay shows "Shape: CsgBoxBrush" and "Valid".
55. Click a corner. It turns cyan and a gizmo shows at the corner. The gizmo of the node goes. Drag the X arrow. The corner and the result follow. Push Ctrl+Z. The box comes back. Push Ctrl+Y. The move comes back.
56. Shift+click two more corners. The overlay shows "Selected handles: 3". Drag the gizmo. The three corners move together.
57. Click a face circle. Only the face is selected and a long line shows along its axis. Drag a gizmo arrow at an angle to the line. The face moves only along the line.
58. Hold Ctrl and drag a corner. The corner stops on multiples of "Snap step" in the brush space. Set "Snap step" to 0. Ctrl does not snap.
59. Start a drag and push Escape before the release. The box goes back to its shape before the drag. No undo step is added.
60. Drag a face through the opposite face. The box keeps the last valid shape. Drag back to the start. The box gets its start shape. The log shows no error.
61. Click on empty floor. The handle selection clears and the scene selection changes as usual. Select two brushes. No handles show.
62. Repeat step 54 with the "Sphere", "Cylinder" and "Ring" presets. Each brush shows the handles of the table above.
63. Clear "Show handles" in the overlay. The handles go. Restart the editor. The check box stays clear.
64. Select the root. Click a cut face of a subtract brush in the view. The brush becomes the selection. For a box, the face handle is cyan and the overlay shows "Selected handles: 1". Alt+click selects the root.
65. Add a box. Drag a face handle by some distance. At the release the node gizmo and the handles show at the center of the box (check box on). Ctrl+Z restores the shape and the node position. Ctrl+Y gives the centered state again.
66. Clear "Recenter pivot after drag". Drag a face. The box stays off the node origin. Use "Recenter pivot" in the "Shape" section. The origin goes to the center and the box does not move. Ctrl+Z reverts it. Restart the editor. The check box keeps its state.
67. Push Escape during a drag. The shape and the node position stay as before the drag.
68. Select "Wedge" in the preset combo. Push B. A `CsgWedgeBrush` appears as a wireframe ramp and the root rebuilds. The overlay line shows "CsgWedgeBrush".
69. Drag the +Y handle of the wedge up. The slope gets steeper and the bottom stays on its plane. Drag the -Z handle. The front edge moves and the back face stays.
70. Set a material in slot 2. Only the slope gets it. Subtract the wedge from a box. The ramp cut stays closed.
71. Select "Stairs" in the preset combo. Push B. A `CsgStairsBrush` appears as a wireframe with 5 steps and the root rebuilds. The overlay line shows "CsgStairsBrush".
72. Set "Steps" to 1, then to 64. The wireframe follows at once. Set a material in slot 0. Only the treads get it.
73. Drag the +Z handle of the stairs. The treads get deeper and the front riser stays. Subtract the stairs from a box. The stair-shaped cut stays closed.
74. Select "Arch" in the preset combo. Push B. A `CsgArchBrush` appears as a wireframe half ring and the root rebuilds. The overlay line shows "CsgArchBrush".
75. Drag the thickness handle down into the hole. The wall gets thicker. At the full radius the arch is a filled half ellipse and the inner face goes.
76. Drag the arc end handle. Both ends of the arch move and the handle stays under the cursor. The arc stops at 1 and at 180 degrees.
77. Put a box wall in a root. Subtract an arch with "Thickness" equal to the radius and a box under it. The door cut stays closed. Set a material in slot 1. Only the inner face gets it.
78. Make a CSG root with a box floor. Set "Generate Collision" on the root. Keep "Collision Static Body" set.
79. Add a node above the floor with a sphere `StaticModel`, a sphere `CollisionShape` and a `RigidBody` with mass 1.
80. Start play mode. The sphere falls and stops on the floor. Stop play mode.
81. Subtract a box from the floor under the sphere. Start play mode. The sphere falls into the hole.
82. Clear "Generate Collision". Start play mode. The sphere falls through the floor.
83. Save the scene and open the file. It has no `CsgResult` node, no collision shape of the result and no body of the result.
84. Save the scene in a resource directory. Add a box brush with B. Choose "Bake to Asset" in the CSG menu. The file `Models/CSG_Baked.mdl` appears in the resource directory. A node "CSG Baked" with the same shape appears and is selected. The root result is gone.
85. Choose "Bake to Asset" again after you enable the root. The new file is `Models/CSG_Baked_1.mdl`. The first file stays.
86. Push Ctrl+Z. The node "CSG Baked" goes and the root result comes back. The file stays. Push Ctrl+Y. The node and the disabled root come back.
87. Select a root with a brush. The overlay shows "CSG stats" with "Brushes 1, tris N, last build X ms, reused 0/1". Add a second brush. The line shows "Brushes 2" and "reused 1/2". Click the header. The block closes.
88. Set the base model of a root to a model with an open face. Add a subtract brush. The overlay shows a red "Invalid:" line with the reason, also when the "CSG stats" header is closed.
89. Add a box. The "Shape" section shows "Size 1.000 x 1.000 x 1.000". Scale the node to 1, 2, 3. The size shows 1.000 x 2.000 x 3.000.
90. Drag a face handle. While the mouse is down, the section shows "Delta: x y z". After the release, the line is gone.

## How to try the sample scene

Currently it is not a simple task to load arbitrary resources and plugins from other directories, therefore you have to copy compiled plugin and sample scene into your editor build (be it SDK, or a build from source).

### With a local build

```sh
BUILD_PRESET=linux-gcc-x64-dll
SRC_PATH=/path/to/rbfx-src

cmake --preset $BUILD_PRESET -DCMAKE_INSTALL_PREFIX=$SRC_PATH
cmake --build --preset $BUILD_PRESET-debug
yes | cp cmake-build/$BUILD_PRESET/bin/Debug/libPlugin.CSG.so $SRC_PATH/cmake-build/$BUILD_PRESET/bin/Debug/
yes | cp -r SampleData/* $SRC_PATH/bin/Data/
pushd $SRC_PATH/cmake-build/$BUILD_PRESET/bin/Debug
./Editor $SRC_PATH/bin
popd
# 1. Load the plugin
# 2. Open Data/Scenes/CsgShowcase.scene
```

## Screenshots

![](https://github.com/user-attachments/assets/5f05d8be-8550-4959-8053-b45fded7d245)
