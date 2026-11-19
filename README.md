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
A procedural brush (`CsgBoxBrush`) generates its model from parameters. See "Procedural box".
The root writes the result to a `StaticModel` on the same node.

- The build runs on the `WorkQueue`. The main thread applies the result. The root drops a stale result and schedules a new build.
- `Deferred Delay Sec` is the quiet time before a rebuild. `Rebuild()` ignores it.
- Solid cache: the root stores a fingerprint for each brush (brush, model, transform, mesh id and operation). If a rebuild only appends brushes and the base model, the epsilon and the earlier brushes did not change, it starts from the cached solid. It applies only the new brushes.
- `AutoApplyResults` (default true) applies a finished result at once. If it is false, the root queues each result. `HasPendingResult`, `GetPendingResultRevision` and `ApplyPendingResult` give access to the oldest queued result.
- `Bake()` folds the applied brushes into the base operand and removes the brush nodes. The baked geometry exists only at run time and the root does not serialize it. `Bake()` returns false if a rebuild is in flight, a brush is dirty or a result is pending.

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
- `Box.mdl`, `Sphere.mdl`, `Cylinder.mdl`, `Cone.mdl`, `Pyramid.mdl` and `Torus.mdl`: a `CsgBrush` with the CoreData model.

The editor settings (`ui.ini`) keep the choice as `CsgEditor.BrushPreset`. A saved name `Box` from an older version now selects the procedural box. An unknown name keeps the default preset. The combo hides a procedural preset if its type is not registered.

Tests.CSG builds a root with one union brush for each preset model. Each build succeeds and gives a closed and valid mesh with a positive volume. If a model file is missing, the test shows a warning and skips the model. To make a missing model a failure, set the CMake option `RBFX_CSG_REQUIRE_CORE_DATA=ON` or the environment variable `CSG_REQUIRE_CORE_DATA=1`.

### Procedural box

`CsgBoxBrush` generates its mesh from parameters. It has no "Model" attribute, so a model drop does not change it.

- "Size": the box size. Each component is in [0.001, 16384]. The default is (1, 1, 1).
- "Corner 0" to "Corner 7": the corners. Index bit 0 is +X, bit 1 is +Y and bit 2 is +Z. Each component is in [-64, 64]. The default is (+-0.5, +-0.5, +-0.5). The local corner position is Size * Corner, per component.
- "Materials": 6 slots, one for each face, in this order: -X, +X, -Y, +Y, -Z, +Z. The order is a file contract. An empty slot uses the material of slot 0. Faces with equal materials share one output geometry.
- Each face is two triangles with a fixed diagonal. A moved corner can make a face that is not planar. The CSG accepts this.
- UV 0 is a planar projection in local units, so the texel density does not change with the size. UV 1 puts each face in one cell of a 3x2 atlas.
- Validity: at each corner, the three edges to the neighbor corners must give a positive triple product, and the volume must be positive. A corner that goes through the opposite face makes the box not valid. A box that is not valid gives no operand and logs one warning.
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
