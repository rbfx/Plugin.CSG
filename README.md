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
- A rounded crossing can fold: it leaves its triangle, it crosses a segment or a chain edge of a triangle map, it breaks the order of the crossings on its edge, or it turns over a closed cycle of segments inside its triangle. The exact sign of the cycle area comes from a double estimate with an error bound. If the bound does not decide, the cycle rule does not apply. `Corefinement::ConstrainCrossings` runs after the crossings are collected. It tests each crossing that is not exactly on the grid. If the crossing folds, it tries the other corners of the grid cell of the exact crossing, and keeps the first corner that does not fold. A crossing with no valid corner keeps its nearest rounding. The switch `ConstrainCrossingsOnGrid` (in `CsgCorefine.h`) turns the pass off.
- The bounds of the exact values follow from the grid (`CsgExact.h`): minors below 2^50 and determinants below 2^75 in float mode, minors below 2^62 and determinants below 2^93 in double mode.
- The SIMD filter stays on float in both modes. Its error bounds are `CsgPlanarFilterBound` (planar determinants and dot products) and `CsgDistanceFilterBound` (distances). In double mode the conversion of a difference to float can round, and the bounds cover this error. A value inside the bound goes to the exact path.

### Splitter, classification and result

- The splitter works on rounded grid points. It uses exact 2D grid predicates in the plane of a face and an exact cyclic order of the edges around a vertex.
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
- In rare cases the result stays open (`IsClosed()` false). In a sweep of 1028 rotations of a box union with coplanar contact, no sample fails in float mode and 1 sample fails in double mode. In that sample the union is right, but the next boolean on the result fails. Some crossings have no valid corner in their cell: a group of crossings on one edge is near a segment, and only a joint move of the group removes the fold. In the sweep, no closed result has a wrong volume.
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
The root writes the result to a `StaticModel` on the same node.

- The build runs on the `WorkQueue`. The main thread applies the result. The root drops a stale result and schedules a new build.
- `Deferred Delay Sec` is the quiet time before a rebuild. `Rebuild()` ignores it.
- Solid cache: the root stores a fingerprint for each brush (brush, model, transform, mesh id and operation). If a rebuild only appends brushes and the base model, the epsilon and the earlier brushes did not change, it starts from the cached solid. It applies only the new brushes.
- `AutoApplyResults` (default true) applies a finished result at once. If it is false, the root queues each result. `HasPendingResult`, `GetPendingResultRevision` and `ApplyPendingResult` give access to the oldest queued result.
- `Bake()` folds the applied brushes into the base operand and removes the brush nodes. The baked geometry exists only at run time and the root does not serialize it. `Bake()` returns false if a rebuild is in flight, a brush is dirty or a result is pending.

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
