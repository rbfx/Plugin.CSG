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
- The coordinate limit is 2^13 (8192) units in both modes. For an operand coordinate outside this limit, or not finite, the engine logs a warning and clamps the grid point to 8191 units. The solid sets the `outOfRange` flag, `IsOutOfRange()` returns true, and the result is not valid.
- One rounding rule makes a grid coordinate: floor(x + 1/2). A tie goes toward plus infinity.
- An operand snaps once. `CsgSolid::FromOperand` welds the corners within `epsilon`, repairs the T-junctions, and snaps each welded position to the grid. The fast entry (`PresnappedFaces`) does the weld and the snap in one pass. It is enough if the snapped faces close up exactly. A constructor without grid points (the polygon list entry) also welds within `epsilon`. No later step snaps a position again.
- A crossing of an edge through a triangle rounds once with the same rule. `CsgExact::RoundOnEdge` and `CsgExact::RoundCrossing` apply the rule exactly to a point on an edge.
- The bounds of the exact values follow from the grid (`CsgExact.h`): minors below 2^50 and determinants below 2^75 in float mode, minors below 2^62 and determinants below 2^93 in double mode.
- The SIMD filter stays on float in both modes. Its error bounds are `CsgPlanarFilterBound` (planar determinants and dot products) and `CsgDistanceFilterBound` (distances). In double mode the conversion of a difference to float can round, and the bounds cover this error. A value inside the bound goes to the exact path.

### Splitter, classification and result

- The splitter works on rounded grid points. It uses exact 2D grid predicates in the plane of a face and an exact cyclic order of the edges around a vertex.
- Classification works per patch. Elements that an edge off the curve joins form a patch. The elements use grid-canonical ids: ids with equal grid points count as one id. The sum of the votes of the curve edges of a patch labels all its elements. A patch with no net vote falls back to a point query.
- The result path has no weld within `epsilon` and no float repair. The kept faces go through an exact clean-up by grid point and id. The clean-up welds ids with equal grid points, removes repeated ids and zero-width spikes, cancels twin faces (two faces that visit the same ids in opposite directions), and splits an edge at the vertices that are exactly on it (T-junctions).
- The result is closed if every directed edge has an opposite partner. This test is exact and has no crack budget. If the surface is not closed, the result reports `IsClosed()` false.
- `epsilon` is used for the weld of operand corners, for the bounds of operands and for the point queries. An operand without exact ids (a solid from `FromOperand` or from polygons) also uses it for the crack budget of `IsClosed()`. An epsilon that is not positive gives the default epsilon.

Use `CsgBuildPolygonsFromModel` or `CsgBuildSolidFromModel` to get an operand from a `Model`. Use `CsgTriangulatePolygons` and `CsgBuildModel` to get a `Model` from polygons.
`CsgSimplifyPolygons` (in `CsgOptimization.h`) simplifies a polygon list.

## Known limits

- Rounding changes the shape. A vertex moves by up to half a grid step on each axis (1/2048 units in float mode, 1/131072 units in double mode).
- In rare cases the result stays open (`IsClosed()` false). In a sweep of 1028 rotations of a box union with coplanar contact, 9 samples fail in float mode and 11 samples fail in double mode. Rounding turns thin slivers over, and the surface does not close.
- `MergeCrossingsOnGrid` (in `CsgCorefine.h`) is a compile-time switch. It is off. With the switch on, crossings that round to one grid point share one id and the splitter closes all its traces. But more samples fail: 21 in float mode and 18 in double mode. The open results have coplanar regions that stay twice, because the two operands split them differently. A topology-preserving snap rounding is the next design step.
- A coordinate outside of +-8192 units gives a result that is not valid.

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
