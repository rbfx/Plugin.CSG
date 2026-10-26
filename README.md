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
- The exact tests work on an integer grid with a scale of 2^-10 (1/1024) units. Grid coordinates are below 2^23, so differences are below 2^24 and exact in float.
- The coordinate limit is 2^13 (8192) units. The engine logs a warning for an operand coordinate outside this limit. It clamps the coordinate to 8191 units and sets the `outOfRange` flag. The result is not valid in this case.
- The CMake option `CSG_EXACT_USE_DOUBLE` (default OFF) makes `CsgReal` a double. The library builds with strict float flags: `-ffp-contract=off -fno-fast-math` (GCC, Clang) or `/fp:strict` (MSVC).
- With `CSG_EXACT_USE_DOUBLE`, the grid step is 2^-26 units. Grid coordinates are 64 bit integers below 2^39, so differences are below 2^40 and exact in double. The coordinate limit stays 8192 units. The grid keeps each float coordinate with an absolute value of 2^-3 or more without a change.
- The bounds of the exact values follow from the grid (`CsgExact.h`): minors below 2^50 and determinants below 2^75 in float mode, minors below 2^82 and determinants below 2^123 in double mode. The SIMD filter stays on float in both modes. In double mode the conversion of a difference to float can round, and the filter bound covers this error.
- The result keeps whole faces from the operands and builds new faces only along the curve.

`epsilon` is used for the weld of vertices, for the bounds and for the point queries. An epsilon that is not positive gives the default epsilon.

Use `CsgBuildPolygonsFromModel` or `CsgBuildSolidFromModel` to get an operand from a `Model`. Use `CsgTriangulatePolygons` and `CsgBuildModel` to get a `Model` from polygons.
`CsgSimplifyPolygons` (in `CsgOptimization.h`) simplifies a polygon list.

## Known limits

- The predicates see positions that are snapped to the grid (1/1024 units in float mode). The output keeps the unsnapped float positions.
- Rotated coplanar brushes can lose the coplanar contact. A union of such brushes can be wrong.
- `CSG_EXACT_USE_DOUBLE` makes this problem smaller. In a sweep of 1028 rotations, 114 samples fail in float mode and 25 samples fail in double mode. In these samples the union of the first two brushes is right, and the next boolean fails.
- In both modes a union of rotated brushes can keep zero-width spikes in its faces. The next boolean can then leave a hole.

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
