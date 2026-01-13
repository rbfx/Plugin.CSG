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
