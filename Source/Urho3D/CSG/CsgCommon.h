// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/vector.h>

#include <cstdint>

namespace Urho3D
{

struct CsgPolygon;
class ModelVertex;

static constexpr float CSG_DEFAULT_EPSILON = 0.0001f;

/// Pick a stable perpendicular axis for the provided normal.
PLUGIN_RBFXCSG_API Vector3 MakePerpendicular(const Vector3& normal);
/// Make stable orthonormal basis vectors for the provided normal.
PLUGIN_RBFXCSG_API void MakeAxesFromNormal(const Vector3& normal, Vector3& axisU, Vector3& axisV);
/// Quantize position into integer key using 1/epsilon scale.
PLUGIN_RBFXCSG_API IntVector3 QuantizePosition(const Vector3& p, float invEps);
/// Lerp vertex attributes for CSG operations.
PLUGIN_RBFXCSG_API ModelVertex LerpModelVertex(const ModelVertex& a, const ModelVertex& b, float t);

/// Pack an ordered index pair into a key. (a, b) and (b, a) give different keys.
PLUGIN_RBFXCSG_API uint64_t CsgDirectedKey(unsigned a, unsigned b);
/// Pack an index pair into a key. (a, b) and (b, a) give the same key.
PLUGIN_RBFXCSG_API uint64_t CsgUndirectedKey(unsigned a, unsigned b);

/// Hash for packed 64-bit keys. It uses a full avalanche mix.
/// The default hash of a 64-bit key puts packed index pairs into few buckets.
struct CsgPackedKeyHash
{
    size_t operator()(uint64_t key) const noexcept
    {
        key ^= key >> 33;
        key *= 0xff51afd7ed558ccdULL;
        key ^= key >> 33;
        key *= 0xc4ceb9fe1a85ec53ULL;
        key ^= key >> 33;
        return static_cast<size_t>(key);
    }
};

}   // namespace Urho3D
