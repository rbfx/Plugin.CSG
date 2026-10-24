// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

// This header is internal. It uses CsgDouble4, which depends on the compile flags.
// Do not export these functions. Do not use them across a module boundary.

#include "Urho3D/CSG/CsgSimd.h"

namespace Urho3D
{

namespace CsgInternal
{

/// Floating point filter of CsgExact::Orient for up to three determinants u . (v x w) at once.
/// Lane k of the inputs gives the vectors of the determinant k.
/// Return a bit for each lane (bits 0 to 2) where the filter decides the sign.
/// For a decided lane, the sign of the lane of det is the sign of the determinant.
/// The function is defined in CsgExact.cpp.
int Filter(const CsgDouble4& ux, const CsgDouble4& uy, const CsgDouble4& uz, const CsgDouble4& vx,
    const CsgDouble4& vy, const CsgDouble4& vz, const CsgDouble4& wx, const CsgDouble4& wy, const CsgDouble4& wz,
    CsgDouble4& det);

} // namespace CsgInternal

} // namespace Urho3D
