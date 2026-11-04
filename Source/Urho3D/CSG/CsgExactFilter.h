// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

// This header is internal. It uses CsgFloat4, which depends on the compile flags.
// Do not export these functions. Do not use them across a module boundary.

#include "Urho3D/CSG/CsgSimd.h"

namespace Urho3D
{

namespace CsgInternal
{

/// Floating point filter of CsgExact::OrientPlane and CsgExact::OrientLines for up to three determinants
/// u . (v x w) at once.
/// Lane k of the inputs gives the vectors of the determinant k.
/// Return a bit for each lane (bits 0 to 2) where the filter decides the sign.
/// For a decided lane, the sign of the lane of det is the sign of the determinant.
/// If g_csgExactOnly is false, a lane that can be within the thickness stays undecided.
/// The extent is the largest absolute coordinate of the common corner, or of its face if that is larger.
/// The function is defined in CsgExact.cpp.
int Filter(const CsgFloat4& ux, const CsgFloat4& uy, const CsgFloat4& uz, const CsgFloat4& vx,
    const CsgFloat4& vy, const CsgFloat4& vz, const CsgFloat4& wx, const CsgFloat4& wy, const CsgFloat4& wz,
    double extent, CsgFloat4& det);

} // namespace CsgInternal

} // namespace Urho3D
