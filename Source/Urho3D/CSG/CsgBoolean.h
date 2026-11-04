// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

// This header is internal. Do not export these functions.

#include "Urho3D/CSG/CsgFaceBvh.h"
#include "Urho3D/CSG/CsgSolid.h"
#include <Urho3D/Container/Ptr.h>

namespace Urho3D
{

namespace CsgInternal
{

/// Compute a boolean operation of two solids. Both solids must be closed and not null.
///
/// An epsilon that is not positive (or NaN) gives the default epsilon.
/// An empty operand gives one of the operands as the result. The result is not a new solid.
/// An empty first operand gives the second operand for a union, and the first operand for any other operation.
/// An empty second operand gives the second operand for an intersection, and the first operand for any other operation.
/// A triangle that has a corner which is not finite is left out if the curve reaches its face.
/// Every other face stays as it is.
///
/// The parallel options only change how the face pairs of the corefinement are found. The result is the same
/// for every option. The default options give a serial run. The code forces a serial run
/// only on worker threads. A caller inside any work queue task must pass a null work queue.
///
/// The first pass uses the exact decisions of the predicates, see g_csgExactOnly. If its result has more open edges
/// than the two operands together, the boolean runs again with the thick decisions. The result with fewer open
/// edges is returned. The function sets g_csgExactOnly itself, so the value of the caller has no effect.
SharedPtr<const CsgSolid> Boolean(const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b,
    CsgOperation operation, float epsilon, const CsgParallelOptions& parallel = {});

/// Run one pass of the boolean with the current value of g_csgExactOnly and with no second pass.
/// The operands must have faces and must be in the range of the grid. Only the tests use this function.
SharedPtr<const CsgSolid> CsgBooleanOnceForTest(const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b,
    CsgOperation operation, float epsilon);

} // namespace CsgInternal

} // namespace Urho3D
