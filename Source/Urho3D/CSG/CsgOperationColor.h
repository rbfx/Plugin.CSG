// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Urho3D/Math/Color.h>

namespace Urho3D
{

/// The debug colors of the CSG operations. The index is the value of `CsgOperation`.
/// The header has no link dependency. The editor addon uses it without the CSG library.
static const Color CsgOperationColors[] = {
    Color(0.3f, 0.9f, 0.3f), // Union
    Color(0.95f, 0.3f, 0.25f), // DifferenceAB
    Color(0.3f, 0.55f, 1.0f), // Intersection
};

/// The debug color for an operation index that is not in the table.
static const Color CsgUnknownOperationColor{1.0f, 0.85f, 0.0f};

/// Return the opaque debug color of the operation index.
inline Color GetOperationColor(int operation)
{
    const int count = static_cast<int>(sizeof(CsgOperationColors) / sizeof(CsgOperationColors[0]));
    return operation >= 0 && operation < count ? CsgOperationColors[operation] : CsgUnknownOperationColor;
}

} // namespace Urho3D
