// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Reference oracle for the exact predicates. It uses 128 bit integers and no floating point filter.

#pragma once

#include "TestInt128.h"

#include "Urho3D/CSG/CsgExact.h"

namespace CsgTest
{

/// Exact 128 bit vector.
struct ReferenceNormal
{
    Int128 x_;
    Int128 y_;
    Int128 z_;
};

/// Data of one crossing on an edge. It is the input of ReferenceCompareOnEdge.
struct ReferenceCrossing
{
    /// Plane volumes of the lower and the higher end of the edge.
    Int128 lowVolume_;
    Int128 highVolume_;
    /// Normal of the crossed triangle.
    ReferenceNormal normal_;
    /// Whether the edge belongs to the second operand.
    bool edgeInSecond_{};
};

/// Sign of (d - a) . ((b - a) x (c - a)) with the symbolic displacement of the marked points.
/// Bit i of second marks the argument i as a vertex of the second operand.
int ReferenceOrient(const Urho3D::CsgGridPoint& a, const Urho3D::CsgGridPoint& b, const Urho3D::CsgGridPoint& c,
    const Urho3D::CsgGridPoint& d, int second);

/// Unperturbed (d - a) . ((b - a) x (c - a)), exactly.
Int128 ReferenceVolume(const Urho3D::CsgGridPoint& a, const Urho3D::CsgGridPoint& b, const Urho3D::CsgGridPoint& c,
    const Urho3D::CsgGridPoint& d);

/// (b - a) x (c - a), exactly.
ReferenceNormal ReferenceNormalOf(
    const Urho3D::CsgGridPoint& a, const Urho3D::CsgGridPoint& b, const Urho3D::CsgGridPoint& c);

/// Order two crossings of one edge by their parameter. Return -1, 0 or 1.
int ReferenceCompareOnEdge(const ReferenceCrossing& x, const ReferenceCrossing& y);

} // namespace CsgTest
