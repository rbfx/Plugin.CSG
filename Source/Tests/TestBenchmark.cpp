// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Benchmark baseline for the CSG boolean. The test case has the hidden tag [.benchmark].
// A normal test run skips it. Run it with:
//   Tests.CSG "[.benchmark]" --benchmark-samples 20
//
// Baseline: host omarchy, date 2026-10-26, RelWithDebInfo build (-O2 -g -DNDEBUG), 20 samples.
// The build directory cmake-build/linux-gcc-x64-dll is the float mode.
// The build directory cmake-build/linux-gcc-x64-dll-double has -DCSG_EXACT_USE_DOUBLE=ON.
// Mean time for each call sequence:
//   case                      float        double
//   box-box union             169.6 us     188.4 us
//   box-sphere difference     4.962 ms     8.296 ms
//   rotated coplanar union    524.4 us     662.0 us
//   union chain of 8 brushes  1.376 ms     1.438 ms

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoolean.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"

#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <vector>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

using SolidPtr = SharedPtr<const CsgSolid>;

SolidPtr MakeSolid(ea::vector<CsgPolygon>&& polygons)
{
    return SolidPtr(CsgSolid::FromOperand(ea::move(polygons)));
}

SolidPtr MakeBox(const Vector3& center, unsigned shared, const Matrix3x4& transform = Matrix3x4::IDENTITY)
{
    return MakeSolid(MakeBoxPolygons(center, Vector3(1.0f, 1.0f, 1.0f), shared, transform));
}

SolidPtr Run(const SolidPtr& a, const SolidPtr& b, CsgOperation operation)
{
    return Boolean(a, b, operation, CSG_DEFAULT_EPSILON);
}

} // namespace

TEST_CASE("benchmark: booleans", "[.benchmark]")
{
    // Case 1: two overlapping boxes.
    const auto boxA = MakeBox(Vector3::ZERO, 1);
    const auto boxB = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2);

    // Case 2: a box and a sphere. The sphere crosses the face x = 1 of the box.
    const auto sphere = MakeSolid(MakeSpherePolygons(Vector3(1.0f, 0.3f, 0.2f), 1.0f, 48, 96, 1));

    // Case 3: the sweep sample at axis (1 1 1) and angle 18.5 degrees. B shares a plane with A.
    const Quaternion world(18.5f, Vector3::ONE.Normalized());
    const Quaternion own(37.0f, Vector3(1.0f, 1.0f, 0.0f).Normalized());
    const Matrix3x4 worldTransform(Vector3::ZERO, world, 1.0f);
    const auto rotA = MakeBox(Vector3::ZERO, 1, worldTransform);
    const auto rotB = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2, worldTransform);
    const auto rotC = MakeBox(Vector3(0.5f, 1.0f, 0.3f), 3, Matrix3x4(Vector3::ZERO, world * own, 1.0f));

    // Case 4: a chain of 8 boxes. Each box overlaps the one before by half a unit.
    // Each box has 10 degrees more rotation about (1 1 0) than the one before.
    std::vector<SolidPtr> chain;
    for (unsigned i = 0; i < 8; ++i)
    {
        const Quaternion rotation(10.0f * static_cast<float>(i), Vector3(1.0f, 1.0f, 0.0f).Normalized());
        chain.push_back(MakeBox(Vector3(1.5f * static_cast<float>(i), 0.0f, 0.0f), i + 1,
            Matrix3x4(Vector3::ZERO, rotation, 1.0f)));
    }

    BENCHMARK("box-box union")
    {
        return Run(boxA, boxB, CsgOperation::Union);
    };

    BENCHMARK("box-sphere difference")
    {
        return Run(boxA, sphere, CsgOperation::DifferenceAB);
    };

    BENCHMARK("rotated coplanar union")
    {
        const auto ab = Run(rotA, rotB, CsgOperation::Union);
        return Run(ab, rotC, CsgOperation::Union);
    };

    BENCHMARK("union chain of 8 brushes")
    {
        SolidPtr result = chain[0];
        for (unsigned i = 1; i < chain.size(); ++i)
            result = Run(result, chain[i], CsgOperation::Union);
        return result;
    };
}
