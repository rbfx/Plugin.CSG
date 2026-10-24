// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "TestGeometry.h"

#include "Urho3D/CSG/CsgCommon.h"

#include <catch2/catch_amalgamated.hpp>

using namespace Urho3D;

TEST_CASE("Box polygons have the expected volume")
{
    const auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 0);
    REQUIRE(polygons.size() == 6);
    CHECK(SignedVolume(polygons) == Catch::Approx(8.0).epsilon(1e-5));
}

TEST_CASE("Box polygons are edge-closed")
{
    const auto polygons = MakeBoxPolygons(Vector3(1.0f, 2.0f, 3.0f), Vector3(1.0f, 2.0f, 0.5f), 0);
    CHECK(IsEdgeClosed(polygons, CSG_DEFAULT_EPSILON));
}

TEST_CASE("Box polygons with a transform keep the volume")
{
    const Matrix3x4 transform(Vector3(5.0f, 0.0f, 0.0f), Quaternion(30.0f, Vector3::UP), 1.0f);
    const auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 0, transform);
    CHECK(SignedVolume(polygons) == Catch::Approx(8.0).epsilon(1e-4));
    CHECK(IsEdgeClosed(polygons, CSG_DEFAULT_EPSILON));
}

TEST_CASE("Sphere polygons have the expected volume and are edge-closed")
{
    const float radius = 2.0f;
    const auto polygons = MakeSpherePolygons(Vector3(1.0f, 0.0f, -1.0f), radius, 16, 32, 0);
    const double expected = 4.0 / 3.0 * M_PI * radius * radius * radius;
    const double volume = SignedVolume(polygons);
    CHECK(volume > 0.0);
    CHECK(std::abs(volume - expected) < 0.05 * expected);
    CHECK(IsEdgeClosed(polygons, CSG_DEFAULT_EPSILON));
}

TEST_CASE("An open mesh is not edge-closed")
{
    auto polygons = MakeBoxPolygons(Vector3::ZERO, Vector3::ONE, 0);
    polygons.pop_back();
    CHECK_FALSE(IsEdgeClosed(polygons, CSG_DEFAULT_EPSILON));
}

TEST_CASE("Packed keys")
{
    CHECK(CsgPackedKeyHash()(CsgDirectedKey(1, 2)) != CsgPackedKeyHash()(CsgDirectedKey(2, 1)));
    CHECK(CsgDirectedKey(1, 2) != CsgDirectedKey(2, 1));
    CHECK(CsgUndirectedKey(1, 2) == CsgUndirectedKey(2, 1));
}
