// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Robustness tests: a procedural box minus a procedural sphere near a corner of the box.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "Urho3D/CSG/CsgTriangulation.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Model.h>

#include <catch2/catch_amalgamated.hpp>

#include <cstdio>

using namespace Urho3D;

namespace
{

/// The result of a box of size 1 at the origin minus a sphere, as the root builds it.
struct BoxMinusSphere
{
    SharedPtr<const CsgSolid> solid_;
    CsgModelCheck check_;
    bool closed_{};
    SharedPtr<CsgTriangulatedModel> model_;
};

BoxMinusSphere MakeBoxMinusSphere(Context* context, float radius, const Vector3& center, bool smooth = true)
{
    CsgShapeMesh boxMesh;
    REQUIRE(CsgGenerateBox(CsgBoxParams{}, boxMesh));
    CsgSphereParams sphereParams;
    sphereParams.radius_ = Vector3(radius, radius, radius);
    sphereParams.smoothNormals_ = smooth;
    CsgShapeMesh sphereMesh;
    REQUIRE(CsgGenerateSphere(sphereParams, sphereMesh));

    const SharedPtr<Model> boxModel = CsgBuildModelFromMesh(context, boxMesh);
    const SharedPtr<Model> sphereModel = CsgBuildModelFromMesh(context, sphereMesh);
    REQUIRE(boxModel);
    REQUIRE(sphereModel);
    const SharedPtr<CsgSolid> box = CsgSolid::FromOperand(CsgBuildPolygonsFromModel(boxModel, Matrix3x4::IDENTITY, 1));
    const SharedPtr<CsgSolid> sphere = CsgSolid::FromOperand(
        CsgBuildPolygonsFromModel(sphereModel, Matrix3x4(center, Quaternion::IDENTITY, 1.0f), 2));

    BoxMinusSphere result;
    result.solid_ = CsgBooleanOperation(box, sphere, CsgOperation::DifferenceAB);
    REQUIRE(result.solid_);
    result.closed_ = result.solid_->IsClosed();
    result.model_ = CsgTriangulateSolid(*result.solid_);
    REQUIRE(result.model_);
    result.check_ = CsgCheckModel(*result.model_);
    return result;
}

/// The rule of `CsgBuildStats::valid_` in the root build for a result with faces.
bool IsValid(const BoxMinusSphere& r)
{
    return r.closed_ && r.check_.openEdges_ == 0 && r.check_.duplicates_ == 0 && r.check_.degenerates_ == 0
        && r.check_.volume_ > 0.0;
}

/// A small generator with a fixed sequence, so the sweep is the same on all platforms.
struct SweepRandom
{
    unsigned state_;
    unsigned Next()
    {
        state_ = state_ * 1664525u + 1013904223u;
        return state_ >> 8;
    }
    /// A value in [-0.2, 0.2] on a grid of 1/4096 units.
    float Offset() { return static_cast<float>(static_cast<int>(Next() % 1639u) - 819) / 4096.0f; }
};

/// Run `count` random cases of a box minus a sphere near the corner (0.5, 0.5, 0.5) and return the failed count.
/// A third of the cases put the center on a diagonal plane of the box (two equal offsets).
unsigned SweepCorner(Context* context, unsigned seed, unsigned count)
{
    static constexpr float radii[] = {0.25f, 0.3f, 0.5f, 0.75f};
    SweepRandom random{seed};
    unsigned failed = 0;
    for (unsigned i = 0; i < count; ++i)
    {
        const float radius = radii[random.Next() % 4u];
        float a = random.Offset();
        float b = random.Offset();
        float c = random.Offset();
        switch (random.Next() % 9u)
        {
        case 0: c = a; break;
        case 1: b = a; break;
        case 2: c = b; break;
        default: break;
        }
        const bool smooth = (random.Next() & 1u) != 0;
        const Vector3 center(0.5f + a, 0.5f + b, 0.5f + c);
        const BoxMinusSphere result = MakeBoxMinusSphere(context, radius, center, smooth);
        if (!IsValid(result))
        {
            ++failed;
            UNSCOPED_INFO("radius " << radius << " center (" << center.x_ << ", " << center.y_ << ", " << center.z_
                                    << ") duplicates " << result.check_.duplicates_ << " open " << result.check_.openEdges_);
        }
    }
    return failed;
}

} // namespace

TEST_CASE("Box minus a sphere: a crossing on the twin of a coplanar edge keeps the grid point of the twin",
    "[boolean][regression]")
{
    // The sphere has its 45 degree meridian in the plane x = z, and the edge x = z = 0.5 of the box is in this
    // plane. The box edge crosses a sphere triangle at the point where the meridian edge crosses a box face.
    // The rounding of the two twin crossings is one grid point, outside the projected sphere triangle.
    // The fold rule moved one twin a grid step away. The sphere piece between the two points then lay flat on the
    // box face, and its triangulation gave a duplicate of a triangle of the box face.
    auto context = MakeShared<Context>();
    for (const bool smooth : {true, false})
    {
        const BoxMinusSphere result = MakeBoxMinusSphere(context, 0.3f, Vector3(0.3f, 0.5f, 0.3f), smooth);
        CHECK(result.closed_);
        CHECK(result.check_.duplicates_ == 0);
        CHECK(result.check_.openEdges_ == 0);
        CHECK(result.check_.degenerates_ == 0);
        CHECK(result.solid_->FaceCount() > 0);
        CHECK(IsValid(result));
    }
}

TEST_CASE("Box minus a sphere: a crossing inside its triangle does not round onto a triangle edge",
    "[boolean][regression]")
{
    // A sphere edge crosses a box face triangle a third of a grid step from the diagonal of the face.
    // The nearest rounding is on the diagonal. The region between the curve and the diagonal got zero width,
    // and the sphere piece at the diagonal lay flat on the box face. This gave a duplicate triangle.
    auto context = MakeShared<Context>();
    for (const bool smooth : {true, false})
    {
        const BoxMinusSphere result = MakeBoxMinusSphere(context, 0.25f, Vector3(0.45f, 0.4f, 0.7f), smooth);
        CHECK(result.closed_);
        CHECK(result.check_.duplicates_ == 0);
        CHECK(result.check_.openEdges_ == 0);
        CHECK(result.check_.degenerates_ == 0);
        CHECK(result.solid_->FaceCount() > 0);
        CHECK(IsValid(result));
    }
}

TEST_CASE("Box minus a sphere near a corner gives a valid result over a random sweep", "[boolean][regression]")
{
    auto context = MakeShared<Context>();
    CHECK(SweepCorner(context, 20261119u, 80) == 0);
}

TEST_CASE("Box minus a sphere near a corner over a long random sweep", "[.][boolean][sweep]")
{
    auto context = MakeShared<Context>();
    CHECK(SweepCorner(context, 1u, 5000) == 0);
}

TEST_CASE("Box minus a sphere near a corner over a grid of centers", "[.][boolean][sweep]")
{
    // Offsets in [-0.2, 0.2] with the step 0.05, and small offsets near zero, on each axis.
    auto context = MakeShared<Context>();
    static constexpr float radii[] = {0.25f, 0.3f, 0.5f, 0.75f};
    float offsets[13];
    for (int i = -4; i <= 4; ++i)
        offsets[i + 4] = static_cast<float>(i) * 0.05f;
    offsets[9] = 1e-4f;
    offsets[10] = 1.0f / 1024.0f;
    offsets[11] = 1.0f / 2048.0f;
    offsets[12] = -1.0f / 1024.0f;
    unsigned failed = 0;
    for (const float radius : radii)
        for (const float a : offsets)
            for (const float b : offsets)
                for (const float c : offsets)
                {
                    const Vector3 center(0.5f + a, 0.5f + b, 0.5f + c);
                    if (!IsValid(MakeBoxMinusSphere(context, radius, center)))
                    {
                        ++failed;
                        UNSCOPED_INFO("radius " << radius << " center (" << center.x_ << ", " << center.y_ << ", "
                                                << center.z_ << ")");
                    }
                }
    CHECK(failed == 0);
}
