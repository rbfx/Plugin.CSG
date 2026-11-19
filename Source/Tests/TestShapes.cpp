// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Tests for the procedural shape meshes. The tests use no render device.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "TestGeometry.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#include <catch2/catch_amalgamated.hpp>

using namespace Urho3D;

namespace
{

/// The four corners of each box face. Corner index bits: bit 0 is +X, bit 1 is +Y, bit 2 is +Z.
/// The order follows MakeBoxPolygons, so the cross product of the first two sides points out.
const unsigned CubeFaces[6][4] = {{1, 3, 7, 5}, {2, 6, 7, 3}, {4, 5, 7, 6}, {0, 4, 6, 2}, {0, 1, 5, 4}, {0, 2, 3, 1}};

/// Make a unit cube from 0 to 1 with 8 shared vertices and 12 triangles.
CsgShapeMesh MakeHandCubeMesh()
{
    CsgShapeMesh mesh;
    for (unsigned corner = 0; corner < 8; ++corner)
    {
        ModelVertex vertex{};
        vertex.SetPosition(Vector3(corner & 1u ? 1.0f : 0.0f, corner & 2u ? 1.0f : 0.0f, corner & 4u ? 1.0f : 0.0f));
        vertex.SetNormal(Vector3::UP);
        vertex.color_[0] = Color::WHITE.ToVector4();
        mesh.vertices_.push_back(vertex);
    }
    for (const auto& face : CubeFaces)
    {
        for (const unsigned index : {face[0], face[1], face[2], face[0], face[2], face[3]})
            mesh.indices_.push_back(index);
    }
    return mesh;
}

} // namespace

TEST_CASE("CsgBuildModelFromMesh gives a model that CSG reads as a closed solid", "[shapes]")
{
    auto context = MakeShared<Context>();
    const CsgShapeMesh mesh = MakeHandCubeMesh();

    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    REQUIRE(model->GetNumGeometries() == 1);
    CHECK(model->GetName().empty());
    CHECK(model->GetBoundingBox().min_.Equals(Vector3::ZERO));
    CHECK(model->GetBoundingBox().max_.Equals(Vector3::ONE));

    // The geometry uses the shape layout.
    const Geometry* geometry = model->GetGeometry(0, 0);
    REQUIRE(geometry);
    REQUIRE(geometry->GetVertexBuffer(0));
    const ea::vector<VertexElement>& elements = geometry->GetVertexBuffer(0)->GetElements();
    const ea::vector<VertexElement>& expected = CsgGetShapeVertexElements();
    REQUIRE(elements.size() == expected.size());
    for (unsigned i = 0; i < elements.size(); ++i)
    {
        CHECK(elements[i].type_ == expected[i].type_);
        CHECK(elements[i].semantic_ == expected[i].semantic_);
        CHECK(elements[i].index_ == expected[i].index_);
        CHECK(elements[i].offset_ == expected[i].offset_);
    }

    ea::vector<CsgPolygon> polygons = CsgBuildPolygonsFromModel(model, Matrix3x4::IDENTITY);
    REQUIRE(polygons.size() == 12);
    CHECK(SignedVolume(polygons) == Catch::Approx(1.0).margin(1e-6));
    SharedPtr<CsgSolid> solid = CsgSolid::FromOperand(ea::move(polygons));
    REQUIRE(solid);
    CHECK(solid->IsClosed());
    CHECK(solid->Volume() == Catch::Approx(1.0).margin(1e-6));
}

TEST_CASE("CsgBuildModelFromMesh refuses a mesh that is not valid", "[shapes]")
{
    auto context = MakeShared<Context>();
    CHECK(!CsgBuildModelFromMesh(context, CsgShapeMesh{}));

    CsgShapeMesh badIndex = MakeHandCubeMesh();
    badIndex.indices_[5] = 8;
    CHECK(!CsgBuildModelFromMesh(context, badIndex));

    CsgShapeMesh partialTriangle = MakeHandCubeMesh();
    partialTriangle.indices_.pop_back();
    CHECK(!CsgBuildModelFromMesh(context, partialTriangle));
}
