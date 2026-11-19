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
#include <Urho3D/Core/WorkQueue.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/IO/VirtualFileSystem.h>
#include <Urho3D/Resource/ResourceCache.h>

#include <cstdlib>
#include <fstream>
#include <limits>

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

/// Convert the mesh triangles to CSG polygons.
ea::vector<CsgPolygon> MeshToPolygons(const CsgShapeMesh& mesh)
{
    ea::vector<CsgPolygon> polygons;
    for (unsigned i = 0; i + 2 < mesh.indices_.size(); i += 3)
    {
        ea::vector<ModelVertex> vertices{mesh.vertices_[mesh.indices_[i]], mesh.vertices_[mesh.indices_[i + 1]],
            mesh.vertices_[mesh.indices_[i + 2]]};
        polygons.emplace_back(ea::move(vertices), 0u);
    }
    return polygons;
}

/// Return the volume of the CSG solid of the mesh, or -1 if the solid is not closed.
double SolidVolume(const CsgShapeMesh& mesh)
{
    SharedPtr<CsgSolid> solid = CsgSolid::FromOperand(MeshToPolygons(mesh));
    return solid && solid->IsClosed() ? solid->Volume() : -1.0;
}

/// Return true if the triangle is on the quad and has the same cyclic order.
bool IsTriangleOnQuad(const CsgPolygon& triangle, const CsgPolygon& quad)
{
    int slots[3];
    for (unsigned i = 0; i < 3; ++i)
    {
        slots[i] = -1;
        for (unsigned j = 0; j < quad.vertices_.size(); ++j)
        {
            if (triangle.vertices_[i].GetPosition().Equals(quad.vertices_[j].GetPosition(), 1e-6f))
                slots[i] = static_cast<int>(j);
        }
        if (slots[i] < 0)
            return false;
    }
    // The steps around the quad must sum to one full turn in the forward direction.
    const auto step = [](int from, int to) { return (to - from + 4) % 4; };
    return step(slots[0], slots[1]) + step(slots[1], slots[2]) + step(slots[2], slots[0]) == 4;
}

/// Return true if each triangle of the mesh has its own vertices with the same position at shared corners.
/// The test checks that no triangle has two equal positions.
bool HasNoDegenerateTriangle(const CsgShapeMesh& mesh)
{
    for (unsigned i = 0; i + 2 < mesh.indices_.size(); i += 3)
    {
        const Vector3 a = mesh.vertices_[mesh.indices_[i]].GetPosition();
        const Vector3 b = mesh.vertices_[mesh.indices_[i + 1]].GetPosition();
        const Vector3 c = mesh.vertices_[mesh.indices_[i + 2]].GetPosition();
        if (a == b || b == c || c == a)
            return false;
    }
    return true;
}

/// Return true if the test must find the CoreData models. See the preset test in TestRoot.cpp.
bool IsCoreDataRequiredForShapes()
{
#ifdef CSG_REQUIRE_CORE_DATA
    return true;
#else
    const char* value = std::getenv("CSG_REQUIRE_CORE_DATA");
    return value && *value && ea::string(value) != "0";
#endif
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

    CsgShapeMesh emptyGroup = MakeHandCubeMesh();
    emptyGroup.groups_ = {CsgShapeGroup{0, 30}, CsgShapeGroup{30, 0}, CsgShapeGroup{30, 6}};
    CHECK(!CsgBuildModelFromMesh(context, emptyGroup));

    CsgShapeMesh outOfRange = MakeHandCubeMesh();
    outOfRange.groups_ = {CsgShapeGroup{0, 30}, CsgShapeGroup{30, 9}};
    CHECK(!CsgBuildModelFromMesh(context, outOfRange));
}

TEST_CASE("CsgBuildModelFromMesh makes one geometry for each face group on shared buffers", "[shapes]")
{
    auto context = MakeShared<Context>();
    CsgShapeMesh mesh = MakeHandCubeMesh();
    mesh.groups_ = {CsgShapeGroup{0, 12}, CsgShapeGroup{12, 24}};

    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    REQUIRE(model->GetNumGeometries() == 2);
    const Geometry* first = model->GetGeometry(0, 0);
    const Geometry* second = model->GetGeometry(1, 0);
    CHECK(first->GetIndexBuffer() == second->GetIndexBuffer());
    CHECK(first->GetVertexBuffer(0) == second->GetVertexBuffer(0));
    CHECK(first->GetIndexStart() == 0);
    CHECK(first->GetIndexCount() == 12);
    CHECK(second->GetIndexStart() == 12);
    CHECK(second->GetIndexCount() == 24);

    // The polygons of each geometry come from its draw range only.
    ea::vector<CsgPolygon> polygons = CsgBuildPolygonsFromModel(model, Matrix3x4::IDENTITY);
    REQUIRE(polygons.size() == 12);
    unsigned secondCount = 0;
    for (const CsgPolygon& polygon : polygons)
        secondCount += (polygon.shared_ & 0xFFFFu) == 1 ? 1 : 0;
    CHECK(secondCount == 8);
    SharedPtr<CsgSolid> solid = CsgSolid::FromOperand(ea::move(polygons));
    CHECK(solid->IsClosed());
    CHECK(solid->Volume() == Catch::Approx(1.0).margin(1e-6));
}

TEST_CASE("CsgGenerateBox at the defaults gives the unit box with an outward winding", "[shapes]")
{
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateBox(CsgBoxParams{}, mesh));
    REQUIRE(mesh.vertices_.size() == 36);
    REQUIRE(mesh.indices_.size() == 36);
    CHECK(HasNoDegenerateTriangle(mesh));

    // The face groups are -X, +X, -Y, +Y, -Z, +Z. Each group has 2 triangles with a normal along its axis.
    const Vector3 groupNormals[6] = {
        Vector3::LEFT, Vector3::RIGHT, Vector3::DOWN, Vector3::UP, Vector3::BACK, Vector3::FORWARD};
    REQUIRE(mesh.groups_.size() == 6);
    for (unsigned group = 0; group < 6; ++group)
    {
        INFO("Group " << group);
        CHECK(mesh.groups_[group].indexStart_ == group * 6);
        CHECK(mesh.groups_[group].indexCount_ == 6);
        for (unsigned i = 0; i < 6; ++i)
        {
            const ModelVertex& vertex = mesh.vertices_[mesh.indices_[group * 6 + i]];
            CHECK(vertex.GetNormal().Equals(groupNormals[group], 1e-6f));
        }
    }

    // The model has one geometry for each face group.
    auto context = MakeShared<Context>();
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    CHECK(model->GetNumGeometries() == 6);
    CHECK(CsgBuildPolygonsFromModel(model, Matrix3x4::IDENTITY).size() == 12);

    const ea::vector<CsgPolygon> triangles = MeshToPolygons(mesh);
    CHECK(IsEdgeClosed(triangles, 1e-4f));
    // The signed volume is positive, so the winding of MakeBoxPolygons is outward for the CSG.
    CHECK(SignedVolume(triangles) == Catch::Approx(1.0).margin(1e-6));
    CHECK(SolidVolume(mesh) == Catch::Approx(1.0).margin(1e-6));

    // Each triangle is on one quad of MakeBoxPolygons with the same winding.
    const ea::vector<CsgPolygon> quads = MakeBoxPolygons(Vector3::ZERO, Vector3(0.5f, 0.5f, 0.5f), 0);
    CHECK(SignedVolume(quads) == Catch::Approx(SignedVolume(triangles)).margin(1e-6));
    ea::vector<unsigned> trianglesOnQuad(quads.size());
    for (const CsgPolygon& triangle : triangles)
    {
        bool found = false;
        for (unsigned q = 0; q < quads.size() && !found; ++q)
        {
            if (IsTriangleOnQuad(triangle, quads[q]))
            {
                ++trianglesOnQuad[q];
                found = true;
            }
        }
        CHECK(found);
    }
    for (const unsigned count : trianglesOnQuad)
        CHECK(count == 2);

    // The normals are flat and point out. The UV 0 is in local units.
    for (unsigned i = 0; i < 36; i += 3)
    {
        const ModelVertex& v0 = mesh.vertices_[i];
        CHECK(v0.GetNormal().DotProduct(v0.GetPosition()) > 0.0f);
        CHECK(v0.GetNormal().Equals(mesh.vertices_[i + 1].GetNormal()));
        CHECK(Abs(v0.tangent_.ToVector3().DotProduct(v0.GetNormal())) < 1e-6f);
        CHECK(Abs(v0.tangent_.w_) == 1.0f);
        CHECK(Abs(v0.uv_[0].x_) == Catch::Approx(0.5f));
        CHECK(v0.uv_[1].x_ >= 0.0f);
        CHECK(v0.uv_[1].x_ <= 1.0f);
    }
}

TEST_CASE("CsgGenerateBox scales with the size and keeps the texel density", "[shapes]")
{
    CsgBoxParams params;
    params.size_ = Vector3(2.0f, 3.0f, 4.0f);
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateBox(params, mesh));
    CHECK(IsEdgeClosed(MeshToPolygons(mesh), 1e-4f));
    CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(24.0).margin(1e-5));
    CHECK(SolidVolume(mesh) == Catch::Approx(24.0).margin(1e-5));

    // UV 0 is the local position on the face plane, so a face of length 4 has a UV range of 4.
    float minU = 1e9f, maxU = -1e9f;
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        if (vertex.GetNormal().Equals(Vector3::UP))
        {
            minU = Min(minU, vertex.uv_[0].x_);
            maxU = Max(maxU, vertex.uv_[0].x_);
        }
    }
    CHECK(maxU - minU == Catch::Approx(4.0f));
}

TEST_CASE("CsgGenerateBox accepts a moved corner and refuses an inverted box", "[shapes]")
{
    SECTION("A corner moved out along +X adds two tetrahedra of volume d/6")
    {
        // The value is on the grid of the solid, so the snap does not change the volume.
        const float d = 0.5f;
        CsgBoxParams params;
        params.corners_[7].x_ += d;
        CsgShapeMesh mesh;
        REQUIRE(CsgGenerateBox(params, mesh));
        CHECK(HasNoDegenerateTriangle(mesh));
        CHECK(IsEdgeClosed(MeshToPolygons(mesh), 1e-4f));
        CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(1.0 + d / 3.0).margin(1e-5));
        CHECK(SolidVolume(mesh) == Catch::Approx(1.0 + d / 3.0).margin(1e-5));
    }

    SECTION("A corner through the opposite face is not valid")
    {
        CsgBoxParams params;
        params.corners_[7].x_ = -1.0f;
        CsgShapeMesh mesh;
        CHECK(!CsgGenerateBox(params, mesh));
        CHECK(mesh.vertices_.empty());
        CHECK(mesh.indices_.empty());
        CHECK(mesh.groups_.empty());
    }

    SECTION("A mirrored box is not valid")
    {
        CsgBoxParams params;
        for (Vector3& corner : params.corners_)
            corner.x_ = -corner.x_;
        CsgShapeMesh mesh;
        CHECK(!CsgGenerateBox(params, mesh));
    }

    SECTION("A flat box is not valid")
    {
        CsgBoxParams params;
        for (Vector3& corner : params.corners_)
            corner.y_ = 0.0f;
        CsgShapeMesh mesh;
        CHECK(!CsgGenerateBox(params, mesh));
    }
}

TEST_CASE("CsgSanitizeBoxParams clamps the range and replaces values that are not finite", "[shapes]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    CsgBoxParams params;
    params.size_ = Vector3(nan, -5.0f, 1e9f);
    params.corners_[0] = Vector3(inf, -1000.0f, nan);
    params.corners_[3] = Vector3(100.0f, 0.25f, -inf);
    CsgSanitizeBoxParams(params);
    CHECK(params.size_.Equals(Vector3(1.0f, CsgBoxParams::MinSize, CsgBoxParams::MaxSize)));
    CHECK(params.corners_[0].Equals(Vector3(-0.5f, -64.0f, -0.5f)));
    CHECK(params.corners_[3].Equals(Vector3(64.0f, 0.25f, -0.5f)));

    // The generator uses the sanitized values and does not crash.
    CsgBoxParams raw;
    raw.size_ = Vector3(nan, nan, nan);
    raw.corners_[5] = Vector3(nan, inf, -inf);
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateBox(raw, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(1.0).margin(1e-6));

    // The smallest and the largest size give a valid box.
    CsgBoxParams tiny;
    tiny.size_ = Vector3(0.0f, 0.0f, 0.0f);
    CHECK(CsgGenerateBox(tiny, mesh));
    CsgBoxParams huge;
    huge.size_ = Vector3(inf, 1e30f, 1e30f);
    CHECK(CsgGenerateBox(huge, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);
}

TEST_CASE("The shape vertex layout is the layout of CoreData Box.mdl", "[shapes]")
{
    // The model load needs the resource subsystems and the graphics object factories.
    auto context = MakeShared<Context>();
    context->RegisterSubsystem<Log>();
    context->RegisterSubsystem<WorkQueue>();
    context->RegisterSubsystem<VirtualFileSystem>();
    context->RegisterSubsystem<ResourceCache>();
    RegisterGraphicsLibrary(context);
    const ea::string path = ea::string(CSG_CORE_DATA_DIR) + "/Models/Box.mdl";
    SharedPtr<Model> model;
    if (std::ifstream(path.c_str()).good())
    {
        File file(context, path, FILE_READ);
        model = MakeShared<Model>(context);
        if (!file.IsOpen() || !model->Load(file))
            model = nullptr;
    }
    if (!model)
    {
        INFO("The model is missing: " << path.c_str());
        REQUIRE(!IsCoreDataRequiredForShapes());
        WARN("Test skipped. The model is missing: " << path.c_str());
        return;
    }

    REQUIRE(model->GetNumGeometries() == 1);
    const Geometry* geometry = model->GetGeometry(0, 0);
    REQUIRE(geometry);
    REQUIRE(geometry->GetVertexBuffer(0));
    const ea::vector<VertexElement>& boxElements = geometry->GetVertexBuffer(0)->GetElements();
    const ea::vector<VertexElement>& shapeElements = CsgGetShapeVertexElements();
    REQUIRE(boxElements.size() == shapeElements.size());
    for (unsigned i = 0; i < boxElements.size(); ++i)
    {
        INFO("Element " << i);
        CHECK(boxElements[i].type_ == shapeElements[i].type_);
        CHECK(boxElements[i].semantic_ == shapeElements[i].semantic_);
        CHECK(boxElements[i].index_ == shapeElements[i].index_);
        CHECK(boxElements[i].offset_ == shapeElements[i].offset_);
        CHECK(boxElements[i].stepRate_ == shapeElements[i].stepRate_);
    }
}
