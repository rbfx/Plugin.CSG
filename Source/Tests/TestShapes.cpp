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

#include <cmath>
#include <cstdlib>
#include <cstring>
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

namespace
{

/// Return the exact volume of the unit UV polyhedron, computed from the frustums between the ring planes.
/// Each band between two latitudes is a frustum of a regular polygon pyramid.
/// A pole fan is a cone. The test does not use the mesh generator.
double UnitUvPolyhedronVolume(unsigned segments, unsigned rings)
{
    const double pi = 3.14159265358979323846;
    const auto area = [&](double r) { return 0.5 * segments * r * r * std::sin(2.0 * pi / segments); };
    double volume = 0.0;
    for (unsigned row = 0; row < rings; ++row)
    {
        const double theta0 = pi * row / rings;
        const double theta1 = pi * (row + 1) / rings;
        const double a0 = area(std::sin(theta0));
        const double a1 = area(std::sin(theta1));
        const double height = std::cos(theta0) - std::cos(theta1);
        volume += height / 3.0 * (a0 + a1 + std::sqrt(a0 * a1));
    }
    return volume;
}

/// Return true if the two vertices have bit-identical positions.
bool SamePositionBits(const ModelVertex& a, const ModelVertex& b)
{
    const Vector3 pa = a.GetPosition();
    const Vector3 pb = b.GetPosition();
    return std::memcmp(&pa, &pb, sizeof(Vector3)) == 0;
}

} // namespace

TEST_CASE("CsgGenerateSphere gives a closed mesh without degenerate triangles", "[shapes]")
{
    const unsigned cases[][2] = {{3, 2}, {3, 3}, {4, 2}, {24, 12}, {7, 5}, {128, 64}, {128, 2}, {3, 64}};
    for (const auto& c : cases)
    {
        INFO("Segments " << c[0] << " rings " << c[1]);
        CsgSphereParams params;
        params.segments_ = c[0];
        params.rings_ = c[1];
        CsgShapeMesh mesh;
        REQUIRE(CsgGenerateSphere(params, mesh));
        // Fans at the poles: 2 * segments triangles. Quads in between: 2 triangles each.
        const unsigned triangles = 2 * c[0] + 2 * c[0] * (c[1] - 2);
        CHECK(mesh.indices_.size() == 3 * triangles);
        CHECK(mesh.vertices_.size() == 3 * triangles);
        REQUIRE(mesh.groups_.size() == 1);
        CHECK(mesh.groups_[0].indexStart_ == 0);
        CHECK(mesh.groups_[0].indexCount_ == mesh.indices_.size());
        CHECK(HasNoDegenerateTriangle(mesh));
        const ea::vector<CsgPolygon> polygons = MeshToPolygons(mesh);
        CHECK(IsEdgeClosed(polygons, 1e-6f));
        CHECK(SignedVolume(polygons) > 0.0);
        CHECK(SolidVolume(mesh) > 0.0);
    }
}

TEST_CASE("CsgGenerateSphere volume matches the exact UV polyhedron and the ellipsoid", "[shapes]")
{
    const unsigned cases[][2] = {{3, 2}, {24, 12}, {128, 64}, {9, 7}};
    for (const auto& c : cases)
    {
        INFO("Segments " << c[0] << " rings " << c[1]);
        CsgSphereParams params;
        params.segments_ = c[0];
        params.rings_ = c[1];
        params.radius_ = Vector3(1.0f, 2.0f, 3.0f);
        CsgShapeMesh mesh;
        REQUIRE(CsgGenerateSphere(params, mesh));
        const double expected = UnitUvPolyhedronVolume(c[0], c[1]) * 6.0;
        const ea::vector<CsgPolygon> polygons = MeshToPolygons(mesh);
        CHECK(SignedVolume(polygons) == Catch::Approx(expected).epsilon(1e-4));
        // The solid snaps the positions to a grid, so its tolerance is wider.
        CHECK(SolidVolume(mesh) == Catch::Approx(expected).epsilon(1e-3));
    }

    // The default sphere is close to 4/3 pi r^3 with r = 0.5.
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(CsgSphereParams{}, mesh));
    const double analytic = 4.0 / 3.0 * 3.14159265358979323846 * 0.125;
    const double volume = SignedVolume(MeshToPolygons(mesh));
    CHECK(volume == Catch::Approx(UnitUvPolyhedronVolume(24, 12) * 0.125).epsilon(1e-4));
    CHECK(volume == Catch::Approx(analytic).epsilon(0.04));
    CHECK(volume < analytic);

    // A fine sphere is close to the analytic ellipsoid volume.
    CsgSphereParams fine;
    fine.segments_ = 128;
    fine.rings_ = 64;
    fine.radius_ = Vector3(1.0f, 2.0f, 3.0f);
    REQUIRE(CsgGenerateSphere(fine, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(4.0 / 3.0 * 3.14159265358979323846 * 6.0).epsilon(2e-3));
}

TEST_CASE("CsgGenerateSphere has outward smooth normals, tangents and UV", "[shapes]")
{
    CsgSphereParams params;
    params.radius_ = Vector3(1.0f, 2.0f, 3.0f);
    params.segments_ = 16;
    params.rings_ = 8;
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(params, mesh));
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        const Vector3 p = vertex.GetPosition();
        const Vector3 expected = Vector3(p.x_ / 1.0f, p.y_ / 4.0f, p.z_ / 9.0f).Normalized();
        CHECK(vertex.GetNormal().Equals(expected, 1e-5f));
        CHECK(vertex.GetNormal().Length() == Catch::Approx(1.0f).margin(1e-5));
        CHECK(Abs(vertex.tangent_.ToVector3().DotProduct(vertex.GetNormal())) < 1e-5f);
        CHECK(vertex.tangent_.ToVector3().Length() == Catch::Approx(1.0f).margin(1e-5));
        CHECK(Abs(vertex.tangent_.w_) == 1.0f);
        // The longitude of the tangent: its Y part is zero, only for the vertices off the poles.
        CHECK(vertex.uv_[0].x_ >= 0.0f);
        CHECK(vertex.uv_[0].x_ <= 1.0f);
        CHECK(vertex.uv_[0].y_ >= 0.0f);
        CHECK(vertex.uv_[0].y_ <= 1.0f);
        CHECK(vertex.uv_[1] == vertex.uv_[0]);
        // V is the latitude: 0 at the top pole (+Y) and 1 at the bottom pole.
        const float latitude = std::acos(Clamp(p.y_ / 2.0f, -1.0f, 1.0f)) / 3.14159265f;
        CHECK(vertex.uv_[0].y_ == Catch::Approx(latitude).margin(1e-5));
        if (vertex.uv_[0].y_ > 0.0f && vertex.uv_[0].y_ < 1.0f)
        {
            // U is the longitude, counted from +X toward +Z.
            const float longitude = std::atan2(p.z_ / 3.0f, p.x_ / 1.0f) / (2.0f * 3.14159265f);
            const float wrapped = longitude < 0.0f ? longitude + 1.0f : longitude;
            const float u = vertex.uv_[0].x_;
            CHECK((Abs(u - wrapped) < 1e-5f || Abs(Abs(u - wrapped) - 1.0f) < 1e-5f));
            // The tangent follows the longitude: +U direction, and the bitangent follows +V (toward the bottom).
            CHECK(Abs(vertex.tangent_.y_) < 1e-5f);
            const Vector3 bitangent = vertex.tangent_.ToVector3().CrossProduct(vertex.GetNormal()) * vertex.tangent_.w_;
            CHECK(bitangent.y_ < 0.0f);
        }
    }

    // The flat normals are one for each triangle and point out.
    params.smoothNormals_ = false;
    REQUIRE(CsgGenerateSphere(params, mesh));
    for (unsigned i = 0; i < mesh.vertices_.size(); i += 3)
    {
        const Vector3 p0 = mesh.vertices_[i].GetPosition();
        const Vector3 p1 = mesh.vertices_[i + 1].GetPosition();
        const Vector3 p2 = mesh.vertices_[i + 2].GetPosition();
        const Vector3 normal = (p1 - p0).CrossProduct(p2 - p0).Normalized();
        for (unsigned k = 0; k < 3; ++k)
        {
            CHECK(mesh.vertices_[i + k].GetNormal().Equals(normal, 1e-5f));
            CHECK(Abs(mesh.vertices_[i + k].tangent_.ToVector3().DotProduct(normal)) < 1e-5f);
        }
        const Vector3 center = (p0 + p1 + p2) / 3.0f;
        CHECK(normal.DotProduct(center) > 0.0f);
    }
}

TEST_CASE("CsgGenerateSphere shares bit-identical positions at the seam and the poles", "[shapes]")
{
    CsgSphereParams params;
    params.radius_ = Vector3(1.3f, 0.7f, 2.1f);
    params.segments_ = 13;
    params.rings_ = 6;
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(params, mesh));

    const ModelVertex* topPole = nullptr;
    const ModelVertex* bottomPole = nullptr;
    unsigned seamVertices = 0;
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        const float u = vertex.uv_[0].x_;
        const float v = vertex.uv_[0].y_;
        if (v == 0.0f || v == 1.0f)
        {
            const ModelVertex*& pole = v == 0.0f ? topPole : bottomPole;
            if (!pole)
                pole = &vertex;
            CHECK(SamePositionBits(vertex, *pole));
            CHECK(vertex.GetPosition().x_ == 0.0f);
            CHECK(vertex.GetPosition().z_ == 0.0f);
            continue;
        }
        if (u != 1.0f)
            continue;
        // A seam vertex copies the position of the vertex with u == 0 in the same row.
        ++seamVertices;
        bool found = false;
        for (const ModelVertex& other : mesh.vertices_)
            found = found || (other.uv_[0].x_ == 0.0f && other.uv_[0].y_ == v && SamePositionBits(vertex, other));
        CHECK(found);
    }
    REQUIRE(topPole);
    REQUIRE(bottomPole);
    CHECK(topPole->GetPosition().y_ == 0.7f);
    CHECK(bottomPole->GetPosition().y_ == -0.7f);
    // The seam has 2 triangles with 3 seam corners over the (rings - 2) middle bands, and 1 for each fan.
    CHECK(seamVertices > 0);

    // Any two vertices with nearly equal positions have bit-identical positions.
    for (unsigned i = 0; i < mesh.vertices_.size(); ++i)
    {
        for (unsigned j = i + 1; j < mesh.vertices_.size(); ++j)
        {
            if (mesh.vertices_[i].GetPosition().Equals(mesh.vertices_[j].GetPosition(), 1e-6f))
                CHECK(SamePositionBits(mesh.vertices_[i], mesh.vertices_[j]));
        }
    }

    // Each ring is symmetric around the Y axis: the ring has `segments` different positions.
    ea::vector<Vector3> positions;
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        const Vector3 p = vertex.GetPosition();
        bool known = false;
        for (const Vector3& q : positions)
            known = known || std::memcmp(&p, &q, sizeof(Vector3)) == 0;
        if (!known)
            positions.push_back(p);
    }
    CHECK(positions.size() == 2 + (params.rings_ - 1) * params.segments_);
}

TEST_CASE("CsgSanitizeSphereParams clamps the range and replaces values that are not finite", "[shapes]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    CsgSphereParams params;
    params.radius_ = Vector3(nan, -5.0f, 1e9f);
    params.segments_ = 1;
    params.rings_ = 1000;
    CsgSanitizeSphereParams(params);
    CHECK(params.radius_.Equals(Vector3(0.5f, CsgSphereParams::MinRadius, CsgSphereParams::MaxRadius)));
    CHECK(params.segments_ == 3);
    CHECK(params.rings_ == 64);

    params.segments_ = 100000;
    params.rings_ = -3;
    CsgSanitizeSphereParams(params);
    CHECK(params.segments_ == 128);
    CHECK(params.rings_ == 2);

    // The generator uses the sanitized values and does not crash.
    CsgSphereParams raw;
    raw.radius_ = Vector3(nan, inf, -inf);
    raw.segments_ = 0;
    raw.rings_ = 0;
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(raw, mesh));
    CHECK(mesh.indices_.size() == 3 * 2 * 3);
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);

    CsgSphereParams tiny;
    tiny.radius_ = Vector3(0.0f, 0.0f, 0.0f);
    CHECK(CsgGenerateSphere(tiny, mesh));
    CsgSphereParams huge;
    huge.radius_ = Vector3(inf, 1e30f, 1e30f);
    CHECK(CsgGenerateSphere(huge, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);
}

TEST_CASE("CsgGenerateSphere gives the vertex layout of Box.mdl in the model", "[shapes]")
{
    auto context = MakeShared<Context>();
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateSphere(CsgSphereParams{}, mesh));
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    CHECK(model->GetNumGeometries() == 1);
    const Geometry* geometry = model->GetGeometry(0, 0);
    REQUIRE(geometry);
    REQUIRE(geometry->GetVertexBuffer(0));
    // The layout of Box.mdl is compared to the shape layout in the Box.mdl test above.
    const ea::vector<VertexElement>& shapeElements = CsgGetShapeVertexElements();
    const ea::vector<VertexElement>& elements = geometry->GetVertexBuffer(0)->GetElements();
    REQUIRE(elements.size() == shapeElements.size());
    for (unsigned i = 0; i < elements.size(); ++i)
    {
        CHECK(elements[i].type_ == shapeElements[i].type_);
        CHECK(elements[i].semantic_ == shapeElements[i].semantic_);
        CHECK(elements[i].index_ == shapeElements[i].index_);
        CHECK(elements[i].offset_ == shapeElements[i].offset_);
    }
}

namespace
{

/// Return the independent volume of the cylinder polyhedron, from the frustum formula for each slab.
/// The rows are polygons with equal ratio of the two radii, so each slab is a frustum with planar sides.
/// The test does not use the mesh generator.
double ExpectedCylinderVolume(const CsgCylinderParams& p)
{
    const double pi = 3.14159265358979323846;
    const double n = p.segments_;
    const auto area = [&](double t)
    {
        const double k = (1.0 + p.bulge_ * 4.0 * t * (1.0 - t));
        const double rx = (p.bottomRadius_.x_ + (p.topRadius_.x_ - p.bottomRadius_.x_) * t) * k;
        const double rz = (p.bottomRadius_.y_ + (p.topRadius_.y_ - p.bottomRadius_.y_) * t) * k;
        return 0.5 * n * rx * rz * std::sin(2.0 * pi / n);
    };
    const double height = std::fabs(static_cast<double>(p.topCenter_.y_) - p.bottomCenter_.y_) / p.heightSegments_;
    double volume = 0.0;
    for (int row = 0; row < p.heightSegments_; ++row)
    {
        const double a0 = area(static_cast<double>(row) / p.heightSegments_);
        const double a1 = area(static_cast<double>(row + 1) / p.heightSegments_);
        volume += height / 3.0 * (a0 + a1 + std::sqrt(a0 * a1));
    }
    return volume;
}

/// The cylinder variants of the tests.
ea::vector<CsgCylinderParams> MakeCylinderVariants(unsigned segments, unsigned heightSegments)
{
    ea::vector<CsgCylinderParams> variants;
    CsgCylinderParams base;
    base.segments_ = segments;
    base.heightSegments_ = heightSegments;
    variants.push_back(base);

    CsgCylinderParams topCone = base;
    topCone.topRadius_ = Vector2::ZERO;
    variants.push_back(topCone);

    CsgCylinderParams bottomCone = base;
    bottomCone.bottomRadius_ = Vector2::ZERO;
    bottomCone.bottomCenter_ = Vector3(1.0f, -2.0f, 0.5f);
    variants.push_back(bottomCone);

    CsgCylinderParams frustum = base;
    frustum.bottomRadius_ = Vector2(1.0f, 0.5f);
    frustum.topRadius_ = Vector2(0.4f, 0.2f);
    variants.push_back(frustum);

    CsgCylinderParams bulged = base;
    bulged.bulge_ = 2.0f;
    variants.push_back(bulged);

    CsgCylinderParams tilted = base;
    tilted.bulge_ = -0.5f;
    tilted.topCenter_ = Vector3(1.5f, 1.0f, -0.75f);
    tilted.bottomRadius_ = Vector2(0.8f, 0.8f);
    variants.push_back(tilted);

    CsgCylinderParams bulgedCone = base;
    bulgedCone.bulge_ = 1.0f;
    bulgedCone.topRadius_ = Vector2::ZERO;
    variants.push_back(bulgedCone);
    return variants;
}

unsigned CountDistinctPositions(const CsgShapeMesh& mesh)
{
    ea::vector<Vector3> positions;
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        const Vector3 p = vertex.GetPosition();
        bool known = false;
        for (const Vector3& q : positions)
            known = known || std::memcmp(&p, &q, sizeof(Vector3)) == 0;
        if (!known)
            positions.push_back(p);
    }
    return positions.size();
}

} // namespace

TEST_CASE("CsgGenerateCylinder gives a closed mesh without degenerate triangles", "[shapes][procedural]")
{
    const unsigned cases[][2] = {{3, 1}, {128, 64}, {24, 1}, {24, 4}, {7, 3}};
    for (const auto& c : cases)
    {
        for (const CsgCylinderParams& params : MakeCylinderVariants(c[0], c[1]))
        {
            INFO("Segments " << c[0] << " height segments " << c[1] << " bulge " << params.bulge_
                             << " top " << params.topRadius_.x_ << " bottom " << params.bottomRadius_.x_);
            CsgShapeMesh mesh;
            REQUIRE(CsgGenerateCylinder(params, mesh));
            const bool bottomApex = params.bottomRadius_ == Vector2::ZERO;
            const bool topApex = params.topRadius_ == Vector2::ZERO;
            const unsigned apexes = (bottomApex ? 1 : 0) + (topApex ? 1 : 0);
            const unsigned sideTriangles = 2 * c[0] * c[1] - apexes * c[0];
            const unsigned capTriangles = (2 - apexes) * c[0];
            CHECK(mesh.indices_.size() == 3 * (sideTriangles + capTriangles));
            CHECK(mesh.vertices_.size() == mesh.indices_.size());
            CHECK(HasNoDegenerateTriangle(mesh));
            const ea::vector<CsgPolygon> polygons = MeshToPolygons(mesh);
            CHECK(IsEdgeClosed(polygons, 1e-6f));
            CHECK(SignedVolume(polygons) > 0.0);
            if (c[0] * c[1] <= 400)
                CHECK(SolidVolume(mesh) > 0.0);

            // The group order is side, top cap, bottom cap. A cone end has no group.
            unsigned expectedGroups = 1 + (topApex ? 0 : 1) + (bottomApex ? 0 : 1);
            REQUIRE(mesh.groups_.size() == expectedGroups);
            CHECK(mesh.groups_[0].indexStart_ == 0);
            CHECK(mesh.groups_[0].indexCount_ == 3 * sideTriangles);
            CHECK(mesh.groups_[0].slot_ == 0);
            unsigned next = 3 * sideTriangles;
            for (unsigned i = 1; i < mesh.groups_.size(); ++i)
            {
                CHECK(mesh.groups_[i].indexStart_ == next);
                CHECK(mesh.groups_[i].indexCount_ == 3 * c[0]);
                next += mesh.groups_[i].indexCount_;
            }
            CHECK(next == mesh.indices_.size());
        }
    }
}

TEST_CASE("CsgGenerateCylinder volume matches the independent polyhedron volume", "[shapes][procedural]")
{
    const double pi = 3.14159265358979323846;
    // The default cylinder: n / 2 * sin(2pi / n) * r^2 * h.
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(CsgCylinderParams{}, mesh));
    const double cylinder = 24.0 / 2.0 * std::sin(2.0 * pi / 24.0) * 0.25 * 1.0;
    CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(cylinder).epsilon(1e-5));
    CHECK(SolidVolume(mesh) == Catch::Approx(cylinder).epsilon(1e-3));

    // A frustum with circular ends: h / 3 * (A0 + A1 + sqrt(A0 A1)).
    CsgCylinderParams frustum;
    frustum.bottomRadius_ = Vector2(1.0f, 1.0f);
    frustum.topRadius_ = Vector2(0.5f, 0.5f);
    frustum.topCenter_ = Vector3(0.0f, 2.0f, 0.0f);
    REQUIRE(CsgGenerateCylinder(frustum, mesh));
    const double area0 = 12.0 * std::sin(2.0 * pi / 24.0) * 1.0;
    const double area1 = 12.0 * std::sin(2.0 * pi / 24.0) * 0.25;
    CHECK(SignedVolume(MeshToPolygons(mesh)) ==
        Catch::Approx(2.5 / 3.0 * (area0 + area1 + std::sqrt(area0 * area1))).epsilon(1e-5));

    // A cone: h / 3 * A.
    CsgCylinderParams cone;
    cone.topRadius_ = Vector2::ZERO;
    REQUIRE(CsgGenerateCylinder(cone, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(1.0 / 3.0 * 12.0 * std::sin(2.0 * pi / 24.0) * 0.25).epsilon(1e-5));

    // Slabs, bulge, shear, tilt, elliptical ends with one ratio, and a cone with a bulge.
    for (const CsgCylinderParams& base : MakeCylinderVariants(24, 4))
    {
        for (const float bulge : {0.0f, 2.0f, -0.9f})
        {
            CsgCylinderParams params = base;
            params.bulge_ = bulge;
            // Skip the variants with different ratios.
            if (params.bottomRadius_.x_ != params.bottomRadius_.y_ && params.bottomRadius_ != Vector2::ZERO)
                params.bottomRadius_ = Vector2(1.0f, 0.5f);
            if (params.topRadius_ != Vector2::ZERO)
                params.topRadius_ = params.bottomRadius_ == Vector2::ZERO ? Vector2(0.4f, 0.4f) : params.bottomRadius_ * 0.5f;
            if (params.bottomRadius_ == Vector2::ZERO && params.topRadius_ == Vector2::ZERO)
                continue;
            INFO("Bulge " << bulge);
            REQUIRE(CsgGenerateCylinder(params, mesh));
            CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(ExpectedCylinderVolume(params)).epsilon(2e-5));
        }
    }

    // The fine mesh is close to pi r^2 h.
    CsgCylinderParams fine;
    fine.segments_ = 128;
    fine.heightSegments_ = 64;
    REQUIRE(CsgGenerateCylinder(fine, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(pi * 0.25).epsilon(1e-3));
}

TEST_CASE("CsgGenerateCylinder keeps the volume positive when the centers are swapped in Y", "[shapes][procedural]")
{
    for (const CsgCylinderParams& base : MakeCylinderVariants(12, 3))
    {
        CsgCylinderParams swapped = base;
        swapped.bottomCenter_ = base.topCenter_;
        swapped.topCenter_ = base.bottomCenter_;
        swapped.bottomRadius_ = base.topRadius_;
        swapped.topRadius_ = base.bottomRadius_;
        if (swapped.bottomRadius_ == Vector2::ZERO && swapped.topRadius_ == Vector2::ZERO)
            continue;
        CsgShapeMesh mesh;
        CsgShapeMesh original;
        REQUIRE(CsgGenerateCylinder(base, original));
        REQUIRE(CsgGenerateCylinder(swapped, mesh));
        const ea::vector<CsgPolygon> polygons = MeshToPolygons(mesh);
        CHECK(IsEdgeClosed(polygons, 1e-6f));
        CHECK(SignedVolume(polygons) > 0.0);
        CHECK(SignedVolume(polygons) == Catch::Approx(SignedVolume(MeshToPolygons(original))).epsilon(1e-4));
        CHECK(SolidVolume(mesh) > 0.0);
    }

    // Only the centers are swapped: the shape is the same cylinder upside down. Normals point out.
    CsgCylinderParams params;
    params.bottomCenter_ = Vector3(0.0f, 0.5f, 0.0f);
    params.topCenter_ = Vector3(0.0f, -0.5f, 0.0f);
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(params, mesh));
    REQUIRE(mesh.groups_.size() == 3);
    // The top cap is at the topCenter_, which is now at y = -0.5, and its normal is -Y.
    const ModelVertex& topCapVertex = mesh.vertices_[mesh.indices_[mesh.groups_[1].indexStart_]];
    CHECK(topCapVertex.GetPosition().y_ == -0.5f);
    CHECK(topCapVertex.GetNormal().Equals(Vector3::DOWN, 1e-5f));
    for (const ModelVertex& vertex : mesh.vertices_)
    {
        const Vector3 p = vertex.GetPosition();
        CHECK(vertex.GetNormal().DotProduct(Vector3(p.x_, 0.0f, p.z_)) >= -1e-5f);
    }
}

TEST_CASE("CsgGenerateCylinder has outward smooth normals, flat caps and UV", "[shapes][procedural]")
{
    for (const CsgCylinderParams& base : MakeCylinderVariants(16, 3))
    {
        for (const bool smooth : {true, false})
        {
            CsgCylinderParams params = base;
            params.smoothNormals_ = smooth;
            // Use the straight axis to check the normals against the body.
            params.topCenter_ = Vector3(0.0f, 1.0f, 0.0f);
            params.bottomCenter_ = Vector3(0.0f, -1.0f, 0.0f);
            CsgShapeMesh mesh;
            REQUIRE(CsgGenerateCylinder(params, mesh));
            for (unsigned g = 0; g < mesh.groups_.size(); ++g)
            {
                const CsgShapeGroup& group = mesh.groups_[g];
                for (unsigned i = group.indexStart_; i < group.indexStart_ + group.indexCount_; ++i)
                {
                    const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i]];
                    const Vector3 n = vertex.GetNormal();
                    INFO("Group " << g << " smooth " << smooth);
                    CHECK(n.Length() == Catch::Approx(1.0).margin(1e-4));
                    const Vector3 t = Vector3(vertex.tangent_.x_, vertex.tangent_.y_, vertex.tangent_.z_);
                    CHECK(t.Length() == Catch::Approx(1.0).margin(1e-4));
                    CHECK(std::fabs(t.DotProduct(n)) < 1e-3f);
                    CHECK(vertex.uv_[0].x_ == vertex.uv_[1].x_);
                    CHECK(vertex.uv_[0].y_ == vertex.uv_[1].y_);
                    const Vector3 p = vertex.GetPosition();
                    if (g == 0)
                    {
                        // The side normal has an outward radial part, except at an apex.
                        const bool apex = p.x_ == 0.0f && p.z_ == 0.0f;
                        if (!apex)
                            CHECK(n.DotProduct(Vector3(p.x_, 0.0f, p.z_)) > 0.0f);
                        if (apex)
                            CHECK(std::fabs(n.y_) > 0.0f);
                        // UV: u is the angle, v is the body fraction.
                        CHECK(vertex.uv_[0].y_ >= 0.0f);
                        CHECK(vertex.uv_[0].y_ <= 1.0f);
                        CHECK(vertex.uv_[0].x_ >= 0.0f);
                        CHECK(vertex.uv_[0].x_ <= 1.0f);
                    }
                    else
                    {
                        // A cap normal is flat and points away from the body.
                        CHECK(n.Equals(group.slot_ == 1 ? Vector3::UP : Vector3::DOWN, 1e-5f));
                        CHECK(vertex.uv_[0].x_ == Catch::Approx(p.x_ + 0.5f).margin(1e-6));
                        CHECK(vertex.uv_[0].y_ == Catch::Approx(p.z_ + 0.5f).margin(1e-6));
                    }
                }
            }
            if (!smooth)
            {
                // Flat: the normal of a triangle is its face normal.
                for (unsigned i = 0; i + 2 < mesh.indices_.size(); i += 3)
                {
                    const Vector3 p0 = mesh.vertices_[mesh.indices_[i]].GetPosition();
                    const Vector3 p1 = mesh.vertices_[mesh.indices_[i + 1]].GetPosition();
                    const Vector3 p2 = mesh.vertices_[mesh.indices_[i + 2]].GetPosition();
                    const Vector3 face = (p1 - p0).CrossProduct(p2 - p0).Normalized();
                    for (unsigned k = 0; k < 3; ++k)
                        CHECK(mesh.vertices_[mesh.indices_[i + k]].GetNormal().Equals(face, 1e-4f));
                }
            }
        }
    }

    // The smooth normal of a straight cylinder is radial. The slope of a frustum tilts it up.
    CsgCylinderParams straight;
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(straight, mesh));
    for (unsigned i = 0; i < mesh.groups_[0].indexCount_; ++i)
    {
        const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i]];
        const Vector3 p = vertex.GetPosition();
        CHECK(vertex.GetNormal().Equals(Vector3(p.x_, 0.0f, p.z_).Normalized(), 1e-4f));
    }
    CsgCylinderParams frustum;
    frustum.topRadius_ = Vector2(0.25f, 0.25f);
    REQUIRE(CsgGenerateCylinder(frustum, mesh));
    for (unsigned i = 0; i < mesh.groups_[0].indexCount_; ++i)
        CHECK(mesh.vertices_[mesh.indices_[i]].GetNormal().y_ > 0.0f);
    // The slope matches the analytic value: tan(a) = (0.5 - 0.25) / 1.
    const Vector3 n = mesh.vertices_[mesh.indices_[0]].GetNormal();
    CHECK(n.y_ / Vector3(n.x_, 0.0f, n.z_).Length() == Catch::Approx(0.25).epsilon(1e-3));
}

TEST_CASE("CsgGenerateCylinder shares bit-identical positions at the seam, the apex and the rims", "[shapes][procedural]")
{
    for (const CsgCylinderParams& base : MakeCylinderVariants(13, 3))
    {
        CsgCylinderParams params = base;
        params.heightSegments_ = 3;
        CsgShapeMesh mesh;
        REQUIRE(CsgGenerateCylinder(params, mesh));
        const bool bottomApex = params.bottomRadius_ == Vector2::ZERO;
        const bool topApex = params.topRadius_ == Vector2::ZERO;

        // The position table: the rows (13 positions, 1 at an apex) and the cap centers.
        const unsigned expected = 13 * 2 + (bottomApex ? 1 : 13) + (topApex ? 1 : 13)
            + (bottomApex ? 0 : 1) + (topApex ? 0 : 1);
        CHECK(CountDistinctPositions(mesh) == expected);

        // The seam vertices (u == 1) copy the position of a vertex with u == 0 in the same row.
        for (unsigned i = 0; i < mesh.groups_[0].indexCount_; ++i)
        {
            const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i]];
            if (vertex.uv_[0].x_ != 1.0f)
                continue;
            bool found = false;
            for (const ModelVertex& other : mesh.vertices_)
                found = found || (other.uv_[0].x_ == 0.0f && other.uv_[0].y_ == vertex.uv_[0].y_ && SamePositionBits(vertex, other));
            CHECK(found);
        }

        // Any two vertices with nearly equal positions have bit-identical positions.
        for (unsigned i = 0; i < mesh.vertices_.size(); ++i)
        {
            for (unsigned j = i + 1; j < mesh.vertices_.size(); ++j)
            {
                if (mesh.vertices_[i].GetPosition().Equals(mesh.vertices_[j].GetPosition(), 1e-6f))
                    CHECK(SamePositionBits(mesh.vertices_[i], mesh.vertices_[j]));
            }
        }

        // The apex is the end center. The cap center is the end center.
        for (const ModelVertex& vertex : mesh.vertices_)
        {
            const Vector3 p = vertex.GetPosition();
            if (bottomApex && p.x_ == params.bottomCenter_.x_ && p.z_ == params.bottomCenter_.z_ && p.y_ == params.bottomCenter_.y_)
                CHECK(vertex.uv_[0].y_ == 0.0f);
            if (topApex && p == params.topCenter_)
                CHECK(vertex.uv_[0].y_ == 1.0f);
        }

        // Each rim vertex of a cap has the position of a body vertex of the end ring.
        for (unsigned g = 1; g < mesh.groups_.size(); ++g)
        {
            const CsgShapeGroup& group = mesh.groups_[g];
            const Vector3 center = group.slot_ == 1 ? params.topCenter_ : params.bottomCenter_;
            const float v = group.slot_ == 1 ? 1.0f : 0.0f;
            for (unsigned i = group.indexStart_; i < group.indexStart_ + group.indexCount_; ++i)
            {
                const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i]];
                if (vertex.GetPosition() == center)
                    continue;
                bool found = false;
                for (unsigned j = 0; j < mesh.groups_[0].indexCount_; ++j)
                {
                    const ModelVertex& body = mesh.vertices_[mesh.indices_[j]];
                    found = found || (body.uv_[0].y_ == v && SamePositionBits(vertex, body));
                }
                CHECK(found);
            }
        }
    }

    // A pole of a cone is exactly the center.
    CsgCylinderParams cone;
    cone.topRadius_ = Vector2::ZERO;
    cone.topCenter_ = Vector3(0.3f, 0.7f, -0.1f);
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(cone, mesh));
    bool foundApex = false;
    for (const ModelVertex& vertex : mesh.vertices_)
        foundApex = foundApex || vertex.GetPosition() == cone.topCenter_;
    CHECK(foundApex);
}

TEST_CASE("CsgGenerateCylinder refuses shapes that are not valid", "[shapes][procedural]")
{
    CsgShapeMesh mesh;
    mesh.indices_.push_back(1);

    CsgCylinderParams coincident;
    coincident.topCenter_ = coincident.bottomCenter_;
    CHECK(!CsgGenerateCylinder(coincident, mesh));
    CHECK(mesh.vertices_.empty());
    CHECK(mesh.indices_.empty());
    CHECK(mesh.groups_.empty());

    CsgCylinderParams close;
    close.topCenter_ = close.bottomCenter_ + Vector3(0.0f, 0.0005f, 0.0f);
    CHECK(!CsgGenerateCylinder(close, mesh));
    close.topCenter_ = close.bottomCenter_ + Vector3(0.0f, 0.002f, 0.0f);
    CHECK(CsgGenerateCylinder(close, mesh));

    CsgCylinderParams both;
    both.bottomRadius_ = Vector2::ZERO;
    both.topRadius_ = Vector2(0.0f, 0.0005f);
    CHECK(!CsgGenerateCylinder(both, mesh));

    // One zero component is raised to the minimum: the end is an ellipse, not an apex.
    CsgCylinderParams oneZero;
    oneZero.topRadius_ = Vector2(0.0f, 0.5f);
    REQUIRE(CsgGenerateCylinder(oneZero, mesh));
    CHECK(mesh.groups_.size() == 3);
    CsgSanitizeCylinderParams(oneZero);
    CHECK(oneZero.topRadius_.x_ == CsgCylinderParams::MinRadius);
    CHECK(oneZero.topRadius_.y_ == 0.5f);

    // Cone with a horizontal axis: the end planes are the same plane.
    CsgCylinderParams horizontal;
    horizontal.bottomCenter_ = Vector3(-0.5f, 0.0f, 0.0f);
    horizontal.topCenter_ = Vector3(0.5f, 0.0f, 0.0f);
    CHECK(!CsgGenerateCylinder(horizontal, mesh));

    // A near-horizontal axis: the Y offset is below the minimum, so a sliver is refused.
    CsgCylinderParams nearHorizontal;
    nearHorizontal.bottomCenter_ = Vector3(-0.5f, 0.0f, 0.0f);
    nearHorizontal.topCenter_ = Vector3(0.5f, 1e-4f, 0.0f);
    CHECK(!CsgGenerateCylinder(nearHorizontal, mesh));
    CHECK(mesh.vertices_.empty());
    nearHorizontal.topCenter_ = Vector3(0.5f, 0.002f, 0.0f);
    REQUIRE(CsgGenerateCylinder(nearHorizontal, mesh));
    CHECK(mesh.groups_.size() == 3);
    CHECK(IsEdgeClosed(MeshToPolygons(mesh), 1e-6f));
    CHECK(SolidVolume(mesh) > 0.0);
}

TEST_CASE("CsgSanitizeCylinderParams clamps the range and replaces values that are not finite", "[shapes][procedural]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    CsgCylinderParams params;
    params.bottomCenter_ = Vector3(nan, -1e9f, 1e9f);
    params.topCenter_ = Vector3(inf, 5.0f, -inf);
    params.bottomRadius_ = Vector2(nan, 1e9f);
    params.topRadius_ = Vector2(-1.0f, -2.0f);
    params.segments_ = 1;
    params.heightSegments_ = 1000;
    params.bulge_ = nan;
    CsgSanitizeCylinderParams(params);
    CHECK(params.bottomCenter_.Equals(Vector3(0.0f, -CsgCylinderParams::MaxCoordinate, CsgCylinderParams::MaxCoordinate)));
    CHECK(params.topCenter_.Equals(Vector3(0.0f, 5.0f, 0.0f)));
    CHECK(params.bottomRadius_.Equals(Vector2(0.5f, CsgCylinderParams::MaxRadius)));
    CHECK(params.topRadius_ == Vector2::ZERO);
    CHECK(params.segments_ == 3);
    CHECK(params.heightSegments_ == 64);
    CHECK(params.bulge_ == 0.0f);

    params.segments_ = 100000;
    params.heightSegments_ = -3;
    params.bulge_ = 100.0f;
    CsgSanitizeCylinderParams(params);
    CHECK(params.segments_ == 128);
    CHECK(params.heightSegments_ == 1);
    CHECK(params.bulge_ == CsgCylinderParams::MaxBulge);
    params.bulge_ = -100.0f;
    CsgSanitizeCylinderParams(params);
    CHECK(params.bulge_ == CsgCylinderParams::MinBulge);

    // The generator uses the sanitized values and does not crash.
    CsgCylinderParams raw;
    raw.bottomRadius_ = Vector2(nan, inf);
    raw.topRadius_ = Vector2(-inf, 1e30f);
    raw.segments_ = 0;
    raw.heightSegments_ = 0;
    raw.bulge_ = inf;
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(raw, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);

    // The extreme bulge does not give a negative radius.
    CsgCylinderParams bulged;
    bulged.bulge_ = CsgCylinderParams::MinBulge;
    REQUIRE(CsgGenerateCylinder(bulged, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);
    CHECK(HasNoDegenerateTriangle(mesh));
}

TEST_CASE("CsgBuildModelFromMesh uses the slot of the group as the geometry index", "[shapes][procedural]")
{
    auto context = MakeShared<Context>();

    // A mesh with a free slot gets an empty geometry in it.
    CsgShapeMesh mesh = MakeHandCubeMesh();
    mesh.groups_ = {CsgShapeGroup{0, 12, 0}, CsgShapeGroup{12, 24, 2}};
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    REQUIRE(model->GetNumGeometries() == 3);
    CHECK(model->GetGeometry(0, 0)->GetIndexCount() == 12);
    CHECK(model->GetGeometry(1, 0)->GetIndexCount() == 0);
    CHECK(model->GetGeometry(2, 0)->GetIndexStart() == 12);
    CHECK(model->GetGeometry(2, 0)->GetIndexCount() == 24);
    ea::vector<CsgPolygon> polygons = CsgBuildPolygonsFromModel(model, Matrix3x4::IDENTITY);
    REQUIRE(polygons.size() == 12);
    for (const CsgPolygon& polygon : polygons)
        CHECK((polygon.shared_ & 0xFFFFu) != 1);

    // The default slot is the group index.
    mesh.groups_ = {CsgShapeGroup{0, 12}, CsgShapeGroup{12, 24}};
    model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    CHECK(model->GetNumGeometries() == 2);

    // Two groups in one slot, and a slot that is too large, are refused.
    mesh.groups_ = {CsgShapeGroup{0, 12, 1}, CsgShapeGroup{12, 24, 1}};
    CHECK(!CsgBuildModelFromMesh(context, mesh));
    mesh.groups_ = {CsgShapeGroup{0, 12, 0}, CsgShapeGroup{12, 24, 1000}};
    CHECK(!CsgBuildModelFromMesh(context, mesh));
    mesh.groups_ = {CsgShapeGroup{0, 12}, CsgShapeGroup{12, 24, 0}};
    CHECK(!CsgBuildModelFromMesh(context, mesh));
}

TEST_CASE("CsgGenerateCylinder gives the cone group and slot mapping in the model", "[shapes][procedural]")
{
    auto context = MakeShared<Context>();

    // The default cylinder: the side, the top cap and the bottom cap.
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(CsgCylinderParams{}, mesh));
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    CHECK(model->GetNumGeometries() == 3);
    CHECK(model->GetGeometry(0, 0)->GetIndexCount() == 3 * 48);
    CHECK(model->GetGeometry(1, 0)->GetIndexCount() == 3 * 24);
    CHECK(model->GetGeometry(2, 0)->GetIndexCount() == 3 * 24);

    // A cone with the bottom apex: side = slot 0, top cap = slot 1, no slot 2.
    CsgCylinderParams bottomApex;
    bottomApex.bottomRadius_ = Vector2::ZERO;
    REQUIRE(CsgGenerateCylinder(bottomApex, mesh));
    REQUIRE(mesh.groups_.size() == 2);
    CHECK(mesh.groups_[0].slot_ == 0);
    CHECK(mesh.groups_[1].slot_ == 1);
    model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    CHECK(model->GetNumGeometries() == 2);
    // The top cap normal is +Y.
    const Geometry* cap = model->GetGeometry(1, 0);
    CHECK(cap->GetIndexCount() == 3 * 24);
    CHECK(mesh.vertices_[mesh.indices_[cap->GetIndexStart()]].GetNormal().Equals(Vector3::UP));

    // A cone with the top apex: side = slot 0, no slot 1, bottom cap = slot 2.
    CsgCylinderParams topApex;
    topApex.topRadius_ = Vector2::ZERO;
    REQUIRE(CsgGenerateCylinder(topApex, mesh));
    REQUIRE(mesh.groups_.size() == 2);
    CHECK(mesh.groups_[0].slot_ == 0);
    CHECK(mesh.groups_[1].slot_ == 2);
    model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    REQUIRE(model->GetNumGeometries() == 3);
    CHECK(model->GetGeometry(1, 0)->GetIndexCount() == 0);
    const Geometry* bottomCap = model->GetGeometry(2, 0);
    CHECK(bottomCap->GetIndexCount() == 3 * 24);
    CHECK(mesh.vertices_[mesh.indices_[bottomCap->GetIndexStart()]].GetNormal().Equals(Vector3::DOWN));
    ea::vector<CsgPolygon> polygons = CsgBuildPolygonsFromModel(model, Matrix3x4::IDENTITY);
    CHECK(polygons.size() == 24 + 24);
    SharedPtr<CsgSolid> solid = CsgSolid::FromOperand(ea::move(polygons));
    REQUIRE(solid);
    CHECK(solid->IsClosed());
}

TEST_CASE("CsgGenerateCylinder gives the vertex layout of Box.mdl in the model", "[shapes][procedural]")
{
    auto context = MakeShared<Context>();
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateCylinder(CsgCylinderParams{}, mesh));
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
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
        CHECK(elements[i].offset_ == expected[i].offset_);
    }
}

namespace
{

/// Return the independent volume of the ring polyhedron.
/// The wedge between two section planes with the angle step d is a prismatoid with planar side faces.
/// Its volume is sin(d) * integral of the radial coordinate over the section polygon.
/// The test computes the integral with the shoelace formula. It does not use the mesh generator.
double ExpectedRingVolume(const CsgRingParams& p)
{
    const double pi = 3.14159265358979323846;
    const unsigned m = p.tubeSegments_;
    double integral = 0.0;
    for (unsigned j = 0; j < m; ++j)
    {
        const double a0 = (p.tubeAngle_ + 360.0 * j / m) * pi / 180.0;
        const double a1 = (p.tubeAngle_ + 360.0 * (j + 1) / m) * pi / 180.0;
        const double x0 = p.radius_ + p.tubeRadius_.x_ * std::cos(a0);
        const double y0 = p.tubeRadius_.y_ * std::sin(a0);
        const double x1 = p.radius_ + p.tubeRadius_.x_ * std::cos(a1);
        const double y1 = p.tubeRadius_.y_ * std::sin(a1);
        integral += (x0 + x1) * (x0 * y1 - x1 * y0) / 6.0;
    }
    const double step = p.arc_ * pi / 180.0 / p.segments_;
    return p.segments_ * std::sin(step) * integral;
}

/// The ring variants of the tests.
ea::vector<CsgRingParams> MakeRingVariants(unsigned segments, unsigned tubeSegments)
{
    ea::vector<CsgRingParams> variants;
    for (const float tubeAngle : {0.0f, 45.0f})
    {
        for (const float arc : {360.0f, 90.0f, 180.0f})
        {
            for (const bool elliptic : {false, true})
            {
                CsgRingParams params;
                params.segments_ = segments;
                params.tubeSegments_ = tubeSegments;
                params.tubeAngle_ = tubeAngle;
                params.arc_ = arc;
                if (elliptic)
                {
                    params.radius_ = 1.2f;
                    params.tubeRadius_ = Vector2(0.4f, 0.15f);
                }
                variants.push_back(params);
            }
        }
    }
    return variants;
}

} // namespace

TEST_CASE("CsgGenerateRing gives a closed mesh without degenerate triangles", "[shapes][procedural]")
{
    const unsigned cases[][2] = {{3, 3}, {128, 64}, {32, 12}, {32, 4}, {7, 5}};
    for (const auto& c : cases)
    {
        for (const CsgRingParams& params : MakeRingVariants(c[0], c[1]))
        {
            INFO("Segments " << c[0] << " tube segments " << c[1] << " tube angle " << params.tubeAngle_ << " arc "
                             << params.arc_ << " tube " << params.tubeRadius_.x_);
            CsgShapeMesh mesh;
            REQUIRE(CsgGenerateRing(params, mesh));
            const bool full = params.arc_ >= 360.0f;
            const unsigned bodyTriangles = 2 * c[0] * c[1];
            const unsigned capTriangles = full ? 0 : 2 * c[1];
            CHECK(mesh.indices_.size() == 3 * (bodyTriangles + capTriangles));
            CHECK(mesh.vertices_.size() == mesh.indices_.size());
            CHECK(HasNoDegenerateTriangle(mesh));
            const ea::vector<CsgPolygon> polygons = MeshToPolygons(mesh);
            CHECK(IsEdgeClosed(polygons, 1e-6f));
            CHECK(SignedVolume(polygons) > 0.0);
            if (c[0] * c[1] <= 400)
                CHECK(SolidVolume(mesh) > 0.0);

            // The groups follow the order, do not overlap and cover the index list.
            unsigned next = 0;
            unsigned lastSlot = 0;
            for (unsigned i = 0; i < mesh.groups_.size(); ++i)
            {
                const CsgShapeGroup& group = mesh.groups_[i];
                CHECK(group.indexStart_ == next);
                CHECK(group.indexCount_ > 0);
                CHECK(group.indexCount_ % 3 == 0);
                CHECK(group.slot_ < 6);
                CHECK((i == 0 || group.slot_ > lastSlot));
                lastSlot = group.slot_;
                next += group.indexCount_;
            }
            CHECK(next == mesh.indices_.size());
        }
    }
}

TEST_CASE("CsgGenerateRing volume matches the independent polyhedron volume", "[shapes][procedural]")
{
    const double pi = 3.14159265358979323846;
    CsgShapeMesh mesh;
    for (const auto& c : {ea::pair<unsigned, unsigned>{3, 3}, {32, 12}, {32, 4}, {128, 64}, {9, 7}})
    {
        for (const CsgRingParams& params : MakeRingVariants(c.first, c.second))
        {
            INFO("Segments " << c.first << " tube segments " << c.second << " tube angle " << params.tubeAngle_
                             << " arc " << params.arc_ << " tube " << params.tubeRadius_.x_);
            REQUIRE(CsgGenerateRing(params, mesh));
            CHECK(SignedVolume(MeshToPolygons(mesh)) == Catch::Approx(ExpectedRingVolume(params)).epsilon(1e-4));
        }
    }

    // The fine full ring is close to the torus volume 2 pi^2 R a b. The polyhedron is inside the torus.
    CsgRingParams fine;
    fine.segments_ = 128;
    fine.tubeSegments_ = 64;
    fine.radius_ = 1.0f;
    fine.tubeRadius_ = Vector2(0.4f, 0.2f);
    REQUIRE(CsgGenerateRing(fine, mesh));
    const double torus = 2.0 * pi * pi * 1.0 * 0.4 * 0.2;
    const double volume = SignedVolume(MeshToPolygons(mesh));
    CHECK(volume < torus);
    CHECK(volume == Catch::Approx(torus).epsilon(5e-3));

    // The default ring has a CSG solid with the same volume.
    REQUIRE(CsgGenerateRing(CsgRingParams{}, mesh));
    CHECK(SolidVolume(mesh) == Catch::Approx(ExpectedRingVolume(CsgRingParams{})).epsilon(3e-3));
}

TEST_CASE("CsgGenerateRing has analytic normals, flat caps and UV", "[shapes][procedural]")
{
    for (const CsgRingParams& base : MakeRingVariants(16, 10))
    {
        for (const bool smooth : {true, false})
        {
            CsgRingParams params = base;
            params.smoothNormals_ = smooth;
            INFO("Smooth " << smooth << " tube angle " << params.tubeAngle_ << " arc " << params.arc_);
            CsgShapeMesh mesh;
            REQUIRE(CsgGenerateRing(params, mesh));
            const double arcRad = params.arc_ * 3.14159265358979323846 / 180.0;
            const double a = params.tubeRadius_.x_;
            const double b = params.tubeRadius_.y_;

            for (const CsgShapeGroup& group : mesh.groups_)
            {
                for (unsigned i = group.indexStart_; i < group.indexStart_ + group.indexCount_; i += 3)
                {
                    const Vector3 p0 = mesh.vertices_[mesh.indices_[i]].GetPosition();
                    const Vector3 p1 = mesh.vertices_[mesh.indices_[i + 1]].GetPosition();
                    const Vector3 p2 = mesh.vertices_[mesh.indices_[i + 2]].GetPosition();
                    const Vector3 flat = (p1 - p0).CrossProduct(p2 - p0).Normalized();
                    for (unsigned k = 0; k < 3; ++k)
                    {
                        const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i + k]];
                        const Vector3 p = vertex.GetPosition();
                        const Vector3 n = vertex.GetNormal();
                        CHECK(n.Length() == Catch::Approx(1.0f).margin(1e-3));
                        CHECK(vertex.uv_[0] == vertex.uv_[1]);
                        if (group.slot_ >= 4)
                        {
                            // A cap is flat. The normal points along the end tangent, away from the ring.
                            CHECK(n.Equals(flat, 1e-3f));
                            const double phi = std::atan2(p.z_, p.x_);
                            const Vector3 along(static_cast<float>(-std::sin(phi)), 0.0f, static_cast<float>(std::cos(phi)));
                            CHECK(n.Equals(group.slot_ == 4 ? -along : along, 1e-3f));
                            CHECK(vertex.uv_[0].x_ >= -1e-5f);
                            continue;
                        }
                        if (smooth)
                        {
                            // The analytic normal of the elliptic section.
                            const double rho = std::hypot(p.x_, p.z_) - params.radius_;
                            const Vector3 radial(p.x_, 0.0f, p.z_);
                            const Vector3 expected =
                                (radial.Normalized() * static_cast<float>(b * rho / a) + Vector3::UP * static_cast<float>(a * p.y_ / b))
                                    .Normalized();
                            CHECK(n.Equals(expected, 2e-3f));
                        }
                        else
                        {
                            CHECK(n.Equals(flat, 1e-3f));
                        }
                        // The normal points out of the tube: away from the tube center.
                        const double rho = std::hypot(p.x_, p.z_) - params.radius_;
                        const Vector3 radial = Vector3(p.x_, 0.0f, p.z_).Normalized();
                        CHECK(n.DotProduct(radial * static_cast<float>(rho) + Vector3::UP * p.y_) > 0.0f);

                        // UV 0: the segment fraction and the tube fraction.
                        const float u = vertex.uv_[0].x_;
                        const float v = vertex.uv_[0].y_;
                        CHECK(u >= 0.0f);
                        CHECK(u <= 1.0f);
                        CHECK(v >= 0.0f);
                        CHECK(v <= 1.0f);
                        CHECK(u * 16.0f == Catch::Approx(std::round(u * 16.0f)).margin(1e-4));
                        CHECK(v * 10.0f == Catch::Approx(std::round(v * 10.0f)).margin(1e-4));
                        double phi = std::atan2(p.z_, p.x_);
                        if (phi < -1e-6)
                            phi += 2.0 * 3.14159265358979323846;
                        if (u == 1.0f)
                            phi += arcRad > 6.28 ? 2.0 * 3.14159265358979323846 : 0.0;
                        CHECK(std::fmod(phi, 2.0 * 3.14159265358979323846) == Catch::Approx(std::fmod(u * arcRad, 2.0 * 3.14159265358979323846)).margin(1e-3));
                        // The tangent follows the U axis and is perpendicular to the normal.
                        const Vector3 tangent = vertex.GetTangent();
                        CHECK(tangent.DotProduct(n) == Catch::Approx(0.0f).margin(1e-3));
                        const Vector3 along(static_cast<float>(-std::sin(phi)), 0.0f, static_cast<float>(std::cos(phi)));
                        CHECK(tangent.DotProduct(along) > 0.9f);
                    }
                }
            }
        }
    }
}

TEST_CASE("CsgGenerateRing shares bit-identical positions at the seams and the cap rims", "[shapes][procedural]")
{
    for (const float arc : {360.0f, 120.0f})
    {
        CsgRingParams params;
        params.segments_ = 7;
        params.tubeSegments_ = 5;
        params.arc_ = arc;
        params.tubeAngle_ = 13.0f;
        const bool full = arc >= 360.0f;
        INFO("Arc " << arc);
        CsgShapeMesh mesh;
        REQUIRE(CsgGenerateRing(params, mesh));

        // The position table: 7 rings of 5 positions (8 for an arc) and the 2 section centers of an arc.
        CHECK(CountDistinctPositions(mesh) == (full ? 7u * 5u : 8u * 5u + 2u));

        // A seam vertex copies the position of the vertex at the start, in the same tube row.
        for (const CsgShapeGroup& group : mesh.groups_)
        {
            if (group.slot_ >= 4)
                continue;
            for (unsigned k = group.indexStart_; k < group.indexStart_ + group.indexCount_; ++k)
            {
                const ModelVertex& vertex = mesh.vertices_[mesh.indices_[k]];
                // The end of a full ring wraps to u 0. The end of an arc is its own ring.
                const float u = full && vertex.uv_[0].x_ == 1.0f ? 0.0f : vertex.uv_[0].x_;
                const float v = vertex.uv_[0].y_ == 1.0f ? 0.0f : vertex.uv_[0].y_;
                bool found = false;
                for (const ModelVertex& other : mesh.vertices_)
                    found = found || (other.uv_[0].x_ == u && other.uv_[0].y_ == v && SamePositionBits(vertex, other));
                CHECK(found);
            }
        }

        // Any two vertices with nearly equal positions have bit-identical positions.
        for (unsigned i = 0; i < mesh.vertices_.size(); ++i)
        {
            for (unsigned j = i + 1; j < mesh.vertices_.size(); ++j)
            {
                if (mesh.vertices_[i].GetPosition().Equals(mesh.vertices_[j].GetPosition(), 1e-6f))
                    CHECK(SamePositionBits(mesh.vertices_[i], mesh.vertices_[j]));
            }
        }

        // Each cap rim vertex equals a body vertex of the end ring. The cap center is the section center.
        for (const CsgShapeGroup& group : mesh.groups_)
        {
            if (group.slot_ < 4)
                continue;
            const float endU = group.slot_ == 4 ? 0.0f : 1.0f;
            const double phi = group.slot_ == 4 ? 0.0 : params.arc_ * 3.14159265358979323846 / 180.0;
            const Vector3 center(static_cast<float>(params.radius_ * std::cos(phi)), 0.0f, static_cast<float>(params.radius_ * std::sin(phi)));
            unsigned numCenter = 0;
            for (unsigned i = group.indexStart_; i < group.indexStart_ + group.indexCount_; ++i)
            {
                const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i]];
                if (vertex.GetPosition().Equals(center, 1e-6f))
                {
                    ++numCenter;
                    continue;
                }
                bool found = false;
                for (unsigned j = 0; j < mesh.groups_[0].indexStart_ + mesh.indices_.size(); ++j)
                {
                    if (j >= mesh.indices_.size())
                        break;
                    if (j >= group.indexStart_)
                        break;
                    const ModelVertex& body = mesh.vertices_[mesh.indices_[j]];
                    found = found || (body.uv_[0].x_ == endU && SamePositionBits(vertex, body));
                }
                CHECK(found);
            }
            CHECK(numCenter == 5);
        }
    }
}

TEST_CASE("CsgGenerateRing gives the group order and the slot mapping", "[shapes][procedural]")
{
    auto context = MakeShared<Context>();
    CsgShapeMesh mesh;

    // A full ring: the four surface groups. The caps 4 and 5 are absent. The model has four geometries.
    // The tube angle 7 puts no quad middle on a quadrant border: 12 quads in the quadrants 3 + 3 + 3 + 3.
    CsgRingParams params;
    params.segments_ = 8;
    params.tubeSegments_ = 12;
    params.tubeAngle_ = 7.0f;
    REQUIRE(CsgGenerateRing(params, mesh));
    REQUIRE(mesh.groups_.size() == 4);
    for (unsigned i = 0; i < 4; ++i)
    {
        CHECK(mesh.groups_[i].slot_ == i);
        CHECK(mesh.groups_[i].indexCount_ == 3 * 2 * 8 * 3);
    }

    // The quad of a group is in the expected quadrant of the section angle.
    // Middle angles are 22, 52, 82, 112, 142, 172, 202, 232, 262, 292, 322 and 352 degrees.
    // Quadrant to group: outer 0, inner 1, top 2, bottom 3.
    const auto expectedGroup = [](unsigned j)
    {
        const unsigned byQuad[12] = {0, 2, 2, 2, 1, 1, 1, 3, 3, 3, 0, 0};
        return byQuad[j];
    };
    for (const CsgShapeGroup& group : mesh.groups_)
    {
        for (unsigned i = group.indexStart_; i < group.indexStart_ + group.indexCount_; i += 3)
        {
            float minV = 2.0f;
            for (unsigned k = 0; k < 3; ++k)
                minV = ea::min(minV, mesh.vertices_[mesh.indices_[i + k]].uv_[0].y_);
            const unsigned j = static_cast<unsigned>(std::lround(minV * 12.0f));
            CHECK(expectedGroup(j) == group.slot_);
        }
    }
    SharedPtr<Model> model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    CHECK(model->GetNumGeometries() == 4);

    // The normal of the group: the outer group faces away from the axis, the top group faces up.
    const auto averageNormal = [&](const CsgShapeGroup& group)
    {
        Vector3 sum;
        for (unsigned i = group.indexStart_; i < group.indexStart_ + group.indexCount_; ++i)
        {
            const ModelVertex& vertex = mesh.vertices_[mesh.indices_[i]];
            const Vector3 p = vertex.GetPosition();
            // The radial component is relative to the position, so the sum does not cancel around the ring.
            const Vector3 radial = Vector3(p.x_, 0.0f, p.z_).Normalized();
            const Vector3 n = vertex.GetNormal();
            sum += Vector3(n.DotProduct(radial), n.y_, 0.0f);
        }
        return sum;
    };
    CHECK(averageNormal(mesh.groups_[0]).x_ > 0.0f);
    CHECK(averageNormal(mesh.groups_[1]).x_ < 0.0f);
    CHECK(averageNormal(mesh.groups_[2]).y_ > 0.0f);
    CHECK(averageNormal(mesh.groups_[3]).y_ < 0.0f);

    // An arc: the groups 4 and 5 are the caps. The model has six geometries.
    params.arc_ = 90.0f;
    REQUIRE(CsgGenerateRing(params, mesh));
    REQUIRE(mesh.groups_.size() == 6);
    for (unsigned i = 0; i < 6; ++i)
        CHECK(mesh.groups_[i].slot_ == i);
    CHECK(mesh.groups_[4].indexCount_ == 3 * 12);
    CHECK(mesh.groups_[5].indexCount_ == 3 * 12);
    CHECK(mesh.vertices_[mesh.indices_[mesh.groups_[4].indexStart_]].GetNormal().Equals(Vector3::BACK, 1e-5f));
    CHECK(mesh.vertices_[mesh.indices_[mesh.groups_[5].indexStart_]].GetNormal().Equals(Vector3::LEFT, 1e-5f));
    model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    CHECK(model->GetNumGeometries() == 6);

    // The diamond (4 tube segments, tube angle 0): the quad middles are on the borders and go to the next quadrant.
    // Each quadrant has one quad. The outer group is the quad around 315 degrees.
    CsgRingParams diamond;
    diamond.tubeSegments_ = 4;
    REQUIRE(CsgGenerateRing(diamond, mesh));
    REQUIRE(mesh.groups_.size() == 4);
    for (unsigned i = 0; i < 4; ++i)
        CHECK(mesh.groups_[i].indexCount_ == 3 * 2 * 32);
    // The square (tube angle 45): the quad middles are on the axes. One group each.
    CsgRingParams square = diamond;
    square.tubeAngle_ = 45.0f;
    REQUIRE(CsgGenerateRing(square, mesh));
    REQUIRE(mesh.groups_.size() == 4);
    // Group 2 (top): the quad from 45 to 135 degrees. Its vertices have a positive Y.
    for (unsigned i = mesh.groups_[2].indexStart_; i < mesh.groups_[2].indexStart_ + mesh.groups_[2].indexCount_; ++i)
        CHECK(mesh.vertices_[mesh.indices_[i]].GetPosition().y_ > 0.0f);
    for (unsigned i = mesh.groups_[3].indexStart_; i < mesh.groups_[3].indexStart_ + mesh.groups_[3].indexCount_; ++i)
        CHECK(mesh.vertices_[mesh.indices_[i]].GetPosition().y_ < 0.0f);

    // A model with an empty quadrant keeps the slots. Three tube segments at tube angle 0: the middles are
    // at 60, 180 and 300 degrees: top, inner and bottom. No outer quad.
    CsgRingParams triangle;
    triangle.tubeSegments_ = 3;
    REQUIRE(CsgGenerateRing(triangle, mesh));
    REQUIRE(mesh.groups_.size() == 3);
    CHECK(mesh.groups_[0].slot_ == 1);
    CHECK(mesh.groups_[1].slot_ == 2);
    CHECK(mesh.groups_[2].slot_ == 3);
    triangle.tubeAngle_ = 90.0f;
    // The middles are at 150, 270 and 30 degrees: inner, bottom and outer. No top quad.
    REQUIRE(CsgGenerateRing(triangle, mesh));
    REQUIRE(mesh.groups_.size() == 3);
    CHECK(mesh.groups_[0].slot_ == 0);
    CHECK(mesh.groups_[1].slot_ == 1);
    CHECK(mesh.groups_[2].slot_ == 3);
    model = CsgBuildModelFromMesh(context, mesh);
    REQUIRE(model);
    REQUIRE(model->GetNumGeometries() == 4);
    CHECK(model->GetGeometry(2, 0)->GetIndexCount() == 0);
    CHECK(model->GetGeometry(3, 0)->GetIndexCount() == 3 * 2 * 32);
}

TEST_CASE("CsgSanitizeRingParams clamps the range and replaces values that are not finite", "[shapes][procedural]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    CsgRingParams params;
    params.radius_ = nan;
    params.tubeRadius_ = Vector2(inf, -1.0f);
    params.segments_ = 1;
    params.tubeSegments_ = 1000;
    params.arc_ = nan;
    params.tubeAngle_ = inf;
    CsgSanitizeRingParams(params);
    CHECK(params.radius_ == 0.5f);
    // The infinite radial radius gets the default 0.15. The negative axial radius gets the minimum.
    CHECK(params.tubeRadius_.x_ == 0.15f);
    CHECK(params.tubeRadius_.y_ == CsgRingParams::MinTubeRadius);
    CHECK(params.segments_ == 3);
    CHECK(params.tubeSegments_ == 64);
    CHECK(params.arc_ == 360.0f);
    CHECK(params.tubeAngle_ == 0.0f);

    params.radius_ = 1e9f;
    params.tubeRadius_ = Vector2(1e9f, 1e9f);
    params.segments_ = 100000;
    params.tubeSegments_ = -3;
    params.arc_ = 1000.0f;
    params.tubeAngle_ = -1000.0f;
    CsgSanitizeRingParams(params);
    CHECK(params.radius_ == CsgRingParams::MaxRadius);
    CHECK(params.tubeRadius_.x_ == Catch::Approx(CsgRingParams::MaxRadius * 0.99f));
    CHECK(params.tubeRadius_.y_ == CsgRingParams::MaxTubeRadius);
    CHECK(params.segments_ == 128);
    CHECK(params.tubeSegments_ == 3);
    CHECK(params.arc_ == 360.0f);
    CHECK(params.tubeAngle_ == -180.0f);
    params.arc_ = -5.0f;
    params.tubeAngle_ = 1000.0f;
    CsgSanitizeRingParams(params);
    CHECK(params.arc_ == 1.0f);
    CHECK(params.tubeAngle_ == 180.0f);

    // The radial tube radius stays below the major radius, so the inner edge has a positive radius.
    CsgRingParams thick;
    thick.radius_ = 0.2f;
    thick.tubeRadius_ = Vector2(0.5f, 0.1f);
    CsgSanitizeRingParams(thick);
    CHECK(thick.tubeRadius_.x_ == Catch::Approx(0.2f * 0.99f));
    CHECK(thick.tubeRadius_.x_ < thick.radius_);
    CsgShapeMesh mesh;
    REQUIRE(CsgGenerateRing(thick, mesh));
    for (const ModelVertex& vertex : mesh.vertices_)
        CHECK(std::hypot(vertex.GetPosition().x_, vertex.GetPosition().z_) > 0.0f);
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);

    // The generator uses the sanitized values and does not crash.
    CsgRingParams raw;
    raw.radius_ = nan;
    raw.tubeRadius_ = Vector2(nan, inf);
    raw.segments_ = 0;
    raw.tubeSegments_ = 0;
    raw.arc_ = inf;
    raw.tubeAngle_ = -inf;
    REQUIRE(CsgGenerateRing(raw, mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);
    CHECK(HasNoDegenerateTriangle(mesh));

    // The smallest valid values.
    CsgRingParams small;
    small.radius_ = 0.0f;
    small.tubeRadius_ = Vector2::ZERO;
    small.segments_ = 128;
    small.tubeSegments_ = 64;
    small.arc_ = 1.0f;
    REQUIRE(CsgGenerateRing(small, mesh));
    CHECK(HasNoDegenerateTriangle(mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);
}

TEST_CASE("CsgGenerateRing keeps the shape valid for the extreme values", "[shapes][procedural]")
{
    // The float positions of a thin tube at the largest radius do not merge in a triangle:
    // the Y coordinate stays precise and the section stays distinct in Y.
    CsgShapeMesh mesh;
    CsgRingParams thin;
    thin.radius_ = CsgRingParams::MaxRadius;
    thin.tubeRadius_ = Vector2(CsgRingParams::MinTubeRadius, CsgRingParams::MinTubeRadius);
    thin.segments_ = 128;
    thin.tubeSegments_ = 64;
    REQUIRE(CsgGenerateRing(thin, mesh));
    CHECK(HasNoDegenerateTriangle(mesh));
    CHECK(SignedVolume(MeshToPolygons(mesh)) > 0.0);

    // The generator clears the old mesh data.
    mesh.indices_.push_back(1);
    CsgRingParams params;
    params.arc_ = 90.0f;
    REQUIRE(CsgGenerateRing(params, mesh));
    CHECK(mesh.indices_.size() == 3 * (2 * 32 * 12 + 2 * 12));
}
