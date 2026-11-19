// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgShapes.h"

#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Math/Color.h>
#include <Urho3D/Math/MathDefs.h>

#include <cmath>

namespace Urho3D
{

namespace
{

/// Return the value clamped to the range, or the default value if the value is not finite.
float SanitizeValue(float value, float minValue, float maxValue, float defaultValue)
{
    if (!std::isfinite(value))
        return defaultValue;
    return Clamp(value, minValue, maxValue);
}

Vector3 SanitizeVector(const Vector3& value, float minValue, float maxValue, const Vector3& defaultValue)
{
    return Vector3(SanitizeValue(value.x_, minValue, maxValue, defaultValue.x_),
        SanitizeValue(value.y_, minValue, maxValue, defaultValue.y_),
        SanitizeValue(value.z_, minValue, maxValue, defaultValue.z_));
}

/// The data of one nominal box face.
struct BoxFace
{
    /// The face corners. Corner index bits: bit 0 is +X, bit 1 is +Y, bit 2 is +Z.
    /// The order follows MakeBoxPolygons of the tests: (p1 - p0) x (p2 - p0) points out of the box.
    unsigned corners_[4];
    /// The nominal normal and the UV axes. u x v = normal.
    Vector3 normal_;
    Vector3 u_;
    Vector3 v_;
};

/// The faces in the order of the face groups: -X, +X, -Y, +Y, -Z, +Z.
const BoxFace BoxFaces[6] = {
    {{0, 4, 6, 2}, Vector3::LEFT, Vector3::FORWARD, Vector3::UP},
    {{1, 3, 7, 5}, Vector3::RIGHT, Vector3::UP, Vector3::FORWARD},
    {{0, 1, 5, 4}, Vector3::DOWN, Vector3::RIGHT, Vector3::FORWARD},
    {{2, 6, 7, 3}, Vector3::UP, Vector3::FORWARD, Vector3::RIGHT},
    {{0, 2, 3, 1}, Vector3::BACK, Vector3::UP, Vector3::RIGHT},
    {{4, 5, 7, 6}, Vector3::FORWARD, Vector3::RIGHT, Vector3::UP},
};

/// The two triangles of a face use the fixed diagonal from face corner 0 to face corner 2.
const unsigned BoxFaceTriangles[2][3] = {{0, 1, 2}, {0, 2, 3}};

double TripleProduct(const Vector3& a, const Vector3& b, const Vector3& c)
{
    const double ax = a.x_, ay = a.y_, az = a.z_;
    const double bx = b.x_, by = b.y_, bz = b.z_;
    const double cx = c.x_, cy = c.y_, cz = c.z_;
    return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
}

/// Return true if the three edges at each corner give a positive triple product.
/// The edge along an axis goes to the neighbor corner. A corner on the + side of an axis has the edge to the - side.
/// So the sign of the corner octant corrects the sign of the product.
bool AreBoxCornersValid(const ea::array<Vector3, 8>& positions)
{
    // The relative limit refuses a corner with three nearly coplanar edges.
    constexpr double minRelativeProduct = 1e-6;
    for (unsigned corner = 0; corner < 8; ++corner)
    {
        const Vector3& p = positions[corner];
        const Vector3 ex = positions[corner ^ 1u] - p;
        const Vector3 ey = positions[corner ^ 2u] - p;
        const Vector3 ez = positions[corner ^ 4u] - p;
        const unsigned bits = (corner & 1u) + ((corner >> 1u) & 1u) + ((corner >> 2u) & 1u);
        const double sign = bits % 2 == 0 ? 1.0 : -1.0;
        const double scale = static_cast<double>(ex.Length()) * ey.Length() * ez.Length();
        if (!(sign * TripleProduct(ex, ey, ez) > minRelativeProduct * scale))
            return false;
    }
    return true;
}

} // namespace

const ea::vector<VertexElement>& CsgGetShapeVertexElements()
{
    static const ea::vector<VertexElement> elements = []
    {
        ea::vector<VertexElement> result{
            VertexElement{TYPE_VECTOR3, SEM_POSITION, 0},
            VertexElement{TYPE_VECTOR3, SEM_NORMAL, 0},
            VertexElement{TYPE_VECTOR4, SEM_TANGENT, 0},
            VertexElement{TYPE_VECTOR2, SEM_TEXCOORD, 0},
            VertexElement{TYPE_VECTOR2, SEM_TEXCOORD, 1},
            VertexElement{TYPE_UBYTE4_NORM, SEM_COLOR, 0},
        };
        VertexBuffer::UpdateOffsets(result);
        return result;
    }();
    return elements;
}

void CsgSanitizeBoxParams(CsgBoxParams& params)
{
    params.size_ = SanitizeVector(params.size_, CsgBoxParams::MinSize, CsgBoxParams::MaxSize, Vector3::ONE);
    for (unsigned i = 0; i < 8; ++i)
    {
        params.corners_[i] = SanitizeVector(
            params.corners_[i], -CsgBoxParams::MaxCorner, CsgBoxParams::MaxCorner, CsgBoxParams::GetDefaultCorner(i));
    }
}

bool CsgGenerateBox(const CsgBoxParams& params, CsgShapeMesh& mesh)
{
    mesh.vertices_.clear();
    mesh.indices_.clear();
    mesh.groups_.clear();

    CsgBoxParams sanitized = params;
    CsgSanitizeBoxParams(sanitized);

    // Each vertex copies its position from this table, so the shared corners are bit-identical.
    ea::array<Vector3, 8> positions;
    for (unsigned i = 0; i < 8; ++i)
        positions[i] = sanitized.size_ * sanitized.corners_[i];

    if (!AreBoxCornersValid(positions))
        return false;

    // UV 1 cell of a face: 3 columns and 2 rows, with a small margin.
    constexpr float cellMargin = 1.0f / 64.0f;
    const Vector2 cellSize(1.0f / 3.0f, 1.0f / 2.0f);
    // The nominal UV 1 coordinates of the face corners in the cell.
    const Vector2 cellCorners[4] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
    const Vector4 white = Color::WHITE.ToVector4();

    double volume = 0.0;
    mesh.vertices_.reserve(36);
    mesh.indices_.reserve(36);
    for (unsigned faceIndex = 0; faceIndex < 6; ++faceIndex)
    {
        const BoxFace& face = BoxFaces[faceIndex];
        const Vector2 cellOrigin(cellSize.x_ * (faceIndex % 3), cellSize.y_ * (faceIndex / 3));
        mesh.groups_.push_back(CsgShapeGroup{static_cast<unsigned>(mesh.indices_.size()), 6u});

        for (const auto& triangle : BoxFaceTriangles)
        {
            const Vector3& p0 = positions[face.corners_[triangle[0]]];
            const Vector3& p1 = positions[face.corners_[triangle[1]]];
            const Vector3& p2 = positions[face.corners_[triangle[2]]];
            volume += TripleProduct(p0, p1, p2) / 6.0;

            const Vector3 normal = (p1 - p0).CrossProduct(p2 - p0).NormalizedOrDefault(face.normal_);
            // The tangent is the U axis in the triangle plane. The shader bitangent is cross(tangent, normal) * w.
            // V of UV 0 goes along -v, so w makes the bitangent point along -v.
            const Vector3 tangent =
                (face.u_ - normal * normal.DotProduct(face.u_)).NormalizedOrDefault(face.u_);
            const float handedness = tangent.CrossProduct(normal).DotProduct(-face.v_) >= 0.0f ? 1.0f : -1.0f;

            for (const unsigned faceCorner : triangle)
            {
                const Vector3& position = positions[face.corners_[faceCorner]];
                const Vector2 cell = cellCorners[faceCorner];

                ModelVertex vertex{};
                vertex.SetPosition(position);
                vertex.SetNormal(normal);
                vertex.tangent_ = Vector4(tangent, handedness);
                vertex.uv_[0] = Vector4(position.DotProduct(face.u_), -position.DotProduct(face.v_), 0.0f, 0.0f);
                const Vector2 lightmap = cellOrigin + cellSize * (cell * (1.0f - 2.0f * cellMargin) + Vector2(cellMargin, cellMargin));
                vertex.uv_[1] = Vector4(lightmap.x_, lightmap.y_, 0.0f, 0.0f);
                vertex.color_[0] = white;

                mesh.indices_.push_back(mesh.vertices_.size());
                mesh.vertices_.push_back(vertex);
            }
        }
    }

    if (!(volume > 0.0))
    {
        mesh.vertices_.clear();
        mesh.indices_.clear();
        mesh.groups_.clear();
        return false;
    }
    return true;
}

void CsgSanitizeSphereParams(CsgSphereParams& params)
{
    params.radius_ = SanitizeVector(params.radius_, CsgSphereParams::MinRadius, CsgSphereParams::MaxRadius,
        Vector3(0.5f, 0.5f, 0.5f));
    params.segments_ = Clamp(params.segments_, CsgSphereParams::MinSegments, CsgSphereParams::MaxSegments);
    params.rings_ = Clamp(params.rings_, CsgSphereParams::MinRings, CsgSphereParams::MaxRings);
}

namespace
{

/// One corner of a sphere triangle.
struct SphereCorner
{
    /// The position from the table.
    Vector3 position_;
    /// The latitude row: 0 is the top pole, `rings` is the bottom pole.
    int row_;
    /// The longitude index. The seam has `segments` here. A pole corner uses the middle of the fan.
    double lon_;
};

} // namespace

bool CsgGenerateSphere(const CsgSphereParams& params, CsgShapeMesh& mesh)
{
    mesh.vertices_.clear();
    mesh.indices_.clear();
    mesh.groups_.clear();

    CsgSphereParams sanitized = params;
    CsgSanitizeSphereParams(sanitized);
    const unsigned segments = static_cast<unsigned>(sanitized.segments_);
    const unsigned rings = static_cast<unsigned>(sanitized.rings_);
    const Vector3 radius = sanitized.radius_;
    const double twoPi = 2.0 * M_PI;

    // The trigonometric tables. The poles and the equator have exact values.
    ea::vector<double> rowCos(rings + 1);
    ea::vector<double> rowSin(rings + 1);
    for (unsigned row = 0; row <= rings; ++row)
    {
        const double theta = M_PI * row / rings;
        rowCos[row] = row == 0 ? 1.0 : row == rings ? -1.0 : 2 * row == rings ? 0.0 : std::cos(theta);
        rowSin[row] = row == 0 || row == rings ? 0.0 : std::sin(theta);
    }
    ea::vector<double> lonCos(segments + 1);
    ea::vector<double> lonSin(segments + 1);
    for (unsigned lon = 0; lon <= segments; ++lon)
    {
        lonCos[lon] = std::cos(twoPi * lon / segments);
        lonSin[lon] = std::sin(twoPi * lon / segments);
    }

    // Each vertex copies its position from this table, so the shared positions are bit-identical.
    // Index 0 is the top pole. Then come the rings. The last index is the bottom pole.
    const unsigned topPole = 0;
    const unsigned bottomPole = 1 + (rings - 1) * segments;
    ea::vector<Vector3> positions(bottomPole + 1);
    positions[topPole] = Vector3(0.0f, radius.y_, 0.0f);
    positions[bottomPole] = Vector3(0.0f, -radius.y_, 0.0f);
    for (unsigned row = 1; row < rings; ++row)
    {
        for (unsigned lon = 0; lon < segments; ++lon)
        {
            positions[1 + (row - 1) * segments + lon] =
                Vector3(static_cast<float>(radius.x_ * rowSin[row] * lonCos[lon]),
                    static_cast<float>(radius.y_ * rowCos[row]),
                    static_cast<float>(radius.z_ * rowSin[row] * lonSin[lon]));
        }
    }
    const auto ringCorner = [&](unsigned row, unsigned lon) -> SphereCorner
    {
        // The seam (lon == segments) copies the position of lon 0.
        return {positions[1 + (row - 1) * segments + lon % segments], static_cast<int>(row), static_cast<double>(lon)};
    };

    const Vector4 white = Color::WHITE.ToVector4();
    const double rx2 = static_cast<double>(radius.x_) * radius.x_;
    const double ry2 = static_cast<double>(radius.y_) * radius.y_;
    const double rz2 = static_cast<double>(radius.z_) * radius.z_;
    double volume = 0.0;

    const auto emitTriangle = [&](const SphereCorner& c0, const SphereCorner& c1, const SphereCorner& c2)
    {
        const SphereCorner* corners[3] = {&c0, &c1, &c2};
        const Vector3 flatNormal =
            (c1.position_ - c0.position_).CrossProduct(c2.position_ - c0.position_).NormalizedOrDefault(Vector3::UP);
        volume += TripleProduct(c0.position_, c1.position_, c2.position_) / 6.0;

        for (const SphereCorner* corner : corners)
        {
            const Vector3& p = corner->position_;
            const unsigned row = static_cast<unsigned>(corner->row_);
            const double cosTheta = rowCos[row];
            const double sinTheta = rowSin[row];
            double cosPhi;
            double sinPhi;
            const unsigned lonIndex = static_cast<unsigned>(corner->lon_);
            if (static_cast<double>(lonIndex) == corner->lon_)
            {
                cosPhi = lonCos[lonIndex];
                sinPhi = lonSin[lonIndex];
            }
            else
            {
                // A pole corner uses the middle of its fan.
                const double phi = twoPi * corner->lon_ / segments;
                cosPhi = std::cos(phi);
                sinPhi = std::sin(phi);
            }

            Vector3 normal = flatNormal;
            if (sanitized.smoothNormals_)
            {
                normal = Vector3(static_cast<float>(p.x_ / rx2), static_cast<float>(p.y_ / ry2),
                    static_cast<float>(p.z_ / rz2)).NormalizedOrDefault(flatNormal);
            }

            // The tangent follows the longitude direction, in the tangent plane.
            const Vector3 longitude(static_cast<float>(-radius.x_ * sinPhi), 0.0f, static_cast<float>(radius.z_ * cosPhi));
            const Vector3 longitudeDir = longitude.NormalizedOrDefault(Vector3::FORWARD);
            const Vector3 tangent =
                (longitudeDir - normal * normal.DotProduct(longitudeDir)).NormalizedOrDefault(longitudeDir);
            // V grows toward the bottom pole. The shader bitangent is cross(tangent, normal) * w.
            const Vector3 down(static_cast<float>(radius.x_ * cosTheta * cosPhi), static_cast<float>(-radius.y_ * sinTheta),
                static_cast<float>(radius.z_ * cosTheta * sinPhi));
            const float handedness = tangent.CrossProduct(normal).DotProduct(down) >= 0.0f ? 1.0f : -1.0f;

            const float u = static_cast<float>(corner->lon_ / segments);
            const float v = static_cast<float>(static_cast<double>(corner->row_) / rings);

            ModelVertex vertex{};
            vertex.SetPosition(p);
            vertex.SetNormal(normal);
            vertex.tangent_ = Vector4(tangent, handedness);
            vertex.uv_[0] = Vector4(u, v, 0.0f, 0.0f);
            vertex.uv_[1] = Vector4(u, v, 0.0f, 0.0f);
            vertex.color_[0] = white;

            mesh.indices_.push_back(mesh.vertices_.size());
            mesh.vertices_.push_back(vertex);
        }
    };

    mesh.vertices_.reserve(3 * 2 * segments * (rings - 1));
    mesh.indices_.reserve(3 * 2 * segments * (rings - 1));
    for (unsigned lon = 0; lon < segments; ++lon)
    {
        // The pole fans. The pole corner uses the middle longitude of its triangle for UV and tangent.
        const SphereCorner top{positions[topPole], 0, lon + 0.5};
        emitTriangle(top, ringCorner(1, lon + 1), ringCorner(1, lon));
    }
    for (unsigned row = 1; row + 1 < rings; ++row)
    {
        for (unsigned lon = 0; lon < segments; ++lon)
        {
            const SphereCorner a = ringCorner(row, lon);
            const SphereCorner b = ringCorner(row, lon + 1);
            const SphereCorner c = ringCorner(row + 1, lon + 1);
            const SphereCorner d = ringCorner(row + 1, lon);
            emitTriangle(a, b, d);
            emitTriangle(b, c, d);
        }
    }
    for (unsigned lon = 0; lon < segments; ++lon)
    {
        const SphereCorner bottom{positions[bottomPole], static_cast<int>(rings), lon + 0.5};
        emitTriangle(ringCorner(rings - 1, lon), ringCorner(rings - 1, lon + 1), bottom);
    }

    mesh.groups_.push_back(CsgShapeGroup{0u, static_cast<unsigned>(mesh.indices_.size())});

    if (!(volume > 0.0))
    {
        mesh.vertices_.clear();
        mesh.indices_.clear();
        mesh.groups_.clear();
        return false;
    }
    return true;
}

} // namespace Urho3D
