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

} // namespace Urho3D
