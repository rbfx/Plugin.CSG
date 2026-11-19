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

namespace
{

/// A minimal double precision vector for the cylinder generator.
struct Vector2d
{
    Vector2d(double x, double y) : x_(x), y_(y) {}
    double x_;
    double y_;
};

struct Vector3d
{
    Vector3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}
    double x_;
    double y_;
    double z_;

    double DotProduct(const Vector3d& rhs) const { return x_ * rhs.x_ + y_ * rhs.y_ + z_ * rhs.z_; }
    Vector3d CrossProduct(const Vector3d& rhs) const
    {
        return Vector3d(y_ * rhs.z_ - z_ * rhs.y_, z_ * rhs.x_ - x_ * rhs.z_, x_ * rhs.y_ - y_ * rhs.x_);
    }
    Vector3d operator*(double scale) const { return Vector3d(x_ * scale, y_ * scale, z_ * scale); }
};

/// Return the sanitized radius pair of a cylinder end.
Vector2 SanitizeEndRadius(const Vector2& value, const Vector2& defaultValue)
{
    Vector2 radius(SanitizeValue(value.x_, 0.0f, CsgCylinderParams::MaxRadius, defaultValue.x_),
        SanitizeValue(value.y_, 0.0f, CsgCylinderParams::MaxRadius, defaultValue.y_));
    // An apex needs both components below the minimum. A single small component is raised.
    if (radius.x_ < CsgCylinderParams::MinRadius && radius.y_ < CsgCylinderParams::MinRadius)
        return Vector2::ZERO;
    radius.x_ = ea::max(radius.x_, CsgCylinderParams::MinRadius);
    radius.y_ = ea::max(radius.y_, CsgCylinderParams::MinRadius);
    return radius;
}

} // namespace

void CsgSanitizeCylinderParams(CsgCylinderParams& params)
{
    const CsgCylinderParams defaults;
    params.bottomCenter_ = SanitizeVector(params.bottomCenter_, -CsgCylinderParams::MaxCoordinate,
        CsgCylinderParams::MaxCoordinate, defaults.bottomCenter_);
    params.topCenter_ = SanitizeVector(
        params.topCenter_, -CsgCylinderParams::MaxCoordinate, CsgCylinderParams::MaxCoordinate, defaults.topCenter_);
    params.bottomRadius_ = SanitizeEndRadius(params.bottomRadius_, defaults.bottomRadius_);
    params.topRadius_ = SanitizeEndRadius(params.topRadius_, defaults.topRadius_);
    params.segments_ = Clamp(params.segments_, CsgCylinderParams::MinSegments, CsgCylinderParams::MaxSegments);
    params.heightSegments_ =
        Clamp(params.heightSegments_, CsgCylinderParams::MinHeightSegments, CsgCylinderParams::MaxHeightSegments);
    params.bulge_ = SanitizeValue(params.bulge_, CsgCylinderParams::MinBulge, CsgCylinderParams::MaxBulge, 0.0f);
}

namespace
{

/// One corner of a cylinder triangle with all attributes except the final normal.
struct CylinderCorner
{
    Vector3 position_;
    Vector2 uv_;
    /// The direction of the U axis in space.
    Vector3 tangentDir_;
    /// The direction of the V axis in space.
    Vector3 vDir_;
    /// The smooth normal. A cap corner always uses it.
    Vector3 normal_;
    bool isCap_{};
};

} // namespace

bool CsgGenerateCylinder(const CsgCylinderParams& params, CsgShapeMesh& mesh)
{
    mesh.vertices_.clear();
    mesh.indices_.clear();
    mesh.groups_.clear();

    CsgCylinderParams sanitized = params;
    CsgSanitizeCylinderParams(sanitized);
    const Vector3 bottom = sanitized.bottomCenter_;
    const Vector3 top = sanitized.topCenter_;
    const bool bottomApex = sanitized.bottomRadius_ == Vector2::ZERO;
    const bool topApex = sanitized.topRadius_ == Vector2::ZERO;
    // The end planes are perpendicular to Y, so the Y distance must be large enough, not only the 3D distance.
    if ((top - bottom).Length() < CsgCylinderParams::MinCenterDistance
        || std::fabs(top.y_ - bottom.y_) < CsgCylinderParams::MinCenterDistance || (bottomApex && topApex))
        return false;

    const unsigned segments = static_cast<unsigned>(sanitized.segments_);
    const unsigned heightSegments = static_cast<unsigned>(sanitized.heightSegments_);
    const double twoPi = 2.0 * M_PI;
    const double bulge = sanitized.bulge_;
    // Rings are in planes of constant Y. A top below the bottom flips the winding.
    const double side = top.y_ >= bottom.y_ ? 1.0 : -1.0;
    const bool flip = side < 0.0;

    const auto lerpRadius = [&](double t, double a, double b) { return (a + (b - a) * t) * (1.0 + bulge * 4.0 * t * (1.0 - t)); };
    // The radii of the ring at t. The profile is a polynomial, so it is valid outside [0, 1].
    const auto radiusAt = [&](double t)
    {
        return Vector2d(lerpRadius(t, sanitized.bottomRadius_.x_, sanitized.topRadius_.x_),
            lerpRadius(t, sanitized.bottomRadius_.y_, sanitized.topRadius_.y_));
    };
    const auto radiusSlopeAt = [&](double t)
    {
        constexpr double h = 1e-3;
        const Vector2d r1 = radiusAt(t + h);
        const Vector2d r0 = radiusAt(t - h);
        return Vector2d((r1.x_ - r0.x_) / (2.0 * h), (r1.y_ - r0.y_) / (2.0 * h));
    };
    const Vector3d axis(static_cast<double>(top.x_) - bottom.x_, static_cast<double>(top.y_) - bottom.y_,
        static_cast<double>(top.z_) - bottom.z_);

    ea::vector<double> lonCos(segments + 1);
    ea::vector<double> lonSin(segments + 1);
    for (unsigned lon = 0; lon <= segments; ++lon)
    {
        lonCos[lon] = std::cos(twoPi * lon / segments);
        lonSin[lon] = std::sin(twoPi * lon / segments);
    }

    // Each vertex copies its position from this table, so the shared positions are bit-identical.
    // The rows come first. A row at an apex end has one position. The cap centers come last.
    const unsigned numRows = heightSegments + 1;
    ea::vector<unsigned> rowStart(numRows + 1);
    for (unsigned row = 0; row < numRows; ++row)
    {
        const bool apexRow = (row == 0 && bottomApex) || (row == heightSegments && topApex);
        rowStart[row + 1] = rowStart[row] + (apexRow ? 1u : segments);
    }
    const unsigned bottomCapCenter = rowStart[numRows];
    const unsigned topCapCenter = bottomCapCenter + 1;
    ea::vector<Vector3> positions(topCapCenter + 1);
    positions[bottomCapCenter] = bottom;
    positions[topCapCenter] = top;
    for (unsigned row = 0; row < numRows; ++row)
    {
        const double t = static_cast<double>(row) / heightSegments;
        if (rowStart[row + 1] - rowStart[row] == 1)
        {
            positions[rowStart[row]] = row == 0 ? bottom : top;
            continue;
        }
        const Vector2d radius = radiusAt(t);
        for (unsigned lon = 0; lon < segments; ++lon)
        {
            positions[rowStart[row] + lon] = Vector3(
                static_cast<float>(bottom.x_ * (1.0 - t) + top.x_ * t + radius.x_ * lonCos[lon]),
                static_cast<float>(bottom.y_ * (1.0 - t) + top.y_ * t),
                static_cast<float>(bottom.z_ * (1.0 - t) + top.z_ * t + radius.y_ * lonSin[lon]));
        }
    }
    // The seam (lon == segments) copies the position of lon 0.
    const auto ringPosition = [&](unsigned row, unsigned lon) -> const Vector3&
    { return positions[rowStart[row] + (rowStart[row + 1] - rowStart[row] == 1 ? 0 : lon % segments)]; };

    // The attributes of the body at (t, lon). The lon is a fractional index for an apex corner.
    struct BodyFrame
    {
        Vector3 normal_;
        Vector3 tangentDir_;
        Vector3 vDir_;
    };
    const auto bodyFrame = [&](double t, double lon) -> BodyFrame
    {
        const double phi = twoPi * lon / segments;
        const double c = std::cos(phi);
        const double s = std::sin(phi);
        const Vector2d radius = radiusAt(t);
        const Vector2d slope = radiusSlopeAt(t);
        const Vector3d dPdt(axis.x_ + slope.x_ * c, axis.y_, axis.z_ + slope.y_ * s);
        const Vector3d dPda(-radius.x_ * s, 0.0, radius.y_ * c);
        const Vector3d n = dPdt.CrossProduct(dPda) * side;
        BodyFrame frame;
        frame.normal_ = Vector3(static_cast<float>(n.x_), static_cast<float>(n.y_), static_cast<float>(n.z_));
        frame.tangentDir_ = Vector3(static_cast<float>(-radius.x_ * s), 0.0f, static_cast<float>(radius.y_ * c));
        frame.vDir_ = Vector3(static_cast<float>(dPdt.x_), static_cast<float>(dPdt.y_), static_cast<float>(dPdt.z_));
        return frame;
    };

    // A corner of the body. An apex corner uses the middle of its fan and the row next to the apex.
    const auto bodyCorner = [&](unsigned row, unsigned lonIndex) -> CylinderCorner
    {
        CylinderCorner corner;
        corner.position_ = ringPosition(row, lonIndex);
        corner.uv_ = Vector2(static_cast<float>(static_cast<double>(lonIndex) / segments),
            static_cast<float>(static_cast<double>(row) / heightSegments));
        const double t = static_cast<double>(row) / heightSegments;
        const BodyFrame frame = bodyFrame(t, lonIndex);
        corner.normal_ = frame.normal_.NormalizedOrDefault(Vector3::UP);
        corner.tangentDir_ = frame.tangentDir_.NormalizedOrDefault(Vector3::FORWARD);
        corner.vDir_ = frame.vDir_;
        return corner;
    };
    const auto apexCorner = [&](bool atBottom, unsigned fan) -> CylinderCorner
    {
        CylinderCorner corner;
        const unsigned row = atBottom ? 0 : heightSegments;
        const unsigned nextRow = atBottom ? 1 : heightSegments - 1;
        corner.position_ = ringPosition(row, 0);
        const double mid = fan + 0.5;
        corner.uv_ = Vector2(static_cast<float>(mid / segments), static_cast<float>(static_cast<double>(row) / heightSegments));
        const double t = static_cast<double>(nextRow) / heightSegments;
        // The apex normal is the average of the normals of the neighbors in the next row.
        const Vector3 n0 = bodyFrame(t, fan).normal_.NormalizedOrDefault(Vector3::UP);
        const Vector3 n1 = bodyFrame(t, fan + 1).normal_.NormalizedOrDefault(Vector3::UP);
        const BodyFrame frame = bodyFrame(t, mid);
        corner.normal_ = (n0 + n1).NormalizedOrDefault(frame.normal_.NormalizedOrDefault(Vector3::UP));
        corner.tangentDir_ = frame.tangentDir_.NormalizedOrDefault(Vector3::FORWARD);
        corner.vDir_ = frame.vDir_;
        return corner;
    };
    const auto capCorner = [&](const Vector3& position, const Vector3& normal) -> CylinderCorner
    {
        CylinderCorner corner;
        corner.position_ = position;
        corner.uv_ = Vector2(position.x_ + 0.5f, position.z_ + 0.5f);
        corner.tangentDir_ = Vector3::RIGHT;
        corner.vDir_ = Vector3::BACK;
        corner.normal_ = normal;
        corner.isCap_ = true;
        return corner;
    };

    const Vector4 white = Color::WHITE.ToVector4();
    double volume = 0.0;
    bool degenerate = false;
    const Vector3 origin = (bottom + top) * 0.5f;

    const auto emitTriangle = [&](const CylinderCorner& c0, const CylinderCorner& c1, const CylinderCorner& c2)
    {
        const CylinderCorner* corners[3] = {&c0, flip ? &c2 : &c1, flip ? &c1 : &c2};
        const Vector3& p0 = corners[0]->position_;
        const Vector3& p1 = corners[1]->position_;
        const Vector3& p2 = corners[2]->position_;
        if (p0 == p1 || p1 == p2 || p2 == p0)
            degenerate = true;
        const Vector3 flatNormal = (p1 - p0).CrossProduct(p2 - p0).NormalizedOrDefault(Vector3::UP);
        const auto relative = [&](const Vector3& p)
        { return Vector3d(static_cast<double>(p.x_) - origin.x_, static_cast<double>(p.y_) - origin.y_, static_cast<double>(p.z_) - origin.z_); };
        const Vector3d r0 = relative(p0);
        const Vector3d r1 = relative(p1);
        const Vector3d r2 = relative(p2);
        volume += r0.DotProduct(r1.CrossProduct(r2)) / 6.0;

        for (const CylinderCorner* corner : corners)
        {
            const Vector3 normal = corner->isCap_ || sanitized.smoothNormals_ ? corner->normal_ : flatNormal;
            // The shader bitangent is cross(tangent, normal) * w. It must point along the V axis.
            const Vector3 tangent = (corner->tangentDir_ - normal * normal.DotProduct(corner->tangentDir_))
                                        .NormalizedOrDefault(corner->tangentDir_);
            const float handedness = tangent.CrossProduct(normal).DotProduct(corner->vDir_) >= 0.0f ? 1.0f : -1.0f;

            ModelVertex vertex{};
            vertex.SetPosition(corner->position_);
            vertex.SetNormal(normal);
            vertex.tangent_ = Vector4(tangent, handedness);
            vertex.uv_[0] = Vector4(corner->uv_.x_, corner->uv_.y_, 0.0f, 0.0f);
            vertex.uv_[1] = vertex.uv_[0];
            vertex.color_[0] = white;

            mesh.indices_.push_back(mesh.vertices_.size());
            mesh.vertices_.push_back(vertex);
        }
    };

    const unsigned numTriangles = 2 * segments * heightSegments + 2 * segments;
    mesh.vertices_.reserve(3 * numTriangles);
    mesh.indices_.reserve(3 * numTriangles);

    // Group 0: the side. The rows go from the bottom to the top.
    for (unsigned row = 0; row < heightSegments; ++row)
    {
        const bool fromApex = row == 0 && bottomApex;
        const bool toApex = row + 1 == heightSegments && topApex;
        for (unsigned lon = 0; lon < segments; ++lon)
        {
            if (fromApex)
            {
                emitTriangle(apexCorner(true, lon), bodyCorner(row + 1, lon), bodyCorner(row + 1, lon + 1));
                continue;
            }
            if (toApex)
            {
                emitTriangle(bodyCorner(row, lon), apexCorner(false, lon), bodyCorner(row, lon + 1));
                continue;
            }
            const CylinderCorner a = bodyCorner(row, lon);
            const CylinderCorner b = bodyCorner(row, lon + 1);
            const CylinderCorner c = bodyCorner(row + 1, lon + 1);
            const CylinderCorner d = bodyCorner(row + 1, lon);
            emitTriangle(a, c, b);
            emitTriangle(a, d, c);
        }
    }
    mesh.groups_.push_back(CsgShapeGroup{0u, static_cast<unsigned>(mesh.indices_.size()), 0u});

    // Group 1: the top cap. Its normal points away from the body.
    if (!topApex)
    {
        const unsigned start = mesh.indices_.size();
        const CylinderCorner center = capCorner(top, Vector3(0.0f, static_cast<float>(side), 0.0f));
        for (unsigned lon = 0; lon < segments; ++lon)
        {
            emitTriangle(center, capCorner(ringPosition(heightSegments, lon + 1), center.normal_),
                capCorner(ringPosition(heightSegments, lon), center.normal_));
        }
        mesh.groups_.push_back(CsgShapeGroup{start, static_cast<unsigned>(mesh.indices_.size()) - start, 1u});
    }
    // Group 2: the bottom cap.
    if (!bottomApex)
    {
        const unsigned start = mesh.indices_.size();
        const CylinderCorner center = capCorner(bottom, Vector3(0.0f, static_cast<float>(-side), 0.0f));
        for (unsigned lon = 0; lon < segments; ++lon)
        {
            emitTriangle(center, capCorner(ringPosition(0, lon), center.normal_),
                capCorner(ringPosition(0, lon + 1), center.normal_));
        }
        mesh.groups_.push_back(CsgShapeGroup{start, static_cast<unsigned>(mesh.indices_.size()) - start, 2u});
    }

    if (degenerate || !(volume > 0.0))
    {
        mesh.vertices_.clear();
        mesh.indices_.clear();
        mesh.groups_.clear();
        return false;
    }
    return true;
}

void CsgSanitizeRingParams(CsgRingParams& params)
{
    const CsgRingParams defaults;
    params.radius_ = SanitizeValue(params.radius_, CsgRingParams::MinRadius, CsgRingParams::MaxRadius, defaults.radius_);
    params.tubeRadius_ = Vector2(SanitizeValue(params.tubeRadius_.x_, CsgRingParams::MinTubeRadius,
                                     CsgRingParams::MaxTubeRadius, defaults.tubeRadius_.x_),
        SanitizeValue(params.tubeRadius_.y_, CsgRingParams::MinTubeRadius, CsgRingParams::MaxTubeRadius,
            defaults.tubeRadius_.y_));
    // The inner edge must stay at a positive radius.
    params.tubeRadius_.x_ = ea::min(params.tubeRadius_.x_, params.radius_ * CsgRingParams::MaxInnerFraction);
    params.segments_ = Clamp(params.segments_, CsgRingParams::MinSegments, CsgRingParams::MaxSegments);
    params.tubeSegments_ = Clamp(params.tubeSegments_, CsgRingParams::MinTubeSegments, CsgRingParams::MaxTubeSegments);
    params.arc_ = SanitizeValue(params.arc_, CsgRingParams::MinArc, CsgRingParams::MaxArc, defaults.arc_);
    params.tubeAngle_ = SanitizeValue(
        params.tubeAngle_, CsgRingParams::MinTubeAngle, CsgRingParams::MaxTubeAngle, defaults.tubeAngle_);
}

namespace
{

/// One corner of a ring triangle with all attributes except the final normal.
struct RingCorner
{
    Vector3 position_;
    Vector2 uv_;
    /// The direction of the U axis in space.
    Vector3 tangentDir_;
    /// The direction of the V axis in space.
    Vector3 vDir_;
    /// The smooth normal. A cap corner always uses it.
    Vector3 normal_;
    bool isCap_{};
};

} // namespace

bool CsgGenerateRing(const CsgRingParams& params, CsgShapeMesh& mesh)
{
    mesh.vertices_.clear();
    mesh.indices_.clear();
    mesh.groups_.clear();

    CsgRingParams sanitized = params;
    CsgSanitizeRingParams(sanitized);
    const unsigned segments = static_cast<unsigned>(sanitized.segments_);
    const unsigned tubeSegments = static_cast<unsigned>(sanitized.tubeSegments_);
    const double radius = sanitized.radius_;
    const double tubeA = sanitized.tubeRadius_.x_;
    const double tubeB = sanitized.tubeRadius_.y_;
    const double degToRad = M_PI / 180.0;
    const double arcRad = sanitized.arc_ * degToRad;
    const bool fullRing = sanitized.arc_ >= CsgRingParams::MaxArc;
    const unsigned numRings = fullRing ? segments : segments + 1;

    // The trigonometric tables. The index segments is the end of the arc.
    ea::vector<double> segCos(segments + 1);
    ea::vector<double> segSin(segments + 1);
    for (unsigned i = 0; i <= segments; ++i)
    {
        segCos[i] = std::cos(arcRad * i / segments);
        segSin[i] = std::sin(arcRad * i / segments);
    }
    ea::vector<double> tubeCos(tubeSegments + 1);
    ea::vector<double> tubeSin(tubeSegments + 1);
    for (unsigned j = 0; j <= tubeSegments; ++j)
    {
        const double psi = (sanitized.tubeAngle_ + 360.0 * j / tubeSegments) * degToRad;
        tubeCos[j] = std::cos(psi);
        tubeSin[j] = std::sin(psi);
    }

    // Each vertex copies its position from this table, so the shared positions are bit-identical.
    // The rings come first. The section centers of an arc come last.
    const unsigned startCenter = numRings * tubeSegments;
    const unsigned endCenter = startCenter + 1;
    ea::vector<Vector3> positions(endCenter + 1);
    for (unsigned i = 0; i < numRings; ++i)
    {
        for (unsigned j = 0; j < tubeSegments; ++j)
        {
            const double r = radius + tubeA * tubeCos[j];
            positions[i * tubeSegments + j] = Vector3(static_cast<float>(r * segCos[i]),
                static_cast<float>(tubeB * tubeSin[j]), static_cast<float>(r * segSin[i]));
        }
    }
    positions[startCenter] = Vector3(static_cast<float>(radius * segCos[0]), 0.0f, static_cast<float>(radius * segSin[0]));
    positions[endCenter] =
        Vector3(static_cast<float>(radius * segCos[segments]), 0.0f, static_cast<float>(radius * segSin[segments]));
    // The end of a full ring (i == segments) copies ring 0. The tube seam (j == tubeSegments) copies j 0.
    const auto ringPosition = [&](unsigned i, unsigned j) -> const Vector3&
    { return positions[(i % numRings) * tubeSegments + j % tubeSegments]; };

    const auto bodyCorner = [&](unsigned i, unsigned j) -> RingCorner
    {
        const Vector3 radial(static_cast<float>(segCos[i]), 0.0f, static_cast<float>(segSin[i]));
        const Vector3 along(static_cast<float>(-segSin[i]), 0.0f, static_cast<float>(segCos[i]));
        RingCorner corner;
        corner.position_ = ringPosition(i, j);
        corner.uv_ = Vector2(static_cast<float>(static_cast<double>(i) / segments),
            static_cast<float>(static_cast<double>(j) / tubeSegments));
        // The normal of the elliptic section: the gradient is (cos / a, sin / b), so it is (b cos, a sin).
        corner.normal_ = (radial * static_cast<float>(tubeB * tubeCos[j]) + Vector3::UP * static_cast<float>(tubeA * tubeSin[j]))
                             .NormalizedOrDefault(radial);
        corner.tangentDir_ = along;
        corner.vDir_ = radial * static_cast<float>(-tubeA * tubeSin[j]) + Vector3::UP * static_cast<float>(tubeB * tubeCos[j]);
        return corner;
    };
    // A cap corner. The `j` is the rim index. `isCenter` selects the section center instead.
    const auto capCorner = [&](unsigned i, unsigned j, bool atStart, bool isCenter) -> RingCorner
    {
        const Vector3 radial(static_cast<float>(segCos[i]), 0.0f, static_cast<float>(segSin[i]));
        const Vector3 along(static_cast<float>(-segSin[i]), 0.0f, static_cast<float>(segCos[i]));
        RingCorner corner;
        corner.isCap_ = true;
        corner.normal_ = atStart ? -along : along;
        corner.tangentDir_ = radial;
        corner.vDir_ = Vector3::UP;
        if (isCenter)
        {
            corner.position_ = positions[atStart ? startCenter : endCenter];
            corner.uv_ = Vector2(0.5f, 0.5f);
        }
        else
        {
            corner.position_ = ringPosition(i, j);
            corner.uv_ = Vector2(static_cast<float>(tubeA * tubeCos[j] + 0.5), static_cast<float>(tubeB * tubeSin[j] + 0.5));
        }
        return corner;
    };

    const Vector4 white = Color::WHITE.ToVector4();
    double volume = 0.0;
    bool degenerate = false;

    const auto emitTriangle = [&](const RingCorner& c0, const RingCorner& c1, const RingCorner& c2)
    {
        const RingCorner* corners[3] = {&c0, &c1, &c2};
        const Vector3& p0 = c0.position_;
        const Vector3& p1 = c1.position_;
        const Vector3& p2 = c2.position_;
        if (p0 == p1 || p1 == p2 || p2 == p0)
            degenerate = true;
        const Vector3 flatNormal = (p1 - p0).CrossProduct(p2 - p0).NormalizedOrDefault(Vector3::UP);
        volume += TripleProduct(p0, p1, p2) / 6.0;

        for (const RingCorner* corner : corners)
        {
            const Vector3 normal = corner->isCap_ || sanitized.smoothNormals_ ? corner->normal_ : flatNormal;
            // The shader bitangent is cross(tangent, normal) * w. It must point along the V axis.
            const Vector3 tangent = (corner->tangentDir_ - normal * normal.DotProduct(corner->tangentDir_))
                                        .NormalizedOrDefault(corner->tangentDir_);
            const float handedness = tangent.CrossProduct(normal).DotProduct(corner->vDir_) >= 0.0f ? 1.0f : -1.0f;

            ModelVertex vertex{};
            vertex.SetPosition(corner->position_);
            vertex.SetNormal(normal);
            vertex.tangent_ = Vector4(tangent, handedness);
            vertex.uv_[0] = Vector4(corner->uv_.x_, corner->uv_.y_, 0.0f, 0.0f);
            vertex.uv_[1] = vertex.uv_[0];
            vertex.color_[0] = white;

            mesh.indices_.push_back(mesh.vertices_.size());
            mesh.vertices_.push_back(vertex);
        }
    };

    // The group of a tube quad: the quadrant of the angle of its middle. The small bias breaks the tie
    // on a quadrant border the same way for each platform.
    // Quadrant 0 is +radial, 1 is +Y, 2 is -radial, 3 is -Y. Group order: outer, inner, top, bottom.
    const unsigned quadrantGroup[4] = {0, 2, 1, 3};
    ea::vector<unsigned> quadGroup(tubeSegments);
    for (unsigned j = 0; j < tubeSegments; ++j)
    {
        const double middle = sanitized.tubeAngle_ + 360.0 * (j + 0.5) / tubeSegments;
        const long quadrant = static_cast<long>(std::floor((middle + 45.0 + 1e-6) / 90.0));
        quadGroup[j] = quadrantGroup[((quadrant % 4) + 4) % 4];
    }

    const unsigned numTriangles = 2 * segments * tubeSegments + (fullRing ? 0 : 2 * tubeSegments);
    mesh.vertices_.reserve(3 * numTriangles);
    mesh.indices_.reserve(3 * numTriangles);

    for (unsigned group = 0; group < 4; ++group)
    {
        const unsigned start = mesh.indices_.size();
        for (unsigned j = 0; j < tubeSegments; ++j)
        {
            if (quadGroup[j] != group)
                continue;
            for (unsigned i = 0; i < segments; ++i)
            {
                const RingCorner a = bodyCorner(i, j);
                const RingCorner b = bodyCorner(i + 1, j);
                const RingCorner c = bodyCorner(i + 1, j + 1);
                const RingCorner d = bodyCorner(i, j + 1);
                emitTriangle(a, d, b);
                emitTriangle(b, d, c);
            }
        }
        const unsigned count = static_cast<unsigned>(mesh.indices_.size()) - start;
        if (count)
            mesh.groups_.push_back(CsgShapeGroup{start, count, group});
    }

    if (!fullRing)
    {
        // Group 4: the start cap. Group 5: the end cap. A cap normal points away from the ring.
        for (unsigned cap = 0; cap < 2; ++cap)
        {
            const bool atStart = cap == 0;
            const unsigned ring = atStart ? 0 : segments;
            const unsigned start = mesh.indices_.size();
            const RingCorner center = capCorner(ring, 0, atStart, true);
            for (unsigned j = 0; j < tubeSegments; ++j)
            {
                const RingCorner rim0 = capCorner(ring, j, atStart, false);
                const RingCorner rim1 = capCorner(ring, j + 1, atStart, false);
                if (atStart)
                    emitTriangle(center, rim1, rim0);
                else
                    emitTriangle(center, rim0, rim1);
            }
            mesh.groups_.push_back(CsgShapeGroup{start, static_cast<unsigned>(mesh.indices_.size()) - start, 4u + cap});
        }
    }

    if (degenerate || !(volume > 0.0))
    {
        mesh.vertices_.clear();
        mesh.indices_.clear();
        mesh.groups_.clear();
        return false;
    }
    return true;
}

} // namespace Urho3D
