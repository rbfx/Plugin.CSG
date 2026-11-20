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
    if (!std::isfinite(params.startAngle_))
        params.startAngle_ = 0.0f;
    else
    {
        float angle = std::fmod(params.startAngle_, 360.0f);
        if (angle < 0.0f)
            angle += 360.0f;
        // A tiny negative angle can round to 360.
        params.startAngle_ = angle >= 360.0f ? 0.0f : angle;
    }
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

    // Vertex k is at phi = 2pi*k/n + start. M_PI is a float constant in the engine, so its error is about 1e-7.
    // The default shape (start 0, no face radius) keeps this angle, so the old positions stay bit-identical.
    // Any other shape uses the exact value of pi. Then a pyramid base lands exactly on the box corners.
    constexpr double exactPi = 3.14159265358979323846;
    const bool legacyAngles = sanitized.startAngle_ == 0.0f && !sanitized.faceRadius_;
    const double turn = legacyAngles ? twoPi : 2.0 * exactPi;
    const double startRad = static_cast<double>(sanitized.startAngle_) * exactPi / 180.0;
    // Face radius: the polygon has apothem 1, so the corner distance is 1 / cos(pi / n). Off: 1.
    const double polygonScale = sanitized.faceRadius_ ? 1.0 / std::cos(exactPi / segments) : 1.0;
    ea::vector<double> lonCos(segments + 1);
    ea::vector<double> lonSin(segments + 1);
    for (unsigned lon = 0; lon <= segments; ++lon)
    {
        const double phi = turn * lon / segments + startRad;
        lonCos[lon] = std::cos(phi) * polygonScale;
        lonSin[lon] = std::sin(phi) * polygonScale;
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
        const double phi = turn * lon / segments + startRad;
        const double c = std::cos(phi) * polygonScale;
        const double s = std::sin(phi) * polygonScale;
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
        corner.vDir_ = Vector3::FORWARD;
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

namespace
{

/// The UV frame of a face. u x v is the outward normal of the face.
/// UV 0 is (p.u, -p.v) with p relative to the shape center, as for the box.
struct FaceFrame
{
    Vector3 u_;
    Vector3 v_;
};

/// Make the frame of a flat face from its unit normal and a wanted U direction in the plane.
FaceFrame MakeFaceFrame(const Vector3& normal, const Vector3& wantedU)
{
    const Vector3 u = (wantedU - normal * normal.DotProduct(wantedU)).NormalizedOrDefault(wantedU);
    return FaceFrame{u, normal.CrossProduct(u)};
}

/// One corner of a triangle: the index in the position table and the attributes except the flat normal.
struct ShapeCorner
{
    unsigned index_{};
    Vector2 uv_;
    /// The direction of the U axis in space.
    Vector3 tangentDir_;
    /// The direction of the V axis in space.
    Vector3 vDir_;
    /// The smooth normal. Zero gives the flat normal of the triangle.
    Vector3 normal_;
};

/// Write the triangles of a shape with one position table. Each vertex copies its position from the table.
/// The writer adds the face groups, sums the signed volume and detects degenerate triangles.
class ShapeMeshWriter
{
public:
    ShapeMeshWriter(CsgShapeMesh& mesh, const ea::vector<Vector3>& positions, const Vector3& center)
        : mesh_(mesh)
        , positions_(positions)
        , center_(center)
    {
        mesh_.vertices_.clear();
        mesh_.indices_.clear();
        mesh_.groups_.clear();
    }

    /// Return the position relative to the shape center.
    Vector3 Local(unsigned index) const { return positions_[index] - center_; }

    /// Return a flat corner with the UV of the frame.
    ShapeCorner FlatCorner(unsigned index, const FaceFrame& frame) const
    {
        const Vector3 local = Local(index);
        ShapeCorner corner;
        corner.index_ = index;
        corner.uv_ = Vector2(local.DotProduct(frame.u_), -local.DotProduct(frame.v_));
        corner.tangentDir_ = frame.u_;
        corner.vDir_ = -frame.v_;
        return corner;
    }

    void BeginGroup() { groupStart_ = mesh_.indices_.size(); }

    /// Close the group. A group without triangles is not added: its slot stays free.
    void EndGroup(unsigned slot)
    {
        const unsigned count = static_cast<unsigned>(mesh_.indices_.size()) - groupStart_;
        if (count)
            mesh_.groups_.push_back(CsgShapeGroup{groupStart_, count, slot});
    }

    /// Write a flat triangle of a planar face.
    void FlatTriangle(unsigned a, unsigned b, unsigned c, const FaceFrame& frame)
    {
        Triangle(FlatCorner(a, frame), FlatCorner(b, frame), FlatCorner(c, frame));
    }

    /// Write a flat quad (a, b, c, d) as the triangles (a, b, c) and (a, c, d).
    void FlatQuad(unsigned a, unsigned b, unsigned c, unsigned d, const FaceFrame& frame)
    {
        FlatTriangle(a, b, c, frame);
        FlatTriangle(a, c, d, frame);
    }

    void Triangle(const ShapeCorner& c0, const ShapeCorner& c1, const ShapeCorner& c2)
    {
        const ShapeCorner* corners[3] = {&c0, &c1, &c2};
        const Vector3& p0 = positions_[c0.index_];
        const Vector3& p1 = positions_[c1.index_];
        const Vector3& p2 = positions_[c2.index_];
        // The flat normal uses doubles, so a tiny triangle still gets a unit normal.
        const double e1x = p1.x_ - p0.x_, e1y = p1.y_ - p0.y_, e1z = p1.z_ - p0.z_;
        const double e2x = p2.x_ - p0.x_, e2y = p2.y_ - p0.y_, e2z = p2.z_ - p0.z_;
        const double nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
        const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (p0 == p1 || p1 == p2 || p2 == p0 || !(length > 0.0))
            degenerate_ = true;
        const Vector3 flatNormal = length > 0.0
            ? Vector3(static_cast<float>(nx / length), static_cast<float>(ny / length), static_cast<float>(nz / length))
            : Vector3::UP;
        // The volume uses the positions relative to the center, so a far center does not lose the precision.
        volume_ += TripleProduct(Local(c0.index_), Local(c1.index_), Local(c2.index_)) / 6.0;

        for (const ShapeCorner* corner : corners)
        {
            const Vector3 normal = corner->normal_ == Vector3::ZERO ? flatNormal : corner->normal_;
            // The shader bitangent is cross(tangent, normal) * w. It must point along the V axis.
            const Vector3 tangent = (corner->tangentDir_ - normal * normal.DotProduct(corner->tangentDir_))
                                        .NormalizedOrDefault(corner->tangentDir_);
            const float handedness = tangent.CrossProduct(normal).DotProduct(corner->vDir_) >= 0.0f ? 1.0f : -1.0f;

            ModelVertex vertex{};
            vertex.SetPosition(positions_[corner->index_]);
            vertex.SetNormal(normal);
            vertex.tangent_ = Vector4(tangent, handedness);
            vertex.uv_[0] = Vector4(corner->uv_.x_, corner->uv_.y_, 0.0f, 0.0f);
            vertex.uv_[1] = vertex.uv_[0];
            vertex.color_[0] = Color::WHITE.ToVector4();

            mesh_.indices_.push_back(mesh_.vertices_.size());
            mesh_.vertices_.push_back(vertex);
        }
    }

    /// Mark the shape as not valid.
    void Fail() { degenerate_ = true; }

    /// Return true if the mesh is valid. Otherwise clear the mesh and return false.
    bool Finish()
    {
        if (degenerate_ || !(volume_ > 0.0))
        {
            mesh_.vertices_.clear();
            mesh_.indices_.clear();
            mesh_.groups_.clear();
            return false;
        }
        return true;
    }

private:
    CsgShapeMesh& mesh_;
    const ea::vector<Vector3>& positions_;
    Vector3 center_;
    unsigned groupStart_{};
    double volume_{};
    bool degenerate_{};
};

/// The frames of the axis-aligned faces. They are the frames of the box faces.
const FaceFrame FrameLeft{Vector3::FORWARD, Vector3::UP};
const FaceFrame FrameRight{Vector3::UP, Vector3::FORWARD};
const FaceFrame FrameDown{Vector3::RIGHT, Vector3::FORWARD};
const FaceFrame FrameUp{Vector3::FORWARD, Vector3::RIGHT};
const FaceFrame FrameBack{Vector3::UP, Vector3::RIGHT};
const FaceFrame FrameForward{Vector3::RIGHT, Vector3::UP};

Vector3 SanitizeCenter(const Vector3& center)
{
    return SanitizeVector(center, -CsgMaxShapeCenter, CsgMaxShapeCenter, Vector3::ZERO);
}

} // namespace

void CsgSanitizeWedgeParams(CsgWedgeParams& params)
{
    params.size_ = SanitizeVector(params.size_, CsgWedgeParams::MinSize, CsgWedgeParams::MaxSize, Vector3::ONE);
    params.center_ = SanitizeCenter(params.center_);
}

bool CsgGenerateWedge(const CsgWedgeParams& params, CsgShapeMesh& mesh)
{
    CsgWedgeParams sanitized = params;
    CsgSanitizeWedgeParams(sanitized);
    const Vector3 half = sanitized.size_ * 0.5f;
    const Vector3& c = sanitized.center_;

    // Each vertex copies its position from this table, so the shared corners are bit-identical.
    // 0, 1: the front bottom edge (-X, +X). 2, 3: the back bottom edge. 4, 5: the back top edge.
    const ea::vector<Vector3> positions{
        c + Vector3(-half.x_, -half.y_, -half.z_),
        c + Vector3(half.x_, -half.y_, -half.z_),
        c + Vector3(-half.x_, -half.y_, half.z_),
        c + Vector3(half.x_, -half.y_, half.z_),
        c + Vector3(-half.x_, half.y_, half.z_),
        c + Vector3(half.x_, half.y_, half.z_),
    };
    // The slope normal is (0, sz, -sy) normalized. Its V axis goes up the slope.
    const Vector3 slopeNormal = Vector3(0.0f, sanitized.size_.z_, -sanitized.size_.y_).Normalized();
    const FaceFrame slopeFrame = MakeFaceFrame(slopeNormal, Vector3::LEFT);

    ShapeMeshWriter writer(mesh, positions, c);
    mesh.vertices_.reserve(24);
    mesh.indices_.reserve(24);
    writer.BeginGroup();
    writer.FlatQuad(0, 1, 3, 2, FrameDown);
    writer.EndGroup(0);
    writer.BeginGroup();
    writer.FlatQuad(2, 3, 5, 4, FrameForward);
    writer.EndGroup(1);
    writer.BeginGroup();
    writer.FlatQuad(0, 4, 5, 1, slopeFrame);
    writer.EndGroup(2);
    writer.BeginGroup();
    writer.FlatTriangle(0, 2, 4, FrameLeft);
    writer.EndGroup(3);
    writer.BeginGroup();
    writer.FlatTriangle(1, 5, 3, FrameRight);
    writer.EndGroup(4);
    return writer.Finish();
}

void CsgSanitizeStairsParams(CsgStairsParams& params)
{
    params.size_ = SanitizeVector(params.size_, CsgStairsParams::MinSize, CsgStairsParams::MaxSize, Vector3::ONE);
    params.center_ = SanitizeCenter(params.center_);
    params.steps_ = Clamp(params.steps_, CsgStairsParams::MinSteps, CsgStairsParams::MaxSteps);
}

bool CsgGenerateStairs(const CsgStairsParams& params, CsgShapeMesh& mesh)
{
    CsgStairsParams sanitized = params;
    CsgSanitizeStairsParams(sanitized);
    const unsigned steps = static_cast<unsigned>(sanitized.steps_);
    const Vector3& size = sanitized.size_;
    const Vector3& c = sanitized.center_;
    const float hx = size.x_ * 0.5f;

    // The profile in (z, y), relative to the center. The step edges use one division each,
    // so a riser top and the next tread start share the same values.
    const unsigned numProfile = 2 * steps + 2;
    ea::vector<Vector2> profile(numProfile);
    const auto stepZ = [&](unsigned i) { return static_cast<float>(size.z_ * (static_cast<double>(i) / steps - 0.5)); };
    const auto stepY = [&](unsigned i) { return static_cast<float>(size.y_ * (static_cast<double>(i) / steps - 0.5)); };
    profile[0] = Vector2(stepZ(0), stepY(0));
    for (unsigned i = 0; i < steps; ++i)
    {
        profile[2 * i + 1] = Vector2(stepZ(i), stepY(i + 1));
        profile[2 * i + 2] = Vector2(stepZ(i + 1), stepY(i + 1));
    }
    profile[numProfile - 1] = Vector2(stepZ(steps), stepY(0));

    // Each vertex copies its position from this table, so the shared positions are bit-identical.
    // Index k is the profile point k at -X. Index numProfile + k is the same point at +X.
    ea::vector<Vector3> positions(2 * numProfile);
    for (unsigned k = 0; k < numProfile; ++k)
    {
        positions[k] = c + Vector3(-hx, profile[k].y_, profile[k].x_);
        positions[numProfile + k] = c + Vector3(hx, profile[k].y_, profile[k].x_);
    }
    const auto left = [&](unsigned k) { return k % numProfile; };
    const auto right = [&](unsigned k) { return numProfile + k % numProfile; };
    // The quad of the profile edge from k to k + 1. The profile turns clockwise in (z, y), so this winding points out.
    const auto edgeQuad = [&](ShapeMeshWriter& writer, unsigned k, const FaceFrame& frame)
    { writer.FlatQuad(left(k), left(k + 1), right(k + 1), right(k), frame); };

    ShapeMeshWriter writer(mesh, positions, c);
    const unsigned numTriangles = 4 * steps + 4 + 2 * (2 * steps);
    mesh.vertices_.reserve(3 * numTriangles);
    mesh.indices_.reserve(3 * numTriangles);

    writer.BeginGroup();
    for (unsigned i = 0; i < steps; ++i)
        edgeQuad(writer, 2 * i + 1, FrameUp);
    writer.EndGroup(0);

    writer.BeginGroup();
    for (unsigned i = 0; i < steps; ++i)
        edgeQuad(writer, 2 * i, FrameBack);
    writer.EndGroup(1);

    // The sides: a fan from the back bottom corner. Each diagonal from that corner stays inside the profile,
    // because the profile is a staircase that rises from the front to the back.
    const unsigned apex = numProfile - 1;
    writer.BeginGroup();
    for (unsigned k = 0; k + 1 < apex; ++k)
        writer.FlatTriangle(left(apex), left(k + 1), left(k), FrameLeft);
    for (unsigned k = 0; k + 1 < apex; ++k)
        writer.FlatTriangle(right(apex), right(k), right(k + 1), FrameRight);
    writer.EndGroup(2);

    writer.BeginGroup();
    edgeQuad(writer, numProfile - 1, FrameDown);
    writer.EndGroup(3);

    writer.BeginGroup();
    edgeQuad(writer, numProfile - 2, FrameForward);
    writer.EndGroup(4);
    return writer.Finish();
}

void CsgSanitizeArchParams(CsgArchParams& params)
{
    const CsgArchParams defaults;
    params.radius_ = Vector2(
        SanitizeValue(params.radius_.x_, CsgArchParams::MinRadius, CsgArchParams::MaxRadius, defaults.radius_.x_),
        SanitizeValue(params.radius_.y_, CsgArchParams::MinRadius, CsgArchParams::MaxRadius, defaults.radius_.y_));
    const float maxThickness = ea::max(params.radius_.x_, params.radius_.y_);
    params.thickness_ = Clamp(
        SanitizeValue(params.thickness_, CsgArchParams::MinThickness, CsgArchParams::MaxRadius, defaults.thickness_),
        CsgArchParams::MinThickness, maxThickness);
    params.depth_ = SanitizeValue(params.depth_, CsgArchParams::MinDepth, CsgArchParams::MaxDepth, defaults.depth_);
    params.arc_ = SanitizeValue(params.arc_, CsgArchParams::MinArc, CsgArchParams::MaxArc, defaults.arc_);
    params.segments_ = Clamp(params.segments_, CsgArchParams::MinSegments, CsgArchParams::MaxSegments);
    params.center_ = SanitizeCenter(params.center_);
}

Vector2 CsgGetArchInnerRadius(const CsgArchParams& params)
{
    const Vector2 inner = params.radius_ - Vector2(params.thickness_, params.thickness_);
    if (inner.x_ < CsgArchParams::MinInnerRadius || inner.y_ < CsgArchParams::MinInnerRadius)
        return Vector2::ZERO;
    return inner;
}

namespace
{

/// Return the cosine and the sine of the angle in degrees. A multiple of 90 degrees gives exact values.
void CosSinDegrees(double degrees, double& c, double& s)
{
    const double quarter = degrees / 90.0;
    if (quarter == std::floor(quarter))
    {
        const long q = ((static_cast<long>(quarter) % 4) + 4) % 4;
        const double cosTable[4] = {1.0, 0.0, -1.0, 0.0};
        const double sinTable[4] = {0.0, 1.0, 0.0, -1.0};
        c = cosTable[q];
        s = sinTable[q];
        return;
    }
    c = std::cos(degrees * (M_PI / 180.0));
    s = std::sin(degrees * (M_PI / 180.0));
}

/// Return the z component of (b - a) x (c - a) in doubles.
double CrossZ(const Vector3& a, const Vector3& b, const Vector3& c)
{
    const double abx = static_cast<double>(b.x_) - a.x_, aby = static_cast<double>(b.y_) - a.y_;
    const double acx = static_cast<double>(c.x_) - a.x_, acy = static_cast<double>(c.y_) - a.y_;
    return abx * acy - aby * acx;
}

} // namespace

bool CsgGenerateArch(const CsgArchParams& params, CsgShapeMesh& mesh)
{
    CsgArchParams sanitized = params;
    CsgSanitizeArchParams(sanitized);
    const unsigned segments = static_cast<unsigned>(sanitized.segments_);
    const unsigned numPoints = segments + 1;
    const double rx = sanitized.radius_.x_;
    const double ry = sanitized.radius_.y_;
    const Vector2 inner = CsgGetArchInnerRadius(sanitized);
    const bool filled = inner == Vector2::ZERO;
    const double irx = inner.x_;
    const double iry = inner.y_;
    const float hd = sanitized.depth_ * 0.5f;
    const Vector3& c = sanitized.center_;
    const double startAngle = 90.0 - sanitized.arc_ * 0.5;

    ea::vector<double> cosTable(numPoints);
    ea::vector<double> sinTable(numPoints);
    for (unsigned k = 0; k < numPoints; ++k)
        CosSinDegrees(startAngle + static_cast<double>(sanitized.arc_) * k / segments, cosTable[k], sinTable[k]);

    // Each vertex copies its position from this table, so the shared positions are bit-identical.
    const unsigned outerFront = 0;
    const unsigned outerBack = numPoints;
    const unsigned innerFront = 2 * numPoints;
    const unsigned innerBack = filled ? innerFront + 1 : 3 * numPoints;
    ea::vector<Vector3> positions(filled ? 2 * numPoints + 2 : 4 * numPoints);
    for (unsigned k = 0; k < numPoints; ++k)
    {
        const float x = static_cast<float>(rx * cosTable[k]);
        const float y = static_cast<float>(ry * sinTable[k]);
        positions[outerFront + k] = c + Vector3(x, y, hd);
        positions[outerBack + k] = c + Vector3(x, y, -hd);
        if (!filled)
        {
            const float ix = static_cast<float>(irx * cosTable[k]);
            const float iy = static_cast<float>(iry * sinTable[k]);
            positions[innerFront + k] = c + Vector3(ix, iy, hd);
            positions[innerBack + k] = c + Vector3(ix, iy, -hd);
        }
    }
    if (filled)
    {
        positions[innerFront] = c + Vector3(0.0f, 0.0f, hd);
        positions[innerBack] = c + Vector3(0.0f, 0.0f, -hd);
    }
    // The inner point k. A filled sector has the apex for each k.
    const auto innerF = [&](unsigned k) { return filled ? innerFront : innerFront + k; };
    const auto innerB = [&](unsigned k) { return filled ? innerBack : innerBack + k; };

    ShapeMeshWriter writer(mesh, positions, c);
    const unsigned numTriangles = segments * (filled ? 4 : 8) + 4;
    mesh.vertices_.reserve(3 * numTriangles);
    mesh.indices_.reserve(3 * numTriangles);

    // A corner of a curved face: the smooth normal of the ellipse, U along the curve, V along -Z.
    const auto curvedCorner = [&](unsigned index, double a, double b, unsigned k, float arcLength, bool outward)
    {
        ShapeCorner corner;
        corner.index_ = index;
        corner.uv_ = Vector2(arcLength, -(positions[index].z_ - c.z_));
        // The normal of the ellipse is (b cos t, a sin t). The tangent is (-a sin t, b cos t). Both use doubles,
        // so a small radius still gives a unit vector.
        const double nx = b * cosTable[k], ny = a * sinTable[k];
        const double length = std::sqrt(nx * nx + ny * ny);
        const Vector3 normal(static_cast<float>(nx / length), static_cast<float>(ny / length), 0.0f);
        corner.normal_ = outward ? normal : -normal;
        const double tx = -a * sinTable[k], ty = b * cosTable[k];
        const double tangentLength = std::sqrt(tx * tx + ty * ty);
        corner.tangentDir_ = Vector3(static_cast<float>(tx / tangentLength), static_cast<float>(ty / tangentLength), 0.0f);
        corner.vDir_ = Vector3::BACK;
        return corner;
    };
    const auto curvedFace = [&](unsigned front, unsigned back, double a, double b, bool outward)
    {
        float arcLength = 0.0f;
        for (unsigned k = 0; k < segments; ++k)
        {
            const Vector3 chord = positions[front + k + 1] - positions[front + k];
            const float nextLength = arcLength + Vector2(chord.x_, chord.y_).Length();
            const ShapeCorner b0 = curvedCorner(back + k, a, b, k, arcLength, outward);
            const ShapeCorner b1 = curvedCorner(back + k + 1, a, b, k + 1, nextLength, outward);
            const ShapeCorner f1 = curvedCorner(front + k + 1, a, b, k + 1, nextLength, outward);
            const ShapeCorner f0 = curvedCorner(front + k, a, b, k, arcLength, outward);
            if (outward)
            {
                writer.Triangle(b0, b1, f1);
                writer.Triangle(b0, f1, f0);
            }
            else
            {
                writer.Triangle(f0, f1, b1);
                writer.Triangle(f0, b1, b0);
            }
            arcLength = nextLength;
        }
    };

    writer.BeginGroup();
    curvedFace(outerFront, outerBack, rx, ry, true);
    writer.EndGroup(0);

    if (!filled)
    {
        writer.BeginGroup();
        curvedFace(innerFront, innerBack, irx, iry, false);
        writer.EndGroup(1);
    }

    // The front and the back. Each quad (inner k, outer k, outer k + 1, inner k + 1) uses the first diagonal
    // that gives two triangles with a positive area in the XY plane.
    ea::vector<bool> firstDiagonal(segments, true);
    if (!filled)
    {
        for (unsigned k = 0; k < segments; ++k)
        {
            const Vector3& i0 = positions[innerF(k)];
            const Vector3& o0 = positions[outerFront + k];
            const Vector3& o1 = positions[outerFront + k + 1];
            const Vector3& i1 = positions[innerF(k + 1)];
            if (CrossZ(i0, o0, o1) > 0.0 && CrossZ(i0, o1, i1) > 0.0)
                firstDiagonal[k] = true;
            else if (CrossZ(i0, o0, i1) > 0.0 && CrossZ(o0, o1, i1) > 0.0)
                firstDiagonal[k] = false;
            else
                writer.Fail();
        }
    }
    const FaceFrame frontFrame{Vector3::RIGHT, Vector3::UP};
    const FaceFrame backFrame{Vector3::LEFT, Vector3::UP};
    writer.BeginGroup();
    for (unsigned k = 0; k < segments; ++k)
    {
        if (filled)
            writer.FlatTriangle(innerFront, outerFront + k, outerFront + k + 1, frontFrame);
        else if (firstDiagonal[k])
            writer.FlatQuad(innerF(k), outerFront + k, outerFront + k + 1, innerF(k + 1), frontFrame);
        else
            writer.FlatQuad(outerFront + k, outerFront + k + 1, innerF(k + 1), innerF(k), frontFrame);
    }
    writer.EndGroup(2);

    writer.BeginGroup();
    for (unsigned k = 0; k < segments; ++k)
    {
        if (filled)
            writer.FlatTriangle(innerBack, outerBack + k + 1, outerBack + k, backFrame);
        else if (firstDiagonal[k])
            writer.FlatQuad(innerB(k), innerB(k + 1), outerBack + k + 1, outerBack + k, backFrame);
        else
            writer.FlatQuad(outerBack + k, innerB(k), innerB(k + 1), outerBack + k + 1, backFrame);
    }
    writer.EndGroup(3);

    // The ends. The start end faces the direction of a smaller angle, the end faces a larger angle.
    writer.BeginGroup();
    {
        const Vector3 radial = positions[outerFront] - positions[innerF(0)];
        const Vector3 normal = radial.CrossProduct(Vector3::FORWARD).NormalizedOrDefault(Vector3::DOWN);
        writer.FlatQuad(innerF(0), innerB(0), outerBack, outerFront, MakeFaceFrame(normal, radial.Normalized()));
    }
    {
        const Vector3 radial = positions[outerFront + segments] - positions[innerF(segments)];
        const Vector3 normal = Vector3::FORWARD.CrossProduct(radial).NormalizedOrDefault(Vector3::DOWN);
        writer.FlatQuad(innerB(segments), innerF(segments), outerFront + segments, outerBack + segments,
            MakeFaceFrame(normal, radial.Normalized()));
    }
    writer.EndGroup(4);
    return writer.Finish();
}

namespace
{
/// Return the point of the triangle that is nearest to the point.
Vector3 ClosestPointOnTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
{
    const Vector3 ab = b - a;
    const Vector3 ac = c - a;
    const Vector3 ap = p - a;
    const float d1 = ab.DotProduct(ap);
    const float d2 = ac.DotProduct(ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return a;
    const Vector3 bp = p - b;
    const float d3 = ab.DotProduct(bp);
    const float d4 = ac.DotProduct(bp);
    if (d3 >= 0.0f && d4 <= d3)
        return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        return a + ab * (d1 / (d1 - d3));
    const Vector3 cp = p - c;
    const float d5 = ab.DotProduct(cp);
    const float d6 = ac.DotProduct(cp);
    if (d6 >= 0.0f && d5 <= d6)
        return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        return a + ac * (d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}
} // namespace

ea::optional<unsigned> CsgFindSlotAtPoint(const CsgShapeMesh& mesh, const Vector3& point, const Vector3& normal)
{
    if (mesh.vertices_.empty() || mesh.indices_.size() < 3)
        return ea::nullopt;

    BoundingBox box;
    for (const ModelVertex& vertex : mesh.vertices_)
        box.Merge(vertex.GetPosition());
    const float tolerance = 0.001f * box.Size().Length();

    const unsigned groupCount = mesh.groups_.empty() ? 1 : mesh.groups_.size();
    const bool useNormal = normal.LengthSquared() > 0.0f;
    const Vector3 unitNormal = useNormal ? normal.Normalized() : Vector3::ZERO;
    ea::optional<unsigned> bestSlot;
    float bestDistance = M_INFINITY;
    for (unsigned groupIndex = 0; groupIndex < groupCount; ++groupIndex)
    {
        unsigned start = 0;
        unsigned count = mesh.indices_.size();
        unsigned slot = 0;
        if (!mesh.groups_.empty())
        {
            const CsgShapeGroup& group = mesh.groups_[groupIndex];
            start = group.indexStart_;
            count = group.indexCount_;
            slot = group.slot_ == CsgShapeGroup::SlotFromIndex ? groupIndex : group.slot_;
        }
        for (unsigned i = start; i + 2 < start + count && i + 2 < mesh.indices_.size(); i += 3)
        {
            const Vector3 a = mesh.vertices_[mesh.indices_[i]].GetPosition();
            const Vector3 b = mesh.vertices_[mesh.indices_[i + 1]].GetPosition();
            const Vector3 c = mesh.vertices_[mesh.indices_[i + 2]].GetPosition();
            Vector3 faceNormal = (b - a).CrossProduct(c - a);
            if (faceNormal.LengthSquared() <= 0.0f)
                continue;
            faceNormal.Normalize();
            if (useNormal && Abs(faceNormal.DotProduct(unitNormal)) <= 0.9f)
                continue;
            const float distance = (ClosestPointOnTriangle(point, a, b, c) - point).Length();
            const bool better = distance < bestDistance - 1e-6f
                || (distance <= bestDistance + 1e-6f && bestSlot && slot < *bestSlot);
            if (better)
            {
                bestDistance = ea::min(distance, bestDistance);
                bestSlot = slot;
            }
        }
    }
    if (!bestSlot || bestDistance > tolerance)
        return ea::nullopt;
    return bestSlot;
}

ea::string CsgGetSlotName(const ea::string& shapeTypeName, unsigned slot)
{
    static const ea::pair<const char*, ea::vector<const char*>> tables[] = {
        {"CsgBoxBrush", {"Left", "Right", "Bottom", "Top", "Back", "Front"}},
        {"CsgCylinderBrush", {"Side", "Top", "Bottom"}},
        {"CsgRingBrush", {"Outer", "Inner", "Top", "Bottom", "Start cap", "End cap"}},
        {"CsgWedgeBrush", {"Bottom", "Back", "Slope", "Left", "Right"}},
        {"CsgStairsBrush", {"Treads", "Risers", "Sides", "Bottom", "Back"}},
        {"CsgArchBrush", {"Outer", "Inner", "Front", "Back", "Ends"}},
        {"CsgSphereBrush", {"Surface"}},
    };
    for (const auto& table : tables)
    {
        if (shapeTypeName == table.first && slot < table.second.size())
            return table.second[slot];
    }
    return Format("Face {}", slot);
}

} // namespace Urho3D
