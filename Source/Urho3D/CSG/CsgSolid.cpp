// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgSolid.h"

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgFaceBvh.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include <Urho3D/Core/Assert.h>
#include <Urho3D/Core/Profiler.h>

#include <cstring>

#include <EASTL/algorithm.h>
#include <EASTL/hash_map.h>
#include <EASTL/hash_set.h>
#include <EASTL/sort.h>
#include <EASTL/unique_ptr.h>
#include <EASTL/utility.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace Urho3D
{

namespace
{

/// Distance, in epsilons, within which a vertex on the edge of another face forms a T-junction.
constexpr float JUNCTION_SCALE = 8.0f;
/// Length of open edges that a closed solid may have, in epsilons. A solid may also have a fraction of its edge length.
/// Modeled meshes can have hairline cracks that the ray parity tolerates.
constexpr float CRACK_BUDGET_EPSILONS = 200.0f;
constexpr float CRACK_BUDGET_FRACTION = 0.001f;
/// Limit of a grid cell coordinate. It keeps the conversion to an integer defined.
constexpr double MAX_CELL_COORD = 1099511627776.0;
constexpr uint64_t CELL_BITS = 21;
constexpr uint64_t CELL_MASK = (uint64_t(1) << CELL_BITS) - 1;

int64_t CellCoord(double value)
{
    double cell = std::floor(value);
    // The negated test also catches NaN.
    if (!(cell > -MAX_CELL_COORD))
        cell = -MAX_CELL_COORD;
    else if (cell > MAX_CELL_COORD)
        cell = MAX_CELL_COORD;
    return static_cast<int64_t>(cell);
}

uint64_t CellKey(int64_t x, int64_t y, int64_t z)
{
    return ((static_cast<uint64_t>(x) & CELL_MASK) << (2 * CELL_BITS)) | ((static_cast<uint64_t>(y) & CELL_MASK) << CELL_BITS)
        | (static_cast<uint64_t>(z) & CELL_MASK);
}

Vector3 MinOf(const Vector3& a, const Vector3& b)
{
    return Vector3(a.x_ < b.x_ ? a.x_ : b.x_, a.y_ < b.y_ ? a.y_ : b.y_, a.z_ < b.z_ ? a.z_ : b.z_);
}

Vector3 MaxOf(const Vector3& a, const Vector3& b)
{
    return Vector3(a.x_ > b.x_ ? a.x_ : b.x_, a.y_ > b.y_ ? a.y_ : b.y_, a.z_ > b.z_ ? a.z_ : b.z_);
}

constexpr float INF = std::numeric_limits<float>::infinity();

} // namespace

/// Edge between two welded ids, counted over all faces.
struct CsgEdgeState
{
    /// First face on the edge.
    int face_{};
    /// Number of face edges on the edge.
    int count_{};
    /// Sum of the directions of the edge: +1 from the low id to the high id, -1 from the high id to the low id.
    int winding_{};
};

/// Edge states. The key is the undirected key of the two welded ids.
using CsgRawEdgeMap = CsgFlatMap<uint64_t, CsgEdgeState, CsgPackedKeyHash>;

/// Welded point that lies near the interior of an edge. The parameter t is the place along the edge.
struct CsgJunction
{
    float t_{};
    int id_{};
};

/// Finds the welded points near an edge. A BVH over the points answers the box query.
/// The tree is built on first use.
class CsgJunctionFinder
{
public:
    CsgJunctionFinder(const CsgPointWelder& welder, float tolerance)
        : welder_(welder)
        , tolerance_(tolerance)
    {
    }

    /// Return the welded points within the tolerance of the edge interior, in order along the edge.
    void Collect(int from, int to, ea::vector<CsgJunction>& cuts)
    {
        cuts.clear();
        if (!points_)
        {
            // The tree leaves out a point with a NaN coordinate. Such a point is never near an edge.
            const ea::span<const Vector3> positions = welder_.GetPositions();
            points_ = ea::make_unique<CsgFaceBvh>(positions, positions);
            stack_.reserve(64);
        }

        const Vector3 start = welder_.GetPosition(from);
        const Vector3 end = welder_.GetPosition(to);
        const Vector3 span = end - start;
        const float spanLengthSquared = span.LengthSquared();
        const float toleranceSquared = tolerance_ * tolerance_;

        points_->CollectBox(MinOf(start, end), MaxOf(start, end), tolerance_, nearby_, stack_);
        for (const int item : nearby_)
        {
            if (item == from || item == to)
                continue;
            const Vector3& point = welder_.GetPosition(item);
            const float t = (point - start).DotProduct(span) / spanLengthSquared;
            // The negated test also rejects NaN, so the sort below sees only finite keys.
            if (!(t > 0.0f && t < 1.0f))
                continue;
            if ((point - (start + span * t)).LengthSquared() <= toleranceSquared)
                cuts.push_back({t, item});
        }
        // The id breaks a tie, so the order does not depend on the order of the tree.
        ea::sort(cuts.begin(), cuts.end(),
            [](const CsgJunction& a, const CsgJunction& b) { return a.t_ < b.t_ || (a.t_ == b.t_ && a.id_ < b.id_); });
    }

private:
    const CsgPointWelder& welder_;
    float tolerance_{};
    ea::unique_ptr<CsgFaceBvh> points_;
    ea::vector<int> nearby_;
    ea::vector<int> stack_;
};

/// Face corners welded into loops of vertex ids.
///
/// Repeated ids are dropped. A loop with less than three ids, or a face without triangles,
/// bounds no surface and gets a loop length of zero.
/// The solid keeps the loops after the build, so later steps can use the welded ids.
struct CsgWeldedLoops
{
    CsgWeldedLoops(float epsilon, unsigned cornerCount)
        : welder_(epsilon, cornerCount)
        , ids_(cornerCount)
        , vertexIds_(cornerCount)
        , corners_(cornerCount)
    {
    }

    /// Make loops over distinct positions that a boolean assigned. The caller sets the ids.
    CsgWeldedLoops(ea::vector<Vector3>&& positions, unsigned cornerCount)
        : welder_(ea::move(positions))
        , ids_(cornerCount)
        , vertexIds_(cornerCount)
        , corners_(cornerCount)
    {
    }

    /// Return the finder of junctions. The finder is built on first use.
    /// Only the build of a solid and the repair after it call this function. They run before the solid is shared.
    CsgJunctionFinder& GetFinder() const
    {
        if (!finder_)
            finder_ = ea::make_unique<CsgJunctionFinder>(welder_, welder_.GetEpsilon() * JUNCTION_SCALE);
        return *finder_;
    }

    CsgPointWelder welder_;
    /// Welded id of each face corner.
    ea::vector<int> ids_;
    /// Loop ids of each face. The layout is the same as in ids_.
    ea::vector<int> vertexIds_;
    /// Index in its face of the corner that each loop id was taken from. The layout is the same as in ids_.
    ea::vector<int> corners_;
    ea::vector<int> loopLength_;
    /// First corner of each face, and one more entry at the end. It is the layout of ids_.
    ea::vector<unsigned> vertexStart_;
    /// Edge states between loop ids. The edges are built on first use, see EnsureRawEdges.
    mutable CsgRawEdgeMap rawEdges_;
    /// Each face edge after the first on its welded edge, paired with the first face of that edge.
    mutable ea::vector<ea::pair<int, int>> links_;
    /// True if rawEdges_ and links_ are built.
    mutable bool edgesBuilt_{};
    /// True if every corner is exactly at the position of its welded id.
    bool canonical_{true};
    /// Grid point of each welded id. The position of an id is grid / scale of it.
    ea::vector<CsgGridPoint> grid_;
    /// True if the position of an id is not finite or is outside the range of the grid.
    bool outOfRange_{};
    mutable ea::unique_ptr<CsgJunctionFinder> finder_;
};

namespace
{

/// Largest coordinate after the clamp of an out of range position.
constexpr float CLAMPED_COORDINATE = 8191.0f;

/// Return true if the coordinate is finite and its grid coordinate is inside the range of the grid.
/// The limit is half a grid step below the coordinate limit, so the rounding cannot reach the integer limit.
bool InGridRange(float value)
{
    return std::fabs(static_cast<double>(value)) < static_cast<double>(CsgGridCoordinateLimit) - 0.5 / CsgGridScale;
}

/// Return true if each coordinate of the position is in the range of the grid.
bool InGridRange(const Vector3& position)
{
    return InGridRange(position.x_) && InGridRange(position.y_) && InGridRange(position.z_);
}

/// Whether a point lies on the edge between two others, clear of its ends: exactly, or within the distance
/// that counts as straight. The distance is absolute, in grid steps.
bool IsOnEdge(const CsgGridPoint& point, const CsgGridPoint& from, const CsgGridPoint& to, double straight)
{
    const double ex = static_cast<double>(to.x_) - from.x_;
    const double ey = static_cast<double>(to.y_) - from.y_;
    const double ez = static_cast<double>(to.z_) - from.z_;
    const double wx = static_cast<double>(point.x_) - from.x_;
    const double wy = static_cast<double>(point.y_) - from.y_;
    const double wz = static_cast<double>(point.z_) - from.z_;
    const double length = ex * ex + ey * ey + ez * ez;
    const double along = wx * ex + wy * ey + wz * ez;
    // Both branches below return false for a point that is not between the ends, so the exact test can wait.
    if (along <= 0.0 || along >= length)
        return false;
    const CsgNormal normal = CsgExact::Normal(from, to, point);
    if (normal.x_.Sign() == 0 && normal.y_.Sign() == 0 && normal.z_.Sign() == 0)
        return true;
    if (along <= length * 1e-3 || along >= length * (1.0 - 1e-3))
        return false;
    const double cx = wy * ez - wz * ey;
    const double cy = wz * ex - wx * ez;
    const double cz = wx * ey - wy * ex;
    return cx * cx + cy * cy + cz * cz <= straight * straight * length;
}

/// Give area to each triangle that has none. The ear clipper closes an edge shorter than its rounding with a
/// triangle, and on a straight run of the boundary that triangle is flat: one corner lies on the edge between
/// the other two, exactly or within the distance the clipper takes for straight. The triangle across that
/// edge then has the corner on its edge. The diagonal between the two turns: both then end at that corner
/// and have area. The indices are indices of the corners.
void TurnFlatTriangles(ea::span<const ModelVertex> corners, ea::vector<unsigned>& indices)
{
    if (indices.size() < 6)
        return;
    // A corner that has no grid point gives no exact test.
    for (const ModelVertex& corner : corners)
    {
        if (!InGridRange(corner.GetPosition()))
            return;
    }
    ea::vector<CsgGridPoint> grid;
    grid.reserve(corners.size());
    double extent = 0.0;
    for (const ModelVertex& corner : corners)
    {
        const CsgGridPoint point = CsgGridPoint::From(corner.GetPosition());
        grid.push_back(point);
        extent = ea::max(extent, ea::max(std::abs(static_cast<double>(point.x_)),
            ea::max(std::abs(static_cast<double>(point.y_)), std::abs(static_cast<double>(point.z_)))));
    }

    // A turn can leave a flat triangle when more corners are on the line. The budget ends such a chain.
    const auto turn = [&](double straight, int budget)
    {
        unsigned t = 0;
        while (t + 2 < indices.size())
        {
            bool turned = false;
            for (unsigned k = 0; k < 3 && !turned; ++k)
            {
                const unsigned middle = indices[t + k];
                const unsigned p = indices[t + (k + 1) % 3];
                const unsigned q = indices[t + (k + 2) % 3];
                if (!IsOnEdge(grid[middle], grid[p], grid[q], straight))
                    continue;
                if (budget-- <= 0)
                    return;
                for (unsigned o = 0; o + 2 < indices.size() && !turned; o += 3)
                {
                    for (unsigned j = 0; j < 3 && o != t; ++j)
                    {
                        if (grid[indices[o + j]] != grid[q] || grid[indices[o + (j + 1) % 3]] != grid[p])
                            continue;
                        const unsigned far = indices[o + (j + 2) % 3];
                        // The turn must not make the same defect at the far corner. A triangle that is on one line
                        // exactly turns also into two thin triangles: they have area.
                        const double limit = IsOnEdge(grid[middle], grid[p], grid[q], 0.0) ? 0.0 : straight;
                        if (IsOnEdge(grid[middle], grid[p], grid[far], limit)
                            || IsOnEdge(grid[middle], grid[far], grid[q], limit)
                            || IsOnEdge(grid[far], grid[middle], grid[p], limit)
                            || IsOnEdge(grid[far], grid[q], grid[middle], limit))
                            break;
                        indices[t] = middle;
                        indices[t + 1] = p;
                        indices[t + 2] = far;
                        indices[o] = q;
                        indices[o + 1] = middle;
                        indices[o + 2] = far;
                        turned = true;
                        break;
                    }
                }
            }
            // Both new triangles are examined again from the start.
            t = turned ? 0 : t + 3;
        }
    };

    // The triangles that are on one line exactly turn first, with a budget of their own: a thin face has many
    // triangles within the straight distance, and they must not use up the turns. A turn within the straight
    // distance can put three corners on one line again, so the exact turns run one more time.
    const int count = static_cast<int>(indices.size());
    turn(0.0, count * 4);
    turn(extent * CsgStraightDistance, count);
    turn(0.0, count * 4);
}

/// Whether the grid point p is before q in the order of the grid points: by x, then y, then z.
bool IsBefore(const CsgGridPoint& p, const CsgGridPoint& q)
{
    if (p.x_ != q.x_)
        return p.x_ < q.x_;
    if (p.y_ != q.y_)
        return p.y_ < q.y_;
    return p.z_ < q.z_;
}

/// The three grid points of a triangle in one order.
struct TriangleKey
{
    CsgGridPoint a_;
    CsgGridPoint b_;
    CsgGridPoint c_;

    friend bool operator==(const TriangleKey& l, const TriangleKey& r)
    {
        return l.a_ == r.a_ && l.b_ == r.b_ && l.c_ == r.c_;
    }
};

struct TriangleKeyHash
{
    size_t operator()(const TriangleKey& key) const;
};

uint64_t GridPointNumber(const CsgGridPoint& point)
{
    uint64_t number = static_cast<uint64_t>(static_cast<int64_t>(point.x_)) * 0x9E3779B97F4A7C15ULL
        + static_cast<uint64_t>(static_cast<int64_t>(point.y_));
    number = (number ^ number >> 32) * 0xC2B2AE3D27D4EB4FULL + static_cast<uint64_t>(static_cast<int64_t>(point.z_));
    number = (number ^ number >> 29) * 0x165667B19E3779F9ULL;
    return number ^ number >> 32;
}

size_t TriangleKeyHash::operator()(const TriangleKey& key) const
{
    return static_cast<size_t>(GridPointNumber(key.a_) + GridPointNumber(key.b_) + GridPointNumber(key.c_));
}

/// Return the key of the triangle on three points and its direction, 1 or -1. Return false if two corners
/// are on one point.
bool MakeTriangleKey(CsgGridPoint a, CsgGridPoint b, CsgGridPoint c, TriangleKey& key, int& direction)
{
    direction = 1;
    if (IsBefore(b, a))
    {
        ea::swap(a, b);
        direction = -direction;
    }
    if (IsBefore(c, b))
    {
        ea::swap(b, c);
        direction = -direction;
    }
    if (IsBefore(b, a))
    {
        ea::swap(a, b);
        direction = -direction;
    }
    key = {a, b, c};
    return a != b && b != c;
}

/// Whether the triangles can have a triangle to cancel: a triangle with two corners on one grid point, or two
/// triangles with one number. The number of a triangle comes from its three grid points and is the same
/// for each order of them, so two triangles on the same three grid points have one number. Most regions
/// have no such triangle, and this test needs no map of the grid points.
/// The positions must be in the range of the grid.
bool CanCancel(ea::span<const Vector3> positions)
{
    static thread_local ea::vector<uint64_t> numbers;
    unsigned size = 16;
    while (size < positions.size())
        size *= 2;
    numbers.assign(size, 0);
    for (unsigned t = 0; t + 2 < positions.size(); t += 3)
    {
        const CsgGridPoint a = CsgGridPoint::From(positions[t]);
        const CsgGridPoint b = CsgGridPoint::From(positions[t + 1]);
        const CsgGridPoint c = CsgGridPoint::From(positions[t + 2]);
        if (a == b || b == c || a == c)
            return true;
        const uint64_t number = GridPointNumber(a) + GridPointNumber(b) + GridPointNumber(c);
        // Zero marks a free slot.
        if (number == 0)
            return true;
        for (unsigned slot = static_cast<unsigned>(number >> 32) & (size - 1);; slot = (slot + 1) & (size - 1))
        {
            if (numbers[slot] == number)
                return true;
            if (numbers[slot] != 0)
                continue;
            numbers[slot] = number;
            break;
        }
    }
    return false;
}

/// Snap a position to the grid. A coordinate outside of the range is clamped, a NaN becomes zero.
/// Only the operand entry and a solid without exact grid points call this. A boolean result keeps its grid points.
CsgGridPoint SnapToGrid(const Vector3& position)
{
    float values[3] = {position.x_, position.y_, position.z_};
    for (float& value : values)
    {
        if (!std::isfinite(value))
            value = 0.0f;
        else if (!InGridRange(value))
            value = value < 0.0f ? -CLAMPED_COORDINATE : CLAMPED_COORDINATE;
    }
    return CsgGridPoint::From(Vector3(values[0], values[1], values[2]));
}

/// Return the position at grid / scale of the grid point. A coordinate out of the range of the grid stays as it is.
Vector3 SnapPosition(const Vector3& position, const CsgGridPoint& point)
{
    const Vector3 onGrid = point.ToPosition();
    return Vector3(InGridRange(position.x_) ? onGrid.x_ : position.x_,
        InGridRange(position.y_) ? onGrid.y_ : position.y_, InGridRange(position.z_) ? onGrid.z_ : position.z_);
}

/// Weld the corners within epsilon in face order, and snap the first position of each welded id to the grid.
/// Return the faces with each corner at the snapped position of its id, and the grid point of each corner.
/// A face that does not move is the same face. This is the weld and the snap of the operand entry in one pass,
/// without the T-junction repair. So it is correct only if the faces close up exactly after the snap.
void PresnappedFaces(const ea::vector<CsgSharedPolygon>& faces, float epsilon, ea::vector<CsgSharedPolygon>& snapped,
    ea::vector<CsgGridPoint>& grid)
{
    unsigned cornerCount = 0;
    for (const CsgSharedPolygon& face : faces)
        cornerCount += face->vertices_.size();
    CsgPointWelder welder(epsilon, cornerCount);
    ea::vector<Vector3> idPositions;
    ea::vector<CsgGridPoint> idGrid;
    idPositions.reserve(cornerCount);
    idGrid.reserve(cornerCount);
    // A vector with a size of zero must not allocate. Keep the buffer pointers null.
    ea::vector<int> ids;
    if (cornerCount != 0)
        ids.resize(cornerCount);
    unsigned corner = 0;
    for (const CsgSharedPolygon& face : faces)
    {
        for (const ModelVertex& vertex : face->vertices_)
        {
            const int id = welder.Add(vertex.GetPosition());
            if (id == static_cast<int>(idGrid.size()))
            {
                const Vector3& position = welder.GetPosition(id);
                idGrid.push_back(SnapToGrid(position));
                idPositions.push_back(SnapPosition(position, idGrid.back()));
            }
            ids[corner++] = id;
        }
    }

    snapped.clear();
    snapped.reserve(faces.size());
    grid.resize(cornerCount);
    corner = 0;
    for (const CsgSharedPolygon& face : faces)
    {
        const unsigned start = corner;
        bool moved = false;
        for (const ModelVertex& vertex : face->vertices_)
        {
            const int id = ids[corner];
            grid[corner++] = idGrid[id];
            moved |= idPositions[id] != vertex.GetPosition();
        }
        if (!moved)
        {
            snapped.push_back(face);
            continue;
        }
        ea::vector<ModelVertex> vertices = face->vertices_;
        for (unsigned i = 0; i < vertices.size(); ++i)
            vertices[i].SetPosition(idPositions[ids[start + i]]);
        // The constructor computes the plane again from the snapped positions.
        snapped.push_back(ea::make_shared<CsgPolygon>(ea::move(vertices), face->shared_));
    }
}

void BuildLoops(CsgWeldedLoops& loops, ea::span<const unsigned> vertexStart, ea::span<const unsigned> triangleStart)
{
    const unsigned faceCount = vertexStart.size() - 1;
    loops.loopLength_.resize(faceCount);
    for (unsigned face = 0; face < faceCount; ++face)
    {
        const unsigned start = vertexStart[face];
        const unsigned count = vertexStart[face + 1] - start;
        unsigned length = 0;
        for (unsigned i = 0; i < count; ++i)
        {
            const int id = loops.ids_[start + i];
            if (length != 0 && loops.vertexIds_[start + length - 1] == id)
                continue;
            loops.corners_[start + length] = static_cast<int>(i);
            loops.vertexIds_[start + length++] = id;
        }
        while (length > 1 && loops.vertexIds_[start + length - 1] == loops.vertexIds_[start])
            --length;
        const bool hasSurface = triangleStart[face] != triangleStart[face + 1];
        loops.loopLength_[face] = hasSurface && length >= 3 ? static_cast<int>(length) : 0;
    }
}

/// Build the edge states and the links on the first call. A later call does nothing.
/// Only the build of a solid and the repair after it call this function, before any other thread sees the solid.
/// Because of this, the function needs no lock.
void EnsureRawEdges(const CsgWeldedLoops& loops)
{
    if (loops.edgesBuilt_)
        return;
    loops.edgesBuilt_ = true;
    const ea::span<const unsigned> vertexStart(loops.vertexStart_.data(), loops.vertexStart_.size());
    loops.rawEdges_.Reserve(loops.vertexIds_.size());
    const unsigned faceCount = loops.loopLength_.size();
    for (unsigned face = 0; face < faceCount; ++face)
    {
        const unsigned count = loops.loopLength_[face];
        const unsigned start = vertexStart[face];
        for (unsigned i = 0; i < count; ++i)
        {
            const int from = loops.vertexIds_[start + i];
            const int to = loops.vertexIds_[start + (i + 1) % count];
            bool existed = false;
            CsgEdgeState& state = loops.rawEdges_.FindOrAdd(CsgUndirectedKey(from, to), existed);
            if (existed)
                loops.links_.push_back({static_cast<int>(face), state.face_});
            else
                state.face_ = static_cast<int>(face);
            ++state.count_;
            state.winding_ += from < to ? 1 : -1;
        }
    }
}

/// Return true if every directed edge has an opposite partner, so the loops close up exactly.
bool IsBalanced(const CsgWeldedLoops& loops)
{
    EnsureRawEdges(loops);
    for (const auto& entry : loops.rawEdges_)
    {
        if (entry.value_.winding_ != 0)
            return false;
    }
    return true;
}

/// Return true if the edge has an opposite partner, so it cannot end at a T-junction.
/// Solids that touch along an edge pinch it with two balanced pairs.
bool IsMatched(const CsgWeldedLoops& loops, int from, int to)
{
    EnsureRawEdges(loops);
    const CsgEdgeState* state = loops.rawEdges_.Find(CsgUndirectedKey(from, to));
    return state && state->winding_ == 0;
}

/// Return true if a face with triangles welds to less than three ids.
bool HasCollapsedFace(const CsgWeldedLoops& loops, ea::span<const unsigned> triangleStart)
{
    for (unsigned face = 0; face < loops.loopLength_.size(); ++face)
    {
        if (loops.loopLength_[face] < 3 && triangleStart[face] != triangleStart[face + 1])
            return true;
    }
    return false;
}

/// Disjoint sets of faces with union by rank.
class FaceSets
{
public:
    explicit FaceSets(unsigned count)
    {
        // A vector with a size of zero must not allocate. Keep the buffer pointers null.
        if (count == 0)
            return;
        parents_.resize(count);
        ranks_.resize(count, 0);
        for (unsigned i = 0; i < count; ++i)
            parents_[i] = static_cast<int>(i);
    }

    int Find(int face)
    {
        while (parents_[face] != face)
        {
            parents_[face] = parents_[parents_[face]];
            face = parents_[face];
        }
        return face;
    }

    void Union(int first, int second)
    {
        first = Find(first);
        second = Find(second);
        if (first == second)
            return;
        if (ranks_[first] < ranks_[second])
            ea::swap(first, second);
        parents_[second] = first;
        if (ranks_[first] == ranks_[second])
            ++ranks_[first];
    }

private:
    ea::vector<int> parents_;
    ea::vector<unsigned char> ranks_;
};

/// Number the components in the order of their first face and list the faces of each component.
void GroupComponents(unsigned faceCount, FaceSets& sets, ea::vector<unsigned>& componentStart,
    ea::vector<unsigned>& componentFaces, ea::vector<unsigned>& faceComponents)
{
    // A vector with a size of zero must not allocate. Keep the buffer pointers null.
    ea::vector<int> componentIndices;
    if (faceCount != 0)
    {
        componentIndices.resize(faceCount, -1);
        faceComponents.resize(faceCount);
    }
    else
        faceComponents.clear();
    unsigned componentCount = 0;
    for (unsigned face = 0; face < faceCount; ++face)
    {
        const int root = sets.Find(static_cast<int>(face));
        if (componentIndices[root] < 0)
            componentIndices[root] = static_cast<int>(componentCount++);
        faceComponents[face] = componentIndices[root];
    }
    componentStart.assign(componentCount + 1, 0);
    for (unsigned face = 0; face < faceCount; ++face)
        ++componentStart[faceComponents[face] + 1];
    for (unsigned i = 0; i < componentCount; ++i)
        componentStart[i + 1] += componentStart[i];
    componentFaces.resize(faceCount);
    ea::vector<unsigned> fill(componentStart.begin(), componentStart.begin() + componentCount);
    for (unsigned face = 0; face < faceCount; ++face)
        componentFaces[fill[faceComponents[face]]++] = face;
}

/// Result of the topology build.
struct Topology
{
    ea::vector<unsigned> componentStart;
    ea::vector<unsigned> componentFaces;
    ea::vector<unsigned> faceComponents;
    ea::unique_ptr<CsgWeldedLoops> exact;
    /// Corners welded within epsilon. It is empty if the exact loops are balanced.
    ea::unique_ptr<CsgWeldedLoops> welded;
    ea::vector<ea::pair<Vector3, Vector3>> openEdges;
    bool closed{true};
    bool hasJunctions{};
};

/// Make the loops of the corners welded by identical position.
///
/// With a grid and all corners in the range of the grid, the corners with equal grid points get one id.
/// The ids are numbered in the order of first use, and the position of an id is the position of its first corner.
/// If a corner is not at the position of its id, the loops are not canonical, and the solid needs a repair.
/// Out of range, the exact ids of a boolean weld the corners. The ids must be in [0, idSpace).
/// Out of range and without valid ids, the function welds the corners by identical position, because two corners
/// can have one clamped grid point. Then the grid point of an id is the one of its first corner.
/// Without a grid and without valid ids, the function welds the corners by position.
/// The grid has one grid point for each corner, or none. The grid point of an id is the one of its first corner.
/// Without a grid, the function snaps the position of each id to the grid. This is the only place that snaps.
ea::unique_ptr<CsgWeldedLoops> BuildExactLoops(ea::span<const Vector3> corners, ea::span<const unsigned> vertexStart,
    ea::span<const unsigned> triangleStart, ea::span<const int> exactIds, int exactIdSpace,
    ea::span<const CsgGridPoint> exactGrid)
{
    const unsigned cornerCount = corners.size();
    ea::unique_ptr<CsgWeldedLoops> result;
    // Empty ids are legal. Other ids must have one entry for each corner and a positive id space.
    URHO3D_ASSERT(exactIds.empty() || (exactIds.size() == cornerCount && exactIdSpace > 0),
        "CsgSolid exact ids must have one entry for each corner");
    URHO3D_ASSERT(exactGrid.empty() || exactGrid.size() == cornerCount,
        "CsgSolid exact grid must have one entry for each corner");
    const bool hasGrid = exactGrid.size() == cornerCount && cornerCount != 0;
    // A grid point out of range is clamped, so two such corners can have one grid point. Then the ids weld.
    bool gridInRange = hasGrid;
    for (unsigned i = 0; i < cornerCount && gridInRange; ++i)
        gridInRange = InGridRange(corners[i]);
    ea::vector<CsgGridPoint> grid;
    if (!gridInRange && exactIds.size() == cornerCount && cornerCount != 0 && exactIdSpace > 0)
    {
        ea::vector<int> dense(exactIdSpace, -1);
        ea::vector<int> denseIds(cornerCount);
        ea::vector<Vector3> positions;
        positions.reserve(cornerCount);
        grid.reserve(cornerCount);
        bool valid = true;
        for (unsigned i = 0; i < cornerCount; ++i)
        {
            const int id = exactIds[i];
            URHO3D_ASSERT(id >= 0 && id < exactIdSpace, "CsgSolid exact id is out of range");
            if (id < 0 || id >= exactIdSpace)
            {
                valid = false;
                break;
            }
            int& slot = dense[id];
            if (slot < 0)
            {
                slot = static_cast<int>(positions.size());
                positions.push_back(corners[i]);
                grid.push_back(hasGrid ? exactGrid[i] : SnapToGrid(corners[i]));
            }
            // Corners with one id are one vertex, so they have one grid point.
            URHO3D_ASSERT(!hasGrid || grid[slot] == exactGrid[i],
                "CsgSolid corners with one exact id have two grid points");
            denseIds[i] = slot;
        }
        if (valid)
        {
            result = ea::make_unique<CsgWeldedLoops>(ea::move(positions), cornerCount);
            result->ids_ = ea::move(denseIds);
        }
        else
            grid.clear();
    }
    if (!result && gridInRange)
    {
        // The grid point is the identity of a vertex: corners with equal grid points weld, also with two exact ids.
        // Two grid points can have one float position in double mode.
        CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash> gridIds;
        bool canonical = true;
        gridIds.Reserve(exactIdSpace > 0 ? ea::min(cornerCount, static_cast<unsigned>(exactIdSpace)) : cornerCount);
        ea::vector<int> ids(cornerCount);
        ea::vector<Vector3> positions;
        positions.reserve(cornerCount);
        grid.reserve(cornerCount);
        for (unsigned i = 0; i < cornerCount; ++i)
        {
            const CsgGridPoint& point = exactGrid[i];
            CsgVec3Key key;
            key.x_ = static_cast<uint32_t>(point.x_);
            key.y_ = static_cast<uint32_t>(point.y_);
            key.z_ = static_cast<uint32_t>(point.z_);
            bool existed = false;
            int& slot = gridIds.FindOrAdd(key, existed);
            if (!existed)
            {
                slot = static_cast<int>(positions.size());
                positions.push_back(corners[i]);
                grid.push_back(point);
            }
            ids[i] = slot;
            // Two corners closer than half a grid step can have one grid point and two positions.
            canonical &= corners[i] == positions[slot];
        }
        result = ea::make_unique<CsgWeldedLoops>(ea::move(positions), cornerCount);
        result->ids_ = ea::move(ids);
        result->canonical_ = canonical;
    }
    if (!result)
    {
        // The weld by identical position. An id takes the given grid point of its first corner, or else the snap.
        result = ea::make_unique<CsgWeldedLoops>(0.0f, cornerCount);
        grid.reserve(cornerCount);
        for (unsigned i = 0; i < cornerCount; ++i)
        {
            const int id = result->welder_.Add(corners[i]);
            if (id == static_cast<int>(grid.size()))
                grid.push_back(hasGrid ? exactGrid[i] : SnapToGrid(corners[i]));
            result->ids_[i] = id;
        }
    }
    result->grid_ = ea::move(grid);
    for (const Vector3& position : result->welder_.GetPositions())
    {
        if (!InGridRange(position))
        {
            result->outOfRange_ = true;
            break;
        }
    }
    result->vertexStart_.assign(vertexStart.begin(), vertexStart.end());
    BuildLoops(*result, vertexStart, triangleStart);
    return result;
}

/// Split the faces into components and find out if the faces bound a closed surface.
/// Faces are in one component if they share a welded edge.
///
/// The corners are welded exactly first. If every edge has an opposite partner, the exact loops give the components.
/// Otherwise the distinct positions are welded again within epsilon.
/// Then an edge that ends at a T-junction is split at the welded points along it.
/// The optional exact ids come from a boolean. See BuildExactLoops.
void BuildTopology(ea::span<const Vector3> corners, ea::span<const unsigned> vertexStart,
    ea::span<const unsigned> triangleStart, float epsilon, ea::span<const int> exactIds, int exactIdSpace,
    ea::span<const CsgGridPoint> exactGrid, Topology& result)
{
    URHO3D_PROFILE("CsgSolid::BuildTopology");
    const unsigned faceCount = vertexStart.size() - 1;
    const unsigned cornerCount = corners.size();
    FaceSets sets(faceCount);

    // An empty solid bounds nothing and is closed. A face without a surface is a hole.
    result.closed = true;
    for (unsigned face = 0; face < faceCount; ++face)
    {
        if (vertexStart[face + 1] - vertexStart[face] < 3 || triangleStart[face] == triangleStart[face + 1])
            result.closed = false;
    }

    result.exact = BuildExactLoops(corners, vertexStart, triangleStart, exactIds, exactIdSpace, exactGrid);
    CsgWeldedLoops& exact = *result.exact;

    // Loops that close up exactly, as boolean results and most modeled meshes do, are closed within epsilon
    // as well and need no second welding. Only float noise between neighboring faces calls for it.
    const bool balanced = IsBalanced(exact);
    // A boolean result has exact ids and grid points. Its topology is exact: no weld within epsilon, no junctions
    // and no crack budget. The surface is closed if every directed edge has an opposite partner.
    const bool exactOnly = !exactIds.empty();
    if (!balanced && !exactOnly)
    {
        result.welded = ea::make_unique<CsgWeldedLoops>(epsilon, cornerCount);
        CsgWeldedLoops& welded = *result.welded;
        ea::vector<int> map(exact.welder_.Count());
        welded.grid_.reserve(map.size());
        // A corner off the position of its exact id stays off the position of its welded id.
        welded.canonical_ = exact.canonical_;
        for (unsigned i = 0; i < map.size(); ++i)
        {
            map[i] = welded.welder_.Add(exact.welder_.GetPosition(i));
            welded.canonical_ &= map[i] == static_cast<int>(i);
            // A new welded id takes the position of exact id i, so it takes its grid point too.
            if (map[i] == static_cast<int>(welded.grid_.size()))
                welded.grid_.push_back(exact.grid_[i]);
        }
        welded.outOfRange_ = exact.outOfRange_;
        for (unsigned i = 0; i < cornerCount; ++i)
            welded.ids_[i] = map[exact.ids_[i]];
        welded.vertexStart_.assign(vertexStart.begin(), vertexStart.end());
        BuildLoops(welded, vertexStart, triangleStart);
    }
    CsgWeldedLoops& loops = result.welded ? *result.welded : exact;
    EnsureRawEdges(loops);
    const bool collapsed = HasCollapsedFace(loops, triangleStart);

    // A face that welds to less than three ids takes no part in the edge graph.
    // It belongs to the component of the faces that share its vertices.
    const auto joinCollapsed = [&]()
    {
        CsgFlatMap<uint64_t, int, CsgPackedKeyHash> vertexOwner;
        for (unsigned face = 0; face < faceCount; ++face)
        {
            if (loops.loopLength_[face] < 3)
                continue;
            for (int i = 0; i < loops.loopLength_[face]; ++i)
            {
                const int id = loops.vertexIds_[vertexStart[face] + i];
                vertexOwner.TryAdd(static_cast<uint64_t>(id), static_cast<int>(face));
            }
        }
        for (unsigned face = 0; face < faceCount; ++face)
        {
            if (loops.loopLength_[face] >= 3)
                continue;
            for (unsigned i = vertexStart[face]; i < vertexStart[face + 1]; ++i)
            {
                if (const int* owner = vertexOwner.Find(static_cast<uint64_t>(loops.ids_[i])))
                {
                    sets.Union(static_cast<int>(face), *owner);
                    break;
                }
            }
        }
    };

    // Balanced loops match every edge exactly. Nothing can end at a T-junction, and the faces
    // that each edge joins are known already. Exact loops of a boolean result use the same path.
    if ((balanced && !collapsed) || exactOnly)
    {
        for (const auto& link : loops.links_)
            sets.Union(link.first, link.second);
        if (collapsed)
            joinCollapsed();
        result.closed &= balanced;
        if (!balanced)
        {
            for (const auto& entry : loops.rawEdges_)
            {
                if (entry.value_.winding_ == 0)
                    continue;
                result.openEdges.push_back({loops.welder_.GetPosition(static_cast<int>(entry.key_ >> 32)),
                    loops.welder_.GetPosition(static_cast<int>(static_cast<uint32_t>(entry.key_)))});
            }
        }
        GroupComponents(faceCount, sets, result.componentStart, result.componentFaces, result.faceComponents);
        return;
    }

    // Only edges without an opposite partner can end at a T-junction. Split those at the welded points on their
    // interior, so a long edge matches the shorter edges along it.
    CsgRawEdgeMap edges;
    edges.Reserve(loops.rawEdges_.Size());
    const auto registerEdge = [&](int face, int from, int to)
    {
        bool existed = false;
        CsgEdgeState& state = edges.FindOrAdd(CsgUndirectedKey(from, to), existed);
        if (existed)
            sets.Union(face, state.face_);
        else
            state.face_ = face;
        ++state.count_;
        state.winding_ += from < to ? 1 : -1;
    };

    CsgJunctionFinder& finder = loops.GetFinder();
    ea::vector<CsgJunction> cuts;
    for (unsigned face = 0; face < faceCount; ++face)
    {
        const unsigned count = loops.loopLength_[face];
        if (count < 3)
            continue;
        const unsigned start = vertexStart[face];
        for (unsigned i = 0; i < count; ++i)
        {
            const int from = loops.vertexIds_[start + i];
            const int to = loops.vertexIds_[start + (i + 1) % count];
            if (IsMatched(loops, from, to))
            {
                registerEdge(face, from, to);
                continue;
            }
            finder.Collect(from, to, cuts);
            result.hasJunctions |= !cuts.empty();
            int previous = from;
            for (const CsgJunction& cut : cuts)
            {
                if (cut.id_ != previous)
                    registerEdge(face, previous, cut.id_);
                previous = cut.id_;
            }
            if (previous != to)
                registerEdge(face, previous, to);
        }
    }

    // A face smaller than epsilon welds to less than three ids.
    if (collapsed)
        joinCollapsed();

    // Solids that touch along an edge pinch it: four or more faces with balanced winding still bound a volume,
    // so ray parity remains valid. Unbalanced or odd edges are cracks, tolerated up to a budget.
    // The edges are in insertion order, so the sums do not depend on the hash table.
    double crackLength = 0.0;
    double totalLength = 0.0;
    bool hasCrack = false;
    for (const auto& entry : edges)
    {
        const Vector3& from = loops.welder_.GetPosition(static_cast<int>(entry.key_ >> 32));
        const Vector3& to = loops.welder_.GetPosition(static_cast<int>(static_cast<uint32_t>(entry.key_)));
        const double length = (from - to).Length();
        totalLength += length;
        if ((entry.value_.count_ & 1) != 0 || entry.value_.winding_ != 0)
        {
            crackLength += length;
            hasCrack = true;
        }
    }
    // Non-finite lengths come from non-finite input. Such a surface is not closed.
    const double budget = ea::max(static_cast<double>(epsilon * CRACK_BUDGET_EPSILONS),
        totalLength * static_cast<double>(CRACK_BUDGET_FRACTION));
    result.closed &= std::isfinite(crackLength) && std::isfinite(totalLength) && crackLength <= budget;
    if (hasCrack)
    {
        for (const auto& entry : edges)
        {
            if ((entry.value_.count_ & 1) != 0 || entry.value_.winding_ != 0)
            {
                result.openEdges.push_back({loops.welder_.GetPosition(static_cast<int>(entry.key_ >> 32)),
                    loops.welder_.GetPosition(static_cast<int>(static_cast<uint32_t>(entry.key_)))});
            }
        }
    }

    GroupComponents(faceCount, sets, result.componentStart, result.componentFaces, result.faceComponents);
}

/// Join the faces from one face on that share an edge, as the links of the raw edges do for all faces.
/// Return false if a face with triangles welds to less than three ids, or if an undirected edge has a winding
/// that is not zero. Then the loops need the full topology build.
/// The edges are sorted into arrays by their lower id instead of a hash map. A boolean result needs its edges
/// for nothing else.
bool LinkRegion(const CsgWeldedLoops& loops, ea::span<const unsigned> triangleStart, unsigned first, FaceSets& sets)
{
    const unsigned idCount = loops.welder_.Count();
    const unsigned faceEnd = loops.loopLength_.size();
    ea::vector<int> bucketStart(idCount + 1, 0);
    unsigned edgeCount = 0;
    for (unsigned face = first; face < faceEnd; ++face)
    {
        const unsigned count = loops.loopLength_[face];
        if (count == 0)
        {
            if (triangleStart[face] != triangleStart[face + 1])
                return false;
            continue;
        }
        const unsigned start = loops.vertexStart_[face];
        for (unsigned i = 0; i < count; ++i)
        {
            const int from = loops.vertexIds_[start + i];
            const int to = loops.vertexIds_[start + (i + 1 == count ? 0 : i + 1)];
            ++bucketStart[ea::min(from, to) + 1];
        }
        edgeCount += count;
    }
    for (unsigned id = 0; id < idCount; ++id)
        bucketStart[id + 1] += bucketStart[id];

    // Each edge keeps its higher id and its face. The face is complemented when the edge runs downward.
    ea::vector<int> highs(edgeCount);
    ea::vector<int> owners(edgeCount);
    ea::vector<int> fill(bucketStart.begin(), bucketStart.end() - 1);
    for (unsigned face = first; face < faceEnd; ++face)
    {
        const unsigned count = loops.loopLength_[face];
        const unsigned start = loops.vertexStart_[face];
        for (unsigned i = 0; i < count; ++i)
        {
            const int from = loops.vertexIds_[start + i];
            const int to = loops.vertexIds_[start + (i + 1 == count ? 0 : i + 1)];
            const int slot = from < to ? fill[from]++ : fill[to]++;
            highs[slot] = ea::max(from, to);
            owners[slot] = from < to ? static_cast<int>(face - first) : ~static_cast<int>(face - first);
        }
    }

    // Within a bucket, the first edge to each higher id collects the winding of the edges that repeat it.
    const auto faceOf = [&](int owner) { return owner < 0 ? ~owner : owner; };
    ea::vector<int> firstEdge(idCount, 0);
    ea::vector<int> winding(edgeCount, 0);
    for (unsigned low = 0; low < idCount; ++low)
    {
        const int begin = bucketStart[low];
        const int end = bucketStart[low + 1];
        for (int edge = begin; edge < end; ++edge)
        {
            const int high = highs[edge];
            int group = firstEdge[high];
            if (group < begin || group >= edge || highs[group] != high)
                firstEdge[high] = group = edge;
            else
                sets.Union(faceOf(owners[edge]), faceOf(owners[group]));
            winding[group] += owners[edge] < 0 ? -1 : 1;
        }
        for (int edge = begin; edge < end; ++edge)
        {
            if (winding[edge] != 0)
                return false;
        }
    }
    return true;
}

} // namespace

CsgPointWelder::CsgPointWelder(float epsilon, unsigned capacity)
    : epsilon_(epsilon)
    , inverseCell_(epsilon > 0.0f ? 1.0 / (4.0 * static_cast<double>(epsilon)) : 0.0)
{
    positions_.reserve(capacity);
    if (epsilon > 0.0f)
    {
        next_.reserve(capacity);
        cells_.Reserve(capacity);
    }
    else
        exact_.Reserve(capacity);
}

CsgPointWelder::CsgPointWelder(ea::vector<Vector3>&& positions)
    : epsilon_(0.0f)
    , inverseCell_(0.0)
    , positions_(ea::move(positions))
    , exactIndexed_(false)
{
}

void CsgPointWelder::IndexExact() const
{
    if (exactIndexed_)
        return;
    exactIndexed_ = true;
    exact_.Reserve(positions_.size());
    for (unsigned i = 0; i < positions_.size(); ++i)
        exact_.TryAdd(CsgVec3Key(positions_[i]), static_cast<int>(i));
}

int CsgPointWelder::Add(const Vector3& point)
{
    const int index = static_cast<int>(positions_.size());
    if (epsilon_ <= 0.0f)
    {
        IndexExact();
        bool existed = false;
        int& id = exact_.FindOrAdd(CsgVec3Key(point), existed);
        if (!existed)
        {
            id = index;
            positions_.push_back(point);
        }
        return id;
    }

    const int found = Find(point);
    if (found >= 0)
        return found;

    positions_.push_back(point);
    bool existed = false;
    const uint64_t key = CellKey(CellCoord(point.x_ * inverseCell_), CellCoord(point.y_ * inverseCell_),
        CellCoord(point.z_ * inverseCell_));
    int& head = cells_.FindOrAdd(key, existed);
    next_.push_back(existed ? head : -1);
    head = index;
    return index;
}

int CsgPointWelder::Find(const Vector3& point) const
{
    if (epsilon_ <= 0.0f)
    {
        IndexExact();
        const int* id = exact_.Find(CsgVec3Key(point));
        return id ? *id : -1;
    }

    const double x = point.x_ * inverseCell_;
    const double y = point.y_ * inverseCell_;
    const double z = point.z_ * inverseCell_;
    const int64_t cx = CellCoord(x);
    const int64_t cy = CellCoord(y);
    const int64_t cz = CellCoord(z);
    const double margin = static_cast<double>(epsilon_) * inverseCell_;

    const int64_t x0 = x - static_cast<double>(cx) <= margin ? cx - 1 : cx;
    const int64_t x1 = static_cast<double>(cx) + 1.0 - x <= margin ? cx + 1 : cx;
    const int64_t y0 = y - static_cast<double>(cy) <= margin ? cy - 1 : cy;
    const int64_t y1 = static_cast<double>(cy) + 1.0 - y <= margin ? cy + 1 : cy;
    const int64_t z0 = z - static_cast<double>(cz) <= margin ? cz - 1 : cz;
    const int64_t z1 = static_cast<double>(cz) + 1.0 - z <= margin ? cz + 1 : cz;

    int best = std::numeric_limits<int>::max();
    const float epsilonSquared = epsilon_ * epsilon_;
    for (int64_t ix = x0; ix <= x1; ++ix)
    {
        for (int64_t iy = y0; iy <= y1; ++iy)
        {
            for (int64_t iz = z0; iz <= z1; ++iz)
            {
                const int* head = cells_.Find(CellKey(ix, iy, iz));
                if (!head)
                    continue;
                for (int id = *head; id >= 0; id = next_[id])
                {
                    if (id < best && (positions_[id] - point).LengthSquared() <= epsilonSquared)
                        best = id;
                }
            }
        }
    }
    return best == std::numeric_limits<int>::max() ? -1 : best;
}

struct CsgSolid::TriangleOutput
{
    ea::vector<unsigned>& triangleStart_;
    ea::vector<Vector3>& trianglePositions_;
    ea::vector<unsigned>& triangleVertices_;
    ea::vector<Vector3>& faceMin_;
    ea::vector<Vector3>& faceMax_;
    ea::vector<unsigned>& fallbackFaces_;
    ea::vector<Vector3>* corners_{};
    ea::vector<unsigned>* vertexStart_{};
};

namespace
{

/// Count the edges that have no edge in the reverse direction, by position. Two corners are one point only if
/// their positions are equal, so a crack of any width counts. A sheet of two overlapping faces in one direction
/// also counts. The ids of the exact loops do not show these defects: two ids can have one position.
/// The count is the sum of two counts: one for the edges of the triangles, and one for the edges of the face loops.
/// The loops count too, because a face can keep stored triangles that do not follow its loop.
/// The key of an edge is the bits of its two end positions, so one lookup is sufficient for each edge.
class PositionEdgeCounter
{
public:
    explicit PositionEdgeCounter(size_t edges) { balance_.Reserve(edges); }

    void AddEdge(const Vector3& from, const Vector3& to)
    {
        const Point a = ToPoint(from);
        const Point b = ToPoint(to);
        if (a == b)
            return;
        const bool forward = a < b;
        bool existed = false;
        balance_.FindOrAdd(forward ? EdgeKey{a, b} : EdgeKey{b, a}, existed) += forward ? 1 : -1;
    }

    /// Return the count of unbalanced edges and clear the edges.
    unsigned TakeOpen()
    {
        unsigned open = 0;
        for (const auto& entry : balance_)
            open += entry.value_ != 0 ? 1u : 0u;
        balance_.Clear();
        return open;
    }

private:
    struct Point
    {
        uint32_t x_, y_, z_;

        bool operator==(const Point& rhs) const { return x_ == rhs.x_ && y_ == rhs.y_ && z_ == rhs.z_; }
        bool operator<(const Point& rhs) const
        {
            if (x_ != rhs.x_)
                return x_ < rhs.x_;
            return y_ != rhs.y_ ? y_ < rhs.y_ : z_ < rhs.z_;
        }
    };

    struct EdgeKey
    {
        Point from_;
        Point to_;

        bool operator==(const EdgeKey& rhs) const { return from_ == rhs.from_ && to_ == rhs.to_; }
    };

    struct EdgeKeyHash
    {
        size_t operator()(const EdgeKey& key) const noexcept
        {
            const CsgPackedKeyHash mix;
            uint64_t h = mix((static_cast<uint64_t>(key.from_.y_) << 32) | key.from_.x_);
            h ^= mix((static_cast<uint64_t>(key.to_.x_) << 32) | key.from_.z_);
            h ^= (static_cast<uint64_t>(key.to_.z_) << 32 | key.to_.y_) * 0x9e3779b97f4a7c15ULL;
            return mix(h);
        }
    };

    static Point ToPoint(const Vector3& position) { return {Bits(position.x_), Bits(position.y_), Bits(position.z_)}; }

    static uint32_t Bits(float value)
    {
        // The sum turns -0 into +0, so the two zeros are one key.
        value += 0.0f;
        uint32_t result;
        std::memcpy(&result, &value, sizeof(result));
        return result;
    }

    CsgFlatMap<EdgeKey, int, EdgeKeyHash> balance_;
};

/// Two hashes of a position, for the fast test of the balance. See IsBalanced.
struct PointHash
{
    uint64_t f_;
    uint64_t g_;

    explicit PointHash(const Vector3& position)
    {
        const CsgPackedKeyHash mix;
        uint32_t bits[3];
        for (unsigned k = 0; k < 3; ++k)
        {
            // The sum turns -0 into +0, so the two zeros are one key.
            const float value = position.Data()[k] + 0.0f;
            std::memcpy(&bits[k], &value, sizeof(float));
        }
        f_ = mix((static_cast<uint64_t>(bits[1]) << 32) | bits[0]);
        f_ = mix(f_ ^ (bits[2] * 0x9e3779b97f4a7c15ULL));
        g_ = mix(f_ + 0x632be59bd9b4e019ULL);
    }
};

/// Return the term of a directed edge. The term of the reverse edge is the negative value, so an edge and its
/// reverse edge cancel. The arithmetic is modulo 2^64.
uint64_t EdgeTerm(const PointHash& from, const PointHash& to) { return from.f_ * to.g_ - to.f_ * from.g_; }

/// Return true if the sums of the edge terms of the triangles and of the face loops are zero. If all edges are
/// balanced, the sums are zero. Unbalanced edges give a zero sum only by a hash collision, which is very rare.
/// The test does not use a map, so it is much faster than the full count.
bool IsBalanced(ea::span<const Vector3> trianglePositions, ea::span<const CsgSharedPolygon> faces)
{
    uint64_t triangleSum = 0;
    for (size_t t = 0; t + 2 < trianglePositions.size(); t += 3)
    {
        const PointHash a(trianglePositions[t]);
        const PointHash b(trianglePositions[t + 1]);
        const PointHash c(trianglePositions[t + 2]);
        triangleSum += EdgeTerm(a, b) + EdgeTerm(b, c) + EdgeTerm(c, a);
    }
    uint64_t loopSum = 0;
    for (const CsgSharedPolygon& face : faces)
    {
        const auto& vertices = face->vertices_;
        if (vertices.size() < 3)
            continue;
        const PointHash first(vertices[0].GetPosition());
        PointHash from = first;
        for (unsigned i = 1; i < vertices.size(); ++i)
        {
            const PointHash to(vertices[i].GetPosition());
            loopSum += EdgeTerm(from, to);
            from = to;
        }
        loopSum += EdgeTerm(from, first);
    }
    return triangleSum == 0 && loopSum == 0;
}

unsigned CountOpenPositionEdges(ea::span<const Vector3> trianglePositions, ea::span<const CsgSharedPolygon> faces)
{
    // Most solids are closed. The fast test finds them, and only the other solids get the full count.
    if (IsBalanced(trianglePositions, faces))
        return 0;
    PositionEdgeCounter counter(trianglePositions.size() / 2 + 1);
    for (size_t t = 0; t + 2 < trianglePositions.size(); t += 3)
    {
        counter.AddEdge(trianglePositions[t], trianglePositions[t + 1]);
        counter.AddEdge(trianglePositions[t + 1], trianglePositions[t + 2]);
        counter.AddEdge(trianglePositions[t + 2], trianglePositions[t]);
    }
    unsigned open = counter.TakeOpen();
    for (const CsgSharedPolygon& face : faces)
    {
        const auto& vertices = face->vertices_;
        if (vertices.size() < 3)
            continue;
        for (unsigned i = 0; i < vertices.size(); ++i)
            counter.AddEdge(vertices[i].GetPosition(), vertices[(i + 1) % vertices.size()].GetPosition());
    }
    return open + counter.TakeOpen();
}

} // namespace

CsgSolid::CsgSolid(ea::span<const CsgPolygon> polygons, float epsilon)
{
    URHO3D_PROFILE("CsgSolid::CsgSolid");
    ea::vector<CsgSharedPolygon> faces;
    faces.reserve(polygons.size());
    for (const CsgPolygon& polygon : polygons)
        faces.push_back(ea::make_shared<CsgPolygon>(polygon));
    // The fast entry: weld, snap and one build. It is enough if the snapped faces close up exactly.
    ea::vector<CsgGridPoint> grid;
    {
        ea::vector<CsgSharedPolygon> snapped;
        PresnappedFaces(faces, epsilon <= 0.0f ? CSG_DEFAULT_EPSILON : epsilon, snapped, grid);
        Build(ea::move(snapped), epsilon, {}, {}, 0, grid);
        if (IsCleanOperand())
            return;
        grid.clear();
    }
    // Otherwise the faces are welded and repaired in float first, so a T-junction vertex is on its edge
    // before the snap.
    Build(ea::move(faces), epsilon);
    // Corefinement needs the neighboring faces to share their edge vertices at identical positions.
    if (NeedsRepair())
    {
        ea::vector<CsgSharedPolygon> repaired = WatertightFaces(grid);
        Build(ea::move(repaired), epsilon_, {}, {}, 0, grid);
    }
    // The only snap of the operand: after it, every position is grid / scale.
    ea::vector<CsgSharedPolygon> snapped;
    if (SnappedFaces(snapped, grid))
        Build(ea::move(snapped), epsilon_, {}, {}, 0, grid);
    if (NeedsRepair())
    {
        ea::vector<CsgSharedPolygon> repaired = WatertightFaces(grid);
        Build(ea::move(repaired), epsilon_, {}, {}, 0, grid);
    }
}

CsgSolid::CsgSolid(ea::vector<CsgSharedPolygon>&& faces, float epsilon, ea::span<const CsgGridPoint> grid)
{
    URHO3D_PROFILE("CsgSolid::CsgSolid");
    Build(ea::move(faces), epsilon, {}, {}, 0, grid);
}

CsgSolid::CsgSolid(ea::vector<CsgSharedPolygon>&& faces, float epsilon, ea::span<const CsgFaceOrigin> origins,
    ea::span<const int> exactIds, int exactIdSpace, ea::span<const CsgGridPoint> exactGrid)
{
    URHO3D_PROFILE("CsgSolid::CsgSolid");
    Build(ea::move(faces), epsilon, origins, exactIds, exactIdSpace, exactGrid);
}

CsgSolid::CsgSolid(Builder&& builder)
    : faces_(ea::move(builder.faces_))
    , triangleStart_(ea::move(builder.triangleStart_))
    , trianglePositions_(ea::move(builder.trianglePositions_))
    , triangleVertices_(ea::move(builder.triangleVertices_))
    , faceMin_(ea::move(builder.faceMin_))
    , faceMax_(ea::move(builder.faceMax_))
    , componentStart_(ea::move(builder.componentStart_))
    , componentFaces_(ea::move(builder.componentFaces_))
    , faceComponents_(ea::move(builder.faceComponents_))
    , fallbackFaces_(ea::move(builder.fallbackFaces_))
    , epsilon_(builder.epsilon_ > 0.0f ? builder.epsilon_ : CSG_DEFAULT_EPSILON)
    , closed_(builder.closed_)
    , outOfRange_(builder.outOfRange_)
{
    URHO3D_PROFILE("CsgSolid::CsgSolid");
    // The topology is copied. Only the exact loops are built, and the edges of the loops only when someone needs them.
    ea::vector<Vector3> corners;
    ea::vector<unsigned> vertexStart;
    vertexStart.reserve(faces_.size() + 1);
    for (const CsgSharedPolygon& face : faces_)
    {
        vertexStart.push_back(corners.size());
        for (const ModelVertex& vertex : face->vertices_)
            corners.push_back(vertex.GetPosition());
    }
    vertexStart.push_back(corners.size());
    exact_ = BuildExactLoops(
        corners, vertexStart, triangleStart_, builder.exactIds_, builder.exactIdSpace_, builder.grid_);
    if (builder.regionFaces_ != 0)
    {
        outOfRange_ = outOfRange_ || exact_->outOfRange_;
        GroupRegion(builder, corners, vertexStart);
    }
    BuildBounds();
}

void CsgSolid::GroupRegion(const Builder& builder, ea::span<const Vector3> corners, ea::span<const unsigned> vertexStart)
{
    URHO3D_PROFILE("CsgSolid::GroupRegion");
    const unsigned first = builder.regionFace_;
    const unsigned faceCount = builder.regionFaces_;
    FaceSets sets(faceCount);
    ea::vector<unsigned> componentStart;
    ea::vector<unsigned> componentFaces;
    ea::vector<unsigned> faceComponents;
    if (LinkRegion(*exact_, triangleStart_, first, sets))
    {
        GroupComponents(faceCount, sets, componentStart, componentFaces, faceComponents);
        // A face without a surface is a hole.
        for (unsigned face = first; face < first + faceCount; ++face)
        {
            if (vertexStart[face + 1] - vertexStart[face] < 3 || triangleStart_[face] == triangleStart_[face + 1])
                closed_ = false;
        }
    }
    else
    {
        // A crack or a collapsed face: the topology build of the region faces alone decides.
        const unsigned cornerBase = vertexStart[first];
        ea::vector<unsigned> regionVertexStart;
        regionVertexStart.reserve(faceCount + 1);
        for (unsigned face = first; face <= first + faceCount; ++face)
            regionVertexStart.push_back(vertexStart[face] - cornerBase);
        ea::vector<unsigned> regionTriangleStart;
        regionTriangleStart.reserve(faceCount + 1);
        for (unsigned face = first; face <= first + faceCount; ++face)
            regionTriangleStart.push_back(triangleStart_[face] - triangleStart_[first]);
        ea::vector<int> regionIds;
        regionIds.reserve(builder.exactIds_.size() - cornerBase);
        for (unsigned i = cornerBase; i < builder.exactIds_.size(); ++i)
            regionIds.push_back(builder.exactIds_[i] - builder.regionIdBase_);

        Topology topology;
        BuildTopology(corners.subspan(cornerBase), regionVertexStart, regionTriangleStart, epsilon_, regionIds,
            builder.regionIdSpace_, ea::span<const CsgGridPoint>(builder.grid_).subspan(cornerBase), topology);
        closed_ = closed_ && topology.closed;
        componentStart = ea::move(topology.componentStart);
        componentFaces = ea::move(topology.componentFaces);
        faceComponents = ea::move(topology.faceComponents);
    }

    const unsigned componentBase = componentStart_.size() - 1;
    for (const unsigned component : faceComponents)
        faceComponents_.push_back(componentBase + component);
    for (unsigned component = 0; component + 1 < componentStart.size(); ++component)
    {
        for (unsigned c = componentStart[component]; c < componentStart[component + 1]; ++c)
            componentFaces_.push_back(first + componentFaces[c]);
        componentStart_.push_back(componentFaces_.size());
    }
}

SharedPtr<CsgSolid> CsgSolid::FromKeptFaces(ea::vector<CsgSharedPolygon>&& faces, float epsilon,
    ea::span<const CsgFaceOrigin> origins, ea::span<const int> exactIds, int exactIdSpace,
    ea::span<const CsgGridPoint> exactGrid)
{
    return SharedPtr<CsgSolid>(new CsgSolid(ea::move(faces), epsilon, origins, exactIds, exactIdSpace, exactGrid));
}

CsgSharedPolygon CsgSolid::Flipped(const CsgPolygon& face)
{
    auto result = ea::make_shared<CsgPolygon>(face);
    result->Flip();
    return result;
}

bool CsgSolid::Keep(bool inside, CsgOperation operation, bool secondOperand)
{
    if (operation == CsgOperation::Intersection)
        return inside;
    return secondOperand && operation == CsgOperation::DifferenceAB ? inside : !inside;
}

CsgExactLoops CsgSolid::GetExactLoops() const
{
    const CsgWeldedLoops& exact = *exact_;
    CsgExactLoops result;
    result.vertexStart_ = {exact.vertexStart_.data(), exact.vertexStart_.size()};
    result.ids_ = {exact.ids_.data(), exact.ids_.size()};
    result.vertexIds_ = {exact.vertexIds_.data(), exact.vertexIds_.size()};
    result.loopLength_ = {exact.loopLength_.data(), exact.loopLength_.size()};
    result.positions_ = exact.welder_.GetPositions();
    result.grid_ = {exact.grid_.data(), exact.grid_.size()};
    return result;
}

int CsgSolid::VertexKey(unsigned face, unsigned vertex) const
{
    return exact_->ids_[exact_->vertexStart_[face] + vertex];
}

unsigned CsgSolid::VertexKeySpace() const
{
    return exact_->welder_.Count();
}

bool CsgSolid::IsFallbackFace(unsigned face) const
{
    return ea::binary_search(fallbackFaces_.begin(), fallbackFaces_.end(), face);
}

bool CsgSolid::CopyTriangles(unsigned face, ea::vector<unsigned>& output) const
{
    if (IsFallbackFace(face))
    {
        if (!CsgTriangulatePolygon(*faces_[face], output))
            return false;
        // The fresh triangles can be flat on a straight run of the boundary also, so the flat ones turn.
        TurnFlatTriangles(faces_[face]->vertices_, output);
        return true;
    }
    output.clear();
    const unsigned begin = triangleStart_[face] * 3;
    const unsigned end = triangleStart_[face + 1] * 3;
    output.assign(triangleVertices_.begin() + begin, triangleVertices_.begin() + end);
    return !output.empty();
}

CsgSolid::Builder::Builder(unsigned faceCapacity, unsigned cornerCapacity, float epsilon)
    : epsilon_(epsilon)
{
    faces_.reserve(faceCapacity);
    triangleStart_.reserve(faceCapacity + 1);
    triangleStart_.push_back(0);
    trianglePositions_.reserve(cornerCapacity);
    triangleVertices_.reserve(cornerCapacity);
    faceMin_.reserve(faceCapacity);
    faceMax_.reserve(faceCapacity);
    componentStart_.push_back(0);
    componentFaces_.reserve(faceCapacity);
    faceComponents_.reserve(faceCapacity);
    exactIds_.reserve(cornerCapacity);
    grid_.reserve(cornerCapacity);
}

unsigned CsgSolid::Builder::IdOffset(const CsgSolid& source)
{
    for (const auto& entry : idOffsets_)
    {
        if (entry.first == &source)
            return entry.second;
    }
    const int offset = exactIdSpace_;
    idOffsets_.push_back({&source, offset});
    exactIdSpace_ += static_cast<int>(source.VertexKeySpace());
    return offset;
}

void CsgSolid::Builder::AppendComponent(const CsgSolid& source, unsigned component, bool flip)
{
    const unsigned componentIndex = componentStart_.size() - 1;
    for (unsigned c = source.componentStart_[component]; c < source.componentStart_[component + 1]; ++c)
    {
        componentFaces_.push_back(faces_.size());
        AppendFace(source, source.componentFaces_[c], flip, componentIndex);
    }
    componentStart_.push_back(componentFaces_.size());
}

void CsgSolid::Builder::AppendSolid(const CsgSolid& source)
{
    const unsigned faceBase = faces_.size();
    const unsigned componentBase = componentStart_.size() - 1;
    for (unsigned face = 0; face < source.faces_.size(); ++face)
        AppendFace(source, face, false, componentBase + source.faceComponents_[face]);
    for (unsigned component = 0; component < source.ComponentCount(); ++component)
    {
        for (unsigned c = source.componentStart_[component]; c < source.componentStart_[component + 1]; ++c)
            componentFaces_.push_back(faceBase + source.componentFaces_[c]);
        componentStart_.push_back(componentFaces_.size());
    }
    closed_ = closed_ && source.IsClosed();
}

void CsgSolid::Builder::AppendFace(const CsgSolid& source, unsigned face, bool flip, unsigned component)
{
    if (source.IsFallbackFace(face))
        fallbackFaces_.push_back(faces_.size());
    faces_.push_back(flip ? Flipped(*source.faces_[face]) : source.faces_[face]);
    faceComponents_.push_back(component);

    const int offset = IdOffset(source);
    const CsgWeldedLoops& weld = *source.exact_;
    const unsigned idStart = weld.vertexStart_[face];
    const unsigned idCount = weld.vertexStart_[face + 1] - idStart;
    for (unsigned i = 0; i < idCount; ++i)
    {
        const int id = weld.ids_[idStart + (flip ? idCount - 1 - i : i)];
        exactIds_.push_back(offset + id);
        grid_.push_back(weld.grid_[id]);
    }
    outOfRange_ = outOfRange_ || source.outOfRange_;
    faceMin_.push_back(source.faceMin_[face]);
    faceMax_.push_back(source.faceMax_[face]);

    const unsigned vertexCount = source.faces_[face]->vertices_.size();
    const unsigned first = source.triangleStart_[face];
    const unsigned count = source.triangleStart_[face + 1] - first;
    const Vector3* positions = source.trianglePositions_.data() + first * 3;
    const unsigned* locals = source.triangleVertices_.data() + first * 3;
    if (flip)
    {
        // Flipping reverses the vertex list. Keep the same corners under their new indices and reverse
        // each triangle, so that it winds with the flipped face.
        for (unsigned t = 0; t < count; ++t)
        {
            for (int corner = 2; corner >= 0; --corner)
            {
                trianglePositions_.push_back(positions[t * 3 + corner]);
                triangleVertices_.push_back(vertexCount - 1 - locals[t * 3 + corner]);
            }
        }
    }
    else
    {
        trianglePositions_.insert(trianglePositions_.end(), positions, positions + count * 3);
        triangleVertices_.insert(triangleVertices_.end(), locals, locals + count * 3);
    }
    triangleStart_.push_back(triangleStart_.back() + count);
}

void CsgSolid::Builder::AppendRegion(ea::vector<CsgSharedPolygon>&& faces, ea::span<const CsgFaceOrigin> origins,
    ea::span<const int> exactIds, int exactIdSpace, ea::span<const CsgGridPoint> exactGrid)
{
    URHO3D_ASSERT(regionFaces_ == 0, "CsgSolid::Builder takes one region");
    URHO3D_ASSERT(origins.empty() || origins.size() == faces.size(), "CsgSolid origins must have one entry for each face");
    if (faces.empty())
        return;
    regionFace_ = faces_.size();
    regionFaces_ = faces.size();
    regionIdBase_ = exactIdSpace_;
    regionIdSpace_ = exactIdSpace;
    exactIdSpace_ += exactIdSpace;

    TriangleOutput output{triangleStart_, trianglePositions_, triangleVertices_, faceMin_, faceMax_, fallbackFaces_};
    Triangulate(faces, origins, regionFace_, output);
    faces_.insert(faces_.end(), ea::make_move_iterator(faces.begin()), ea::make_move_iterator(faces.end()));
    // The ids of the region start after the ids of the sources before it.
    for (const int id : exactIds)
        exactIds_.push_back(regionIdBase_ + id);
    grid_.insert(grid_.end(), exactGrid.begin(), exactGrid.end());
    CancelTriangles();
}

/// Cancel the triangles of the region that are on the same three points. Two triangles in opposite directions are
/// a sheet with no volume: the two go away. Two in one direction stay: each has its own neighbors, and one alone
/// leaves open edges. A triangle with two corners on one point has no area, and its other two edges cancel: it
/// goes away. A face that loses all its triangles goes away.
///
/// The points are the exact ids of the corners. The loops of the faces do not change, so a face that loses only
/// some of its triangles would leave the edges of the faces that lost all of theirs without a partner. Then the
/// triangles of the three points stay in all faces. A face that loses a triangle keeps its stored triangles.
///
/// The function keeps the faces, the bounds, the fallback faces, the exact ids, the grid points and the
/// triangles of the region in step.
void CsgSolid::Builder::CancelTriangles()
{
    const unsigned firstFace = regionFace_;
    const unsigned endFace = regionFace_ + regionFaces_;
    URHO3D_ASSERT(endFace == faces_.size() && triangleStart_.size() == faces_.size() + 1);
    const unsigned firstTriangle = triangleStart_[firstFace];
    const unsigned endTriangle = trianglePositions_.size() / 3;
    const ea::span<const Vector3> positions(
        trianglePositions_.data() + firstTriangle * 3, (endTriangle - firstTriangle) * 3);
    // A position outside the range of the grid has no grid point. Such a region keeps its triangles.
    for (const Vector3& position : positions)
    {
        if (!InGridRange(position))
            return;
    }
    if (!CanCancel(positions))
        return;

    // The corners of the region faces own the exact ids and the grid points. A face keeps the corners of its own.
    unsigned cornerBase = 0;
    for (unsigned face = 0; face < firstFace; ++face)
        cornerBase += faces_[face]->vertices_.size();
    unsigned regionCorners = 0;
    for (unsigned face = firstFace; face < endFace; ++face)
        regionCorners += faces_[face]->vertices_.size();
    const bool hasIds = exactIds_.size() == cornerBase + regionCorners;
    const bool hasGrid = grid_.size() == cornerBase + regionCorners;

    // Two triangles cancel only if they have the same exact ids: then their edges cancel as well. A region without
    // ids uses the grid points of the positions.
    struct TriangleInfo
    {
        TriangleKey key_;
        int direction_{};
        bool degenerate_{};
    };
    const unsigned triangleCount = endTriangle - firstTriangle;
    ea::vector<TriangleInfo> info(triangleCount);
    ea::hash_map<TriangleKey, int, TriangleKeyHash> net;
    bool any = false;
    {
        unsigned cornerStart = cornerBase;
        for (unsigned face = firstFace; face < endFace; ++face)
        {
            for (unsigned t = triangleStart_[face]; t < triangleStart_[face + 1]; ++t)
            {
                CsgGridPoint points[3];
                for (unsigned corner = 0; corner < 3; ++corner)
                {
                    if (hasIds)
                        points[corner].x_ = exactIds_[cornerStart + triangleVertices_[t * 3 + corner]];
                    else
                        points[corner] = CsgGridPoint::From(trianglePositions_[t * 3 + corner]);
                }
                TriangleInfo& triangle = info[t - firstTriangle];
                triangle.degenerate_ = !MakeTriangleKey(
                    points[0], points[1], points[2], triangle.key_, triangle.direction_);
                if (triangle.degenerate_)
                {
                    any = true;
                    continue;
                }
                const auto result = net.insert(triangle.key_);
                result.first->second += triangle.direction_;
                any |= !result.second;
            }
            cornerStart += faces_[face]->vertices_.size();
        }
    }
    if (!any)
        return;

    // Of the triangles on three points, the ones that have no opposite partner stay. A face that would lose some of
    // its triangles but not all of them vetoes the three points of the lost ones, and the pass runs again.
    // The thick mode does not make the veto unnecessary. Without the veto, a face loses only a part of its triangles.
    // Then the thick hole tests and the rotated sweep samples in TestBoolean.cpp give open results.
    ea::vector<bool> drop(triangleCount);
    ea::hash_set<TriangleKey, TriangleKeyHash> vetoed;
    for (bool changed = true; changed;)
    {
        changed = false;
        auto working = net;
        for (unsigned t = 0; t < triangleCount; ++t)
        {
            const TriangleInfo& triangle = info[t];
            if (triangle.degenerate_)
            {
                drop[t] = true;
                continue;
            }
            drop[t] = false;
            if (vetoed.find(triangle.key_) != vetoed.end())
                continue;
            int& sum = working[triangle.key_];
            if (sum != 0 && (sum > 0) == (triangle.direction_ > 0))
                sum -= triangle.direction_;
            else
                drop[t] = true;
        }
        for (unsigned face = firstFace; face < endFace; ++face)
        {
            unsigned lost = 0;
            unsigned kept = 0;
            for (unsigned t = triangleStart_[face]; t < triangleStart_[face + 1]; ++t)
            {
                if (!info[t - firstTriangle].degenerate_)
                    (drop[t - firstTriangle] ? lost : kept)++;
            }
            if (lost == 0 || kept == 0)
                continue;
            for (unsigned t = triangleStart_[face]; t < triangleStart_[face + 1]; ++t)
            {
                if (!info[t - firstTriangle].degenerate_ && drop[t - firstTriangle])
                    changed |= vetoed.insert(info[t - firstTriangle].key_).second;
            }
        }
    }
    bool anyDrop = false;
    for (unsigned t = 0; t < triangleCount; ++t)
        anyDrop |= drop[t];
    if (!anyDrop)
        return;

    // The fallback faces are in ascending order. The ones of the region are set again below.
    ea::vector<unsigned> regionFallback;
    const auto fallbackBegin = ea::lower_bound(fallbackFaces_.begin(), fallbackFaces_.end(), firstFace);
    regionFallback.assign(fallbackBegin, fallbackFaces_.end());
    fallbackFaces_.erase(fallbackBegin, fallbackFaces_.end());

    unsigned writeFace = firstFace;
    unsigned writeTriangle = firstTriangle;
    unsigned writeCorner = cornerBase;
    unsigned readTriangle = firstTriangle;
    unsigned readCorner = cornerBase;
    for (unsigned face = firstFace; face < endFace; ++face)
    {
        const unsigned end = triangleStart_[face + 1];
        const unsigned keptStart = writeTriangle;
        for (unsigned t = readTriangle; t < end; ++t)
        {
            if (drop[t - firstTriangle])
                continue;
            for (unsigned corner = 0; corner < 3; ++corner)
            {
                trianglePositions_[writeTriangle * 3 + corner] = trianglePositions_[t * 3 + corner];
                triangleVertices_[writeTriangle * 3 + corner] = triangleVertices_[t * 3 + corner];
            }
            ++writeTriangle;
        }
        // A face that loses no triangle keeps its fallback mark. A face that loses some keeps its stored triangles.
        const bool isFallback = ea::binary_search(regionFallback.begin(), regionFallback.end(), face)
            && writeTriangle - keptStart == end - readTriangle;
        const bool emptied = end > readTriangle && writeTriangle == keptStart;
        const unsigned cornerCount = faces_[face]->vertices_.size();
        readTriangle = end;
        const unsigned cornerStart = readCorner;
        readCorner += cornerCount;
        // A face that loses all its triangles goes away.
        if (emptied)
            continue;
        if (writeFace != face)
        {
            faces_[writeFace] = ea::move(faces_[face]);
            faceMin_[writeFace] = faceMin_[face];
            faceMax_[writeFace] = faceMax_[face];
        }
        if (isFallback)
            fallbackFaces_.push_back(writeFace);
        for (unsigned i = 0; i < cornerCount; ++i)
        {
            if (hasIds)
                exactIds_[writeCorner + i] = exactIds_[cornerStart + i];
            if (hasGrid)
                grid_[writeCorner + i] = grid_[cornerStart + i];
        }
        writeCorner += cornerCount;
        triangleStart_[++writeFace] = writeTriangle;
    }
    faces_.erase(faces_.begin() + writeFace, faces_.end());
    faceMin_.erase(faceMin_.begin() + writeFace, faceMin_.end());
    faceMax_.erase(faceMax_.begin() + writeFace, faceMax_.end());
    triangleStart_.erase(triangleStart_.begin() + writeFace + 1, triangleStart_.end());
    if (hasIds)
        exactIds_.erase(exactIds_.begin() + writeCorner, exactIds_.end());
    if (hasGrid)
        grid_.erase(grid_.begin() + writeCorner, grid_.end());
    trianglePositions_.erase(trianglePositions_.begin() + writeTriangle * 3, trianglePositions_.end());
    triangleVertices_.erase(triangleVertices_.begin() + writeTriangle * 3, triangleVertices_.end());
    regionFaces_ = writeFace - firstFace;
}

SharedPtr<CsgSolid> CsgSolid::Builder::Finish()
{
    return SharedPtr<CsgSolid>(new CsgSolid(ea::move(*this)));
}

CsgSolid::~CsgSolid() = default;

SharedPtr<CsgSolid> CsgSolid::OutOfRangeResult(float epsilon)
{
    Builder builder(0, 0, epsilon);
    builder.outOfRange_ = true;
    return builder.Finish();
}

SharedPtr<CsgSolid> CsgSolid::FromOperand(ea::vector<CsgPolygon>&& polygons, float epsilon)
{
    URHO3D_PROFILE("CsgSolid::FromOperand");
    ea::vector<CsgSharedPolygon> faces;
    faces.reserve(polygons.size());
    for (CsgPolygon& polygon : polygons)
        faces.push_back(ea::make_shared<CsgPolygon>(ea::move(polygon)));
    // The fast entry: weld, snap and one build. It is enough if the snapped faces close up exactly.
    {
        ea::vector<CsgSharedPolygon> snapped;
        ea::vector<CsgGridPoint> grid;
        PresnappedFaces(faces, epsilon <= 0.0f ? CSG_DEFAULT_EPSILON : epsilon, snapped, grid);
        SharedPtr<CsgSolid> solid(new CsgSolid(ea::move(snapped), epsilon, grid));
        if (solid->IsCleanOperand())
            return solid;
    }
    // Otherwise the corners are welded within epsilon and repaired in float first, so a welded vertex gets one
    // grid point, and a T-junction vertex is on its edge before the snap.
    SharedPtr<CsgSolid> solid = Repaired(SharedPtr<CsgSolid>(new CsgSolid(ea::move(faces), epsilon)));
    // The only snap of the operand: after it, every position is grid / scale.
    ea::vector<CsgSharedPolygon> snapped;
    ea::vector<CsgGridPoint> grid;
    if (!solid->SnappedFaces(snapped, grid))
        return solid;
    return Repaired(SharedPtr<CsgSolid>(new CsgSolid(ea::move(snapped), solid->epsilon_, grid)));
}

bool CsgSolid::SnappedFaces(ea::vector<CsgSharedPolygon>& faces, ea::vector<CsgGridPoint>& grid) const
{
    URHO3D_PROFILE("CsgSolid::SnappedFaces");
    const CsgWeldedLoops& exact = *exact_;
    const auto positions = exact.welder_.GetPositions();
    // A coordinate out of the range of the grid stays as it is. Its grid point is clamped.
    const auto snap = SnapPosition;
    // Each corner is compared, not only the position of its id: two corners of one id can differ.
    const auto moves = [&](unsigned face, unsigned i)
    {
        const Vector3& position = faces_[face]->vertices_[i].GetPosition();
        const int id = exact.ids_[exact.vertexStart_[face] + i];
        return snap(positions[id], exact.grid_[id]) != position;
    };
    bool any = false;
    for (unsigned face = 0; face < faces_.size() && !any; ++face)
    {
        for (unsigned i = 0; i < faces_[face]->vertices_.size() && !any; ++i)
            any = moves(face, i);
    }
    if (!any)
        return false;

    faces.clear();
    faces.reserve(faces_.size());
    grid.clear();
    grid.reserve(exact.ids_.size());
    for (unsigned face = 0; face < faces_.size(); ++face)
    {
        const CsgPolygon& source = *faces_[face];
        const unsigned start = exact.vertexStart_[face];
        bool changed = false;
        for (unsigned i = 0; i < source.vertices_.size() && !changed; ++i)
            changed = moves(face, i);
        for (unsigned i = 0; i < source.vertices_.size(); ++i)
            grid.push_back(exact.grid_[exact.ids_[start + i]]);
        if (!changed)
        {
            faces.push_back(faces_[face]);
            continue;
        }
        ea::vector<ModelVertex> vertices = source.vertices_;
        for (unsigned i = 0; i < source.vertices_.size(); ++i)
        {
            const int id = exact.ids_[start + i];
            vertices[i].SetPosition(snap(positions[id], exact.grid_[id]));
        }
        // The constructor computes the plane again from the snapped positions.
        faces.push_back(ea::make_shared<CsgPolygon>(ea::move(vertices), source.shared_));
    }
    return true;
}

SharedPtr<CsgSolid> CsgSolid::Repaired(const SharedPtr<CsgSolid>& solid)
{
    if (!solid->NeedsRepair())
        return solid;
    ea::vector<CsgGridPoint> grid;
    ea::vector<CsgSharedPolygon> faces = solid->WatertightFaces(grid);
    return SharedPtr<CsgSolid>(new CsgSolid(ea::move(faces), solid->epsilon_, grid));
}

bool CsgSolid::NeedsRepair() const
{
    return hasJunctions_ || !GetWeld().canonical_;
}

bool CsgSolid::IsCleanOperand() const
{
    return closed_ && openEdges_.empty() && !NeedsRepair();
}

unsigned CsgSolid::OpenPositionEdgeCount() const
{
    return CountOpenPositionEdges(trianglePositions_, faces_);
}

double CsgSolid::Volume() const
{
    double sum = 0.0;
    for (unsigned i = 0; i + 2 < trianglePositions_.size(); i += 3)
    {
        const Vector3& a = trianglePositions_[i];
        const Vector3& b = trianglePositions_[i + 1];
        const Vector3& c = trianglePositions_[i + 2];
        sum += (static_cast<double>(a.x_) * (static_cast<double>(b.y_) * c.z_ - static_cast<double>(b.z_) * c.y_)
                   + static_cast<double>(a.y_) * (static_cast<double>(b.z_) * c.x_ - static_cast<double>(b.x_) * c.z_)
                   + static_cast<double>(a.z_) * (static_cast<double>(b.x_) * c.y_ - static_cast<double>(b.y_) * c.x_))
            / 6.0;
    }
    return sum;
}

float CsgSolid::OpenEdgeLength() const
{
    double total = 0.0;
    for (const auto& edge : openEdges_)
        total += (edge.second - edge.first).Length();
    return static_cast<float>(total);
}

const CsgWeldedLoops& CsgSolid::GetWeld() const
{
    return welded_ ? *welded_ : *exact_;
}

ea::vector<CsgSharedPolygon> CsgSolid::WatertightFaces(ea::vector<CsgGridPoint>& grid) const
{
    URHO3D_PROFILE("CsgSolid::WatertightFaces");
    const CsgWeldedLoops& loops = GetWeld();
    const CsgWeldedLoops& exact = *exact_;
    grid.clear();
    CsgJunctionFinder& finder = loops.GetFinder();
    const CsgPointWelder& welder = loops.welder_;
    ea::vector<CsgSharedPolygon> result;
    result.reserve(faces_.size());
    ea::vector<CsgJunction> cuts;
    ea::vector<ModelVertex> buffer;

    // The welded loops start at the first corner of each face.
    unsigned start = 0;
    for (unsigned face = 0; face < faces_.size(); ++face)
    {
        const CsgPolygon& source = *faces_[face];
        const unsigned cornerStart = start;
        start += source.vertices_.size();

        const unsigned count = loops.loopLength_[face];
        if (count < 3)
            continue;
        bool changed = count != source.vertices_.size();
        buffer.clear();
        for (unsigned i = 0; i < count; ++i)
        {
            const int from = loops.vertexIds_[cornerStart + i];
            const unsigned cornerIndex = cornerStart + loops.corners_[cornerStart + i];
            ModelVertex corner = source.vertices_[loops.corners_[cornerStart + i]];
            const Vector3& position = welder.GetPosition(from);
            // A NaN position is never equal, so such a corner counts as changed. The result is the same corner.
            // A corner that moves takes the grid point of its welded id. A corner that stays keeps its own.
            if (corner.GetPosition() != position)
            {
                corner.SetPosition(position);
                changed = true;
                grid.push_back(loops.grid_[from]);
            }
            else
                grid.push_back(exact.grid_[exact.ids_[cornerIndex]]);
            buffer.push_back(corner);

            const int to = loops.vertexIds_[cornerStart + (i + 1) % count];
            if (IsMatched(loops, from, to))
                continue;
            finder.Collect(from, to, cuts);
            if (cuts.empty())
                continue;
            const ModelVertex& next = source.vertices_[loops.corners_[cornerStart + (i + 1) % count]];
            for (const CsgJunction& cut : cuts)
            {
                ModelVertex vertex = LerpModelVertex(corner, next, cut.t_);
                vertex.SetPosition(welder.GetPosition(cut.id_));
                buffer.push_back(vertex);
                grid.push_back(loops.grid_[cut.id_]);
            }
            changed = true;
        }
        if (!changed)
        {
            // The face keeps all its corners. The grid has the grid points of its exact ids already.
            result.push_back(faces_[face]);
            continue;
        }
        auto polygon = ea::make_shared<CsgPolygon>(ea::move(buffer), source.shared_);
        polygon->plane_ = source.plane_;
        result.push_back(ea::move(polygon));
    }
    return result;
}

bool CsgSolid::BoundsOverlap(const Vector3& min, const Vector3& max, float epsilon) const
{
    return boundsMin_.x_ <= max.x_ + epsilon && min.x_ <= boundsMax_.x_ + epsilon
        && boundsMin_.y_ <= max.y_ + epsilon && min.y_ <= boundsMax_.y_ + epsilon
        && boundsMin_.z_ <= max.z_ + epsilon && min.z_ <= boundsMax_.z_ + epsilon;
}

const CsgFaceBvh& CsgSolid::GetComponentTree() const
{
    return *componentTree_;
}

const CsgFaceBvh& CsgSolid::GetFaceTree(unsigned component) const
{
    FaceTreeSlot& slot = faceTrees_[component];
    std::call_once(slot.once_,
        [&]
        {
            slot.tree_ = ea::make_unique<CsgFaceBvh>(GetFaceMin(), GetFaceMax(),
                ea::span<const unsigned>(componentFaces_.data() + componentStart_[component],
                    componentStart_[component + 1] - componentStart_[component]));
            slot.built_.store(true, std::memory_order_release);
        });
    return *slot.tree_;
}

const CsgFaceBvh& CsgSolid::GetNearFaces(unsigned component, const Vector3& min, const Vector3& max, float epsilon,
    ea::unique_ptr<CsgFaceBvh>& temporary) const
{
    if (faceTrees_[component].built_.load(std::memory_order_acquire))
        return *faceTrees_[component].tree_;
    const unsigned begin = componentStart_[component];
    const unsigned faceCount = componentStart_[component + 1] - begin;
    const Vector3 low(min.x_ - epsilon, min.y_ - epsilon, min.z_ - epsilon);
    const Vector3 high(max.x_ + epsilon, max.y_ + epsilon, max.z_ + epsilon);
    ea::vector<unsigned> nearFaces;
    for (unsigned i = begin; i < begin + faceCount; ++i)
    {
        const unsigned face = componentFaces_[i];
        const Vector3& faceMin = faceMin_[face];
        const Vector3& faceMax = faceMax_[face];
        if (faceMin.x_ <= high.x_ && faceMin.y_ <= high.y_ && faceMin.z_ <= high.z_ && faceMax.x_ >= low.x_
            && faceMax.y_ >= low.y_ && faceMax.z_ >= low.z_)
            nearFaces.push_back(face);
    }
    if (nearFaces.size() * 2 > faceCount)
        return GetFaceTree(component);
    temporary = ea::make_unique<CsgFaceBvh>(GetFaceMin(), GetFaceMax(), ea::span<const unsigned>(nearFaces.data(), nearFaces.size()));
    return *temporary;
}

ea::vector<CsgPolygon> CsgSolid::ToPolygons() const
{
    ea::vector<CsgPolygon> result;
    result.reserve(faces_.size());
    for (const CsgSharedPolygon& face : faces_)
        result.push_back(*face);
    return result;
}

void CsgSolid::Triangulate(ea::span<const CsgSharedPolygon> faces, ea::span<const CsgFaceOrigin> origins,
    unsigned firstFace, TriangleOutput& output)
{
    URHO3D_PROFILE("CsgSolid::Triangulate");
    const unsigned faceCount = faces.size();

    // Two planar faces can share a run of several edges only along the line where their planes meet, so a
    // vertex whose edges both lead to the same neighbor is straight up to rounding. If both faces clipped it as
    // an ear, both would claim the sliver under the run and share a diagonal. Both faces of such a pair have
    // more than three vertices, so only those enter the list. The list is sorted by edge and by face.
    // For an edge in many faces, the last face is the owner.
    struct EdgeOwner
    {
        Vector3 from_;
        Vector3 to_;
        unsigned face_;
    };
    const auto hasNan = [](const Vector3& v) { return v.x_ != v.x_ || v.y_ != v.y_ || v.z_ != v.z_; };
    const auto less = [](const Vector3& a, const Vector3& b)
    {
        if (a.x_ != b.x_)
            return a.x_ < b.x_;
        if (a.y_ != b.y_)
            return a.y_ < b.y_;
        return a.z_ < b.z_;
    };
    const auto edgeLess = [&](const EdgeOwner& a, const EdgeOwner& b)
    {
        if (less(a.from_, b.from_) || less(b.from_, a.from_))
            return less(a.from_, b.from_);
        if (less(a.to_, b.to_) || less(b.to_, a.to_))
            return less(a.to_, b.to_);
        return a.face_ < b.face_;
    };
    ea::vector<EdgeOwner> edgeOwners;
    for (unsigned i = 0; i < faceCount; ++i)
    {
        const auto& vertices = faces[i]->vertices_;
        if (vertices.size() <= 3 || (i < origins.size() && origins[i].solid_))
            continue;
        for (unsigned k = 0; k < vertices.size(); ++k)
        {
            const Vector3 from = vertices[k].GetPosition();
            const Vector3 to = vertices[(k + 1) % vertices.size()].GetPosition();
            // A position with a NaN is never equal to another one.
            if (!hasNan(from) && !hasNan(to))
                edgeOwners.push_back({from, to, i});
        }
    }
    ea::sort(edgeOwners.begin(), edgeOwners.end(), edgeLess);
    // Return the owner of the edge from `from` to `to`, or -1.
    const auto findOwner = [&](const Vector3& from, const Vector3& to) -> int
    {
        if (hasNan(from) || hasNan(to))
            return -1;
        // The first entry after the edge. The entry before it is the last face of the edge.
        const auto it = ea::upper_bound(edgeOwners.begin(), edgeOwners.end(), EdgeOwner{from, to, ~0u}, edgeLess);
        if (it == edgeOwners.begin())
            return -1;
        const EdgeOwner& owner = *(it - 1);
        const bool match =
            !less(owner.from_, from) && !less(from, owner.from_) && !less(owner.to_, to) && !less(to, owner.to_);
        return match ? static_cast<int>(owner.face_) : -1;
    };

    ea::vector<unsigned> scratch;
    ea::vector<bool> straight;
    CsgTriangulationScratch triangulationScratch;
    const auto triangulateFace = [&](unsigned i)
    {
        const auto& vertices = faces[i]->vertices_;

        // A face with an origin copies the triangles and the bounds of its source face.
        if (i < origins.size() && origins[i].solid_ && CopyOrigin(faces[i], origins[i], firstFace + i, output))
            return;

        Vector3 min(INF, INF, INF);
        Vector3 max(-INF, -INF, -INF);
        for (const ModelVertex& vertex : vertices)
        {
            const Vector3& point = vertex.GetPosition();
            min = MinOf(min, point);
            max = MaxOf(max, point);
        }
        output.faceMin_.push_back(min);
        output.faceMax_.push_back(max);

        const auto appendTriangle = [&](unsigned a, unsigned b, unsigned c)
        {
            output.trianglePositions_.push_back(vertices[a].GetPosition());
            output.trianglePositions_.push_back(vertices[b].GetPosition());
            output.trianglePositions_.push_back(vertices[c].GetPosition());
            output.triangleVertices_.push_back(a);
            output.triangleVertices_.push_back(b);
            output.triangleVertices_.push_back(c);
        };

        if (vertices.size() < 3)
            return;
        if (vertices.size() == 3)
        {
            appendTriangle(0, 1, 2);
            return;
        }

        // Only the later face of a pair refrains from clipping a vertex of a shared run. This is enough to keep
        // the sliver from being claimed twice, and it leaves the earlier face free to triangulate as it must.
        const unsigned vertexCount = vertices.size();
        straight.resize(vertexCount);
        bool anyStraight = false;
        for (unsigned k = 0; k < vertexCount; ++k)
        {
            const Vector3 previous = vertices[(k + vertexCount - 1) % vertexCount].GetPosition();
            const Vector3 current = vertices[k].GetPosition();
            const Vector3 next = vertices[(k + 1) % vertexCount].GetPosition();
            bool interior = false;
            if (!edgeOwners.empty())
            {
                const int before = findOwner(current, previous);
                const int after = before >= 0 ? findOwner(next, current) : -1;
                interior = before >= 0 && before == after && static_cast<unsigned>(before) < i;
            }
            straight[k] = interior;
            anyStraight |= interior;
        }
        bool clean = true;
        if (CsgTriangulatePolygon(*faces[i], scratch,
                anyStraight ? ea::span<const bool>(straight.data(), vertexCount) : ea::span<const bool>(), clean,
                triangulationScratch))
        {
            if (!clean)
                output.fallbackFaces_.push_back(firstFace + i);
            TurnFlatTriangles(vertices, scratch);
            for (unsigned t = 0; t + 2 < scratch.size(); t += 3)
                appendTriangle(scratch[t], scratch[t + 1], scratch[t + 2]);
        }
    };

    for (unsigned i = 0; i < faceCount; ++i)
    {
        if (output.corners_)
        {
            output.vertexStart_->push_back(output.corners_->size());
            for (const ModelVertex& vertex : faces[i]->vertices_)
                output.corners_->push_back(vertex.GetPosition());
        }
        triangulateFace(i);
        output.triangleStart_.push_back(output.triangleVertices_.size() / 3);
    }
}

void CsgSolid::Build(ea::vector<CsgSharedPolygon>&& faces, float epsilon, ea::span<const CsgFaceOrigin> origins,
    ea::span<const int> exactIds, int exactIdSpace, ea::span<const CsgGridPoint> exactGrid)
{
    URHO3D_PROFILE("CsgSolid::Build");
    if (epsilon <= 0.0f)
        epsilon = CSG_DEFAULT_EPSILON;

    // The function can run again on the same solid, after a repair.
    faces_ = ea::move(faces);
    const unsigned faceCount = faces_.size();
    triangleStart_.clear();
    trianglePositions_.clear();
    triangleVertices_.clear();
    fallbackFaces_.clear();
    faceMin_.clear();
    faceMax_.clear();
    URHO3D_ASSERT(origins.empty() || origins.size() == faceCount, "CsgSolid origins must have one entry for each face");
    triangleStart_.reserve(faceCount + 1);
    faceMin_.reserve(faceCount);
    faceMax_.reserve(faceCount);

    unsigned estimatedTriangles = 0;
    unsigned cornerCount = 0;
    for (const CsgSharedPolygon& face : faces_)
    {
        const unsigned vertexCount = face->vertices_.size();
        estimatedTriangles += vertexCount > 2 ? vertexCount - 2 : 0;
        cornerCount += vertexCount;
    }
    trianglePositions_.reserve(estimatedTriangles * 3);
    triangleVertices_.reserve(estimatedTriangles * 3);

    // Flat corner positions in face order. The topology reads only these.
    ea::vector<Vector3> corners;
    corners.reserve(cornerCount);
    ea::vector<unsigned> vertexStart;
    vertexStart.reserve(faceCount + 1);

    triangleStart_.push_back(0);
    TriangleOutput output{triangleStart_, trianglePositions_, triangleVertices_, faceMin_, faceMax_, fallbackFaces_,
        &corners, &vertexStart};
    Triangulate(faces_, origins, 0, output);
    vertexStart.push_back(corners.size());

    {
        Topology topology;
        BuildTopology(corners, vertexStart, triangleStart_, epsilon, exactIds, exactIdSpace, exactGrid, topology);
        componentStart_ = ea::move(topology.componentStart);
        componentFaces_ = ea::move(topology.componentFaces);
        faceComponents_ = ea::move(topology.faceComponents);
        exact_ = ea::move(topology.exact);
        welded_ = ea::move(topology.welded);
        openEdges_ = ea::move(topology.openEdges);
        closed_ = topology.closed;
        hasJunctions_ = topology.hasJunctions;
        outOfRange_ = exact_->outOfRange_;
        epsilon_ = epsilon;
    }

    BuildBounds();
}

bool CsgSolid::CopyOrigin(
    const CsgSharedPolygon& face, const CsgFaceOrigin& origin, unsigned index, TriangleOutput& output)
{
    const CsgSolid& source = *origin.solid_;
    // An origin that does not fit the face is a programming error. A release build ignores it
    // and triangulates the face like a face without an origin.
    const bool fits = origin.face_ < source.faces_.size()
        && source.faces_[origin.face_]->vertices_.size() == face->vertices_.size();
    URHO3D_ASSERT(fits, "CsgSolid origin does not fit the face");
    if (!fits)
        return false;

    output.faceMin_.push_back(source.faceMin_[origin.face_]);
    output.faceMax_.push_back(source.faceMax_[origin.face_]);
    if (source.IsFallbackFace(origin.face_))
        output.fallbackFaces_.push_back(index);

    const unsigned first = source.triangleStart_[origin.face_];
    const unsigned count = source.triangleStart_[origin.face_ + 1] - first;
    const Vector3* positions = source.trianglePositions_.data() + first * 3;
    const unsigned* locals = source.triangleVertices_.data() + first * 3;
    if (origin.flip_)
    {
        // The face is the flipped copy of the source face. Reverse each triangle and renumber its corners.
        const unsigned vertexCount = source.faces_[origin.face_]->vertices_.size();
        for (unsigned t = 0; t < count; ++t)
        {
            for (unsigned corner = 0; corner < 3; ++corner)
            {
                output.trianglePositions_.push_back(positions[t * 3 + 2 - corner]);
                output.triangleVertices_.push_back(vertexCount - 1 - locals[t * 3 + 2 - corner]);
            }
        }
    }
    else
    {
        output.trianglePositions_.insert(output.trianglePositions_.end(), positions, positions + count * 3);
        output.triangleVertices_.insert(output.triangleVertices_.end(), locals, locals + count * 3);
    }
    return true;
}

void CsgSolid::BuildBounds()
{
    const unsigned faceCount = faces_.size();
    boundsMin_ = Vector3(INF, INF, INF);
    boundsMax_ = Vector3(-INF, -INF, -INF);
    for (unsigned i = 0; i < faceCount; ++i)
    {
        boundsMin_ = MinOf(boundsMin_, faceMin_[i]);
        boundsMax_ = MaxOf(boundsMax_, faceMax_[i]);
    }

    const unsigned componentCount = ComponentCount();
    componentMin_.resize(componentCount);
    componentMax_.resize(componentCount);
    for (unsigned component = 0; component < componentCount; ++component)
    {
        Vector3 min(INF, INF, INF);
        Vector3 max(-INF, -INF, -INF);
        for (unsigned c = componentStart_[component]; c < componentStart_[component + 1]; ++c)
        {
            min = MinOf(min, faceMin_[componentFaces_[c]]);
            max = MaxOf(max, faceMax_[componentFaces_[c]]);
        }
        componentMin_[component] = min;
        componentMax_[component] = max;
    }
    faceTrees_ = ea::make_unique<FaceTreeSlot[]>(componentCount);
    // Built here, so that a query from many threads only reads the tree.
    componentTree_ = ea::make_unique<CsgFaceBvh>(
        ea::span<const Vector3>(componentMin_.data(), componentMin_.size()),
        ea::span<const Vector3>(componentMax_.data(), componentMax_.size()));
}

bool CsgSolid::ContainsPoint(const Vector3& point) const
{
    // A point that is not finite or not in the range of the grid is in no solid.
    if (faces_.empty() || !exact_ || outOfRange_ || !InGridRange(point))
        return false;
    return CsgInternal::CsgContainsGridPoint(*this, CsgGridPoint::From(point));
}

} // namespace Urho3D
