// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgSolid.h"

#include "Urho3D/CSG/CsgFaceBvh.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include <Urho3D/Core/Profiler.h>

#include <EASTL/algorithm.h>
#include <EASTL/fixed_vector.h>
#include <EASTL/sort.h>
#include <EASTL/unique_ptr.h>
#include <EASTL/utility.h>

#include <cmath>
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
            if (t <= 0.0f || t >= 1.0f)
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

    /// Return the finder of junctions. The finder is built on first use.
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
    /// Edge states between loop ids.
    CsgRawEdgeMap rawEdges_;
    /// Each face edge after the first on its welded edge, paired with the first face of that edge.
    ea::vector<ea::pair<int, int>> links_;
    /// True if every corner is exactly at the position of its welded id.
    bool canonical_{true};
    mutable ea::unique_ptr<CsgJunctionFinder> finder_;
};

namespace
{

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

void BuildRawEdges(CsgWeldedLoops& loops, ea::span<const unsigned> vertexStart)
{
    loops.rawEdges_.Reserve(loops.vertexIds_.size());
    const unsigned faceCount = vertexStart.size() - 1;
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
        : parents_(count)
        , ranks_(count, 0)
    {
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
    ea::vector<int> componentIndices(faceCount, -1);
    faceComponents.resize(faceCount);
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

/// Directions of the parity rays. They are not axis aligned, so the rays avoid most shared edges and vertices.
const Vector3* GetRayDirections()
{
    static const Vector3 directions[3] = {
        Vector3(1.0f, 0.371f, 0.173f).Normalized(),
        Vector3(0.239f, 1.0f, 0.419f).Normalized(),
        Vector3(0.193f, 0.317f, 1.0f).Normalized(),
    };
    return directions;
}

/// Return true if the ray from the origin can hit the box. The inverse is one over the direction.
/// A NaN value never rejects the box. The triangle test rejects a NaN hit.
bool RayHitsBox(const Vector3& origin, const Vector3& inverse, const Vector3& min, const Vector3& max)
{
    float enter = 0.0f;
    float exit = INF;
    for (int axis = 0; axis < 3; ++axis)
    {
        float t0 = (min.Data()[axis] - origin.Data()[axis]) * inverse.Data()[axis];
        float t1 = (max.Data()[axis] - origin.Data()[axis]) * inverse.Data()[axis];
        if (t0 > t1)
            ea::swap(t0, t1);
        if (t0 > enter)
            enter = t0;
        if (t1 < exit)
            exit = t1;
    }
    return enter <= exit;
}

/// Two-sided Moeller-Trumbore test. Return infinity if the ray misses. NaN input can give a NaN distance.
float RayTriangle(
    const Vector3& origin, const Vector3& direction, const Vector3& v0, const Vector3& v1, const Vector3& v2)
{
    const Vector3 edge1 = v1 - v0;
    const Vector3 edge2 = v2 - v0;
    const Vector3 p = direction.CrossProduct(edge2);
    float det = edge1.DotProduct(p);
    if (std::abs(det) < M_EPSILON)
        return INF;
    // A negative determinant is the back face. Mirror the barycentric tests instead of a rewind.
    const float sign = det < 0.0f ? -1.0f : 1.0f;
    det *= sign;
    const Vector3 offset = origin - v0;
    const float u = offset.DotProduct(p) * sign;
    if (u < 0.0f || u > det)
        return INF;
    const Vector3 q = offset.CrossProduct(edge1);
    const float v = direction.DotProduct(q) * sign;
    if (v < 0.0f || u + v > det)
        return INF;
    const float distance = edge2.DotProduct(q) * sign / det;
    return distance >= 0.0f ? distance : INF;
}

using RayHits = ea::fixed_vector<float, 64, true>;
using NodeStack = ea::fixed_vector<int, 64, true>;

void CollectRayHits(const CsgSolid& solid, const CsgFaceBvh& tree, const Vector3& origin, const Vector3& direction,
    const Vector3& inverse, RayHits& hits)
{
    if (tree.ItemCount() == 0)
        return;
    const ea::span<const CsgFaceBvh::Node> nodes = tree.GetNodes();
    const ea::span<const int> items = tree.GetItems();
    const ea::span<const unsigned> triangleStart = solid.GetTriangleStart();
    const ea::span<const Vector3> positions = solid.GetTrianglePositions();
    NodeStack stack;
    stack.push_back(0);
    while (!stack.empty())
    {
        const CsgFaceBvh::Node& node = nodes[stack.back()];
        stack.pop_back();
        if (!RayHitsBox(origin, inverse, node.min_, node.max_))
            continue;
        if (node.count_ == 0)
        {
            stack.push_back(node.first_ + 1);
            stack.push_back(node.first_);
            continue;
        }
        for (int i = node.first_; i < node.first_ + node.count_; ++i)
        {
            const unsigned face = items[i];
            for (unsigned t = triangleStart[face]; t < triangleStart[face + 1]; ++t)
            {
                const float distance = RayTriangle(origin, direction, positions[t * 3],
                    positions[t * 3 + 1], positions[t * 3 + 2]);
                // The negated test also catches NaN. A hit at the origin is no crossing.
                if (!(distance > 0.0f) || !std::isfinite(distance))
                    continue;
                hits.push_back(distance);
            }
        }
    }
}

int CountCrossings(const CsgSolid& solid, const Vector3& origin, const Vector3& direction, float epsilon)
{
    const Vector3 inverse(1.0f / direction.x_, 1.0f / direction.y_, 1.0f / direction.z_);
    RayHits hits;
    NodeStack stack;

    const CsgFaceBvh& components = solid.GetComponentTree();
    if (components.ItemCount() != 0)
    {
        const ea::span<const CsgFaceBvh::Node> nodes = components.GetNodes();
        const ea::span<const int> items = components.GetItems();
        stack.push_back(0);
        while (!stack.empty())
        {
            const CsgFaceBvh::Node& node = nodes[stack.back()];
            stack.pop_back();
            if (!RayHitsBox(origin, inverse, node.min_, node.max_))
                continue;
            if (node.count_ == 0)
            {
                stack.push_back(node.first_ + 1);
                stack.push_back(node.first_);
                continue;
            }
            for (int i = node.first_; i < node.first_ + node.count_; ++i)
            {
                const unsigned component = items[i];
                if (RayHitsBox(origin, inverse, solid.GetComponentMin(component), solid.GetComponentMax(component)))
                    CollectRayHits(solid, solid.GetFaceTree(component), origin, direction, inverse, hits);
            }
        }
    }

    // A crossing is a distance that differs from the last by more than two epsilons. This way two triangles
    // on the same surface do not change the parity twice. All distances are finite, so the sort is safe.
    ea::sort(hits.begin(), hits.end());
    int crossings = 0;
    float last = -INF;
    for (const float hit : hits)
    {
        if (hit - last <= epsilon * 2.0f)
            continue;
        ++crossings;
        last = hit;
    }
    return crossings;
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

/// Split the faces into components and find out if the faces bound a closed surface.
/// Faces are in one component if they share a welded edge.
///
/// The corners are welded exactly first. If every edge has an opposite partner, the exact loops give the components.
/// Otherwise the distinct positions are welded again within epsilon.
/// Then an edge that ends at a T-junction is split at the welded points along it.
void BuildTopology(ea::span<const Vector3> corners, ea::span<const unsigned> vertexStart,
    ea::span<const unsigned> triangleStart, float epsilon, Topology& result)
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

    result.exact = ea::make_unique<CsgWeldedLoops>(0.0f, cornerCount);
    CsgWeldedLoops& exact = *result.exact;
    for (unsigned i = 0; i < cornerCount; ++i)
        exact.ids_[i] = exact.welder_.Add(corners[i]);
    BuildLoops(exact, vertexStart, triangleStart);
    BuildRawEdges(exact, vertexStart);

    // Loops that close up exactly, as boolean results and most modeled meshes do, are closed within epsilon
    // as well and need no second welding. Only float noise between neighboring faces calls for it.
    const bool balanced = IsBalanced(exact);
    if (!balanced)
    {
        result.welded = ea::make_unique<CsgWeldedLoops>(epsilon, cornerCount);
        CsgWeldedLoops& welded = *result.welded;
        ea::vector<int> map(exact.welder_.Count());
        for (unsigned i = 0; i < map.size(); ++i)
        {
            map[i] = welded.welder_.Add(exact.welder_.GetPosition(i));
            welded.canonical_ &= map[i] == static_cast<int>(i);
        }
        for (unsigned i = 0; i < cornerCount; ++i)
            welded.ids_[i] = map[exact.ids_[i]];
        BuildLoops(welded, vertexStart, triangleStart);
        BuildRawEdges(welded, vertexStart);
    }
    CsgWeldedLoops& loops = balanced ? exact : *result.welded;
    const bool collapsed = HasCollapsedFace(loops, triangleStart);

    // Balanced loops match every edge exactly. Nothing can end at a T-junction, and the faces
    // that each edge joins are known already.
    if (balanced && !collapsed)
    {
        for (const auto& link : loops.links_)
            sets.Union(link.first, link.second);
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

    // A face smaller than epsilon welds to less than three ids and takes no part in the edge graph.
    // It belongs to the component of the faces that share its vertices.
    if (collapsed)
    {
        CsgFlatMap<uint64_t, int, CsgPackedKeyHash> vertexOwner;
        for (unsigned face = 0; face < faceCount; ++face)
        {
            if (loops.loopLength_[face] < 3)
                continue;
            for (int i = 0; i < loops.loopLength_[face]; ++i)
                vertexOwner.TryAdd(static_cast<uint64_t>(loops.vertexIds_[vertexStart[face] + i]), static_cast<int>(face));
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
    }

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

int CsgPointWelder::Add(const Vector3& point)
{
    const int index = static_cast<int>(positions_.size());
    if (epsilon_ <= 0.0f)
    {
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

CsgSolid::CsgSolid(ea::span<const CsgPolygon> polygons, float epsilon)
{
    URHO3D_PROFILE("CsgSolid::CsgSolid");
    ea::vector<CsgSharedPolygon> faces;
    faces.reserve(polygons.size());
    for (const CsgPolygon& polygon : polygons)
        faces.push_back(ea::make_shared<CsgPolygon>(polygon));
    Build(ea::move(faces), epsilon);
    // Corefinement needs the neighboring faces to share their edge vertices at identical positions.
    if (NeedsRepair())
        Build(WatertightFaces(), epsilon_);
}

CsgSolid::CsgSolid(ea::vector<CsgSharedPolygon>&& faces, float epsilon)
{
    URHO3D_PROFILE("CsgSolid::CsgSolid");
    Build(ea::move(faces), epsilon);
}

CsgSolid::~CsgSolid() = default;

SharedPtr<CsgSolid> CsgSolid::FromOperand(ea::vector<CsgPolygon>&& polygons, float epsilon)
{
    URHO3D_PROFILE("CsgSolid::FromOperand");
    ea::vector<CsgSharedPolygon> faces;
    faces.reserve(polygons.size());
    for (CsgPolygon& polygon : polygons)
        faces.push_back(ea::make_shared<CsgPolygon>(ea::move(polygon)));
    SharedPtr<CsgSolid> solid(new CsgSolid(ea::move(faces), epsilon));
    if (!solid->NeedsRepair())
        return solid;
    return SharedPtr<CsgSolid>(new CsgSolid(solid->WatertightFaces(), solid->epsilon_));
}

bool CsgSolid::NeedsRepair() const
{
    return hasJunctions_ || !GetWeld().canonical_;
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

ea::vector<CsgSharedPolygon> CsgSolid::WatertightFaces() const
{
    URHO3D_PROFILE("CsgSolid::WatertightFaces");
    const CsgWeldedLoops& loops = GetWeld();
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
            ModelVertex corner = source.vertices_[loops.corners_[cornerStart + i]];
            const Vector3& position = welder.GetPosition(from);
            // A NaN position is never equal, so such a corner counts as changed. The result is the same corner.
            if (corner.GetPosition() != position)
            {
                corner.SetPosition(position);
                changed = true;
            }
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
            }
            changed = true;
        }
        if (!changed)
        {
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
    if (!componentTree_)
        componentTree_ = ea::make_unique<CsgFaceBvh>(
            ea::span<const Vector3>(componentMin_.data(), componentMin_.size()),
            ea::span<const Vector3>(componentMax_.data(), componentMax_.size()));
    return *componentTree_;
}

const CsgFaceBvh& CsgSolid::GetFaceTree(unsigned component) const
{
    ea::unique_ptr<CsgFaceBvh>& tree = faceTrees_[component];
    if (!tree)
    {
        tree = ea::make_unique<CsgFaceBvh>(GetFaceMin(), GetFaceMax(),
            ea::span<const unsigned>(componentFaces_.data() + componentStart_[component],
                componentStart_[component + 1] - componentStart_[component]));
    }
    return *tree;
}

const CsgFaceBvh& CsgSolid::GetNearFaces(unsigned component, const Vector3& min, const Vector3& max, float epsilon,
    ea::unique_ptr<CsgFaceBvh>& temporary) const
{
    if (faceTrees_[component])
        return *faceTrees_[component];
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

void CsgSolid::Build(ea::vector<CsgSharedPolygon>&& faces, float epsilon)
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
    triangleStart_.reserve(faceCount + 1);
    faceMin_.resize(faceCount);
    faceMax_.resize(faceCount);

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

    {
        URHO3D_PROFILE("CsgSolid::Triangulate");
        ea::vector<unsigned> scratch;
        unsigned triangleCount = 0;
        for (unsigned i = 0; i < faceCount; ++i)
        {
            const auto& vertices = faces_[i]->vertices_;
            triangleStart_.push_back(triangleCount);
            vertexStart.push_back(corners.size());

            Vector3 min(INF, INF, INF);
            Vector3 max(-INF, -INF, -INF);
            for (const ModelVertex& vertex : vertices)
            {
                const Vector3& point = vertex.GetPosition();
                corners.push_back(point);
                min = MinOf(min, point);
                max = MaxOf(max, point);
            }
            faceMin_[i] = min;
            faceMax_[i] = max;

            const auto appendTriangle = [&](unsigned a, unsigned b, unsigned c)
            {
                trianglePositions_.push_back(corners[vertexStart[i] + a]);
                trianglePositions_.push_back(corners[vertexStart[i] + b]);
                trianglePositions_.push_back(corners[vertexStart[i] + c]);
                triangleVertices_.push_back(a);
                triangleVertices_.push_back(b);
                triangleVertices_.push_back(c);
                ++triangleCount;
            };

            if (vertices.size() < 3)
                continue;
            if (vertices.size() == 3)
                appendTriangle(0, 1, 2);
            else if (CsgTriangulatePolygon(*faces_[i], scratch))
            {
                for (unsigned t = 0; t + 2 < scratch.size(); t += 3)
                    appendTriangle(scratch[t], scratch[t + 1], scratch[t + 2]);
            }
        }
        triangleStart_.push_back(triangleCount);
        vertexStart.push_back(corners.size());
    }

    {
        Topology topology;
        BuildTopology(corners, vertexStart, triangleStart_, epsilon, topology);
        componentStart_ = ea::move(topology.componentStart);
        componentFaces_ = ea::move(topology.componentFaces);
        faceComponents_ = ea::move(topology.faceComponents);
        exact_ = ea::move(topology.exact);
        welded_ = ea::move(topology.welded);
        openEdges_ = ea::move(topology.openEdges);
        closed_ = topology.closed;
        hasJunctions_ = topology.hasJunctions;
        epsilon_ = epsilon;
    }

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
    faceTrees_.clear();
    faceTrees_.resize(componentCount);
    componentTree_.reset();
}

bool CsgSolid::ContainsPoint(const Vector3& point, float epsilon) const
{
    // The negated test also catches NaN.
    if (!(epsilon > 0.0f))
        epsilon = CSG_DEFAULT_EPSILON;
    if (faces_.empty())
        return false;
    // A point that is not finite is in no solid, and a ray from it has no meaning.
    if (!std::isfinite(point.x_) || !std::isfinite(point.y_) || !std::isfinite(point.z_))
        return false;
    if (!BoundsOverlap(point, point, 0.0f))
        return false;

    const Vector3* directions = GetRayDirections();
    int insideVotes = 0;
    for (int ray = 0; ray < 3; ++ray)
    {
        if ((CountCrossings(*this, point, directions[ray], epsilon) & 1) != 0)
            ++insideVotes;
        // Majority of three: stop once the remaining ray cannot change the outcome.
        if (insideVotes == 2 || ray - insideVotes == 1)
            break;
    }
    return insideVotes >= 2;
}

} // namespace Urho3D
