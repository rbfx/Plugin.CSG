// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgSolid.h"

#include "Urho3D/CSG/CsgTriangulation.h"
#include <Urho3D/Core/Profiler.h>

#include <EASTL/algorithm.h>
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

/// Edge between two welded ids, counted over all faces.
struct EdgeState
{
    int face_{};
    /// Sum of the directions of the edge: +1 from the low id to the high id, -1 from the high id to the low id.
    int winding_{};
};

using RawEdgeMap = CsgFlatMap<uint64_t, EdgeState, CsgPackedKeyHash>;

/// Face corners welded into loops of vertex ids.
///
/// Repeated ids are dropped. A loop with less than three ids, or a face without triangles,
/// bounds no surface and gets a loop length of zero.
struct WeldedLoops
{
    WeldedLoops(float epsilon, unsigned cornerCount)
        : welder_(epsilon, cornerCount)
        , ids_(cornerCount)
        , vertexIds_(cornerCount)
    {
    }

    CsgPointWelder welder_;
    /// Welded id of each face corner.
    ea::vector<int> ids_;
    /// Loop ids of each face. The layout is the same as in ids_.
    ea::vector<int> vertexIds_;
    ea::vector<int> loopLength_;
    /// Edge counts between loop ids. Built on first use.
    RawEdgeMap rawEdges_;
    bool hasRawEdges_{};
    /// Each face edge after the first on its welded edge, paired with the first face of that edge.
    ea::vector<ea::pair<int, int>> links_;
};

void BuildLoops(WeldedLoops& loops, ea::span<const unsigned> vertexStart, ea::span<const unsigned> triangleStart)
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
            loops.vertexIds_[start + length++] = id;
        }
        while (length > 1 && loops.vertexIds_[start + length - 1] == loops.vertexIds_[start])
            --length;
        const bool hasSurface = triangleStart[face] != triangleStart[face + 1];
        loops.loopLength_[face] = hasSurface && length >= 3 ? static_cast<int>(length) : 0;
    }
}

void EnsureRawEdges(WeldedLoops& loops, ea::span<const unsigned> vertexStart)
{
    if (loops.hasRawEdges_)
        return;
    loops.hasRawEdges_ = true;
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
            EdgeState& state = loops.rawEdges_.FindOrAdd(CsgUndirectedKey(from, to), existed);
            if (existed)
                loops.links_.push_back({static_cast<int>(face), state.face_});
            else
                state.face_ = static_cast<int>(face);
            state.winding_ += from < to ? 1 : -1;
        }
    }
}

/// Return true if every directed edge has an opposite partner, so the loops close up exactly.
bool IsBalanced(const WeldedLoops& loops)
{
    for (const auto& entry : loops.rawEdges_)
    {
        if (entry.value_.winding_ != 0)
            return false;
    }
    return true;
}

/// Return true if the edge has an opposite partner, so it cannot end at a T-junction.
bool IsMatched(const WeldedLoops& loops, int from, int to)
{
    const EdgeState* state = loops.rawEdges_.Find(CsgUndirectedKey(from, to));
    return state && state->winding_ == 0;
}

/// Return true if a face with triangles welds to less than three ids.
bool HasCollapsedFace(const WeldedLoops& loops, ea::span<const unsigned> triangleStart)
{
    for (unsigned face = 0; face < loops.loopLength_.size(); ++face)
    {
        if (loops.loopLength_[face] < 3 && triangleStart[face] != triangleStart[face + 1])
            return true;
    }
    return false;
}

/// Welded point that lies near the interior of an edge. The parameter t is the place along the edge.
struct Junction
{
    float t_{};
    int id_{};
};

/// Finds the welded points near an edge. The points are sorted along their longest axis on first use.
/// The next steps can replace this index with the face BVH.
class JunctionFinder
{
public:
    JunctionFinder(const CsgPointWelder& welder, float tolerance)
        : welder_(welder)
        , tolerance_(tolerance)
    {
    }

    /// Return the welded points within the tolerance of the edge interior, in order along the edge.
    void Collect(int from, int to, ea::vector<Junction>& cuts)
    {
        cuts.clear();
        if (!built_)
            BuildIndex();

        const Vector3 start = welder_.GetPosition(from);
        const Vector3 end = welder_.GetPosition(to);
        const Vector3 boxMin = MinOf(start, end);
        const Vector3 boxMax = MaxOf(start, end);
        const Vector3 span = end - start;
        const float spanLengthSquared = span.LengthSquared();
        const float toleranceSquared = tolerance_ * tolerance_;

        const float low = boxMin.Data()[axis_] - tolerance_;
        const float high = boxMax.Data()[axis_] + tolerance_;
        const unsigned count = keys_.size();
        for (unsigned i = ea::lower_bound(keys_.begin(), keys_.end(), low) - keys_.begin(); i < count && keys_[i] <= high;
             ++i)
        {
            const int item = order_[i];
            if (item == from || item == to)
                continue;
            const Vector3& point = welder_.GetPosition(item);
            if (point.x_ < boxMin.x_ - tolerance_ || point.x_ > boxMax.x_ + tolerance_
                || point.y_ < boxMin.y_ - tolerance_ || point.y_ > boxMax.y_ + tolerance_
                || point.z_ < boxMin.z_ - tolerance_ || point.z_ > boxMax.z_ + tolerance_)
                continue;
            const float t = (point - start).DotProduct(span) / spanLengthSquared;
            if (t <= 0.0f || t >= 1.0f)
                continue;
            if ((point - (start + span * t)).LengthSquared() <= toleranceSquared)
                cuts.push_back({t, item});
        }
        ea::sort(cuts.begin(), cuts.end(),
            [](const Junction& a, const Junction& b) { return a.t_ < b.t_ || (a.t_ == b.t_ && a.id_ < b.id_); });
    }

private:
    void BuildIndex()
    {
        built_ = true;
        const unsigned count = welder_.Count();
        Vector3 min(INF, INF, INF);
        Vector3 max(-INF, -INF, -INF);
        for (const Vector3& point : welder_.GetPositions())
        {
            min = MinOf(min, point);
            max = MaxOf(max, point);
        }
        axis_ = 0;
        for (int i = 1; i < 3; ++i)
        {
            if (max.Data()[i] - min.Data()[i] > max.Data()[axis_] - min.Data()[axis_])
                axis_ = i;
        }
        // A point with a NaN coordinate never passes the box test. It must not enter the sort,
        // because a NaN key breaks the strict weak order of the comparator.
        const int axis = axis_;
        order_.reserve(count);
        for (unsigned i = 0; i < count; ++i)
        {
            const Vector3& point = welder_.GetPosition(static_cast<int>(i));
            if (point.x_ == point.x_ && point.y_ == point.y_ && point.z_ == point.z_)
                order_.push_back(static_cast<int>(i));
        }
        ea::sort(order_.begin(), order_.end(),
            [this, axis](int a, int b)
            {
                const float ka = welder_.GetPosition(a).Data()[axis];
                const float kb = welder_.GetPosition(b).Data()[axis];
                return ka < kb || (ka == kb && a < b);
            });
        keys_.resize(order_.size());
        for (unsigned i = 0; i < order_.size(); ++i)
            keys_[i] = welder_.GetPosition(order_[i]).Data()[axis];
    }

    const CsgPointWelder& welder_;
    float tolerance_{};
    bool built_{};
    int axis_{};
    ea::vector<int> order_;
    ea::vector<float> keys_;
};

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

/// Split the faces into components. Faces are in one component if they share a welded edge.
///
/// The corners are welded exactly first. If every edge has an opposite partner, the exact loops give the components.
/// Otherwise the distinct positions are welded again within epsilon.
/// Then an edge that ends at a T-junction is split at the welded points along it.
void BuildComponents(ea::span<const Vector3> corners, ea::span<const unsigned> vertexStart,
    ea::span<const unsigned> triangleStart, float epsilon, ea::vector<unsigned>& componentStart,
    ea::vector<unsigned>& componentFaces, ea::vector<unsigned>& faceComponents)
{
    URHO3D_PROFILE("CsgSolid::BuildComponents");
    const unsigned faceCount = vertexStart.size() - 1;
    const unsigned cornerCount = corners.size();
    FaceSets sets(faceCount);

    WeldedLoops exact(0.0f, cornerCount);
    for (unsigned i = 0; i < cornerCount; ++i)
        exact.ids_[i] = exact.welder_.Add(corners[i]);
    BuildLoops(exact, vertexStart, triangleStart);
    EnsureRawEdges(exact, vertexStart);

    // Loops that close up exactly, as boolean results and most modeled meshes do, are closed within epsilon
    // as well and need no second welding. Only float noise between neighboring faces calls for it.
    const bool balanced = IsBalanced(exact);
    ea::unique_ptr<WeldedLoops> welded;
    if (!balanced)
    {
        welded = ea::make_unique<WeldedLoops>(epsilon, cornerCount);
        ea::vector<int> map(exact.welder_.Count());
        for (unsigned i = 0; i < map.size(); ++i)
            map[i] = welded->welder_.Add(exact.welder_.GetPosition(i));
        for (unsigned i = 0; i < cornerCount; ++i)
            welded->ids_[i] = map[exact.ids_[i]];
        BuildLoops(*welded, vertexStart, triangleStart);
        EnsureRawEdges(*welded, vertexStart);
    }
    WeldedLoops& loops = balanced ? exact : *welded;
    const bool collapsed = HasCollapsedFace(loops, triangleStart);

    // Balanced loops match every edge exactly. Nothing can end at a T-junction, and the faces
    // that each edge joins are known already.
    if (balanced && !collapsed)
    {
        for (const auto& link : loops.links_)
            sets.Union(link.first, link.second);
        GroupComponents(faceCount, sets, componentStart, componentFaces, faceComponents);
        return;
    }

    // Only edges without an opposite partner can end at a T-junction. Split those at the welded points on their
    // interior, so a long edge matches the shorter edges along it.
    CsgFlatMap<uint64_t, int, CsgPackedKeyHash> edgeFaces;
    edgeFaces.Reserve(loops.rawEdges_.Size());
    const auto registerEdge = [&](int face, int from, int to)
    {
        bool existed = false;
        int& first = edgeFaces.FindOrAdd(CsgUndirectedKey(from, to), existed);
        if (existed)
            sets.Union(face, first);
        else
            first = face;
    };

    JunctionFinder finder(loops.welder_, loops.welder_.GetEpsilon() * JUNCTION_SCALE);
    ea::vector<Junction> cuts;
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
            int previous = from;
            for (const Junction& cut : cuts)
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

    GroupComponents(faceCount, sets, componentStart, componentFaces, faceComponents);
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
}

CsgSolid::~CsgSolid() = default;

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

    faces_ = ea::move(faces);
    const unsigned faceCount = faces_.size();
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

    BuildComponents(corners, vertexStart, triangleStart_, epsilon, componentStart_, componentFaces_, faceComponents_);

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
}

} // namespace Urho3D
