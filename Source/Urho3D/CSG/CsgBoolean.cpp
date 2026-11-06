// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgBoolean.h"

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include <Urho3D/IO/Log.h>

#include <EASTL/algorithm.h>
#include <EASTL/fixed_vector.h>
#include <EASTL/sort.h>

#include <cmath>

namespace Urho3D
{

namespace CsgInternal
{

namespace
{

uint64_t UndirectedKey(int a, int b)
{
    return CsgUndirectedKey(static_cast<unsigned>(a), static_cast<unsigned>(b));
}

uint64_t DirectedKey(int from, int to)
{
    return CsgDirectedKey(static_cast<unsigned>(from), static_cast<unsigned>(to));
}

double PolygonArea(const CsgPolygon& polygon)
{
    const unsigned count = polygon.vertices_.size();
    Vector3 normal = Vector3::ZERO;
    for (unsigned i = 0; i < count; ++i)
    {
        normal += polygon.vertices_[i].GetPosition().CrossProduct(polygon.vertices_[(i + 1) % count].GetPosition());
    }
    return static_cast<double>(normal.Length()) * 0.5;
}

/// Return the number of ids in the loop of an element.
int ElementLoopSize(const Mesh& mesh, const CsgBooleanElement& element)
{
    return element.sourceFace_ >= 0 ? mesh.loops_.loopLength_[element.sourceFace_] : element.idCount_;
}

bool ClassifyGridCentroid(
    const Mesh& other, const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, int second);

/// Tell if a triangle of grid points has a nonzero area. The cross product of the differences is exact in 64 bits.
bool HasExactArea(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c)
{
    const int64_t abx = static_cast<int64_t>(b.x_) - a.x_, aby = static_cast<int64_t>(b.y_) - a.y_,
                  abz = static_cast<int64_t>(b.z_) - a.z_;
    const int64_t acx = static_cast<int64_t>(c.x_) - a.x_, acy = static_cast<int64_t>(c.y_) - a.y_,
                  acz = static_cast<int64_t>(c.z_) - a.z_;
    return aby * acz - abz * acy != 0 || abz * acx - abx * acz != 0 || abx * acy - aby * acx != 0;
}

/// Tell if the loop of an element has three grid points that are not on one line.
/// The function reads the loop of the element in s.loops_.
bool HasGridArea(const Corefinement& core, const CsgBooleanScratch& s, int element)
{
    const int* loop = s.loops_.data() + s.edgeStart_[element];
    const int length = s.edgeStart_[element + 1] - s.edgeStart_[element];
    if (length < 3)
        return false;
    const CsgGridPoint& first = PointGrid(core, loop[0]);
    int second = 1;
    while (second < length && PointGrid(core, loop[second]) == first)
        ++second;
    for (int i = second + 1; i < length; ++i)
    {
        if (HasExactArea(first, PointGrid(core, loop[second]), PointGrid(core, loop[i])))
            return true;
    }
    return false;
}

/// The result of the exact classification of an element.
enum class ExactResult
{
    /// The exact test gave the answer.
    Done,
    /// Every triangle of the element has zero exact area.
    ZeroArea,
    /// The polygon does not match its loop, or the triangulation failed.
    Mismatch
};

/// Classify the centroid of the largest triangle of an element with the exact ray test.
/// The loop of the element in s.loops_ has the order of the vertices of its polygon. This holds for a region
/// (RegionPolygon takes the vertex k from the id k), for a corner triangle, and for a face whose loop has no
/// dropped id. The function takes only triangles with a nonzero exact area.
/// If the exact test is not possible, the function leaves the result unchanged and tells the reason.
ExactResult ClassifyElementExact(const Corefinement& core, CsgBooleanScratch& s, int element, const CsgPolygon& polygon,
    const Mesh& otherMesh, bool second, bool& inside)
{
    const auto& vertices = polygon.vertices_;
    const unsigned loopStart = static_cast<unsigned>(s.edgeStart_[element]);
    const unsigned length = static_cast<unsigned>(s.edgeStart_[element + 1]) - loopStart;
    if (vertices.size() != length || length < 3)
        return ExactResult::Mismatch;
    ea::vector<unsigned>& indices = s.indices_;
    indices.clear();
    if (length == 3)
    {
        indices = {0u, 1u, 2u};
    }
    else if (!CsgTriangulatePolygon(polygon, indices) || indices.size() < 3)
    {
        return ExactResult::Mismatch;
    }
    unsigned best = 0;
    float bestArea = -1.0f;
    for (unsigned t = 0; t + 2 < indices.size(); t += 3)
    {
        if (indices[t] >= length || indices[t + 1] >= length || indices[t + 2] >= length)
            continue;
        const CsgGridPoint& ga = PointGrid(core, s.loops_[loopStart + indices[t]]);
        const CsgGridPoint& gb = PointGrid(core, s.loops_[loopStart + indices[t + 1]]);
        const CsgGridPoint& gc = PointGrid(core, s.loops_[loopStart + indices[t + 2]]);
        if (!HasExactArea(ga, gb, gc))
            continue;
        const Vector3 a = vertices[indices[t]].GetPosition();
        const float area =
            (vertices[indices[t + 1]].GetPosition() - a).CrossProduct(vertices[indices[t + 2]].GetPosition() - a)
                .LengthSquared();
        if (area > bestArea)
        {
            bestArea = area;
            best = t;
        }
    }
    if (bestArea < 0.0f)
        return ExactResult::ZeroArea;
    // The second operand moves. A triangle of the first operand uses the mask 0b0111, where the other mesh moves.
    // A triangle of the second operand uses the mask 0b1000, where the triangle moves.
    inside = ClassifyGridCentroid(otherMesh, PointGrid(core, s.loops_[loopStart + indices[best]]),
        PointGrid(core, s.loops_[loopStart + indices[best + 1]]),
        PointGrid(core, s.loops_[loopStart + indices[best + 2]]), second ? 0b1000 : 0b0111);
    return ExactResult::Done;
}

/// Classify the elements of an operand against the other operand. The result is in the labels of the operand:
/// not zero if inside.
///
/// Each curve edge of an element decides which side of the curve is inside the other operand:
/// the element on the left of an oriented segment is inside for the first operand, and on its right for the second.
/// Elements joined by an edge off the curve form a patch. All elements of a patch get the same label:
/// the sign of the sum of the votes of all curve edges of the patch.
/// A patch with no net vote gets a point query on one element. The query is the exact ray test of the centroid of
/// a triangle. The query uses an element with an exact area. A patch with no such element has no side.
/// The patch is removed.
/// The perturbation of the crossing step keeps the curve off the edges of both operands.
/// Because of this only regions of split triangles have curve edges.
void ClassifyElements(const Corefinement& core, const Mesh& mesh, CsgOperandElements& o, CsgBooleanScratch& s,
    const Mesh& otherMesh, bool second)
{
    const int elementCount = static_cast<int>(o.elements_.size());

    // Every element loop in one buffer. An unsplit face reads its loop from the mesh.
    s.edgeStart_.assign(elementCount + 1, 0);
    for (int e = 0; e < elementCount; ++e)
        s.edgeStart_[e + 1] = s.edgeStart_[e] + ElementLoopSize(mesh, o.elements_[e]);
    const int edgeTotal = s.edgeStart_[elementCount];
    s.loops_.resize(edgeTotal);
    for (int e = 0; e < elementCount; ++e)
    {
        const CsgBooleanElement& element = o.elements_[e];
        if (element.sourceFace_ < 0)
        {
            for (int i = 0; i < element.idCount_; ++i)
                s.loops_[s.edgeStart_[e] + i] = o.elementIds_[element.idStart_ + i];
            continue;
        }
        const unsigned start = mesh.loops_.vertexStart_[element.sourceFace_];
        for (int i = s.edgeStart_[e]; i < s.edgeStart_[e + 1]; ++i)
            s.loops_[i] = core.Global(mesh, mesh.loops_.vertexIds_[start + i - s.edgeStart_[e]]);
    }

    // Weld by grid point: each id maps to the first id with its grid point, as in AppendRegion.
    // The curve edges use the same canonical ids as the loops.
    s.curveEdges_.Clear();
    s.curveEdges_.Reserve(core.curveEdges_.Size());
    s.firstId_.Clear();
    s.firstId_.Reserve(edgeTotal);
    const auto canonical = [&](int id)
    {
        const CsgGridPoint& point = PointGrid(core, id);
        CsgVec3Key key;
        key.x_ = static_cast<uint32_t>(point.x_);
        key.y_ = static_cast<uint32_t>(point.y_);
        key.z_ = static_cast<uint32_t>(point.z_);
        bool existed = false;
        int& slot = s.firstId_.FindOrAdd(key, existed);
        if (!existed)
            slot = id;
        return slot;
    };
    for (int i = 0; i < edgeTotal; ++i)
        s.loops_[i] = canonical(s.loops_[i]);
    for (const auto& entry : core.curveEdges_)
    {
        const int first = canonical(entry.value_.first);
        const int second = canonical(entry.value_.second);
        if (first == second)
            continue;
        bool existed = false;
        auto& slotc = s.curveEdges_.FindOrAdd(UndirectedKey(first, second), existed);
        // Two segments on one edge with opposite sides cancel each other: the edge gets no vote and no twin.
        if (existed && (slotc.first < 0 || slotc.first != first))
            slotc = ea::make_pair(-1, -1);
        else
            slotc = ea::make_pair(first, second);
    }

    // The last element with a directed edge owns it, as in the reference.
    s.edgeOwner_.Clear();
    s.edgeOwner_.Reserve(edgeTotal);
    for (int e = 0; e < elementCount; ++e)
    {
        const int* loop = s.loops_.data() + s.edgeStart_[e];
        const int length = s.edgeStart_[e + 1] - s.edgeStart_[e];
        for (int i = 0; i < length; ++i)
        {
            bool existed = false;
            s.edgeOwner_.FindOrAdd(DirectedKey(loop[i], loop[(i + 1) % length]), existed) = e;
        }
    }

    // The element across each edge off the curve, or -1 for a curve edge or an unmatched one.
    s.twins_.assign(edgeTotal, -1);
    o.inside_.assign(elementCount, 0);
    s.votes_.assign(elementCount, 0);
    for (int e = 0; e < elementCount; ++e)
    {
        const int* loop = s.loops_.data() + s.edgeStart_[e];
        const int length = s.edgeStart_[e + 1] - s.edgeStart_[e];
        const bool split = o.elements_[e].split_;
        int votes = 0;
        for (int i = 0; i < length; ++i)
        {
            const int from = loop[i];
            const int to = loop[(i + 1) % length];
            if (from == to)
                continue;
            // An edge on the curve has no twin, also in an element that has no vote.
            const auto* segment = s.curveEdges_.Find(UndirectedKey(from, to));
            if (segment)
            {
                if (split && segment->first >= 0)
                {
                    // A curve edge shorter than epsilon comes from a near-coincident cut. It gets no vote, so it
                    // cannot outvote the real edges. Its twin stays -1, so the patch does not cross it.
                    if ((PointPosition(core, to) - PointPosition(core, from)).Length() < core.epsilon_)
                        continue;
                    const bool forward = segment->first == from && segment->second == to;
                    votes += forward != second ? 1 : -1;
                }
                continue;
            }
            if (const int* twin = s.edgeOwner_.Find(DirectedKey(to, from)))
            {
                if (*twin != e)
                    s.twins_[s.edgeStart_[e] + i] = *twin;
            }
        }
        s.votes_[e] = votes;
    }

    // Join the elements across the edges off the curve: they form a patch.
    s.parent_.resize(elementCount);
    for (int e = 0; e < elementCount; ++e)
        s.parent_[e] = e;
    const auto find = [&](int e)
    {
        while (s.parent_[e] != e)
        {
            s.parent_[e] = s.parent_[s.parent_[e]];
            e = s.parent_[e];
        }
        return e;
    };
    for (int e = 0; e < elementCount; ++e)
    {
        for (int k = s.edgeStart_[e]; k < s.edgeStart_[e + 1]; ++k)
        {
            if (s.twins_[k] < 0)
                continue;
            const int a = find(e);
            const int b = find(s.twins_[k]);
            if (a != b)
                s.parent_[ea::max(a, b)] = ea::min(a, b);
        }
    }

    // The sum of the votes of all curve edges of a patch decides all its elements.
    s.patchVotes_.assign(elementCount, 0);
    for (int e = 0; e < elementCount; ++e)
        s.patchVotes_[find(e)] += s.votes_[e];

    // A patch with a zero sum gets a point query on one element. In the exact range, an element with an exact area
    // goes before an element with none. The float area of the polygon decides between equal elements.
    s.patchBest_.assign(elementCount, -1);
    s.patchArea_.assign(elementCount, -1.0);
    s.patchExact_.assign(elementCount, 0);
    for (int e = 0; e < elementCount; ++e)
    {
        const int root = find(e);
        if (s.patchVotes_[root] != 0)
            continue;
        const bool exact = HasGridArea(core, s, e);
        const double area = PolygonArea(*o.elements_[e].polygon_);
        if ((exact && !s.patchExact_[root]) || (exact == (s.patchExact_[root] != 0) && area > s.patchArea_[root]))
        {
            s.patchExact_[root] = exact ? 1 : 0;
            s.patchArea_[root] = area;
            s.patchBest_[root] = e;
        }
    }
#ifdef CSG_DIAGNOSTICS
    {
        ea::vector<uint8_t> signs(elementCount, 0);
        for (int e = 0; e < elementCount; ++e)
        {
            if (s.votes_[e] != 0)
                signs[find(e)] |= s.votes_[e] > 0 ? 1 : 2;
        }
        for (int e = 0; e < elementCount; ++e)
        {
            if (find(e) != e)
                continue;
            CSG_DIAG_COUNT(patches_);
            if (signs[e] == 3)
                CSG_DIAG_COUNT(mixedVotePatches_);
            if (s.patchBest_[e] >= 0)
                CSG_DIAG_COUNT(zeroSumPatches_);
        }
    }
#endif
    constexpr int PATCH_REMOVED = -2;
    for (int root = 0; root < elementCount; ++root)
    {
        if (s.patchBest_[root] < 0)
            continue;
        const int best = s.patchBest_[root];
        if (!s.patchExact_[root])
        {
            // No element of the patch has an exact area, so no side of the other operand is right for the patch.
            // The patch is removed. It leaves no kept face. Its curve edges have no net vote and no twin.
            CSG_DIAG_COUNT(zeroSumDegeneratePatches_);
            s.patchBest_[root] = PATCH_REMOVED;
            continue;
        }
        bool inside = false;
        const ExactResult result =
            ClassifyElementExact(core, s, best, *o.elements_[best].polygon_, otherMesh, second, inside);
        if (result != ExactResult::Done)
        {
            // A polygon that does not match its loop, or a triangulation with zero exact area, has no exact test.
            // The patch is removed, as a patch with no element of exact area.
            CSG_DIAG_COUNT(zeroSumUnclassifiedPatches_);
            s.patchBest_[root] = PATCH_REMOVED;
            continue;
        }
        s.patchVotes_[root] = inside ? 1 : -1;
    }
    for (int e = 0; e < elementCount; ++e)
    {
        const int root = find(e);
        if (s.patchBest_[root] == PATCH_REMOVED)
            o.inside_[e] = CSG_ELEMENT_REMOVED;
        else
            o.inside_[e] = s.patchVotes_[root] > 0 ? 1 : 0;
    }
}

bool ClassifyGridPoint(const Mesh& other, const CsgGridPoint& p, int second);

/// The side of a component that the curve does not reach.
enum class ComponentSide
{
    Outside,
    Inside,
    /// The component has no face with a loop, so it has no exact side. The boolean removes it.
    None
};

/// The largest face extent of a mesh. See Mesh::faceExtent_.
double MaxFaceExtent(const Mesh& mesh)
{
    double result = 0.0;
    for (const double extent : mesh.faceExtent_)
        result = ea::max(result, extent);
    return result;
}

/// Tell if a grid point lies within the thickness of the plane of a triangle of a mesh near the point.
/// Only the thick mode uses it: in the exact mode, FlatPlane is always false.
/// The test is conservative. It uses the plane of each triangle with a face box near the point.
/// The maxFaceExtent is the largest face extent of the other mesh. The query margin uses it, as PlaneThickness does.
/// The components, faces and stack are scratch vectors that the caller reuses between queries.
bool InThickSurface(const Mesh& other, const CsgGridPoint& p, double maxFaceExtent, ea::vector<int>& components,
    ea::vector<int>& faces, ea::vector<int>& stack)
{
    const CsgSolid& solid = other.solid_;
    const Vector3 position = p.ToPosition();
    const float margin = static_cast<float>(
        4.0 * CsgExact::Thickness(ea::max(CsgExact::Extent(p), maxFaceExtent)) / CsgGridScale);
    const ea::span<const unsigned> triangleStart = solid.GetTriangleStart();
    solid.GetComponentTree().CollectBox(position, position, margin, components, stack);
    for (const int component : components)
    {
        solid.GetFaceTree(static_cast<unsigned>(component)).CollectBox(position, position, margin, faces, stack);
        for (const int face : faces)
        {
            for (unsigned t = triangleStart[face]; t < triangleStart[face + 1]; ++t)
            {
                const int* ids = &other.triangleIds_[t * 3];
                if (CsgExact::FlatPlane(
                        other.grid_[ids[0]], other.grid_[ids[1]], other.grid_[ids[2]], p, other.faceExtent_[face]))
                    return true;
            }
        }
    }
    return false;
}

/// Tell if a component that the curve does not reach lies inside the other solid.
///
/// The answer is exact. The component has no curve, so it
/// does not cross the displaced surface of the other operand. All its vertices lie on the same side, also the
/// vertices that touch the other surface. The function classifies the first vertex of the first face with a loop.
/// The exact test breaks a tie on a shared face by the displacement of the second operand.
/// In thick mode, the component does not cross the thick surface. A vertex in the thickness of a plane of the other
/// operand can lie on the wrong side of the exact surface. So the function classifies the first vertex clear of
/// the thickness. If no vertex is clear, it classifies the first vertex.
/// If no face of the component has a loop, the component has no side: the function counts it and returns None.
ComponentSide ClassifyComponent(const Corefinement& core, const Mesh& mesh, unsigned component, const Mesh& other, bool second)
{
    const ea::span<const unsigned> componentStart = mesh.solid_.GetComponentStart();
    const ea::span<const unsigned> componentFaces = mesh.solid_.GetComponentFaces();
    int first = -1;
    int chosen = -1;
    ea::vector<int> components;
    ea::vector<int> faces;
    ea::vector<int> stack;
    stack.reserve(64);
    const double maxFaceExtent = MaxFaceExtent(other);
    for (unsigned i = componentStart[component]; i < componentStart[component + 1] && chosen < 0; ++i)
    {
        const unsigned face = componentFaces[i];
        const int length = mesh.loops_.loopLength_[face];
        if (length <= 0)
            continue;
        const unsigned start = mesh.loops_.vertexStart_[face];
        if (first < 0)
            first = mesh.loops_.vertexIds_[start];
        if (g_csgExactOnly)
            break;
        for (int k = 0; k < length && chosen < 0; ++k)
        {
            const int id = mesh.loops_.vertexIds_[start + k];
            if (!InThickSurface(other, mesh.grid_[id], maxFaceExtent, components, faces, stack))
                chosen = id;
        }
    }
    if (first < 0)
    {
        CSG_DIAG_COUNT(componentsRemoved_);
        return ComponentSide::None;
    }
    const int id = chosen >= 0 ? chosen : first;
    // The second operand moves. A vertex of the first operand uses the mask 0b0111, where the other mesh
    // moves. A vertex of the second operand uses the mask 0b1000, where the point moves.
    return ClassifyGridPoint(other, mesh.grid_[id], second ? 0b1000 : 0b0111) ? ComponentSide::Inside :
                                                                                 ComponentSide::Outside;
}

/// Collect the elements of one operand. This function does not classify.
///
/// The components that the curve reaches are split along the curve. Their faces, or the regions of their split
/// triangles, are the elements. ClassifyElements groups the elements into patches between curve edges and labels them.
/// The components that the curve does not reach are not elements. ClassifyAndEmit classifies them whole.
///
/// A triangle that is collapsed is left out of a face that the curve reaches. The reference leaves out only
/// triangles with welded corners. The function does the same.
///
/// A whole face clear of the bounds of the other operand is outside it, and the curve does not reach it.
/// It is not an element, so it gets no classification. If the operation keeps it, it goes to clearFaces_.
void AppendOperand(const Corefinement& core, const Mesh& mesh, const CsgSegmentLists& segments,
    const CsgSplitResult& split, CsgOperation operation, bool second, CsgOperandElements& o, CsgBooleanScratch& s)
{
    const CsgSolid& solid = mesh.solid_;
    const CsgSolid& other = second ? core.a_.solid_ : core.b_.solid_;
    // The margin covers the rounding of the grid. The float bounds are exact on the grid positions.
    const float margin = ea::max(core.epsilon_ * 4.0f, static_cast<float>(4.0 / CsgGridScale));
    const bool keepClear = CsgSolid::Keep(false, operation, second);
    const auto faceMin = solid.GetFaceMin();
    const auto faceMax = solid.GetFaceMax();
    const unsigned componentCount = solid.ComponentCount();
    const auto faceComponents = solid.GetFaceComponents();
    const auto triangleStart = solid.GetTriangleStart();
    const auto triangleVertices = solid.GetTriangleVertices();
    const auto componentStart = solid.GetComponentStart();
    const auto componentFaces = solid.GetComponentFaces();

    o.touched_.assign(componentCount, 0);
    s.frontier_.assign(solid.FaceCount(), 0);
    for (unsigned t = 0; t < mesh.triangleFace_.size(); ++t)
    {
        // A triangle with a crossing on its boundary and no segment is split too.
        if (segments.Count(static_cast<int>(t)) == 0 && split.GetState(static_cast<int>(t)) != CsgSplitResult::State::Split)
            continue;
        const int face = mesh.triangleFace_[t];
        s.frontier_[face] = 1;
        o.touched_[faceComponents[face]] = 1;
    }

    o.elements_.clear();
    o.clearFaces_.clear();
    o.wholeFaces_.clear();
    o.elementIds_.clear();
    for (unsigned component = 0; component < componentCount; ++component)
    {
        if (!o.touched_[component])
            continue;
        for (unsigned c = componentStart[component]; c < componentStart[component + 1]; ++c)
        {
            const int face = static_cast<int>(componentFaces[c]);
            if (mesh.loops_.loopLength_[face] < 3)
                continue;
            if (!s.frontier_[face])
            {
                if (!other.BoundsOverlap(faceMin[face], faceMax[face], margin))
                {
                    if (keepClear)
                        o.clearFaces_.push_back({static_cast<unsigned>(o.elements_.size()), face});
                    continue;
                }
                CsgBooleanElement& element = o.elements_.emplace_back();
                element.polygon_ = solid.GetFace(face);
                element.sourceFace_ = face;
                continue;
            }

            // A face that the curve reaches is output triangle by triangle: split where the curve runs, whole elsewhere.
            const CsgPolygon& source = *solid.GetFace(face);
            for (unsigned t = triangleStart[face]; t < triangleStart[face + 1]; ++t)
            {
                if (mesh.IsCollapsed(static_cast<int>(t)))
                    continue;
                if (split.GetState(static_cast<int>(t)) == CsgSplitResult::State::Split)
                {
                    const size_t elementMark = o.elements_.size();
                    const size_t idMark = o.elementIds_.size();
                    bool ok = true;
                    for (int r = split.triangleRegionStart_[t]; r < split.triangleRegionStart_[t + 1]; ++r)
                    {
                        CsgPolygon polygon;
                        if (!RegionPolygon(core, mesh, face, split.RegionIds(r), split.RegionSize(r), polygon))
                        {
                            ok = false;
                            break;
                        }
                        CsgBooleanElement& element = o.elements_.emplace_back();
                        element.polygon_ = ea::make_shared<CsgPolygon>(ea::move(polygon));
                        element.idStart_ = static_cast<int>(o.elementIds_.size());
                        element.idCount_ = split.RegionSize(r);
                        element.split_ = true;
                        o.elementIds_.insert(o.elementIds_.end(), split.RegionIds(r),
                            split.RegionIds(r) + split.RegionSize(r));
                    }
                    if (ok)
                        continue;
                    // A region does not fit its face. The triangle stays whole, as for a failed split.
                    CSG_DIAG_COUNT(failedRegionFits_);
                    o.elements_.resize(elementMark);
                    o.elementIds_.resize(idMark);
                }

                ea::vector<ModelVertex> corners;
                corners.reserve(3);
                CsgBooleanElement& element = o.elements_.emplace_back();
                element.idStart_ = static_cast<int>(o.elementIds_.size());
                element.idCount_ = 3;
                for (unsigned k = 0; k < 3; ++k)
                {
                    ModelVertex corner = source.vertices_[triangleVertices[t * 3 + k]];
                    const int cornerId = core.Global(mesh, mesh.triangleIds_[t * 3 + k]);
                    // One id has one position: that of the first operand.
                    if (core.IsAlias(mesh, cornerId))
                        corner.SetPosition(PointPosition(core, cornerId));
                    corners.push_back(corner);
                    o.elementIds_.push_back(cornerId);
                }
                auto polygon = ea::make_shared<CsgPolygon>(ea::move(corners), source.shared_);
                polygon->plane_ = source.plane_;
                element.polygon_ = ea::move(polygon);
            }
        }
    }
    o.inside_.clear();
}

/// Make the list of kept faces from the labels of both operands. The first operand goes first.
/// The faces of a flipped operand get the flipped polygon and the reversed ids.
/// A kept face clear of the other operand keeps its place among the elements.
void BuildKeptSet(const Corefinement& core, CsgOperation operation, CsgBooleanScratch& s)
{
    s.kept_.clear();
    s.keptIds_.clear();
    for (int index = 0; index < 2; ++index)
    {
        const bool second = index != 0;
        const bool flip = second && operation == CsgOperation::DifferenceAB;
        const CsgOperandElements& o = s.operands_[index];
        const CsgSolid& solid = second ? core.b_.solid_ : core.a_.solid_;
        unsigned nextClear = 0;
        for (const int face : o.wholeFaces_)
        {
            CsgKeptFace& kept = s.kept_.emplace_back();
            kept.polygon_ = flip ? CsgSolid::Flipped(*solid.GetFace(face)) : solid.GetFace(face);
            kept.sourceFace_ = face;
            kept.second_ = second;
            kept.flip_ = flip;
        }
        for (unsigned i = 0; i <= o.elements_.size(); ++i)
        {
            for (; nextClear < o.clearFaces_.size() && o.clearFaces_[nextClear].first == i; ++nextClear)
            {
                const int face = o.clearFaces_[nextClear].second;
                CsgKeptFace& kept = s.kept_.emplace_back();
                kept.polygon_ = flip ? CsgSolid::Flipped(*solid.GetFace(face)) : solid.GetFace(face);
                kept.sourceFace_ = face;
                kept.second_ = second;
                kept.flip_ = flip;
            }
            if (i == o.elements_.size())
                break;
            if (o.inside_[i] == CSG_ELEMENT_REMOVED || !CsgSolid::Keep(o.inside_[i] != 0, operation, second))
                continue;
            const CsgBooleanElement& element = o.elements_[i];
            CsgKeptFace& kept = s.kept_.emplace_back();
            kept.polygon_ = flip ? CsgSolid::Flipped(*element.polygon_) : element.polygon_;
            kept.sourceFace_ = element.sourceFace_;
            kept.second_ = second;
            kept.flip_ = flip;
            if (element.sourceFace_ >= 0)
                continue;
            kept.idStart_ = static_cast<int>(s.keptIds_.size());
            kept.idCount_ = element.idCount_;
            const int* ids = o.elementIds_.data() + element.idStart_;
            for (int k = 0; k < element.idCount_; ++k)
                s.keptIds_.push_back(ids[flip ? element.idCount_ - 1 - k : k]);
        }
    }
}

/// Return true if the second loop visits the vertices of the first loop in the opposite direction.
bool IsReverse(const int* x, const int* y, int n)
{
    int start = -1;
    for (int i = 0; i < n; ++i)
    {
        if (y[i] == x[0])
        {
            start = i;
            break;
        }
    }
    if (start < 0)
        return false;
    for (int k = 0; k < n; ++k)
    {
        if (y[(start - k + n) % n] != x[k])
            return false;
    }
    return true;
}

/// A corner of a cleaned loop. An original corner has the index of its polygon vertex in from_, and to_ is -1.
/// A corner that a T-junction split adds lies on the polygon edge from the vertex from_ to the vertex to_.
struct LoopCorner
{
    int id_;
    int from_;
    int to_;
    CsgGridPoint grid_;
};

/// Remove the repeated ids and the zero-width spikes of a loop.
/// A repeated id is equal to the id before it, also from the last id to the first one.
/// A spike is an id with equal ids before and after it: the loop goes to the spike and back on one edge.
/// The pass starts again from the first corner after each removal. A loop without removals costs one pass.
/// The removal order decides which copy of a repeated corner stays. The later float steps depend on this choice,
/// so the order must stay the same.
void NormalizeLoop(ea::vector<LoopCorner>& loop)
{
    bool changed = true;
    while (changed)
    {
        changed = false;
        const int count = static_cast<int>(loop.size());
        for (int i = 0; i < count && count > 1; ++i)
        {
            const int next = (i + 1) % count;
            if (loop[i].id_ == loop[next].id_)
            {
                loop.erase(loop.begin() + next);
                changed = true;
                break;
            }
            const int previous = (i + count - 1) % count;
            if (count >= 3 && loop[previous].id_ == loop[next].id_)
            {
                // Remove the spike. The next id is then a repeat of the previous id and goes on the next pass.
                loop.erase(loop.begin() + i);
                changed = true;
                break;
            }
        }
    }
}

/// Return true if the exact grid point lies inside the projected loop corners [begin, end) or on its boundary.
bool ContainsProjected(const ea::vector<LoopCorner>& loop, unsigned begin, unsigned end, const CsgGridPoint2& point,
    int dropAxis)
{
    bool inside = false;
    for (unsigned i = begin, j = end - 1; i < end; j = i++)
    {
        const CsgGridPoint2 p = CsgExact::Project(loop[i].grid_, dropAxis, false);
        const CsgGridPoint2 q = CsgExact::Project(loop[j].grid_, dropAxis, false);
        const int side = CsgExact::Orient2(q, p, point);
        if (side == 0 && ea::min(p.u_, q.u_) <= point.u_ && point.u_ <= ea::max(p.u_, q.u_)
            && ea::min(p.v_, q.v_) <= point.v_ && point.v_ <= ea::max(p.v_, q.v_))
            return true;
        if ((p.v_ > point.v_) != (q.v_ > point.v_) && ((p.v_ > q.v_) ? side > 0 : side < 0))
            inside = !inside;
    }
    return inside;
}

/// Return true if the loop is a region with a hole that touches it at the repeated id of the positions first and
/// second. Then the loop is weakly simple, and a split would make the hole a separate face of the wrong winding.
/// One lobe turns with the face and the other lobe turns against it. Each corner of the hole lobe lies inside the
/// region lobe or on its boundary. The tests are exact on the grid points, projected along the main axis of
/// the normal. Two lobes side by side, or a twisted sliver, fail the tests and split.
bool IsTouchingHole(const ea::vector<LoopCorner>& loop, unsigned first, unsigned second, const Vector3& normal)
{
    const Vector3 a = normal.Abs();
    const int dropAxis = a.x_ >= a.y_ && a.x_ >= a.z_ ? 0 : (a.y_ >= a.z_ ? 1 : 2);
    const float along = dropAxis == 0 ? normal.x_ : (dropAxis == 1 ? normal.y_ : normal.z_);
    if (along == 0.0f || !std::isfinite(along))
        return false;
    const int expected = along > 0.0f ? 1 : -1;

    // The inner lobe is [first, second). The outer lobe is [second, size) + [0, first). The copy rotates it.
    const unsigned size = loop.size();
    ea::vector<CsgGridPoint2> points;
    points.reserve(size);
    for (unsigned k = first; k < second; ++k)
        points.push_back(CsgExact::Project(loop[k].grid_, dropAxis, false));
    const int innerSign = CsgExact::LoopAreaSign(points.data(), static_cast<int>(points.size()));
    points.clear();
    ea::vector<LoopCorner> outer(loop.begin() + second, loop.end());
    outer.insert(outer.end(), loop.begin(), loop.begin() + first);
    for (const LoopCorner& corner : outer)
        points.push_back(CsgExact::Project(corner.grid_, dropAxis, false));
    const int outerSign = CsgExact::LoopAreaSign(points.data(), static_cast<int>(points.size()));
    if (innerSign * outerSign != -1)
        return false;

    const bool innerIsHole = innerSign != expected;
    const ea::vector<LoopCorner> inner(loop.begin() + first, loop.begin() + second);
    const ea::vector<LoopCorner>& hole = innerIsHole ? inner : outer;
    const ea::vector<LoopCorner>& region = innerIsHole ? outer : inner;
    for (const LoopCorner& corner : hole)
    {
        if (!ContainsProjected(region, 0, region.size(), CsgExact::Project(corner.grid_, dropAxis, false), dropAxis))
            return false;
    }
    return true;
}

/// Find the first two positions first < second with one id. Return false if all ids are distinct.
/// A pair where `keep(first, second)` is true is not a match.
template <class Keep>
bool FindRepeatedId(
    const ea::vector<LoopCorner>& loop, ea::vector<int>& sorted, unsigned& first, unsigned& second, Keep&& keep)
{
    const unsigned size = loop.size();
    if (size > 16)
    {
        sorted.resize(size);
        for (unsigned i = 0; i < size; ++i)
            sorted[i] = loop[i].id_;
        ea::sort(sorted.begin(), sorted.end());
        if (ea::adjacent_find(sorted.begin(), sorted.end()) == sorted.end())
            return false;
    }
    for (unsigned i = 0; i < size; ++i)
    {
        for (unsigned j = i + 1; j < size; ++j)
        {
            if (loop[i].id_ == loop[j].id_ && !keep(i, j))
            {
                first = i;
                second = j;
                return true;
            }
        }
    }
    return false;
}

/// Return true if all grid points of the loop are on one line. Such a loop bounds no surface.
/// A zero normal is not sufficient: a sliver that the snap twists has two lobes of opposite sign and a zero normal.
bool IsOnOneLine(const ea::vector<LoopCorner>& loop)
{
    static_assert(2 * (CsgGridBits + 1) + 2 < 63, "The products of grid differences must fit in 64 bits.");
    const CsgGridPoint& u = loop[0].grid_;
    unsigned far = 1;
    while (far < loop.size() && loop[far].grid_ == u)
        ++far;
    if (far == loop.size())
        return true;
    const CsgGridPoint& v = loop[far].grid_;
    const int64_t ex = int64_t{v.x_} - u.x_, ey = int64_t{v.y_} - u.y_, ez = int64_t{v.z_} - u.z_;
    for (unsigned i = far + 1; i < loop.size(); ++i)
    {
        const CsgGridPoint& w = loop[i].grid_;
        const int64_t dx = int64_t{w.x_} - u.x_, dy = int64_t{w.y_} - u.y_, dz = int64_t{w.z_} - u.z_;
        if (ey * dz != ez * dy || ez * dx != ex * dz || ex * dy != ey * dx)
            return false;
    }
    return true;
}

/// Return the place of the grid point w along the open segment from u to v, as the dot product (w - u) . (v - u).
/// Return 0 if w is not exactly on the segment between its ends. The products fit in 64 bits, see CsgGridBits.
int64_t PlaceOnSegment(const CsgGridPoint& u, const CsgGridPoint& v, const CsgGridPoint& w)
{
    static_assert(2 * (CsgGridBits + 1) + 2 < 63, "The products of grid differences must fit in 64 bits.");
    const int64_t ex = int64_t{v.x_} - u.x_, ey = int64_t{v.y_} - u.y_, ez = int64_t{v.z_} - u.z_;
    const int64_t dx = int64_t{w.x_} - u.x_, dy = int64_t{w.y_} - u.y_, dz = int64_t{w.z_} - u.z_;
    if (ey * dz != ez * dy || ez * dx != ex * dz || ex * dy != ey * dx)
        return 0;
    const int64_t place = dx * ex + dy * ey + dz * ez;
    const int64_t length = ex * ex + ey * ey + ez * ez;
    return place > 0 && place < length ? place : 0;
}

/// Split each edge of a loop at the corners of the same loop that are exactly on the edge. The loop of a split region
/// can run along one line twice: once on an edge of its triangle, and once back on curve segments whose ends are on
/// that edge. After the split, the clean-up sees the repeated ids and removes the zero-width part. The split does not
/// change the shape. The corners are sorted by x, so an edge tests only the corners in its x range.
/// Return true if an edge splits. The loop is then replaced by the split loop.
bool SplitAtOwnCorners(ea::vector<LoopCorner>& loop, ea::vector<unsigned>& byX, ea::vector<LoopCorner>& output)
{
    const unsigned count = loop.size();
    if (count < 4)
        return false;
    byX.resize(count);
    for (unsigned i = 0; i < count; ++i)
        byX[i] = i;
    ea::sort(byX.begin(), byX.end(), [&](unsigned x, unsigned y) { return loop[x].grid_.x_ < loop[y].grid_.x_; });
    struct Cut
    {
        int64_t place_;
        unsigned corner_;
    };
    ea::fixed_vector<Cut, 8> cuts;
    bool split = false;
    output.clear();
    for (unsigned k = 0; k < count; ++k)
    {
        const LoopCorner& from = loop[k];
        const LoopCorner& to = loop[(k + 1) % count];
        output.push_back(from);
        if (from.id_ == to.id_)
            continue;
        const CsgGridInt low = ea::min(from.grid_.x_, to.grid_.x_);
        const CsgGridInt high = ea::max(from.grid_.x_, to.grid_.x_);
        auto it = ea::lower_bound(
            byX.begin(), byX.end(), low, [&](unsigned x, CsgGridInt value) { return loop[x].grid_.x_ < value; });
        cuts.clear();
        for (; it != byX.end() && loop[*it].grid_.x_ <= high; ++it)
        {
            const LoopCorner& corner = loop[*it];
            if (corner.id_ == from.id_ || corner.id_ == to.id_)
                continue;
            if (const int64_t place = PlaceOnSegment(from.grid_, to.grid_, corner.grid_))
                cuts.push_back({place, *it});
        }
        if (cuts.empty())
            continue;
        ea::sort(cuts.begin(), cuts.end(), [](const Cut& x, const Cut& y) { return x.place_ < y.place_; });
        for (const Cut& cut : cuts)
            output.push_back({loop[cut.corner_].id_, from.from_, to.from_, loop[cut.corner_].grid_});
        split = true;
    }
    if (split)
        loop.swap(output);
    return split;
}

/// A loop of a cleaned face. The loop owns the corners [start_, start_ + count_) in loopIds_ and loopCorners_.
struct CsgCleanLoop
{
    /// The index of the kept face of the loop.
    unsigned face_;
    unsigned start_;
    unsigned count_;
    uint64_t hash_;
};

/// An edge of the cleaned faces that has no opposite partner. The winding is the sum of the directions of its uses.
struct CsgOpenEdge
{
    /// The ids of the ends, with from_ < to_.
    int from_;
    int to_;
    int winding_;
};

/// The result of Clean: the cleaned loops of the kept faces. Emit builds the solid from it.
/// A caller can read the open edges and then call Clean again with another list of kept faces.
struct CsgRegionClean
{
    /// The ids and the grid points of the corners of each kept face, in the order of its polygon.
    ea::vector<int> cornerIds_;
    ea::vector<CsgGridPoint> cornerGrid_;
    /// Face f owns the corners [cornerStart_[f], cornerStart_[f + 1]).
    ea::vector<unsigned> cornerStart_;
    /// The cleaned loops. A loop that a T-split replaces stays here, marked removed.
    ea::vector<CsgCleanLoop> loops_;
    ea::vector<int> loopIds_;
    ea::vector<LoopCorner> loopCorners_;
    ea::vector<uint8_t> removed_;
    /// The number of loops before the T-split. The loops from this index on are the replacements.
    unsigned cleanCount_{};
    /// For each loop before the T-split, the range of its replacement loops. It is empty if no loop split.
    ea::vector<ea::pair<unsigned, unsigned>> replacements_;
    /// The edges with a winding that is not zero, before the T-split.
    ea::vector<CsgOpenEdge> openEdges_;
};

/// Clean the kept faces and write the loops to the result. The result is reset first. No solid is built.
/// A split region reads its exact ids from the ids of the kept faces. An unsplit face reads its ids from its solid,
/// offset like the mesh.
/// Each corner gets the grid point of its id: a crossing gives its rounded grid point, a kept vertex the grid
/// point that its solid stores. So the result keeps the grid points and does not snap a position.
///
/// The faces are welded and cleaned exactly, by grid point and id:
/// - ids with equal grid points become one id, the first id with that grid point;
/// - a loop loses its repeated ids and its zero-width spikes;
/// - an edge of a loop splits at the corners of the same loop that are exactly on it;
/// - a loop that passes twice through one id splits into two loops at that id;
/// - a face with less than three ids or with all corners on one line bounds no surface and is left out;
/// - two faces that visit the same ids in opposite directions cancel;
/// - an edge without a partner splits at the vertices exactly on it. Then its loop is cleaned again,
///   and the twins cancel again, because a twin pair can show only after the split.
void Clean(const Corefinement& core, const ea::vector<CsgKeptFace>& kept, const ea::vector<int>& keptIds,
    CsgRegionClean& out)
{
    using Loop = CsgCleanLoop;
    ea::vector<int>& cornerIds = out.cornerIds_;
    ea::vector<CsgGridPoint>& cornerGrid = out.cornerGrid_;
    ea::vector<unsigned>& cornerStart = out.cornerStart_;
    ea::vector<Loop>& loops = out.loops_;
    ea::vector<int>& loopIds = out.loopIds_;
    ea::vector<LoopCorner>& loopCorners = out.loopCorners_;
    ea::vector<uint8_t>& removed = out.removed_;
    ea::vector<ea::pair<unsigned, unsigned>>& replacements = out.replacements_;
    cornerIds.clear();
    cornerGrid.clear();
    cornerStart.clear();
    loops.clear();
    loopIds.clear();
    loopCorners.clear();
    removed.clear();
    replacements.clear();
    out.openEdges_.clear();

    // The ids and the grid points of the corners of each kept face, in the order of its polygon.
    cornerStart.reserve(kept.size() + 1);
    for (const CsgKeptFace& face : kept)
    {
        cornerStart.push_back(cornerIds.size());
        if (face.sourceFace_ >= 0)
        {
            const Mesh& mesh = face.second_ ? core.b_ : core.a_;
            const unsigned start = mesh.loops_.vertexStart_[face.sourceFace_];
            const int count = static_cast<int>(mesh.loops_.vertexStart_[face.sourceFace_ + 1] - start);
            for (int k = 0; k < count; ++k)
            {
                const int id = mesh.loops_.ids_[start + (face.flip_ ? count - 1 - k : k)];
                cornerIds.push_back(core.Global(mesh, id));
                cornerGrid.push_back(mesh.grid_[id]);
            }
        }
        else
        {
            for (int k = face.idStart_; k < face.idStart_ + face.idCount_; ++k)
            {
                const int id = keptIds[k];
                cornerIds.push_back(id);
                cornerGrid.push_back(PointGrid(core, id));
            }
        }
        URHO3D_ASSERT(cornerIds.size() - cornerStart.back() == face.polygon_->vertices_.size(),
            "A kept face must have one id for each vertex");
    }
    cornerStart.push_back(cornerIds.size());

    // Weld by grid point: each id maps to the first id with its grid point.
    CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash> firstId;
    firstId.Reserve(cornerIds.size());
    for (unsigned i = 0; i < cornerIds.size(); ++i)
    {
        const CsgGridPoint& point = cornerGrid[i];
        CsgVec3Key key;
        key.x_ = static_cast<uint32_t>(point.x_);
        key.y_ = static_cast<uint32_t>(point.y_);
        key.z_ = static_cast<uint32_t>(point.z_);
        bool existed = false;
        int& slot = firstId.FindOrAdd(key, existed);
        if (!existed)
            slot = cornerIds[i];
        cornerIds[i] = slot;
    }

    // Clean the loops. A loop with no surface is left out, and a loop that is not simple splits.
    loops.reserve(kept.size());
    loopIds.reserve(cornerIds.size());
    loopCorners.reserve(cornerIds.size());
    removed.reserve(kept.size());
    ea::vector<LoopCorner> piece;
    ea::vector<LoopCorner> pool;
    ea::vector<ea::pair<unsigned, unsigned>> ranges;
    ea::vector<int> sorted;
    ea::vector<unsigned> byX;
    ea::vector<LoopCorner> splitPiece;
    // The input loop is in piece. The clean simple loops go to the end of loops.
    const auto appendClean = [&](unsigned face)
    {
        pool.assign(piece.begin(), piece.end());
        ranges.clear();
        ranges.push_back({0u, static_cast<unsigned>(pool.size())});
        while (!ranges.empty())
        {
            const auto range = ranges.back();
            ranges.pop_back();
            piece.assign(pool.begin() + range.first, pool.begin() + range.first + range.second);
            NormalizeLoop(piece);
            if (piece.size() < 3)
                continue;
            // A loop that passes twice through one id is not simple. It splits at that id into two loops.
            // A hole that touches its region at the id stays in the loop: the loop is then weakly simple.
            unsigned first = 0;
            unsigned second = 0;
            const Vector3& normal = kept[face].polygon_->plane_.normal_;
            if (FindRepeatedId(piece, sorted, first, second,
                    [&](unsigned i, unsigned j) { return IsTouchingHole(piece, i, j, normal); }))
            {
                const unsigned inner = pool.size();
                pool.insert(pool.end(), piece.begin() + first, piece.begin() + second);
                ranges.push_back({inner, second - first});
                const unsigned outer = pool.size();
                pool.insert(pool.end(), piece.begin(), piece.begin() + first);
                pool.insert(pool.end(), piece.begin() + second, piece.end());
                ranges.push_back({outer, static_cast<unsigned>(piece.size()) - (second - first)});
                continue;
            }
            if (IsOnOneLine(piece))
                continue;
            uint64_t hash = 0;
            for (const LoopCorner& corner : piece)
            {
                const uint64_t key = static_cast<uint64_t>(static_cast<uint32_t>(corner.id_));
                hash += (key + 0x9E3779B97F4A7C15ULL) * 0xBF58476D1CE4E5B9ULL;
            }
            loops.push_back({face, static_cast<unsigned>(loopIds.size()), static_cast<unsigned>(piece.size()), hash});
            removed.push_back(0);
            for (const LoopCorner& corner : piece)
            {
                loopIds.push_back(corner.id_);
                loopCorners.push_back(corner);
            }
        }
    };
    for (unsigned f = 0; f < kept.size(); ++f)
    {
        piece.clear();
        for (unsigned k = cornerStart[f]; k < cornerStart[f + 1]; ++k)
            piece.push_back({cornerIds[k], static_cast<int>(k - cornerStart[f]), -1, cornerGrid[k]});
        SplitAtOwnCorners(piece, byX, splitPiece);
        appendClean(f);
    }

    // Two loops with the same ids in opposite directions cancel. The order of the loops decides the pairs.
    ea::vector<int> order;
    const auto cancelTwins = [&]()
    {
        order.clear();
        for (unsigned i = 0; i < loops.size(); ++i)
        {
            if (!removed[i])
                order.push_back(static_cast<int>(i));
        }
        ea::sort(order.begin(), order.end(), [&](int x, int y)
        {
            if (loops[x].count_ != loops[y].count_)
                return loops[x].count_ < loops[y].count_;
            if (loops[x].hash_ != loops[y].hash_)
                return loops[x].hash_ < loops[y].hash_;
            return x < y;
        });
        unsigned begin = 0;
        while (begin < order.size())
        {
            unsigned end = begin + 1;
            while (end < order.size() && loops[order[end]].count_ == loops[order[begin]].count_
                && loops[order[end]].hash_ == loops[order[begin]].hash_)
                ++end;
            for (unsigned i = begin; i < end; ++i)
            {
                const Loop& x = loops[order[i]];
                if (removed[order[i]])
                    continue;
                for (unsigned j = i + 1; j < end; ++j)
                {
                    const Loop& y = loops[order[j]];
                    if (removed[order[j]]
                        || !IsReverse(loopIds.data() + x.start_, loopIds.data() + y.start_, static_cast<int>(x.count_)))
                        continue;
                    removed[order[i]] = removed[order[j]] = 1;
                    break;
                }
            }
            begin = end;
        }
    };
    cancelTwins();

    // The loops that replace a split loop. The result keeps them at the place of that loop, so the face order
    // does not change.
    const unsigned cleanCount = loops.size();
    out.cleanCount_ = cleanCount;

    // An edge without an opposite partner can pass exactly through a vertex of another loop: a T-junction.
    // The edge is split at such vertices. The test is exact on the grid points, and only the ends of the edges
    // without a partner are candidates. The candidates are sorted by x, so an edge tests only the candidates
    // in its x range. A loop with a split is cleaned again, and then the twins cancel again.
    CsgFlatMap<uint64_t, int, CsgPackedKeyHash> winding;
    winding.Reserve(loopIds.size());
    for (unsigned l = 0; l < loops.size(); ++l)
    {
        if (removed[l])
            continue;
        const Loop& loop = loops[l];
        for (unsigned k = 0; k < loop.count_; ++k)
        {
            const int from = loopIds[loop.start_ + k];
            const int to = loopIds[loop.start_ + (k + 1) % loop.count_];
            bool existed = false;
            int& sum = winding.FindOrAdd(CsgUndirectedKey(from, to), existed);
            if (!existed)
                sum = 0;
            sum += from < to ? 1 : -1;
        }
    }
    for (const auto& entry : winding)
    {
        if (entry.value_ != 0)
        {
            out.openEdges_.push_back(
                {static_cast<int>(entry.key_ >> 32), static_cast<int>(entry.key_ & 0xFFFFFFFFu), entry.value_});
        }
    }
    const auto isOpen = [&](int from, int to) { return *winding.Find(CsgUndirectedKey(from, to)) != 0; };
    ea::vector<ea::pair<int, CsgGridPoint>> candidates;
    {
        CsgFlatMap<uint64_t, int, CsgPackedKeyHash> seen;
        for (unsigned l = 0; l < loops.size(); ++l)
        {
            if (removed[l])
                continue;
            const Loop& loop = loops[l];
            for (unsigned k = 0; k < loop.count_; ++k)
            {
                const unsigned next = (k + 1) % loop.count_;
                if (!isOpen(loopIds[loop.start_ + k], loopIds[loop.start_ + next]))
                    continue;
                for (const unsigned end : {k, next})
                {
                    const LoopCorner& corner = loopCorners[loop.start_ + end];
                    bool existed = false;
                    seen.FindOrAdd(static_cast<uint64_t>(static_cast<uint32_t>(corner.id_)), existed);
                    if (!existed)
                        candidates.push_back({corner.id_, corner.grid_});
                }
            }
        }
    }
    ea::sort(candidates.begin(), candidates.end(),
        [](const ea::pair<int, CsgGridPoint>& x, const ea::pair<int, CsgGridPoint>& y)
    { return x.second.x_ != y.second.x_ ? x.second.x_ < y.second.x_ : x.first < y.first; });

    struct Cut
    {
        int64_t place_;
        int id_;
        CsgGridPoint grid_;
    };
    ea::vector<Cut> cuts;
    bool anyCut = false;
    const unsigned loopCount = loops.size();
    for (unsigned l = 0; l < loopCount && !candidates.empty(); ++l)
    {
        if (removed[l])
            continue;
        const Loop loop = loops[l];
        bool split = false;
        piece.clear();
        for (unsigned k = 0; k < loop.count_; ++k)
        {
            const unsigned next = (k + 1) % loop.count_;
            const LoopCorner& fromCorner = loopCorners[loop.start_ + k];
            const LoopCorner& toCorner = loopCorners[loop.start_ + next];
            piece.push_back(fromCorner);
            if (!isOpen(fromCorner.id_, toCorner.id_))
                continue;
            const CsgGridInt low = ea::min(fromCorner.grid_.x_, toCorner.grid_.x_);
            const CsgGridInt high = ea::max(fromCorner.grid_.x_, toCorner.grid_.x_);
            auto it = ea::lower_bound(candidates.begin(), candidates.end(), low,
                [](const ea::pair<int, CsgGridPoint>& x, CsgGridInt value) { return x.second.x_ < value; });
            cuts.clear();
            for (; it != candidates.end() && it->second.x_ <= high; ++it)
            {
                if (it->first == fromCorner.id_ || it->first == toCorner.id_)
                    continue;
                if (const int64_t place = PlaceOnSegment(fromCorner.grid_, toCorner.grid_, it->second))
                    cuts.push_back({place, it->first, it->second});
            }
            if (cuts.empty())
                continue;
            ea::sort(cuts.begin(), cuts.end(), [](const Cut& x, const Cut& y) { return x.place_ < y.place_; });
            for (const Cut& cut : cuts)
                piece.push_back({cut.id_, fromCorner.from_, toCorner.from_, cut.grid_});
            split = true;
        }
        if (!split)
            continue;
        removed[l] = 1;
        anyCut = true;
        if (replacements.empty())
            replacements.assign(cleanCount, {0u, 0u});
        const unsigned first = loops.size();
        appendClean(loop.face_);
        replacements[l] = {first, static_cast<unsigned>(loops.size())};
    }
    if (anyCut)
        cancelTwins();

}

/// Give the cleaned faces to the builder. A face that loses or gets a corner gets a new polygon and is
/// triangulated again. Give the same kept faces as the last call to Clean.
void Emit(const Corefinement& core, const ea::vector<CsgKeptFace>& kept, const CsgRegionClean& clean,
    const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b, CsgSolid::Builder& result)
{
    using Loop = CsgCleanLoop;
    const ea::vector<CsgGridPoint>& cornerGrid = clean.cornerGrid_;
    const ea::vector<unsigned>& cornerStart = clean.cornerStart_;
    const ea::vector<Loop>& loops = clean.loops_;
    const ea::vector<int>& loopIds = clean.loopIds_;
    const ea::vector<LoopCorner>& loopCorners = clean.loopCorners_;
    const ea::vector<uint8_t>& removed = clean.removed_;
    const ea::vector<ea::pair<unsigned, unsigned>>& replacements = clean.replacements_;
    const unsigned cleanCount = clean.cleanCount_;

    ea::vector<CsgSharedPolygon> faces;
    ea::vector<CsgFaceOrigin> origins;
    ea::vector<int> exactIds;
    ea::vector<CsgGridPoint> exactGrid;
    ea::vector<ModelVertex> vertices;
    faces.reserve(loops.size());
    origins.reserve(loops.size());
    exactIds.reserve(loopIds.size());
    exactGrid.reserve(loopIds.size());
    const auto emit = [&](unsigned l)
    {
        if (removed[l])
            return;
        const Loop& loop = loops[l];
        const CsgKeptFace& face = kept[loop.face_];
        const CsgPolygon& polygon = *face.polygon_;
        // Only removals change the count of a loop without a split. So a loop with all corners is the polygon.
        bool whole = loop.count_ == polygon.vertices_.size();
        for (unsigned k = 0; k < loop.count_; ++k)
        {
            const LoopCorner& corner = loopCorners[loop.start_ + k];
            exactIds.push_back(corner.id_);
            exactGrid.push_back(corner.grid_);
            whole &= corner.to_ < 0;
        }
        CsgFaceOrigin& origin = origins.emplace_back();
        if (whole)
        {
            faces.push_back(face.polygon_);
            if (face.sourceFace_ >= 0)
            {
                origin.solid_ = face.second_ ? b : a;
                origin.face_ = static_cast<unsigned>(face.sourceFace_);
                origin.flip_ = face.flip_;
            }
            return;
        }
        vertices.clear();
        for (unsigned k = 0; k < loop.count_; ++k)
        {
            const LoopCorner& corner = loopCorners[loop.start_ + k];
            if (corner.to_ < 0)
            {
                vertices.push_back(polygon.vertices_[corner.from_]);
                continue;
            }
            // A split corner takes the attributes along its polygon edge, and its position from its grid point.
            const unsigned base = cornerStart[loop.face_];
            const Vector3 fromPosition = cornerGrid[base + corner.from_].ToPosition();
            const Vector3 span = cornerGrid[base + corner.to_].ToPosition() - fromPosition;
            const Vector3 position = corner.grid_.ToPosition();
            const float spanLengthSquared = span.LengthSquared();
            const float t =
                spanLengthSquared > 0.0f ? (position - fromPosition).DotProduct(span) / spanLengthSquared : 0.5f;
            ModelVertex vertex = LerpModelVertex(polygon.vertices_[corner.from_], polygon.vertices_[corner.to_], t);
            vertex.SetPosition(position);
            vertices.push_back(vertex);
        }
        auto copy = ea::make_shared<CsgPolygon>(ea::move(vertices), polygon.shared_);
        copy->plane_ = polygon.plane_;
        faces.push_back(ea::move(copy));
        vertices = {};
    };
    for (unsigned l = 0; l < cleanCount; ++l)
    {
        if (replacements.empty() || replacements[l].first == replacements[l].second)
        {
            emit(l);
            continue;
        }
        for (unsigned r = replacements[l].first; r < replacements[l].second; ++r)
            emit(r);
    }
    const int idSpace = core.CrossingBase() + static_cast<int>(core.crossings_.size());
    result.AppendRegion(ea::move(faces), origins, exactIds, idSpace, exactGrid);
}

/// Append the kept faces to the builder: the exact clean-up and then the append. The builder groups the faces.
void AppendRegion(const Corefinement& core, const CsgBooleanScratch& s, const ea::vector<CsgKeptFace>& kept,
    const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b, CsgSolid::Builder& result)
{
    CsgRegionClean clean;
    Clean(core, kept, s.keptIds_, clean);
    Emit(core, kept, clean, a, b, result);
}

/// Label the elements of both operands, make the kept set and build the result.
///
/// A component that the curve does not reach is classified whole. A component with no side is removed.
/// A kept component goes to the builder. The first
/// operand goes first. The elements of the other components get the labels of ClassifyElements. The kept faces then go through
/// the exact clean-up. The result path has no float repair. If the exact surface is not closed, the result reports
/// IsClosed() false.
void ClassifyAndEmit(const Corefinement& core, const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b,
    CsgOperation operation, float epsilon, CsgSolid::Builder& result, CsgBooleanScratch& s)
{
    for (int index = 0; index < 2; ++index)
    {
        const bool second = index != 0;
        const Mesh& mesh = second ? core.b_ : core.a_;
        const CsgSolid& solid = mesh.solid_;
        const Mesh& otherMesh = second ? core.a_ : core.b_;
        CsgOperandElements& o = s.operands_[index];
        const bool flip = second && operation == CsgOperation::DifferenceAB;
        for (unsigned component = 0; component < solid.ComponentCount(); ++component)
        {
            if (o.touched_[component])
                continue;
            const ComponentSide side = ClassifyComponent(core, mesh, component, otherMesh, second);
            if (side == ComponentSide::None)
                continue;
            if (!CsgSolid::Keep(side == ComponentSide::Inside, operation, second))
                continue;
            // A face on the vertices of a face of the other operand can cancel against it. The component then
            // goes with the kept faces, where the two faces meet. The exact pass has no aliases: it tests the
            // grid points of the other operand.
            const auto componentStart = solid.GetComponentStart();
            const auto componentFaces = solid.GetComponentFaces();
            bool meets = false;
            for (unsigned c = componentStart[component]; c < componentStart[component + 1] && !meets; ++c)
                meets = core.SharedLoop(mesh, static_cast<int>(componentFaces[c]), flip)
                    || core.OnOtherVertices(mesh, static_cast<int>(componentFaces[c]));
            if (!meets)
            {
                result.AppendComponent(solid, component, flip);
                continue;
            }
            for (unsigned c = componentStart[component]; c < componentStart[component + 1]; ++c)
            {
                const int face = static_cast<int>(componentFaces[c]);
                if (mesh.loops_.loopLength_[face] >= 3)
                    o.wholeFaces_.push_back(face);
            }
        }
        if (!o.elements_.empty())
            ClassifyElements(core, mesh, o, s, otherMesh, second);
    }

    BuildKeptSet(core, operation, s);
    if (s.kept_.empty())
        return;

    // The exact clean-up of the kept faces gives the result: a weld by grid point, a cancel of twin faces
    // and a closed test on the grid. This holds for operands in the grid range.
    AppendRegion(core, s, s.kept_, a, b, result);
}

/// Whether a ray from the origin along +x can hit a box. The pad covers the float rounding of the origin
/// and the pad of the BVH boxes.
bool AxisRayHitsBox(const Vector3& origin, const Vector3& min, const Vector3& max, float pad)
{
    return origin.y_ >= min.y_ - pad && origin.y_ <= max.y_ + pad && origin.z_ >= min.z_ - pad
        && origin.z_ <= max.z_ + pad && origin.x_ <= max.x_ + pad;
}

/// Count the exact crossings of a ray along +x with the triangles of a solid and return their parity.
/// The origin is a float position of the ray. It only selects the triangles with the float boxes.
/// The callback corner(f, t, k) returns the grid point of the corner k of the triangle t of the face f.
/// The callback crosses(a, b, c) gets the grid points of a triangle and returns the exact result.
/// No float test decides a crossing. The float boxes only skip triangles that the ray cannot hit.
template <class Corner, class Crosses>
bool ClassifyRay(const CsgSolid& solid, const Vector3& origin, const Corner& corner, const Crosses& crosses)
{
    constexpr float pad = 0.002f;
    const ea::span<const unsigned> triangleStart = solid.GetTriangleStart();

    const CsgFaceBvh& components = solid.GetComponentTree();
    if (components.ItemCount() == 0)
        return false;

    bool parity = false;
    const auto addFaceTriangles = [&](unsigned face)
    {
        for (unsigned t = triangleStart[face]; t < triangleStart[face + 1]; ++t)
            parity ^= crosses(corner(face, t, 0), corner(face, t, 1), corner(face, t, 2));
    };
    const auto walk = [&](const CsgFaceBvh& tree, auto&& visitItem)
    {
        if (tree.ItemCount() == 0)
            return;
        const ea::span<const CsgFaceBvh::Node> nodes = tree.GetNodes();
        const ea::span<const int> items = tree.GetItems();
        ea::fixed_vector<int, 64, true> stack;
        stack.push_back(0);
        while (!stack.empty())
        {
            const CsgFaceBvh::Node& node = nodes[stack.back()];
            stack.pop_back();
            if (!AxisRayHitsBox(origin, node.min_, node.max_, pad))
                continue;
            if (node.count_ == 0)
            {
                stack.push_back(node.first_ + 1);
                stack.push_back(node.first_);
                continue;
            }
            for (int i = node.first_; i < node.first_ + node.count_; ++i)
                visitItem(static_cast<unsigned>(items[i]));
        }
    };
    walk(components,
        [&](unsigned component)
        {
            if (AxisRayHitsBox(origin, solid.GetComponentMin(component), solid.GetComponentMax(component), pad))
                walk(solid.GetFaceTree(component), addFaceTriangles);
        });
    return parity;
}

/// Count the exact crossings with the triangles of a mesh. The ids of the mesh give the corners.
template <class Crosses> bool ClassifyRay(const Mesh& other, const Vector3& origin, const Crosses& crosses)
{
    const ea::span<const unsigned> triangleStart = other.solid_.GetTriangleStart();
    URHO3D_ASSERT(other.triangleIds_.size() == 3 * static_cast<size_t>(triangleStart.empty() ? 0 : triangleStart.back()));
    return ClassifyRay(
        other.solid_, origin,
        [&](unsigned, unsigned triangle, unsigned corner) -> const CsgGridPoint&
        { return other.grid_[other.triangleIds_[triangle * 3 + corner]]; },
        crosses);
}

/// Return true if a grid point is inside the solid of a mesh. The point and the mesh have the displacement
/// mask second, as in CsgExact::AxisRayCrossesTriangle. It is 0b0111 if the mesh is the second operand.
/// The result is the parity of the exact crossings of a ray along +x.
bool ClassifyGridPoint(const Mesh& other, const CsgGridPoint& p, int second)
{
    return ClassifyRay(other, p.ToPosition(),
        [&](const CsgGridPoint& t0, const CsgGridPoint& t1, const CsgGridPoint& t2)
        { return CsgExact::AxisRayCrossesTriangle(p, t0, t1, t2, second); });
}

/// Return true if the centroid (a + b + c) / 3 of a grid triangle is inside the solid of a mesh.
/// The mask second has the meaning of ClassifyGridPoint. The three points have the same operand.
/// The float origin is the centroid in double, rounded to float.
bool ClassifyGridCentroid(
    const Mesh& other, const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, int second)
{
    constexpr double step = 1.0 / CsgGridScale / 3.0;
    const Vector3 origin(static_cast<float>((static_cast<double>(a.x_) + b.x_ + c.x_) * step),
        static_cast<float>((static_cast<double>(a.y_) + b.y_ + c.y_) * step),
        static_cast<float>((static_cast<double>(a.z_) + b.z_ + c.z_) * step));
    return ClassifyRay(other, origin,
        [&](const CsgGridPoint& t0, const CsgGridPoint& t1, const CsgGridPoint& t2)
        { return CsgExact::AxisRayCrossesTriangleCentroid(a, b, c, t0, t1, t2, second); });
}

/// Run the boolean once with the current value of g_csgExactOnly. Each call builds a new corefinement, so the
/// aliases and the placed crossings of an earlier call do not carry over.
/// With preferWantedSide, ConstrainCrossings uses the wanted side rule with the wanted sides of the operation.
/// Only DifferenceAB and Intersection may set it.
SharedPtr<const CsgSolid> BooleanOnce(const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b,
    CsgOperation operation, float epsilon, const CsgParallelOptions& parallel, bool preferWantedSide = false)
{
    Corefinement core(*a, *b, epsilon);
    CsgScratch scratch;
    core.CollectCrossings(scratch, parallel);
    CsgWantedSides sides;
    if (operation == CsgOperation::Intersection)
        sides.outsideB_ = false;
    core.ConstrainCrossings(scratch, preferWantedSide, sides);
    core.SplitFaces(scratch);

    CsgSolid::Builder result(a->FaceCount() + b->FaceCount(),
        a->GetTrianglePositions().size() + b->GetTrianglePositions().size(), epsilon);
    CsgBooleanScratch& s = scratch.boolean_;
    AppendOperand(core, core.a_, core.segmentsA_, core.splitA_, operation, false, s.operands_[0], s);
    AppendOperand(core, core.b_, core.segmentsB_, core.splitB_, operation, true, s.operands_[1], s);
    ClassifyAndEmit(core, a, b, operation, epsilon, result, s);
    return result.Finish();
}

/// The open position edge counts of an exact pass. The tail of Boolean uses them, so a count does not run twice.
struct CsgOpenCounts
{
    /// The count of the result. Only valid if hasResult_ is set.
    unsigned result_{};
    bool hasResult_{};
    /// The sum of the counts of the operands. Only valid if hasOperands_ is set.
    unsigned operands_{};
    bool hasOperands_{};

    /// Return the sum of the counts of the operands. Count them on the first call only.
    unsigned Operands(const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b)
    {
        if (!hasOperands_)
        {
            operands_ = a->OpenPositionEdgeCount() + b->OpenPositionEdgeCount();
            hasOperands_ = true;
        }
        return operands_;
    }
};

/// The exact pass. Call it with the exact decisions. DifferenceAB and Intersection use the wanted side rule of
/// ConstrainCrossings on the fine grid. Then a repeated identical cut does not add triangles. A move of the rule can
/// open a result that the nearest rounding keeps closed. If the result has more open edges than the operands, the
/// pass runs again with the nearest rounding. The result with fewer open edges stays.
/// Union does not use the rule. In the sweep of rotated boxes, a union with the rule gave a closed result, but the
/// next union of that result opened in all passes (the axis (0, 1, 0) at 0.6 and 90.6 degrees). The rule with the
/// edge planes only keeps the sweep open count, but it adds thick retries in a finer sweep.
SharedPtr<const CsgSolid> ExactPass(const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b,
    CsgOperation operation, float epsilon, const CsgParallelOptions& parallel, CsgOpenCounts& counts)
{
    const bool prefer = CsgPreferWantedSide
        && (operation == CsgOperation::DifferenceAB || operation == CsgOperation::Intersection);
    SharedPtr<const CsgSolid> result = BooleanOnce(a, b, operation, epsilon, parallel, prefer);
    if (!prefer)
        return result;
    counts.result_ = result->OpenPositionEdgeCount();
    counts.hasResult_ = true;
    if (counts.result_ == 0 || counts.result_ <= counts.Operands(a, b))
        return result;
    CSG_DIAG_COUNT(wantedSideRetries_);
    SharedPtr<const CsgSolid> nearest = BooleanOnce(a, b, operation, epsilon, parallel);
    const unsigned nearestOpen = nearest->OpenPositionEdgeCount();
    if (nearestOpen >= counts.result_)
        return result;
    counts.result_ = nearestOpen;
    return nearest;
}

} // namespace

SharedPtr<const CsgSolid> Boolean(const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b,
    CsgOperation operation, float epsilon, const CsgParallelOptions& parallel)
{
    // The negated test also catches NaN.
    if (!(epsilon > 0.0f))
        epsilon = CSG_DEFAULT_EPSILON;

    // An operand that is out of range gives an empty result that is out of range.
    if (a->IsOutOfRange() || b->IsOutOfRange())
        return CsgSolid::OutOfRangeResult(epsilon);

    // Solids are immutable, so an unchanged operand is itself the result.
    if (a->FaceCount() == 0)
        return operation == CsgOperation::Union ? b : a;
    if (b->FaceCount() == 0)
        return operation == CsgOperation::Intersection ? b : a;

    // The first pass uses the exact decisions. The C# code runs the thick pass first, but the C++ classification
    // is exact: a thick pass can give a closed result with a wrong volume, and no count detects that.
    // The scope sets the flag explicitly, because a nested boolean on one thread can see the scope of an outer one.
    SharedPtr<const CsgSolid> result;
    CsgOpenCounts counts;
    {
        const CsgExactOnlyScope exact(true);
        result = ExactPass(a, b, operation, epsilon, parallel, counts);
    }
    // Rounded near-coincident input can open the exact result. If the result has more open edges than the operands,
    // run again with the thick decisions: elements within the thickness of each other are coincident.
    // Keep the result with fewer open edges. The count is by position, as in the C# code: the exact ids of a result
    // can hide a sheet or a crack.
    const unsigned open = counts.hasResult_ ? counts.result_ : result->OpenPositionEdgeCount();
    if (open == 0 || open <= counts.Operands(a, b))
        return result;
    CSG_DIAG_COUNT(thickRetries_);
    const CsgExactOnlyScope thick(false);
    SharedPtr<const CsgSolid> thickResult = BooleanOnce(a, b, operation, epsilon, parallel);
    return thickResult->OpenPositionEdgeCount() < open ? thickResult : result;
}

SharedPtr<const CsgSolid> CsgBooleanOnceForTest(const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b,
    CsgOperation operation, float epsilon)
{
    return BooleanOnce(a, b, operation, epsilon, {});
}

bool CsgContainsGridPoint(const CsgSolid& solid, const CsgGridPoint& p)
{
    if (solid.FaceCount() == 0)
        return false;
    const CsgExactLoops loops = solid.GetExactLoops();
    const ea::span<const unsigned> triangleVertices = solid.GetTriangleVertices();
    return ClassifyRay(
        solid, p.ToPosition(),
        [&](unsigned face, unsigned triangle, unsigned corner) -> const CsgGridPoint&
        { return loops.grid_[loops.ids_[loops.vertexStart_[face] + triangleVertices[triangle * 3 + corner]]]; },
        [&](const CsgGridPoint& t0, const CsgGridPoint& t1, const CsgGridPoint& t2)
        { return CsgExact::AxisRayCrossesTriangle(p, t0, t1, t2, 0b0111); });
}

bool CsgClassifyGridPointForTest(const CsgSolid& other, const CsgGridPoint& p, int second)
{
    const Mesh mesh(other, second == 0b0111, 0);
    return ClassifyGridPoint(mesh, p, second);
}

bool CsgInThickSurfaceForTest(const CsgSolid& other, const CsgGridPoint& p)
{
    const Mesh mesh(other, false, 0);
    ea::vector<int> components;
    ea::vector<int> faces;
    ea::vector<int> stack;
    return InThickSurface(mesh, p, MaxFaceExtent(mesh), components, faces, stack);
}

bool CsgClassifyGridCentroidForTest(
    const CsgSolid& other, const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c, int second)
{
    const Mesh mesh(other, second == 0b0111, 0);
    return ClassifyGridCentroid(mesh, a, b, c, second);
}

} // namespace CsgInternal

} // namespace Urho3D
