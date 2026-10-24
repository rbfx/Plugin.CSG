// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgBoolean.h"

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include <Urho3D/IO/Log.h>

#include <EASTL/algorithm.h>
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

/// Return a point inside a polygon: the centroid of its largest triangle.
Vector3 PolygonInteriorPoint(const CsgPolygon& polygon, ea::vector<unsigned>& indices)
{
    const auto& vertices = polygon.vertices_;
    indices.clear();
    if (vertices.size() > 3 && CsgTriangulatePolygon(polygon, indices) && indices.size() >= 3)
    {
        unsigned best = 0;
        float bestArea = -1.0f;
        for (unsigned t = 0; t + 2 < indices.size(); t += 3)
        {
            // A bad index from the triangulator is skipped, so the read stays inside the polygon.
            if (indices[t] >= vertices.size() || indices[t + 1] >= vertices.size() || indices[t + 2] >= vertices.size())
                continue;
            const Vector3 a = vertices[indices[t]].GetPosition();
            const float area = (vertices[indices[t + 1]].GetPosition() - a)
                                   .CrossProduct(vertices[indices[t + 2]].GetPosition() - a)
                                   .LengthSquared();
            if (area > bestArea)
            {
                bestArea = area;
                best = t;
            }
        }
        if (bestArea >= 0.0f)
        {
            return (vertices[indices[best]].GetPosition() + vertices[indices[best + 1]].GetPosition()
                       + vertices[indices[best + 2]].GetPosition())
                / 3.0f;
        }
    }
    Vector3 sum = Vector3::ZERO;
    for (const ModelVertex& vertex : vertices)
        sum += vertex.GetPosition();
    return vertices.empty() ? sum : sum / static_cast<float>(vertices.size());
}

/// Return the number of ids in the loop of an element.
int ElementLoopSize(const Mesh& mesh, const CsgBooleanElement& element)
{
    return element.sourceFace_ >= 0 ? mesh.loops_.loopLength_[element.sourceFace_] : element.idCount_;
}

/// Classify the elements against the other operand. The result is in the scratch: not zero if inside.
///
/// Each curve edge of an element decides exactly which side of the curve is inside the other operand:
/// the element on the left of an oriented segment is inside for the first operand, and on its right for the second.
/// Elements joined by an edge off the curve form a patch. Decisions spread across such edges from the nearest
/// decided element. A patch without curve edges falls back to a point query.
/// The perturbation of the crossing step keeps the curve off the edges of both operands.
/// Because of this only regions of split triangles have curve edges.
void ClassifyElements(
    const Corefinement& core, const Mesh& mesh, CsgBooleanScratch& s, const CsgSolid& other, bool second, float epsilon)
{
    const int elementCount = static_cast<int>(s.elements_.size());

    // Every element loop in one buffer. An unsplit face reads its loop from the mesh.
    s.edgeStart_.assign(elementCount + 1, 0);
    for (int e = 0; e < elementCount; ++e)
        s.edgeStart_[e + 1] = s.edgeStart_[e] + ElementLoopSize(mesh, s.elements_[e]);
    const int edgeTotal = s.edgeStart_[elementCount];
    s.loops_.resize(edgeTotal);
    for (int e = 0; e < elementCount; ++e)
    {
        const CsgBooleanElement& element = s.elements_[e];
        if (element.sourceFace_ < 0)
        {
            for (int i = 0; i < element.idCount_; ++i)
                s.loops_[s.edgeStart_[e] + i] = s.elementIds_[element.idStart_ + i];
            continue;
        }
        const unsigned start = mesh.loops_.vertexStart_[element.sourceFace_];
        for (int i = s.edgeStart_[e]; i < s.edgeStart_[e + 1]; ++i)
            s.loops_[i] = core.Global(mesh, mesh.loops_.vertexIds_[start + i - s.edgeStart_[e]]);
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
    s.inside_.assign(elementCount, 0);
    s.decided_.assign(elementCount, 0);
    s.queue_.clear();
    for (int e = 0; e < elementCount; ++e)
    {
        const int* loop = s.loops_.data() + s.edgeStart_[e];
        const int length = s.edgeStart_[e + 1] - s.edgeStart_[e];
        const bool split = s.elements_[e].split_;
        int votes = 0;
        for (int i = 0; i < length; ++i)
        {
            const int from = loop[i];
            const int to = loop[(i + 1) % length];
            if (split)
            {
                if (const auto* segment = core.curveEdges_.Find(UndirectedKey(from, to)))
                {
                    const bool forward = segment->first == from && segment->second == to;
                    votes += forward != second ? 1 : -1;
                    continue;
                }
            }
            if (const int* twin = s.edgeOwner_.Find(DirectedKey(to, from)))
            {
                if (*twin != e)
                    s.twins_[s.edgeStart_[e] + i] = *twin;
            }
        }
        // Each element next to the curve keeps its own exact decision, so the elements on either side of a curve
        // edge always differ. The other elements inherit from the nearest decided element across edges off the curve.
        if (votes == 0)
            continue;
        s.inside_[e] = votes > 0 ? 1 : 0;
        s.decided_[e] = 1;
        s.queue_.push_back(e);
    }

    for (unsigned head = 0; head < s.queue_.size(); ++head)
    {
        const int e = s.queue_[head];
        for (int k = s.edgeStart_[e]; k < s.edgeStart_[e + 1]; ++k)
        {
            const int twin = s.twins_[k];
            if (twin < 0 || s.decided_[twin])
                continue;
            s.inside_[twin] = s.inside_[e];
            s.decided_[twin] = 1;
            s.queue_.push_back(twin);
        }
    }

    // What remains are patches that the curve does not reach: sample the largest element of each one.
    for (int e = 0; e < elementCount; ++e)
    {
        if (s.decided_[e])
            continue;
        s.patch_.clear();
        s.decided_[e] = 1;
        s.patch_.push_back(e);
        for (unsigned p = 0; p < s.patch_.size(); ++p)
        {
            for (int k = s.edgeStart_[s.patch_[p]]; k < s.edgeStart_[s.patch_[p] + 1]; ++k)
            {
                const int twin = s.twins_[k];
                if (twin >= 0 && !s.decided_[twin])
                {
                    s.decided_[twin] = 1;
                    s.patch_.push_back(twin);
                }
            }
        }
        int best = e;
        double bestArea = -1.0;
        for (const int member : s.patch_)
        {
            const double area = PolygonArea(*s.elements_[member].polygon_);
            if (area > bestArea)
            {
                bestArea = area;
                best = member;
            }
        }
        bool reliable = true;
        const bool inside =
            other.ClassifyPoint(PolygonInteriorPoint(*s.elements_[best].polygon_, s.indices_), epsilon, reliable);
        for (const int member : s.patch_)
            s.inside_[member] = inside ? 1 : 0;
    }
}

/// Tell if a component that the curve does not reach lies inside the other solid.
///
/// The reference tests one sample point of the component. That point can lie on the surface of the other solid,
/// for example when two boxes share a face and the curve has no crossing. The test can then give a wrong answer
/// and still report that it is reliable. All faces of the component have the same answer, so this function
/// takes the sample points of the faces in order. It stops when one answer leads by two votes.
/// If no answer leads after 16 samples, it uses the test of the first sample, as the reference does.
bool ClassifyComponent(const CsgSolid& solid, unsigned component, const CsgSolid& other, float epsilon)
{
    static constexpr unsigned MAX_SAMPLES = 16;
    const ea::span<const unsigned> componentStart = solid.GetComponentStart();
    const ea::span<const unsigned> componentFaces = solid.GetComponentFaces();
    const unsigned first = componentStart[component];
    const unsigned sampleCount = ea::min(componentStart[component + 1] - first, MAX_SAMPLES);
    int votes = 0;
    for (unsigned i = 0; i < sampleCount; ++i)
    {
        bool reliable = true;
        const bool inside = other.ClassifyPoint(solid.FaceSample(componentFaces[first + i]), epsilon, reliable);
        if (!reliable)
            continue;
        votes += inside ? 1 : -1;
        if (votes >= 2 || votes <= -2)
            return votes > 0;
    }
    if (votes != 0)
        return votes > 0;
    return other.ContainsPoint(solid.ComponentSample(component), epsilon);
}

/// Append the kept surface of one operand.
///
/// Components that the curve does not reach are classified whole and go to the builder.
/// The other components are split along the curve, grouped into patches between curve edges and classified patch
/// by patch. The kept patches go to the list of kept faces.
///
/// A triangle that is collapsed is left out of a face that the curve reaches. The reference leaves out only
/// triangles with welded corners. Here a triangle with a corner that is not finite is left out too, because
/// a result vertex that is not finite has no meaning for the later steps.
/// A face that the curve does not reach is kept as it is, also with a corner that is not finite.
void AppendOperand(const Corefinement& core, const Mesh& mesh, const CsgSegmentLists& segments, const CsgSplitResult& split,
    const CsgSolid& other, CsgOperation operation, bool second, CsgSolid::Builder& result, CsgBooleanScratch& s)
{
    const CsgSolid& solid = mesh.solid_;
    const bool flip = second && operation == CsgOperation::DifferenceAB;
    const unsigned componentCount = solid.ComponentCount();
    const auto faceComponents = solid.GetFaceComponents();
    const auto triangleStart = solid.GetTriangleStart();
    const auto triangleVertices = solid.GetTriangleVertices();
    const auto componentStart = solid.GetComponentStart();
    const auto componentFaces = solid.GetComponentFaces();

    s.touched_.assign(componentCount, 0);
    s.frontier_.assign(solid.FaceCount(), 0);
    for (unsigned t = 0; t < mesh.triangleFace_.size(); ++t)
    {
        if (segments.Count(static_cast<int>(t)) == 0)
            continue;
        const int face = mesh.triangleFace_[t];
        s.frontier_[face] = 1;
        s.touched_[faceComponents[face]] = 1;
    }

    const float epsilon = core.epsilon_;
    for (unsigned component = 0; component < componentCount; ++component)
    {
        if (s.touched_[component])
            continue;
        const bool inside = ClassifyComponent(solid, component, other, epsilon);
        if (CsgSolid::Keep(inside, operation, second))
            result.AppendComponent(solid, component, flip);
    }

    s.elements_.clear();
    s.elementIds_.clear();
    for (unsigned component = 0; component < componentCount; ++component)
    {
        if (!s.touched_[component])
            continue;
        for (unsigned c = componentStart[component]; c < componentStart[component + 1]; ++c)
        {
            const int face = static_cast<int>(componentFaces[c]);
            if (mesh.loops_.loopLength_[face] < 3)
                continue;
            if (!s.frontier_[face])
            {
                CsgBooleanElement& element = s.elements_.emplace_back();
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
                    const size_t elementMark = s.elements_.size();
                    const size_t idMark = s.elementIds_.size();
                    bool ok = true;
                    for (int r = split.triangleRegionStart_[t]; r < split.triangleRegionStart_[t + 1]; ++r)
                    {
                        CsgPolygon polygon;
                        if (!RegionPolygon(core, mesh, face, split.RegionIds(r), split.RegionSize(r), polygon))
                        {
                            ok = false;
                            break;
                        }
                        CsgBooleanElement& element = s.elements_.emplace_back();
                        element.polygon_ = ea::make_shared<CsgPolygon>(ea::move(polygon));
                        element.idStart_ = static_cast<int>(s.elementIds_.size());
                        element.idCount_ = split.RegionSize(r);
                        element.split_ = true;
                        s.elementIds_.insert(s.elementIds_.end(), split.RegionIds(r),
                            split.RegionIds(r) + split.RegionSize(r));
                    }
                    if (ok)
                        continue;
                    // A region does not fit its face. The triangle stays whole, as for a failed split.
                    s.elements_.resize(elementMark);
                    s.elementIds_.resize(idMark);
                }

                ea::vector<ModelVertex> corners;
                corners.reserve(3);
                CsgBooleanElement& element = s.elements_.emplace_back();
                element.idStart_ = static_cast<int>(s.elementIds_.size());
                element.idCount_ = 3;
                for (unsigned k = 0; k < 3; ++k)
                {
                    corners.push_back(source.vertices_[triangleVertices[t * 3 + k]]);
                    s.elementIds_.push_back(core.Global(mesh, mesh.triangleIds_[t * 3 + k]));
                }
                auto polygon = ea::make_shared<CsgPolygon>(ea::move(corners), source.shared_);
                polygon->plane_ = source.plane_;
                element.polygon_ = ea::move(polygon);
            }
        }
    }
    if (s.elements_.empty())
        return;

    ClassifyElements(core, mesh, s, other, second, epsilon);
    for (unsigned i = 0; i < s.elements_.size(); ++i)
    {
        if (!CsgSolid::Keep(s.inside_[i] != 0, operation, second))
            continue;
        const CsgBooleanElement& element = s.elements_[i];
        CsgKeptFace& kept = s.kept_.emplace_back();
        kept.polygon_ = flip ? CsgSolid::Flipped(*element.polygon_) : element.polygon_;
        kept.sourceFace_ = element.sourceFace_;
        kept.second_ = second;
        kept.flip_ = flip;
        if (element.sourceFace_ >= 0)
            continue;
        kept.idStart_ = static_cast<int>(s.keptIds_.size());
        kept.idCount_ = element.idCount_;
        const int* ids = s.elementIds_.data() + element.idStart_;
        for (int k = 0; k < element.idCount_; ++k)
            s.keptIds_.push_back(ids[flip ? element.idCount_ - 1 - k : k]);
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

/// Remove pairs of kept faces that visit the same vertices in opposite directions.
///
/// Where a closed curve passes through collinear crossings, both operands keep a zero-area island with the same
/// vertices. The two cancel. The faces around them then meet along the same edges.
/// Faces are bucketed by an order-independent hash of their vertex ids and then compared exactly.
/// The pairs depend only on the order of the faces, not on the order of the buckets.
void CancelOpposites(CsgBooleanScratch& s)
{
    auto& candidates = s.candidates_;
    candidates.clear();
    for (unsigned i = 0; i < s.kept_.size(); ++i)
    {
        // Unsplit faces of the two operands never share vertex ids, so only regions can cancel.
        const CsgKeptFace& face = s.kept_[i];
        if (face.sourceFace_ >= 0)
            continue;
        uint64_t hash = 0;
        for (int k = 0; k < face.idCount_; ++k)
            hash += (static_cast<uint64_t>(static_cast<uint32_t>(s.keptIds_[face.idStart_ + k])) + 0x9E3779B97F4A7C15ULL)
                * 0xBF58476D1CE4E5B9ULL;
        candidates.push_back({face.idCount_, hash, static_cast<int>(i)});
    }
    if (candidates.size() < 2)
        return;
    ea::sort(candidates.begin(), candidates.end(), [](const CsgOppositeCandidate& x, const CsgOppositeCandidate& y)
    {
        if (x.count_ != y.count_)
            return x.count_ < y.count_;
        if (x.hash_ != y.hash_)
            return x.hash_ < y.hash_;
        return x.index_ < y.index_;
    });

    s.removed_.assign(s.kept_.size(), 0);
    bool any = false;
    unsigned begin = 0;
    while (begin < candidates.size())
    {
        unsigned end = begin + 1;
        while (end < candidates.size() && candidates[end].count_ == candidates[begin].count_
            && candidates[end].hash_ == candidates[begin].hash_)
            ++end;
        for (unsigned a = begin; a < end; ++a)
        {
            const int indexA = candidates[a].index_;
            if (s.removed_[indexA])
                continue;
            const CsgKeptFace& faceA = s.kept_[indexA];
            for (unsigned b = a + 1; b < end; ++b)
            {
                const int indexB = candidates[b].index_;
                if (s.removed_[indexB])
                    continue;
                const CsgKeptFace& faceB = s.kept_[indexB];
                if (!IsReverse(s.keptIds_.data() + faceA.idStart_, s.keptIds_.data() + faceB.idStart_, faceA.idCount_))
                    continue;
                s.removed_[indexA] = s.removed_[indexB] = 1;
                any = true;
                break;
            }
        }
        begin = end;
    }
    if (!any)
        return;

    unsigned out = 0;
    for (unsigned i = 0; i < s.kept_.size(); ++i)
    {
        if (!s.removed_[i])
            s.kept_[out++] = ea::move(s.kept_[i]);
    }
    s.kept_.resize(out);
}

} // namespace

SharedPtr<const CsgSolid> Boolean(
    const SharedPtr<const CsgSolid>& a, const SharedPtr<const CsgSolid>& b, CsgOperation operation, float epsilon)
{
    // The negated test also catches NaN.
    if (!(epsilon > 0.0f))
        epsilon = CSG_DEFAULT_EPSILON;

    // Solids are immutable, so an unchanged operand is itself the result.
    if (a->FaceCount() == 0)
        return operation == CsgOperation::Union ? b : a;
    if (b->FaceCount() == 0)
        return operation == CsgOperation::Intersection ? b : a;

    Corefinement core(*a, *b, epsilon);
    CsgScratch scratch;
    core.CollectCrossings(scratch);
    core.SplitFaces(scratch);

    CsgSolid::Builder result(a->FaceCount() + b->FaceCount(),
        a->GetTrianglePositions().size() + b->GetTrianglePositions().size(), epsilon);
    CsgBooleanScratch& s = scratch.boolean_;
    AppendOperand(core, core.a_, core.segmentsA_, core.splitA_, *b, operation, false, result, s);
    AppendOperand(core, core.b_, core.segmentsB_, core.splitB_, *a, operation, true, result, s);

    SharedPtr<CsgSolid> regionSolid;
    if (!s.kept_.empty())
    {
        CancelOpposites(s);
        // The ids of a kept region are recorded. An unsplit face reads its ids from its solid, offset like the mesh.
        ea::vector<CsgSharedPolygon> faces;
        ea::vector<CsgFaceOrigin> origins;
        ea::vector<int> exactIds;
        faces.reserve(s.kept_.size());
        origins.reserve(s.kept_.size());
        for (const CsgKeptFace& kept : s.kept_)
        {
            faces.push_back(kept.polygon_);
            CsgFaceOrigin& origin = origins.emplace_back();
            if (kept.sourceFace_ >= 0)
            {
                origin.solid_ = kept.second_ ? b : a;
                origin.face_ = static_cast<unsigned>(kept.sourceFace_);
                origin.flip_ = kept.flip_;
                const Mesh& mesh = kept.second_ ? core.b_ : core.a_;
                const unsigned start = mesh.loops_.vertexStart_[kept.sourceFace_];
                const int count = static_cast<int>(mesh.loops_.vertexStart_[kept.sourceFace_ + 1] - start);
                for (int k = 0; k < count; ++k)
                    exactIds.push_back(mesh.offset_ + mesh.loops_.ids_[start + (kept.flip_ ? count - 1 - k : k)]);
            }
            else
            {
                exactIds.insert(exactIds.end(), s.keptIds_.begin() + kept.idStart_,
                    s.keptIds_.begin() + kept.idStart_ + kept.idCount_);
            }
        }
        const int idSpace = core.CrossingBase() + static_cast<int>(core.crossings_.size());
        regionSolid = CsgSolid::FromKeptFaces(ea::move(faces), epsilon, origins, exactIds, idSpace);
        result.AppendSolid(*regionSolid);
    }
    return result.Finish();
}

} // namespace CsgInternal

} // namespace Urho3D
