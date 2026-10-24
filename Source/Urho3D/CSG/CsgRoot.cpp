// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgRootInternal.h"
#include "Urho3D/CSG/CsgSolid.h"
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Timer.h>
#include <Urho3D/Core/WorkQueue.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>

#include <cmath>
#include <memory>

namespace Urho3D
{

struct CsgRoot::BrushStep
{
    ea::vector<CsgPolygon> polygons_;

    SharedPtr<Model> model_;
    ResourceRefList materials_;
    unsigned meshId_{};
    CsgOperation op_{};
    WeakPtr<CsgBrush> brush_;
    /// Brush to root frame transform.
    Matrix3x4 transform_{Matrix3x4::IDENTITY};
    ea::vector<BrushStep> children_;
};

struct CsgRoot::BuildJob
{
    WeakPtr<CsgRoot> root_;
    Context* context_{};
    unsigned buildId_{};
    unsigned componentId_{};
    float epsilon_{};
    /// World scale of the root. The root frame carries this scale.
    Vector3 metricScale_{Vector3::ONE};
    /// True if an operand has a coordinate that the exact math cannot handle.
    bool outOfRange_{};

    SharedPtr<Model> baseModel_;
    ResourceRefList baseMaterials_;
    BrushStep rootStep_;

    ea::vector<SharedPtr<Model>> sourceModels_;
    ea::vector<ResourceRefList> sourceMaterials_;

    /// Solid of the earlier build. If set, the job evaluates only the new brushes on it.
    SharedPtr<const CsgSolid> cachedSolid_;
    /// Number of root children that the cached solid already contains.
    unsigned reusedRootChildren_{};
    /// Fingerprints of the root children. Empty if the tree is not flat.
    ea::vector<BrushFingerprint> flatBrushes_;
    /// Final solid of the job. The worker thread sets it.
    SharedPtr<const CsgSolid> resultSolid_;
};

namespace
{

inline unsigned PackSourceGeometryKey(unsigned meshId, unsigned geometryIndex)
{
    return (meshId << 16u) | (geometryIndex & 0xFFFFu);
}

bool IsSupportedCsgVertexElement(const VertexElement& element)
{
    if (element.stepRate_ != 0)
        return false;

    switch (element.semantic_)
    {
    case SEM_POSITION:
    case SEM_NORMAL:
    case SEM_BINORMAL:
    case SEM_TANGENT:
    case SEM_TEXCOORD:
    case SEM_COLOR: return true;

    case SEM_BLENDWEIGHTS:
    case SEM_BLENDINDICES:
    case SEM_OBJECTINDEX:
    default: return false;
    }
}

ea::vector<VertexElement> GetSupportedCsgVertexElements(const Model* sourceModel, unsigned geometryIndex)
{
    Geometry* sourceGeometry = sourceModel ? sourceModel->GetGeometry(geometryIndex, 0) : nullptr;
    ea::vector<VertexElement> elements;

    if (sourceGeometry)
    {
        unsigned vertexSize = 0;
        unsigned indexSize = 0;
        const unsigned char* vertexData = nullptr;
        const unsigned char* indexData = nullptr;
        const ea::vector<VertexElement>* sourceElements = nullptr;
        sourceGeometry->GetRawData(vertexData, vertexSize, indexData, indexSize, sourceElements);

        if (sourceElements)
        {
            elements.reserve(sourceElements->size());
            ea::copy_if(sourceElements->begin(), sourceElements->end(), ea::back_inserter(elements), IsSupportedCsgVertexElement);
        }
    }

    if (elements.empty())
        elements.emplace_back(TYPE_VECTOR3, SEM_POSITION, 0);

    VertexBuffer::UpdateOffsets(elements);
    return elements;
}

bool AreVertexLayoutsCompatible(const ea::vector<VertexElement>& lhs, const ea::vector<VertexElement>& rhs)
{
    if (lhs.size() != rhs.size())
        return false;

    for (unsigned i = 0; i < lhs.size(); ++i)
    {
        const VertexElement& a = lhs[i];
        const VertexElement& b = rhs[i];
        if (a.type_ != b.type_ || a.semantic_ != b.semantic_ || a.index_ != b.index_ || a.stepRate_ != b.stepRate_
            || a.offset_ != b.offset_)
        {
            return false;
        }
    }

    return true;
}

struct SharedKeyCompatibilityGroup
{
    unsigned representativeShared_{};
    ea::string materialName_;
    ea::vector<VertexElement> elements_;
};

ea::hash_map<unsigned, unsigned> BuildCanonicalSharedKeyMap(
    const ea::vector<SharedPtr<Model>>& sourceModels, const ea::vector<ResourceRefList>& sourceMaterials)
{
    ea::hash_map<unsigned, unsigned> canonicalSharedKeys;
    ea::vector<SharedKeyCompatibilityGroup> groups;

    for (unsigned meshId = 0; meshId < sourceModels.size(); ++meshId)
    {
        const Model* sourceModel = sourceModels[meshId];
        if (!sourceModel)
            continue;

        const ResourceRefList* materialList = meshId < sourceMaterials.size() ? &sourceMaterials[meshId] : nullptr;
        const unsigned numGeometries = sourceModel->GetNumGeometries();
        for (unsigned geometryIndex = 0; geometryIndex < numGeometries; ++geometryIndex)
        {
            const unsigned shared = PackSourceGeometryKey(meshId, geometryIndex);
            const ea::string materialName =
                materialList && geometryIndex < materialList->names_.size() ? materialList->names_[geometryIndex] : ea::string{};
            const ea::vector<VertexElement> elements = GetSupportedCsgVertexElements(sourceModel, geometryIndex);

            unsigned representativeShared = shared;
            for (const SharedKeyCompatibilityGroup& group : groups)
            {
                if (group.materialName_ == materialName && AreVertexLayoutsCompatible(group.elements_, elements))
                {
                    representativeShared = group.representativeShared_;
                    break;
                }
            }

            canonicalSharedKeys[shared] = representativeShared;
            if (representativeShared == shared)
                groups.push_back({shared, materialName, elements});
        }
    }

    return canonicalSharedKeys;
}

void CanonicalizeSharedKeys(ea::vector<CsgPolygon>& polygons, const ea::hash_map<unsigned, unsigned>& canonicalSharedKeys)
{
    for (CsgPolygon& polygon : polygons)
    {
        const auto it = canonicalSharedKeys.find(polygon.shared_);
        if (it != canonicalSharedKeys.end())
            polygon.shared_ = it->second;
    }
}

Node* GetOrCreateTemporaryChild(Node* parent, const ea::string& name)
{
    if (!parent)
        return nullptr;

    if (Node* existing = parent->GetChild(StringHash(name), false))
        return existing;

    Node* child = parent->CreateTemporaryChild(name);
    child->SetEnabled(false);
    return child;
}

void SetStaticModelMaterials(StaticModel* staticModel, const ea::vector<SharedPtr<Material>>& materials)
{
    if (!staticModel)
        return;

    const unsigned numGeometries = staticModel->GetNumGeometries();
    for (unsigned i = 0; i < numGeometries; ++i)
    {
        Material* material = i < materials.size() ? materials[i].Get() : nullptr;
        staticModel->SetMaterial(i, material);
    }
}

} // namespace

CsgRoot::CsgRoot(Context* context)
    : CsgComponent(context)
{
}

void CsgRoot::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgRoot>(Category_Scene);

    URHO3D_ATTRIBUTE("Deferred Delay Sec", float, deferredDelaySec_, 0.1f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Epsilon", float, epsilon_, CSG_DEFAULT_EPSILON, AM_DEFAULT);
    URHO3D_COPY_BASE_ATTRIBUTES(CsgComponent);
    URHO3D_ACTION_STATIC_LABEL("Rebuild", Rebuild, "Rebuild component CSG now");
}

void CsgRoot::ApplyAttributes()
{
    Component::ApplyAttributes();

    // Node::Load calls ApplyAttributes() after children/components exist.
    // Generated output model is not serialized as a resource reference, so we must rebuild after load.
    RequestRebuild();
}

void CsgRoot::OnSetEnabled()
{
    Component::OnSetEnabled();
    UpdateSubscriptions();
}

void CsgRoot::OnNodeSet(Node* previousNode, Node* currentNode)
{
    Component::OnNodeSet(previousNode, currentNode);
    UpdateSubscriptions();
}

void CsgRoot::OnSceneSet(Scene* previousScene, Scene* scene)
{
    Component::OnSceneSet(previousScene, scene);
    UpdateSubscriptions();
}

void CsgRoot::OnMarkedDirty(Node* node)
{
    // Called by Node::MarkDirty when any listened node becomes dirty.
    // This is the most reliable way to catch editor-driven transform changes.
    if (!IsEnabledEffective())
        return;

    Time* time = GetSubsystem<Time>();
    const unsigned currentFrame = time->GetFrameNumber();

    // Rebuild is pointless when root node moves: it doesn't change inputs in root-local space,
    // it only moves the whole output together with the node.
    // Note: Node::MarkDirty marks children dirty too, so we must ignore the child notifications
    // caused by this same root dirty cascade.
    if (node == node_)
    {
        ignoreChildDirtyFrame_ = currentFrame;
        return;
    }

    if (ignoreChildDirtyFrame_ == currentFrame)
        return;

    rebuildDirty_ = true;
    ++rebuildRequestId_;
    lastChangeTimeMs_ = Time::GetSystemTime();

    ScheduleDeferredRebuild();
}

void CsgRoot::OnModelChanged()
{
    BaseClassName::OnModelChanged();
    cachedSolid_ = nullptr;
    cachedBrushes_.clear();
    RequestRebuild();
}

void CsgRoot::OnMaterialsChanged()
{
    BaseClassName::OnMaterialsChanged();
    // The canonical shared keys of the cached solid depend on the materials.
    cachedSolid_ = nullptr;
    cachedBrushes_.clear();
    RequestRebuild();
}

void CsgRoot::SetDeferredDelaySec(float value)
{
    deferredDelaySec_ = ea::max(0.0f, value);
}

void CsgRoot::SetEpsilon(float value)
{
    epsilon_ = ea::max(0.0f, value);
    RequestRebuild();
}

void CsgRoot::RequestRebuild()
{
    rebuildDirty_ = true;

    ++rebuildRequestId_;

    lastChangeTimeMs_ = Time::GetSystemTime();

    ScheduleDeferredRebuild();
}

void CsgRoot::RegisterBrush(CsgBrush* brush)
{
    RegisterBrushInternal(brush, true);
}

void CsgRoot::UnregisterBrush(CsgBrush* brush)
{
    UnregisterBrushInternal(brush);
}

StaticModel* CsgRoot::GetOrCreateOutputStaticModel()
{
    if (!node_)
        return nullptr;

    if (output_)
        return output_;

    output_ = node_->GetOrCreateComponent<StaticModel>();
    return output_;
}

void CsgRoot::CollectBrushesAndNestedRoots(
    Node* rootNode, ea::vector<CsgBrush*>& outBrushes, ea::vector<Node*>& outNestedRootNodes) const
{
    outBrushes.clear();
    outNestedRootNodes.clear();

    if (!rootNode)
        return;

    ea::vector<Node*> stack;
    stack.push_back(rootNode);

    while (!stack.empty())
    {
        Node* node = stack.back();
        stack.pop_back();

        if (node != rootNode)
        {
            if (CsgRoot* nestedRoot = node->GetComponent<CsgRoot>())
            {
                if (nestedRoot != this)
                {
                    outNestedRootNodes.push_back(node);
                    continue;
                }
            }
        }

        if (CsgBrush* brush = node->GetComponent<CsgBrush>())
        {
            if (brush->IsEnabledEffective())
                outBrushes.push_back(brush);
        }

        const unsigned numChildren = node->GetNumChildren();
        // Preserve child order deterministically for stable operand ordering.
        for (unsigned i = numChildren; i-- > 0;)
        {
            Node* child = node->GetChild(i);
            if (child && child->IsEnabled())
                stack.push_back(child);
        }
    }
}

void CsgRoot::ApplyResultToOutput(Model* model, const ResourceRefList& materials)
{
    StaticModel* output = GetOrCreateOutputStaticModel();
    if (!output)
        return;

    output->SetModel(model);

    output->SetMaterialsAttr(materials);
}

void CsgRoot::UpdateSubscriptions()
{
    Scene* scene = GetScene();

    const bool isActive = IsEnabledEffective() && scene;

    // Always detach any previously attached listeners.
    for (const WeakPtr<Node>& weakNode : listenedNodes_)
    {
        if (Node* oldNode = weakNode)
            oldNode->RemoveListener(this);
    }
    listenedNodes_.clear();

    if (!isActive)
    {
        return;
    }

    PruneBrushes();

    // Rebuild listener set from registered brushes.
    listenedNodes_.reserve(brushes_.size() + 1);

    // Listen to the root itself to detect parent/root transform cascades.
    if (node_)
    {
        node_->AddListener(this);
        listenedNodes_.push_back(WeakPtr<Node>(node_));
    }

    for (const WeakPtr<CsgBrush>& weakBrush : brushes_)
    {
        CsgBrush* brush = weakBrush;
        if (!brush || !brush->IsEnabledEffective())
            continue;

        Node* brushNode = brush->GetNode();
        if (!brushNode)
            continue;

        brushNode->AddListener(this);

        bool alreadyTracked = false;
        for (const WeakPtr<Node>& existing : listenedNodes_)
        {
            if (existing == brushNode)
            {
                alreadyTracked = true;
                break;
            }
        }
        if (!alreadyTracked)
            listenedNodes_.push_back(WeakPtr<Node>(brushNode));
    }
}

void CsgRoot::RegisterBrushInternal(CsgBrush* brush, bool requestRebuild)
{
    if (!brush)
        return;

    PruneBrushes();

    for (const WeakPtr<CsgBrush>& weakBrush : brushes_)
    {
        if (weakBrush == brush)
            return;
    }

    brushes_.push_back(WeakPtr<CsgBrush>(brush));

    if (IsEnabledEffective() && GetScene())
    {
        if (Node* brushNode = brush->GetNode())
        {
            brushNode->AddListener(this);

            bool alreadyTracked = false;
            for (const WeakPtr<Node>& existing : listenedNodes_)
            {
                if (existing == brushNode)
                {
                    alreadyTracked = true;
                    break;
                }
            }
            if (!alreadyTracked)
                listenedNodes_.push_back(WeakPtr<Node>(brushNode));
        }
    }

    if (requestRebuild)
        RequestRebuild();
}

void CsgRoot::UnregisterBrushInternal(CsgBrush* brush)
{
    if (!brush)
        return;

    Node* brushNode = brush->GetNode();
    if (brushNode && brushNode != node_)
        brushNode->RemoveListener(this);

    for (unsigned i = 0; i < brushes_.size();)
    {
        CsgBrush* existing = brushes_[i];
        if (!existing || existing == brush)
            brushes_.erase(brushes_.begin() + i);
        else
            ++i;
    }

    for (unsigned i = 0; i < listenedNodes_.size();)
    {
        Node* existing = listenedNodes_[i];
        if (!existing || existing == brushNode)
            listenedNodes_.erase(listenedNodes_.begin() + i);
        else
            ++i;
    }

    RequestRebuild();
}

void CsgRoot::PruneBrushes()
{
    for (unsigned i = 0; i < brushes_.size();)
    {
        if (!brushes_[i])
            brushes_.erase(brushes_.begin() + i);
        else
            ++i;
    }

    for (unsigned i = 0; i < listenedNodes_.size();)
    {
        if (!listenedNodes_[i])
            listenedNodes_.erase(listenedNodes_.begin() + i);
        else
            ++i;
    }
}

void CsgRoot::ScheduleDeferredRebuild()
{
    if (!IsEnabledEffective() || !node_)
        return;

    // If base model is not set, avoid scheduling rebuilds that can never succeed.
    if (GetModelAttr().name_.empty())
        return;

    WorkQueue* queue = GetSubsystem<WorkQueue>();
    if (!queue)
        return;

    if (deferredDelaySec_ <= 0.0f)
    {
        Rebuild();
        return;
    }

    if (deferredRebuildScheduled_)
        return;

    deferredRebuildScheduled_ = true;
    const unsigned scheduleToken = ++deferredScheduleToken_;

    queue->PostDelayedTaskForMainThread([weak = WeakPtr<CsgRoot>(this), scheduleToken](unsigned, WorkQueue* queue)
    {
        if (CsgRoot* root = weak)
            root->DeferredRebuildTick(scheduleToken, queue);
    });
}

void CsgRoot::DeferredRebuildTick(unsigned scheduleToken, WorkQueue* queue)
{
    if (scheduleToken != deferredScheduleToken_)
        return;

    if (!IsEnabledEffective() || !node_)
    {
        deferredRebuildScheduled_ = false;
        return;
    }

    if (!rebuildDirty_ || GetModelAttr().name_.empty())
    {
        deferredRebuildScheduled_ = false;
        return;
    }

    const unsigned nowMs = Time::GetSystemTime();
    const unsigned delayMs = static_cast<unsigned>(deferredDelaySec_ * 1000.0f);
    if (nowMs - lastChangeTimeMs_ >= delayMs)
    {
        deferredRebuildScheduled_ = false;
        Rebuild();
        return;
    }

    if (!queue)
    {
        deferredRebuildScheduled_ = false;
        return;
    }

    queue->PostDelayedTaskForMainThread([weak = WeakPtr<CsgRoot>(this), scheduleToken](unsigned, WorkQueue* queue)
    {
        if (CsgRoot* root = weak)
            root->DeferredRebuildTick(scheduleToken, queue);
    });
}

namespace CsgInternal
{

Vector3 GetMetricScale(const Vector3& worldScale)
{
    const auto axis = [](float value) { return std::isfinite(value) && value > 1e-6f ? value : 1.0f; };
    return Vector3(axis(worldScale.x_), axis(worldScale.y_), axis(worldScale.z_));
}

void ToNodeSpace(CsgTriangulatedModel& model, const Vector3& scale)
{
    if (scale == Vector3::ONE)
        return;

    const Vector3 inverse(1.0f / scale.x_, 1.0f / scale.y_, 1.0f / scale.z_);
    for (CsgTriangulatedModel::Geometry& geometry : model.geometries_)
    {
        for (ModelVertex& vertex : geometry.vertices_)
        {
            vertex.SetPosition(vertex.GetPosition() * inverse);
            // Normals use the inverse transpose of the position transform. Surface directions use it directly.
            if (vertex.HasNormal())
                vertex.SetNormal((vertex.GetNormal() * scale).NormalizedOrDefault(vertex.GetNormal()));
            if (vertex.HasTangent())
            {
                const Vector3 tangent = (vertex.GetTangent() * inverse).NormalizedOrDefault(vertex.GetTangent());
                vertex.tangent_ = tangent.ToVector4(vertex.tangent_.w_);
            }
            if (vertex.HasBinormal())
            {
                const Vector3 binormal = vertex.binormal_.ToVector3();
                vertex.binormal_ = (binormal * inverse).NormalizedOrDefault(binormal).ToVector4(vertex.binormal_.w_);
            }
        }
    }
    if (model.hasAnyVertex_)
        model.boundingBox_ = BoundingBox(model.boundingBox_.min_ * inverse, model.boundingBox_.max_ * inverse);
}

} // namespace CsgInternal

namespace
{

/// Set the flag if a bound of the solid is at or beyond the limit of the exact math.
void CheckSolidRange(const CsgSolid& solid, bool& outOfRange)
{
    if (solid.FaceCount() == 0)
        return;
    for (const Vector3* bound : {&solid.GetBoundsMin(), &solid.GetBoundsMax()})
    {
        if (!(Abs(bound->x_) < CsgInternal::CSG_EXACT_COORDINATE_LIMIT && Abs(bound->y_) < CsgInternal::CSG_EXACT_COORDINATE_LIMIT
                && Abs(bound->z_) < CsgInternal::CSG_EXACT_COORDINATE_LIMIT))
            outOfRange = true;
    }
}

} // namespace


void CsgRoot::Rebuild()
{
    if (!node_)
        return;

    if (rebuildInFlight_)
        return;

    StaticModel* tempB = GetOrCreateTempStaticModel("Operand", tempB_);
    StaticModel* tempBase = GetOrCreateTempStaticModel("Base", tempBase_);

    if (!tempB || !tempBase)
    {
        ApplyResultToOutput(nullptr, {});
        return;
    }

    // Base operand must be provided explicitly on CsgRoot.
    if (GetModelAttr().name_.empty())
    {
        URHO3D_LOGERRORF(
            "CsgRoot (%s:%u) has no <Model> set. CSG build is discarded.", node_->GetName().c_str(), node_->GetID());
        ApplyResultToOutput(nullptr, {});
        return;
    }

    if (!SetupOperandStaticModel(tempBase, GetModelAttr(), GetMaterialsAttr()))
    {
        ApplyResultToOutput(nullptr, {});
        return;
    }

    auto job = std::make_shared<BuildJob>();
    job->root_ = this;
    job->context_ = context_;
    job->buildId_ = rebuildRequestId_;
    job->componentId_ = GetID();
    job->epsilon_ = epsilon_;
    job->metricScale_ = CsgInternal::GetMetricScale(node_->GetWorldScale());
    job->baseModel_ = tempBase->GetModel();
    job->baseMaterials_ = tempBase->GetMaterialsAttr();

    // Prepare source lookup tables for output vertex layout and materials.
    // MeshId 0 is reserved for the base operand.
    job->sourceModels_.clear();
    job->sourceMaterials_.clear();
    job->sourceModels_.push_back(job->baseModel_);
    job->sourceMaterials_.push_back(job->baseMaterials_);

    // The base polygons are extracted only if the cached solid cannot be reused.
    job->rootStep_.model_ = job->baseModel_;
    job->rootStep_.materials_ = job->baseMaterials_;
    job->rootStep_.meshId_ = 0;

    // Build operation tree recursively starting from root node.
    const Matrix3x4 rootFrame = Matrix3x4(Vector3::ZERO, Quaternion::IDENTITY, job->metricScale_) *
        node_->GetWorldTransform().Inverse();
    if (!BuildOperationTree(job->rootStep_, node_, rootFrame, tempB, job->sourceModels_, job->sourceMaterials_))
    {
        cachedBrushes_.clear();
        cachedSolid_ = nullptr;
        // No operations: return base model directly.
        const ResourceRefList baseMaterials = tempBase->GetMaterialsAttr();
        ApplyResultToOutput(tempBase->GetModel(), baseMaterials);
        rebuildDirty_ = false;
        return;
    }

    // Reuse only for an append to a flat tree. A same-length rebuild may reflect an in-place
    // model edit even if the resource object and the brush references are the same.
    if (TryCaptureFlatBrushes(job->rootStep_, job->flatBrushes_) && cachedSolid_ && job->baseModel_ == cachedBaseModel_
        && job->epsilon_ == cachedEpsilon_ && job->metricScale_ == cachedMetricScale_
        && cachedBrushes_.size() < job->flatBrushes_.size())
    {
        bool prefixUnchanged = true;
        for (unsigned i = 0; i < cachedBrushes_.size(); ++i)
        {
            const CsgBrush* brush = job->flatBrushes_[i].brush_;
            if (cachedBrushes_[i] != job->flatBrushes_[i] || !brush || brush->IsDirty())
            {
                prefixUnchanged = false;
                break;
            }
        }
        if (prefixUnchanged)
        {
            job->cachedSolid_ = cachedSolid_;
            job->reusedRootChildren_ = cachedBrushes_.size();
        }
    }
    if (!job->cachedSolid_)
    {
        // The root frame has the world scale of the root but not its rotation or position.
        job->rootStep_.polygons_ = CsgBuildPolygonsFromModel(
            job->baseModel_.Get(), Matrix3x4(Vector3::ZERO, Quaternion::IDENTITY, job->metricScale_), 0);
    }

    // The map is built from the full source tables. A key of a reused brush stays stable: a new model
    // joins an older group or opens a new group, and the representative of a group is its first member.
    const ea::hash_map<unsigned, unsigned> canonicalSharedKeys =
        BuildCanonicalSharedKeyMap(job->sourceModels_, job->sourceMaterials_);
    const auto canonicalizeStep = [&](auto&& self, BrushStep& step) -> void
    {
        CanonicalizeSharedKeys(step.polygons_, canonicalSharedKeys);
        for (BrushStep& child : step.children_)
            self(self, child);
    };
    canonicalizeStep(canonicalizeStep, job->rootStep_);

    WorkQueue* workQueue = GetSubsystem<WorkQueue>();
    rebuildInFlight_ = true;
    rebuildInFlightId_ = job->buildId_;
    rebuildDirty_ = false;

    workQueue->PostTask([job](unsigned threadIndex, WorkQueue* queue) { ExecuteBuildJob(threadIndex, queue, job); },
        TaskPriority::Medium);
}

void CsgRoot::ExecuteBuildJob(unsigned threadIndex, WorkQueue* queue, const std::shared_ptr<BuildJob>& job)
{
    (void)threadIndex;

    HiresTimer timer;
    SharedPtr<const CsgSolid> solid = job->cachedSolid_;
    if (!solid)
    {
        solid = CsgSolid::FromOperand(ea::vector<CsgPolygon>(job->rootStep_.polygons_), job->epsilon_);
        CheckSolidRange(*solid, job->outOfRange_);
    }
    EvaluateOperationTree(job->rootStep_, solid, job->epsilon_, job->outOfRange_, job->reusedRootChildren_);
    job->resultSolid_ = solid;

    SharedPtr<CsgTriangulatedModel> triangulated;
    if (solid && solid->FaceCount() != 0)
    {
        triangulated = CsgTriangulateSolid(*solid, job->epsilon_);
        if (triangulated)
            CsgInternal::ToNodeSpace(*triangulated, job->metricScale_);
    }
    if (!triangulated)
        triangulated = MakeShared<CsgTriangulatedModel>();
    URHO3D_LOGDEBUG("CsgRoot({}): rebuild took {}.{}ms", job->componentId_, timer.GetUSec() / 1000, timer.GetUSec() % 1000);

    queue->PostDelayedTaskForMainThread([job, triangulated = ea::move(triangulated)](unsigned, WorkQueue*) mutable
    { ApplyBuildJobResultOnMainThread(job, ea::move(triangulated)); });
}

void CsgRoot::ApplyBuildJobResultOnMainThread(
    const std::shared_ptr<BuildJob>& job, SharedPtr<CsgTriangulatedModel> triangulated)
{
    CsgRoot* root = job->root_;
    if (!root)
        return;

    if (root->rebuildInFlightId_ != job->buildId_)
        return;

    root->rebuildInFlight_ = false;

    // Drop stale results.
    if (root->rebuildRequestId_ != job->buildId_)
    {
        if (root->rebuildDirty_)
            root->Rebuild();
        return;
    }

    if (job->outOfRange_)
    {
        URHO3D_LOGWARNING("CsgRoot ({}:{}): an operand has a coordinate at or beyond 65536 units; "
                          "the exact math is not reliable",
            root->GetNode() ? root->GetNode()->GetName() : EMPTY_STRING, root->GetNode() ? root->GetNode()->GetID() : 0u);
    }

    ResourceRefList materials;
    SharedPtr<Model> renderableModel;
    if (!triangulated)
    {
        // Triangulation failed or resulted in empty output.
        renderableModel = MakeShared<Model>(root->context_);
    }
    else
    {
        ea::vector<const Model*> sourceModels;
        sourceModels.reserve(job->sourceModels_.size());
        for (const SharedPtr<Model>& model : job->sourceModels_)
            sourceModels.push_back(model.Get());

        ea::vector<const ResourceRefList*> sourceMaterials;
        sourceMaterials.reserve(job->sourceMaterials_.size());
        for (const ResourceRefList& mats : job->sourceMaterials_)
            sourceMaterials.push_back(&mats);

        renderableModel = CsgBuildModel(root->context_, *triangulated,
            ea::span<const Model* const>(sourceModels.data(), sourceModels.size()),
            ea::span<const ResourceRefList* const>(sourceMaterials.data(), sourceMaterials.size()), &materials);
    }

    // Acknowledge changes.
    root->AcknowledgeBrushes(job->rootStep_);

    // Keep the solid as a base for an append only if the tree is flat.
    root->cachedSolid_ = job->flatBrushes_.size() == job->rootStep_.children_.size() ? job->resultSolid_ : nullptr;
    root->cachedBaseModel_ = job->baseModel_;
    root->cachedEpsilon_ = job->epsilon_;
    root->cachedMetricScale_ = job->metricScale_;
    root->cachedSourceModels_ = job->sourceModels_;
    root->cachedSourceMaterials_ = job->sourceMaterials_;
    root->cachedBrushes_.clear();
    if (root->cachedSolid_)
        root->cachedBrushes_ = job->flatBrushes_;

    if (root->autoApplyResults_)
        root->ApplyResultToOutput(renderableModel, materials);
    else
        root->pendingResults_.push_back({renderableModel, materials, job->buildId_});
}

bool CsgRoot::ApplyPendingResult()
{
    if (pendingResults_.empty())
        return false;

    const PendingBuildResult result = ea::move(pendingResults_.front());
    pendingResults_.erase(pendingResults_.begin());
    ApplyResultToOutput(result.model_, result.materials_);
    return true;
}

StaticModel* CsgRoot::GetOrCreateTempStaticModel(const ea::string& name, WeakPtr<StaticModel>& cache)
{
    if (!node_)
        return nullptr;

    Node* tempParent = tempNode_;
    if (!tempParent)
    {
        tempParent = node_->CreateTemporaryChild("__CsgRootTemp");
        tempParent->SetEnabled(false);
        tempNode_ = tempParent;
    }

    Node* child = GetOrCreateTemporaryChild(tempParent, name);

    if (!cache)
    {
        cache = child->GetOrCreateComponent<StaticModel>();
        cache->SetTemporary(true);
    }

    return cache;
}

bool CsgRoot::SetupOperandStaticModel(
    StaticModel* staticModel, const ResourceRef& modelAttr, const ResourceRefList& materialsAttr) const
{
    if (!staticModel)
        return false;

    const ea::string& modelName = modelAttr.name_;
    if (modelName.empty())
        return false;

    auto* cache = GetSubsystem<ResourceCache>();
    Model* model = cache->GetResource<Model>(modelName);
    if (!model)
        return false;

    staticModel->SetModel(model);

    // Material fallback policy:
    // If material list is empty, treat it as "use the model's default materials".
    // StaticModel::ApplyMaterialList() uses the model's resource name with extension .txt by default.
    if (materialsAttr.names_.empty())
        staticModel->ApplyMaterialList();
    else
        staticModel->SetMaterialsAttr(materialsAttr);

    return true;
}

bool CsgRoot::SetupOperandStaticModel(StaticModel* staticModel, const CsgBrush* brush) const
{
    if (!staticModel || !brush)
        return false;

    return SetupOperandStaticModel(staticModel, brush->GetModelAttr(), brush->GetMaterialsAttr());
}

bool CsgRoot::BuildOperationTree(BrushStep& rootStep, Node* node, const Matrix3x4& rootFrame, StaticModel* tempModel,
    ea::vector<SharedPtr<Model>>& sourceModels, ea::vector<ResourceRefList>& sourceMaterials)
{

    if (!node)
        return false;

    const unsigned numChildren = node->GetNumChildren();
    for (unsigned i = 0; i < numChildren; ++i)
    {
        Node* child = node->GetChild(i);
        if (!child || !child->IsEnabled())
            continue;

        // Ignore nested CsgRoots (except this one).
        if (CsgRoot* nestedRoot = child->GetComponent<CsgRoot>())
        {
            if (nestedRoot != this)
                continue;
        }

        if (CsgBrush* brush = child->GetComponent<CsgBrush>())
        {
            if (!brush->IsEnabledEffective())
                continue;

            // Ensure we listen to this brush even if it wasn't explicitly registered yet.
            RegisterBrushInternal(brush, false);

            if (!SetupOperandStaticModel(tempModel, brush))
                continue;

            BrushStep step;
            step.model_ = tempModel->GetModel();

            step.materials_ = tempModel->GetMaterialsAttr();
            step.meshId_ = sourceModels.size();
            step.op_ = brush->GetOperation();
            step.brush_ = brush;

            // Store source metadata for final model build on the main thread.
            sourceModels.push_back(step.model_);
            sourceMaterials.push_back(step.materials_);

            // Extract polygons in the root frame (main thread).
            step.transform_ = rootFrame * child->GetWorldTransform();
            step.polygons_ = CsgBuildPolygonsFromModel(step.model_.Get(), step.transform_, step.meshId_);

            // Recurse into the brush subtree.
            BuildOperationTree(step, child, rootFrame, tempModel, sourceModels, sourceMaterials);
            rootStep.children_.push_back(ea::move(step));
        }
        else
        {
            // Recurse into nodes without brushes.
            BuildOperationTree(rootStep, child, rootFrame, tempModel, sourceModels, sourceMaterials);
        }
    }

    return !rootStep.children_.empty();
}

bool CsgRoot::TryCaptureFlatBrushes(const BrushStep& rootStep, ea::vector<BrushFingerprint>& output)
{
    for (const BrushStep& child : rootStep.children_)
    {
        if (!child.children_.empty() || !child.brush_)
        {
            output.clear();
            return false;
        }
        output.push_back({child.brush_, child.model_, child.transform_, child.meshId_, child.op_});
    }
    return true;
}

void CsgRoot::EvaluateOperationTree(const BrushStep& step, SharedPtr<const CsgSolid>& currentSolid, float epsilon,
    bool& outOfRange, unsigned startChild)
{
    for (unsigned childIndex = startChild; childIndex < step.children_.size(); ++childIndex)
    {
        const BrushStep& child = step.children_[childIndex];
        SharedPtr<const CsgSolid> childSolid = CsgSolid::FromOperand(ea::vector<CsgPolygon>(child.polygons_), epsilon);
        CheckSolidRange(*childSolid, outOfRange);
        if (!child.children_.empty())
            EvaluateOperationTree(child, childSolid, epsilon, outOfRange);

        // A null result is an empty solid.
        SharedPtr<const CsgSolid> result = CsgBooleanOperation(currentSolid.Get(), childSolid.Get(), child.op_, epsilon);
        currentSolid = result ? result : SharedPtr<const CsgSolid>(CsgSolid::FromOperand({}, epsilon));
    }
}

void CsgRoot::AcknowledgeBrushes(const BrushStep& step)
{
    for (const BrushStep& child : step.children_)
    {
        if (CsgBrush* brush = child.brush_)
            brush->ClearDirty();

        AcknowledgeBrushes(child);
    }
}

} // namespace Urho3D
