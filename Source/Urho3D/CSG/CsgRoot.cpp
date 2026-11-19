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
#include <Urho3D/Resource/ResourceEvents.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>

#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <utility>

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
};

struct CsgRoot::BuildJob
{
    WeakPtr<CsgRoot> root_;
    Context* context_{};
    unsigned buildId_{};
    unsigned componentId_{};
    float epsilon_{};
    /// Minimum volume of a shell as a fraction of the largest shell. See `CsgRoot::SetMinComponentVolumeFraction`.
    float minComponentVolumeFraction_{};
    /// World scale of the root. The root frame carries this scale.
    Vector3 metricScale_{Vector3::ONE};

    SharedPtr<Model> baseModel_;
    ResourceRefList baseMaterials_;
    /// Polygons of the base operand in the root frame.
    ea::vector<CsgPolygon> basePolygons_;
    /// Operands in operation order.
    ea::vector<BrushStep> operands_;

    ea::vector<SharedPtr<Model>> sourceModels_;
    ea::vector<ResourceRefList> sourceMaterials_;

    /// Solid of the base operand. If null, the worker thread makes it from the base polygons.
    SharedPtr<const CsgSolid> baseSolid_;
    /// Fold state after each operand. The main thread puts the reused states here.
    /// The worker thread appends the states of the other operands.
    ea::vector<SharedPtr<const CsgSolid>> stepSolids_;
    /// Number of operands with a reused fold state.
    unsigned reusedOperands_{};
    /// Fingerprints of the operands. Empty if a brush is dead.
    ea::vector<BrushFingerprint> capturedBrushes_;
    /// Final solid of the job after the shell filter. The worker thread sets it.
    /// It is null if no shell dropped. Then the last fold state is the final solid.
    SharedPtr<const CsgSolid> resultSolid_;
    /// Statistics of the job. The worker thread fills it before it posts the main thread task.
    /// The main thread task adds the model build time. The posted task orders the two writes, so there is no data race.
    CsgBuildStats stats_;
    /// True if the boolean threw. The worker thread sets it before it posts the main thread task.
    bool failed_{};
};

namespace
{

/// Test only action that the next build job runs. The mutex guards it, as the main thread sets it and a worker takes it.
std::mutex testBuildHookMutex;
std::function<void()> testBuildHook;

/// Take the test action. The job takes it once.
std::function<void()> TakeTestBuildHook()
{
    const std::lock_guard<std::mutex> lock(testBuildHookMutex);
    std::function<void()> hook = std::move(testBuildHook);
    testBuildHook = nullptr;
    return hook;
}

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

/// Name of the temporary child node that holds the result.
const char* const CSG_RESULT_NODE_NAME = "CsgResult";

/// Return true if the node can be the result node: the reserved name, no children and only StaticModel components.
bool IsResultNodeCandidate(const Node* node)
{
    if (!node || node->GetName() != CSG_RESULT_NODE_NAME || node->GetNumChildren() != 0)
        return false;
    for (const SharedPtr<Component>& component : node->GetComponents())
    {
        if (component->GetType() != StaticModel::GetTypeStatic())
            return false;
    }
    return true;
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
    URHO3D_ATTRIBUTE("Min Component Volume Fraction", float, minComponentVolumeFraction_, 0.0f, AM_DEFAULT);
    URHO3D_COPY_BASE_ATTRIBUTES(CsgComponent);
    URHO3D_ACTION_STATIC_LABEL("Rebuild", Rebuild, "Rebuild component CSG now");
    URHO3D_ACTION_STATIC_LABEL("Add Brush", AddBrush, "Create new brush child node");
    // The editor shows the status. The scene does not save it, and a set of the value has no effect.
    URHO3D_CUSTOM_ACCESSOR_ATTRIBUTE(
        "Build Status", [](const CsgRoot& self, Variant& value) { value = self.GetBuildStatus(); },
        [](CsgRoot&, const Variant&) {}, ea::string, EMPTY_STRING, AM_EDIT | AM_READONLY);
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
    UpdateResultEnabled();
}

void CsgRoot::OnNodeSet(Node* previousNode, Node* currentNode)
{
    Component::OnNodeSet(previousNode, currentNode);
    // The result node belongs to the old node. A node that the scene destroys has no children at this time.
    if (previousNode != currentNode)
        RemoveResultNode();
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

    MarkRebuildRequested();

    ScheduleDeferredRebuild();
}

void CsgRoot::OnModelChanged()
{
    BaseClassName::OnModelChanged();
    ClearBuildCache();
    baked_.reset();
    RequestRebuild();
}

void CsgRoot::OnMaterialsChanged()
{
    BaseClassName::OnMaterialsChanged();
    // The canonical shared keys of the cached solid depend on the materials.
    ClearBuildCache();
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

void CsgRoot::SetMinComponentVolumeFraction(float value)
{
    minComponentVolumeFraction_ = ea::clamp(value, 0.0f, 1.0f);
    RequestRebuild();
}

void CsgRoot::RequestRebuild()
{
    MarkRebuildRequested();

    ScheduleDeferredRebuild();
}

void CsgRoot::MarkRebuildRequested()
{
    // The wait starts at the first open change. Later changes do not reset it.
    if (!rebuildDirty_ || !deferredRebuildScheduled_)
        waitStartTimeMs_ = Time::GetSystemTime();
    rebuildDirty_ = true;
    ++rebuildRequestId_;
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

    Node* resultNode = resultNode_;
    if (!resultNode || resultNode->GetParent() != node_)
    {
        output_ = nullptr;
        resultNode = nullptr;
        // A copy of the child list, because the loop removes nodes. A duplicate can come from a scene file
        // that a tool saved with temporary nodes. The root keeps the first node and removes the others.
        // Only a node with no children and no components other than StaticModel is a candidate.
        // The root does not change or remove another node with the reserved name. It makes its own node.
        const ea::vector<SharedPtr<Node>> children = node_->GetChildren();
        for (const SharedPtr<Node>& child : children)
        {
            if (!IsResultNodeCandidate(child))
                continue;
            if (!resultNode)
                resultNode = child;
            else
                child->Remove();
        }
        // The child is not a brush, so the node events and the dirty listeners do not request a rebuild.
        if (!resultNode)
            resultNode = node_->CreateTemporaryChild(CSG_RESULT_NODE_NAME);
        resultNode->SetTemporary(true);
        resultNode->SetTransform(Vector3::ZERO, Quaternion::IDENTITY, Vector3::ONE);
        resultNode_ = resultNode;
    }

    if (!output_)
    {
        output_ = resultNode->GetOrCreateComponent<StaticModel>();
        output_->SetTemporary(true);
    }
    UpdateResultEnabled();
    return output_;
}

void CsgRoot::UpdateResultEnabled()
{
    // A deep enable of the root node can enable the child node too. The disabled model stays hidden then.
    const bool enabled = IsEnabledEffective();
    if (Node* resultNode = resultNode_)
        resultNode->SetEnabled(enabled);
    if (StaticModel* output = output_)
        output->SetEnabled(enabled);
}

void CsgRoot::RemoveResultNode()
{
    // Clear the pointers first. The removal can destroy the node and send scene events.
    const SharedPtr<Node> resultNode{resultNode_.Get()};
    resultNode_ = nullptr;
    output_ = nullptr;
    if (resultNode)
        resultNode->Remove();
}

Node* CsgRoot::GetResultNode() const
{
    return resultNode_;
}

StaticModel* CsgRoot::GetResultModel() const
{
    return output_;
}

ea::string CsgRoot::GetBuildStatus() const
{
    if (rebuildInFlight_)
        return "Building...";
    if (lastBuildFailed_)
        return "Failed";
    if (!hasAppliedBuild_)
        return "Not built";
    if (!lastBuildStats_.valid_)
        return "Invalid";
    const float totalMs = lastBuildStats_.booleanMs_ + lastBuildStats_.triangulateMs_ + lastBuildStats_.buildModelMs_;
    return Format("OK: {:.1f} ms, {} triangles", totalMs, lastBuildStats_.triangleCount_);
}

void CsgRoot::AddBrush()
{
    if (!node_)
        return;

    Node* child = node_->CreateChild("Brush");
    child->CreateComponent<CsgBrush>();
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
    UnsubscribeFromEvent(E_NODEADDED);
    UnsubscribeFromEvent(E_NODEREMOVED);
    UnsubscribeFromEvent(this, StringHash("CsgRequestRebuild"));

    if (!isActive)
    {
        return;
    }

    // A reparent of a child does not call OnNodeSet. The scene events report it.
    SubscribeToEvent(scene, E_NODEADDED, URHO3D_HANDLER(CsgRoot, HandleNodeAdded));
    SubscribeToEvent(scene, E_NODEREMOVED, URHO3D_HANDLER(CsgRoot, HandleNodeRemoved));
    // The editor sends this event after a change that no other event reports, for example a child reorder.
    SubscribeToEvent(this, StringHash("CsgRequestRebuild"), URHO3D_HANDLER(CsgRoot, HandleRequestRebuild));

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

void CsgRoot::UpdateWatchedModels(const BuildJob* job)
{
    // The set covers each model with geometry in the cache or in the build in flight. A reload changes the
    // geometry in place. The root listens also while it is inactive, because the cache stays.
    ea::vector<Model*> models;
    const auto add = [&models](Model* model)
    {
        if (model && ea::find(models.begin(), models.end(), model) == models.end())
            models.push_back(model);
    };
    if (cachedBaseSolid_)
    {
        add(cachedBaseModel_);
        for (const BrushFingerprint& fingerprint : cachedBrushes_)
            add(fingerprint.model_);
    }
    if (job)
    {
        add(job->baseModel_);
        for (const BrushStep& operand : job->operands_)
            add(operand.model_);
    }
    // The handler of a reload can start a build. The root must not remove the handler that runs.
    add(reloadingModel_);

    for (unsigned i = 0; i < watchedModels_.size();)
    {
        Model* model = watchedModels_[i];
        if (model && ea::find(models.begin(), models.end(), model) != models.end())
        {
            ++i;
            continue;
        }
        if (model)
            UnsubscribeFromEvent(model, E_RELOADFINISHED);
        watchedModels_.erase(watchedModels_.begin() + i);
    }
    for (Model* model : models)
    {
        const auto isWatched = [model](const WeakPtr<Model>& watched) { return watched == model; };
        if (ea::find_if(watchedModels_.begin(), watchedModels_.end(), isWatched) != watchedModels_.end())
            continue;
        watchedModels_.push_back(WeakPtr<Model>(model));
        SubscribeToEvent(model, E_RELOADFINISHED, URHO3D_HANDLER(CsgRoot, HandleModelReloaded));
    }
}

void CsgRoot::HandleRequestRebuild(StringHash /*eventType*/, VariantMap& /*eventData*/)
{
    RequestRebuild();
}

void CsgRoot::HandleModelReloaded(StringHash /*eventType*/, VariantMap& /*eventData*/)
{
    // An inactive root does not start a build here. ScheduleDeferredRebuild checks the state.
    // A model can change in place during a job. The result of that job reads the model, so it is dropped.
    if (rebuildInFlight_)
        modelReloadedDuringJob_ = true;
    reloadingModel_ = static_cast<Model*>(GetEventSender());
    ClearBuildCache();
    RequestRebuild();
    reloadingModel_ = nullptr;
}

void CsgRoot::HandleNodeAdded(StringHash /*eventType*/, VariantMap& eventData)
{
    using namespace NodeAdded;
    if (eventData[P_PARENT].GetPtr() != node_)
        return;

    // The brush leaves its old root and registers with this root. Both roots request a rebuild.
    auto* child = static_cast<Node*>(eventData[P_NODE].GetPtr());
    if (CsgBrush* brush = child ? child->GetDerivedComponent<CsgBrush>() : nullptr)
        brush->UpdateRootRegistration();
}

void CsgRoot::HandleNodeRemoved(StringHash /*eventType*/, VariantMap& eventData)
{
    using namespace NodeRemoved;
    if (eventData[P_PARENT].GetPtr() != node_)
        return;

    // The node still has this parent during the event, so the brush cannot find its new root yet.
    // The root releases the brush now. The new root registers it when it receives the node.
    auto* child = static_cast<Node*>(eventData[P_NODE].GetPtr());
    CsgBrush* brush = child ? child->GetDerivedComponent<CsgBrush>() : nullptr;
    if (!brush)
        return;

    if (brush->root_ == this)
        brush->root_ = nullptr;
    // The rebuild request can start a build at once. That build must not use the node that leaves.
    removedChild_ = child;
    UnregisterBrushInternal(brush);
    removedChild_ = nullptr;
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

    WorkQueue* queue = GetSubsystem<WorkQueue>();
    if (!queue)
        return;

    // Bake removes nodes. It requests one rebuild after the last removal.
    if (baking_)
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

    if (!rebuildDirty_)
    {
        deferredRebuildScheduled_ = false;
        return;
    }

    const unsigned nowMs = Time::GetSystemTime();
    const unsigned delayMs = static_cast<unsigned>(deferredDelaySec_ * 1000.0f);
    if (nowMs - waitStartTimeMs_ >= delayMs)
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

ea::vector<CsgPolygon> RescalePolygons(const CsgSolid& solid, const Vector3& factor)
{
    if (factor == Vector3::ONE)
        return solid.ToPolygons();

    const Vector3 normalFactor(1.0f / factor.x_, 1.0f / factor.y_, 1.0f / factor.z_);
    ea::vector<CsgPolygon> result;
    result.reserve(solid.FaceCount());
    for (unsigned faceIndex = 0; faceIndex < solid.FaceCount(); ++faceIndex)
    {
        const CsgPolygon& face = *solid.GetFace(faceIndex);
        ea::vector<ModelVertex> vertices = face.vertices_;
        for (ModelVertex& vertex : vertices)
        {
            vertex.SetPosition(vertex.GetPosition() * factor);
            if (vertex.HasNormal())
                vertex.SetNormal((vertex.GetNormal() * normalFactor).NormalizedOrDefault(vertex.GetNormal()));
        }
        result.emplace_back(ea::move(vertices), face.shared_);
    }
    return result;
}

} // namespace CsgInternal

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
        lastGeometryComponents_.clear();
        return;
    }

    // An empty <Model> means an empty start solid. The fold applies the operands on it.
    const bool hasBaseModel = !GetModelAttr().name_.empty();
    if (hasBaseModel && !SetupOperandStaticModel(tempBase, GetModelAttr(), GetMaterialsAttr()))
    {
        ApplyResultToOutput(nullptr, {});
        lastGeometryComponents_.clear();
        return;
    }

    auto job = std::make_shared<BuildJob>();
    job->root_ = this;
    job->context_ = context_;
    job->buildId_ = rebuildRequestId_;
    job->componentId_ = GetID();
    job->epsilon_ = epsilon_;
    job->minComponentVolumeFraction_ = minComponentVolumeFraction_;
    job->metricScale_ = CsgInternal::GetMetricScale(node_->GetWorldScale());
    if (hasBaseModel)
    {
        job->baseModel_ = tempBase->GetModel();
        job->baseMaterials_ = tempBase->GetMaterialsAttr();
    }

    // A bake belongs to the base operand and to the epsilon that it was made with.
    if (baked_ && (baked_->baseModel_ != job->baseModel_ || baked_->epsilon_ != epsilon_))
        baked_.reset();

    // Prepare source lookup tables for output vertex layout and materials.
    // MeshId 0 is reserved for the base operand. An empty base keeps a null entry there. Baked faces refer to the tables of the build
    // that made them, so the new brushes extend these tables.
    job->sourceModels_.clear();
    job->sourceMaterials_.clear();
    if (baked_)
    {
        job->sourceModels_ = baked_->sourceModels_;
        job->sourceMaterials_ = baked_->sourceMaterials_;
    }
    else
    {
        job->sourceModels_.push_back(job->baseModel_);
        job->sourceMaterials_.push_back(job->baseMaterials_);
    }

    // The base polygons are extracted only if the cached solid cannot be reused.
    const Matrix3x4 rootFrame = Matrix3x4(Vector3::ZERO, Quaternion::IDENTITY, job->metricScale_) *
        node_->GetWorldTransform().Inverse();
    const bool hasOperands = BuildOperands(job->operands_, rootFrame, tempB, job->sourceModels_, job->sourceMaterials_);
    UpdateWatchedModels(job.get());
    if (!hasOperands)
    {
        ClearBuildCache();
        rebuildDirty_ = false;
        if (baked_)
        {
            // No new operations after a bake: the baked result is the output and the base for new brushes.
            cachedSolid_ = baked_->solid_;
            cachedBaseSolid_ = baked_->solid_;
            cachedBaseModel_ = job->baseModel_;
            cachedEpsilon_ = baked_->epsilon_;
            cachedMetricScale_ = baked_->metricScale_;
            cachedMinComponentVolumeFraction_ = minComponentVolumeFraction_;
            const ResourceRefList outputMaterials = baked_->outputMaterials_;
            ApplyResultToOutput(baked_->outputModel_, outputMaterials);
            lastGeometryComponents_.clear();
            lastBuildFailed_ = false;
            return;
        }
        // No operations: return base model directly. An empty base gives an empty output.
        if (hasBaseModel)
            ApplyResultToOutput(tempBase->GetModel(), tempBase->GetMaterialsAttr());
        else
            ApplyResultToOutput(nullptr, {});
        lastGeometryComponents_.clear();
        lastBuildFailed_ = false;
        return;
    }

    // Reuse the fold states before the first changed operand. A change of the base, the epsilon,
    // the scale or the shell filter discards all states.
    const bool allCaptured = TryCaptureBrushes(job->operands_, job->capturedBrushes_);
    if (allCaptured && cachedBaseSolid_ && job->baseModel_ == cachedBaseModel_ && job->epsilon_ == cachedEpsilon_
        && job->metricScale_ == cachedMetricScale_
        && job->minComponentVolumeFraction_ == cachedMinComponentVolumeFraction_)
    {
        unsigned firstChanged = 0;
        const unsigned count = ea::min(cachedBrushes_.size(), job->capturedBrushes_.size());
        while (firstChanged < count)
        {
            const CsgBrush* brush = job->capturedBrushes_[firstChanged].brush_;
            // The fingerprint compares the model pointer. A model change without an event, for example SetGeometry, is not detected.
            if (cachedBrushes_[firstChanged] != job->capturedBrushes_[firstChanged] || !brush || brush->IsDirty())
                break;
            ++firstChanged;
        }
        // No visible change can mean an in-place model edit. Then the job evaluates all operands again.
        const bool unchanged =
            firstChanged == cachedBrushes_.size() && firstChanged == job->capturedBrushes_.size();
        if (!unchanged)
        {
            job->baseSolid_ = cachedBaseSolid_;
            job->stepSolids_.assign(cachedStepSolids_.begin(), cachedStepSolids_.begin() + firstChanged);
            job->reusedOperands_ = firstChanged;
        }
    }
    // The worker thread must not move the reused states, so the vector gets its full size here.
    job->stepSolids_.reserve(job->operands_.size());
    if (!job->baseSolid_)
    {
        if (baked_)
        {
            const Vector3& oldScale = baked_->metricScale_;
            const Vector3 factor(job->metricScale_.x_ / oldScale.x_, job->metricScale_.y_ / oldScale.y_,
                job->metricScale_.z_ / oldScale.z_);
            // The baked polygons have float positions, and FromOperand snaps them to the grid again.
            // In float mode, and in double mode below 256 units, each float is its grid point, so nothing moves.
            // Above 256 units in double mode, a vertex can move by one grid step in this round trip.
            job->basePolygons_ = CsgInternal::RescalePolygons(*baked_->solid_, factor);
        }
        else if (hasBaseModel)
        {
            // The root frame has the world scale of the root but not its rotation or position.
            job->basePolygons_ = CsgBuildPolygonsFromModel(
                job->baseModel_.Get(), Matrix3x4(Vector3::ZERO, Quaternion::IDENTITY, job->metricScale_), 0);
        }
    }

    // The map is built from the full source tables. A key of a reused brush stays stable: a new model
    // joins an older group or opens a new group, and the representative of a group is its first member.
    const ea::hash_map<unsigned, unsigned> canonicalSharedKeys =
        BuildCanonicalSharedKeyMap(job->sourceModels_, job->sourceMaterials_);
    CanonicalizeSharedKeys(job->basePolygons_, canonicalSharedKeys);
    for (BrushStep& operand : job->operands_)
        CanonicalizeSharedKeys(operand.polygons_, canonicalSharedKeys);

    WorkQueue* workQueue = GetSubsystem<WorkQueue>();
    rebuildInFlight_ = true;
    modelReloadedDuringJob_ = false;
    rebuildInFlightId_ = job->buildId_;
    rebuildDirty_ = false;

    workQueue->PostTask([job](unsigned threadIndex, WorkQueue* queue) { ExecuteBuildJob(threadIndex, queue, job); },
        TaskPriority::Medium);
}

void CsgRoot::SetTestBuildHook(std::function<void()> hook)
{
    const std::lock_guard<std::mutex> lock(testBuildHookMutex);
    testBuildHook = std::move(hook);
}

void CsgRoot::ExecuteBuildJob(unsigned threadIndex, WorkQueue* queue, const std::shared_ptr<BuildJob>& job)
{
    (void)threadIndex;

    // An exception must not skip the post. The main thread waits for it and the root stays in flight without it.
    SharedPtr<CsgTriangulatedModel> triangulated;
    try
    {
        if (const std::function<void()> hook = TakeTestBuildHook())
            hook();
        HiresTimer timer;
        if (!job->baseSolid_)
            job->baseSolid_ = CsgSolid::FromOperand(ea::vector<CsgPolygon>(job->basePolygons_), job->epsilon_);
        EvaluateOperands(job->operands_, job->baseSolid_, job->stepSolids_, job->epsilon_);
        job->stats_.foldSteps_ = job->operands_.size() - job->reusedOperands_;
        // The shell filter applies to the final solid only. The fold states keep all shells.
        // A raw pointer to the last fold state prevents reference count changes that are not necessary.
        const CsgSolid* solid = job->stepSolids_.back().Get();
        unsigned droppedComponents = 0;
        job->resultSolid_ =
            CsgSolid::DropSmallComponentsOrNull(*solid, job->minComponentVolumeFraction_, &droppedComponents);
        if (job->resultSolid_)
            solid = job->resultSolid_.Get();
        const long long booleanUSec = timer.GetUSec(true);

        CsgModelCheck check;
        if (solid && solid->FaceCount() != 0)
        {
            triangulated = CsgTriangulateSolid(*solid, job->epsilon_);
            if (triangulated)
            {
                // The check runs over the triangles that go to the output, so a defect of the triangulation shows too.
                // It runs before the scale, as the grid of the check (2^-24 units) is the grid of the root frame.
                check = CsgCheckModel(*triangulated);
                CsgInternal::ToNodeSpace(*triangulated, job->metricScale_);
            }
        }
        if (!triangulated)
            triangulated = MakeShared<CsgTriangulatedModel>();
        const long long triangulateUSec = timer.GetUSec(false);

        unsigned triangleCount = 0;
        for (const CsgTriangulatedModel::Geometry& geometry : triangulated->geometries_)
            triangleCount += geometry.indexData_.size() / 3;
        job->stats_.booleanMs_ = static_cast<float>(booleanUSec) * 0.001f;
        job->stats_.triangulateMs_ = static_cast<float>(triangulateUSec) * 0.001f;
        job->stats_.faceCount_ = solid ? solid->FaceCount() : 0;
        job->stats_.triangleCount_ = triangleCount;
        job->stats_.componentCount_ = solid ? solid->ComponentCount() : 0;
        job->stats_.droppedComponents_ = droppedComponents;
        job->stats_.closed_ = solid ? solid->IsClosed() : true;
        job->stats_.volume_ = solid ? solid->Volume() : 0.0;
        job->stats_.openEdges_ = check.openEdges_;
        job->stats_.duplicates_ = check.duplicates_;
        job->stats_.degenerates_ = check.degenerates_;
        job->stats_.valid_ = job->stats_.faceCount_ == 0
            || (job->stats_.closed_ && check.openEdges_ == 0 && check.duplicates_ == 0 && check.degenerates_ == 0
                && check.volume_ > 0.0);

        const long long elapsed = booleanUSec + triangulateUSec;
        URHO3D_LOGDEBUG("CsgRoot({}): rebuild took {}.{}ms", job->componentId_, elapsed / 1000, elapsed % 1000);
    }
    catch (const std::exception& e)
    {
        job->failed_ = true;
        triangulated = nullptr;
        URHO3D_LOGERROR("CsgRoot({}): the rebuild failed, the output does not change: {}", job->componentId_, e.what());
    }
    catch (...)
    {
        job->failed_ = true;
        triangulated = nullptr;
        URHO3D_LOGERROR("CsgRoot({}): the rebuild failed, the output does not change", job->componentId_);
    }

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
    const bool reloaded = root->modelReloadedDuringJob_;
    root->modelReloadedDuringJob_ = false;

    // A result is stale if a change came after the job started. A stale result still shows if a newer
    // rebuild is pending, because that rebuild replaces it. This gives live updates during a drag.
    // The root drops a stale result of a failed job, of a job with a model that reloaded, of a root that is not active, or when auto-apply is off.
    const bool stale = root->rebuildRequestId_ != job->buildId_;
    if (stale && (job->failed_ || reloaded || !root->autoApplyResults_ || !root->rebuildDirty_ || !root->IsEnabledEffective() || !root->node_))
    {
        root->UpdateWatchedModels(nullptr);
        if (root->rebuildDirty_)
            root->Rebuild();
        return;
    }

    // A failed build changes nothing: the cache, the dirty brushes and the output stay as they are.
    // The watch set drops the models of the failed build that the cache does not use.
    if (job->failed_)
    {
        root->UpdateWatchedModels(nullptr);
        if (root->autoApplyResults_)
            root->lastBuildFailed_ = true;
        else
        {
            PendingBuildResult failure;
            failure.model_ = MakeShared<Model>(root->context_);
            failure.revision_ = job->buildId_;
            failure.failed_ = true;
            root->pendingResults_.push_back(ea::move(failure));
        }
        return;
    }

    ResourceRefList materials;
    ea::vector<unsigned> components;
    SharedPtr<Model> renderableModel;
    HiresTimer buildModelTimer;
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
            ea::span<const ResourceRefList* const>(sourceMaterials.data(), sourceMaterials.size()), &materials,
            &components);
    }

    job->stats_.buildModelMs_ = static_cast<float>(buildModelTimer.GetUSec(false)) * 0.001f;

    // Acknowledge changes. A stale result keeps the dirty flags, so the next build checks the brushes again.
    if (!stale)
        root->AcknowledgeBrushes(job->operands_);

    // A stale result leaves the cache as it is. The fingerprints decide the reuse for the next build.
    if (stale)
        root->UpdateWatchedModels(nullptr);
    else
    {
        // Keep the fold states only if all brushes are alive.
        // An invalid result is never cached: the next rebuild starts from the brushes again.
        root->ClearBuildCache();
        if (job->stats_.valid_ && job->capturedBrushes_.size() == job->operands_.size())
        {
            // A null result solid means that no shell dropped. Then the final solid is the last fold state.
            root->cachedSolid_ = job->resultSolid_ ? job->resultSolid_ : job->stepSolids_.back();
            root->cachedBaseSolid_ = job->baseSolid_;
            root->cachedStepSolids_ = job->stepSolids_;
            root->cachedBrushes_ = job->capturedBrushes_;
        }
        root->cachedBaseModel_ = job->baseModel_;
        root->cachedEpsilon_ = job->epsilon_;
        root->cachedMetricScale_ = job->metricScale_;
        root->cachedMinComponentVolumeFraction_ = job->minComponentVolumeFraction_;
        root->cachedSourceModels_ = job->sourceModels_;
        root->cachedSourceMaterials_ = job->sourceMaterials_;
        // The watch set now follows the stored cache.
        root->UpdateWatchedModels(nullptr);
    }

    if (root->autoApplyResults_)
    {
        root->ApplyResultToOutput(renderableModel, materials);
        root->lastBuildStats_ = job->stats_;
        root->lastGeometryComponents_ = ea::move(components);
        root->lastBuildFailed_ = false;
        root->hasAppliedBuild_ = true;
    }
    else
    {
        PendingBuildResult pending{renderableModel, materials, job->buildId_, job->stats_};
        pending.components_ = ea::move(components);
        root->pendingResults_.push_back(ea::move(pending));
    }

    // The changes that came during the job start a new wait now.
    if (stale)
    {
        if (!root->deferredRebuildScheduled_)
            root->waitStartTimeMs_ = Time::GetSystemTime();
        root->ScheduleDeferredRebuild();
    }
}

bool CsgRoot::ApplyPendingResult()
{
    if (pendingResults_.empty())
        return false;

    const PendingBuildResult result = ea::move(pendingResults_.front());
    pendingResults_.erase(pendingResults_.begin());
    lastBuildFailed_ = result.failed_;
    if (result.failed_)
        return false;

    ApplyResultToOutput(result.model_, result.materials_);
    lastBuildStats_ = result.stats_;
    lastGeometryComponents_ = result.components_;
    hasAppliedBuild_ = true;
    return true;
}

bool CsgRoot::Bake()
{
    // After a failed build the cache does not match the inputs, so the root must not bake it.
    if (!node_ || rebuildInFlight_ || rebuildDirty_ || lastBuildFailed_ || HasPendingResult() || !cachedSolid_
        || cachedBrushes_.empty() || cachedSourceModels_.empty())
    {
        return false;
    }
    for (const BrushFingerprint& fingerprint : cachedBrushes_)
    {
        const CsgBrush* brush = fingerprint.brush_;
        if (brush && brush->IsDirty())
            return false;
    }

    const StaticModel* output = GetResultModel();
    if (!output || !output->GetModel())
        return false;

    // The baked state must exist before the brush nodes go away. Each removal requests a rebuild.
    auto baked = ea::make_unique<BakedState>();
    baked->solid_ = cachedSolid_;
    baked->metricScale_ = cachedMetricScale_;
    baked->epsilon_ = cachedEpsilon_;
    baked->baseModel_ = cachedBaseModel_;
    baked->sourceModels_ = cachedSourceModels_;
    baked->sourceMaterials_ = cachedSourceMaterials_;
    baked->outputModel_ = output->GetModel();
    baked->outputMaterials_ = output->GetMaterialsAttr();
    baked_ = ea::move(baked);

    // Removal of a node calls UnregisterBrush, so the loop uses copies.
    const ea::vector<BrushFingerprint> brushes = cachedBrushes_;
    // The fold states contain the unbaked base. The baked solid is the new base.
    ClearBuildCache();
    cachedSolid_ = baked_->solid_;
    cachedBaseSolid_ = baked_->solid_;
    ea::vector<WeakPtr<Node>> brushNodes;
    for (const BrushFingerprint& fingerprint : brushes)
    {
        CsgBrush* brush = fingerprint.brush_;
        if (brush && brush->GetNode())
            brushNodes.push_back(WeakPtr<Node>(brush->GetNode()));
    }
    // Each removal requests a rebuild. The rebuild waits until the last removal, so it never sees a node in removal.
    baking_ = true;
    for (const WeakPtr<Node>& brushNode : brushNodes)
    {
        if (Node* removed = brushNode)
            removed->Remove();
    }
    baking_ = false;
    RequestRebuild();
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

    // A procedural brush gives its own model. A null model is an invalid procedural brush: no operand.
    SharedPtr<Model> generatedModel;
    if (brush->GetGeneratedModel(generatedModel))
    {
        if (!generatedModel)
            return false;

        staticModel->SetModel(generatedModel);
        // A generated model has no name, so it has no material list file. Empty materials clear the old ones.
        const ResourceRefList& materialsAttr = brush->GetMaterialsAttr();
        if (materialsAttr.names_.empty())
        {
            staticModel->SetMaterial(nullptr);
            return true;
        }

        // An empty or missing slot uses the material of slot 0. This is done before the material names
        // are compared for the output geometries, so equal faces merge into one output geometry.
        ResourceRefList resolved(materialsAttr.type_);
        const unsigned numGeometries = generatedModel->GetNumGeometries();
        resolved.names_.resize(numGeometries);
        for (unsigned i = 0; i < numGeometries; ++i)
        {
            const bool hasName = i < materialsAttr.names_.size() && !materialsAttr.names_[i].empty();
            resolved.names_[i] = hasName ? materialsAttr.names_[i] : materialsAttr.names_[0];
        }
        staticModel->SetMaterialsAttr(resolved);
        return true;
    }

    return SetupOperandStaticModel(staticModel, brush->GetModelAttr(), brush->GetMaterialsAttr());
}

bool CsgRoot::BuildOperands(ea::vector<BrushStep>& operands, const Matrix3x4& rootFrame, StaticModel* tempModel,
    ea::vector<SharedPtr<Model>>& sourceModels, ea::vector<ResourceRefList>& sourceMaterials)
{
    operands.clear();
    if (!node_)
        return false;

    // Only the direct children are operands. The child order is the operation order.
    for (Node* child : node_->GetChildren())
    {
        // A child without a parent is in removal.
        if (!child || !child->IsEnabled() || !child->GetParent() || child == removedChild_)
            continue;

        // A child with its own root builds on its own.
        if (child->GetComponent<CsgRoot>())
            continue;

        CsgBrush* brush = child->GetDerivedComponent<CsgBrush>();
        if (!brush || !brush->IsEnabledEffective())
            continue;

        // Listen to the brush even if it did not register yet.
        RegisterBrushInternal(brush, false);
        if (!brush->root_)
            brush->root_ = this;

        if (!SetupOperandStaticModel(tempModel, brush))
            continue;

        BrushStep& step = operands.emplace_back();
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
    }

    return !operands.empty();
}

void CsgRoot::ClearBuildCache()
{
    cachedSolid_ = nullptr;
    cachedBaseSolid_ = nullptr;
    cachedStepSolids_.clear();
    cachedBrushes_.clear();
}

bool CsgRoot::TryCaptureBrushes(const ea::vector<BrushStep>& operands, ea::vector<BrushFingerprint>& output)
{
    for (const BrushStep& operand : operands)
    {
        if (!operand.brush_)
        {
            output.clear();
            return false;
        }
        output.push_back({operand.brush_, operand.model_, operand.transform_, operand.meshId_, operand.op_});
    }
    return true;
}

void CsgRoot::EvaluateOperands(const ea::vector<BrushStep>& operands, const SharedPtr<const CsgSolid>& baseSolid,
    ea::vector<SharedPtr<const CsgSolid>>& stepSolids, float epsilon)
{
    for (unsigned i = stepSolids.size(); i < operands.size(); ++i)
    {
        const BrushStep& operand = operands[i];
        const CsgSolid* currentSolid = i > 0 ? stepSolids[i - 1].Get() : baseSolid.Get();
        SharedPtr<const CsgSolid> operandSolid =
            CsgSolid::FromOperand(ea::vector<CsgPolygon>(operand.polygons_), epsilon);

        // A null result is an empty solid.
        SharedPtr<const CsgSolid> result =
            CsgBooleanOperation(currentSolid, operandSolid.Get(), operand.op_, epsilon);
        stepSolids.push_back(result ? result : SharedPtr<const CsgSolid>(CsgSolid::FromOperand({}, epsilon)));
    }
}

void CsgRoot::AcknowledgeBrushes(const ea::vector<BrushStep>& operands)
{
    for (const BrushStep& operand : operands)
    {
        if (CsgBrush* brush = operand.brush_)
            brush->ClearDirty();
    }
}

} // namespace Urho3D
