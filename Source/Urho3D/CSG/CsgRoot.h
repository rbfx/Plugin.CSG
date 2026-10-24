// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

/// \file

#pragma once

#include <memory>

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include "Urho3D/CSG/CsgComponent.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Core/Variant.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>

namespace Urho3D
{

class CsgBrush;
class CsgSolid;
class Material;
class Model;
class StaticModel;
class WorkQueue;

/// Root (builder) component for component-based CSG workflow.
///
/// High-level behavior:
/// - Produces a combined mesh by evaluating CSG operations of all enabled `CsgBrush` components
///   under this node's subtree.
/// - Ignores nested `CsgRoot` subtrees: traversal stops at the first nested root encountered.
///
/// Rebuild policy and threading:
/// - Rebuilds are async: the heavy work runs on `WorkQueue` worker thread(s) and the resulting
///   `Model` is applied on the main thread.
/// - Rebuild requests are debounced using `deferredDelaySec_`. Any relevant change schedules a rebuild
///   for after the quiet period; setting delay to 0 triggers immediate rebuild.
/// - Results are de-staled: if multiple rebuilds are requested while a build is in-flight, older
///   results are discarded and a fresh rebuild is scheduled.
///
/// Root frame and coordinates:
/// - The build runs in the root frame. The root frame carries the world scale of the root, but not
///   its rotation or position. Brushes keep their relative transform in this frame.
/// - An axis with a scale that is not finite or not greater than 1e-6 uses the scale 1.
/// - The result returns to node space. The output model has the same frame as the node of the root.
/// - The build uses exact math on solids. An operand coordinate at or beyond 65536 units
///   makes the root log a warning.
///
/// Brush registration:
/// - Brushes register/unregister themselves with the nearest root (see `CsgBrush`).
/// - The root listens to transform/dirty notifications from registered brush nodes.
///
/// Output ownership:
/// - The computed result is stored in a persistent `StaticModel` component on the same node.
/// - Only the `Model` and material slots are overwritten; unrelated `StaticModel` settings are preserved.
class PLUGIN_RBFXCSG_API CsgRoot : public CsgComponent
{
    URHO3D_OBJECT(CsgRoot, CsgComponent);

public:
    explicit CsgRoot(Context* context);

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);
    /// Rebuild immediately, ignoring scheduling policy.
    void Rebuild();
    /// Get debounce delay for rebuild scheduling.
    float GetDeferredDelaySec() const { return deferredDelaySec_; }
    /// Set debounce delay for rebuild scheduling. Values below 0 are clamped to 0.
    void SetDeferredDelaySec(float value);
    /// Get epsilon used by polygon operations.
    float GetEpsilon() const { return epsilon_; }
    /// Set epsilon used by polygon operations. Values below 0 are clamped to 0.
    void SetEpsilon(float value);
    /// Get whether a finished rebuild goes to the output model at once.
    bool GetAutoApplyResults() const { return autoApplyResults_; }
    /// Set whether a finished rebuild goes to the output model at once.
    /// If false, the root queues each finished result. Call ApplyPendingResult to apply the oldest one.
    void SetAutoApplyResults(bool value) { autoApplyResults_ = value; }
    /// Get the current rebuild revision. The revision changes when an input changes.
    unsigned GetRebuildRevision() const { return rebuildRequestId_; }
    /// Return true if a finished result waits to be applied.
    bool HasPendingResult() const { return !pendingResults_.empty(); }
    /// Get the revision of the oldest finished result. Return 0 if no result is pending.
    unsigned GetPendingResultRevision() const { return pendingResults_.empty() ? 0u : pendingResults_.front().revision_; }
    /// Apply the oldest finished result that the root queued while auto-apply was off.
    /// Return true if the root applied a result.
    bool ApplyPendingResult();

protected:
    void ApplyAttributes() override;
    void RequestRebuild();
    void CollectBrushesAndNestedRoots(Node* rootNode,
        ea::vector<CsgBrush*>& outBrushes,
        ea::vector<Node*>& outNestedRootNodes) const;
    void ApplyResultToOutput(Model* model, const ResourceRefList& materials);
    void RegisterBrush(CsgBrush* brush);
    void UnregisterBrush(CsgBrush* brush);
    void OnSetEnabled() override;
    void OnNodeSet(Node* previousNode, Node* currentNode) override;
    void OnSceneSet(Scene* previousScene, Scene* scene) override;
    void OnMarkedDirty(Node* node) override;
    const ResourceRef& GetDebugModelAttr() const override { return GetModelAttr(); }
    void OnModelChanged() override;
    void OnMaterialsChanged() override;

private:
    friend class CsgBrush;

    struct BrushStep;
    struct BuildJob;

    /// Data of a brush that decides if a cached solid is valid for this brush.
    struct BrushFingerprint
    {
        WeakPtr<CsgBrush> brush_;
        SharedPtr<Model> model_;
        Matrix3x4 transform_{Matrix3x4::IDENTITY};
        unsigned meshId_{};
        CsgOperation op_{};

        bool operator==(const BrushFingerprint& rhs) const
        {
            return brush_ == rhs.brush_ && model_ == rhs.model_ && transform_ == rhs.transform_ && meshId_ == rhs.meshId_
                && op_ == rhs.op_;
        }
        bool operator!=(const BrushFingerprint& rhs) const { return !(*this == rhs); }
    };

    /// Finished result that waits for ApplyPendingResult.
    struct PendingBuildResult
    {
        SharedPtr<Model> model_;
        ResourceRefList materials_;
        unsigned revision_{};
    };

    /// Return persistent output StaticModel on the same node.
    StaticModel* GetOrCreateOutputStaticModel();
    void UpdateSubscriptions();
    void RegisterBrushInternal(CsgBrush* brush, bool requestRebuild);
    void UnregisterBrushInternal(CsgBrush* brush);
    void PruneBrushes();
    void ScheduleDeferredRebuild();
    void DeferredRebuildTick(unsigned scheduleToken, WorkQueue* queue);
    StaticModel* GetOrCreateTempStaticModel(const ea::string& name, WeakPtr<StaticModel>& cache);
    bool SetupOperandStaticModel(StaticModel* staticModel, const CsgBrush* brush) const;
    bool SetupOperandStaticModel(StaticModel* staticModel, const ResourceRef& modelAttr, const ResourceRefList& materialsAttr) const;
    bool BuildOperationTree(BrushStep& rootStep, Node* node, const Matrix3x4& rootFrame, StaticModel* tempModel,
        ea::vector<SharedPtr<Model>>& sourceModels, ea::vector<ResourceRefList>& sourceMaterials);

    static void ExecuteBuildJob(unsigned threadIndex, WorkQueue* queue, const std::shared_ptr<BuildJob>& job);
    static void ApplyBuildJobResultOnMainThread(const std::shared_ptr<BuildJob>& job, SharedPtr<CsgTriangulatedModel> triangulated);
    static void EvaluateOperationTree(const BrushStep& step, SharedPtr<const CsgSolid>& currentSolid, float epsilon,
        bool& outOfRange, unsigned startChild = 0);
    /// Copy the fingerprints of the brushes. Return false if the tree is not flat or a brush is dead.
    static bool TryCaptureFlatBrushes(const BrushStep& rootStep, ea::vector<BrushFingerprint>& output);
    static void AcknowledgeBrushes(const BrushStep& step);

    float deferredDelaySec_{0.1f};
    float epsilon_{1e-4f};
    bool rebuildDirty_{true};
    unsigned lastChangeTimeMs_{};
    unsigned deferredScheduleToken_{};
    bool deferredRebuildScheduled_{};
    unsigned rebuildRequestId_{1};
    unsigned rebuildInFlightId_{};
    bool rebuildInFlight_{};
    bool autoApplyResults_{true};
    ea::vector<PendingBuildResult> pendingResults_;

    /// Solid of the last applied build. It is the base for an append of brushes.
    SharedPtr<const CsgSolid> cachedSolid_;
    SharedPtr<Model> cachedBaseModel_;
    float cachedEpsilon_{};
    Vector3 cachedMetricScale_{Vector3::ONE};
    ea::vector<SharedPtr<Model>> cachedSourceModels_;
    ea::vector<ResourceRefList> cachedSourceMaterials_;
    ea::vector<BrushFingerprint> cachedBrushes_;

    WeakPtr<StaticModel> output_;
    WeakPtr<Node> tempNode_;
    WeakPtr<StaticModel> tempB_;
    WeakPtr<StaticModel> tempBase_;
    ea::vector<WeakPtr<Node>> listenedNodes_;
    ea::vector<WeakPtr<CsgBrush>> brushes_;

    unsigned ignoreChildDirtyFrame_{};
};

} // namespace Urho3D
