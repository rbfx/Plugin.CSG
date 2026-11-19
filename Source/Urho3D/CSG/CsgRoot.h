// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

/// \file

#pragma once

#include <functional>
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

/// Phase times and result sizes of one rebuild. See `CsgRoot::GetLastBuildStats`.
///
/// The times are in milliseconds. The size and shape values describe the result solid in the root frame.
/// A root that has not applied a build has the default value: all numbers are zero and `closed_` is false.
struct CsgBuildStats
{
    /// Time of the boolean operations on the worker thread.
    float booleanMs_{};
    /// Time of the triangulation of the result solid on the worker thread.
    float triangulateMs_{};
    /// Time to build the output model on the main thread.
    float buildModelMs_{};
    /// Number of faces of the result solid.
    unsigned faceCount_{};
    /// Number of components of the result solid, after the removal of the small components.
    unsigned componentCount_{};
    /// Number of components that the build removed because of `CsgRoot::GetMinComponentVolumeFraction`.
    unsigned droppedComponents_{};
    /// Number of triangles of the triangulated result.
    unsigned triangleCount_{};
    /// True if the result solid is closed. An empty solid is closed.
    bool closed_{};
    /// Signed volume of the result solid. See `CsgSolid::Volume`.
    double volume_{};
    /// Number of open edges of the triangulated result. See `CsgCheckModel`.
    unsigned openEdges_{};
    /// Number of duplicate triangles of the triangulated result.
    unsigned duplicates_{};
    /// Number of degenerate triangles of the triangulated result.
    unsigned degenerates_{};
    /// True if the result is a valid closed model: the solid is closed, the triangulated result has no open edge,
    /// no duplicate triangle and no degenerate triangle, and its volume is positive. An empty result is valid.
    /// The root does not cache the solid of an invalid result. The output model still shows the result.
    bool valid_{};
};

/// Root (builder) component for component-based CSG workflow.
///
/// High-level behavior:
/// - Produces a combined mesh from the base model and the operands of the root.
/// - An operand is a direct child node with an enabled `CsgBrush`. The child order is the operation order.
/// - A brush deeper in the tree is not an operand and has no effect.
/// - A direct child with its own `CsgRoot` is not an operand. It builds its own result.
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
/// - The build uses exact math on solids. An operand coordinate at or beyond 8192 units
///   gives an empty result with `IsOutOfRange()` true. The root does not log a message.
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
    /// Get the minimum volume of a shell as a fraction of the volume of the largest shell.
    float GetMinComponentVolumeFraction() const { return minComponentVolumeFraction_; }
    /// Set the minimum volume of a shell as a fraction of the volume of the largest shell. The root clamps it to 0..1.
    /// The build drops a shell with a volume below the fraction of the largest shell volume.
    /// A cavity inside a dropped shell goes with it. The value 1 keeps only the largest shell.
    /// The value 0 keeps all shells. A changed fraction does not clear the cached solid.
    void SetMinComponentVolumeFraction(float value);
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
    /// Get the statistics of the build whose result the root applied last.
    /// A result that waits in the queue has no effect on the value until ApplyPendingResult applies it.
    const CsgBuildStats& GetLastBuildStats() const { return lastBuildStats_; }
    /// Return the component of each geometry of the output model, in geometry order.
    /// The geometries with one value form one connected shape.
    /// The values belong to the build of GetLastBuildStats.
    /// The span is empty if the output is not the result of a build: no output model, the base model, or a baked model.
    ea::span<const unsigned> GetLastGeometryComponents() const
    {
        return {lastGeometryComponents_.data(), lastGeometryComponents_.size()};
    }
    /// Return true if the last build that the root took was a failed build.
    /// A build fails if the boolean operation throws. The output model does not change then.
    /// A later build that the root applies clears the flag.
    /// While auto-apply is off, a failed build is a queued result. ApplyPendingResult sets the flag.
    bool GetLastBuildFailed() const { return lastBuildFailed_; }
    /// Test only. Set an action that the next build job runs on its worker thread before the boolean.
    /// The job takes the action once. Pass an empty function to remove it.
    /// If the action throws, the build fails. This function is thread safe.
    static void SetTestBuildHook(std::function<void()> hook);
    /// Fold all applied brushes into the base operand and remove the brush nodes.
    /// Later brushes apply to the baked result, so each one costs one boolean operation.
    /// Baking is permanent and exists only at run time. The root does not serialize the baked geometry.
    /// A change of the base model, or of the epsilon, discards the baked result.
    /// The function fails and returns false in these cases:
    /// - The root has no node.
    /// - A rebuild is in flight, or the root needs a rebuild.
    /// - A brush is dirty.
    /// - A finished result waits to be applied.
    /// - The root has no cached solid, no cached brushes or no cached sources.
    /// Return true if the root baked the brushes.
    bool Bake();

protected:
    void ApplyAttributes() override;
    void RequestRebuild();
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
        CsgBuildStats stats_;
        /// True if the build failed. The model is empty and the output must not change.
        bool failed_{};
        /// The component of each geometry of the model.
        ea::vector<unsigned> components_;
    };

    /// Result of a bake. It replaces the base operand until the base model or the epsilon changes.
    struct BakedState
    {
        SharedPtr<const CsgSolid> solid_;
        Vector3 metricScale_{Vector3::ONE};
        float epsilon_{};
        SharedPtr<Model> baseModel_;
        ea::vector<SharedPtr<Model>> sourceModels_;
        ea::vector<ResourceRefList> sourceMaterials_;
        SharedPtr<Model> outputModel_;
        ResourceRefList outputMaterials_;
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
    /// Collect the operands from the direct children of the node. Return false if there are no operands.
    bool BuildOperands(ea::vector<BrushStep>& operands, const Matrix3x4& rootFrame, StaticModel* tempModel,
        ea::vector<SharedPtr<Model>>& sourceModels, ea::vector<ResourceRefList>& sourceMaterials);
    void HandleNodeAdded(StringHash eventType, VariantMap& eventData);
    void HandleNodeRemoved(StringHash eventType, VariantMap& eventData);

    static void ExecuteBuildJob(unsigned threadIndex, WorkQueue* queue, const std::shared_ptr<BuildJob>& job);
    static void ApplyBuildJobResultOnMainThread(const std::shared_ptr<BuildJob>& job, SharedPtr<CsgTriangulatedModel> triangulated);
    /// Apply the operands in order to the current solid. Skip the operands before `startIndex`.
    static void EvaluateOperands(const ea::vector<BrushStep>& operands, SharedPtr<const CsgSolid>& currentSolid,
        float epsilon, unsigned startIndex);
    /// Copy the fingerprints of the operands. Return false if a brush is dead.
    static bool TryCaptureBrushes(const ea::vector<BrushStep>& operands, ea::vector<BrushFingerprint>& output);
    static void AcknowledgeBrushes(const ea::vector<BrushStep>& operands);

    float deferredDelaySec_{0.1f};
    float epsilon_{1e-4f};
    float minComponentVolumeFraction_{0.0f};
    bool rebuildDirty_{true};
    unsigned lastChangeTimeMs_{};
    unsigned deferredScheduleToken_{};
    bool deferredRebuildScheduled_{};
    unsigned rebuildRequestId_{1};
    unsigned rebuildInFlightId_{};
    bool rebuildInFlight_{};
    /// True while Bake removes the brush nodes. Then a rebuild request only sets the dirty flag.
    bool baking_{};
    bool autoApplyResults_{true};
    ea::vector<PendingBuildResult> pendingResults_;
    CsgBuildStats lastBuildStats_;
    ea::vector<unsigned> lastGeometryComponents_;
    bool lastBuildFailed_{};

    /// Solid of the last applied build. It is the base for an append of brushes.
    SharedPtr<const CsgSolid> cachedSolid_;
    SharedPtr<Model> cachedBaseModel_;
    float cachedEpsilon_{};
    Vector3 cachedMetricScale_{Vector3::ONE};
    ea::vector<SharedPtr<Model>> cachedSourceModels_;
    ea::vector<ResourceRefList> cachedSourceMaterials_;
    ea::vector<BrushFingerprint> cachedBrushes_;
    /// Baked result. It is null if the root has no bake.
    ea::unique_ptr<BakedState> baked_;

    WeakPtr<StaticModel> output_;
    WeakPtr<Node> tempNode_;
    WeakPtr<StaticModel> tempB_;
    WeakPtr<StaticModel> tempBase_;
    ea::vector<WeakPtr<Node>> listenedNodes_;
    ea::vector<WeakPtr<CsgBrush>> brushes_;

    unsigned ignoreChildDirtyFrame_{};
};

} // namespace Urho3D
