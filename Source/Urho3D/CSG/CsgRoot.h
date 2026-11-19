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
class Serializer;
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
    /// Number of operands that the build evaluated. The build reuses the cached fold states of the other operands.
    unsigned foldSteps_{};
    /// Number of operands of the build. `operandCount_ - foldSteps_` operands come from the reused prefix.
    unsigned operandCount_{};
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

/// Triangles of an applied result with the source of each triangle. The pick uses it.
struct CsgPickTable
{
    /// Three corners for each triangle, in the space of the result node.
    ea::vector<Vector3> positions_;
    /// Source of each triangle: the node ID of the brush that made it. 0 is the base model or an unknown source.
    ea::vector<unsigned> sources_;
};

/// Result of `CsgRoot::Pick`.
struct CsgPickHit
{
    bool hit_{};
    /// Node ID of the brush that made the hit face. 0 is the base model or an unknown source.
    unsigned source_{};
    /// Hit position in world space.
    Vector3 position_{Vector3::ZERO};
    /// Unit world normal of the front face of the hit triangle.
    Vector3 normal_{Vector3::ZERO};
    /// Distance from the ray origin to the hit position in world units.
    float distance_{};
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
/// - Rebuild requests are debounced using `deferredDelaySec_`. The wait starts at the first open change.
///   A later change does not reset it. A rebuild with the newest state runs when the wait ends.
///   A change during a build starts a new wait. Setting delay to 0 triggers immediate rebuild.
/// - One build runs at a time. A change during a build leaves the root dirty. If the build result is
///   older than the change, the root still shows it and starts a new wait for the newer state.
///   The root drops a stale result of a failed build, of a job with a model that reloaded, of a root that is not active, or when auto-apply is off.
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
/// - The result goes to a `StaticModel` on a temporary child node with the name "CsgResult".
/// - "CsgResult" is a reserved child name. Do not give it to other children of the root.
/// - The root reuses a child with that name only if it has no children and no components other than StaticModel.
///   The root removes other such children. It does not change or remove a child with that name that has more.
/// - The scene does not save or clone the result node. The root makes it again after a load or a clone.
/// - The result node has the identity local transform. It has no `CsgBrush`, so it is not an operand.
/// - The result node and its model follow the effective enabled state of the root.
/// - The root removes the result node when the root leaves its node.
/// - Only the `Model` and material slots are overwritten; unrelated `StaticModel` settings are preserved.
///
/// Collision:
/// - If "Generate Collision" is on, the result node gets a temporary `CollisionShape` with a triangle mesh.
///   The mesh is the applied result model. Each applied result updates the same shape.
/// - If "Collision Static Body" is also on, the result node gets a temporary static `RigidBody` (mass 0).
/// - The root removes the shape and the body when the attribute turns off or when the result is empty.
/// - If the engine has no physics, or the context did not register it, the root makes no components.
///   The root logs one warning.
/// - A `CollisionShape` adds a `PhysicsWorld` to the scene if the scene has none. The scene saves that world.
class PLUGIN_RBFXCSG_API CsgRoot : public CsgComponent
{
    URHO3D_OBJECT(CsgRoot, CsgComponent);

public:
    explicit CsgRoot(Context* context);

    /// Save the applied result model with `Model::Save`. The editor chooses the destination.
    /// Return false if the root has no result model or the save fails. The model keeps its materials separate:
    /// the caller reads them from `GetResultModel()->GetMaterialsAttr()`.
    bool SaveResultModel(Serializer& dest) const;

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
    /// Get whether the result node gets a collision shape from the result model.
    bool GetGenerateCollision() const { return generateCollision_; }
    /// Set whether the result node gets a collision shape from the result model.
    void SetGenerateCollision(bool value);
    /// Get whether the result node gets a static rigid body together with the collision shape.
    bool GetCollisionStaticBody() const { return collisionStaticBody_; }
    /// Set whether the result node gets a static rigid body together with the collision shape.
    void SetCollisionStaticBody(bool value);
    /// Return the collision shape on the result node. Return null if the root has no collision shape now.
    Component* GetResultCollisionShape() const;
    /// Return the rigid body that the root made on the result node. Return null if the root made no body.
    Component* GetResultRigidBody() const;
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
    /// Return the temporary child node that holds the result. Return null if the root did not make it yet.
    Node* GetResultNode() const;
    /// Return the StaticModel that shows the result. Return null if the root did not make it yet.
    StaticModel* GetResultModel() const;
    /// Return the build status text: "Building...", "OK: <ms> ms, <N> triangles", "Failed", "Invalid" or "Not built".
    /// The time is the sum of the boolean, the triangulation and the model build of the last applied build.
    ea::string GetBuildStatus() const;
    /// Create a child node with the name "Brush" and a `CsgBrush`.
    void AddBrush();
    /// Cast a world ray into the applied result. See `E_CSGROOTPICK` in CsgRootProtocol.h.
    /// The direction can have any length. The ray hits only front faces. The nearest hit wins.
    /// No hit if no build result is applied, or if the direction is zero or not finite.
    CsgPickHit Pick(const Vector3& origin, const Vector3& direction, float maxDistance = 1e9f) const;
    /// Return the pick table of the applied result. It is empty if no build result is applied.
    const CsgPickTable& GetPickTable() const { return pickTable_; }

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
        CsgUvMode uvMode_{};
        float uvScale_{1.0f};
        Vector2 uvOffset_{Vector2::ZERO};
        /// Node ID of the brush. The cached solids carry it as the polygon source, so a new ID invalidates them.
        unsigned sourceId_{};

        bool operator==(const BrushFingerprint& rhs) const
        {
            return brush_ == rhs.brush_ && model_ == rhs.model_ && transform_ == rhs.transform_ && meshId_ == rhs.meshId_
                && op_ == rhs.op_ && uvMode_ == rhs.uvMode_ && uvScale_ == rhs.uvScale_ && uvOffset_ == rhs.uvOffset_
                && sourceId_ == rhs.sourceId_;
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
        /// The pick table of the model.
        CsgPickTable pickTable_;
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

    /// Return the StaticModel on the result node. Make the node and the model if necessary.
    StaticModel* GetOrCreateOutputStaticModel();
    /// Set the enabled state of the result node and of its model to the effective state of the root.
    void UpdateResultEnabled();
    /// Remove the result node from the node of the root.
    void RemoveResultNode();
    /// Make, update or remove the collision components of the result node. They follow the applied model.
    void UpdateResultCollision();
    void UpdateSubscriptions();
    void RegisterBrushInternal(CsgBrush* brush, bool requestRebuild);
    void UnregisterBrushInternal(CsgBrush* brush);
    void PruneBrushes();
    void MarkRebuildRequested();
    void ScheduleDeferredRebuild();
    /// Post one task that compares the brush transforms with the last values that the root saw.
    void ScheduleTransformCheck();
    /// Request a build if the local transform of a brush changed. Clear the dirty flags of the nodes.
    void CheckOperandTransforms();
    void DeferredRebuildTick(unsigned scheduleToken, WorkQueue* queue);
    StaticModel* GetOrCreateTempStaticModel(const ea::string& name, WeakPtr<StaticModel>& cache);
    bool SetupOperandStaticModel(StaticModel* staticModel, const CsgBrush* brush) const;
    bool SetupOperandStaticModel(StaticModel* staticModel, const ResourceRef& modelAttr, const ResourceRefList& materialsAttr) const;
    /// Collect the operands from the direct children of the node. Return false if there are no operands.
    bool BuildOperands(ea::vector<BrushStep>& operands, const Matrix3x4& rootFrame, StaticModel* tempModel,
        ea::vector<SharedPtr<Model>>& sourceModels, ea::vector<ResourceRefList>& sourceMaterials);
    /// Listen for a reload of each model of the cache and of the build job. Stop to listen for other models.
    void UpdateWatchedModels(const BuildJob* job);
    void HandleModelReloaded(StringHash eventType, VariantMap& eventData);
    void HandleRequestRebuild(StringHash eventType, VariantMap& eventData);
    void HandleNodeAdded(StringHash eventType, VariantMap& eventData);
    void HandleNodeRemoved(StringHash eventType, VariantMap& eventData);
    void HandlePick(StringHash eventType, VariantMap& eventData);
    void HandleQueryStats(StringHash eventType, VariantMap& eventData);
    /// Set the pick table of the applied result. A null model clears it.
    void SetPickTable(Model* model, CsgPickTable table);

    static void ExecuteBuildJob(unsigned threadIndex, WorkQueue* queue, const std::shared_ptr<BuildJob>& job);
    static void ApplyBuildJobResultOnMainThread(const std::shared_ptr<BuildJob>& job, SharedPtr<CsgTriangulatedModel> triangulated);
    /// Fold the operands in order onto the base solid. Append the fold state after each operand.
    /// The operands that already have a state in `stepSolids` are skipped.
    static void EvaluateOperands(const ea::vector<BrushStep>& operands, const SharedPtr<const CsgSolid>& baseSolid,
        ea::vector<SharedPtr<const CsgSolid>>& stepSolids, float epsilon);
    /// Discard the cached solids and the cached brushes.
    void ClearBuildCache();
    /// Copy the fingerprints of the operands. Return false if a brush is dead.
    static bool TryCaptureBrushes(const ea::vector<BrushStep>& operands, ea::vector<BrushFingerprint>& output);
    static void AcknowledgeBrushes(const ea::vector<BrushStep>& operands);

    float deferredDelaySec_{0.1f};
    float epsilon_{1e-4f};
    float minComponentVolumeFraction_{0.0f};
    bool rebuildDirty_{true};
    unsigned waitStartTimeMs_{};
    unsigned deferredScheduleToken_{};
    bool deferredRebuildScheduled_{};
    unsigned rebuildRequestId_{1};
    unsigned rebuildInFlightId_{};
    bool rebuildInFlight_{};
    /// A model reloaded while a job ran. The stale result of that job is dropped.
    bool modelReloadedDuringJob_{};
    /// True while Bake removes the brush nodes. Then a rebuild request only sets the dirty flag.
    bool baking_{};
    bool autoApplyResults_{true};
    ea::vector<PendingBuildResult> pendingResults_;
    CsgBuildStats lastBuildStats_;
    ea::vector<unsigned> lastGeometryComponents_;
    /// Pick table of the applied result. It is valid only while the output shows `pickModel_`.
    CsgPickTable pickTable_;
    WeakPtr<Model> pickModel_;
    bool lastBuildFailed_{};
    /// True after the root applied the result of a build that did not fail.
    bool hasAppliedBuild_{};

    /// Final solid of the last applied build, after the shell filter. Bake uses it.
    SharedPtr<const CsgSolid> cachedSolid_;
    /// Base solid of the last applied build.
    SharedPtr<const CsgSolid> cachedBaseSolid_;
    /// Fold state after each operand of the last applied build. Index i is the base with operands 0 to i.
    ea::vector<SharedPtr<const CsgSolid>> cachedStepSolids_;
    float cachedMinComponentVolumeFraction_{};
    SharedPtr<Model> cachedBaseModel_;
    float cachedEpsilon_{};
    Vector3 cachedMetricScale_{Vector3::ONE};
    ea::vector<SharedPtr<Model>> cachedSourceModels_;
    ea::vector<ResourceRefList> cachedSourceMaterials_;
    ea::vector<BrushFingerprint> cachedBrushes_;
    /// Baked result. It is null if the root has no bake.
    ea::unique_ptr<BakedState> baked_;

    /// The root holds weak pointers, not a name lookup. A weak pointer becomes null when the node
    /// or the scene destroys the result node, so no pointer dangles. The lookup by name runs only
    /// once, when the root makes the node, to reuse a node that already exists.
    WeakPtr<Node> resultNode_;
    WeakPtr<StaticModel> output_;
    /// Collision shape on the result node. The type is Component, so the header does not need the physics headers.
    WeakPtr<Component> collisionShape_;
    /// Rigid body that the root made on the result node.
    WeakPtr<Component> collisionBody_;
    bool generateCollision_{};
    bool collisionStaticBody_{true};
    WeakPtr<Node> tempNode_;
    WeakPtr<StaticModel> tempB_;
    WeakPtr<StaticModel> tempBase_;
    ea::vector<WeakPtr<Node>> listenedNodes_;
    ea::vector<WeakPtr<CsgBrush>> brushes_;
    /// Models that the root listens to for a reload.
    ea::vector<WeakPtr<Model>> watchedModels_;
    /// Model that sends the reload event that the root handles now.
    Model* reloadingModel_{};
    /// Child node that leaves the root now. The scene sends the removal event before the parent changes.
    Node* removedChild_{};

    /// A transform check task is in the queue.
    bool transformCheckScheduled_{};
};

} // namespace Urho3D
