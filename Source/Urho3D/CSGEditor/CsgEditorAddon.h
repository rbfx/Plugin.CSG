// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSGEditor/CsgEditorOps.h"
#include "Urho3D/CSGEditor/CsgShapeEditOps.h"

#include <Editor/Core/CommonEditorActionBuilders.h>
#include <Editor/Foundation/SceneViewTab.h>

#include <EASTL/unique_ptr.h>

namespace Urho3D
{

struct CsgBrushPreset;

class Component;
class Node;

/// The editor plugin function. Register it with EditorPluginManager::AddPlugin.
void RbfxCsg_EditorAddon(Context* context, SceneViewTab* sceneViewTab);

/// Scene view addon for fast CSG prototyping.
///
/// The "active root" is the CsgRoot of the active selected node or of its parent.
/// The addon adds brushes and sets the operation of the selected brushes.
/// A new brush from a hotkey or a toolbar button uses the chosen preset: a procedural type or a model.
/// It does nothing and hides its toolbar if the type "CsgRoot" is not registered.
///
/// A Model dropped in the viewport has two CSG cases. Other cases go to the stock handlers.
/// - The active node is a CSG root or its result node: the drop adds a brush with the model.
///   The brush is a Union brush. Shift at drop time makes it a subtract brush.
/// - The active node is a brush: the drop sets the model of all selected brushes.
///
/// If a root or a brush is active, the addon draws the debug shapes of all operands of the root.
/// Each shape has the color of its operation. The selection renderer of the editor draws the selected brushes.
/// The addon draws the other operands, at most 64 in a frame.
/// An overlay in the view corner lists the brushes of the root and the build status.
/// A click on a list item selects the brush. The hotkeys `[` and `]` move the active brush in the list.
/// The hotkey `H` toggles the selected brushes. A disabled brush is not an operand.
///
/// Shape handles: if exactly one node is selected and it has a procedural brush, the addon draws its handles.
/// A click selects a handle. A translate gizmo moves the selected handles. One undo step reverts a drag.
class CsgEditorAddon : public SceneViewAddon
{
    URHO3D_OBJECT(CsgEditorAddon, SceneViewAddon);

public:
    explicit CsgEditorAddon(SceneViewTab* owner);

    /// Implement SceneViewAddon.
    /// @{
    ea::string GetUniqueName() const override { return "CsgEditor"; }
    int GetToolbarPriority() const override { return 50; }
    bool RenderToolbar() override;
    bool RenderTabContextMenu() override;
    void Render(SceneViewPage& page) override;
    void ProcessInput(SceneViewPage& page, bool& mouseConsumed) override;
    void WriteIniSettings(ImGuiTextBuffer& output) override;
    void ReadIniSettings(const char* line) override;
    /// The priority is above 0. Then the addon gets a Model drop before SceneDragAndDropPrefab.
    int GetInputPriority() const override { return 10; }
    bool IsDragDropPayloadSupported(SceneViewPage& page, DragDropPayload* payload) const override;
    void BeginDragDrop(SceneViewPage& page, DragDropPayload* payload) override;
    void UpdateDragDrop(DragDropPayload* payload) override;
    void CompleteDragDrop(DragDropPayload* payload) override;
    void CancelDragDrop() override;
    /// @}

    /// Recenter the pivot of the active shape brush. One undo step reverts the shape and the node position.
    /// It does nothing while a handle drag is active. Return true if the shape changed.
    bool RecenterActiveShape();

    /// Add a brush with the operation. The cursor ray gives the position.
    void AddBrush(int operation, bool useCursorRay);
    /// Set the operation on all selected brush nodes.
    void SetSelectedOperation(int operation);
    /// Move the active brush by the step in the operand list of its root. One undo step reverts the move.
    void MoveActiveBrush(int step);
    /// Toggle the enabled state of all selected brushes. One undo step reverts the change.
    void ToggleSelectedBrushes();
    /// Save the result of the active root as a model file, add a sibling node with the model and disable the root.
    /// One undo step removes the node and enables the root again. The file stays.
    void BakeToAsset();

private:
    enum class DropMode
    {
        None,
        NewBrush,
        ReplaceModel,
        /// A material or a texture goes to the selected brushes.
        AssignMaterial,
        AssignTexture,
        /// The drop shows a hint and changes nothing.
        Hint,
        /// The cursor is over a result but no brush made the face. The drop changes nothing.
        HintNoBrush,
    };

    /// The drop state of the cursor: the decision and the brush under the cursor.
    struct DropEvaluation
    {
        DropDecision decision_;
        /// The brush node that made the face under the cursor, or null.
        Node* hoverBrushNode_{};
    };

    bool IsAvailable() const;
    /// Return the "Draw Debug Geometry" option of the selection renderer. The addon follows it.
    bool IsSelectionDebugGeometryEnabled() const;
    /// Render the toolbar combo of the brush presets.
    void RenderPresetCombo();
    /// Return the chosen preset. A hidden preset gives the first preset that the editor can use.
    const CsgBrushPreset& GetActivePreset() const;
    /// Render the operand list of the root in the corner of the view.
    void RenderOperandOverlay(SceneViewPage& page, Node* rootNode);
    /// Return the drop case for the payload and the selection of the page.
    DropMode GetDropMode(SceneViewPage& page, DragDropPayload* payload) const;
    /// Read the selection and the cursor and call DecideDrop. The pick uses the result of ANY root.
    DropEvaluation EvaluateDrop(SceneViewPage& page, DropPayloadKind kind) const;
    /// Return the face slot for a Ctrl drop on the brush: the slot of the selected face handle. Else nothing.
    ea::optional<unsigned> GetCtrlFaceSlot(Component* brush) const;
    /// Find the target brush components for the cursor. Restore the old preview and set the new one.
    void RefreshDropTargets(SceneViewPage& page);
    /// Set the material on the brush components of the drop with the scope and slot of the drop.
    void ApplyDropMaterial(const ea::string& materialName);
    /// Set the old "Shape Material" and "Materials" values back on the brush components of the drop.
    void RestoreDropMaterials();
    /// Move the preview brush to the cursor hit point.
    void UpdateDropBrushPosition();
    /// Clear the drag and drop state. The function does not change the scene.
    void ResetDrop();
    /// Return the operation index of the enum name, or -1.
    int FindOperation(const ea::string& name) const;
    /// Return the selected brush nodes.
    ea::vector<Node*> GetSelectedBrushes(SceneViewPage& page) const;

    void AddUnionBrush();
    void AddSubtractBrush();
    void SetUnion();
    void SetDifference();
    void SetIntersection();
    void MoveEarlier();
    void MoveLater();

    /// The state of a handle drag.
    enum class HandleDragState
    {
        None,
        Active,
        /// Escape cancelled the drag. The gizmo stays in use until the mouse button is released.
        Cancelled,
    };

    /// Shape handles.
    /// @{
    /// Draw the handles of the active shape brush, pick them and run the gizmo.
    void ProcessShapeHandles(SceneViewPage& page, bool& mouseConsumed);
    /// Set the active shape brush. A new brush clears the handle selection and ends a drag.
    void SetActiveShapeBrush(Component* brush);
    /// Start a drag of the selected handles. Take the attribute snapshot.
    void BeginHandleDrag();
    /// Apply the total drag delta from the drag start to the brush.
    void UpdateHandleDrag(Node* node, bool snap);
    /// End the drag. A commit pushes one undo action. A cancel restores the snapshot.
    void EndHandleDrag(bool commit);
    /// Select the brush that made the clicked face of the result. Run on a left release without a drag.
    void ProcessResultPick(SceneViewPage& page, bool& mouseConsumed, bool viewHovered);
    /// Render the "Shape" section of the operand overlay.
    void RenderStatsSection(Node* rootNode);
    void RenderShapeSection();
    /// Render the rows of the shape material and the face overrides of a shape brush.
    void RenderMaterialRows(Component* brush);
    /// Render the settings of the shape handles.
    void RenderShapeSettings();
    /// Return true if the gizmo of the transform manipulator is in local space.
    bool IsTransformGizmoLocal() const;
    /// @}

    /// The index of the preset model for new brushes. The editor settings keep the preset name.
    unsigned presetIndex_{};
    /// The screen rectangle of the operand overlay in the last frame.
    Rect overlayRect_;
    bool overlayVisible_{};

    DropMode dropMode_{DropMode::None};
    WeakPtr<SceneViewPage> dropPage_;
    /// The new brush case: the preview brush, its preview model and the action builder.
    SharedPtr<Node> dropBrush_;
    WeakPtr<Component> dropPreview_;
    ea::unique_ptr<CreateNodeActionBuilder> dropActionBuilder_;
    /// The replace case: the brush components with their old and new model attributes.
    ea::vector<WeakPtr<Component>> dropComponents_;
    VariantVector dropOldModels_;
    Variant dropNewModel_;
    /// The material case: the old "Materials" values of the components in dropComponents_.
    VariantVector dropOldMaterials_;
    /// The material case: the old "Shape Material" values. Empty for a brush without the attribute.
    VariantVector dropOldShape_;
    /// The material case: the material resource name, or the texture resource name.
    ea::string dropResourceName_;
    /// The material case: the preview has this scope and slot.
    MaterialScope dropScope_{MaterialScope::Shape};
    unsigned dropSlot_{};
    /// The material case: a Ctrl drop had no face handle. The hint shows.
    bool dropNoFace_{};
    /// The material case: the preview material is set on the brushes.
    bool dropPreviewApplied_{};

    /// The settings of the shape handles. A grid step of 0 turns off the Ctrl snap.
    bool showHandles_{true};
    /// Recenter the pivot of the shape at the commit of a handle drag.
    bool autoRecenter_{true};
    float handleGridStep_{0.125f};
    /// The active shape brush and its handles of this frame.
    WeakPtr<Component> handleBrush_;
    ShapeHandleSet handleSet_;
    /// The selected handle ids. The first id is the primary handle of the Ctrl snap.
    ea::vector<unsigned> selectedHandles_;
    ea::optional<unsigned> hoveredHandle_;
    /// A left click was consumed by the handles. The mouse stays consumed until the button is released.
    bool handleClickPending_{};
    /// The gizmo of the selected handles is in use or under the cursor in this frame.
    bool gizmoHot_{};
    /// A left press can start a result pick. The release decides.
    bool resultPickArmed_{};
    /// The face to select after the selection changes to one shape brush. The hit is in world space.
    struct PendingFace
    {
        WeakPtr<Component> brush_;
        Vector3 worldPoint_;
        Vector3 worldNormal_;
    };
    ea::optional<PendingFace> pendingFace_;
    HandleDragState dragState_{HandleDragState::None};
    ShapeAttributeSnapshot dragSnapshot_;
    /// The local position of the brush node at the drag start.
    Vector3 dragStartPosition_;
    ea::vector<ShapeHandle> dragStartHandles_;
    /// The total world delta of the gizmo from the drag start.
    Vector3 dragWorldDelta_;
    /// The local delta of the last move call and of the last accepted move.
    ea::optional<Vector3> dragTriedDelta_;
    ea::optional<Vector3> dragAcceptedDelta_;
};

} // namespace Urho3D
