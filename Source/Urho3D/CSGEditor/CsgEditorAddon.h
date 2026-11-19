// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Editor/Core/CommonEditorActionBuilders.h>
#include <Editor/Foundation/SceneViewTab.h>

#include <EASTL/unique_ptr.h>

namespace Urho3D
{

class Component;
class Node;

/// The editor plugin function. Register it with EditorPluginManager::AddPlugin.
void RbfxCsg_EditorAddon(Context* context, SceneViewTab* sceneViewTab);

/// Scene view addon for fast CSG prototyping.
///
/// The "active root" is the CsgRoot of the active selected node or of its parent.
/// The addon adds brushes and sets the operation of the selected brushes.
/// It does nothing and hides its toolbar if the type "CsgRoot" is not registered.
///
/// A Model dropped in the viewport has two CSG cases. Other cases go to the stock handlers.
/// - The active node is a CSG root or its result node: the drop adds a brush with the model.
///   The brush is a Union brush. Shift at drop time makes it a subtract brush.
/// - The active node is a brush: the drop sets the model of all selected brushes.
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
    /// The priority is above 0. Then the addon gets a Model drop before SceneDragAndDropPrefab.
    int GetInputPriority() const override { return 10; }
    bool IsDragDropPayloadSupported(SceneViewPage& page, DragDropPayload* payload) const override;
    void BeginDragDrop(SceneViewPage& page, DragDropPayload* payload) override;
    void UpdateDragDrop(DragDropPayload* payload) override;
    void CompleteDragDrop(DragDropPayload* payload) override;
    void CancelDragDrop() override;
    /// @}

    /// Add a brush with the operation. The cursor ray gives the position.
    void AddBrush(int operation, bool useCursorRay);
    /// Set the operation on all selected brush nodes.
    void SetSelectedOperation(int operation);

private:
    enum class DropMode
    {
        None,
        NewBrush,
        ReplaceModel,
    };

    bool IsAvailable() const;
    /// Return the drop case for the payload and the selection of the page.
    DropMode GetDropMode(SceneViewPage& page, DragDropPayload* payload) const;
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
};

} // namespace Urho3D
