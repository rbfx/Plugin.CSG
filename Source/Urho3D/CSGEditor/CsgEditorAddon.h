// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Editor/Foundation/SceneViewTab.h>

namespace Urho3D
{

class Node;

/// The editor plugin function. Register it with EditorPluginManager::AddPlugin.
void RbfxCsg_EditorAddon(Context* context, SceneViewTab* sceneViewTab);

/// Scene view addon for fast CSG prototyping.
///
/// The "active root" is the CsgRoot of the active selected node or of its parent.
/// The addon adds brushes and sets the operation of the selected brushes.
/// It does nothing and hides its toolbar if the type "CsgRoot" is not registered.
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
    /// @}

    /// Add a brush with the operation. The cursor ray gives the position.
    void AddBrush(int operation, bool useCursorRay);
    /// Set the operation on all selected brush nodes.
    void SetSelectedOperation(int operation);

private:
    bool IsAvailable() const;
    /// Return the operation index of the enum name, or -1.
    int FindOperation(const ea::string& name) const;
    /// Return the selected brush nodes.
    ea::vector<Node*> GetSelectedBrushes(SceneViewPage& page) const;

    void AddUnionBrush();
    void AddSubtractBrush();
    void SetUnion();
    void SetDifference();
    void SetIntersection();
};

} // namespace Urho3D
