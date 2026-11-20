// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSGEditor/CsgShapeEditOps.h"

#include <Urho3D/Container/Str.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/optional.h>
#include <EASTL/span.h>
#include <EASTL/vector.h>

namespace Urho3D
{

class Component;
class Context;
class Node;
class StaticModel;

// Scene functions for the CSG editor addon.
//
// The functions do not link the CSG library. They find the types "CsgRoot" and "CsgBrush" by name.
// They read and write the attributes by name. If "CsgRoot" is not registered, a function returns null or false.

/// Return the CsgBrush of the node, or a component of a type that derives from CsgBrush.
Component* FindBrushComponent(Node* node);
/// Return true if the component has the attribute "Model". A brush with a generated model has none.
bool HasModelAttribute(Component* component);
/// Return true if the node has a brush with the attribute "Model".
bool IsModelBrushNode(Node* node);
/// Return the node that has a CsgRoot: the selected node or its parent. A temporary node maps to its parent.
Node* FindActiveRoot(Node* selected);
/// Return true if the node has a CsgBrush or a type that derives from it.
bool IsBrushNode(Node* node);
/// Return true if the node has a CsgRoot.
bool IsRootNode(Node* node);

/// Create a child node "CSG" with a CsgRoot at the world position.
Node* CreateRootNode(Node* parent, const Vector3& worldPosition);
/// Create a child node "Brush" at the end of the operand list. The model name may be empty.
/// The local scale is the inverse of the root world scale. The brush world scale is about 1.
/// A disabled brush is not an operand. The root does not rebuild when it gets the node.
Node* AddBrushNode(
    Node* rootNode, const ea::string& modelName, int operation, const Vector3& worldPosition, bool enabled = true);

/// Create a child node "Brush" with a brush component of the type at the end of the operand list.
/// The type must derive from CsgBrush. A procedural type needs no model. The transform rules are those of AddBrushNode.
/// Return null with no error log if the type is not registered or is not a brush type.
Node* AddBrushNodeOfType(
    Node* rootNode, const ea::string& typeName, int operation, const Vector3& worldPosition, bool enabled = true);
/// Return true if the type is registered and derives from CsgBrush.
bool IsBrushTypeRegistered(Context* context, const ea::string& typeName);

/// A preset for new brushes. It has a model path (a CsgBrush with the model) or a brush type name.
struct CsgBrushPreset
{
    /// The name in the editor and in the editor settings.
    const char* name_;
    /// The model of a model preset, or null.
    const char* model_;
    /// The brush type of a procedural preset, or null.
    const char* typeName_;
};

/// Return the brush presets. The first presets are the procedural box, sphere, cylinder, ring, wedge, stairs
/// and arch.
ea::span<const CsgBrushPreset> GetBrushPresets();
/// Return true if the editor can use the preset. A procedural preset needs its registered type.
bool IsBrushPresetAvailable(Context* context, const CsgBrushPreset& preset);
/// Add a brush node from the preset. See AddBrushNode and AddBrushNodeOfType.
Node* AddPresetBrushNode(
    Node* rootNode, const CsgBrushPreset& preset, int operation, const Vector3& worldPosition, bool enabled = true);

/// Return the operation of the brush, or -1 if the node has no brush.
int GetOperation(Node* brush);
bool SetOperation(Node* brush, int operation);
/// Return the number of operations. The names come from the attribute "Operation" of CsgBrush.
int OperationCount(Context* context);
/// Return the name of the operation, or an empty string if the index is not valid.
ea::string OperationName(Context* context, int operation);
/// Add the step to the operation and wrap around.
int CycleOperation(Context* context, int current, int step);

/// Set the model of the brush. Return false if the brush has no attribute "Model".
bool SetBrushModel(Node* brush, const ea::string& modelName);

/// Set the material of one slot of the brush through the attribute "Materials". The other slots stay.
/// Work with a model brush and a procedural brush. An empty list grows to one slot for slot 0.
/// Return false with no log if the node has no brush or the slot is not valid.
bool SetBrushMaterial(Node* brush, const ea::string& materialName, unsigned slot);
/// Set the material of all slots of the brush. Return false with no log if the node has no brush.
bool SetAllBrushMaterialSlots(Node* brush, const ea::string& materialName);

/// Return true if the resource name has the extension of a texture that the engine loads as Texture2D.
bool IsTextureResourceName(const ea::string& resourceName);
/// Return the name of the material for the texture: the full texture name plus ".material".
/// Example: "Textures/Brick.png" gives "Textures/Brick.png.material". Two textures with the same base name stay apart.
ea::string GetTextureMaterialName(const ea::string& textureResourceName);
/// Make the material file for the texture next to the texture file. Return the material resource name.
/// An existing material file is reused and never changed. The material uses the technique "Techniques/LitOpaque.xml",
/// the shader define PBR and the texture as the albedo (diffuse) texture.
/// Log one error and return an empty string if the texture is not in a mounted directory or the file cannot be written.
ea::string EnsureTextureMaterial(Context* context, const ea::string& textureResourceName);

/// The kind of a drag and drop payload for the scene view.
enum class DropPayloadKind
{
    Other,
    Model,
    Material,
    Texture,
};

/// The selection of the scene view when a drop starts.
struct DropSelection
{
    /// The active node is a brush with the attribute "Model".
    bool activeIsModelBrush_{};
    /// The active node is a brush.
    bool activeIsBrush_{};
    /// The active node is a root, or the result node of a root.
    bool activeIsRoot_{};
    /// At least one selected node is a brush.
    bool anySelectedBrush_{};
    /// The nearest geometry under the cursor is the result node of ANY root.
    bool hoveringResult_{};
    /// The pick found a brush of that root under the cursor. False for source 0 or a missing node.
    bool hoverBrush_{};
    /// The brush under the cursor is selected.
    bool hoverBrushSelected_{};
};

/// What the addon does with a drop.
enum class DropAction
{
    /// The addon does not take the drop. The stock handlers take it.
    None,
    NewBrush,
    ReplaceModel,
    /// Set the material of the target brushes.
    AssignMaterial,
    /// Make a material for the texture and set it on the target brushes.
    AssignTexture,
    /// The addon takes the drop and shows a hint. The scene does not change.
    HintSelectBrush,
    /// The cursor is over a result but no brush made the face. The hint is shown and nothing changes.
    HintNoBrush,
};

/// The brushes that get a material drop.
enum class DropTarget
{
    /// All selected brushes.
    Selected,
    /// Only the brush under the cursor.
    Hovered,
};

/// What a material edit changes on a brush.
enum class MaterialScope
{
    /// Set the shape material. Keep the face overrides.
    Shape,
    /// Set the shape material and clear all face overrides.
    ShapeClearOverrides,
    /// Set one face override.
    Slot,
};

/// The result of DecideDrop.
struct DropDecision
{
    DropAction action_{DropAction::None};
    DropTarget target_{DropTarget::Selected};
    MaterialScope scope_{MaterialScope::Shape};
};

/// Return true if the node is the temporary result node "CsgResult" of the root node.
bool IsResultNodeOfRoot(Node* node, Node* rootNode);

/// Decide what to do with a drop. The function is pure.
/// Material and texture: the default scope is the whole shape. Shift clears the face overrides. Ctrl sets one face.
/// Ctrl wins over Shift.
/// - The cursor is over the result of any root and no brush made the face: hint, no change.
/// - The brush under the cursor is selected: all selected brushes get the edit.
/// - The brush under the cursor is not selected: only that brush gets the edit.
/// - No result under the cursor: a texture goes to the selected brushes. A material goes to the stock handler.
DropDecision DecideDrop(DropPayloadKind kind, const DropSelection& selection, bool shift, bool ctrl);

/// The materials of a brush as the editor sees them.
struct BrushMaterials
{
    /// True for a brush with a generated shape. It has the shape material and the face overrides.
    bool procedural_{};
    /// The shape material. Always empty for a model brush.
    ea::string shape_;
    /// The face overrides of a procedural brush, or the material slots of a model brush.
    ea::vector<ea::string> slots_;
};

/// Plan one material edit. The function is pure. Return false and keep `io` if the slot is not valid.
/// - Shape: set the shape material and keep the overrides. A model brush gets all slots.
/// - ShapeClearOverrides: set the shape material and clear all overrides.
/// - Slot: set one slot. The slot must be in range, but slot 0 of an empty model list grows the list.
/// A procedural brush with no shape material and a set slot 0 is an old brush. The plan first moves
/// slot 0 to the shape material and clears the slots equal to it. The render result stays the same.
bool PlanMaterialEdit(BrushMaterials& io, MaterialScope scope, unsigned slot, const ea::string& material);

/// Return the slots of a procedural brush that differ from the shape material.
/// A slot counts if it is not empty and not equal to the shape material.
/// With no shape material, slot 0 is the shape material (old brush). A model brush gives none.
ea::vector<unsigned> GetOverriddenSlots(const BrushMaterials& materials);

/// Return the face slot of a handle of a shape type, or nothing if the shape has no known mapping.
/// Only the box has one: the face handles 20 to 25 are the face groups 0 to 5 (id minus 20).
ea::optional<unsigned> GetFaceSlotOfHandle(const ea::string& shapeTypeName, unsigned handleId);

/// Read the materials of the brush node. Return false if the node has no brush or no material list.
bool ReadBrushMaterials(Node* brush, BrushMaterials& materials);
/// Plan the edit on the attributes of the brush node and write the attributes "Shape Material" and "Materials".
/// Return true if an attribute changed. Return false with no change for an invalid edit.
bool EditBrushMaterial(Node* brush, MaterialScope scope, unsigned slot, const ea::string& material);

/// Return the direct children with an enabled CsgBrush, in child order.
ea::vector<Node*> GetOperands(Node* rootNode);
/// Return the direct children with a CsgBrush, enabled or not, in child order.
ea::vector<Node*> GetAllBrushChildren(Node* rootNode);

/// Move a brush child. The indices count the brush children only. The root rebuilds after the move.
bool MoveOperand(Node* rootNode, unsigned fromIndex, unsigned toIndex);
/// Set the enabled state of the CsgBrush component. The root rebuilds after the change.
bool SetBrushEnabled(Node* brush, bool enabled);
/// Toggle the enabled state of the CsgBrush component.
bool ToggleBrushEnabled(Node* brush);

/// Return a text with one line for each operand and the build status of the root.
/// A brush without a "Model" attribute shows its type name.
ea::string DescribeRoot(Node* rootNode);

/// Return the StaticModel of the temporary result node of the root node, or null.
StaticModel* FindResultStaticModel(Node* rootNode);

/// The destination of a baked model file.
struct BakedModelPath
{
    /// The resource name of the file, for example "Models/Wall_Baked.mdl". Empty if no resource directory is known.
    ea::string resourceName_;
    /// The absolute file name.
    ea::string fileName_;
};

/// Choose the file for the baked model of a root: "Models/<name>_Baked.mdl" in a resource directory.
/// The resource directory is that of the first hint that is a resource with a file name. A hint is a resource name
/// or an absolute file name, for example the scene file. If no hint works, the fallback directory is used.
/// The fallback is an absolute directory name with a final slash, or empty. It must be a resource directory.
/// The function never chooses an existing file.
/// It appends _1, _2 and so on to the name. The name of the node gets only letters, digits, '_', '-' and '.'.
/// Return empty names if no hint and no fallback gives a directory.
BakedModelPath ChooseBakedModelPath(Context* context, ea::span<const ea::string> hintResourceNames,
    const ea::string& fallbackDirectory, const ea::string& rootNodeName);

/// Save the result model of the root node to the file with Model::Save. The directory of the file must exist or
/// the function makes it. Return false with an error log if the root has no result or the write fails.
/// A failed write removes the file.
bool SaveBakedModel(Context* context, Node* rootNode, const ea::string& fileName);

/// Return true if the result of the root is final: no build runs and no rebuild is pending.
/// A bake of a result that is not final saves an old model.
bool CanBakeNow(const RootStats& stats);

/// Create the sibling node "<root name> Baked" with a StaticModel. The node has the local transform of the root node.
/// The two nodes have the same parent, so the world transforms are equal.
/// The model comes from the resource cache. The function releases a cached model of that name first, so a reused
/// file name loads the new file. The materials are those of the result of the root.
/// Return null with an error log if the root node has no parent or the model does not load.
Node* CreateBakedNode(Node* rootNode, const ea::string& modelResourceName);

} // namespace Urho3D
