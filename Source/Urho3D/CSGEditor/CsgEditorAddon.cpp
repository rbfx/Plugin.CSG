// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSGEditor/CsgEditorAddon.h"

#include "Urho3D/CSG/CsgOperationColor.h"
#include "Urho3D/CSGEditor/CsgEditorOps.h"

#include <Editor/Core/CommonEditorActions.h>
#include <Editor/Core/HotkeyManager.h>
#include <Editor/Core/IniHelpers.h>
#include <Editor/Foundation/SceneViewTab/SceneSelectionRenderer.h>
#include <Editor/Foundation/SceneViewTab/TransformManipulator.h>

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Format.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Drawable.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Component.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/SystemUI/DragDropPayload.h>
#include <Urho3D/SystemUI/SystemUI.h>
#include <Urho3D/SystemUI/TransformGizmo.h>
#include <Urho3D/SystemUI/Widgets.h>
#include <Urho3D/Utility/SceneRendererToTexture.h>

#include <IconFontCppHeaders/IconsFontAwesome6.h>
#include <ImGuizmo/ImGuizmo.h>

#include <cmath>

namespace Urho3D
{

namespace
{

/// The editor settings key of the chosen preset.
const char* const PresetIniKey = "CsgEditor.BrushPreset";
/// The distance of a new brush from the camera if the ray hits nothing.
const float FallbackDistance = 10.0f;

/// The enum names of the operations. The hotkeys use them to find the operation index.
const char* const UnionName = "Union";
const char* const DifferenceName = "DifferenceAB";
const char* const IntersectionName = "Intersection";

const auto Hotkey_AddUnion = EditorHotkey{"Csg.AddUnion"}.Press(KEY_B);
const auto Hotkey_AddSubtract = EditorHotkey{"Csg.AddSubtract"}.Shift().Press(KEY_B);
const auto Hotkey_SetUnion = EditorHotkey{"Csg.SetUnion"}.Press(KEY_1);
const auto Hotkey_SetDifference = EditorHotkey{"Csg.SetDifference"}.Press(KEY_2);
const auto Hotkey_SetIntersection = EditorHotkey{"Csg.SetIntersection"}.Press(KEY_3);
// Some keyboard layouts need AltGr for [ and ]. On Windows, AltGr is Ctrl+Alt. The bindings accept Ctrl and Alt.
const auto Hotkey_MoveEarlier = EditorHotkey{"Csg.MoveEarlier"}.MaybeCtrl().MaybeAlt().Press(KEY_LEFTBRACKET);
const auto Hotkey_MoveLater = EditorHotkey{"Csg.MoveLater"}.MaybeCtrl().MaybeAlt().Press(KEY_RIGHTBRACKET);
const auto Hotkey_ToggleBrush = EditorHotkey{"Csg.ToggleBrush"}.Press(KEY_H);

const StringHash RootType("CsgRoot");
const StringHash BrushType("CsgBrush");
const StringHash RebuildEvent("CsgRequestRebuild");

/// The width of the operand overlay in pixels and its distance from the view corner.
const float OverlayWidth = 260.0f;
const float OverlayMargin = 8.0f;
/// The maximum height of the operand overlay as a fraction of the view height. A longer list scrolls.
const float OverlayMaxHeightFraction = 0.4f;
/// The maximum number of debug shapes that the addon draws in one frame. The other operands are skipped.
const unsigned MaxDebugShapesPerFrame = 64;
/// The editor settings key of the "Draw Debug Geometry" option of the selection renderer.
const char* const DrawDebugGeometryIniKey = "SceneSelectionRenderer.DrawDebugGeometry";

/// The editor settings keys of the shape handles.
const char* const ShowHandlesIniKey = "CsgEditor.ShowHandles";
const char* const AutoRecenterIniKey = "CsgEditor.AutoRecenter";
const char* const HandleGridStepIniKey = "CsgEditor.HandleGridStep";
const float DefaultHandleGridStep = 0.125f;
/// The size of a handle and the pick radius in pixels.
const float HandleSize = 8.0f;
const float HandlePickRadius = 10.0f;
/// The length of the short axis line of an axis handle in pixels.
const float HandleAxisLineLength = 18.0f;
/// The half length of the long axis line of a selected axis handle in pixels. The draw list clips it.
const float HandleAxisGuideLength = 4000.0f;
const ImU32 HandlePointColor = IM_COL32(255, 255, 255, 255);
const ImU32 HandleAxisColor = IM_COL32(255, 150, 30, 255);
const ImU32 HandleHoveredColor = IM_COL32(255, 230, 0, 255);
const ImU32 HandleSelectedColor = IM_COL32(0, 230, 255, 255);
const ImU32 HandleOutlineColor = IM_COL32(0, 0, 0, 200);

/// Return true if the selection renderer draws the debug shape of the brush.
/// It draws a selected component, and all components of a selected node and of its children.
bool IsDrawnBySelectionRenderer(const SceneSelection& selection, Node* brushNode, Component* brush)
{
    if (selection.IsSelected(brush))
        return true;
    // The selection renderer does not draw the children of a selected scene.
    for (Node* node = brushNode; node && node->GetParent(); node = node->GetParent())
    {
        if (selection.IsSelected(node))
            return true;
    }
    return false;
}

/// Return the read-only build status of the root, or an empty string.
ea::string GetBuildStatus(Node* rootNode)
{
    Component* root = rootNode ? rootNode->GetComponent(RootType) : nullptr;
    return root ? root->GetAttribute("Build Status").GetString() : EMPTY_STRING;
}

/// Send the rebuild event to the root of the node in the scene, if the root exists.
void RequestRebuild(Scene* scene, unsigned rootNodeId)
{
    Node* rootNode = scene ? scene->GetNode(rootNodeId) : nullptr;
    if (Component* root = rootNode ? rootNode->GetComponent(RootType) : nullptr)
        root->SendEvent(RebuildEvent);
}

/// Wrap an action and make the root rebuild after each undo and redo.
/// Node::ReorderChild sends no event, so the root does not see the new order without the event.
class RebuildRootActionWrapper : public BaseEditorActionWrapper
{
public:
    RebuildRootActionWrapper(SharedPtr<EditorAction> action, Node* rootNode)
        : BaseEditorActionWrapper(action)
        , scene_(rootNode->GetScene())
        , rootNodeId_(rootNode->GetID())
    {
    }

    void Redo() const override
    {
        BaseEditorActionWrapper::Redo();
        RequestRebuild(scene_, rootNodeId_);
    }

    void Undo() const override
    {
        BaseEditorActionWrapper::Undo();
        RequestRebuild(scene_, rootNodeId_);
    }

private:
    const WeakPtr<Scene> scene_;
    const unsigned rootNodeId_{};
};

/// The undo action of one handle drag or one recenter. It keeps the attributes of the brush
/// and the position of its node before and after. One undo step reverts both.
/// It finds the component by the node id and the type. Then it works after the node is created again.
class ShapeHandleDragAction : public EditorAction
{
public:
    ShapeHandleDragAction(Component* component, ShapeEditState oldState, ShapeEditState newState)
        : scene_(component->GetScene())
        , nodeId_(component->GetNode()->GetID())
        , type_(component->GetType())
        , oldState_(ea::move(oldState))
        , newState_(ea::move(newState))
    {
    }

    /// Implement EditorAction.
    /// @{
    bool CanUndoRedo() const override { return GetComponent() != nullptr; }
    void Redo() const override { RestoreShapeEditState(GetComponent(), newState_); }
    void Undo() const override { RestoreShapeEditState(GetComponent(), oldState_); }
    bool MergeWith(const EditorAction& other) override { return false; }
    /// @}

private:
    Component* GetComponent() const
    {
        Node* node = scene_ ? scene_->GetNode(nodeId_) : nullptr;
        return node ? node->GetComponent(type_) : nullptr;
    }

    const WeakPtr<Scene> scene_;
    const unsigned nodeId_{};
    const StringHash type_;
    const ShapeEditState oldState_;
    const ShapeEditState newState_;
};

/// Return the integer value of the key in the editor settings output of the addon.
ea::optional<int> ReadAddonIniInt(SceneViewAddon* addon, const char* key)
{
    if (!addon)
        return ea::nullopt;

    ImGuiTextBuffer buffer;
    addon->WriteIniSettings(buffer);
    const ea::string_view text{buffer.c_str(), static_cast<eastl_size_t>(buffer.size())};
    eastl_size_t start = 0;
    while (start < text.size())
    {
        eastl_size_t end = text.find('\n', start);
        if (end == ea::string_view::npos)
            end = text.size();
        if (const auto value = ReadIntFromIni(text.substr(start, end - start), key))
            return value;
        start = end + 1;
    }
    return ea::nullopt;
}

/// Return the screen position of the world point in the view, or none if the point is behind the camera.
ea::optional<Vector2> ProjectToView(Camera* camera, const Rect& area, const Vector3& worldPos)
{
    const Vector3 eyePos = camera->GetView() * worldPos;
    if (eyePos.z_ <= camera->GetNearClip())
        return ea::nullopt;
    return area.min_ + camera->WorldToScreenPoint(worldPos) * area.Size();
}

/// Return the screen direction of the world axis at the world point, or none.
ea::optional<Vector2> ProjectAxisToView(Camera* camera, const Rect& area, const Vector3& worldPos, const Vector3& worldAxis)
{
    const auto start = ProjectToView(camera, area, worldPos);
    // A step of 10% of the camera distance gives a stable direction.
    const float step = ea::max(0.001f, (worldPos - camera->GetNode()->GetWorldPosition()).Length() * 0.1f);
    const auto end = ProjectToView(camera, area, worldPos + worldAxis.Normalized() * step);
    if (!start || !end || (*end - *start).LengthSquared() < 1e-6f)
        return ea::nullopt;
    return (*end - *start).Normalized();
}

/// Return the index of the child in the child list of its parent.
unsigned GetChildIndex(Node* child)
{
    const ea::vector<SharedPtr<Node>>& children = child->GetParent()->GetChildren();
    unsigned index = 0;
    while (index < children.size() && children[index] != child)
        ++index;
    return index;
}

const char* GetOperationIcon(const ea::string& name)
{
    if (name == UnionName)
        return ICON_FA_PLUS;
    if (name == DifferenceName)
        return ICON_FA_MINUS;
    if (name == IntersectionName)
        return ICON_FA_CIRCLE_HALF_STROKE;
    return ICON_FA_QUESTION;
}

const EditorHotkey* GetOperationHotkey(const ea::string& name)
{
    if (name == UnionName)
        return &Hotkey_SetUnion;
    if (name == DifferenceName)
        return &Hotkey_SetDifference;
    if (name == IntersectionName)
        return &Hotkey_SetIntersection;
    return nullptr;
}

/// Return the nearest hit point of the ray in the scene. The query skips the excluded node and its children.
ea::optional<Vector3> QueryHitPoint(Scene* scene, const Ray& ray, Node* excluded = nullptr)
{
    ea::optional<Vector3> result;
    float bestDistance = M_INFINITY;
    for (const RayQueryResult& hit : QueryGeometriesFromScene(scene, ray))
    {
        if (excluded && hit.node_ && (hit.node_ == excluded || hit.node_->IsChildOf(excluded)))
            continue;
        if (hit.distance_ < bestDistance)
        {
            bestDistance = hit.distance_;
            result = hit.position_;
        }
    }
    return result;
}

/// Return the first resource of the payload, or null.
const ResourceFileDescriptor* GetFirstResource(DragDropPayload* payload)
{
    const auto resourcePayload = dynamic_cast<ResourceDragDropPayload*>(payload);
    if (!resourcePayload || resourcePayload->resources_.empty())
        return nullptr;
    return &resourcePayload->resources_[0];
}

DropPayloadKind GetPayloadKind(const ResourceFileDescriptor* desc)
{
    if (!desc)
        return DropPayloadKind::Other;
    if (desc->HasObjectType<Model>())
        return DropPayloadKind::Model;
    if (desc->HasObjectType<Material>())
        return DropPayloadKind::Material;
    if (IsTextureResourceName(desc->resourceName_))
        return DropPayloadKind::Texture;
    return DropPayloadKind::Other;
}

/// Return the first resource of the payload if it is a Model.
const ResourceFileDescriptor* GetModelResource(DragDropPayload* payload)
{
    const ResourceFileDescriptor* desc = GetFirstResource(payload);
    return GetPayloadKind(desc) == DropPayloadKind::Model ? desc : nullptr;
}

bool IsShiftDown() { return ui::IsKeyDown(KEY_LSHIFT) || ui::IsKeyDown(KEY_RSHIFT); }

} // namespace

void RbfxCsg_EditorAddon(Context* context, SceneViewTab* sceneViewTab)
{
    sceneViewTab->RegisterAddon<CsgEditorAddon>();
}

CsgEditorAddon::CsgEditorAddon(SceneViewTab* owner)
    : SceneViewAddon(owner)
{
    auto hotkeyManager = owner_->GetHotkeyManager();
    hotkeyManager->BindHotkey(this, Hotkey_AddUnion, &CsgEditorAddon::AddUnionBrush);
    hotkeyManager->BindHotkey(this, Hotkey_AddSubtract, &CsgEditorAddon::AddSubtractBrush);
    hotkeyManager->BindHotkey(this, Hotkey_SetUnion, &CsgEditorAddon::SetUnion);
    hotkeyManager->BindHotkey(this, Hotkey_SetDifference, &CsgEditorAddon::SetDifference);
    hotkeyManager->BindHotkey(this, Hotkey_SetIntersection, &CsgEditorAddon::SetIntersection);
    hotkeyManager->BindHotkey(this, Hotkey_MoveEarlier, &CsgEditorAddon::MoveEarlier);
    hotkeyManager->BindHotkey(this, Hotkey_MoveLater, &CsgEditorAddon::MoveLater);
    hotkeyManager->BindHotkey(this, Hotkey_ToggleBrush, &CsgEditorAddon::ToggleSelectedBrushes);

    URHO3D_LOGDEBUG("CSG editor addon is registered");
}

bool CsgEditorAddon::IsAvailable() const
{
    // Plugin.CSG registers the types when the project loads it. The check runs each time.
    return context_->GetReflection(RootType) != nullptr && context_->GetReflection(BrushType) != nullptr;
}

int CsgEditorAddon::FindOperation(const ea::string& name) const
{
    const int count = OperationCount(context_);
    for (int index = 0; index < count; ++index)
    {
        if (OperationName(context_, index) == name)
            return index;
    }
    return -1;
}

ea::vector<Node*> CsgEditorAddon::GetSelectedBrushes(SceneViewPage& page) const
{
    ea::vector<Node*> result;
    for (Node* node : page.selection_.GetNodes())
    {
        if (node && IsBrushNode(node))
            result.push_back(node);
    }
    return result;
}

void CsgEditorAddon::AddBrush(int operation, bool useCursorRay)
{
    if (!IsAvailable() || operation < 0)
        return;

    SceneViewPage* page = owner_->GetActivePage();
    if (!page || page->IsSimulationActive())
        return;

    Scene* scene = page->scene_;
    Node* activeNode = page->selection_.GetActiveNode();
    Node* rootNode = FindActiveRoot(activeNode);

    // The hotkey uses the cursor ray. The toolbar button uses the ray through the center of the view.
    Camera* camera = page->renderer_->GetCamera();
    const Ray ray = useCursorRay ? page->cameraRay_ : camera->GetScreenRay(0.5f, 0.5f);

    Vector3 position;
    if (const auto hitPoint = QueryHitPoint(scene, ray))
        position = *hitPoint;
    else if (rootNode)
        position = rootNode->GetWorldPosition();
    else
        position = ray.origin_ + ray.direction_ * FallbackDistance;

    // One action creates the top node with all its children. Then one undo step removes all.
    const CreateNodeActionBuilder builder{scene, AttributeScopeHint::Node};

    Node* createdNode = nullptr;
    if (!rootNode)
    {
        Node* parent = activeNode && activeNode->GetParent() ? activeNode->GetParent() : scene;
        rootNode = CreateRootNode(parent, position);
        if (!rootNode)
            return;
        createdNode = rootNode;
    }

    Node* brushNode = AddPresetBrushNode(rootNode, GetActivePreset(), operation, position);
    if (!brushNode)
    {
        if (createdNode)
            createdNode->Remove();
        return;
    }
    if (!createdNode)
        createdNode = brushNode;

    page->selection_.Clear();
    page->selection_.SetSelected(brushNode, true);

    owner_->PushAction(builder.Build(createdNode));
}

void CsgEditorAddon::SetSelectedOperation(int operation)
{
    if (!IsAvailable() || operation < 0)
        return;

    SceneViewPage* page = owner_->GetActivePage();
    if (!page || page->IsSimulationActive())
        return;

    ea::vector<Component*> components;
    VariantVector oldValues;
    VariantVector newValues;
    for (Node* brushNode : GetSelectedBrushes(*page))
    {
        Component* component = FindBrushComponent(brushNode);
        if (!component || GetOperation(brushNode) == operation)
            continue;

        const Variant oldValue = component->GetAttribute("Operation");
        // SetOperation calls ApplyAttributes. The brush marks itself dirty and the root rebuilds.
        if (!SetOperation(brushNode, operation))
            continue;

        components.push_back(component);
        oldValues.push_back(oldValue);
        newValues.push_back(component->GetAttribute("Operation"));
    }

    if (components.empty())
        return;

    owner_->PushAction<ChangeComponentAttributesAction>(page->scene_.Get(), "Operation", components, oldValues, newValues);
}

void CsgEditorAddon::MoveActiveBrush(int step)
{
    if (!IsAvailable())
        return;

    SceneViewPage* page = owner_->GetActivePage();
    if (!page || page->IsSimulationActive())
        return;

    Node* brushNode = page->selection_.GetActiveNode();
    Node* rootNode = brushNode ? brushNode->GetParent() : nullptr;
    if (!IsBrushNode(brushNode) || !IsRootNode(rootNode))
        return;

    // The operand index counts the brush children only. The move skips the other children.
    const ea::vector<Node*> brushes = GetAllBrushChildren(rootNode);
    const auto iter = ea::find(brushes.begin(), brushes.end(), brushNode);
    const int fromIndex = static_cast<int>(iter - brushes.begin());
    const int toIndex = fromIndex + step;
    if (iter == brushes.end() || toIndex < 0 || toIndex >= static_cast<int>(brushes.size()))
        return;

    const unsigned oldChildIndex = GetChildIndex(brushNode);
    // MoveOperand sends the rebuild event. The wrapper sends it again on undo and redo.
    if (!MoveOperand(rootNode, fromIndex, toIndex))
        return;
    const unsigned newChildIndex = GetChildIndex(brushNode);

    const auto reorder = MakeShared<ReorderNodeAction>(brushNode, oldChildIndex, newChildIndex);
    owner_->PushAction(MakeShared<RebuildRootActionWrapper>(reorder, rootNode));
}

void CsgEditorAddon::ToggleSelectedBrushes()
{
    if (!IsAvailable())
        return;

    SceneViewPage* page = owner_->GetActivePage();
    if (!page || page->IsSimulationActive())
        return;

    ea::vector<Component*> components;
    VariantVector oldValues;
    VariantVector newValues;
    for (Node* brushNode : GetSelectedBrushes(*page))
    {
        Component* component = FindBrushComponent(brushNode);
        const Variant oldValue = component->GetAttribute("Is Enabled");
        // The brush registers with the root when its enabled state changes. The root rebuilds then.
        if (!ToggleBrushEnabled(brushNode))
            continue;
        components.push_back(component);
        oldValues.push_back(oldValue);
        newValues.push_back(component->GetAttribute("Is Enabled"));
    }

    if (components.empty())
        return;

    // The attribute "Is Enabled" calls SetEnabled. Then undo and redo also make the root rebuild.
    owner_->PushAction<ChangeComponentAttributesAction>(page->scene_.Get(), "Is Enabled", components, oldValues, newValues);
}

void CsgEditorAddon::MoveEarlier()
{
    MoveActiveBrush(-1);
}

void CsgEditorAddon::MoveLater()
{
    MoveActiveBrush(1);
}

void CsgEditorAddon::AddUnionBrush()
{
    AddBrush(FindOperation(UnionName), true);
}

void CsgEditorAddon::AddSubtractBrush()
{
    AddBrush(FindOperation(DifferenceName), true);
}

void CsgEditorAddon::SetUnion()
{
    SetSelectedOperation(FindOperation(UnionName));
}

void CsgEditorAddon::SetDifference()
{
    SetSelectedOperation(FindOperation(DifferenceName));
}

void CsgEditorAddon::SetIntersection()
{
    SetSelectedOperation(FindOperation(IntersectionName));
}

bool CsgEditorAddon::RenderToolbar()
{
    if (!IsAvailable())
        return false;

    auto hotkeyManager = owner_->GetHotkeyManager();
    SceneViewPage* page = owner_->GetActivePage();
    Node* activeNode = page ? page->selection_.GetActiveNode() : nullptr;
    // The tooltips show the build status of the active root.
    const ea::string buildStatus = GetBuildStatus(FindActiveRoot(activeNode));

    const auto makeTooltip = [&](const ea::string& title, const EditorHotkey* hotkey)
    {
        const ea::string label = hotkey ? hotkeyManager->GetHotkeyLabel(*hotkey) : ea::string{};
        const ea::string text = label.empty() ? title : Format("{} ({})", title, label);
        return buildStatus.empty() ? text : Format("{}\nCSG Build Status: {}", text, buildStatus);
    };

    RenderPresetCombo();

    if (Widgets::ToolbarButton(ICON_FA_SQUARE_PLUS, makeTooltip("Add CSG Union Brush", &Hotkey_AddUnion).c_str()))
        AddBrush(FindOperation(UnionName), false);
    if (Widgets::ToolbarButton(
            ICON_FA_SQUARE_MINUS, makeTooltip("Add CSG Subtract Brush", &Hotkey_AddSubtract).c_str()))
        AddBrush(FindOperation(DifferenceName), false);

    // The operation buttons show the operation of the active brush. They change all selected brushes.
    const bool hasBrushes = page && !GetSelectedBrushes(*page).empty();
    const int activeOperation = activeNode ? GetOperation(activeNode) : -1;

    ui::BeginDisabled(!hasBrushes);
    const int count = OperationCount(context_);
    for (int index = 0; index < count; ++index)
    {
        const ea::string name = OperationName(context_, index);
        const ea::string tooltip = makeTooltip(Format("CSG Operation: {}", name), GetOperationHotkey(name));
        if (Widgets::ToolbarButton(GetOperationIcon(name), tooltip.c_str(), index == activeOperation))
            SetSelectedOperation(index);
    }
    ui::EndDisabled();

    Widgets::ToolbarSeparator();
    return true;
}

const CsgBrushPreset& CsgEditorAddon::GetActivePreset() const
{
    // A hidden preset falls back to the first preset that the editor can use.
    const ea::span<const CsgBrushPreset> presets = GetBrushPresets();
    if (presetIndex_ < presets.size() && IsBrushPresetAvailable(context_, presets[presetIndex_]))
        return presets[presetIndex_];
    for (const CsgBrushPreset& preset : presets)
    {
        if (IsBrushPresetAvailable(context_, preset))
            return preset;
    }
    return presets[0];
}

void CsgEditorAddon::RenderPresetCombo()
{
    // The padding gives the combo the height of a toolbar button.
    const ImGuiStyle& style = ui::GetStyle();
    const float paddingY = ea::max(0.0f, (Widgets::GetSmallButtonSize() - ui::GetFontSize()) * 0.5f);
    ui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2{style.FramePadding.x, paddingY});
    ui::SetNextItemWidth(ui::CalcTextSize("Cylinder.mdl").x + style.FramePadding.x * 2.0f + ui::GetFrameHeight());

    const ea::span<const CsgBrushPreset> presets = GetBrushPresets();
    const CsgBrushPreset& active = GetActivePreset();
    if (ui::BeginCombo("##CsgBrushPreset", active.name_))
    {
        for (unsigned index = 0; index < presets.size(); ++index)
        {
            // A procedural preset is hidden if its type is not registered.
            if (!IsBrushPresetAvailable(context_, presets[index]))
                continue;
            const bool selected = &presets[index] == &active;
            if (ui::Selectable(presets[index].name_, selected))
                presetIndex_ = index;
            if (selected)
                ui::SetItemDefaultFocus();
        }
        ui::EndCombo();
    }
    ui::PopStyleVar();

    if (ui::IsItemHovered())
        ui::SetTooltip("%s",
            Format("CSG brush preset: {}", active.typeName_ ? active.typeName_ : active.model_).c_str());
    ui::SameLine(0, 0);
    Widgets::ToolbarSeparator();
}

void CsgEditorAddon::WriteIniSettings(ImGuiTextBuffer& output)
{
    WriteStringToIni(output, PresetIniKey, GetBrushPresets()[presetIndex_].name_);
    WriteIntToIni(output, ShowHandlesIniKey, showHandles_);
    WriteIntToIni(output, AutoRecenterIniKey, autoRecenter_);
    WriteStringToIni(output, HandleGridStepIniKey, Format("{}", handleGridStep_));
}

void CsgEditorAddon::ReadIniSettings(const char* line)
{
    // An unknown name keeps the current preset.
    if (const auto value = ReadStringFromIni(line, PresetIniKey))
    {
        const ea::span<const CsgBrushPreset> presets = GetBrushPresets();
        for (unsigned index = 0; index < presets.size(); ++index)
        {
            if (*value == presets[index].name_)
                presetIndex_ = index;
        }
    }
    if (const auto value = ReadIntFromIni(line, ShowHandlesIniKey))
        showHandles_ = *value != 0;
    if (const auto value = ReadIntFromIni(line, AutoRecenterIniKey))
        autoRecenter_ = *value != 0;
    if (const auto value = ReadStringFromIni(line, HandleGridStepIniKey))
    {
        // A value that is not finite gives the default. A negative value turns off the snap.
        const float step = ToFloat(*value);
        handleGridStep_ = std::isfinite(step) ? ea::max(0.0f, step) : DefaultHandleGridStep;
    }
}

CsgEditorAddon::DropMode CsgEditorAddon::GetDropMode(SceneViewPage& page, DragDropPayload* payload) const
{
    const DropPayloadKind kind = GetPayloadKind(GetFirstResource(payload));
    if (!IsAvailable() || page.IsSimulationActive() || kind == DropPayloadKind::Other)
        return DropMode::None;

    DropSelection selection;
    Node* activeNode = page.selection_.GetActiveNode();
    if (activeNode)
    {
        selection.activeIsModelBrush_ = IsModelBrushNode(activeNode);
        selection.activeIsBrush_ = IsBrushNode(activeNode);
        // The result node of a root is a temporary child. It maps to the root.
        selection.activeIsRoot_ = IsRootNode(activeNode) || (activeNode->IsTemporary() && IsRootNode(activeNode->GetParent()));
    }
    selection.anySelectedBrush_ = !GetSelectedBrushes(page).empty();
    selection.hoveringResult_ = kind == DropPayloadKind::Material && IsHoveringSelectedResult(page);

    switch (DecideDrop(kind, selection, IsShiftDown()).action_)
    {
    case DropAction::NewBrush: return DropMode::NewBrush;
    case DropAction::ReplaceModel: return DropMode::ReplaceModel;
    case DropAction::AssignMaterial: return DropMode::AssignMaterial;
    case DropAction::AssignTexture: return DropMode::AssignTexture;
    case DropAction::HintSelectBrush: return DropMode::Hint;
    case DropAction::None: break;
    }
    return DropMode::None;
}

bool CsgEditorAddon::IsHoveringSelectedResult(SceneViewPage& page) const
{
    const auto hits = QueryGeometriesFromScene(page.scene_, page.cameraRay_);
    if (hits.empty() || !hits[0].node_)
        return false;
    // The roots of the selected nodes: a selected root, or the parent of a selected brush.
    for (Node* node : page.selection_.GetNodes())
    {
        if (IsResultNodeOfRoot(hits[0].node_, FindActiveRoot(node)))
            return true;
    }
    return false;
}

void CsgEditorAddon::ApplyDropMaterial(const ea::string& materialName, bool allSlots)
{
    for (Component* component : dropComponents_)
    {
        if (!component || !component->GetNode())
            continue;
        if (allSlots)
            SetAllBrushMaterialSlots(component->GetNode(), materialName);
        else
            SetBrushMaterial(component->GetNode(), materialName, 0);
    }
}

void CsgEditorAddon::RestoreDropMaterials()
{
    for (unsigned index = 0; index < dropComponents_.size() && index < dropOldMaterials_.size(); ++index)
    {
        if (Component* component = dropComponents_[index])
        {
            component->SetAttribute("Materials", dropOldMaterials_[index]);
            component->ApplyAttributes();
        }
    }
    dropPreviewApplied_ = false;
}

bool CsgEditorAddon::IsDragDropPayloadSupported(SceneViewPage& page, DragDropPayload* payload) const
{
    // The tab gives the drop to the first addon by input priority that supports it.
    // A false result here gives the drop to the stock handlers.
    return GetDropMode(page, payload) != DropMode::None;
}

void CsgEditorAddon::BeginDragDrop(SceneViewPage& page, DragDropPayload* payload)
{
    ResetDrop();

    const ResourceFileDescriptor* desc = GetFirstResource(payload);
    if (!desc)
        return;

    dropMode_ = GetDropMode(page, payload);
    dropPage_ = &page;
    Node* activeNode = page.selection_.GetActiveNode();

    if (dropMode_ == DropMode::AssignMaterial || dropMode_ == DropMode::AssignTexture)
    {
        // A material gives its own name. A texture gives the name of its material file.
        dropResourceName_ =
            dropMode_ == DropMode::AssignMaterial ? desc->resourceName_ : GetTextureMaterialName(desc->resourceName_);
        dropAllSlots_ = IsShiftDown();
        for (Node* brushNode : GetSelectedBrushes(page))
        {
            Component* component = FindBrushComponent(brushNode);
            if (!component || component->GetAttribute("Materials").GetType() != VAR_RESOURCEREFLIST)
                continue;
            dropComponents_.emplace_back(component);
            dropOldMaterials_.push_back(component->GetAttribute("Materials"));
        }
        // The live preview sets the material now. A texture has a preview only if its material file exists.
        // The drop never makes a file before the user releases the button.
        if (dropMode_ == DropMode::AssignMaterial || GetSubsystem<ResourceCache>()->Exists(dropResourceName_))
        {
            ApplyDropMaterial(dropResourceName_, dropAllSlots_);
            dropPreviewApplied_ = true;
        }
        return;
    }
    if (dropMode_ == DropMode::Hint)
        return;

    if (dropMode_ == DropMode::ReplaceModel)
    {
        // The live preview sets the model now. CancelDragDrop restores the old models.
        for (Node* brushNode : GetSelectedBrushes(page))
        {
            // A brush with a generated model has no "Model" attribute. The drop skips it.
            Component* component = FindBrushComponent(brushNode);
            if (!HasModelAttribute(component))
                continue;
            const Variant oldValue = component->GetAttribute("Model");
            if (!SetBrushModel(brushNode, desc->resourceName_))
                continue;
            dropComponents_.emplace_back(component);
            dropOldModels_.push_back(oldValue);
            dropNewModel_ = component->GetAttribute("Model");
        }
    }
    else if (dropMode_ == DropMode::NewBrush)
    {
        Node* rootNode = FindActiveRoot(activeNode);
        const int operation = FindOperation(UnionName);
        if (!rootNode || operation < 0)
            return;

        // The builder must exist before the brush. Then one undo step removes the brush.
        dropActionBuilder_ = ea::make_unique<CreateNodeActionBuilder>(page.scene_, AttributeScopeHint::Node);
        // The brush is disabled during the drag, so the CSG result does not change.
        // If the result contains the brush, the cursor ray hits the brush and the brush moves to the camera.
        dropBrush_ = AddBrushNode(rootNode, desc->resourceName_, operation, rootNode->GetWorldPosition(), false);
        if (!dropBrush_)
        {
            dropActionBuilder_ = nullptr;
            return;
        }

        // A temporary StaticModel shows the brush shape.
        auto cache = GetSubsystem<ResourceCache>();
        if (auto model = cache->GetResource<Model>(desc->resourceName_))
        {
            auto preview = dropBrush_->CreateComponent<StaticModel>();
            preview->SetTemporary(true);
            preview->SetModel(model);
            preview->ApplyMaterialList();
            if (auto defaultMaterial = cache->GetResource<Material>("Materials/DefaultWhite.xml"))
            {
                for (unsigned i = 0; i < preview->GetNumGeometries(); ++i)
                {
                    if (!preview->GetMaterial(i))
                        preview->SetMaterial(i, defaultMaterial);
                }
            }
            dropPreview_ = preview;
        }
        UpdateDropBrushPosition();
    }
}

void CsgEditorAddon::UpdateDropBrushPosition()
{
    if (!dropPage_ || !dropBrush_)
        return;

    Node* rootNode = dropBrush_->GetParent();
    if (const auto hitPoint = QueryHitPoint(dropPage_->scene_, dropPage_->cameraRay_, dropBrush_))
        dropBrush_->SetWorldPosition(*hitPoint);
    else if (rootNode)
        dropBrush_->SetWorldPosition(rootNode->GetWorldPosition());
}

void CsgEditorAddon::UpdateDragDrop(DragDropPayload* payload)
{
    if (dropMode_ == DropMode::NewBrush)
        UpdateDropBrushPosition();
    else if (dropMode_ == DropMode::Hint)
        ui::SetTooltip("Select a brush to assign a material");
    else if (dropMode_ == DropMode::AssignMaterial || dropMode_ == DropMode::AssignTexture)
    {
        ui::SetTooltip("Material: %s%s", dropResourceName_.c_str(), IsShiftDown() ? " (all slots)" : "");
        // Shift changes the slot mode. The preview follows it.
        if (dropPreviewApplied_ && IsShiftDown() != dropAllSlots_)
        {
            RestoreDropMaterials();
            dropAllSlots_ = IsShiftDown();
            ApplyDropMaterial(dropResourceName_, dropAllSlots_);
            dropPreviewApplied_ = true;
        }
    }
}

void CsgEditorAddon::CompleteDragDrop(DragDropPayload* payload)
{
    Scene* scene = dropPage_ ? dropPage_->scene_.Get() : nullptr;
    if (dropMode_ == DropMode::ReplaceModel && scene)
    {
        // The action skips a component that is removed or that is not in the scene of the page.
        // It also skips a component with no change of the model.
        ea::vector<Component*> components;
        VariantVector oldValues;
        VariantVector newValues;
        for (unsigned index = 0; index < dropComponents_.size(); ++index)
        {
            Component* component = dropComponents_[index];
            if (!component || !component->GetNode() || component->GetScene() != scene)
                continue;
            if (dropOldModels_[index] == dropNewModel_)
                continue;
            components.push_back(component);
            oldValues.push_back(dropOldModels_[index]);
            newValues.push_back(dropNewModel_);
        }
        if (!components.empty())
            owner_->PushAction<ChangeComponentAttributesAction>(scene, "Model", components, oldValues, newValues);
    }
    else if ((dropMode_ == DropMode::AssignMaterial || dropMode_ == DropMode::AssignTexture) && scene)
    {
        // The texture case makes its material file now. It reuses an existing file.
        ea::string materialName = dropResourceName_;
        if (dropMode_ == DropMode::AssignTexture)
        {
            const auto* resource = GetFirstResource(payload);
            materialName = resource ? EnsureTextureMaterial(context_, resource->resourceName_) : EMPTY_STRING;
        }

        RestoreDropMaterials();
        if (!materialName.empty())
        {
            ApplyDropMaterial(materialName, IsShiftDown());

            // The action skips a component that is removed or that is not in the scene of the page.
            // It also skips a component with no change of the materials.
            ea::vector<Component*> components;
            VariantVector oldValues;
            VariantVector newValues;
            for (unsigned index = 0; index < dropComponents_.size(); ++index)
            {
                Component* component = dropComponents_[index];
                if (!component || !component->GetNode() || component->GetScene() != scene)
                    continue;
                const Variant newValue = component->GetAttribute("Materials");
                if (dropOldMaterials_[index] == newValue)
                    continue;
                components.push_back(component);
                oldValues.push_back(dropOldMaterials_[index]);
                newValues.push_back(newValue);
            }
            if (!components.empty())
                owner_->PushAction<ChangeComponentAttributesAction>(scene, "Materials", components, oldValues, newValues);
        }
    }
    else if (dropMode_ == DropMode::NewBrush && dropBrush_ && dropActionBuilder_ && scene
        && dropBrush_->GetParent() && dropBrush_->GetScene() == scene)
    {
        if (dropPreview_)
            dropPreview_->Remove();

        // Shift at drop time makes a subtract brush.
        const int difference = FindOperation(DifferenceName);
        const bool shift = ui::IsKeyDown(KEY_LSHIFT) || ui::IsKeyDown(KEY_RSHIFT);
        if (shift && difference >= 0)
            SetOperation(dropBrush_, difference);
        SetBrushEnabled(dropBrush_, true);

        dropPage_->selection_.Clear();
        dropPage_->selection_.SetSelected(dropBrush_, true);

        owner_->PushAction(dropActionBuilder_->Build(dropBrush_));
    }
    else if (dropMode_ == DropMode::NewBrush && dropBrush_ && dropBrush_->GetParent())
    {
        // The root or the scene changed during the drag. The preview brush must not stay without an undo step.
        dropBrush_->Remove();
    }

    ResetDrop();
}

void CsgEditorAddon::CancelDragDrop()
{
    if (dropMode_ == DropMode::ReplaceModel)
    {
        for (unsigned index = 0; index < dropComponents_.size(); ++index)
        {
            if (Component* component = dropComponents_[index])
            {
                component->SetAttribute("Model", dropOldModels_[index]);
                component->ApplyAttributes();
            }
        }
    }
    else if (dropMode_ == DropMode::NewBrush && dropBrush_)
    {
        dropBrush_->Remove();
    }
    else if (dropMode_ == DropMode::AssignMaterial || dropMode_ == DropMode::AssignTexture)
    {
        RestoreDropMaterials();
    }

    ResetDrop();
}

void CsgEditorAddon::ResetDrop()
{
    dropMode_ = DropMode::None;
    dropPage_ = nullptr;
    dropBrush_ = nullptr;
    dropPreview_ = nullptr;
    dropActionBuilder_ = nullptr;
    dropComponents_.clear();
    dropOldModels_.clear();
    dropNewModel_ = Variant::EMPTY;
    dropOldMaterials_.clear();
    dropResourceName_.clear();
    dropAllSlots_ = false;
    dropPreviewApplied_ = false;
}

void CsgEditorAddon::Render(SceneViewPage& page)
{
    if (!IsAvailable())
        return;

    Node* rootNode = FindActiveRoot(page.selection_.GetActiveNode());
    if (!rootNode)
        return;

    // The tab creates the DebugRenderer of the scene before the addons render.
    // The selection renderer draws the selected brushes. The addon adds only the other operands.
    auto debug = page.scene_->GetComponent<DebugRenderer>();
    if (debug && IsSelectionDebugGeometryEnabled())
    {
        unsigned numDrawn = 0;
        for (Node* brushNode : GetOperands(rootNode))
        {
            Component* brush = FindBrushComponent(brushNode);
            if (!brush || IsDrawnBySelectionRenderer(page.selection_, brushNode, brush))
                continue;
            if (numDrawn++ >= MaxDebugShapesPerFrame)
                break;
            // DrawDebugGeometry is a Component virtual. The brush uses the color of its operation.
            brush->DrawDebugGeometry(debug, true);
        }
    }

    RenderOperandOverlay(page, rootNode);
}

bool CsgEditorAddon::IsSelectionDebugGeometryEnabled() const
{
    // The option is private in the selection renderer. Its editor settings output gives the value.
    const auto value = ReadAddonIniInt(owner_->GetAddon<SceneSelectionRenderer>(), DrawDebugGeometryIniKey);
    return !value || *value != 0;
}

bool CsgEditorAddon::IsTransformGizmoLocal() const
{
    // The handle gizmo uses the space of the transform manipulator. Without it, the gizmo uses world space.
    const auto manipulator = owner_->GetAddon<TransformManipulator>();
    return manipulator && manipulator->IsLocal();
}

void CsgEditorAddon::ProcessInput(SceneViewPage& page, bool& mouseConsumed)
{
    // The overlay rectangle is from the last frame. A click on the overlay must not select in the scene.
    if (overlayVisible_ && ui::IsMouseHoveringRect(ToImGui(overlayRect_.min_), ToImGui(overlayRect_.max_)))
        mouseConsumed = true;
    overlayVisible_ = false;

    // The content area is the last item. The scene selector uses the same check.
    const bool viewHovered = ui::IsItemHovered();
    gizmoHot_ = false;
    ProcessShapeHandles(page, mouseConsumed);
    ProcessResultPick(page, mouseConsumed, viewHovered);
}

void CsgEditorAddon::ProcessResultPick(SceneViewPage& page, bool& mouseConsumed, bool viewHovered)
{
    // A press that the handles, the overlay or the camera own does not start a pick.
    if (ui::IsMouseClicked(MOUSEB_LEFT))
        resultPickArmed_ = viewHovered && !mouseConsumed && !gizmoHot_ && !ui::IsMouseDown(MOUSEB_RIGHT);

    if (!ui::IsMouseReleased(MOUSEB_LEFT))
        return;
    const bool armed = resultPickArmed_;
    resultPickArmed_ = false;

    const bool alt = ui::IsKeyDown(KEY_LALT) || ui::IsKeyDown(KEY_RALT);
    if (!armed || !IsAvailable() || page.IsSimulationActive() || mouseConsumed || !viewHovered || alt
        || ui::IsMouseDragPastThreshold(MOUSEB_LEFT))
        return;

    // The nearest hit must be the result node of a root. The drop code uses the same query.
    const auto hits = QueryGeometriesFromScene(page.scene_, page.cameraRay_);
    if (hits.empty() || !hits[0].node_ || !hits[0].node_->IsTemporary())
        return;
    Node* rootNode = hits[0].node_->GetParent();
    if (!IsResultNodeOfRoot(hits[0].node_, rootNode) || !IsRootNode(rootNode))
        return;
    Component* rootComponent = rootNode->GetComponent(StringHash("CsgRoot"));

    // Source 0, a removed node and a node outside this root go to the stock selector, which selects the root.
    const PickResult pick = PickRootSource(rootComponent, page.cameraRay_);
    if (!pick.hit_ || pick.source_ == 0)
        return;
    Node* brushNode = page.scene_->GetNode(pick.source_);
    if (!brushNode || brushNode->GetParent() != rootNode)
        return;

    // The rules of the stock selector: Ctrl toggles, Shift adds, a plain click replaces.
    const bool toggle = ui::IsKeyDown(KEY_LCTRL) || ui::IsKeyDown(KEY_RCTRL);
    const bool append = ui::IsKeyDown(KEY_LSHIFT) || ui::IsKeyDown(KEY_RSHIFT);
    SceneSelection& selection = page.selection_;
    selection.ConvertToNodes();
    if (toggle)
        selection.SetSelected(brushNode, !selection.IsSelected(brushNode));
    else if (append)
        selection.SetSelected(brushNode, true);
    else
    {
        selection.Clear();
        selection.SetSelected(brushNode, true);
    }
    mouseConsumed = true;

    // A plain click on a shape brush also selects the face handle. The next frame applies it.
    pendingFace_.reset();
    if (!toggle && !append)
    {
        if (Component* shapeBrush = FindShapeBrush(brushNode))
            pendingFace_ = PendingFace{WeakPtr<Component>(shapeBrush), pick.position_, pick.normal_};
    }
}

void CsgEditorAddon::RenderOperandOverlay(SceneViewPage& page, Node* rootNode)
{
    const Rect& area = page.contentArea_;
    if (area.Size().x_ < OverlayWidth + 2.0f * OverlayMargin)
        return;

    // The overlay is in the top right corner. The debug HUD uses the top left corner.
    ui::SetCursorScreenPos(ImVec2{area.max_.x_ - OverlayWidth - OverlayMargin, area.min_.y_ + OverlayMargin});
    ui::PushStyleColor(ImGuiCol_ChildBg, ImVec4{0.0f, 0.0f, 0.0f, 0.6f});
    // The height follows the list up to the maximum. A longer list scrolls.
    const float maxHeight = ea::max(ui::GetFrameHeight() * 3.0f, area.Size().y_ * OverlayMaxHeightFraction);
    ui::SetNextWindowSizeConstraints(ImVec2{OverlayWidth, 0.0f}, ImVec2{OverlayWidth, maxHeight});
    const ImGuiChildFlags childFlags = ImGuiChildFlags_Border | ImGuiChildFlags_AutoResizeY;
    if (ui::BeginChild("##CsgOperands", ImVec2{OverlayWidth, 0.0f}, childFlags))
    {
        ui::TextUnformatted(Format("{} {}", ICON_FA_CUBES, rootNode->GetName()).c_str());
        ui::TextWrapped("%s", GetBuildStatus(rootNode).c_str());
        ui::Separator();

        Node* activeNode = page.selection_.GetActiveNode();
        unsigned index = 0;
        for (Node* brushNode : GetAllBrushChildren(rootNode))
        {
            Component* brush = FindBrushComponent(brushNode);
            const bool enabled = brush && brush->IsEnabled();
            const ea::string& name = brushNode->GetName();

            ui::TextColored(ToImGui(GetOperationColor(GetOperation(brushNode))), "%s", ICON_FA_SQUARE);
            ui::SameLine();
            // The ID suffix keeps the label unique for nodes with the same name.
            const ea::string label = Format("{}. {}{}##{}", index, name.empty() ? "(no name)" : name.c_str(),
                enabled ? "" : " (off)", brushNode->GetID());
            // A disabled brush has dim text. It stays selectable, so H can enable it again.
            if (!enabled)
                ui::PushStyleColor(ImGuiCol_Text, ui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            const bool clicked = ui::Selectable(label.c_str(), brushNode == activeNode);
            if (!enabled)
                ui::PopStyleColor();
            if (clicked)
            {
                page.selection_.Clear();
                page.selection_.SetSelected(brushNode, true);
            }
            ++index;
        }
        if (index == 0)
            ui::TextDisabled("No brushes");

        if (handleBrush_)
            RenderShapeSection();
    }
    ui::EndChild();
    ui::PopStyleColor();

    overlayRect_ = Rect{ToVector2(ui::GetItemRectMin()), ToVector2(ui::GetItemRectMax())};
    overlayVisible_ = true;
}

bool CsgEditorAddon::RenderTabContextMenu()
{
    if (!IsAvailable())
        return false;

    if (!ui::BeginMenu("CSG"))
        return true;

    auto hotkeyManager = owner_->GetHotkeyManager();
    if (ui::MenuItem("Add Union Brush", hotkeyManager->GetHotkeyLabel(Hotkey_AddUnion).c_str()))
        AddBrush(FindOperation(UnionName), false);
    if (ui::MenuItem("Add Subtract Brush", hotkeyManager->GetHotkeyLabel(Hotkey_AddSubtract).c_str()))
        AddBrush(FindOperation(DifferenceName), false);

    ui::Separator();

    SceneViewPage* page = owner_->GetActivePage();
    const bool hasBrushes = page && !GetSelectedBrushes(*page).empty();
    if (ui::MenuItem("Toggle Brush", hotkeyManager->GetHotkeyLabel(Hotkey_ToggleBrush).c_str(), false, hasBrushes))
        ToggleSelectedBrushes();
    if (ui::MenuItem("Move Brush Earlier", hotkeyManager->GetHotkeyLabel(Hotkey_MoveEarlier).c_str(), false, hasBrushes))
        MoveActiveBrush(-1);
    if (ui::MenuItem("Move Brush Later", hotkeyManager->GetHotkeyLabel(Hotkey_MoveLater).c_str(), false, hasBrushes))
        MoveActiveBrush(1);

    ui::Separator();

    const int count = OperationCount(context_);
    for (int index = 0; index < count; ++index)
    {
        const ea::string name = OperationName(context_, index);
        const EditorHotkey* hotkey = GetOperationHotkey(name);
        const ea::string label = hotkey ? hotkeyManager->GetHotkeyLabel(*hotkey) : ea::string{};
        if (ui::MenuItem(Format("Set {}", name).c_str(), label.c_str(), false, hasBrushes))
            SetSelectedOperation(index);
    }

    ui::Separator();
    RenderShapeSettings();

    ui::EndMenu();
    return true;
}

void CsgEditorAddon::SetActiveShapeBrush(Component* brush)
{
    const bool wasDragging = dragState_ != HandleDragState::None;
    if (dragState_ == HandleDragState::Active)
        EndHandleDrag(false);
    dragState_ = HandleDragState::None;
    // The gizmo can still be in use. The mouse stays consumed until the release, so the node gizmo does not take it.
    if (wasDragging && ui::IsMouseDown(MOUSEB_LEFT))
        handleClickPending_ = true;
    handleBrush_ = brush;
    handleSet_ = {};
    selectedHandles_.clear();
    hoveredHandle_.reset();
}

void CsgEditorAddon::BeginHandleDrag()
{
    dragSnapshot_ = SnapshotShapeAttributes(handleBrush_);
    dragStartPosition_ = handleBrush_->GetNode()->GetPosition();
    dragStartHandles_ = handleSet_.handles_;
    dragWorldDelta_ = Vector3::ZERO;
    dragTriedDelta_.reset();
    dragAcceptedDelta_.reset();
    dragState_ = HandleDragState::Active;
}

void CsgEditorAddon::UpdateHandleDrag(Node* node, bool snap)
{
    Component* brush = handleBrush_;
    if (!brush || selectedHandles_.empty())
        return;

    const ShapeHandle* primary = nullptr;
    for (const ShapeHandle& handle : dragStartHandles_)
    {
        if (handle.id_ == selectedHandles_.front())
            primary = &handle;
    }
    const bool isAxis = primary && primary->kind_ == CsgShapeHandleKind::Axis;

    // The world delta goes to the local space as a direction. The node and parent scale apply.
    Vector3 localDelta;
    if (isAxis)
    {
        // An axis handle moves only along its axis line in the view. The world delta is projected on the world axis.
        const Vector3 worldAxis = node->LocalToWorld(Vector4(primary->axis_, 0.0f));
        localDelta = node->WorldToLocal(Vector4(ProjectDeltaOnAxis(dragWorldDelta_, worldAxis), 0.0f));
    }
    else
        localDelta = node->WorldToLocal(Vector4(dragWorldDelta_, 0.0f));

    // Ctrl snaps the local position of the primary handle to the grid. An axis handle snaps only along the axis.
    if (snap && primary)
        localDelta = SnapHandleDelta(primary->position_, localDelta, isAxis ? primary->axis_ : Vector3::ZERO, handleGridStep_);

    if (dragTriedDelta_ && *dragTriedDelta_ == localDelta)
        return;
    dragTriedDelta_ = localDelta;

    // The move is stateless: restore the snapshot, then move by the total delta.
    // A refused move keeps the last accepted state. A zero delta gives the start shape.
    ApplyShapeHandleDrag(brush, dragSnapshot_, selectedHandles_, localDelta, dragAcceptedDelta_);
}

bool CsgEditorAddon::RecenterActiveShape()
{
    Component* brush = handleBrush_;
    if (!brush || dragState_ != HandleDragState::None)
        return false;

    const ShapeEditState oldState = CaptureShapeEditState(brush);
    if (!RecenterShapePivot(brush))
        return false;

    const auto action = MakeShared<ShapeHandleDragAction>(brush, oldState, CaptureShapeEditState(brush));
    if (Node* rootNode = FindActiveRoot(brush->GetNode()))
    {
        RequestRebuild(rootNode->GetScene(), rootNode->GetID());
        owner_->PushAction(MakeShared<RebuildRootActionWrapper>(action, rootNode));
    }
    else
        owner_->PushAction(action);
    return true;
}

void CsgEditorAddon::EndHandleDrag(bool commit)
{
    Component* brush = handleBrush_;
    if (brush && dragState_ == HandleDragState::Active)
    {
        if (!commit)
            RestoreShapeAttributes(brush, dragSnapshot_);
        else
        {
            // The drag is over and it changed the shape. The recenter is a part of the same undo step.
            if (SnapshotShapeAttributes(brush) != dragSnapshot_)
            {
                if (autoRecenter_)
                    RecenterShapePivot(brush);
                ShapeEditState oldState{dragSnapshot_, dragStartPosition_};
                const auto action =
                    MakeShared<ShapeHandleDragAction>(brush, ea::move(oldState), CaptureShapeEditState(brush));
                if (Node* rootNode = FindActiveRoot(brush->GetNode()))
                    owner_->PushAction(MakeShared<RebuildRootActionWrapper>(action, rootNode));
                else
                    owner_->PushAction(action);
            }
        }
    }
    dragState_ = HandleDragState::None;
    dragSnapshot_.clear();
    dragStartHandles_.clear();
}

void CsgEditorAddon::ProcessShapeHandles(SceneViewPage& page, bool& mouseConsumed)
{
    // On entry the camera, the overlay or a drop can own the mouse. Then the handles only draw.
    const bool consumedOnEntry = mouseConsumed;
    const auto dropPendingFace = [this](Component* activeBrush)
    {
        if (pendingFace_ && pendingFace_->brush_ != activeBrush)
            pendingFace_.reset();
    };
    // The content area is the last item. The scene selector uses the same check.
    const bool viewHovered = ui::IsItemHovered();

    // The active shape brush: exactly one selected node with a procedural brush.
    Component* brush = nullptr;
    if (IsAvailable() && !page.IsSimulationActive() && page.selection_.GetNodes().size() == 1)
    {
        for (Node* node : page.selection_.GetNodes())
            brush = FindShapeBrush(node);
    }
    if (brush != handleBrush_.Get())
        SetActiveShapeBrush(brush);

    const auto finish = [&]
    {
        if (handleClickPending_)
        {
            mouseConsumed = true;
            if (!ui::IsMouseDown(MOUSEB_LEFT))
                handleClickPending_ = false;
        }
    };

    if (!brush)
    {
        pendingFace_.reset();
        finish();
        return;
    }

    handleSet_ = QueryShapeHandles(brush);
    dropPendingFace(brush);
    if (pendingFace_ && showHandles_)
    {
        // The hit of the click chooses the face. No match keeps the selection as it is.
        if (const auto id = ChooseFaceHandleForWorldHit(
                brush->GetNode(), handleSet_.handles_, pendingFace_->worldPoint_, pendingFace_->worldNormal_))
            selectedHandles_ = {*id};
    }
    pendingFace_.reset();
    if (!showHandles_)
    {
        if (dragState_ == HandleDragState::Active)
            EndHandleDrag(false);
        selectedHandles_.clear();
        hoveredHandle_.reset();
        finish();
        return;
    }

    const auto findHandle = [this](unsigned id) -> const ShapeHandle*
    {
        for (const ShapeHandle& handle : handleSet_.handles_)
        {
            if (handle.id_ == id)
                return &handle;
        }
        return nullptr;
    };

    // A shape can drop handles after a change, for example the cylinder at an apex. A drag keeps its ids.
    if (dragState_ == HandleDragState::None)
        ea::erase_if(selectedHandles_, [&](unsigned id) { return !findHandle(id); });

    if (ui::IsKeyPressed(KEY_ESCAPE) && (viewHovered || ui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)))
    {
        if (dragState_ == HandleDragState::Active)
        {
            EndHandleDrag(false);
            dragState_ = HandleDragState::Cancelled;
        }
        else if (dragState_ == HandleDragState::None)
            selectedHandles_.clear();
    }

    // Ctrl+Z, Ctrl+Y and Ctrl+Shift+Z during a drag cancel the drag. Ctrl can be down for the snap.
    // The project applies the hotkeys after the tabs render, so the cancel comes before the undo or redo.
    // Then the undo stack and the brush stay in sync.
    if (dragState_ == HandleDragState::Active && ui::IsKeyDown(KEY_CTRL)
        && (ui::IsKeyPressed(KEY_Z) || ui::IsKeyPressed(KEY_Y)))
    {
        EndHandleDrag(false);
        dragState_ = HandleDragState::Cancelled;
    }

    Node* node = brush->GetNode();
    Camera* camera = page.renderer_->GetCamera();
    const Rect& area = page.contentArea_;
    const Matrix3x4& worldTransform = node->GetWorldTransform();
    const Vector2 mousePos = ToVector2(ui::GetMousePos());

    ea::vector<HandleScreenPoint> points;
    for (const ShapeHandle& handle : handleSet_.handles_)
    {
        const Vector3 worldPos = worldTransform * handle.position_;
        const auto screenPos = ProjectToView(camera, area, worldPos);
        const float depth = (camera->GetView() * worldPos).z_;
        points.push_back(HandleScreenPoint{handle.id_, screenPos.value_or(Vector2::ZERO), screenPos.has_value(), depth});
    }

    // No pick during the camera navigation (the right mouse button) and during a drag.
    const bool canPick =
        !consumedOnEntry && viewHovered && !ui::IsMouseDown(MOUSEB_RIGHT) && dragState_ == HandleDragState::None;
    hoveredHandle_.reset();
    if (canPick)
    {
        if (const auto index = PickNearestHandle(points, mousePos, HandlePickRadius))
            hoveredHandle_ = points[*index].id_;
    }

    // The translate gizmo of the selected handles.
    bool gizmoActive = false;
    const bool runGizmo = !selectedHandles_.empty()
        && (dragState_ != HandleDragState::None || (!consumedOnEntry && !ui::IsMouseDown(MOUSEB_RIGHT)));
    if (runGizmo)
    {
        const auto centroid = ComputeSelectionCentroid(handleSet_.handles_, selectedHandles_)
            .value_or(ComputeSelectionCentroid(dragStartHandles_, selectedHandles_).value_or(Vector3::ZERO));
        const bool local = IsTransformGizmoLocal();
        const Quaternion rotation = local ? node->GetWorldRotation() : Quaternion::IDENTITY;
        const Matrix4 transform = Matrix3x4(worldTransform * centroid, rotation, Vector3::ONE).ToMatrix4();
        const TransformGizmoAxes axes = TransformGizmoAxis::X | TransformGizmoAxis::Y | TransformGizmoAxis::Z;

        // The gizmo returns the world delta of this frame. The drag sums it from the drag start.
        const TransformGizmo gizmo{camera, area};
        const auto frameDelta = gizmo.ManipulatePosition(transform, axes, local, Vector3::ZERO);
        const bool inUse = ImGuizmo::IsUsing();
        gizmoActive = inUse || ImGuizmo::IsOver();
        gizmoHot_ = gizmoActive;

        if (inUse && dragState_ == HandleDragState::None)
            BeginHandleDrag();
        if (dragState_ == HandleDragState::Active && inUse)
        {
            if (frameDelta)
                dragWorldDelta_ += *frameDelta;
            UpdateHandleDrag(node, ui::IsKeyDown(KEY_CTRL));
        }
        if (dragState_ != HandleDragState::None && !inUse)
        {
            if (dragState_ == HandleDragState::Active)
                EndHandleDrag(true);
            dragState_ = HandleDragState::None;
        }
    }

    // The pick. A click on a handle is consumed. A click on no handle clears the selection and goes to the scene.
    if (canPick && !gizmoActive && ui::IsMouseClicked(MOUSEB_LEFT))
    {
        const ShapeHandle* handle = hoveredHandle_ ? findHandle(*hoveredHandle_) : nullptr;
        if (!handle)
            selectedHandles_.clear();
        else
        {
            const bool shift = ui::IsKeyDown(KEY_LSHIFT) || ui::IsKeyDown(KEY_RSHIFT);
            const bool hasAxis = ea::any_of(selectedHandles_.begin(), selectedHandles_.end(),
                [&](unsigned id)
                {
                    const ShapeHandle* selected = findHandle(id);
                    return selected && selected->kind_ == CsgShapeHandleKind::Axis;
                });
            // An axis handle is always single. Shift toggles a point handle among the selected points.
            if (handle->kind_ == CsgShapeHandleKind::Axis || !shift || hasAxis)
                selectedHandles_ = {handle->id_};
            else if (auto iter = ea::find(selectedHandles_.begin(), selectedHandles_.end(), handle->id_);
                iter != selectedHandles_.end())
                selectedHandles_.erase(iter);
            else
                selectedHandles_.push_back(handle->id_);
            mouseConsumed = true;
            handleClickPending_ = true;
        }
    }

    // The handles draw over the view image. The debug wireframe is in the image, so the handles are above it.
    ImDrawList* drawList = ui::GetWindowDrawList();
    drawList->PushClipRect(ToImGui(area.min_), ToImGui(area.max_), true);
    const float half = HandleSize * 0.5f;
    for (unsigned index = 0; index < handleSet_.handles_.size(); ++index)
    {
        const ShapeHandle& handle = handleSet_.handles_[index];
        if (!points[index].visible_)
            continue;

        const bool selected = ea::find(selectedHandles_.begin(), selectedHandles_.end(), handle.id_) != selectedHandles_.end();
        const bool hovered = hoveredHandle_ == handle.id_;
        const bool isAxis = handle.kind_ == CsgShapeHandleKind::Axis;
        const ImU32 color = selected ? HandleSelectedColor
            : hovered               ? HandleHoveredColor
            : isAxis                ? HandleAxisColor
                                    : HandlePointColor;
        const ImVec2 center = ToImGui(points[index].position_);

        if (isAxis)
        {
            const Vector3 worldPos = worldTransform * handle.position_;
            const Vector3 worldAxis = node->LocalToWorld(Vector4(handle.axis_, 0.0f));
            if (const auto direction = ProjectAxisToView(camera, area, worldPos, worldAxis))
            {
                // A selected axis handle shows its axis line through the view: the drag moves along this line.
                if (selected)
                {
                    drawList->AddLine(ToImGui(points[index].position_ - *direction * HandleAxisGuideLength),
                        ToImGui(points[index].position_ + *direction * HandleAxisGuideLength), color, 1.0f);
                }
                drawList->AddLine(center, ToImGui(points[index].position_ + *direction * HandleAxisLineLength),
                    color, 2.0f);
            }
            drawList->AddCircleFilled(center, half + 1.0f, HandleOutlineColor);
            drawList->AddCircleFilled(center, half, color);
        }
        else
        {
            drawList->AddRectFilled(ImVec2{center.x - half - 1.0f, center.y - half - 1.0f},
                ImVec2{center.x + half + 1.0f, center.y + half + 1.0f}, HandleOutlineColor);
            drawList->AddRectFilled(ImVec2{center.x - half, center.y - half}, ImVec2{center.x + half, center.y + half}, color);
        }
    }
    drawList->PopClipRect();

    if (hoveredHandle_ && !gizmoActive)
    {
        if (const ShapeHandle* handle = findHandle(*hoveredHandle_))
            ui::SetTooltip("%s", handle->name_.c_str());
    }

    // While handles are selected, the handles own the mouse. Then the gizmo of the node does not run.
    if (gizmoActive || !selectedHandles_.empty())
        mouseConsumed = true;
    finish();
}

void CsgEditorAddon::RenderShapeSection()
{
    Component* brush = handleBrush_;
    if (!brush)
        return;

    ui::Separator();
    ui::TextUnformatted(Format("{} Shape: {}", ICON_FA_SHAPES, brush->GetTypeName()).c_str());
    if (handleSet_.valid_)
        ui::TextUnformatted("Valid");
    else
        ui::TextColored(ImVec4{1.0f, 0.35f, 0.35f, 1.0f}, "Invalid: parameters are not valid");

    ui::Text("Selected handles: %u", static_cast<unsigned>(selectedHandles_.size()));
    if (!selectedHandles_.empty())
    {
        for (const ShapeHandle& handle : handleSet_.handles_)
        {
            if (handle.id_ != selectedHandles_.front())
                continue;
            if (selectedHandles_.size() == 1)
                ui::TextWrapped("%s", handle.name_.c_str());
            else
                ui::TextWrapped("%s (+%u)", handle.name_.c_str(), static_cast<unsigned>(selectedHandles_.size() - 1));
        }
    }
    RenderShapeSettings();
}

void CsgEditorAddon::RenderShapeSettings()
{
    ui::Checkbox("Show handles", &showHandles_);
    ui::Checkbox("Recenter pivot after drag", &autoRecenter_);
    if (ui::IsItemHovered())
        ui::SetTooltip("Move the node origin to the center of the shape at the end of a handle drag.");
    if (ui::MenuItem("Recenter pivot", nullptr, false, handleBrush_ && dragState_ == HandleDragState::None))
        RecenterActiveShape();
    ui::SetNextItemWidth(ui::CalcTextSize("0.0000").x + ui::GetStyle().FramePadding.x * 2.0f + 20.0f);
    if (ui::DragFloat("Snap step", &handleGridStep_, 0.005f, 0.0f, 100.0f, "%.3f"))
        handleGridStep_ = ea::max(0.0f, handleGridStep_);
    if (ui::IsItemHovered())
        ui::SetTooltip("The grid step of the Ctrl snap of a handle drag. 0 turns off the snap.");
    ui::TextDisabled("Click a face to select its brush");
    if (ui::IsItemHovered())
        ui::SetTooltip("Click a face to select its brush (Alt+click: select the root)");
}

} // namespace Urho3D
