// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSGEditor/CsgEditorAddon.h"

#include "Urho3D/CSG/CsgOperationColor.h"
#include "Urho3D/CSGEditor/CsgEditorOps.h"

#include <Editor/Core/CommonEditorActions.h>
#include <Editor/Core/HotkeyManager.h>
#include <Editor/Core/IniHelpers.h>

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
#include <Urho3D/SystemUI/Widgets.h>
#include <Urho3D/Utility/SceneRendererToTexture.h>

#include <IconFontCppHeaders/IconsFontAwesome6.h>

namespace Urho3D
{

namespace
{

/// A primitive model for new brushes.
struct BrushPreset
{
    const char* name_;
    const char* model_;
};

/// The presets use the CoreData models. Tests.CSG checks that CSG accepts each model as a closed mesh.
const BrushPreset BrushPresets[] = {
    {"Box", "Models/Box.mdl"},
    {"Sphere", "Models/Sphere.mdl"},
    {"Cylinder", "Models/Cylinder.mdl"},
    {"Cone", "Models/Cone.mdl"},
    {"Pyramid", "Models/Pyramid.mdl"},
    {"Torus", "Models/Torus.mdl"},
};
const unsigned NumBrushPresets = sizeof(BrushPresets) / sizeof(BrushPresets[0]);
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
const auto Hotkey_MoveEarlier = EditorHotkey{"Csg.MoveEarlier"}.Press(KEY_LEFTBRACKET);
const auto Hotkey_MoveLater = EditorHotkey{"Csg.MoveLater"}.Press(KEY_RIGHTBRACKET);
const auto Hotkey_ToggleBrush = EditorHotkey{"Csg.ToggleBrush"}.Press(KEY_H);

const StringHash RootType("CsgRoot");
const StringHash BrushType("CsgBrush");
const StringHash RebuildEvent("CsgRequestRebuild");

/// The width of the operand overlay in pixels and its distance from the view corner.
const float OverlayWidth = 260.0f;
const float OverlayMargin = 8.0f;

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

/// Return the first resource of the payload if it is a Model.
const ResourceFileDescriptor* GetModelResource(DragDropPayload* payload)
{
    const auto resourcePayload = dynamic_cast<ResourceDragDropPayload*>(payload);
    if (!resourcePayload || resourcePayload->resources_.empty())
        return nullptr;
    const ResourceFileDescriptor& desc = resourcePayload->resources_[0];
    return desc.HasObjectType<Model>() ? &desc : nullptr;
}

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

    Node* brushNode = AddBrushNode(rootNode, BrushPresets[presetIndex_].model_, operation, position);
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
        Component* component = brushNode->GetComponent(BrushType);
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
        Component* component = brushNode->GetComponent(BrushType);
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

void CsgEditorAddon::RenderPresetCombo()
{
    // The padding gives the combo the height of a toolbar button.
    const ImGuiStyle& style = ui::GetStyle();
    const float paddingY = ea::max(0.0f, (Widgets::GetSmallButtonSize() - ui::GetFontSize()) * 0.5f);
    ui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2{style.FramePadding.x, paddingY});
    ui::SetNextItemWidth(ui::CalcTextSize("Cylinder").x + style.FramePadding.x * 2.0f + ui::GetFrameHeight());

    if (ui::BeginCombo("##CsgBrushPreset", BrushPresets[presetIndex_].name_))
    {
        for (unsigned index = 0; index < NumBrushPresets; ++index)
        {
            const bool selected = index == presetIndex_;
            if (ui::Selectable(BrushPresets[index].name_, selected))
                presetIndex_ = index;
            if (selected)
                ui::SetItemDefaultFocus();
        }
        ui::EndCombo();
    }
    ui::PopStyleVar();

    if (ui::IsItemHovered())
        ui::SetTooltip("%s", Format("CSG brush preset: {}", BrushPresets[presetIndex_].model_).c_str());
    ui::SameLine(0, 0);
    Widgets::ToolbarSeparator();
}

void CsgEditorAddon::WriteIniSettings(ImGuiTextBuffer& output)
{
    WriteStringToIni(output, PresetIniKey, BrushPresets[presetIndex_].name_);
}

void CsgEditorAddon::ReadIniSettings(const char* line)
{
    // An unknown name keeps the current preset.
    if (const auto value = ReadStringFromIni(line, PresetIniKey))
    {
        for (unsigned index = 0; index < NumBrushPresets; ++index)
        {
            if (*value == BrushPresets[index].name_)
                presetIndex_ = index;
        }
    }
}

CsgEditorAddon::DropMode CsgEditorAddon::GetDropMode(SceneViewPage& page, DragDropPayload* payload) const
{
    if (!IsAvailable() || page.IsSimulationActive() || !GetModelResource(payload))
        return DropMode::None;

    Node* activeNode = page.selection_.GetActiveNode();
    if (!activeNode)
        return DropMode::None;
    if (IsBrushNode(activeNode))
        return DropMode::ReplaceModel;
    // The result node of a root is a temporary child. It maps to the root.
    if (IsRootNode(activeNode) || (activeNode->IsTemporary() && IsRootNode(activeNode->GetParent())))
        return DropMode::NewBrush;
    return DropMode::None;
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

    const ResourceFileDescriptor* desc = GetModelResource(payload);
    if (!desc)
        return;

    dropMode_ = GetDropMode(page, payload);
    dropPage_ = &page;
    Node* activeNode = page.selection_.GetActiveNode();

    if (dropMode_ == DropMode::ReplaceModel)
    {
        // The live preview sets the model now. CancelDragDrop restores the old models.
        for (Node* brushNode : GetSelectedBrushes(page))
        {
            Component* component = brushNode->GetComponent(BrushType);
            if (!component)
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
}

void CsgEditorAddon::Render(SceneViewPage& page)
{
    if (!IsAvailable())
        return;

    Node* rootNode = FindActiveRoot(page.selection_.GetActiveNode());
    if (!rootNode)
        return;

    // The tab creates the DebugRenderer of the scene before the addons render.
    if (auto debug = page.scene_->GetComponent<DebugRenderer>())
    {
        // DrawDebugGeometry is a Component virtual. The brush uses the color of its operation.
        for (Node* brushNode : GetOperands(rootNode))
        {
            if (Component* brush = brushNode->GetComponent(BrushType))
                brush->DrawDebugGeometry(debug, !page.selection_.IsSelected(brushNode));
        }
    }

    RenderOperandOverlay(page, rootNode);
}

void CsgEditorAddon::ProcessInput(SceneViewPage& page, bool& mouseConsumed)
{
    // The overlay rectangle is from the last frame. A click on the overlay must not select in the scene.
    if (overlayVisible_ && ui::IsMouseHoveringRect(ToImGui(overlayRect_.min_), ToImGui(overlayRect_.max_)))
        mouseConsumed = true;
    overlayVisible_ = false;
}

void CsgEditorAddon::RenderOperandOverlay(SceneViewPage& page, Node* rootNode)
{
    const Rect& area = page.contentArea_;
    if (area.Size().x_ < OverlayWidth + 2.0f * OverlayMargin)
        return;

    // The overlay is in the top right corner. The debug HUD uses the top left corner.
    ui::SetCursorScreenPos(ImVec2{area.max_.x_ - OverlayWidth - OverlayMargin, area.min_.y_ + OverlayMargin});
    ui::PushStyleColor(ImGuiCol_ChildBg, ImVec4{0.0f, 0.0f, 0.0f, 0.6f});
    const ImGuiChildFlags childFlags = ImGuiChildFlags_Border | ImGuiChildFlags_AutoResizeY;
    if (ui::BeginChild("##CsgOperands", ImVec2{OverlayWidth, 0.0f}, childFlags, ImGuiWindowFlags_NoScrollbar))
    {
        ui::TextUnformatted(Format("{} {}", ICON_FA_CUBES, rootNode->GetName()).c_str());
        ui::TextWrapped("%s", GetBuildStatus(rootNode).c_str());
        ui::Separator();

        Node* activeNode = page.selection_.GetActiveNode();
        unsigned index = 0;
        for (Node* brushNode : GetAllBrushChildren(rootNode))
        {
            Component* brush = brushNode->GetComponent(BrushType);
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

    ui::EndMenu();
    return true;
}

} // namespace Urho3D
