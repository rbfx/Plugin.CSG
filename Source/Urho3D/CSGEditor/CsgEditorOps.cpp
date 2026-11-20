// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSGEditor/CsgEditorOps.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Format.h>
#include <Urho3D/Core/ObjectReflection.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/IO/VirtualFileSystem.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Resource/XMLElement.h>
#include <Urho3D/Resource/XMLFile.h>

#include <fstream>
#include <Urho3D/Scene/Component.h>
#include <Urho3D/Scene/Node.h>

namespace Urho3D
{

namespace
{

const StringHash rootType("CsgRoot");
const StringHash brushType("CsgBrush");
const StringHash rebuildEvent("CsgRequestRebuild");

Component* GetRootComponent(Node* node) { return node ? node->GetComponent(rootType) : nullptr; }

Component* GetBrushComponent(Node* node) { return FindBrushComponent(node); }

bool IsRootRegistered(Context* context) { return context && context->GetReflection(rootType) != nullptr; }

const AttributeInfo* FindOperationAttribute(Context* context)
{
    return context ? context->GetAttribute(brushType, "Operation") : nullptr;
}

ResourceRef MakeModelRef(const ea::string& modelName) { return ResourceRef(Model::GetTypeStatic(), modelName); }

} // namespace

Component* FindBrushComponent(Node* node)
{
    if (!node)
        return nullptr;
    // A type that derives from CsgBrush is also a brush.
    for (Component* component : node->GetComponents())
    {
        if (component && component->GetTypeInfo()->IsTypeOf(brushType))
            return component;
    }
    return nullptr;
}

bool HasModelAttribute(Component* component)
{
    const ea::vector<AttributeInfo>* attributes = component ? component->GetAttributes() : nullptr;
    if (!attributes)
        return false;
    for (const AttributeInfo& info : *attributes)
    {
        if (info.name_ == "Model")
            return true;
    }
    return false;
}

bool IsModelBrushNode(Node* node) { return HasModelAttribute(GetBrushComponent(node)); }

Node* FindActiveRoot(Node* selected)
{
    if (!selected)
        return nullptr;
    // A temporary node is the result node of a root. It maps to its parent.
    if (selected->IsTemporary())
        selected = selected->GetParent();
    if (!selected)
        return nullptr;
    if (GetRootComponent(selected))
        return selected;
    Node* parent = selected->GetParent();
    return GetRootComponent(parent) ? parent : nullptr;
}

bool IsBrushNode(Node* node) { return GetBrushComponent(node) != nullptr; }

bool IsRootNode(Node* node) { return GetRootComponent(node) != nullptr; }

Node* CreateRootNode(Node* parent, const Vector3& worldPosition)
{
    if (!parent || !IsRootRegistered(parent->GetContext()))
        return nullptr;
    Node* node = parent->CreateChild("CSG");
    node->SetWorldPosition(worldPosition);
    if (!node->CreateComponent(rootType))
    {
        node->Remove();
        return nullptr;
    }
    return node;
}

namespace
{

/// Create the brush node. A null model name means a brush without a "Model" attribute.
Node* AddBrushNodeInternal(Node* rootNode, StringHash type, const ea::string* modelName, int operation,
    const Vector3& worldPosition, bool enabled, void (*configure)(Component*) = nullptr)
{
    if (!IsRootNode(rootNode))
        return nullptr;

    // The node gets its brush, attributes and transform before it goes under the root.
    // Then an enabled brush registers once and requests one rebuild. A disabled brush requests none.
    auto node = MakeShared<Node>(rootNode->GetContext());
    node->SetName("Brush");
    Component* brush = node->CreateComponent(type);
    if (!brush)
        return nullptr;
    if (!enabled)
        brush->SetEnabled(false);
    if (modelName)
        brush->SetAttribute("Model", MakeModelRef(*modelName));
    brush->SetAttribute("Operation", operation);
    brush->ApplyAttributes();
    // The first build then has the shape of the preset.
    if (configure)
        configure(brush);

    // The world position is correct for a root with any transform.
    node->SetPosition(rootNode->WorldToLocal(worldPosition));
    // The inverse of the root world scale gives the brush a world scale of about 1.
    // The local rotation stays identity, so the root scale cancels.
    // A rotated root with a non-uniform scale can shear the brush when the brush rotates later.
    const Vector3 rootScale = rootNode->GetWorldScale();
    const auto inverse = [](float value) { return Abs(value) > M_EPSILON ? 1.0f / value : 1.0f; };
    node->SetScale(Vector3(inverse(rootScale.x_), inverse(rootScale.y_), inverse(rootScale.z_)));

    rootNode->AddChild(node);
    return node;
}

/// The editor does not link the CSG library, so the configure functions use the attributes of the brush.
/// The cone: the cylinder with the top radius (0, 0). The side is smooth, as in the cylinder.
void ConfigureCone(Component* brush)
{
    brush->SetAttribute("Top Radius", Vector2::ZERO);
    brush->ApplyAttributes();
}

/// The pyramid: 4 sides, the first corner at 45 degrees and the radius at the middle of each face.
/// The base fits a box of the size (2 * radius x, 2 * radius z). The faces are flat.
void ConfigurePyramid(Component* brush)
{
    brush->SetAttribute("Segments", 4);
    brush->SetAttribute("Start Angle", 45.0f);
    brush->SetAttribute("Face Radius", true);
    brush->SetAttribute("Top Radius", Vector2::ZERO);
    brush->SetAttribute("Smooth Normals", false);
    brush->ApplyAttributes();
}

/// The hex prism: 6 flat sides.
void ConfigureHexPrism(Component* brush)
{
    brush->SetAttribute("Segments", 6);
    brush->SetAttribute("Smooth Normals", false);
    brush->ApplyAttributes();
}

/// The procedural box, sphere, cylinder, cone, pyramid, hex prism, ring, wedge, stairs and arch are first. The model presets use the CoreData models.
/// Tests.CSG checks that CSG accepts each model as a closed mesh.
const CsgBrushPreset BrushPresets[] = {
    {"Box", nullptr, "CsgBoxBrush"},
    {"Sphere", nullptr, "CsgSphereBrush"},
    {"Cylinder", nullptr, "CsgCylinderBrush"},
    {"Cone", nullptr, "CsgCylinderBrush", ConfigureCone},
    {"Pyramid", nullptr, "CsgCylinderBrush", ConfigurePyramid},
    {"Hex Prism", nullptr, "CsgCylinderBrush", ConfigureHexPrism},
    {"Ring", nullptr, "CsgRingBrush"},
    {"Wedge", nullptr, "CsgWedgeBrush"},
    {"Stairs", nullptr, "CsgStairsBrush"},
    {"Arch", nullptr, "CsgArchBrush"},
    {"Box.mdl", "Models/Box.mdl", nullptr},
    {"Sphere.mdl", "Models/Sphere.mdl", nullptr},
    {"Cylinder.mdl", "Models/Cylinder.mdl", nullptr},
    {"Cone.mdl", "Models/Cone.mdl", nullptr},
    {"Pyramid.mdl", "Models/Pyramid.mdl", nullptr},
    {"Torus.mdl", "Models/Torus.mdl", nullptr},
};

} // namespace

Node* AddBrushNode(
    Node* rootNode, const ea::string& modelName, int operation, const Vector3& worldPosition, bool enabled)
{
    return AddBrushNodeInternal(rootNode, brushType, &modelName, operation, worldPosition, enabled);
}

bool IsBrushTypeRegistered(Context* context, const ea::string& typeName)
{
    const ObjectReflection* reflection = context && !typeName.empty() ? context->GetReflection(StringHash(typeName)) : nullptr;
    return reflection && reflection->HasObjectFactory() && reflection->GetTypeInfo()
        && reflection->GetTypeInfo()->IsTypeOf(brushType);
}

Node* AddBrushNodeOfType(
    Node* rootNode, const ea::string& typeName, int operation, const Vector3& worldPosition, bool enabled)
{
    // The check comes first: the factory logs an error for an unknown type.
    if (!rootNode || !IsBrushTypeRegistered(rootNode->GetContext(), typeName))
        return nullptr;
    return AddBrushNodeInternal(rootNode, StringHash(typeName), nullptr, operation, worldPosition, enabled);
}

ea::span<const CsgBrushPreset> GetBrushPresets() { return BrushPresets; }

bool IsBrushPresetAvailable(Context* context, const CsgBrushPreset& preset)
{
    if (preset.typeName_)
        return IsBrushTypeRegistered(context, preset.typeName_);
    return context && context->GetReflection(brushType) != nullptr;
}

Node* AddPresetBrushNode(
    Node* rootNode, const CsgBrushPreset& preset, int operation, const Vector3& worldPosition, bool enabled)
{
    if (preset.typeName_)
    {
        // The check comes first: the factory logs an error for an unknown type.
        if (!rootNode || !IsBrushTypeRegistered(rootNode->GetContext(), preset.typeName_))
            return nullptr;
        return AddBrushNodeInternal(
            rootNode, StringHash(preset.typeName_), nullptr, operation, worldPosition, enabled, preset.configure_);
    }
    return AddBrushNode(rootNode, preset.model_ ? preset.model_ : "", operation, worldPosition, enabled);
}

int GetOperation(Node* brush)
{
    Component* component = GetBrushComponent(brush);
    return component ? component->GetAttribute("Operation").GetInt() : -1;
}

bool SetOperation(Node* brush, int operation)
{
    Component* component = GetBrushComponent(brush);
    if (!component || operation < 0 || operation >= OperationCount(brush->GetContext()))
        return false;
    if (!component->SetAttribute("Operation", operation))
        return false;
    component->ApplyAttributes();
    return true;
}

int OperationCount(Context* context)
{
    const AttributeInfo* info = FindOperationAttribute(context);
    return info ? static_cast<int>(info->enumNames_.size()) : 0;
}

ea::string OperationName(Context* context, int operation)
{
    const AttributeInfo* info = FindOperationAttribute(context);
    if (!info || operation < 0 || operation >= static_cast<int>(info->enumNames_.size()))
        return EMPTY_STRING;
    return info->enumNames_[operation];
}

int CycleOperation(Context* context, int current, int step)
{
    const int count = OperationCount(context);
    if (count <= 0)
        return current;
    return ((current + step) % count + count) % count;
}

bool SetBrushModel(Node* brush, const ea::string& modelName)
{
    // A brush with a generated model has no "Model" attribute. It is not a target.
    Component* component = GetBrushComponent(brush);
    if (!HasModelAttribute(component) || !component->SetAttribute("Model", MakeModelRef(modelName)))
        return false;
    component->ApplyAttributes();
    return true;
}

namespace
{

/// Read the list of the attribute "Materials". Return false if the brush has none.
bool ReadMaterialList(Component* component, ResourceRefList& list)
{
    if (!component)
        return false;
    const Variant value = component->GetAttribute("Materials");
    if (value.GetType() != VAR_RESOURCEREFLIST)
        return false;
    list = value.GetResourceRefList();
    list.type_ = Material::GetTypeStatic();
    return true;
}

bool WriteMaterialList(Component* component, const ResourceRefList& list)
{
    if (!component->SetAttribute("Materials", list))
        return false;
    component->ApplyAttributes();
    return true;
}

} // namespace

bool SetBrushMaterial(Node* brush, const ea::string& materialName, unsigned slot)
{
    Component* component = GetBrushComponent(brush);
    ResourceRefList list;
    if (!ReadMaterialList(component, list))
        return false;
    if (list.names_.empty() && slot == 0)
        list.names_.resize(1);
    if (slot >= list.names_.size())
        return false;
    list.names_[slot] = materialName;
    return WriteMaterialList(component, list);
}

bool SetAllBrushMaterialSlots(Node* brush, const ea::string& materialName)
{
    Component* component = GetBrushComponent(brush);
    ResourceRefList list;
    if (!ReadMaterialList(component, list))
        return false;
    if (list.names_.empty())
        list.names_.resize(1);
    for (ea::string& name : list.names_)
        name = materialName;
    return WriteMaterialList(component, list);
}

bool IsTextureResourceName(const ea::string& resourceName)
{
    const ea::string extension = GetExtension(resourceName);
    for (const char* known : {".png", ".jpg", ".jpeg", ".tga", ".dds", ".bmp", ".webp", ".ktx"})
    {
        if (extension == known)
            return true;
    }
    return false;
}

ea::string GetTextureMaterialName(const ea::string& textureResourceName)
{
    return textureResourceName + ".material";
}

ea::string EnsureTextureMaterial(Context* context, const ea::string& textureResourceName)
{
    auto cache = context ? context->GetSubsystem<ResourceCache>() : nullptr;
    if (!cache)
        return EMPTY_STRING;

    const ea::string materialName = GetTextureMaterialName(textureResourceName);
    // An existing material is never changed.
    if (cache->Exists(materialName))
        return materialName;

    // The file name of the texture shows the writable directory. A package has no file name.
    const ea::string textureFile = cache->GetResourceFileName(textureResourceName);
    if (textureFile.empty())
    {
        URHO3D_LOGERROR("Cannot make a material: texture \"{}\" is not in a resource directory", textureResourceName);
        return EMPTY_STRING;
    }

    XMLFile xml(context);
    XMLElement root = xml.CreateRoot("material");
    root.CreateChild("technique").SetAttribute("name", "Techniques/LitOpaque.xml");
    XMLElement shader = root.CreateChild("shader");
    shader.SetAttribute("vsdefines", "PBR");
    shader.SetAttribute("psdefines", "PBR");
    XMLElement texture = root.CreateChild("texture");
    texture.SetAttribute("slot", "Albedo");
    texture.SetAttribute("name", textureResourceName);

    const ea::string materialFile = textureFile + ".material";
    // A plain stream gives no log of its own. The function logs one error.
    const ea::string text = xml.ToString();
    bool saved = false;
    {
        std::ofstream stream(materialFile.c_str(), std::ios::binary);
        stream.write(text.c_str(), static_cast<std::streamsize>(text.size()));
        stream.flush();
        saved = stream.good();
    }
    if (!saved)
    {
        // A broken file must not stay. A later drop would reuse it.
        if (auto fileSystem = context->GetSubsystem<FileSystem>())
            fileSystem->Delete(materialFile);
        URHO3D_LOGERROR("Cannot write the material file \"{}\"", materialFile);
        return EMPTY_STRING;
    }
    return materialName;
}

bool IsResultNodeOfRoot(Node* node, Node* rootNode)
{
    return node && rootNode && node->IsTemporary() && node->GetParent() == rootNode && node->GetName() == "CsgResult";
}

DropDecision DecideDrop(DropPayloadKind kind, const DropSelection& selection, bool shift, bool ctrl)
{
    DropDecision result;
    switch (kind)
    {
    case DropPayloadKind::Model:
        if (selection.activeIsModelBrush_)
            result.action_ = DropAction::ReplaceModel;
        else if (!selection.activeIsBrush_ && selection.activeIsRoot_)
            result.action_ = DropAction::NewBrush;
        break;
    case DropPayloadKind::Material:
    case DropPayloadKind::Texture:
    {
        const DropAction assign =
            kind == DropPayloadKind::Material ? DropAction::AssignMaterial : DropAction::AssignTexture;
        result.scope_ = ctrl ? MaterialScope::Slot : shift ? MaterialScope::ShapeClearOverrides : MaterialScope::Shape;
        if (selection.hoveringResult_)
        {
            if (!selection.hoverBrush_)
            {
                result.action_ = DropAction::HintNoBrush;
                break;
            }
            result.action_ = assign;
            result.target_ = selection.hoverBrushSelected_ ? DropTarget::Selected : DropTarget::Hovered;
            break;
        }
        // The stock handler assigns a material to the geometry under the cursor.
        if (kind == DropPayloadKind::Material)
            break;
        if (selection.anySelectedBrush_)
            result.action_ = assign;
        else if (selection.activeIsRoot_)
            result.action_ = DropAction::HintSelectBrush;
        break;
    }
    case DropPayloadKind::Other: break;
    }
    return result;
}

bool PlanMaterialEdit(BrushMaterials& io, MaterialScope scope, unsigned slot, const ea::string& material)
{
    BrushMaterials plan = io;
    if (!plan.procedural_)
    {
        // A model brush has no shape material. A shape edit sets all geometry slots.
        if (plan.slots_.empty())
            plan.slots_.resize(1);
        if (scope == MaterialScope::Slot)
        {
            if (slot >= plan.slots_.size())
                return false;
            plan.slots_[slot] = material;
        }
        else
        {
            for (ea::string& name : plan.slots_)
                name = material;
        }
        io = ea::move(plan);
        return true;
    }

    if (scope == MaterialScope::Slot && slot >= plan.slots_.size())
        return false;

    // An old brush: slot 0 is the material of the whole shape. Move it, so a new edit keeps the render result.
    if (plan.shape_.empty() && !plan.slots_.empty() && !plan.slots_[0].empty())
    {
        plan.shape_ = plan.slots_[0];
        for (ea::string& name : plan.slots_)
        {
            if (name == plan.shape_)
                name.clear();
        }
    }

    switch (scope)
    {
    case MaterialScope::Shape: plan.shape_ = material; break;
    case MaterialScope::ShapeClearOverrides:
        plan.shape_ = material;
        for (ea::string& name : plan.slots_)
            name.clear();
        break;
    case MaterialScope::Slot: plan.slots_[slot] = material; break;
    }
    io = ea::move(plan);
    return true;
}

ea::vector<unsigned> GetOverriddenSlots(const BrushMaterials& materials)
{
    ea::vector<unsigned> result;
    if (!materials.procedural_)
        return result;
    const ea::string& shape = !materials.shape_.empty() ? materials.shape_
        : materials.slots_.empty()                    ? materials.shape_
                                                      : materials.slots_[0];
    for (unsigned i = 0; i < materials.slots_.size(); ++i)
    {
        if (!materials.slots_[i].empty() && materials.slots_[i] != shape)
            result.push_back(i);
    }
    return result;
}

ea::optional<unsigned> GetFaceSlotOfHandle(const ea::string& shapeTypeName, unsigned handleId)
{
    // The ids of the box are a file contract. See CsgBoxBrush.h.
    constexpr unsigned firstFaceHandle = 20;
    constexpr unsigned faceCount = 6;
    if (shapeTypeName == "CsgBoxBrush" && handleId >= firstFaceHandle && handleId < firstFaceHandle + faceCount)
        return handleId - firstFaceHandle;
    return ea::nullopt;
}

namespace
{
bool HasShapeMaterialAttribute(Component* component)
{
    return component && component->GetAttribute("Shape Material").GetType() == VAR_RESOURCEREF;
}
} // namespace

bool ReadBrushMaterials(Node* brush, BrushMaterials& materials)
{
    Component* component = GetBrushComponent(brush);
    ResourceRefList list;
    if (!ReadMaterialList(component, list))
        return false;
    materials = BrushMaterials{};
    materials.procedural_ = HasShapeMaterialAttribute(component);
    if (materials.procedural_)
        materials.shape_ = component->GetAttribute("Shape Material").GetResourceRef().name_;
    materials.slots_ = list.names_;
    return true;
}

bool EditBrushMaterial(Node* brush, MaterialScope scope, unsigned slot, const ea::string& material)
{
    Component* component = GetBrushComponent(brush);
    BrushMaterials before;
    if (!ReadBrushMaterials(brush, before))
        return false;
    BrushMaterials after = before;
    if (!PlanMaterialEdit(after, scope, slot, material))
        return false;

    bool changed = false;
    if (after.procedural_ && after.shape_ != before.shape_)
    {
        changed |= component->SetAttribute("Shape Material", ResourceRef(Material::GetTypeStatic(), after.shape_));
    }
    if (after.slots_ != before.slots_)
        changed |= component->SetAttribute("Materials", ResourceRefList(Material::GetTypeStatic(), after.slots_));
    if (changed)
        component->ApplyAttributes();
    return changed;
}

ea::vector<Node*> GetAllBrushChildren(Node* rootNode)
{
    ea::vector<Node*> result;
    if (!IsRootNode(rootNode))
        return result;
    for (Node* child : rootNode->GetChildren())
    {
        if (IsBrushNode(child))
            result.push_back(child);
    }
    return result;
}

ea::vector<Node*> GetOperands(Node* rootNode)
{
    ea::vector<Node*> result;
    for (Node* child : GetAllBrushChildren(rootNode))
    {
        if (GetBrushComponent(child)->IsEnabledEffective())
            result.push_back(child);
    }
    return result;
}

bool MoveOperand(Node* rootNode, unsigned fromIndex, unsigned toIndex)
{
    const ea::vector<Node*> brushes = GetAllBrushChildren(rootNode);
    if (fromIndex >= brushes.size() || toIndex >= brushes.size())
        return false;
    if (fromIndex == toIndex)
        return true;

    // Map the target brush to its index in the child list.
    const ea::vector<SharedPtr<Node>>& children = rootNode->GetChildren();
    unsigned realIndex = 0;
    while (realIndex < children.size() && children[realIndex] != brushes[toIndex])
        ++realIndex;
    if (realIndex >= children.size())
        return false;

    // ReorderChild sends no event. The event makes the root rebuild.
    rootNode->ReorderChild(brushes[fromIndex], realIndex);
    GetRootComponent(rootNode)->SendEvent(rebuildEvent);
    return true;
}

bool SetBrushEnabled(Node* brush, bool enabled)
{
    Component* component = GetBrushComponent(brush);
    if (!component)
        return false;
    component->SetEnabled(enabled);
    return true;
}

bool ToggleBrushEnabled(Node* brush)
{
    Component* component = GetBrushComponent(brush);
    if (!component)
        return false;
    // The brush registers with the root when its enabled state changes. The root rebuilds then.
    component->SetEnabled(!component->IsEnabled());
    return true;
}

ea::string DescribeRoot(Node* rootNode)
{
    Component* root = GetRootComponent(rootNode);
    if (!root)
        return EMPTY_STRING;

    Context* context = rootNode->GetContext();
    ea::string text;
    unsigned index = 0;
    for (Node* operand : GetOperands(rootNode))
    {
        Component* brush = GetBrushComponent(operand);
        const bool hasModel = HasModelAttribute(brush);
        const ea::string model = hasModel ? brush->GetAttribute("Model").GetResourceRef().name_ : EMPTY_STRING;
        const char* modelText =
            !hasModel ? brush->GetTypeName().c_str() : model.empty() ? "(no model)" : model.c_str();
        text += ea::string(ea::string::CtorSprintf(), "%u: %s | %s | %s\n", index++, operand->GetName().c_str(),
            OperationName(context, GetOperation(operand)).c_str(), modelText);
    }

    const Variant status = root->GetAttribute("Build Status");
    if (status.GetType() == VAR_STRING && !status.GetString().empty())
        text += status.GetString() + "\n";
    return text;
}

StaticModel* FindResultStaticModel(Node* rootNode)
{
    if (!rootNode)
        return nullptr;
    for (Node* child : rootNode->GetChildren())
    {
        if (IsResultNodeOfRoot(child, rootNode))
            return child->GetComponent<StaticModel>();
    }
    return nullptr;
}

BakedModelPath ChooseBakedModelPath(Context* context, ea::span<const ea::string> hintResourceNames,
    const ea::string& fallbackDirectory, const ea::string& rootNodeName)
{
    auto cache = context ? context->GetSubsystem<ResourceCache>() : nullptr;
    auto fileSystem = context ? context->GetSubsystem<FileSystem>() : nullptr;
    if (!cache || !fileSystem)
        return {};

    // The file name of a hint ends with its resource name. The rest is the resource directory.
    ea::string directory;
    for (const ea::string& hint : hintResourceNames)
    {
        if (hint.empty())
            continue;
        // The hint can also be an absolute file name, for example the file name of a scene.
        ea::string resourceName = hint;
        if (auto vfs = context->GetSubsystem<VirtualFileSystem>(); vfs && !cache->Exists(hint))
        {
            const FileIdentifier identifier = vfs->GetIdentifierFromAbsoluteName(hint);
            if (!identifier.fileName_.empty())
                resourceName = identifier.fileName_;
        }
        const ea::string file = cache->GetResourceFileName(resourceName);
        if (file.length() > resourceName.length() && file.ends_with(resourceName))
        {
            directory = file.substr(0, file.length() - resourceName.length());
            break;
        }
    }
    if (directory.empty() && !fallbackDirectory.empty() && fileSystem->DirExists(fallbackDirectory))
        directory = fallbackDirectory.ends_with("/") ? fallbackDirectory : fallbackDirectory + "/";
    if (directory.empty())
        return {};

    ea::string name;
    for (const char ch : rootNodeName)
    {
        const bool plain = (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
            || ch == '_' || ch == '-' || ch == '.';
        name += plain ? ch : '_';
    }
    if (name.empty())
        name = "CSG";

    for (unsigned index = 0;; ++index)
    {
        const ea::string suffix = index == 0 ? ea::string{} : Format("_{}", index);
        const ea::string resourceName = Format("Models/{}_Baked{}.mdl", name, suffix);
        const ea::string fileName = directory + resourceName;
        if (!fileSystem->FileExists(fileName) && !cache->Exists(resourceName))
            return {resourceName, fileName};
    }
}

bool SaveBakedModel(Context* context, Node* rootNode, const ea::string& fileName)
{
    StaticModel* result = FindResultStaticModel(rootNode);
    const Model* model = result ? result->GetModel() : nullptr;
    if (!model || model->GetNumGeometries() == 0)
    {
        URHO3D_LOGERROR("Cannot bake: the root has no result model");
        return false;
    }

    auto fileSystem = context->GetSubsystem<FileSystem>();
    fileSystem->CreateDirsRecursive(GetPath(fileName));
    bool saved = false;
    {
        File file(context, fileName, FILE_WRITE);
        saved = file.IsOpen() && model->Save(file);
    }
    if (!saved)
    {
        fileSystem->Delete(fileName);
        URHO3D_LOGERROR("Cannot write the model file \"{}\"", fileName);
    }
    return saved;
}

bool CanBakeNow(const RootStats& stats)
{
    return !stats.building_ && !stats.dirty_;
}

Node* CreateBakedNode(Node* rootNode, const ea::string& modelResourceName)
{
    Node* parent = rootNode ? rootNode->GetParent() : nullptr;
    StaticModel* result = FindResultStaticModel(rootNode);
    if (!parent || !result)
    {
        URHO3D_LOGERROR("Cannot make the baked node: the root has no parent or no result");
        return nullptr;
    }
    auto* cache = rootNode->GetSubsystem<ResourceCache>();
    // A cached model of the same name can be old, because a file name can be used again.
    cache->ReleaseResource<Model>(modelResourceName, true);
    auto* model = cache->GetResource<Model>(modelResourceName);
    if (!model)
        return nullptr;

    Node* baked = parent->CreateChild(rootNode->GetName() + " Baked");
    // The local transform keeps shear and non-uniform scale. A world decomposition loses them.
    baked->SetTransform(rootNode->GetPosition(), rootNode->GetRotation(), rootNode->GetScale());
    auto* staticModel = baked->CreateComponent<StaticModel>();
    staticModel->SetModel(model);
    staticModel->SetMaterialsAttr(result->GetMaterialsAttr());
    return baked;
}

} // namespace Urho3D
