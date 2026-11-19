// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSGEditor/CsgEditorOps.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Model.h>
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

Component* GetBrushComponent(Node* node) { return node ? node->GetComponent(brushType) : nullptr; }

bool IsRootRegistered(Context* context) { return context && context->GetReflection(rootType) != nullptr; }

const AttributeInfo* FindOperationAttribute(Context* context)
{
    return context ? context->GetAttribute(brushType, "Operation") : nullptr;
}

ResourceRef MakeModelRef(const ea::string& modelName) { return ResourceRef(Model::GetTypeStatic(), modelName); }

} // namespace

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

Node* AddBrushNode(Node* rootNode, const ea::string& modelName, int operation, const Vector3& worldPosition)
{
    if (!IsRootNode(rootNode))
        return nullptr;
    Node* node = rootNode->CreateChild("Brush");
    Component* brush = node->CreateComponent(brushType);
    if (!brush)
    {
        node->Remove();
        return nullptr;
    }
    // The world position is correct for a root with any transform.
    node->SetWorldPosition(worldPosition);
    // The inverse of the root world scale gives the brush a world scale of about 1.
    // The local rotation stays identity, so the root scale cancels.
    // A rotated root with a non-uniform scale can shear the brush when the brush rotates later.
    const Vector3 rootScale = rootNode->GetWorldScale();
    const auto inverse = [](float value) { return Abs(value) > M_EPSILON ? 1.0f / value : 1.0f; };
    node->SetScale(Vector3(inverse(rootScale.x_), inverse(rootScale.y_), inverse(rootScale.z_)));
    brush->SetAttribute("Model", MakeModelRef(modelName));
    brush->SetAttribute("Operation", operation);
    brush->ApplyAttributes();
    return node;
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
    Component* component = GetBrushComponent(brush);
    if (!component || !component->SetAttribute("Model", MakeModelRef(modelName)))
        return false;
    component->ApplyAttributes();
    return true;
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
        const ea::string model = brush->GetAttribute("Model").GetResourceRef().name_;
        text += ea::string(ea::string::CtorSprintf(), "%u: %s | %s | %s\n", index++, operand->GetName().c_str(),
            OperationName(context, GetOperation(operand)).c_str(), model.empty() ? "(no model)" : model.c_str());
    }

    const Variant status = root->GetAttribute("Build Status");
    if (status.GetType() == VAR_STRING && !status.GetString().empty())
        text += status.GetString() + "\n";
    return text;
}

} // namespace Urho3D
