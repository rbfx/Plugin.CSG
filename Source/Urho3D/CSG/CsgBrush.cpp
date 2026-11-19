// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgOperationColor.h"
#include "Urho3D/CSG/CsgRoot.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>

namespace Urho3D
{

CsgBrush::CsgBrush(Context* context)
    : CsgComponent(context)
{
}

CsgBrush::~CsgBrush()
{
    if (CsgRoot* root = root_)
        root->UnregisterBrush(this);
}

void CsgBrush::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgBrush>(Category_Scene);

    URHO3D_ENUM_ATTRIBUTE("Operation", operation_, CsgOperationNames, CsgOperation::Union, AM_DEFAULT);
    URHO3D_COPY_BASE_ATTRIBUTES(CsgComponent);
}

void CsgBrush::ApplyAttributes()
{
    Component::ApplyAttributes();
    MarkDirty();
}

void CsgBrush::OnSetEnabled()
{
    Component::OnSetEnabled();
    UpdateRootRegistration();
}

void CsgBrush::OnNodeSet(Node* previousNode, Node* currentNode)
{
    Component::OnNodeSet(previousNode, currentNode);
    UpdateRootRegistration();
}

void CsgBrush::OnSceneSet(Scene* previousScene, Scene* scene)
{
    Component::OnSceneSet(previousScene, scene);
    UpdateRootRegistration();
}

void CsgBrush::OnModelChanged()
{
    BaseClassName::OnModelChanged();
    MarkDirty();
}

void CsgBrush::OnMaterialsChanged()
{
    BaseClassName::OnMaterialsChanged();
    MarkDirty();
}

void CsgBrush::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
{
    // The alpha keeps the wireframe light.
    Color wireColor = GetOperationColor(static_cast<int>(operation_));
    wireColor.a_ = 0.4f;
    DrawDebugGeometryInternal(debug, depthTest, GetDebugModelAttr(), wireColor);
}

void CsgBrush::SetOperation(CsgOperation op)
{
    if (operation_ == op)
        return;

    operation_ = op;
    MarkDirty();
}

void CsgBrush::MarkDirty()
{
    dirty_ = true;

    UpdateRootRegistration();
    if (CsgRoot* root = root_)
        root->RequestRebuild();
}

CsgRoot* CsgBrush::FindContainingRoot() const
{
    // A brush on a node with its own root is not an operand.
    if (!node_ || node_->GetComponent<CsgRoot>())
        return nullptr;

    // Only the root of the parent node uses the brush.
    Node* parent = node_->GetParent();
    return parent ? parent->GetComponent<CsgRoot>() : nullptr;
}

void CsgBrush::UpdateRootRegistration()
{
    CsgRoot* newRoot = nullptr;
    if (IsEnabledEffective() && node_)
        newRoot = FindContainingRoot();

    if (newRoot == root_)
        return;

    if (CsgRoot* oldRoot = root_)
        oldRoot->UnregisterBrush(this);

    root_ = newRoot;

    if (CsgRoot* root = root_)
        root->RegisterBrush(this);
}

} // namespace Urho3D
