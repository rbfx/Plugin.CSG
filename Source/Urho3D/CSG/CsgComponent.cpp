// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgComponent.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Node.h>

namespace Urho3D
{

CsgComponent::CsgComponent(Context* context)
    : Component(context)
{
}

void CsgComponent::RegisterObject(Context* context)
{
    // CsgComponent is abstract, so don't add factory reflection.
    // Just register common attributes that derived classes will copy.
    URHO3D_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Model", ResourceRef, modelAttr_, ResourceRef(Model::GetTypeStatic()), AM_DEFAULT);
    URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Materials", GetMaterialsAttr, SetMaterials, ResourceRefList,
        ResourceRefList(Material::GetTypeStatic()), AM_DEFAULT);
}

void CsgComponent::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
{
    const Color wireColor(1.0f, 0.85f, 0.0f, 0.2f);
    DrawDebugGeometryInternal(debug, depthTest, GetDebugModelAttr(), wireColor);
}

const ResourceRefList& CsgComponent::GetMaterialsAttr() const
{
    return materialsAttr_;
}

void CsgComponent::SetModel(const ResourceRef& value)
{
    if (modelAttr_ == value)
        return;

    modelAttr_ = value;
    OnModelChanged();
}

void CsgComponent::SetMaterials(const ResourceRefList& value)
{
    if (materialsAttr_ == value)
        return;

    materialsAttr_ = value;
    OnMaterialsChanged();
}

void CsgComponent::DrawDebugGeometryInternal(DebugRenderer* debug, bool depthTest, const ResourceRef& modelAttr, const Color& wireColor) const
{
    if (!debug || !node_)
        return;

    Model* model = GetCachedDebugModel(modelAttr.name_);
    if (!model)
        return;

    const unsigned numGeometries = model->GetNumGeometries();
    for (unsigned geomIndex = 0; geomIndex < numGeometries; ++geomIndex)
    {
        Geometry* geometry = model->GetGeometry(geomIndex, 0);
        if (!geometry)
            continue;

        IndexBuffer* ib = geometry->GetIndexBuffer();
        if (!ib || !ib->GetShadowData())
            continue;

        const auto& vbs = geometry->GetVertexBuffers();
        for (const auto& vb : vbs)
        {
            if (!vb || !vb->GetShadowData())
                continue;

            debug->AddTriangleMesh(
                vb->GetShadowData(), vb->GetVertexSize(), geometry->GetVertexStart(),
                ib->GetShadowData(), ib->GetIndexSize(), geometry->GetIndexStart(), geometry->GetIndexCount(),
                node_->GetWorldTransform(), wireColor, depthTest);
        }
    }
}

Model* CsgComponent::GetCachedDebugModel(const ea::string& modelName) const
{
    if (modelName.empty())
        return nullptr;

    // A new name or a released model gives one lookup. A missing model gives no lookup until the name changes.
    if (modelName != debugModelName_ || (!debugModel_ && !debugModelMissing_))
    {
        auto* cache = GetSubsystem<ResourceCache>();
        debugModel_ = cache->GetResource<Model>(modelName);
        debugModelName_ = modelName;
        debugModelMissing_ = !debugModel_;
    }
    return debugModel_;
}

void CsgComponent::OnModelChanged()
{
    // A model change resets the debug model cache. A reload of a missing model can then find it.
    debugModel_ = nullptr;
    debugModelName_.clear();
    debugModelMissing_ = false;

    unsigned numGeometries = 0;

    const ea::string& modelName = modelAttr_.name_;
    if (!modelName.empty())
    {
        auto* cache = GetSubsystem<ResourceCache>();
        if (Model* model = cache->GetResource<Model>(modelName))
            numGeometries = model->GetNumGeometries();
        else
            URHO3D_LOGWARNING("CsgComponent: Could not find Model resource '{}'", modelName);
    }

    materialsAttr_.names_.resize(numGeometries);
}

} // namespace Urho3D
