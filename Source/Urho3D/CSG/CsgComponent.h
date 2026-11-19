// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Urho3D/Core/Variant.h>
#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Scene/Component.h>

namespace Urho3D
{

class DebugRenderer;

/// Base class for CSG components with common model and material handling.
/// This is an abstract base class and should not be instantiated directly.
class PLUGIN_RBFXCSG_API CsgComponent : public Component
{
    URHO3D_OBJECT(CsgComponent, Component);

public:
    explicit CsgComponent(Context* context);

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    const ResourceRef& GetModelAttr() const { return modelAttr_; }
    virtual void SetModel(const ResourceRef& value);

    virtual const ResourceRefList& GetMaterialsAttr() const;
    virtual void SetMaterials(const ResourceRefList& value);

protected:
    void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;
    void DrawDebugGeometryInternal(DebugRenderer* debug, bool depthTest, const ResourceRef& modelAttr, const Color& wireColor) const;
    /// Draw the wireframe of the model. A component with a generated model uses this overload.
    void DrawDebugGeometryInternal(DebugRenderer* debug, bool depthTest, Model* model, const Color& wireColor) const;
    virtual const ResourceRef& GetDebugModelAttr() const = 0;
    virtual void OnModelChanged();
    virtual void OnMaterialsChanged() {}

private:
    /// Return the debug model of the name. The component keeps the model, so a frame does no cache lookup.
    Model* GetCachedDebugModel(const ea::string& modelName) const;

    ResourceRef modelAttr_{Model::GetTypeStatic()};
    ResourceRefList materialsAttr_{Material::GetTypeStatic()};

    /// The debug model cache. The model name is the key. A missing model is not looked up again for the same name.
    mutable WeakPtr<Model> debugModel_;
    mutable ea::string debugModelName_;
    mutable bool debugModelMissing_{};
};

} // namespace Urho3D
