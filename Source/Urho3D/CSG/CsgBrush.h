// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Urho3D/Core/Variant.h>
#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgComponent.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

class CsgRoot;

/// Operand component for component-based CSG workflow.
///
/// High-level behavior:
/// - Represents one operand in a `CsgRoot` boolean operation tree.
/// - The operand geometry is taken from the base `CsgComponent` attributes (`Model` + `Materials`).
/// - A procedural brush gives a generated model in place of the `Model` attribute. See `GetGeneratedModel`.
/// - The boolean operation to apply against the accumulated result is controlled by `op_`.
///
/// Root association:
/// - A brush registers itself with the `CsgRoot` of its parent node. Only a direct child of a root is an operand.
/// - A brush on a node that has its own `CsgRoot` is not an operand.
/// - The association is updated when the brush is moved between nodes/scenes or enabled/disabled.
///
/// Dirty semantics:
/// - A brush becomes dirty when its operation/model/materials/state changes.
/// - Dirtiness is acknowledged by the root after a successful rebuild.
class PLUGIN_RBFXCSG_API CsgBrush : public CsgComponent
{
    URHO3D_OBJECT(CsgBrush, CsgComponent);

public:
    explicit CsgBrush(Context* context);
    ~CsgBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Get boolean operation used when combining this brush with the current CSG result.
    CsgOperation GetOperation() const { return operation_; }
    /// Set boolean operation used when combining this brush with the current CSG result.
    void SetOperation(CsgOperation op);
    /// Get the UV mode. Local keeps the UV of the mesh. Root projects the UV in the frame of the root.
    CsgUvMode GetUvMode() const { return uvMode_; }
    void SetUvMode(CsgUvMode mode);
    /// Get the texture repeats for each unit.
    float GetUvScale() const { return uvScale_; }
    void SetUvScale(float scale);
    /// Get the UV offset. It is added after the scale.
    const Vector2& GetUvOffset() const { return uvOffset_; }
    void SetUvOffset(const Vector2& offset);
    /// Whether brush state has changed since last acknowledged by a root.
    bool IsDirty() const { return dirty_; }
    /// Acknowledge brush state change.
    void ClearDirty() { dirty_ = false; }

    /// Draw the model wireframe in the color of the operation. See `GetOperationColor`.
    void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    /// Get the generated model of a procedural brush. Return false when the brush uses the `Model` attribute.
    /// Return true when the brush is procedural. Then the root uses `model` and ignores the `Model` attribute.
    /// A procedural brush with a null `model` is invalid. It gives no operand until it is valid again.
    /// For a generated model, an empty or missing material slot uses the material of slot 0.
    /// @nobind
    virtual bool GetGeneratedModel(SharedPtr<Model>& model) const { return false; }

protected:
    /// Mark the brush dirty and request a rebuild of the root.
    void MarkDirty();

    const ResourceRef& GetDebugModelAttr() const override { return GetModelAttr(); }

    void ApplyAttributes() override;
    void OnSetEnabled() override;
    void OnNodeSet(Node* previousNode, Node* currentNode) override;
    void OnSceneSet(Scene* previousScene, Scene* scene) override;
    void OnModelChanged() override;
    void OnMaterialsChanged() override;

    /// The default is Local. A procedural brush sets Root in its constructor.
    CsgUvMode uvMode_{CsgUvMode::Local};

private:
    friend class CsgRoot;

    CsgRoot* FindContainingRoot() const;
    void UpdateRootRegistration();

    CsgOperation operation_{CsgOperation::Union};
    float uvScale_{1.0f};
    Vector2 uvOffset_{Vector2::ZERO};
    WeakPtr<CsgRoot> root_;
    bool dirty_{};
    /// Local transform of the node at the last build or check of the root.
    Matrix3x4 seenLocalTransform_{Matrix3x4::IDENTITY};
};

} // namespace Urho3D
