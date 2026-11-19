// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/_Plugin.h"

namespace Urho3D
{

struct CsgShapeMesh;

/// Base class of the brushes with a generated shape. The class is abstract and has no factory.
///
/// A derived type keeps the shape parameters and gives the mesh with `GenerateMesh`.
/// - The brush has no "Model" attribute. It owns an unnamed model that is not in the resource cache.
/// - A parameter change marks the mesh dirty. The first reader generates the model on the main thread.
///   The readers are the root (through `GetGeneratedModel`) and the debug draw.
/// - Not valid parameters give a null model. Then the brush gives no operand and logs one warning.
///   The brush logs again only after the parameters were valid in between.
/// - The "Materials" attribute has one entry for each material slot. See `GetNumMaterialSlots`.
///
/// To add a shape: add a generator function in CsgShapes.h and a subclass that calls it.
class PLUGIN_RBFXCSG_API CsgProceduralBrush : public CsgBrush
{
    URHO3D_OBJECT(CsgProceduralBrush, CsgBrush);

public:
    explicit CsgProceduralBrush(Context* context);
    ~CsgProceduralBrush() override;

    /// Register the common attributes: "Is Enabled", "Operation" and "Materials". There is no "Model".
    /// @nobind
    static void RegisterObject(Context* context);

    /// Return the generated model. Return null if the parameters are not valid.
    Model* GetShapeModel() const;
    /// Return true if the parameters give a valid shape.
    bool IsShapeValid() const { return GetShapeModel() != nullptr; }

    /// @nobind
    bool GetGeneratedModel(SharedPtr<Model>& model) const override;

    /// Draw the generated model in the color of the operation.
    void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

protected:
    /// Clamp the parameters to the valid range. A value that is not finite gets its default value.
    virtual void SanitizeParameters() {}
    /// Generate the mesh from the parameters. Return false if the parameters are not valid.
    virtual bool GenerateMesh(CsgShapeMesh& mesh) const = 0;
    /// Return the number of material slots. Each slot is one geometry of the model.
    virtual unsigned GetNumMaterialSlots() const { return 1; }

    /// Sanitize the parameters, mark the mesh dirty and request a rebuild of the root.
    /// A typed setter calls this after it changes a parameter.
    void OnParametersChanged();

    void ApplyAttributes() override;
    void OnNodeSet(Node* previousNode, Node* currentNode) override;

private:
    /// Resize the "Materials" attribute to the number of material slots. The existing names stay.
    void UpdateMaterialSlots();

    /// The generated model. Null if the parameters are not valid.
    mutable SharedPtr<Model> model_;
    /// True if the model must be generated again before the next read.
    mutable bool meshDirty_{true};
    /// True if the last generated shape was valid. The warning shows only when the shape becomes not valid.
    mutable bool lastValid_{true};
};

} // namespace Urho3D
