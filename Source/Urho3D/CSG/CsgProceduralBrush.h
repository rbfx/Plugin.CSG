// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgShapeProtocol.h"
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
/// - Each face group of the mesh is one geometry of the model and one material slot.
///   The "Materials" attribute always has one entry for each slot. See `GetNumMaterialSlots`.
///   An empty slot uses the material of slot 0. Equal materials merge into one output geometry.
///
/// To add a shape: add a generator function in CsgShapes.h and a subclass that calls it.
class PLUGIN_RBFXCSG_API CsgProceduralBrush : public CsgBrush
{
    URHO3D_OBJECT(CsgProceduralBrush, CsgBrush);

public:
    explicit CsgProceduralBrush(Context* context);
    ~CsgProceduralBrush() override;

    /// Register the common attributes: "Is Enabled", "Operation", "Materials", "UV Mode", "UV Scale" and "UV Offset".
    /// There is no "Model". The default "UV Mode" is Root.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Return the generated model. Return null if the parameters are not valid.
    Model* GetShapeModel() const;
    /// Return true if the parameters give a valid shape.
    bool IsShapeValid() const { return GetShapeModel() != nullptr; }

    /// @nobind
    bool GetGeneratedModel(SharedPtr<Model>& model) const override;

    /// Set the materials. The list is resized to the number of material slots. The extra names go.
    void SetMaterials(const ResourceRefList& value) override;

    /// Draw the generated model in the color of the operation.
    void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    /// Fill the list with the handles of the shape. The old content goes. The positions are in the local space of the brush.
    /// The default gives no handle. A shape with handles gives the ids that its header lists.
    virtual void GetHandles(ea::vector<CsgShapeHandle>& handles) const { handles.clear(); }
    /// Move handles by the TOTAL local delta from the drag start. The call is stateless.
    /// It applies the delta to the current parameters, so the caller restores the parameters before each call.
    /// A handle id that is repeated counts once. The handles apply in the given order.
    /// Return false and keep the parameters if an id is not known, the delta is not finite,
    /// the result is not valid or the parameters do not change.
    /// Otherwise set the parameters with the clamping setter (one `MarkDirty`) and return true.
    /// The default has no handle and returns false.
    virtual bool MoveHandles(const ea::vector<unsigned>& ids, const Vector3& localDelta) { return false; }

protected:
    /// Clamp the parameters to the valid range. A value that is not finite gets its default value.
    virtual void SanitizeParameters() {}
    /// Generate the mesh from the parameters. Return false if the parameters are not valid.
    virtual bool GenerateMesh(CsgShapeMesh& mesh) const = 0;
    /// Return the number of material slots. It is fixed for a shape type. Slot i is face group i of the mesh.
    virtual unsigned GetNumMaterialSlots() const { return 1; }

    /// Prepare the input of `MoveHandles`: copy the ids without repeats, in the given order.
    /// Return false if the list is empty or the delta is not finite.
    static bool PrepareHandleMove(const ea::vector<unsigned>& ids, const Vector3& delta, ea::vector<unsigned>& unique);
    /// Return true if all components are finite.
    static bool IsFinite(const Vector3& value);

    /// Sanitize the parameters, mark the mesh dirty and request a rebuild of the root.
    /// A typed setter calls this after it changes a parameter.
    void OnParametersChanged();

    void ApplyAttributes() override;
    void OnNodeSet(Node* previousNode, Node* currentNode) override;

private:
    /// Answer `E_CSGSHAPEQUERYHANDLES`. The data map gets the result.
    void HandleQueryHandles(StringHash eventType, VariantMap& eventData);
    /// Answer `E_CSGSHAPEMOVEHANDLES`. The data map gets the result.
    void HandleMoveHandles(StringHash eventType, VariantMap& eventData);

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
