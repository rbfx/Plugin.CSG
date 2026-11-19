// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoolean.h"
#include "Urho3D/CSG/CsgBoxBrush.h"
#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/CsgComponent.h"
#include "Urho3D/CSG/CsgCylinderBrush.h"
#include "Urho3D/CSG/CsgRingBrush.h"
#include "Urho3D/CSG/CsgProceduralBrush.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSG/CsgShapes.h"
#include "Urho3D/CSG/CsgSphereBrush.h"
#include "Urho3D/CSG/CsgSolid.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include <Urho3D/Container/ByteVector.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Variant.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Math/Matrix3.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Vector4.h>
#include <Urho3D/RenderAPI/RenderAPIDefs.h>
#include <Urho3D/Plugins/PluginApplication.h>

#include <EASTL/algorithm.h>
#include <EASTL/fixed_vector.h>
#include <EASTL/hash_map.h>
#include <EASTL/hash_set.h>
#include <EASTL/sort.h>

#include <cassert>
#include <type_traits>

namespace Urho3D
{

namespace
{

inline unsigned PackSharedKey(unsigned meshId, unsigned geometryIndex)
{
    return (meshId << 16u) | (geometryIndex & 0xFFFFu);
}

inline ea::pair<unsigned, unsigned> UnpackSharedKey(unsigned sharedKey)
{
    const unsigned meshId = (sharedKey >> 16u) & 0xFFFFu;
    const unsigned geometryIndex = sharedKey & 0xFFFFu;
    return {meshId, geometryIndex};
}

bool IsSupportedCsgVertexElement(const VertexElement& element)
{
    if (element.stepRate_ != 0)
        return false; // Per-instance data is not supported.

    switch (element.semantic_)
    {
    case SEM_POSITION:
    case SEM_NORMAL:
    case SEM_BINORMAL:
    case SEM_TANGENT:
    case SEM_TEXCOORD:
    case SEM_COLOR: return true;

    case SEM_BLENDWEIGHTS:
    case SEM_BLENDINDICES:
    case SEM_OBJECTINDEX:
    default: return false;
    }
}

ea::vector<VertexElement> GetSupportedVertexElements(const Geometry* sourceGeometry)
{
    ea::vector<VertexElement> elements;

    if (sourceGeometry)
    {
        unsigned vertexSize = 0;
        unsigned indexSize = 0;
        const unsigned char* vertexData = nullptr;
        const unsigned char* indexData = nullptr;
        const ea::vector<VertexElement>* srcElements = nullptr;
        sourceGeometry->GetRawData(vertexData, vertexSize, indexData, indexSize, srcElements);
        assert(vertexData);
        assert(srcElements);

        elements.reserve(srcElements->size());
        ea::copy_if(srcElements->begin(), srcElements->end(), ea::back_inserter(elements), IsSupportedCsgVertexElement);
    }
    else
    {
        elements.emplace_back(TYPE_VECTOR3, SEM_POSITION, 0);
    }

    VertexBuffer::UpdateOffsets(elements);
    return elements;
}

ByteVector PackVerticesToLayout(const ea::vector<ModelVertex>& vertices, const ea::vector<VertexElement>& elements)
{
    // `ModelVertex` is documented to be equivalent to an array of Vector4. This function relies on that.
    // Keep these checks close to the usage to fail loudly if ModelVertex layout changes.
    constexpr unsigned kModelVertexVector4Count = 6u + ModelVertex::MaxColors + ModelVertex::MaxUVs;
    static_assert(std::is_standard_layout_v<ModelVertex>, "ModelVertex must be standard-layout");
    static_assert(alignof(ModelVertex) == alignof(Vector4), "ModelVertex and Vector4 alignment must match");
    static_assert(sizeof(ModelVertex) == sizeof(Vector4) * kModelVertexVector4Count, "ModelVertex must be exactly N*Vector4 bytes");

    const unsigned vertexCount = vertices.size();
    const unsigned vertexSize = VertexBuffer::GetVertexSize(elements);

    ByteVector packed;
    packed.resize(vertexCount * vertexSize);
    if (vertexCount == 0)
        return packed;

    // ModelVertex is equivalent to an array of Vector4 and vertices are contiguous in memory.
    const Vector4* srcUnpacked = reinterpret_cast<const Vector4*>(vertices.data());
    ea::vector<Vector4> dstUnpacked(vertexCount * elements.size());

    VertexBuffer::ShuffleUnpackedVertexData(
        vertexCount, srcUnpacked, ModelVertex::VertexElements, dstUnpacked.data(), elements, true);

    const unsigned srcStride = sizeof(Vector4) * elements.size();
    for (unsigned elementIndex = 0; elementIndex < elements.size(); ++elementIndex)
    {
        VertexBuffer::PackVertexData(dstUnpacked.data() + elementIndex, srcStride, packed.data(), vertexSize,
            elements[elementIndex], 0, vertexCount);
    }

    return packed;
}

/// Build a shadowed vertex buffer, an index buffer and a triangle list geometry from unpacked vertices.
/// Return false if the vertex buffer cannot get the size.
bool BuildGeometryBuffers(Context* context, const ea::vector<ModelVertex>& vertices, const ea::vector<unsigned>& indices,
    const ea::vector<VertexElement>& elements, SharedPtr<VertexBuffer>& vb, SharedPtr<IndexBuffer>& ib,
    SharedPtr<Geometry>& geom)
{
    const unsigned vertexCount = vertices.size();
    vb = MakeShared<VertexBuffer>(context);
    vb->SetShadowed(true);
    if (!vb->SetSize(vertexCount, elements))
        return false;

    const ByteVector packedVertices = PackVerticesToLayout(vertices, elements);
    vb->Update(packedVertices.data());

    const bool largeIndices = vertexCount > 0xFFFFu;
    ib = MakeShared<IndexBuffer>(context);
    ib->SetShadowed(true);
    ib->SetSize(indices.size(), largeIndices);
    ib->SetUnpackedData(indices.data(), 0, indices.size());

    geom = MakeShared<Geometry>(context);
    geom->SetNumVertexBuffers(1);
    geom->SetVertexBuffer(0, vb);
    geom->SetIndexBuffer(ib);
    geom->SetDrawRange(TRIANGLE_LIST, 0, ib->GetIndexCount(), 0, vb->GetVertexCount());
    return true;
}

} // namespace

ea::vector<CsgPolygon> CsgBuildPolygonsFromModel(const Model* model, const Matrix3x4& worldTransform, unsigned meshId)
{
    ea::vector<CsgPolygon> polygons;
    if (!model)
        return polygons;

    const Matrix3 basis = worldTransform.ToMatrix3();
    const bool basisOk = !basis.IsNaN() && !basis.IsInf() && Abs(basis.Determinant()) > M_EPSILON;
    const Matrix3 normalMatrix = basisOk ? basis.Inverse().Transpose() : Matrix3::IDENTITY;

    const unsigned numGeometries = model->GetNumGeometries();
    for (unsigned i = 0; i < numGeometries; ++i)
    {
        Geometry* geom = model->GetGeometry(i, 0);
        if (!geom)
            continue;

        unsigned vertexSize = 0;
        unsigned indexSize = 0;
        const unsigned char* vertexData = nullptr;
        const unsigned char* indexData = nullptr;
        const ea::vector<VertexElement>* elements = nullptr;

        geom->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
        if (!vertexData || !elements)
            continue;

        // Element lookup table
        ea::fixed_vector<const VertexElement*, MAX_VERTEX_ELEMENT_SEMANTICS * 4> elementPointers;
        elementPointers.resize(elementPointers.max_size(), nullptr);
        for (const VertexElement& element : *elements)
            elementPointers[(element.semantic_ << 2) + element.index_] = &element;

        // The draw range selects the triangles. Geometries of one model can share one index buffer.
        const unsigned indexCount = geom->GetIndexCount();
        const unsigned indexStart = indexData ? geom->GetIndexStart() : geom->GetVertexStart();
        const unsigned indexEnd = indexStart + indexCount;

        for (unsigned j = indexStart; j + 2 < indexEnd; j += 3)
        {
            unsigned i0, i1, i2;
            if (!indexData)
            {
                i0 = j;
                i1 = j + 1;
                i2 = j + 2;
            }
            else if (indexSize == 2)
            {
                i0 = reinterpret_cast<const unsigned short*>(indexData)[j];
                i1 = reinterpret_cast<const unsigned short*>(indexData)[j + 1];
                i2 = reinterpret_cast<const unsigned short*>(indexData)[j + 2];
            }
            else
            {
                i0 = reinterpret_cast<const unsigned*>(indexData)[j];
                i1 = reinterpret_cast<const unsigned*>(indexData)[j + 1];
                i2 = reinterpret_cast<const unsigned*>(indexData)[j + 2];
            }

            ea::vector<ModelVertex> verts;
            for (unsigned index : {i0, i1, i2})
            {
                ModelVertex v{};

                // Position is required.
                if (const VertexElement* element = elementPointers[SEM_POSITION << 2])
                {
                    Vector4 position4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &position4, sizeof(Vector4));
                    v.SetPosition(worldTransform * position4.ToVector3());
                }

                // Normal.
                if (const VertexElement* element = elementPointers[SEM_NORMAL << 2])
                {
                    Vector4 normal4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &normal4, sizeof(Vector4));
                    const Vector3 transformedNormal = normalMatrix * normal4.ToVector3();
                    v.SetNormal(transformedNormal.NormalizedOrDefault(Vector3::UP));
                }

                // Tangent.
                if (const VertexElement* element = elementPointers[SEM_TANGENT << 2])
                {
                    Vector4 tangent4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &tangent4, sizeof(Vector4));
                    const Vector3 transformedTangent =
                        (basis * tangent4.ToVector3()).NormalizedOrDefault(Vector3::RIGHT);
                    v.tangent_ = Vector4(transformedTangent, tangent4.w_);
                }

                // Binormal.
                if (const VertexElement* element = elementPointers[SEM_BINORMAL << 2])
                {
                    Vector4 binormal4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &binormal4, sizeof(Vector4));
                    const Vector3 transformedBinormal =
                        (basis * binormal4.ToVector3()).NormalizedOrDefault(Vector3::UP);
                    v.binormal_ = Vector4(transformedBinormal, 0.0f);
                }

                // Colors and UVs.
                for (unsigned char i = 0; i < ModelVertex::MaxColors; ++i)
                {
                    if (const VertexElement* element = elementPointers[(SEM_COLOR << 2) + i])
                    {
                        VertexBuffer::UnpackVertexData(
                            vertexData, vertexSize, *element, index, 1, &v.color_[i], sizeof(Vector4));
                    }
                }

                for (unsigned char i = 0; i < ModelVertex::MaxUVs; ++i)
                {
                    if (const VertexElement* element = elementPointers[(SEM_TEXCOORD << 2) + i])
                    {
                        VertexBuffer::UnpackVertexData(
                            vertexData, vertexSize, *element, index, 1, &v.uv_[i], sizeof(Vector4));
                    }
                }

                // Ignore animation/instancing data (blend weights/indices, per-instance elements).

                verts.push_back(v);
            }

            polygons.emplace_back(ea::move(verts), PackSharedKey(meshId, i), 0u);
        }
    }

    return polygons;
}

void CsgProjectPolygonUvs(ea::vector<CsgPolygon>& polygons, CsgUvMode mode, float scale, const Vector2& offset)
{
    if (mode == CsgUvMode::Local)
    {
        if (scale == 1.0f && offset == Vector2::ZERO)
            return;
        for (CsgPolygon& polygon : polygons)
        {
            for (ModelVertex& vertex : polygon.vertices_)
            {
                vertex.uv_[0].x_ = vertex.uv_[0].x_ * scale + offset.x_;
                vertex.uv_[0].y_ = vertex.uv_[0].y_ * scale + offset.y_;
            }
        }
        return;
    }

    for (CsgPolygon& polygon : polygons)
    {
        const Vector3& planeNormal = polygon.plane_.normal_;
        const float ax = Abs(planeNormal.x_);
        const float ay = Abs(planeNormal.y_);
        const float az = Abs(planeNormal.z_);
        const unsigned axis = ax >= ay && ax >= az ? 0 : (ay >= az ? 1 : 2);
        const float s = planeNormal.Data()[axis] >= 0.0f ? 1.0f : -1.0f;

        // The axes of the coordinates: first is U, second is up.
        const unsigned firstAxis = axis == 0 ? 2 : 0;
        const unsigned secondAxis = axis == 1 ? 2 : 1;
        const float uSign = axis == 2 ? s : -s;
        Vector3 uDir = Vector3::ZERO;
        uDir.MutableData()[firstAxis] = uSign;
        Vector3 upDir = Vector3::ZERO;
        upDir.MutableData()[secondAxis] = 1.0f;

        for (ModelVertex& vertex : polygon.vertices_)
        {
            const Vector3 position = vertex.GetPosition();
            vertex.uv_[0].x_ = uSign * position.Data()[firstAxis] * scale + offset.x_;
            vertex.uv_[0].y_ = -position.Data()[secondAxis] * scale + offset.y_;

            const Vector3 normal = vertex.GetNormal();
            const Vector3 tangent = (uDir - normal * normal.DotProduct(uDir)).NormalizedOrDefault(uDir);
            vertex.tangent_ = Vector4(tangent, tangent.CrossProduct(normal).DotProduct(upDir) >= 0.0f ? 1.0f : -1.0f);
        }
    }
}

SharedPtr<CsgSolid> CsgBuildSolidFromModel(
    const Model* model, const Matrix3x4& worldTransform, unsigned meshId, float epsilon)
{
    return CsgSolid::FromOperand(CsgBuildPolygonsFromModel(model, worldTransform, meshId), epsilon);
}

SharedPtr<const CsgSolid> CsgBooleanOperation(
    const CsgSolid* a, const CsgSolid* b, CsgOperation op, float epsilon, WorkQueue* workQueue)
{
    if (!a || !b)
        return nullptr;
    CsgParallelOptions parallel;
    parallel.workQueue_ = workQueue;
    return CsgInternal::Boolean(SharedPtr<const CsgSolid>(a), SharedPtr<const CsgSolid>(b), op, epsilon, parallel);
}

SharedPtr<Model> CsgBuildModel(Context* context, const CsgTriangulatedModel& triangulated,
    ea::span<const Model* const> sourceModels, ea::span<const ResourceRefList* const> sourceMaterials,
    ResourceRefList* outMaterials, ea::vector<unsigned>* outComponents)
{
    if (outMaterials)
    {
        outMaterials->type_ = Material::GetTypeStatic();
        outMaterials->names_.clear();
    }
    if (outComponents)
        outComponents->clear();

    if (triangulated.geometries_.empty() || !triangulated.hasAnyVertex_)
        return MakeShared<Model>(context);

    ea::vector<SharedPtr<VertexBuffer>> modelVertexBuffers;
    ea::vector<SharedPtr<IndexBuffer>> modelIndexBuffers;
    ea::vector<SharedPtr<Geometry>> modelGeometries;
    modelVertexBuffers.reserve(triangulated.geometries_.size());
    modelIndexBuffers.reserve(triangulated.geometries_.size());
    modelGeometries.reserve(triangulated.geometries_.size());

    if (outMaterials)
        outMaterials->names_.reserve(triangulated.geometries_.size());
    if (outComponents)
        outComponents->reserve(triangulated.geometries_.size());

    for (const auto& src : triangulated.geometries_)
    {
        const unsigned vertexCount = src.vertices_.size();
        if (vertexCount == 0 || src.indexData_.empty())
            continue;

        const auto [meshId, geometryIndex] = UnpackSharedKey(src.shared_);

        const Model* srcModel = meshId < sourceModels.size() ? sourceModels[meshId] : nullptr;
        Geometry* srcGeometry = srcModel ? srcModel->GetGeometry(geometryIndex, 0) : nullptr;
        const ea::vector<VertexElement> vbElements = GetSupportedVertexElements(srcGeometry);

        SharedPtr<VertexBuffer> vb;
        SharedPtr<IndexBuffer> ib;
        SharedPtr<Geometry> geom;
        if (!BuildGeometryBuffers(context, src.vertices_, src.indexData_, vbElements, vb, ib, geom))
            continue;

        modelVertexBuffers.push_back(vb);
        modelIndexBuffers.push_back(ib);
        modelGeometries.push_back(geom);

        if (outMaterials)
        {
            const ResourceRefList* srcMatList = meshId < sourceMaterials.size() ? sourceMaterials[meshId] : nullptr;
            if (srcMatList && geometryIndex < srcMatList->names_.size())
                outMaterials->names_.push_back(srcMatList->names_[geometryIndex]);
            else
                outMaterials->names_.push_back(ea::string{});
        }
        if (outComponents)
            outComponents->push_back(src.component_);
    }

    if (modelGeometries.empty())
        return MakeShared<Model>(context);

    auto outModel = MakeShared<Model>(context);
    outModel->SetVertexBuffers(modelVertexBuffers, {}, {});
    outModel->SetIndexBuffers(modelIndexBuffers);
    outModel->SetNumGeometries(modelGeometries.size());
    for (unsigned i = 0; i < modelGeometries.size(); ++i)
        outModel->SetGeometry(i, 0, modelGeometries[i]);
    outModel->SetBoundingBox(triangulated.boundingBox_);

    return outModel;
}

SharedPtr<Model> CsgBuildModelFromMesh(
    Context* context, const CsgShapeMesh& mesh, const ea::vector<VertexElement>& elements)
{
    const unsigned vertexCount = mesh.vertices_.size();
    const unsigned indexCount = mesh.indices_.size();
    if (vertexCount == 0 || indexCount == 0 || indexCount % 3 != 0)
        return nullptr;
    for (const unsigned index : mesh.indices_)
    {
        if (index >= vertexCount)
            return nullptr;
    }

    // An empty group list is one group with all indices.
    ea::vector<CsgShapeGroup> groups = mesh.groups_;
    if (groups.empty())
        groups.push_back(CsgShapeGroup{0, indexCount});
    // The slot is the geometry index. A slot without a group gets an empty geometry.
    constexpr unsigned maxSlots = 64;
    unsigned numSlots = 0;
    ea::vector<unsigned> slots(groups.size());
    for (unsigned i = 0; i < groups.size(); ++i)
    {
        const CsgShapeGroup& group = groups[i];
        if (group.indexCount_ == 0 || group.indexCount_ % 3 != 0 || group.indexStart_ > indexCount
            || group.indexCount_ > indexCount - group.indexStart_)
            return nullptr;
        slots[i] = group.slot_ == CsgShapeGroup::SlotFromIndex ? i : group.slot_;
        if (slots[i] >= maxSlots)
            return nullptr;
        for (unsigned j = 0; j < i; ++j)
        {
            if (slots[j] == slots[i])
                return nullptr;
        }
        numSlots = ea::max(numSlots, slots[i] + 1);
    }

    ea::vector<VertexElement> vbElements = elements.empty() ? CsgGetShapeVertexElements() : elements;
    VertexBuffer::UpdateOffsets(vbElements);

    // All groups share the buffers. Each group is one geometry with its own draw range.
    SharedPtr<VertexBuffer> vb;
    SharedPtr<IndexBuffer> ib;
    SharedPtr<Geometry> firstGeometry;
    if (!BuildGeometryBuffers(context, mesh.vertices_, mesh.indices_, vbElements, vb, ib, firstGeometry))
        return nullptr;

    BoundingBox boundingBox;
    for (const ModelVertex& vertex : mesh.vertices_)
        boundingBox.Merge(vertex.GetPosition());

    auto model = MakeShared<Model>(context);
    model->SetVertexBuffers({vb}, {}, {});
    model->SetIndexBuffers({ib});
    model->SetNumGeometries(numSlots);
    ea::vector<CsgShapeGroup> bySlot(numSlots, CsgShapeGroup{0, 0});
    for (unsigned i = 0; i < groups.size(); ++i)
        bySlot[slots[i]] = groups[i];
    for (unsigned slot = 0; slot < numSlots; ++slot)
    {
        SharedPtr<Geometry> geometry = slot == 0 ? firstGeometry : MakeShared<Geometry>(context);
        geometry->SetNumVertexBuffers(1);
        geometry->SetVertexBuffer(0, vb);
        geometry->SetIndexBuffer(ib);
        // A free slot has the draw range 0, 0.
        const CsgShapeGroup& group = bySlot[slot];
        geometry->SetDrawRange(TRIANGLE_LIST, group.indexStart_, group.indexCount_, 0, group.indexCount_ ? vertexCount : 0);
        model->SetGeometry(slot, 0, geometry);
    }
    model->SetBoundingBox(boundingBox);
    return model;
}

void RegisterCsgLibrary(Context* context)
{
    CsgComponent::RegisterObject(context);
    CsgBrush::RegisterObject(context);
    CsgProceduralBrush::RegisterObject(context);
    CsgBoxBrush::RegisterObject(context);
    CsgSphereBrush::RegisterObject(context);
    CsgCylinderBrush::RegisterObject(context);
    CsgRingBrush::RegisterObject(context);
    CsgRoot::RegisterObject(context);
}

void RegisterCsgLibrary(PluginApplication& plugin)
{
    plugin.RegisterObject<CsgComponent>();
    plugin.RegisterObject<CsgBrush>();
    plugin.RegisterObject<CsgProceduralBrush>();
    plugin.RegisterObject<CsgBoxBrush>();
    plugin.RegisterObject<CsgSphereBrush>();
    plugin.RegisterObject<CsgCylinderBrush>();
    plugin.RegisterObject<CsgRingBrush>();
    plugin.RegisterObject<CsgRoot>();
}

} // namespace Urho3D
