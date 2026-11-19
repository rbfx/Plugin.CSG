// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgShapes.h"

#include <Urho3D/Graphics/VertexBuffer.h>

namespace Urho3D
{

const ea::vector<VertexElement>& CsgGetShapeVertexElements()
{
    static const ea::vector<VertexElement> elements = []
    {
        ea::vector<VertexElement> result{
            VertexElement{TYPE_VECTOR3, SEM_POSITION, 0},
            VertexElement{TYPE_VECTOR3, SEM_NORMAL, 0},
            VertexElement{TYPE_VECTOR4, SEM_TANGENT, 0},
            VertexElement{TYPE_VECTOR2, SEM_TEXCOORD, 0},
            VertexElement{TYPE_VECTOR2, SEM_TEXCOORD, 1},
            VertexElement{TYPE_UBYTE4_NORM, SEM_COLOR, 0},
        };
        VertexBuffer::UpdateOffsets(result);
        return result;
    }();
    return elements;
}

} // namespace Urho3D
