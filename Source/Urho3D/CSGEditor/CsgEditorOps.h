// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include <Urho3D/Container/Str.h>
#include <Urho3D/Math/Vector3.h>

#include <EASTL/vector.h>

namespace Urho3D
{

class Context;
class Node;

// Scene functions for the CSG editor addon.
//
// The functions do not link the CSG library. They find the types "CsgRoot" and "CsgBrush" by name.
// They read and write the attributes by name. If "CsgRoot" is not registered, a function returns null or false.

/// Return the node that has a CsgRoot: the selected node or its parent. A temporary node maps to its parent.
Node* FindActiveRoot(Node* selected);
/// Return true if the node has a CsgBrush.
bool IsBrushNode(Node* node);
/// Return true if the node has a CsgRoot.
bool IsRootNode(Node* node);

/// Create a child node "CSG" with a CsgRoot at the world position.
Node* CreateRootNode(Node* parent, const Vector3& worldPosition);
/// Create a child node "Brush" at the end of the operand list. The model name may be empty.
/// The local scale is the inverse of the root world scale. The brush world scale is about 1.
/// A disabled brush is not an operand. The root does not rebuild when it gets the node.
Node* AddBrushNode(
    Node* rootNode, const ea::string& modelName, int operation, const Vector3& worldPosition, bool enabled = true);

/// Return the operation of the brush, or -1 if the node has no brush.
int GetOperation(Node* brush);
bool SetOperation(Node* brush, int operation);
/// Return the number of operations. The names come from the attribute "Operation" of CsgBrush.
int OperationCount(Context* context);
/// Return the name of the operation, or an empty string if the index is not valid.
ea::string OperationName(Context* context, int operation);
/// Add the step to the operation and wrap around.
int CycleOperation(Context* context, int current, int step);

bool SetBrushModel(Node* brush, const ea::string& modelName);

/// Return the direct children with an enabled CsgBrush, in child order.
ea::vector<Node*> GetOperands(Node* rootNode);
/// Return the direct children with a CsgBrush, enabled or not, in child order.
ea::vector<Node*> GetAllBrushChildren(Node* rootNode);

/// Move a brush child. The indices count the brush children only. The root rebuilds after the move.
bool MoveOperand(Node* rootNode, unsigned fromIndex, unsigned toIndex);
/// Set the enabled state of the CsgBrush component. The root rebuilds after the change.
bool SetBrushEnabled(Node* brush, bool enabled);
/// Toggle the enabled state of the CsgBrush component.
bool ToggleBrushEnabled(Node* brush);

/// Return a text with one line for each operand and the build status of the root.
ea::string DescribeRoot(Node* rootNode);

} // namespace Urho3D
