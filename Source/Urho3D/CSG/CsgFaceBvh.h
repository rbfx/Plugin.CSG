// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"
#include <Urho3D/Math/Vector3.h>

#include <EASTL/span.h>
#include <EASTL/utility.h>
#include <EASTL/vector.h>

namespace Urho3D
{

/// Binned-SAH bounding volume hierarchy over item boxes.
///
/// The nodes are in one flat array. The children of an inner node are a pair at first_ and first_ + 1.
/// The build and the traversal follow the C# reference exactly, so the order of the results is the same.
/// The face pair order of CollectOverlaps defines the crossing ids of the boolean.
///
/// An item with a NaN value in its box never overlaps anything. The build leaves such an item out,
/// so no query returns it. The other items keep their order. This keeps a NaN out of the node boxes.
///
/// The tree does not copy the item boxes. The caller keeps the box arrays alive and unchanged.
/// A tree is immutable after the build. Queries are const and thread safe if each thread has its own scratch.
class PLUGIN_RBFXCSG_API CsgFaceBvh
{
public:
    /// Largest number of items in a leaf. A node with more items is split.
    static constexpr int MAX_LEAF_SIZE = 4;
    /// Number of bins for each axis in the SAH build.
    static constexpr int BIN_COUNT = 8;

    /// One node. The count is zero for an inner node.
    struct Node
    {
        Vector3 min_;
        int first_{};
        Vector3 max_;
        int count_{};
    };

    /// Pair of item indices. The first is from the tree that receives the query.
    using ItemPair = ea::pair<int, int>;

    /// Build a tree over all the boxes.
    CsgFaceBvh(ea::span<const Vector3> itemMin, ea::span<const Vector3> itemMax);
    /// Build a tree over a subset of the boxes. The subset holds indices into the box arrays.
    CsgFaceBvh(ea::span<const Vector3> itemMin, ea::span<const Vector3> itemMax, ea::span<const unsigned> subset);

    /// Return the number of items in the tree. Items with a NaN box are not in the tree.
    unsigned ItemCount() const { return items_.size(); }
    /// Return the nodes. The root is the first node. An empty tree has one node that no query can hit.
    ea::span<const Node> GetNodes() const { return {nodes_.data(), nodes_.size()}; }
    /// Return the item indices in leaf order. A leaf owns the range [first_, first_ + count_).
    ea::span<const int> GetItems() const { return {items_.data(), items_.size()}; }

    /// Append every pair of items with boxes that overlap within epsilon.
    /// The pairs have the order of the C# traversal. The scratch is the traversal stack.
    void CollectOverlaps(const CsgFaceBvh& other, float epsilon, ea::vector<ItemPair>& output,
        ea::vector<ItemPair>& scratch) const;
    /// Append every pair of items with boxes that overlap within epsilon.
    void CollectOverlaps(const CsgFaceBvh& other, float epsilon, ea::vector<ItemPair>& output) const;

    /// Clear the output, then add the items with boxes that overlap the query box within epsilon.
    /// The scratch is the traversal stack.
    void CollectBox(const Vector3& min, const Vector3& max, float epsilon, ea::vector<int>& output,
        ea::vector<int>& scratch) const;
    /// Clear the output, then add the items with boxes that overlap the query box within epsilon.
    void CollectBox(const Vector3& min, const Vector3& max, float epsilon, ea::vector<int>& output) const;
    /// Clear the output, then add the items with boxes that overlap the query box.
    void CollectBox(const Vector3& min, const Vector3& max, ea::vector<int>& output) const;

private:
    struct Primitive
    {
        Vector3 min_;
        int index_{};
        Vector3 max_;
    };

    /// Remove the items with a NaN box, then build the nodes.
    void Initialize();
    void Build(ea::vector<Primitive>& primitives);

    ea::span<const Vector3> itemMin_;
    ea::span<const Vector3> itemMax_;
    ea::vector<Node> nodes_;
    ea::vector<int> items_;
};

} // namespace Urho3D
