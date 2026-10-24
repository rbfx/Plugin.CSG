// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgFaceBvh.h"

#include <climits>
#include <limits>

namespace Urho3D
{

namespace
{

constexpr float INF = std::numeric_limits<float>::infinity();
constexpr int BIN_COUNT = CsgFaceBvh::BIN_COUNT;

Vector3 MinOf(const Vector3& a, const Vector3& b)
{
    return Vector3(a.x_ < b.x_ ? a.x_ : b.x_, a.y_ < b.y_ ? a.y_ : b.y_, a.z_ < b.z_ ? a.z_ : b.z_);
}

Vector3 MaxOf(const Vector3& a, const Vector3& b)
{
    return Vector3(a.x_ > b.x_ ? a.x_ : b.x_, a.y_ > b.y_ ? a.y_ : b.y_, a.z_ > b.z_ ? a.z_ : b.z_);
}

float Component(const Vector3& value, int axis)
{
    return axis == 0 ? value.x_ : axis == 1 ? value.y_ : value.z_;
}

float HalfArea(const Vector3& min, const Vector3& max)
{
    const float x = max.x_ - min.x_;
    const float y = max.y_ - min.y_;
    const float z = max.z_ - min.z_;
    return x * y + y * z + z * x;
}

bool IsNumber(const Vector3& value)
{
    return value.x_ == value.x_ && value.y_ == value.y_ && value.z_ == value.z_;
}

/// Convert with truncation. NaN gives zero. Out of range values saturate.
int ToInt(float value)
{
    if (!(value == value))
        return 0;
    if (value >= 2147483520.0f)
        return INT_MAX;
    if (value <= -2147483648.0f)
        return INT_MIN;
    return static_cast<int>(value);
}

/// The test is the same on both sides, with the tolerance on the maximum corner only.
bool Overlaps(const Vector3& minA, const Vector3& maxA, const Vector3& minB, const Vector3& maxB, float epsilon)
{
    return minA.x_ <= maxB.x_ + epsilon && minA.y_ <= maxB.y_ + epsilon && minA.z_ <= maxB.z_ + epsilon
        && maxA.x_ + epsilon >= minB.x_ && maxA.y_ + epsilon >= minB.y_ && maxA.z_ + epsilon >= minB.z_;
}

} // namespace

CsgFaceBvh::CsgFaceBvh(ea::span<const Vector3> itemMin, ea::span<const Vector3> itemMax)
    : itemMin_(itemMin)
    , itemMax_(itemMax)
{
    const unsigned count = itemMin.size();
    items_.resize(count);
    for (unsigned i = 0; i < count; ++i)
        items_[i] = static_cast<int>(i);
    Initialize();
}

CsgFaceBvh::CsgFaceBvh(ea::span<const Vector3> itemMin, ea::span<const Vector3> itemMax, ea::span<const unsigned> subset)
    : itemMin_(itemMin)
    , itemMax_(itemMax)
{
    const unsigned count = subset.size();
    items_.resize(count);
    for (unsigned i = 0; i < count; ++i)
        items_[i] = static_cast<int>(subset[i]);
    Initialize();
}

void CsgFaceBvh::Initialize()
{
    // A box with a NaN value never overlaps anything, but it would give a node box that depends on the item order.
    // Such an item leaves the tree. The remaining items keep their order.
    unsigned count = 0;
    for (unsigned i = 0; i < items_.size(); ++i)
    {
        const int item = items_[i];
        if (IsNumber(itemMin_[item]) && IsNumber(itemMax_[item]))
            items_[count++] = item;
    }
    items_.resize(count);

    nodes_.resize(count * 2 > 1 ? count * 2 : 1);
    if (count == 0)
    {
        // An empty box that no query can hit.
        nodes_[0].min_ = Vector3(INF, INF, INF);
        nodes_[0].max_ = Vector3(-INF, -INF, -INF);
        return;
    }

    // The build works on a contiguous copy of the boxes that it partitions in place.
    // Every pass reads memory in sequence instead of through the index array.
    ea::vector<Primitive> primitives(count);
    for (unsigned i = 0; i < count; ++i)
    {
        const int item = items_[i];
        primitives[i].min_ = itemMin_[item];
        primitives[i].index_ = item;
        primitives[i].max_ = itemMax_[item];
    }
    Build(primitives);
    for (unsigned i = 0; i < count; ++i)
        items_[i] = primitives[i].index_;
}

void CsgFaceBvh::Build(ea::vector<Primitive>& primitives)
{
    struct Bin
    {
        Vector3 min_{INF, INF, INF};
        Vector3 max_{-INF, -INF, -INF};
    };

    const auto updateBounds = [&](Node& node)
    {
        Vector3 min(INF, INF, INF);
        Vector3 max(-INF, -INF, -INF);
        for (int i = node.first_; i < node.first_ + node.count_; ++i)
        {
            min = MinOf(min, primitives[i].min_);
            max = MaxOf(max, primitives[i].max_);
        }
        node.min_ = min;
        node.max_ = max;
    };

    int nodeCount = 1;
    nodes_[0].first_ = 0;
    nodes_[0].count_ = static_cast<int>(primitives.size());
    updateBounds(nodes_[0]);

    // Bins for all three axes. One pass over the items of a node fills them.
    Bin bins[3 * BIN_COUNT];
    int binCount[3 * BIN_COUNT];
    float rightArea[BIN_COUNT];
    int rightCount[BIN_COUNT];
    Vector3 rightMin[BIN_COUNT];
    Vector3 rightMax[BIN_COUNT];

    ea::vector<int> pending;
    pending.push_back(0);
    while (!pending.empty())
    {
        const int nodeIndex = pending.back();
        pending.pop_back();
        if (nodes_[nodeIndex].count_ <= MAX_LEAF_SIZE)
            continue;
        const int first = nodes_[nodeIndex].first_;
        const int end = first + nodes_[nodeIndex].count_;

        // Bin the doubled centroids (min + max) over the doubled box of the node.
        // This skips a pass that would compute tighter centroid bounds.
        const Vector3 nodeMin = nodes_[nodeIndex].min_;
        const Vector3 nodeMax = nodes_[nodeIndex].max_;
        const Vector3 centroidMin(nodeMin.x_ + nodeMin.x_, nodeMin.y_ + nodeMin.y_, nodeMin.z_ + nodeMin.z_);
        const float extent[3] = {
            (nodeMax.x_ - nodeMin.x_) * 2.0f, (nodeMax.y_ - nodeMin.y_) * 2.0f, (nodeMax.z_ - nodeMin.z_) * 2.0f};
        // A flat axis gets scale zero. Every item falls in bin 0 and the axis offers no split.
        const Vector3 scale(extent[0] > 0.0f ? BIN_COUNT / extent[0] : 0.0f,
            extent[1] > 0.0f ? BIN_COUNT / extent[1] : 0.0f, extent[2] > 0.0f ? BIN_COUNT / extent[2] : 0.0f);

        for (Bin& bin : bins)
            bin = Bin();
        for (int& count : binCount)
            count = 0;
        for (int i = first; i < end; ++i)
        {
            const Primitive& primitive = primitives[i];
            for (int axis = 0; axis < 3; ++axis)
            {
                const float center = Component(primitive.min_, axis) + Component(primitive.max_, axis);
                int bin = ToInt((center - Component(centroidMin, axis)) * Component(scale, axis));
                bin = bin < 0 ? 0 : (bin > BIN_COUNT - 1 ? BIN_COUNT - 1 : bin);
                const int index = axis * BIN_COUNT + bin;
                ++binCount[index];
                bins[index].min_ = MinOf(bins[index].min_, primitive.min_);
                bins[index].max_ = MaxOf(bins[index].max_, primitive.max_);
            }
        }

        int bestAxis = -1;
        int bestSplit = 0;
        float bestCost = INF;
        Vector3 bestLeftMin, bestLeftMax, bestRightMin, bestRightMax;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (Component(scale, axis) == 0.0f)
                continue;
            const int offset = axis * BIN_COUNT;
            Vector3 sweepMin(INF, INF, INF);
            Vector3 sweepMax(-INF, -INF, -INF);
            int sweepCount = 0;
            for (int bin = BIN_COUNT - 1; bin > 0; --bin)
            {
                sweepMin = MinOf(sweepMin, bins[offset + bin].min_);
                sweepMax = MaxOf(sweepMax, bins[offset + bin].max_);
                sweepCount += binCount[offset + bin];
                rightMin[bin] = sweepMin;
                rightMax[bin] = sweepMax;
                rightArea[bin] = sweepCount == 0 ? 0.0f : HalfArea(sweepMin, sweepMax);
                rightCount[bin] = sweepCount;
            }
            sweepMin = Vector3(INF, INF, INF);
            sweepMax = Vector3(-INF, -INF, -INF);
            sweepCount = 0;
            for (int bin = 0; bin < BIN_COUNT - 1; ++bin)
            {
                sweepMin = MinOf(sweepMin, bins[offset + bin].min_);
                sweepMax = MaxOf(sweepMax, bins[offset + bin].max_);
                sweepCount += binCount[offset + bin];
                if (sweepCount == 0 || rightCount[bin + 1] == 0)
                    continue;
                const float cost = static_cast<float>(sweepCount) * HalfArea(sweepMin, sweepMax)
                    + static_cast<float>(rightCount[bin + 1]) * rightArea[bin + 1];
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = bin + 1;
                    bestLeftMin = sweepMin;
                    bestLeftMax = sweepMax;
                    bestRightMin = rightMin[bin + 1];
                    bestRightMax = rightMax[bin + 1];
                }
            }
        }

        int middle;
        if (bestAxis < 0)
        {
            // The centroids coincide. Split by the position in the item range to bound the leaf size.
            middle = first + nodes_[nodeIndex].count_ / 2;
        }
        else
        {
            // Move the primitives in the bins below the split to the front.
            const float low = Component(centroidMin, bestAxis);
            const float axisScale = Component(scale, bestAxis);
            int left = first;
            int last = end - 1;
            while (left <= last)
            {
                const Primitive& primitive = primitives[left];
                const float center = Component(primitive.min_, bestAxis) + Component(primitive.max_, bestAxis);
                int bin = ToInt((center - low) * axisScale);
                bin = bin < BIN_COUNT - 1 ? bin : BIN_COUNT - 1;
                if (bin < bestSplit)
                    ++left;
                else
                {
                    ea::swap(primitives[left], primitives[last]);
                    --last;
                }
            }
            middle = left;
        }

        const int left = nodeCount;
        nodeCount += 2;
        nodes_[left].first_ = first;
        nodes_[left].count_ = middle - first;
        nodes_[left + 1].first_ = middle;
        nodes_[left + 1].count_ = end - middle;
        if (bestAxis < 0)
        {
            updateBounds(nodes_[left]);
            updateBounds(nodes_[left + 1]);
        }
        else
        {
            nodes_[left].min_ = bestLeftMin;
            nodes_[left].max_ = bestLeftMax;
            nodes_[left + 1].min_ = bestRightMin;
            nodes_[left + 1].max_ = bestRightMax;
        }
        nodes_[nodeIndex].first_ = left;
        nodes_[nodeIndex].count_ = 0;
        pending.push_back(left + 1);
        pending.push_back(left);
    }
    nodes_.resize(nodeCount);
}

void CsgFaceBvh::CollectOverlaps(const CsgFaceBvh& other, float epsilon, ea::vector<ItemPair>& output) const
{
    ea::vector<ItemPair> scratch;
    CollectOverlaps(other, epsilon, output, scratch);
}

void CsgFaceBvh::CollectOverlaps(
    const CsgFaceBvh& other, float epsilon, ea::vector<ItemPair>& output, ea::vector<ItemPair>& stack) const
{
    if (items_.empty() || other.items_.empty())
        return;
    stack.clear();
    stack.push_back({0, 0});
    while (!stack.empty())
    {
        const ItemPair pair = stack.back();
        stack.pop_back();
        const Node& a = nodes_[pair.first];
        const Node& b = other.nodes_[pair.second];
        if (!Overlaps(a.min_, a.max_, b.min_, b.max_, epsilon))
            continue;
        if (a.count_ != 0 && b.count_ != 0)
        {
            for (int i = a.first_; i < a.first_ + a.count_; ++i)
            {
                const int itemA = items_[i];
                const Vector3& minA = itemMin_[itemA];
                const Vector3& maxA = itemMax_[itemA];
                for (int j = b.first_; j < b.first_ + b.count_; ++j)
                {
                    const int itemB = other.items_[j];
                    if (Overlaps(minA, maxA, other.itemMin_[itemB], other.itemMax_[itemB], epsilon))
                        output.push_back({itemA, itemB});
                }
            }
        }
        else if (b.count_ != 0 || (a.count_ == 0 && HalfArea(a.min_, a.max_) >= HalfArea(b.min_, b.max_)))
        {
            // Visit the first child first.
            stack.push_back({a.first_ + 1, pair.second});
            stack.push_back({a.first_, pair.second});
        }
        else
        {
            stack.push_back({pair.first, b.first_ + 1});
            stack.push_back({pair.first, b.first_});
        }
    }
}

void CsgFaceBvh::CollectBox(const Vector3& min, const Vector3& max, float epsilon, ea::vector<int>& output) const
{
    ea::vector<int> scratch;
    scratch.reserve(64);
    CollectBox(min, max, epsilon, output, scratch);
}

void CsgFaceBvh::CollectBox(const Vector3& min, const Vector3& max, ea::vector<int>& output) const
{
    CollectBox(min, max, 0.0f, output);
}

void CsgFaceBvh::CollectBox(
    const Vector3& min, const Vector3& max, float epsilon, ea::vector<int>& output, ea::vector<int>& stack) const
{
    output.clear();
    if (items_.empty())
        return;
    stack.clear();
    stack.push_back(0);
    while (!stack.empty())
    {
        const Node& node = nodes_[stack.back()];
        stack.pop_back();
        if (!Overlaps(node.min_, node.max_, min, max, epsilon))
            continue;
        if (node.count_ == 0)
        {
            stack.push_back(node.first_ + 1);
            stack.push_back(node.first_);
            continue;
        }
        for (int i = node.first_; i < node.first_ + node.count_; ++i)
        {
            const int item = items_[i];
            if (Overlaps(itemMin_[item], itemMax_[item], min, max, epsilon))
                output.push_back(item);
        }
    }
}

} // namespace Urho3D
