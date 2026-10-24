// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgFaceBvh.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"

#include <catch2/catch_amalgamated.hpp>

#include <EASTL/algorithm.h>
#include <EASTL/sort.h>

#include <chrono>
#include <limits>
#include <random>

using namespace Urho3D;

namespace
{

struct Boxes
{
    ea::vector<Vector3> min_;
    ea::vector<Vector3> max_;
};

Boxes MakeRandomBoxes(unsigned count, unsigned seed, float domain, float maxSize)
{
    std::mt19937 random(seed);
    std::uniform_real_distribution<float> position(0.0f, domain);
    std::uniform_real_distribution<float> size(0.0f, maxSize);
    Boxes boxes;
    for (unsigned i = 0; i < count; ++i)
    {
        const Vector3 min(position(random), position(random), position(random));
        boxes.min_.push_back(min);
        boxes.max_.push_back(min + Vector3(size(random), size(random), size(random)));
    }
    return boxes;
}

ea::span<const Vector3> View(const ea::vector<Vector3>& values)
{
    return {values.data(), values.size()};
}

bool Overlaps(const Vector3& minA, const Vector3& maxA, const Vector3& minB, const Vector3& maxB, float epsilon)
{
    return minA.x_ <= maxB.x_ + epsilon && minA.y_ <= maxB.y_ + epsilon && minA.z_ <= maxB.z_ + epsilon
        && maxA.x_ + epsilon >= minB.x_ && maxA.y_ + epsilon >= minB.y_ && maxA.z_ + epsilon >= minB.z_;
}

float HalfArea(const Vector3& min, const Vector3& max)
{
    const float x = max.x_ - min.x_;
    const float y = max.y_ - min.y_;
    const float z = max.z_ - min.z_;
    return x * y + y * z + z * x;
}

using Pairs = ea::vector<ea::pair<int, int>>;

Pairs BruteForceOverlaps(const Boxes& a, const Boxes& b, float epsilon)
{
    Pairs result;
    for (unsigned i = 0; i < a.min_.size(); ++i)
    {
        for (unsigned j = 0; j < b.min_.size(); ++j)
        {
            if (Overlaps(a.min_[i], a.max_[i], b.min_[j], b.max_[j], epsilon))
                result.push_back({static_cast<int>(i), static_cast<int>(j)});
        }
    }
    return result;
}

/// Recursive version of the C# traversal. The explicit stack of the library must give the same order.
void ReferenceOverlaps(const CsgFaceBvh& a, const Boxes& boxesA, int nodeA, const CsgFaceBvh& b, const Boxes& boxesB,
    int nodeB, float epsilon, Pairs& output)
{
    const CsgFaceBvh::Node& na = a.GetNodes()[nodeA];
    const CsgFaceBvh::Node& nb = b.GetNodes()[nodeB];
    if (!Overlaps(na.min_, na.max_, nb.min_, nb.max_, epsilon))
        return;
    if (na.count_ != 0 && nb.count_ != 0)
    {
        for (int i = na.first_; i < na.first_ + na.count_; ++i)
        {
            const int itemA = a.GetItems()[i];
            for (int j = nb.first_; j < nb.first_ + nb.count_; ++j)
            {
                const int itemB = b.GetItems()[j];
                if (Overlaps(boxesA.min_[itemA], boxesA.max_[itemA], boxesB.min_[itemB], boxesB.max_[itemB], epsilon))
                    output.push_back({itemA, itemB});
            }
        }
    }
    else if (nb.count_ != 0 || (na.count_ == 0 && HalfArea(na.min_, na.max_) >= HalfArea(nb.min_, nb.max_)))
    {
        ReferenceOverlaps(a, boxesA, na.first_, b, boxesB, nodeB, epsilon, output);
        ReferenceOverlaps(a, boxesA, na.first_ + 1, b, boxesB, nodeB, epsilon, output);
    }
    else
    {
        ReferenceOverlaps(a, boxesA, nodeA, b, boxesB, nb.first_, epsilon, output);
        ReferenceOverlaps(a, boxesA, nodeA, b, boxesB, nb.first_ + 1, epsilon, output);
    }
}

Pairs Sorted(Pairs pairs)
{
    ea::sort(pairs.begin(), pairs.end());
    return pairs;
}

ea::vector<int> Sorted(ea::vector<int> values)
{
    ea::sort(values.begin(), values.end());
    return values;
}

} // namespace

TEST_CASE("Bvh structure", "[bvh]")
{
    const Boxes boxes = MakeRandomBoxes(500, 1, 10.0f, 1.0f);
    const CsgFaceBvh tree(View(boxes.min_), View(boxes.max_));
    REQUIRE(tree.ItemCount() == 500);

    // Every item is in exactly one leaf. Leaves are small and bounds contain their items.
    ea::vector<int> seen(500, 0);
    unsigned leafItems = 0;
    for (const CsgFaceBvh::Node& node : tree.GetNodes())
    {
        if (node.count_ == 0)
            continue;
        CHECK(node.count_ <= CsgFaceBvh::MAX_LEAF_SIZE);
        for (int i = node.first_; i < node.first_ + node.count_; ++i)
        {
            const int item = tree.GetItems()[i];
            ++seen[item];
            ++leafItems;
            CHECK(node.min_.x_ <= boxes.min_[item].x_);
            CHECK(node.min_.y_ <= boxes.min_[item].y_);
            CHECK(node.min_.z_ <= boxes.min_[item].z_);
            CHECK(node.max_.x_ >= boxes.max_[item].x_);
            CHECK(node.max_.y_ >= boxes.max_[item].y_);
            CHECK(node.max_.z_ >= boxes.max_[item].z_);
        }
    }
    CHECK(leafItems == 500);
    for (const int count : seen)
        CHECK(count == 1);
}

TEST_CASE("Bvh overlaps equal brute force", "[bvh]")
{
    for (const float epsilon : {0.0f, 0.05f})
    {
        const Boxes boxesA = MakeRandomBoxes(500, 2, 10.0f, 1.5f);
        const Boxes boxesB = MakeRandomBoxes(500, 3, 10.0f, 1.5f);
        const CsgFaceBvh treeA(View(boxesA.min_), View(boxesA.max_));
        const CsgFaceBvh treeB(View(boxesB.min_), View(boxesB.max_));

        Pairs found;
        treeA.CollectOverlaps(treeB, epsilon, found);
        const Pairs expected = BruteForceOverlaps(boxesA, boxesB, epsilon);
        CHECK(expected.size() > 100);
        CHECK(found.size() == expected.size());
        CHECK(Sorted(found) == Sorted(expected));

        // The order is the order of the C# traversal.
        Pairs reference;
        ReferenceOverlaps(treeA, boxesA, 0, treeB, boxesB, 0, epsilon, reference);
        CHECK(found == reference);

        // Two builds from the same input give the same sequence. The scratch can be reused.
        const CsgFaceBvh treeA2(View(boxesA.min_), View(boxesA.max_));
        const CsgFaceBvh treeB2(View(boxesB.min_), View(boxesB.max_));
        Pairs again;
        Pairs scratch;
        treeA2.CollectOverlaps(treeB2, epsilon, again, scratch);
        CHECK(again == found);
        again.clear();
        treeA2.CollectOverlaps(treeB2, epsilon, again, scratch);
        CHECK(again == found);
    }
}

TEST_CASE("Bvh self overlaps", "[bvh]")
{
    const Boxes boxes = MakeRandomBoxes(500, 4, 10.0f, 1.0f);
    const CsgFaceBvh tree(View(boxes.min_), View(boxes.max_));
    Pairs found;
    tree.CollectOverlaps(tree, 0.01f, found);
    CHECK(Sorted(found) == Sorted(BruteForceOverlaps(boxes, boxes, 0.01f)));
    Pairs reference;
    ReferenceOverlaps(tree, boxes, 0, tree, boxes, 0, 0.01f, reference);
    CHECK(found == reference);
}

TEST_CASE("Bvh box query equals brute force", "[bvh]")
{
    const Boxes boxes = MakeRandomBoxes(500, 5, 10.0f, 1.0f);
    const CsgFaceBvh tree(View(boxes.min_), View(boxes.max_));
    const Boxes queries = MakeRandomBoxes(100, 6, 10.0f, 3.0f);
    ea::vector<int> found;
    ea::vector<int> scratch;
    for (const float epsilon : {0.0f, 0.1f})
    {
        for (unsigned q = 0; q < queries.min_.size(); ++q)
        {
            tree.CollectBox(queries.min_[q], queries.max_[q], epsilon, found, scratch);
            ea::vector<int> expected;
            for (unsigned i = 0; i < boxes.min_.size(); ++i)
            {
                if (Overlaps(boxes.min_[i], boxes.max_[i], queries.min_[q], queries.max_[q], epsilon))
                    expected.push_back(static_cast<int>(i));
            }
            CHECK(Sorted(found) == expected);
            ea::vector<int> plain;
            tree.CollectBox(queries.min_[q], queries.max_[q], epsilon, plain);
            CHECK(plain == found);
        }
    }
}

TEST_CASE("Bvh over a subset", "[bvh]")
{
    const Boxes boxes = MakeRandomBoxes(200, 7, 10.0f, 1.0f);
    ea::vector<unsigned> subset;
    for (unsigned i = 0; i < 200; i += 3)
        subset.push_back(i);
    const CsgFaceBvh tree(View(boxes.min_), View(boxes.max_), {subset.data(), subset.size()});
    CHECK(tree.ItemCount() == subset.size());

    ea::vector<int> found;
    tree.CollectBox(Vector3(0.0f, 0.0f, 0.0f), Vector3(10.0f, 10.0f, 10.0f), found);
    ea::vector<int> expected;
    for (const unsigned item : subset)
        expected.push_back(static_cast<int>(item));
    CHECK(Sorted(found) == expected);
}

TEST_CASE("Bvh degenerate cases", "[bvh]")
{
    ea::vector<int> found;
    Pairs pairs;

    SECTION("Empty tree")
    {
        const Boxes none;
        const CsgFaceBvh empty(View(none.min_), View(none.max_));
        CHECK(empty.ItemCount() == 0);
        REQUIRE(empty.GetNodes().size() == 1);
        empty.CollectBox(Vector3(-100.0f, -100.0f, -100.0f), Vector3(100.0f, 100.0f, 100.0f), 1000.0f, found);
        CHECK(found.empty());

        const Boxes some = MakeRandomBoxes(10, 8, 5.0f, 1.0f);
        const CsgFaceBvh other(View(some.min_), View(some.max_));
        empty.CollectOverlaps(other, 1.0f, pairs);
        other.CollectOverlaps(empty, 1.0f, pairs);
        CHECK(pairs.empty());
    }

    SECTION("One box")
    {
        Boxes one;
        one.min_.push_back(Vector3(1.0f, 1.0f, 1.0f));
        one.max_.push_back(Vector3(2.0f, 2.0f, 2.0f));
        const CsgFaceBvh tree(View(one.min_), View(one.max_));
        tree.CollectBox(Vector3(1.5f, 1.5f, 1.5f), Vector3(3.0f, 3.0f, 3.0f), found);
        CHECK(found == ea::vector<int>{0});
        tree.CollectBox(Vector3(2.5f, 2.5f, 2.5f), Vector3(3.0f, 3.0f, 3.0f), found);
        CHECK(found.empty());
        tree.CollectBox(Vector3(2.5f, 2.5f, 2.5f), Vector3(3.0f, 3.0f, 3.0f), 0.6f, found);
        CHECK(found == ea::vector<int>{0});
        tree.CollectOverlaps(tree, 0.0f, pairs);
        CHECK(pairs == Pairs{{0, 0}});
    }

    SECTION("Identical boxes")
    {
        Boxes same;
        for (int i = 0; i < 37; ++i)
        {
            same.min_.push_back(Vector3(1.0f, 2.0f, 3.0f));
            same.max_.push_back(Vector3(2.0f, 3.0f, 4.0f));
        }
        const CsgFaceBvh tree(View(same.min_), View(same.max_));
        for (const CsgFaceBvh::Node& node : tree.GetNodes())
            CHECK(node.count_ <= CsgFaceBvh::MAX_LEAF_SIZE);
        tree.CollectBox(Vector3(1.5f, 2.5f, 3.5f), Vector3(1.5f, 2.5f, 3.5f), found);
        CHECK(found.size() == 37);
        tree.CollectOverlaps(tree, 0.0f, pairs);
        CHECK(pairs.size() == 37 * 37);
        CHECK(Sorted(pairs) == Sorted(BruteForceOverlaps(same, same, 0.0f)));
    }

    SECTION("Zero size boxes")
    {
        Boxes points = MakeRandomBoxes(300, 9, 4.0f, 0.0f);
        // Duplicate some points and put some on a line, so that axes are flat.
        for (int i = 0; i < 40; ++i)
        {
            points.min_.push_back(points.min_[i]);
            points.max_.push_back(points.max_[i]);
            points.min_.push_back(Vector3(static_cast<float>(i) * 0.1f, 1.0f, 1.0f));
            points.max_.push_back(Vector3(static_cast<float>(i) * 0.1f, 1.0f, 1.0f));
        }
        const CsgFaceBvh tree(View(points.min_), View(points.max_));
        Pairs reference;
        tree.CollectOverlaps(tree, 0.0f, pairs);
        ReferenceOverlaps(tree, points, 0, tree, points, 0, 0.0f, reference);
        CHECK(pairs == reference);
        CHECK(Sorted(pairs) == Sorted(BruteForceOverlaps(points, points, 0.0f)));
        tree.CollectOverlaps(tree, 0.25f, pairs);
        CHECK(pairs.size() > 0);
    }
}

TEST_CASE("Bvh timing", "[bvh]")
{
    const Boxes boxesA = MakeRandomBoxes(100000, 10, 100.0f, 1.0f);
    const Boxes boxesB = MakeRandomBoxes(100000, 11, 100.0f, 1.0f);
    using Clock = std::chrono::steady_clock;
    const auto millis = [](Clock::time_point from, Clock::time_point to)
    { return std::chrono::duration<double, std::milli>(to - from).count(); };

    const auto t0 = Clock::now();
    const CsgFaceBvh treeA(View(boxesA.min_), View(boxesA.max_));
    const CsgFaceBvh treeB(View(boxesB.min_), View(boxesB.max_));
    const auto t1 = Clock::now();
    Pairs pairs;
    treeA.CollectOverlaps(treeB, 0.0f, pairs);
    const auto t2 = Clock::now();
    ea::vector<int> found;
    ea::vector<int> scratch;
    size_t total = 0;
    for (unsigned i = 0; i < 100000; ++i)
    {
        treeA.CollectBox(boxesB.min_[i], boxesB.max_[i], 0.0f, found, scratch);
        total += found.size();
    }
    const auto t3 = Clock::now();
    INFO("Build of 2 x 100000 boxes: " << millis(t0, t1) << " ms");
    INFO("Overlap query, " << pairs.size() << " pairs: " << millis(t1, t2) << " ms");
    INFO("100000 box queries, " << total << " hits: " << millis(t2, t3) << " ms");
    WARN("Bvh timing (info only): build of 2 x 100000 boxes " << millis(t0, t1) << " ms, overlap query "
                                                              << millis(t1, t2) << " ms (" << pairs.size()
                                                              << " pairs), 100000 box queries " << millis(t2, t3)
                                                              << " ms (" << total << " hits)");
    CHECK(total == pairs.size());
}

TEST_CASE("Solid lazy trees", "[bvh][solid]")
{
    const auto polygons = MakeBoxPolygons(Vector3(1.0f, 2.0f, 3.0f), Vector3(0.5f, 1.0f, 1.5f), 1);
    const CsgSolid solid(polygons);
    REQUIRE(solid.ComponentCount() == 1);

    CHECK(solid.BoundsOverlap(Vector3(1.0f, 2.0f, 2.0f), Vector3(1.2f, 2.2f, 2.2f), 0.0f));
    CHECK_FALSE(solid.BoundsOverlap(Vector3(5.0f, 2.0f, 2.0f), Vector3(6.0f, 2.2f, 2.2f), 0.0f));
    CHECK(solid.BoundsOverlap(Vector3(1.6f, 2.0f, 2.0f), Vector3(2.0f, 2.2f, 2.2f), 0.2f));

    const CsgFaceBvh& components = solid.GetComponentTree();
    CHECK(&components == &solid.GetComponentTree());
    CHECK(components.ItemCount() == 1);

    // Nothing is near this box, so the near tree is small and not kept.
    ea::unique_ptr<CsgFaceBvh> temporary;
    const CsgFaceBvh& none = solid.GetNearFaces(0, Vector3(9.0f, 9.0f, 9.0f), Vector3(9.5f, 9.5f, 9.5f), 0.0f, temporary);
    CHECK(&none == temporary.get());
    CHECK(none.ItemCount() == 0);

    // The box touches the whole solid, so the full tree is built and kept.
    const CsgFaceBvh& all = solid.GetNearFaces(0, Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f), 0.0f, temporary);
    CHECK(&all == &solid.GetFaceTree(0));
    CHECK(all.ItemCount() == 6);

    // With the full tree built, every query returns it.
    CHECK(&solid.GetNearFaces(0, Vector3(9.0f, 9.0f, 9.0f), Vector3(9.5f, 9.5f, 9.5f), 0.0f, temporary) == &all);

    ea::vector<int> found;
    all.CollectBox(Vector3(0.5f, 1.0f, 1.5f), Vector3(0.6f, 1.1f, 1.6f), found);
    CHECK(found.size() == 3);
}

TEST_CASE("Bvh leaves out NaN boxes", "[bvh]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Boxes boxes = MakeRandomBoxes(100, 12, 10.0f, 1.0f);
    boxes.min_[3].y_ = nan;
    boxes.max_[10].z_ = nan;
    boxes.min_[11] = Vector3(nan, nan, nan);
    boxes.max_[11] = Vector3(nan, nan, nan);
    const CsgFaceBvh tree(View(boxes.min_), View(boxes.max_));
    CHECK(tree.ItemCount() == 97);
    for (const CsgFaceBvh::Node& node : tree.GetNodes())
    {
        CHECK(node.min_.x_ == node.min_.x_);
        CHECK(node.max_.z_ == node.max_.z_);
    }
    ea::vector<int> found;
    tree.CollectBox(Vector3(-1.0f, -1.0f, -1.0f), Vector3(12.0f, 12.0f, 12.0f), found);
    CHECK(found.size() == 97);
    for (const int item : found)
        CHECK(item != 3);
}

TEST_CASE("Solid junctions with a NaN point", "[bvh][solid]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // A big quad and a small quad. The small quad has two vertices on the long bottom edge of the big quad,
    // so the edge has T-junctions. The NaN faces come first, so a NaN point has a low id.
    const auto quad = [](Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        ea::vector<ModelVertex> vertices;
        for (const Vector3& position : {a, b, c, d})
        {
            ModelVertex vertex;
            vertex.SetPosition(position);
            vertices.push_back(vertex);
        }
        return CsgPolygon(ea::move(vertices), 1);
    };
    // The big quad has a long bottom edge. Small quads below it have vertices on that edge, so the edge has
    // T-junctions. Faces with NaN vertices come first and between them, so NaN points have low ids and
    // sit between the finite points.
    constexpr int smallCount = 40;
    ea::vector<CsgPolygon> polygons;
    const auto addNan = [&](int i)
    {
        const float x = 100.0f + static_cast<float>(i);
        const Vector3 first = (i % 3 == 0) ? Vector3(nan, 5.0f, 5.0f) : Vector3(x, 5.0f, 5.0f);
        const Vector3 second = (i % 3 == 1) ? Vector3(x + 1.0f, nan, 5.0f) : Vector3(x + 1.0f, 5.0f, 5.0f);
        const Vector3 third = (i % 3 == 2) ? Vector3(x + 1.0f, 6.0f, nan) : Vector3(x + 1.0f, 6.0f, 5.0f);
        polygons.push_back(quad(first, second, third, Vector3(x, 6.0f, 5.0f)));
    };
    for (int i = 0; i < 10; ++i)
        addNan(i);
    const unsigned firstFinite = polygons.size();
    const float length = static_cast<float>(smallCount);
    polygons.push_back(quad(Vector3(0.0f, 0.0f, 0.0f), Vector3(length, 0.0f, 0.0f), Vector3(length, 1.0f, 0.0f),
        Vector3(0.0f, 1.0f, 0.0f)));
    for (int i = 0; i < smallCount; ++i)
    {
        const float x = static_cast<float>(i);
        polygons.push_back(quad(Vector3(x, 0.0f, 0.0f), Vector3(x + 1.0f, 0.0f, 0.0f), Vector3(x + 1.0f, -1.0f, 0.0f),
            Vector3(x, -1.0f, 0.0f)));
        addNan(10 + 3 * i);
        addNan(11 + 3 * i);
    }
    const unsigned finiteCount = 1 + smallCount;

    // The finite faces form one component, joined only through the junction points on the long edge.
    const CsgSolid solid(polygons);
    unsigned joined = 0;
    for (unsigned face = firstFinite; face < polygons.size(); ++face)
    {
        const bool finite = polygons[face].vertices_[0].GetPosition().x_ < 50.0f;
        if (finite && solid.GetFaceComponents()[face] == solid.GetFaceComponents()[firstFinite])
            ++joined;
    }
    CHECK(joined == finiteCount);

    // The face trees skip the faces with a NaN box.
    unsigned items = 0;
    for (unsigned component = 0; component < solid.ComponentCount(); ++component)
        items += solid.GetFaceTree(component).ItemCount();
    CHECK(items <= polygons.size());
    CHECK(items >= finiteCount);
}
