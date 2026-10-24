// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgFlatMap.h"

#include <catch2/catch_amalgamated.hpp>

using namespace Urho3D;

namespace
{

using PackedMap = CsgFlatMap<uint64_t, int, CsgPackedKeyHash>;

/// Hash that puts all keys into one bucket.
struct ConstantHash
{
    size_t operator()(uint64_t) const noexcept { return 7; }
};

uint64_t MakeKey(unsigned i)
{
    return CsgDirectedKey(i * 3u + 1u, i * 7u + 5u);
}

} // namespace

TEST_CASE("Flat map inserts and finds many keys", "[flatmap]")
{
    PackedMap map;
    constexpr unsigned count = 100000;
    for (unsigned i = 0; i < count; ++i)
    {
        bool existed = true;
        map.FindOrAdd(MakeKey(i), existed) = static_cast<int>(i);
        REQUIRE_FALSE(existed);
    }
    CHECK(map.Size() == count);
    for (unsigned i = 0; i < count; ++i)
    {
        const int* value = map.Find(MakeKey(i));
        REQUIRE(value != nullptr);
        REQUIRE(*value == static_cast<int>(i));
    }
    const PackedMap& constMap = map;
    CHECK(constMap.Find(MakeKey(5)) != nullptr);
}

TEST_CASE("Flat map returns null for missing keys", "[flatmap]")
{
    PackedMap map;
    CHECK(map.Find(1) == nullptr);
    for (unsigned i = 0; i < 1000; ++i)
        map.TryAdd(i * 2, 1);
    for (unsigned i = 0; i < 1000; ++i)
        CHECK(map.Find(i * 2 + 1) == nullptr);
}

TEST_CASE("Flat map FindOrAdd reports existence", "[flatmap]")
{
    PackedMap map;
    bool existed = true;
    int& first = map.FindOrAdd(42, existed);
    CHECK_FALSE(existed);
    CHECK(first == 0);
    first = 9;
    int& second = map.FindOrAdd(42, existed);
    CHECK(existed);
    CHECK(second == 9);
    CHECK(map.Size() == 1);
}

TEST_CASE("Flat map TryAdd keeps the first value", "[flatmap]")
{
    PackedMap map;
    CHECK(map.TryAdd(3, 10));
    CHECK_FALSE(map.TryAdd(3, 20));
    REQUIRE(map.Find(3) != nullptr);
    CHECK(*map.Find(3) == 10);
}

TEST_CASE("Flat map Clear keeps the map usable", "[flatmap]")
{
    PackedMap map;
    for (unsigned i = 0; i < 5000; ++i)
        map.TryAdd(i, static_cast<int>(i));
    map.Clear();
    CHECK(map.Size() == 0);
    CHECK(map.Find(10) == nullptr);
    CHECK(map.begin() == map.end());
    for (unsigned i = 0; i < 5000; ++i)
        CHECK(map.TryAdd(i, static_cast<int>(i) + 1));
    for (unsigned i = 0; i < 5000; ++i)
    {
        REQUIRE(map.Find(i) != nullptr);
        REQUIRE(*map.Find(i) == static_cast<int>(i) + 1);
    }
}

TEST_CASE("Flat map growth keeps entries valid and insertion order", "[flatmap]")
{
    PackedMap map;
    constexpr unsigned count = 3000;
    for (unsigned i = 0; i < count; ++i)
        map.TryAdd(MakeKey(i), static_cast<int>(i));

    unsigned expected = 0;
    for (const auto& entry : map)
    {
        REQUIRE(entry.key_ == MakeKey(expected));
        REQUIRE(entry.value_ == static_cast<int>(expected));
        ++expected;
    }
    CHECK(expected == count);
}

TEST_CASE("Flat map Reserve does not change the content", "[flatmap]")
{
    PackedMap map;
    map.TryAdd(1, 1);
    map.Reserve(10000);
    CHECK(map.Size() == 1);
    REQUIRE(map.Find(1) != nullptr);
    for (unsigned i = 2; i < 10000; ++i)
        map.TryAdd(i, 0);
    CHECK(map.Size() == 9999);
    CHECK(*map.Find(1) == 1);
}

TEST_CASE("CsgVec3Key treats signed zeros as equal", "[flatmap]")
{
    const CsgVec3Key positive(Vector3(0.0f, 0.0f, 0.0f));
    const CsgVec3Key negative(Vector3(-0.0f, -0.0f, -0.0f));
    CHECK(positive == negative);
    CHECK(CsgVec3KeyHash{}(positive) == CsgVec3KeyHash{}(negative));
    CHECK(CsgVec3Key(Vector3(1.0f, 2.0f, 3.0f)) == CsgVec3Key(Vector3(1.0f, 2.0f, 3.0f)));
    CHECK(CsgVec3Key(Vector3(1.0f, 2.0f, 3.0f)) != CsgVec3Key(Vector3(1.0f, 2.0f, 3.0001f)));
    CHECK(CsgVec3Key(Vector3(1.0f, 2.0f, 3.0f)) != CsgVec3Key(Vector3(2.0f, 1.0f, 3.0f)));
    CHECK(CsgVec3Key(Vector3(1.0f, 0.0f, 0.0f)) != CsgVec3Key(Vector3(0.0f, 1.0f, 0.0f)));

    CsgFlatMap<CsgVec3Key, int, CsgVec3KeyHash> map;
    bool existed = true;
    map.FindOrAdd(positive, existed) = 4;
    CHECK_FALSE(existed);
    map.FindOrAdd(negative, existed);
    CHECK(existed);
    CHECK(map.Size() == 1);
}

TEST_CASE("CsgCrossingKey compares each field", "[flatmap]")
{
    const CsgCrossingKey base{1, 2, 3, false};
    CHECK(base == CsgCrossingKey{1, 2, 3, false});
    CHECK(base != CsgCrossingKey{0, 2, 3, false});
    CHECK(base != CsgCrossingKey{1, 0, 3, false});
    CHECK(base != CsgCrossingKey{1, 2, 0, false});
    CHECK(base != CsgCrossingKey{1, 2, 3, true});
    CHECK(CsgCrossingKeyHash{}(base) == CsgCrossingKeyHash{}(CsgCrossingKey{1, 2, 3, false}));

    CsgFlatMap<CsgCrossingKey, int, CsgCrossingKeyHash> map;
    for (int i = 0; i < 2000; ++i)
    {
        CHECK(map.TryAdd(CsgCrossingKey{i, i + 1, i % 7, false}, i));
        CHECK(map.TryAdd(CsgCrossingKey{i, i + 1, i % 7, true}, -i));
    }
    CHECK(map.Size() == 4000);
    REQUIRE(map.Find(CsgCrossingKey{10, 11, 3, true}) != nullptr);
    CHECK(*map.Find(CsgCrossingKey{10, 11, 3, true}) == -10);
}

TEST_CASE("Flat map works with a heavy-collision hash", "[flatmap]")
{
    CsgFlatMap<uint64_t, int, ConstantHash> map;
    for (unsigned i = 0; i < 1500; ++i)
        CHECK(map.TryAdd(i, static_cast<int>(i)));
    for (unsigned i = 0; i < 1500; ++i)
    {
        REQUIRE(map.Find(i) != nullptr);
        REQUIRE(*map.Find(i) == static_cast<int>(i));
    }
    CHECK(map.Find(1500) == nullptr);
    CHECK(map.Size() == 1500);
}

TEST_CASE("Flat set basics", "[flatmap]")
{
    CsgFlatSet<uint64_t, CsgPackedKeyHash> set;
    CHECK_FALSE(set.Contains(1));
    CHECK(set.Insert(5));
    CHECK(set.Insert(1));
    CHECK_FALSE(set.Insert(5));
    CHECK(set.Contains(1));
    CHECK(set.Contains(5));
    CHECK_FALSE(set.Contains(2));
    CHECK(set.Size() == 2);

    uint64_t order[2];
    unsigned n = 0;
    for (const auto& entry : set)
        order[n++] = entry.key_;
    REQUIRE(n == 2);
    CHECK(order[0] == 5);
    CHECK(order[1] == 1);

    set.Clear();
    CHECK(set.Size() == 0);
    CHECK_FALSE(set.Contains(5));
    for (unsigned i = 0; i < 10000; ++i)
        set.Insert(i);
    CHECK(set.Size() == 10000);
}

TEST_CASE("Flat map Clear after a big fill then a small fill gives correct results", "[flatmap]")
{
    PackedMap map;
    for (unsigned i = 0; i < 5000; ++i)
        map.TryAdd(MakeKey(i), static_cast<int>(i));
    for (unsigned round = 0; round < 200; ++round)
    {
        map.Clear();
        CHECK(map.Size() == 0);
        const unsigned count = 1 + round % 7;
        for (unsigned i = 0; i < count; ++i)
        {
            bool existed = true;
            map.FindOrAdd(MakeKey(i + round), existed) = static_cast<int>(round * 100 + i);
            REQUIRE_FALSE(existed);
        }
        REQUIRE(map.Size() == count);
        for (unsigned i = 0; i < count; ++i)
        {
            const int* value = map.Find(MakeKey(i + round));
            REQUIRE(value != nullptr);
            CHECK(*value == static_cast<int>(round * 100 + i));
        }
        // A key of the big fill is gone, and the dense vector keeps the insertion order.
        CHECK(map.Find(MakeKey(4000 + round)) == nullptr);
        unsigned expected = round;
        for (const auto& entry : map)
            CHECK(entry.key_ == MakeKey(expected++));
    }
}

TEST_CASE("Flat map Clear frees chains of one bucket", "[flatmap]")
{
    // All keys share one probe chain. The targeted clear frees slots in the middle of the chain.
    CsgFlatMap<uint64_t, int, ConstantHash> map;
    for (unsigned i = 0; i < 4000; ++i)
        map.TryAdd(i, 1);
    // Few entries after the fill: the next clear takes the targeted path only while entries * 8 < table size.
    for (unsigned round = 0; round < 50; ++round)
    {
        map.Clear();
        for (unsigned i = 0; i < 10; ++i)
            REQUIRE(map.TryAdd(i + round, static_cast<int>(i)));
        for (unsigned i = 0; i < 10; ++i)
            REQUIRE(map.Find(i + round) != nullptr);
        CHECK(map.Find(round + 10) == nullptr);
        CHECK(map.Find(3999) == nullptr);
    }
}

TEST_CASE("Flat set Clear is cheap after one big fill", "[flatmap]")
{
    CsgFlatSet<uint64_t, CsgPackedKeyHash> set;
    for (unsigned i = 0; i < 100000; ++i)
        set.Insert(MakeKey(i));
    CHECK(set.Size() == 100000);
    // A full reset of the table would cost about 1e5 * 2.6e5 slot writes. The targeted clear costs a few probes.
    unsigned long long checksum = 0;
    for (unsigned round = 0; round < 100000; ++round)
    {
        set.Clear();
        CHECK(set.Insert(MakeKey(round % 5)));
        CHECK(set.Insert(MakeKey(round % 5 + 5)));
        CHECK_FALSE(set.Insert(MakeKey(round % 5)));
        CHECK_FALSE(set.Contains(MakeKey(round % 5 + 10)));
        checksum += set.Size();
    }
    CHECK(checksum == 200000ull);
}
