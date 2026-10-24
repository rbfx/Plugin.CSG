// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgCommon.h"
#include <Urho3D/Math/Vector3.h>

#include <EASTL/functional.h>
#include <EASTL/type_traits.h>
#include <EASTL/vector.h>

#include <cstdint>
#include <cstring>

namespace Urho3D
{

/// Hash map with open addressing and linear probing. The map has no erase.
///
/// The map keeps the entries in a dense vector, in insertion order.
/// The hash table stores only int32 indices into this vector.
/// Iteration walks the dense vector. The result never depends on the hash table order.
///
/// An insertion can move the entries. It invalidates all pointers, references and iterators.
/// The table grows when the load factor passes 7/10. Capacity is always a power of two.
/// Find does not allocate memory.
template <class Key, class Value, class Hash, class Equal = ea::equal_to<Key>> class CsgFlatMap
{
    static_assert(ea::is_trivially_copyable<Key>::value, "CsgFlatMap needs a trivially copyable key.");

public:
    /// Key and value pair in the dense vector.
    struct Entry
    {
        Key key_;
        Value value_;
    };

    using Iterator = typename ea::vector<Entry>::iterator;
    using ConstIterator = typename ea::vector<Entry>::const_iterator;

    /// Return the value of the key. Add a value-initialized value if the key is missing.
    /// Set existed to true if the key was already in the map.
    Value& FindOrAdd(const Key& key, bool& existed)
    {
        if (!table_.empty())
        {
            const int32_t found = FindIndex(key);
            if (found >= 0)
            {
                existed = true;
                return entries_[found].value_;
            }
        }
        existed = false;
        return Add(key, Value{});
    }

    /// Return the value of the key or null.
    Value* Find(const Key& key)
    {
        if (table_.empty())
            return nullptr;
        const int32_t found = FindIndex(key);
        return found >= 0 ? &entries_[found].value_ : nullptr;
    }

    /// Return the value of the key or null.
    const Value* Find(const Key& key) const
    {
        if (table_.empty())
            return nullptr;
        const int32_t found = FindIndex(key);
        return found >= 0 ? &entries_[found].value_ : nullptr;
    }

    /// Add the key and value if the key is missing. Return true if the map added the entry.
    bool TryAdd(const Key& key, const Value& value)
    {
        if (!table_.empty() && FindIndex(key) >= 0)
            return false;
        Add(key, value);
        return true;
    }

    /// Remove all entries. The map keeps the memory.
    void Clear()
    {
        entries_.clear();
        for (int32_t& slot : table_)
            slot = EMPTY_SLOT;
    }

    /// Make room for n entries without a rehash.
    void Reserve(size_t n)
    {
        entries_.reserve(n);
        size_t capacity = table_.size() > MIN_CAPACITY ? table_.size() : MIN_CAPACITY;
        while (n * 10 > capacity * 7)
            capacity *= 2;
        if (capacity > table_.size())
            Rehash(capacity);
    }

    /// Return the number of entries.
    size_t Size() const { return entries_.size(); }

    /// Iterate the entries in insertion order.
    Iterator begin() { return entries_.begin(); }
    Iterator end() { return entries_.end(); }
    ConstIterator begin() const { return entries_.begin(); }
    ConstIterator end() const { return entries_.end(); }

private:
    static constexpr int32_t EMPTY_SLOT = -1;
    static constexpr size_t MIN_CAPACITY = 16;

    int32_t FindIndex(const Key& key) const
    {
        const size_t mask = table_.size() - 1;
        size_t slot = Hash{}(key) & mask;
        for (;;)
        {
            const int32_t index = table_[slot];
            if (index == EMPTY_SLOT)
                return EMPTY_SLOT;
            if (Equal{}(entries_[index].key_, key))
                return index;
            slot = (slot + 1) & mask;
        }
    }

    Value& Add(const Key& key, const Value& value)
    {
        if ((entries_.size() + 1) * 10 > table_.size() * 7)
            Rehash(table_.empty() ? MIN_CAPACITY : table_.size() * 2);

        const size_t mask = table_.size() - 1;
        size_t slot = Hash{}(key) & mask;
        while (table_[slot] != EMPTY_SLOT)
            slot = (slot + 1) & mask;
        table_[slot] = static_cast<int32_t>(entries_.size());
        entries_.push_back(Entry{key, value});
        return entries_.back().value_;
    }

    void Rehash(size_t capacity)
    {
        table_.assign(capacity, EMPTY_SLOT);
        const size_t mask = capacity - 1;
        for (size_t i = 0; i < entries_.size(); ++i)
        {
            size_t slot = Hash{}(entries_[i].key_) & mask;
            while (table_[slot] != EMPTY_SLOT)
                slot = (slot + 1) & mask;
            table_[slot] = static_cast<int32_t>(i);
        }
    }

    ea::vector<Entry> entries_;
    ea::vector<int32_t> table_;
};

/// Hash set built on CsgFlatMap. It has the same ordering and invalidation rules.
template <class Key, class Hash, class Equal = ea::equal_to<Key>> class CsgFlatSet
{
    struct Empty
    {
    };
    using Map = CsgFlatMap<Key, Empty, Hash, Equal>;

public:
    using Iterator = typename Map::Iterator;
    using ConstIterator = typename Map::ConstIterator;

    /// Add the key. Return true if the key was not in the set.
    bool Insert(const Key& key) { return map_.TryAdd(key, Empty{}); }
    /// Return true if the set has the key.
    bool Contains(const Key& key) const { return map_.Find(key) != nullptr; }
    /// Remove all keys. The set keeps the memory.
    void Clear() { map_.Clear(); }
    /// Make room for n keys.
    void Reserve(size_t n) { map_.Reserve(n); }
    /// Return the number of keys.
    size_t Size() const { return map_.Size(); }

    /// Iterate the entries in insertion order. Use entry.key_.
    Iterator begin() { return map_.begin(); }
    Iterator end() { return map_.end(); }
    ConstIterator begin() const { return map_.begin(); }
    ConstIterator end() const { return map_.end(); }

private:
    Map map_;
};

/// Exact bit key of a Vector3. The value -0.0f gets the same key as +0.0f, as in .NET.
/// A NaN uses its bit pattern.
struct CsgVec3Key
{
    uint32_t x_{};
    uint32_t y_{};
    uint32_t z_{};

    CsgVec3Key() = default;
    explicit CsgVec3Key(const Vector3& v)
        : x_(ToBits(v.x_))
        , y_(ToBits(v.y_))
        , z_(ToBits(v.z_))
    {
    }

    bool operator==(const CsgVec3Key& rhs) const { return x_ == rhs.x_ && y_ == rhs.y_ && z_ == rhs.z_; }
    bool operator!=(const CsgVec3Key& rhs) const { return !(*this == rhs); }

private:
    static uint32_t ToBits(float value)
    {
        // The comparison is true for both zeros. Then the key uses +0.0f.
        if (value == 0.0f)
            value = 0.0f;
        uint32_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
};

/// Hash of CsgVec3Key. It mixes the three words with the CsgPackedKeyHash avalanche.
struct CsgVec3KeyHash
{
    size_t operator()(const CsgVec3Key& key) const noexcept
    {
        const CsgPackedKeyHash mix;
        uint64_t h = mix((static_cast<uint64_t>(key.y_) << 32) | key.x_);
        h ^= key.z_ + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return mix(h);
    }
};

/// Crossing of an edge of one operand through a triangle of the other.
/// The edge has the welded ids low_ and high_. The triangle belongs to the other operand.
struct CsgCrossingKey
{
    int low_{};
    int high_{};
    int triangle_{};
    bool edgeInSecond_{};

    bool operator==(const CsgCrossingKey& rhs) const
    {
        return low_ == rhs.low_ && high_ == rhs.high_ && triangle_ == rhs.triangle_
            && edgeInSecond_ == rhs.edgeInSecond_;
    }
    bool operator!=(const CsgCrossingKey& rhs) const { return !(*this == rhs); }
};

/// Hash of CsgCrossingKey.
struct CsgCrossingKeyHash
{
    size_t operator()(const CsgCrossingKey& key) const noexcept
    {
        const CsgPackedKeyHash mix;
        uint64_t h = mix((static_cast<uint64_t>(static_cast<uint32_t>(key.high_)) << 32)
            | static_cast<uint32_t>(key.low_));
        const uint64_t rest = (static_cast<uint64_t>(static_cast<uint32_t>(key.triangle_)) << 1)
            | (key.edgeInSecond_ ? 1u : 0u);
        h ^= rest + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return mix(h);
    }
};

}   // namespace Urho3D
