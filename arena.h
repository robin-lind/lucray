// arena.h

// MIT License
//
// Copyright (c) 2022 Robin Lind
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef LUC_ARENA_H
#define LUC_ARENA_H

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <vector>

constexpr size_t KByte(size_t k)
{
    return k * 1024ull;
}

constexpr size_t MByte(size_t m)
{
    return KByte(m) * 1024ull;
}

constexpr size_t GByte(size_t g)
{
    return MByte(g) * 1024ull;
}

constexpr size_t DEFAULT_ARENA_SIZE = MByte(5);

struct arena_allocator {
    char *start;
    char *current;
    size_t size;
    size_t space_left;

    arena_allocator()
    {
        size = DEFAULT_ARENA_SIZE;
        space_left = size;
        start = static_cast<char *>(malloc(size));
        current = start;
    }

    arena_allocator(const size_t size_) :
      size(size_)
    {
        space_left = size;
        start = static_cast<char *>(malloc(size));
        current = start;
    }

    [[nodiscard]] size_t get_size_left() const
    {
        return space_left;
    }

    void clear()
    {
        space_left = size;
        current = start;
    }

    ~arena_allocator()
    {
        free(start);
    }

    void *alloc(size_t size)
    {
        space_left -= size;
        assert(space_left > 0);
        void *to_alloc = current;
        current += size;
        return to_alloc;
    }

    template<typename T>
    T *allocate()
    {
        constexpr size_t size = sizeof(T);
        size_t offset = current_offset_required_for_alignment_of<T>();
        space_left -= size + offset;
        assert(space_left > 0);
        current += offset;
        void *to_alloc = current;
        current += size;
        return static_cast<T *>(to_alloc);
    }

    template<typename T>
    size_t current_offset_required_for_alignment_of()
    {
        constexpr size_t alignment = alignof(T);
        size_t offset = (alignment - (reinterpret_cast<size_t>(current) % alignment)) & (alignment - 1);
        return offset;
    }
};

template<typename T, typename Counter = size_t>
struct arena_list {
    T *ptr = nullptr;
    Counter size = 0;

    arena_list() = default;

    arena_list(arena_allocator& arena, const size_t& count) :
      size(count)
    {
        ptr = reinterpret_cast<T *>(arena.alloc(count * sizeof(T)));
    }

    T& operator[](std::size_t idx)
    {
        return ptr[idx];
    }

    const T& operator[](std::size_t idx) const
    {
        return ptr[idx];
    }
};

template<typename T, typename Counter = size_t>
struct expanding_list {
    arena_allocator& arena;
    T *ptr = nullptr;
    Counter count = 0;

    expanding_list(arena_allocator& _arena) :
      arena(_arena), ptr(reinterpret_cast<T *>(_arena.current)) {}

    void emplace_back(const T& item)
    {
        T *space = arena.allocate<T>();
        *space = item;
        count++;
    }

    T& operator[](std::size_t idx)
    {
        return ptr[idx];
    }

    const T& operator[](std::size_t idx) const
    {
        return ptr[idx];
    }
};

#endif /* LUC_ARENA_H */
