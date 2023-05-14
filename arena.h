// MIT License
//
// Copyright (c) 2023 Robin Lind
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

#ifndef ARENA_H
#define ARENA_H

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <vector>

namespace luc {
constexpr size_t KByte(size_t k)
{
    return k * 1000ull;
}

constexpr size_t MByte(size_t m)
{
    return KByte(m) * 1000ull;
}

constexpr size_t GByte(size_t g)
{
    return MByte(g) * 1000ull;
}

constexpr size_t KiByte(size_t k)
{
    return k * 1024ull;
}

constexpr size_t MiByte(size_t m)
{
    return KByte(m) * 1024ull;
}

constexpr size_t GiByte(size_t g)
{
    return MByte(g) * 1024ull;
}

static constexpr size_t DEFAULT_ARENA_SIZE = KiByte(64);
#define newARENA(arena, T) new (arena.allocate<T>()) T
#define newA(T) newARENA(arena, T)

struct arena_allocator {
    char *start;
    char *current;
    const size_t size;
    size_t space_left;
    const bool owns_memory;

    arena_allocator() :
      size(DEFAULT_ARENA_SIZE), owns_memory(true)
    {
        space_left = size;
        start = static_cast<char *>(malloc(size));
        current = start;
    }

    arena_allocator(size_t size) :
      size(size), owns_memory(true)
    {
        space_left = size;
        start = static_cast<char *>(malloc(size));
        current = start;
    }

    arena_allocator(arena_allocator& arena, size_t size) :
      size(std::min(arena.space_left, size)), owns_memory(false)
    {
        space_left = this->size;
        start = arena.current;
        current = start;
    }

    void clear()
    {
        space_left = size;
        current = start;
    }

    ~arena_allocator()
    {
        if (owns_memory)
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
    T *calloc(size_t count)
    {
        const auto total_size = sizeof(T) * count;
        size_t offset = current_offset_required_for_alignment_of<T>();
        space_left -= total_size + offset;
        assert(space_left > 0);
        current += offset;
        void *to_alloc = current;
        current += total_size;
        return static_cast<T *>(to_alloc);
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

    auto child(size_t size = std::numeric_limits<size_t>::max())
    {
        return arena_allocator(*this, size);
    }
};
} // namespace luc
#endif
