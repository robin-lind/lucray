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

template<typename T = char>
constexpr size_t size_of(size_t count = 1)
{
    const size_t size = sizeof(T);
    const size_t total_size = size * count;
    return total_size;
}

#define newARENA(arena, T) new ((arena).allocate<T>()) T
#define newA(T) newARENA(arena, T)
#define newIA(T) newARENA(iarena, T)

static constexpr size_t DEFAULT_ARENA_SIZE = KiByte(64);

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

    [[nodiscard]] void *alloc(size_t size)
    {
        assert(enough_space(size));
        space_left -= size;
        void *to_alloc = current;
        current += size;
        return to_alloc;
    }

    template<typename T>
    [[nodiscard]] T *allocate(size_t count = 1)
    {
        const auto total_size = sizeof(T) * count;
        const size_t offset = current_offset_required_for_alignment_of<T>();
        const auto alloc_size = total_size + offset;
        return static_cast<T *>(alloc(alloc_size));
    }

    template<typename T = char>
    [[nodiscard]] bool enough_space(size_t count = 1) const
    {
        const size_t size = size_of<T>(count);
        return space_left >= size;
    }

    template<typename T>
    [[nodiscard]] size_t current_offset_required_for_alignment_of() const
    {
        constexpr size_t alignment = alignof(T);
        const size_t offset = (alignment - (reinterpret_cast<size_t>(current) % alignment)) & (alignment - 1);
        return offset;
    }

    [[nodiscard]] auto child(size_t size = std::numeric_limits<size_t>::max())
    {
        return arena_allocator(*this, size);
    }
};

static constexpr size_t DEFAULT_INFINITE_ARENA_SIZE = MiByte(1);

struct infinite_arena {
    struct arena_node {
        arena_node *next;
        arena_node *prev{};
        arena_allocator arena;

        arena_node(size_t size = DEFAULT_INFINITE_ARENA_SIZE, arena_node *prev = nullptr) :
          arena(size), prev(prev)
        {
            next = arena.allocate<arena_node>();
        }
    };

    arena_node *first;
    arena_node *current;

    infinite_arena()
    {
        first = current = new (static_cast<arena_node *>(malloc(sizeof(arena_node)))) arena_node();
    }

    ~infinite_arena()
    {
        arena_node *cur = current;
        do {
            cur->arena.~arena_allocator();
            cur = cur->prev;
        } while (cur != nullptr);
    }

    template<typename T>
    [[nodiscard]] T *allocate(size_t count = 1)
    {
        if (!current->arena.enough_space<T>(count)) {
            const auto size = (((size_of<T>(count) + sizeof(arena_node)) / DEFAULT_INFINITE_ARENA_SIZE) + 1) * DEFAULT_INFINITE_ARENA_SIZE;
            current = new (current->next) arena_node(size, current);
        }
        T *result = current->arena.allocate<T>(count);
        return result;
    }
};
} // namespace luc
#endif
