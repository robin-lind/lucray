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

#define DEFAULT_ARENA_SIZE 5000000

struct arena_allocator
{
    char  *start;
    char  *current;
    size_t size;
    size_t space_left;

    arena_allocator()
    {
        size       = DEFAULT_ARENA_SIZE;
        space_left = size;
        start      = static_cast<char *>(malloc(size));
        current    = start;
    }

    arena_allocator(const size_t size_) :
      size(size_)
    {
        space_left = size;
        start      = static_cast<char *>(malloc(size));
        current    = start;
    }

    [[nodiscard]] size_t get_size_left() const
    {
        return space_left;
    }

    ~arena_allocator()
    {
        free(start);
    }

    void *alloc(size_t s)
    {
        space_left = space_left - s;
        assert(space_left > 0);
        void *to_alloc = current;
        current += s;
        return to_alloc;
    }

    template<typename T>
    T *allocate()
    {
        size_t s   = sizeof(T);
        space_left = space_left - s;
        assert(space_left > 0);
        void *to_alloc = current;
        current += s;
        return static_cast<T *>(to_alloc);
    }
};

template<typename T>
struct arena_list
{
    T     *ptr  = nullptr;
    size_t size = 0;

    arena_list() = default;

    arena_list(arena_allocator& arena, const size_t& count)
    {
        ptr  = reinterpret_cast<T *>(arena.alloc(count * sizeof(T)));
        size = count;
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

template<typename T>
struct expanding_list
{
    arena_allocator& arena;
    T               *ptr   = nullptr;
    size_t           count = 0;

    expanding_list(arena_allocator& _arena) :
      arena(_arena), ptr(reinterpret_cast<T *>(_arena.current)) {}

    void emplace_back(const T& item)
    {
        T *space = arena.allocate<T>();
        *space   = item;
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
