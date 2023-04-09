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

#ifndef PARALLEL_FOR_H
#define PARALLEL_FOR_H

#include "math/vector.h"
#include "math/utils.h"
#include <cstddef>
#include <vector>
#include <thread>
#include <functional>
#include <numeric>
#include <tuple>
#include <mutex>
#include <queue>
#include <optional>

template<typename TSize>
struct work_range {
    TSize minx, maxx, miny, maxy;
    work_range() = default;

    work_range(TSize min_x, TSize max_x, TSize min_y, TSize max_y) :
      minx(min_x), maxx(max_x), miny(min_y), maxy(max_y) {}
};

template<typename TSize>
struct work_domain {
    work_range<TSize> range;
    std::vector<work_range<TSize>> ranges;
    work_domain() = default;

    work_domain(TSize min_x, TSize max_x, TSize min_y, TSize max_y) :
      range(min_x, max_x, min_y, max_y) {}
};

template<typename TSize>
struct work_block {
    work_range<TSize> tile;
    work_range<TSize> domain;
    work_block() = default;

    work_block(work_range<TSize> _tile, work_range<TSize> _domain) :
      tile(_tile), domain(_domain) {}
};

template<typename TSize>
std::pair<work_range<TSize>, work_range<TSize>> split_range(const work_range<TSize>& range)
{
    const auto w = range.maxx - range.minx;
    const auto h = range.maxy - range.miny;
    bool vertical = false;
    auto min = range.miny, max = range.maxy;
    if (w > h) {
        vertical = true;
        min = range.minx;
        max = range.maxx;
    }
    const auto mid = std::midpoint(min, max);
    const work_range<TSize> a_v(range.minx, mid, range.miny, range.maxy);
    const work_range<TSize> b_v(mid, range.maxx, range.miny, range.maxy);
    const work_range<TSize> a_h(range.minx, range.maxx, range.miny, mid);
    const work_range<TSize> b_h(range.minx, range.maxx, mid, range.maxy);
    const work_range<TSize> a = vertical ? a_v : a_h;
    const work_range<TSize> b = vertical ? b_v : b_h;
    return std::make_pair(a, b);
}

template<typename TSize>
work_domain<TSize> generate_parallel_for_domain_tiles(TSize min_x, TSize max_x, TSize min_y, TSize max_y)
{
    work_domain<TSize> domain(min_x, max_x, min_y, max_y);
    std::queue<work_range<TSize>> queue;
    queue.push(domain.range);
    while (!queue.empty()) {
        const auto range = queue.front();
        queue.pop();
        const TSize max_size = 64;
        const auto w = range.maxx - range.minx;
        const auto h = range.maxy - range.miny;
        if (std::max(w, h) > max_size) {
            auto split = split_range(range);
            queue.push(split.first);
            queue.push(split.second);
        }
        else {
            domain.ranges.push_back(range);
        }
    }
    return domain;
}

template<typename TSize>
work_domain<TSize> generate_parallel_for_domain_rows(TSize min_x, TSize max_x, TSize min_y, TSize max_y)
{
    work_domain<TSize> domain(min_x, max_x, min_y, max_y);
    domain.ranges.reserve(max_y - min_y);
    for (TSize y = min_y; y < max_y; y++) {
        work_range<TSize> range(min_x, max_x, y, y + 1);
        domain.ranges.push_back(range);
    }
    return domain;
}

template<typename TSize>
work_domain<TSize> generate_parallel_for_domain_grouped_rows(TSize min_x, TSize max_x, TSize min_y, TSize max_y)
{
    const auto group_size = 4;
    work_domain<TSize> domain(min_x, max_x, min_y, max_y);
    domain.ranges.reserve((max_y - min_y) / group_size + 1);
    for (TSize y = min_y; y < max_y; y += group_size) {
        const auto max_y_clamped = std::min(max_y + 1, y + group_size);
        if (y == max_y_clamped)
            continue;
        work_range<TSize> range(min_x, max_x, y, max_y_clamped);
        domain.ranges.push_back(range);
    }
    return domain;
}

template<typename TSize>
std::queue<work_range<TSize>> range_queue_from_domain(const std::vector<work_range<TSize>>& domain_ranges)
{
    std::queue<work_range<TSize>> range_queue;
    for (const auto& range : domain_ranges)
        range_queue.push(range);
    return range_queue;
}

struct abort_token {
    bool aborted = false;
    abort_token() = default;

    void abort()
    {
        aborted = true;
    }
};

template<typename TSize, typename TFloat = float>
void iterate_over_tile(const work_block<TSize>& block, const abort_token& aborter, auto&& item_func)
{
    for (auto y = block.tile.miny; y < block.tile.maxy && !aborter.aborted; y++) {
        for (auto x = block.tile.minx; x < block.tile.maxx; x++) {
            const auto minx = math::map<TFloat>(x, block.domain.minx, block.domain.maxx, 0, 1) - TFloat(.5);
            const auto miny = math::map<TFloat>(y, block.domain.miny, block.domain.maxy, 0, 1) - TFloat(.5);
            const auto maxx = math::map<TFloat>(x + TFloat(1.), block.domain.minx, block.domain.maxx, 0, 1) - TFloat(.5);
            const auto maxy = math::map<TFloat>(y + TFloat(1.), block.domain.miny, block.domain.maxy, 0, 1) - TFloat(.5);
            auto transform = [minx, miny, maxx, maxy](math::vector<TFloat, 2> uv) {
                const auto su = math::map<TFloat>(uv.u, -.5f, .5f, minx, maxx);
                const auto sv = math::map<TFloat>(uv.v, -.5f, .5f, miny, maxy);
                return math::vector<TFloat, 2>(su, sv);
            };
            item_func(x, y, transform);
        }
    }
}

template<typename TSize = int, bool parallel = true>
void parallel_for(const work_domain<TSize>& domain, auto&& tile_func, abort_token *aborter)
{
    auto range_queue = range_queue_from_domain(domain.ranges);
    std::mutex work_stealing_mutex;
    const auto hardware_concurrency = std::thread::hardware_concurrency();
    // const auto number_of_threads = hardware_concurrency; // base
    // const auto number_of_threads = hardware_concurrency * 4 / 3; // 1% faster than base
    const auto number_of_threads = hardware_concurrency * 3 / 2; // 2% faster than base
    const auto thread_count = parallel ? number_of_threads : 1;
    auto worker = [&]() {
        work_block<TSize> block(domain.range, domain.range);
        while (aborter != nullptr ? !aborter->aborted : true) {
            {
                const std::scoped_lock stealing_work(work_stealing_mutex);
                if (range_queue.size() == 0)
                    break;
                block.tile = range_queue.front();
                range_queue.pop();
                const auto w = block.tile.maxx - block.tile.minx;
                const auto h = block.tile.maxy - block.tile.miny;
                if (range_queue.size() < thread_count && std::min(w, h) > 4) {
                    const auto& split = split_range(block.tile);
                    range_queue.push(split.second);
                    block.tile = split.first;
                }
            }
            tile_func(block);
        }
    };
    std::vector<std::thread> threads;
    for (size_t i = 0; i < thread_count; i++)
        threads.emplace_back(worker);
    for (auto& thread : threads)
        thread.join();
}

#endif
