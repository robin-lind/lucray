#pragma once

#include "bvh.h"
#include <cstddef>
#include <functional>
#include "math/vector.h"

template<typename TFloat = float>
struct traversal {
    std::vector<bvh::PrecomputedTri<TFloat>> ptriangles;

    bvh::Bvh<bvh::Node<TFloat, 3>> accelerator;

    traversal(std::function<bvh::Tri<TFloat, 3>(size_t)> triangle, const size_t triangle_count)
    {
        bvh::ThreadPool thread_pool;
        bvh::ParallelExecutor executor(thread_pool);

        std::vector<bvh::BBox<TFloat, 3>> bboxes(triangle_count);
        std::vector<bvh::Vec<TFloat, 3>> centers(triangle_count);
        executor.for_each(0, triangle_count,
                          [&](size_t begin, size_t end) {
                              for (size_t i = begin; i < end; ++i) {
                                  const auto tri = triangle(i);
                                  bboxes[i] = tri.get_bbox();
                                  centers[i] = tri.get_center();
                              }
                          });

        typename bvh::DefaultBuilder<bvh::Node<TFloat, 3>>::Config config;
        config.quality = bvh::DefaultBuilder<bvh::Node<TFloat, 3>>::Quality::High;
        accelerator = bvh::DefaultBuilder<bvh::Node<TFloat, 3>>::build(thread_pool, bboxes, centers, config);

        ptriangles.resize(triangle_count);
        executor.for_each(0, triangle_count,
                          [&](size_t begin, size_t end) {
                              for (size_t i = begin; i < end; ++i)
                                  ptriangles[i] = triangle(accelerator.prim_ids[i]);
                          });
    }

    bool traverse(const math::vector_tn<TFloat, 3>& org, const math::vector_tn<TFloat, 3>& dir) const
    {
        bvh::Ray<TFloat, 3> ray(
          *(bvh::Vec<TFloat, 3> *)(&org),
          *(bvh::Vec<TFloat, 3> *)(&dir));
        static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
        static constexpr size_t stack_size = 64;

        auto prim_id = invalid_id;
        TFloat u, v;

        auto leaf_test = [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                if (auto hit = ptriangles[i].intersect(ray)) {
                    prim_id = i;
                    std::tie(u, v) = *hit;
                }
            }
            return prim_id != invalid_id;
        };
        bvh::SmallStack<typename bvh::Bvh<bvh::Node<TFloat, 3>>::Index, stack_size> stack;
        accelerator.template intersect<false, true>(ray, accelerator.get_root().index, stack, leaf_test);

        return prim_id != invalid_id;
    }
};
