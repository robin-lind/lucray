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

#include "scene.h"
#include "aixlog.hpp"
#include "math/vector.h"

namespace luc {
void scene::append_model(luc::model&& model)
{
    bvh::ThreadPool thread_pool;
    bvh::ParallelExecutor executor(thread_pool);
    for (const auto& minst : model.instances) {
        if (minst.type == luc::model::camera_instance)
            continue;
        for (const auto& submesh : model.meshes[minst.id].meshes) {
            subscene sscene;
            sscene.transform = minst.transform;
            sscene.material = model.materials[submesh.material];
            auto get_bvh_vec = [&](const math::float3 v) {
                return *(bvh::Vec<float, 3> *)(&v);
            };
            auto get_float3 = [&](const bvh::Vec<float, 3> v) {
                return *(math::float3 *)(&v);
            };
            auto get_tri = [&](size_t i) {
                const auto& t0 = submesh.indices[i * 3 + 0];
                const auto& t1 = submesh.indices[i * 3 + 1];
                const auto& t2 = submesh.indices[i * 3 + 2];
                const auto v0 = get_bvh_vec(submesh.vertices[t0]);
                const auto v1 = get_bvh_vec(submesh.vertices[t1]);
                const auto v2 = get_bvh_vec(submesh.vertices[t2]);
                return bvh::Tri<float, 3>(v0, v1, v2);
            };
            const auto triangle_count = submesh.indices.size() / 3;
            math::double3 centerd;
            std::vector<bvh::BBox<float, 3>> bboxes(triangle_count);
            std::vector<bvh::Vec<float, 3>> centers(triangle_count);
            executor.for_each(0, triangle_count,
                              [&](size_t begin, size_t end) {
                                  for (size_t i = begin; i < end; ++i) {
                                      const auto tri = get_tri(i);
                                      const auto cc = tri.get_center();
                                      const auto bb = tri.get_bbox();
                                      centers[i] = cc;
                                      bboxes[i] = bb;
                                      centerd += math::double3(cc.values[0], cc.values[1], cc.values[2]);
                                      sscene.bounds.extend(get_float3(bb.min));
                                      sscene.bounds.extend(get_float3(bb.max));
                                  }
                              });
            centerd /= (double)triangle_count;
            sscene.center = math::float3(centerd.x, centerd.y, centerd.z);

            typename bvh::DefaultBuilder<bvh::Node<float, 3>>::Config config;
            config.quality = bvh::DefaultBuilder<bvh::Node<float, 3>>::Quality::High;
            sscene.accelerator = bvh::DefaultBuilder<bvh::Node<float, 3>>::build(thread_pool, bboxes, centers, config);

            sscene.triangles.resize(triangle_count);
            executor.for_each(0, triangle_count,
                              [&](size_t begin, size_t end) {
                                  for (size_t i = begin; i < end; ++i)
                                      sscene.triangles[i] = get_tri(sscene.accelerator.prim_ids[i]);
                              });

            scenes.push_back(std::move(sscene));
        }
    }
}

void scene::commit()
{
    bvh::ThreadPool thread_pool;
    bvh::ParallelExecutor executor(thread_pool);

    const auto count = scenes.size();
    std::vector<bvh::BBox<float, 3>> bboxes(count);
    std::vector<bvh::Vec<float, 3>> centers(count);
    executor.for_each(0, count,
                      [&](size_t begin, size_t end) {
                          for (size_t i = begin; i < end; ++i) {
                              const auto& scene = scenes[i];
                              bboxes[i].min.values = scene.bounds.min.E;
                              bboxes[i].max.values = scene.bounds.max.E;
                              centers[i].values = scene.center.E;
                              // these should be transformed
                          }
                      });

    typename bvh::DefaultBuilder<bvh::Node<float, 3>>::Config config;
    config.quality = bvh::DefaultBuilder<bvh::Node<float, 3>>::Quality::High;
    accelerator = bvh::DefaultBuilder<bvh::Node<float, 3>>::build(thread_pool, bboxes, centers, config);

    auto old = std::move(scenes);
    scenes.resize(count);
    executor.for_each(0, count,
                      [&](size_t begin, size_t end) {
                          for (size_t i = begin; i < end; ++i)
                              scenes[i] = std::move(old[accelerator.prim_ids[i]]);
                      });
}

std::optional<scene::intersection> scene::intersect(const math::float3& org, const math::float3& dir)
{
    static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
    static constexpr size_t stack_size = 64;

    auto prim_id = invalid_id;
    auto bbox_intersect = [&](size_t i) {
        return true;
    };
    auto leaf_test_bbox = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i)
            if (bbox_intersect(i))
                prim_id = i;
        return prim_id != invalid_id;
    };
    bvh::Ray<float, 3> ray(*(bvh::Vec<float, 3> *)(&org), *(bvh::Vec<float, 3> *)(&dir));
    bvh::SmallStack<typename bvh::Bvh<bvh::Node<float, 3>>::Index, stack_size> stack;
    accelerator.template intersect<false, true>(ray, accelerator.get_root().index, stack, leaf_test_bbox);
    stack.size = 0; // make sure stack is cleared

    if (prim_id == invalid_id)
        return std::nullopt; // mission failed

    // intersect against triangles in the subscene
    prim_id = invalid_id;
    float u, v;

    auto leaf_test_tri = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            // if (auto hit = ptriangles[i].intersect(ray)) {
            //     prim_id = i;
            //     std::tie(u, v) = *hit;
            // }
        }
        return prim_id != invalid_id;
    };
    accelerator.template intersect<false, true>(ray, accelerator.get_root().index, stack, leaf_test_tri);

    return std::nullopt;
}
} // namespace luc
