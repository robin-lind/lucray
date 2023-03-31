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
#include "math/math.h"
#include <optional>
#include <utility>

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
            sscene.transform = math::inverse(minst.transform);
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

// BVH_ALWAYS_INLINE Tri<T, 3> convert_to_tri() const
// {
//     return Tri<T, 3>(p0, p0 - e1, e2 + p0);
// }
// BVH_ALWAYS_INLINE BBox<T, 3> get_bbox() const
// {
//     return convert_to_tri().get_bbox();
// }
// BVH_ALWAYS_INLINE Vec<T, 3> get_center() const
// {
//     return convert_to_tri().get_center();
// }
// BVH_ALWAYS_INLINE BBox<T, 3> get_bbox() const
// {
//     return BBox(p0).extend(p1).extend(p2);
// }
// BVH_ALWAYS_INLINE Vec<T, 3> get_center() const
// {
//     return (p0 + p1 + p2) * static_cast<T>(1. / 3.);
// }

std::optional<scene::intersection> scene::intersect(const math::float3& org, const math::float3& dir)
{
    static constexpr size_t stack_size = 64;
    auto intersect_tri = [&](const math::float3& org, const math::float3& dir, const bvh::PrecomputedTri<float>& tri, float tmax) -> std::optional<std::tuple<float, float, float>> {
        // const Vec<T, 3>& p0, const Vec<T, 3>& p1, const Vec<T, 3>& p2
        // p0(p0), e1(p0 - p1), e2(p2 - p0), n(cross(e1, e2))
        const math::float3 p0(tri.p0.values), e1(tri.e1.values), e2(tri.e2.values), n(tri.n.values);
        const float tolerance = -std::numeric_limits<float>::epsilon();
        const float tmin = std::numeric_limits<float>::epsilon();
        const auto c = p0 - org;
        const auto r = math::cross(dir, c);
        const auto inv_det = static_cast<float>(1.) / math::dot(n, dir);
        const auto u = math::dot(r, e2) * inv_det;
        const auto v = math::dot(r, e1) * inv_det;
        const auto w = static_cast<float>(1.) - u - v;
        if (u < tolerance || v < tolerance || w < tolerance)
            return std::nullopt;
        const auto t = math::dot(n, c) * inv_det;
        if (t < tmin || t > tmax)
            return std::nullopt;
        return std::make_tuple(u, v, t);
    };
    bvh::Ray<float, 3> ray(*(bvh::Vec<float, 3> *)(&org), *(bvh::Vec<float, 3> *)(&dir));
    intersection inter;
    float outer_tmax = std::numeric_limits<float>::max();
    auto intersect_sub = [&](size_t i) {
        static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
        auto prim_id = invalid_id;

        const auto& sub_scene = scenes[i];
        const auto& sub_acc = sub_scene.accelerator;
        const auto& sub_tris = sub_scene.triangles;

        math::float3 _org, _dir;
        std::tie(_org, _dir) = math::transform_ray(sub_scene.transform, org, dir);
        bvh::Ray<float, 3> _ray(*(bvh::Vec<float, 3> *)(&_org), *(bvh::Vec<float, 3> *)(&_dir));
        math::float3 n;
        float tmax = std::numeric_limits<float>::max();
        auto leaf_test = [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                if (auto hit = intersect_tri(_org, _dir, sub_tris[i], tmax)) {
                    float u, v, t;
                    std::tie(u, v, t) = *hit;
                    if (t < tmax) {
                        tmax = t;
                        prim_id = i;
                        n.E = sub_tris[i].n.values;
                    }
                }
            }
            return prim_id != invalid_id;
        };
#if 1
        bvh::SmallStack<typename bvh::Bvh<bvh::Node<float, 3>>::Index, stack_size> stack;
        sub_acc.template intersect<false, true>(_ray, sub_acc.get_root().index, stack, leaf_test);

        if (tmax < outer_tmax)
        {
            outer_tmax = tmax;
            inter.color = sub_scene.material.albedo.c * std::abs(math::dot(_dir, math::normalize(n)));
        }
        return prim_id != invalid_id;
#else
        const bool ehm = leaf_test(0, sub_tris.size());
        if (tmax < outer_tmax) {
            outer_tmax = tmax;
            inter.color = sub_scene.material.albedo.c;
            return true;
        }
        return false;
#endif
        // return leaf_test(0, sub_tris.size());
    };
    bool outer_hit = false;
    auto leaf_test_bbox = [&](size_t begin, size_t end) {
        bool hit = false;
        for (size_t i = begin; i < end; ++i) {
            if (intersect_sub(i)) {
                hit = true;
                outer_hit = true;
            }
        }
        return hit;
    };
    // bvh::SmallStack<typename bvh::Bvh<bvh::Node<float, 3>>::Index, stack_size> stack;
    // accelerator.template intersect<false, true>(ray, accelerator.get_root().index, stack, leaf_test_bbox);
    // outer_hit = outer_hit || leaf_test_bbox(0, scenes.size());

    leaf_test_bbox(0, scenes.size());

    if (outer_hit)
        return inter;
    return std::nullopt;
}
} // namespace luc
