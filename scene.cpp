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
            sscene.transform = minst.transform;
            sscene.material = model.materials[submesh.material];
            auto get_bvh_vec = [&](const math::float3 v) {
                return *(bvh::Vec<float, 3> *)(&v);
            };
            auto get_float3 = [&](const bvh::Vec<float, 3> v) {
                return *(math::float3 *)(&v);
            };
            const auto triangle_count = submesh.indices.size() / 3;
            math::double3 centerd;
            std::vector<bvh::BBox<float, 3>> bboxes(triangle_count);
            std::vector<bvh::Vec<float, 3>> centers(triangle_count);
            executor.for_each(0, triangle_count,
                              [&](size_t begin, size_t end) {
                                  for (size_t i = begin; i < end; ++i) {
                                      const auto& t0 = submesh.indices[i * 3 + 0];
                                      const auto& t1 = submesh.indices[i * 3 + 1];
                                      const auto& t2 = submesh.indices[i * 3 + 2];
                                      const auto v0 = get_bvh_vec(submesh.vertices[t0]);
                                      const auto v1 = get_bvh_vec(submesh.vertices[t1]);
                                      const auto v2 = get_bvh_vec(submesh.vertices[t2]);
                                      const bvh::Tri<float, 3> tri(v0, v1, v2);
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
            sscene.center = math::float3((float)centerd.x, (float)centerd.y, (float)centerd.z);

            typename bvh::DefaultBuilder<bvh::Node<float, 3>>::Config config;
            config.quality = bvh::DefaultBuilder<bvh::Node<float, 3>>::Quality::High;
            sscene.accelerator = bvh::DefaultBuilder<bvh::Node<float, 3>>::build(thread_pool, bboxes, centers, config);

            sscene.triangles.resize(triangle_count);
            executor.for_each(0, triangle_count,
                              [&](size_t begin, size_t end) {
                                  for (size_t i = begin; i < end; ++i) {
                                      const auto id = sscene.accelerator.prim_ids[i];
                                      const auto& t0 = submesh.indices[id * 3 + 0];
                                      const auto& t1 = submesh.indices[id * 3 + 1];
                                      const auto& t2 = submesh.indices[id * 3 + 2];
                                      const auto v0 = submesh.vertices[t0];
                                      const auto v1 = submesh.vertices[t1];
                                      const auto v2 = submesh.vertices[t2];
                                      sscene.triangles[i] = triangle<float>(v0, v1, v2);
                                  }
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
                              math::bounds3 tbox;
                              math::double3 tcenter;
                              for (const auto& ptri : scene.triangles) {
                                  const auto p0 = ptri.p0, p1 = p0 - ptri.e1, p2 = p0 + ptri.e2;
                                  const auto t0 = math::mul(scene.transform, math::float4(p0, 1.f)).xyz;
                                  const auto t1 = math::mul(scene.transform, math::float4(p1, 1.f)).xyz;
                                  const auto t2 = math::mul(scene.transform, math::float4(p2, 1.f)).xyz;
                                  tbox.extend(t0).extend(t1).extend(t2);
                                  const auto c = (t0 + t1 + t2) * (1.f / 3.f);
                                  tcenter += math::double3(c.x, c.y, c.z);
                              }
                              tcenter /= (double)scene.triangles.size();
                              bboxes[i].min.values = tbox.min.E;
                              bboxes[i].max.values = tbox.max.E;
                              centers[i] = { (float)tcenter.x, (float)tcenter.y, (float)tcenter.z };
                          }
                      });

    typename bvh::DefaultBuilder<bvh::Node<float, 3>>::Config config;
    config.quality = bvh::DefaultBuilder<bvh::Node<float, 3>>::Quality::High;
    accelerator = bvh::DefaultBuilder<bvh::Node<float, 3>>::build(thread_pool, bboxes, centers, config);

    auto old = std::move(scenes);
    scenes.resize(count);
    executor.for_each(0, count,
                      [&](size_t begin, size_t end) {
                          for (size_t i = begin; i < end; ++i) {
                              scenes[i] = std::move(old[accelerator.prim_ids[i]]);
                              scenes[i].transform = math::inverse(scenes[i].transform);
                          }
                      });
}

template<typename T>
std::optional<typename triangle<T>::intersection> triangle<T>::intersect(const math::vector<T, 3>& org, const math::vector<T, 3>& dir) const
{
    constexpr auto tolerance = -std::numeric_limits<T>::epsilon();
    constexpr auto tmin = std::numeric_limits<T>::epsilon();
    constexpr auto tmax = std::numeric_limits<T>::max();
        const auto c = p0 - org;
        const auto r = math::cross(dir, c);
    const auto inv_det = static_cast<T>(1.) / math::dot(n, dir);
        const auto u = math::dot(r, e2) * inv_det;
        const auto v = math::dot(r, e1) * inv_det;
    const auto w = static_cast<T>(1.) - u - v;
        if (u < tolerance || v < tolerance || w < tolerance)
            return std::nullopt;
        const auto t = math::dot(n, c) * inv_det;
        if (t < tmin || t > tmax)
            return std::nullopt;
    triangle::intersection inter;
    inter.uv = { u, v };
    inter.distance = t;
    return inter;
}

std::optional<scene::subscene::intersection> scene::subscene::intersect(const math::float3& org, const math::float3& dir) const
{
    static constexpr size_t stack_size = 64;
        static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
    math::float3 _org, _dir;
    std::tie(_org, _dir) = math::transform_ray(transform, org, dir);

        auto prim_id = invalid_id;
    triangle<float>::intersection t_inter;
    t_inter.distance = std::numeric_limits<float>::max();
        auto leaf_test = [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
            if (const auto hit = triangles[i].intersect(_org, _dir)) {
                if (hit->distance < t_inter.distance) {
                    t_inter = *hit;
                        prim_id = i;
                    }
                }
            }
            return prim_id != invalid_id;
        };
    bvh::Ray<float, 3> _ray(*(bvh::Vec<float, 3> *)(&_org), *(bvh::Vec<float, 3> *)(&_dir));
        bvh::SmallStack<typename bvh::Bvh<bvh::Node<float, 3>>::Index, stack_size> stack;
    accelerator.template intersect<false, true>(_ray, accelerator.get_root().index, stack, leaf_test);

    if (prim_id != invalid_id) {
        scene::subscene::intersection inter;
        const auto& tri = triangles[prim_id];
        const auto pbarycentric = tri.p0 - tri.e1 * t_inter.uv.u + tri.e2 * t_inter.uv.v;
        inter.position = pbarycentric;
        inter.normal = math::normalize(tri.n);
        inter.distance = t_inter.distance;
        return inter;
        }
    return std::nullopt;
}

std::optional<scene::intersection> scene::intersect(const math::float3& org, const math::float3& dir) const
{
    static constexpr size_t stack_size = 64;
    subscene::intersection s_inter;
    s_inter.distance = std::numeric_limits<float>::max();
    bool outer_hit = false;
    auto leaf_test_bbox = [&](size_t begin, size_t end) {
        bool inner_hit = false;
        for (size_t i = begin; i < end; ++i) {
            if (const auto hit = scenes[i].intersect(org, dir)) {
                inner_hit = true;
                if (hit->distance < s_inter.distance) {
                    s_inter = *hit;
                outer_hit = true;
            }
        }
        }
        return inner_hit;
    };
    bvh::Ray<float, 3> ray(*(bvh::Vec<float, 3> *)(&org), *(bvh::Vec<float, 3> *)(&dir));
    bvh::SmallStack<typename bvh::Bvh<bvh::Node<float, 3>>::Index, stack_size> stack;
    accelerator.template intersect<false, true>(ray, accelerator.get_root().index, stack, leaf_test_bbox);

    if (outer_hit) {
        scene::intersection inter;
        inter.color = s_inter.position;
        return inter;
    }
    return std::nullopt;
}
} // namespace luc
