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
#include "math/matrix.h"
#include "math/bounds.h"
#include <optional>
#include <utility>
#include <vector>

namespace luc {
void scene::append_model(luc::model&& model)
{
    bvh::ThreadPool thread_pool;
    bvh::ParallelExecutor executor(thread_pool);
    for (const auto& cam : model.cameras)
        cameras.push_back(cam);
    for (const auto& minst : model.instances) {
        for (const auto& submesh : model.meshes[minst.id].meshes) {
            if (submesh.material < 0) continue;
            subscene sscene;
            sscene.transform = minst.transform;
            sscene.inverse = math::inverse(minst.transform);
            sscene.material = model.materials[submesh.material];
            const auto triangle_count = submesh.indices.size() / 3;
            std::vector<bvh::BBox<float, 3>> bboxes(triangle_count);
            std::vector<bvh::Vec<float, 3>> centers(triangle_count);
            executor.for_each(0, triangle_count,
                              [&](size_t begin, size_t end) {
                                  for (size_t i = begin; i < end; ++i) {
                                      const auto& t0 = submesh.indices[i * 3 + 0];
                                      const auto& t1 = submesh.indices[i * 3 + 1];
                                      const auto& t2 = submesh.indices[i * 3 + 2];
                                      const auto v0 = submesh.vertices[t0];
                                      const auto v1 = submesh.vertices[t1];
                                      const auto v2 = submesh.vertices[t2];
                                      const math::bounds3 bounds(v0, v1, v2);
                                      bboxes[i].min.values = bounds.min.values;
                                      bboxes[i].max.values = bounds.max.values;
                                      centers[i].values = ((v0 + v1 + v2) * (1.f / 3.f)).values;
                                  }
                              });

            typename bvh::DefaultBuilder<bvh::Node<float, 3>>::Config config;
            config.quality = bvh::DefaultBuilder<bvh::Node<float, 3>>::Quality::High;
            sscene.accelerator = bvh::DefaultBuilder<bvh::Node<float, 3>>::build(thread_pool, bboxes, centers, config);

            sscene.triangles.resize(triangle_count);
            if (submesh.normals)
                sscene.normals = std::vector<triplet<math::float3>>(triangle_count);
            if (submesh.texcoords)
                sscene.texcoords = std::vector<triplet<math::float2>>(triangle_count);
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
                                  if (sscene.normals) {
                                      const auto& snormals = *submesh.normals;
                                      auto& normals = *sscene.normals;
                                      for (size_t i = begin; i < end; ++i) {
                                          const auto id = sscene.accelerator.prim_ids[i];
                                          const auto& t0 = submesh.indices[id * 3 + 0];
                                          const auto& t1 = submesh.indices[id * 3 + 1];
                                          const auto& t2 = submesh.indices[id * 3 + 2];
                                          const auto n0 = snormals[t0];
                                          const auto n1 = snormals[t1];
                                          const auto n2 = snormals[t2];
                                          normals[i] = triplet<math::float3>(n0, n1, n2);
                                      }
                                  }
                                  if (sscene.texcoords) {
                                      const auto& stexcoords = *submesh.texcoords;
                                      auto& texcoords = *sscene.texcoords;
                                      for (size_t i = begin; i < end; ++i) {
                                          const auto id = sscene.accelerator.prim_ids[i];
                                          const auto& t0 = submesh.indices[id * 3 + 0];
                                          const auto& t1 = submesh.indices[id * 3 + 1];
                                          const auto& t2 = submesh.indices[id * 3 + 2];
                                          const auto u0 = stexcoords[t0];
                                          const auto u1 = stexcoords[t1];
                                          const auto u2 = stexcoords[t2];
                                          texcoords[i] = triplet<math::float2>(u0, u1, u2);
                                      }
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
                              centers[i] = { (float)tcenter.x, (float)tcenter.y, (float)tcenter.z };
                              bboxes[i].min.values = tbox.min.values;
                              bboxes[i].max.values = tbox.max.values;
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
    math::float3 _org, _dir;
    std::tie(_org, _dir) = math::transform_ray(inverse, org, dir);
    static constexpr size_t stack_size = 64;
    static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
    auto prim_id = invalid_id;
    triangle<float>::intersection inter;
    auto leaf_test = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (const auto hit = triangles[i].intersect(_org, _dir)) {
                if (hit->distance < inter.distance) {
                    inter = *hit;
                    prim_id = i;
                }
            }
        }
        return false;
    };
    bvh::Ray<float, 3> _ray(*(bvh::Vec<float, 3> *)(&_org), *(bvh::Vec<float, 3> *)(&_dir));
    bvh::SmallStack<typename bvh::Bvh<bvh::Node<float, 3>>::Index, stack_size> stack;
    accelerator.template intersect<false, true>(_ray, accelerator.get_root().index, stack, leaf_test);

    if (prim_id != invalid_id) {
        scene::subscene::intersection result;
        result.distance = inter.distance;

        const auto& tri = triangles[prim_id];
        result.position = tri.p0 - tri.e1 * inter.uv.u + tri.e2 * inter.uv.v;
        result.position = math::mul(transform, math::float4(result.position, 1.f)).xyz;

        if (texcoords.has_value()) {
            const auto& triplet_u = (*texcoords)[prim_id];
            result.texcoord = triplet_u.p0 - triplet_u.e1 * inter.uv.u + triplet_u.e2 * inter.uv.v;
        }
        result.normal_g = math::mul(transform, math::float4(tri.n, 0.f)).xyz;
        result.normal_g = math::normalize(result.normal_g);
        if (math::dot(result.normal_g, dir) > 0.f)
            result.normal_g = -result.normal_g;
        if (normals.has_value()) {
            const auto& triplet_n = (*normals)[prim_id];
            result.normal_s = triplet_n.p0 - triplet_n.e1 * inter.uv.u + triplet_n.e2 * inter.uv.v;
            result.normal_s = math::mul(transform, math::float4(result.normal_s, 0.f)).xyz;
            result.normal_s = math::normalize(result.normal_s);
            if (math::dot(result.normal_s, result.normal_g) < 0.f)
                result.normal_s = -result.normal_s;
        }
        else {
            result.normal_s = result.normal_g;
        }

        return result;
    }
    return std::nullopt;
}

std::optional<scene::intersection> scene::intersect(const math::float3& org, const math::float3& dir) const
{
    static constexpr size_t stack_size = 64;
    static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
    auto prim_id = invalid_id;
    subscene::intersection inter;
    auto leaf_test = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (const auto hit = scenes[i].intersect(org, dir)) {
                if (hit->distance < inter.distance) {
                    inter = *hit;
                    prim_id = i;
                }
            }
        }
        return false;
    };
    bvh::Ray<float, 3> ray(*(bvh::Vec<float, 3> *)(&org), *(bvh::Vec<float, 3> *)(&dir));
    bvh::SmallStack<typename bvh::Bvh<bvh::Node<float, 3>>::Index, stack_size> stack;
    accelerator.template intersect<false, true>(ray, accelerator.get_root().index, stack, leaf_test);

    if (prim_id != invalid_id) {
        const auto& scene = scenes[prim_id];
        scene::intersection result;
        result.position = inter.position;
        result.normal_g = inter.normal_g;
        result.normal_s = inter.normal_s;
        if (scene.material.albedo.texture.has_value()) {
            const auto& tex = *scene.material.albedo.texture;
            const auto albedo = tex->sample(inter.texcoord);
            result.albedo = albedo;
        }
        else {
            const auto albedo = scene.material.albedo.value;
            result.albedo = albedo;
        }
        if (scene.material.emission.texture.has_value()) {
            const auto& tex = *scene.material.emission.texture;
            const auto emission = tex->sample(inter.texcoord);
            if (math::length_squared(emission) > 0)
                result.emission = emission * scene.material.emissive_strength;
        }
        else {
            const auto emission = scene.material.emission.value;
            if (math::length_squared(emission) > 0)
                result.emission = emission * scene.material.emissive_strength;
        }
        result.specular = result.albedo;
        result.metallic = scene.material.metallic.value;
        result.roughness = scene.material.roughness.value;
        result.ior = scene.material.ior.value;
        result.transmission = scene.material.transmission.value;
        return result;
    }
    return std::nullopt;
}
} // namespace luc
