// dacrt.cpp

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

#include "dacrt.h"
#include "math/vector.h"
#include "lucmath_gen.h"
#include <atomic>
#include <functional>
#include <limits>
#include <array>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
#include <algorithm>
#include <optional>
#include "arena.h"
#include <fstream>
#include <string>
#include <chrono>

void Intersect(const std::vector<Ray>& rays, const std::vector<TriangleI>& triangles, const std::vector<math::Vector3>& vertices, std::vector<HitRecord>& records)
{
    size_t ray_size = sizeof(HitRecord) + sizeof(Ray) + sizeof(int);
    size_t tri_size = sizeof(TriangleI) + sizeof(math::Vector3) * 3 + sizeof(int);
    arena_allocator arena(20ull * 1024ull * 1024ull * 1024ull);
    // std::vector<std::tuple<math::Bounds3, std::vector<int>, std::vector<int>>> scuffed;

    arena_list<float> ray_dist(arena, rays.size());
    arena_list<math::Vector3> ray_p(arena, rays.size());
    arena_list<math::Vector3> ray_n(arena, rays.size());
    for (size_t i = 0; i < rays.size(); i++)
        ray_dist[i] = std::numeric_limits<float>::max();
    auto intersect_ray_triangle = [&](const Ray& ray, const math::Vector3& v0, const math::Vector3& v1, const math::Vector3& v2) -> std::optional<Intersection>
    {
        auto p0 = v0;
        auto e1 = v0 - v1;
        auto e2 = v2 - v0;
        auto n = math::Cross(e1, e2);

        auto c = p0 - ray.O;
        auto r = math::Cross(ray.D, c);
        auto inv_det = 1.f / math::Dot(n, ray.D);

        auto u = math::Dot(r, e2) * inv_det;
        auto v = math::Dot(r, e1) * inv_det;
        auto w = 1.f - u - v;

        if (u < 0.f || v < 0.f || w < 0.f)
            return {};
        auto t = math::Dot(n, c) * inv_det;
        if (t < 0.f || t > std::numeric_limits<float>::max())
            return {};
        auto p = v1 * u + v2 * v + v0 * w;
        return Intersection{ t, p, n };
    };

    auto partition = [&](math::Bounds3 bounds, const expanding_list<int>& prim_list, const expanding_list<int>& ray_list)
    {
        auto intersect_bounds = [&](math::Bounds3 a, const math::Bounds3& b)
        {
            if (b.min.x > a.max.x) return false;
            if (b.min.y > a.max.y) return false;
            if (b.min.z > a.max.z) return false;
            if (b.max.x < a.min.x) return false;
            if (b.max.y < a.min.y) return false;
            if (b.max.z < a.min.z) return false;
            return true;
        };
        expanding_list<int> new_prim_list(arena);
        math::Bounds3 new_bounds;
        for (int i = 0; i < prim_list.count; ++i)
        {
            const int prim_id = prim_list[i];
            const TriangleI& tri = triangles[prim_id];
            const math::Vector3& a = vertices[tri.A];
            const math::Vector3& b = vertices[tri.B];
            const math::Vector3& c = vertices[tri.C];
            math::Bounds3 tri_bound(a, b, c);
            if (intersect_bounds(bounds, tri_bound))
            {
                new_bounds.Union(tri_bound);
                new_prim_list.emplace_back(prim_id);
            }
        }
        bounds.max = math::Min(new_bounds.max, bounds.max);
        bounds.min = math::Max(new_bounds.min, bounds.min);
        auto intersect_ray_bounds = [&](const math::Bounds3& b, const Ray& ray)
        {
            const math::Vector3 t1 = (b.min - ray.O) / ray.D;
            const math::Vector3 t2 = (b.max - ray.O) / ray.D;
            const math::Vector3 tl = math::Min(t1, t2);
            const math::Vector3 th = math::Max(t1, t2);
            const float l = std::max(tl.x, std::max(tl.y, tl.z));
            const float h = std::min(th.x, std::min(th.y, th.z));
            return l < h;
        };
        expanding_list<int> new_ray_list(arena);
        for (int i = 0; i < ray_list.count; ++i)
        {
            const int ray_id = ray_list[i];
            const Ray& ray = rays[ray_id];
            if (intersect_ray_bounds(bounds, ray))
                new_ray_list.emplace_back(ray_id);
        }
        return std::make_tuple(bounds, new_prim_list, new_ray_list);
    };

    auto split = [](const math::Bounds3& bounds)
    {
        math::Vector3 d = bounds.max - bounds.min;
        int axis = (d.x > d.y && d.x > d.z) ? 0 : ((d.y > d.z) ? 1 : 2);
        float pos = (bounds.min.E[axis] + bounds.max.E[axis]) * 0.5f;
        auto bnear = bounds;
        auto bfar = bounds;
        bnear.max.E[axis] = pos;
        bfar.min.E[axis] = pos;
        return std::make_pair(bnear, bfar);
    };

    const int TriLimit = 8;
    const int RayLimit = 64;
    size_t branches = 0;

    std::function<void(const math::Bounds3&, const expanding_list<int>&, const expanding_list<int>&)>
      trace = [&](const math::Bounds3& bounds, const expanding_list<int>& tri_list, const expanding_list<int>& ray_list)
    {
        branches++;
        if (tri_list.count <= TriLimit || ray_list.count <= RayLimit)
        {
            // if (tri_list.count > 0 && ray_list.count > 0)
            // {
            //     std::vector<int> s_triangles, s_rays;
            //     s_triangles.reserve(tri_list.count);
            //     s_rays.reserve(ray_list.count);
            //     for (int i = 0; i < tri_list.count; ++i)
            //         s_triangles.emplace_back(tri_list[i]);
            //     for (int i = 0; i < ray_list.count; ++i)
            //         s_rays.emplace_back(ray_list[i]);
            //     scuffed.emplace_back(std::make_tuple(bounds, s_triangles, s_rays));
            // }
            for (int t = 0; t < tri_list.count; ++t)
            {
                const int prim_id = tri_list[t];
                const TriangleI& tri = triangles[prim_id];
                const math::Vector3& v0 = vertices[tri.A];
                const math::Vector3& v1 = vertices[tri.B];
                const math::Vector3& v2 = vertices[tri.C];
                for (int r = 0; r < ray_list.count; ++r)
                {
                    const int ray_id = ray_list[r];
                    const Ray& ray = rays[ray_id];
                    auto hit = intersect_ray_triangle(ray, v0, v1, v2);
                    if (!hit) continue;
                    auto& intersection = *hit;
                    if (intersection.distance >= ray_dist[ray_id]) continue;
                    ray_dist[ray_id] = intersection.distance;
                    ray_p[ray_id] = intersection.position;
                    ray_n[ray_id] = math::Normalize(intersection.normal_geometric);
                }
            }
            return;
        }

        auto [near_bounds, far_bounds] = split(bounds);

        auto [new_near_bounds, near_tri_list, near_ray_list] = partition(near_bounds, tri_list, ray_list);
        auto [new_far_bounds, far_tri_list, far_ray_list] = partition(far_bounds, tri_list, ray_list);

        trace(new_near_bounds, near_tri_list, near_ray_list);
        trace(new_far_bounds, far_tri_list, far_ray_list);
    };

    math::Bounds3 total_bounds;
    for (const auto& tri : triangles)
    {
        total_bounds.Union(vertices[tri.A]);
        total_bounds.Union(vertices[tri.B]);
        total_bounds.Union(vertices[tri.C]);
    }
    expanding_list<int> tri_list(arena);
    for (int i = 0; i < triangles.size(); i++)
        tri_list.emplace_back(i);
    expanding_list<int> ray_list(arena);
    for (int i = 0; i < rays.size(); i++)
        ray_list.emplace_back(i);

    auto [new_bounds, new_tri_list, new_ray_list] = partition(total_bounds, tri_list, ray_list);
    trace(new_bounds, new_tri_list, new_ray_list);

    for (int i = 0; i < rays.size(); i++)
    {
        auto& record = records[i];
        record.hit = ray_dist[i] < std::numeric_limits<float>::max();
        if (record.hit)
        {
            record.p = ray_p[i];
            record.n = ray_n[i];
        }
    }

    // auto scuffed_name_id = 0;
    // for (auto& [bounds, prim_ids, ray_ids] : scuffed)
    // {
    //     auto          name_rays = "scuffed/" + std::to_string(scuffed_name_id) + ".txt";
    //     std::ofstream output(name_rays);
    //     output << prim_ids.size() << "," << ray_ids.size() << std::endl;
    //     output << bounds.min.x << "," << bounds.min.y << "," << bounds.min.z << "," << bounds.max.x << "," << bounds.max.y << "," << bounds.max.z << std::endl;
    //     for (auto& prim_id : prim_ids)
    //     {
    //         const TriangleI   & tri = triangles[prim_id];
    //         const math::Vector3& a   = vertices[tri.A];
    //         const math::Vector3& b   = vertices[tri.B];
    //         const math::Vector3& c   = vertices[tri.C];
    //         output << a.x << "," << a.y << "," << a.z << "," << b.x << "," << b.y << "," << b.z << "," << c.x << "," << c.y << "," << c.z << std::endl;
    //     }
    //     for (auto& ray_id : ray_ids)
    //     {
    //         const Ray& ray = rays[ray_id];
    //         output << ray.O.x << "," << ray.O.y << "," << ray.O.z << "," << ray.D.x << "," << ray.D.y << "," << ray.D.z << std::endl;
    //     }
    //     output.close();
    //     scuffed_name_id++;
    // }
}