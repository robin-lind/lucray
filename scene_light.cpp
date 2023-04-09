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

#include "math/vector.h"
#include "math/utils.h"
#include "scene.h"
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <utility>
#include "sampler.h"

namespace luc {
template<typename T>
math::vector<T, 3> sample_cosine_hemisphere(T u, T v)
{
    const auto z = std::sqrt(u);
    const auto r = std::sqrt(T(1) - z * z);
    const auto phi = T(2) * std::numbers::pi_v<T> * v;
    const math::vector<T, 3> result(r * std::cos(phi), r * std::sin(phi), z);
    return result;
}

template<typename T>
auto sample_cosine_hemisphere_pdf(const math::vector<T, 3>& dir)
{
    return (dir.z <= 0) ? T(0) : dir.z / std::numbers::pi_v<T>;
}

template<typename T>
std::optional<std::pair<math::vector<T, 3>, math::vector<T, 3>>> sample_and_eval_diffuse(const math::vector<T, 3>& albedo, const math::vector<T, 3>& wo, const std::array<T, 2>& rand)
{
    const auto wi = sample_cosine_hemisphere(rand[0], rand[1]);
    const auto NdotWo = wo.z;
    const auto NdotWi = wi.z;
    if (NdotWi * NdotWo <= 0) return std::nullopt;
    const auto diffuse = albedo / std::numbers::pi_v<T> * NdotWi;
    return std::make_pair(wi, diffuse);
}

template<typename T>
T sample_diffuse_pdf(const math::vector<T, 3>& wo, const math::vector<T, 3>& wi)
{
    const auto NdotWo = wo.z;
    const auto NdotWi = wi.z;
    if (NdotWi * NdotWo <= 0)
        return std::numeric_limits<T>::infinity();
    return sample_cosine_hemisphere_pdf(wi);
}

template<typename T>
std::optional<std::pair<math::vector<T, 3>, math::vector<T, 3>>> sample_and_eval_bsdf(const material_sample& material, const math::vector<T, 3>& wo, const std::array<T, 2>& rand)
{
    switch (material.type) {
        case material_type::diffuse:
            return sample_and_eval_diffuse(material.albedo, wo, rand);
        default:
            return std::nullopt;
    }
}

template<typename T>
auto sample_bsdf_pdf(const material_sample& material, const math::vector<T, 3>& wo, const math::vector<T, 3>& wi)
{
    switch (material.type) {
        case material_type::diffuse:
            return sample_diffuse_pdf(wo, wi);
        default:
            return std::numeric_limits<T>::infinity();
    }
}

std::optional<std::pair<math::float3, scene::intersection>> scene_light(const luc::scene& scene, std::mt19937& rng, const math::float3& ray_org, const math::float3& ray_dir)
{
    std::optional<scene::intersection> first_hit;
    math::float3 throughput(1.f);
    math::float3 radiance(0.f);
    auto add_light = [&radiance, &throughput](const math::float3& light) {
        radiance += throughput * light;
    };
    math::float3 org = ray_org, dir = ray_dir;
    const int max_depth = 500;
    for (int depth = 0; depth < max_depth; depth++) {
        if (const auto hit = scene.intersect(org, dir)) {
            if (!first_hit.has_value())
                first_hit = *hit;
            if (hit->emission.has_value()) {
                add_light(*hit->emission);
                break;
            }
            const math::ortho_normal_base ortho(hit->normal_s);
            std::uniform_real_distribution<float> uni(0, 1);

            const auto epsilon = std::numeric_limits<float>::epsilon();
            const math::float3 offset(
              hit->normal_g.x > 0 ? epsilon : -epsilon,
              hit->normal_g.y > 0 ? epsilon : -epsilon,
              hit->normal_g.z > 0 ? epsilon : -epsilon);
            org = hit->position + offset;

            const std::array<float, 2> rand{ uni(rng), uni(rng) };
            material_sample material;
            material.type = material_type::diffuse;
            material.albedo = hit->albedo;
            const auto wo = ortho.to_local(-dir);
            if (const auto s = sample_and_eval_diffuse(material.albedo, wo, rand)) {
                math::float3 wi, color;
                std::tie(wi, color) = *s;
                const auto pdf = sample_diffuse_pdf(wo, wi);
                const auto l = color / pdf;
                throughput *= l;
                dir = ortho.to_world(wi);
            }
            else {
                break;
            }
            continue;
        }
        break;
    }
    if (first_hit.has_value())
        return std::make_pair(radiance, *first_hit);
    return std::nullopt;
}
} // namespace luc
