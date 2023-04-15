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
#include "materials/diffuse.h"

namespace luc {
template<typename Diffuse>
struct bsdf {
    template<typename T, size_t N>
    static auto sample(const material_sample& material, const math::vector<T, 3>& wo, random_array<T, N>& rand)
    {
        return Diffuse::sample(material, wo, rand);
        switch (material.type) {
            case material_type::diffuse_material:
                return Diffuse::sample(material, wo, rand);
        }
        return math::vector<T, 3>();
    }

    template<typename T>
    static auto eval(const material_sample& material, const math::vector<T, 3>& wo, const math::vector<T, 3>& wi)
    {
        return Diffuse::eval(material, wo, wi);
        switch (material.type) {
            case material_type::diffuse_material:
                return Diffuse::eval(material, wo, wi);
        }
        return math::vector<T, 3>();
    }

    template<typename T>
    static auto pdf(const material_sample& material, const math::vector<T, 3>& wo, const math::vector<T, 3>& wi)
    {
        return Diffuse::pdf(material, wo, wi);
        switch (material.type) {
            case material_type::diffuse_material:
                return Diffuse::pdf(material, wo, wi);
        }
        return T();
    }
};

std::optional<std::pair<math::float3, scene::intersection>> scene_light(const luc::scene& scene, std::mt19937& rng, const math::float3& ray_org, const math::float3& ray_dir, int max_depth)
{
    std::optional<scene::intersection> first_hit;
    math::float3 throughput(1.f);
    math::float3 radiance(0.f);
    auto add_light = [&radiance, &throughput](const math::float3& light) {
        radiance += throughput * light;
    };
    math::float3 org = ray_org, dir = ray_dir;
    for (int depth = 0; depth < max_depth; depth++) {
        if (const auto hit = scene.intersect(org, dir)) {
            if (!first_hit.has_value())
                first_hit = *hit;
            if (hit->emission.has_value()) {
                add_light(*hit->emission);
                break;
            }
            const auto epsilon = 1e-04f;
            const auto offset = hit->normal_g * epsilon;
            org = hit->position + offset;
            const math::ortho_normal_base ortho(hit->normal_s);
            const auto wo = ortho.to_local(-dir);
            std::uniform_real_distribution<float> uni(0, 1);
            random_array<float, 2> rand;
            for (auto& r : rand.elements)
                r = uni(rng);

            material_sample material;
            material.type = material_type::diffuse_material;
            material.albedo = hit->albedo;
            material.roughness = math::clamp<float>(hit->roughness, epsilon, 1.f);
            material.metallic = hit->metallic;
            material.specular = hit->specular;
            material.eta = hit->eta;

            using Shade = bsdf<lambertian_reflection<float>>;
            const auto wi = Shade::sample(material, wo, rand);
            dir = ortho.to_world(wi);
            if (math::dot(dir, hit->normal_g) <= 0)
                break;

            const auto color = Shade::eval(material, wo, wi);
            const auto pdf = Shade::pdf(material, wo, wi);
            const auto l = color / pdf;
            throughput *= l;
            continue;
        }
        break;
    }
    if (first_hit.has_value())
        return std::make_pair(radiance, *first_hit);
    return std::nullopt;
}
} // namespace luc
