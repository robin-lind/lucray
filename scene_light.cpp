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
#include "sampler.h"

namespace luc {
math::float3 scene_light(const luc::scene& scene, sampler<float>& rng, const math::float3& ray_org, const math::float3& ray_dir)
{
    math::float3 throughput(1.f);
    math::float3 radiance(0.f);
    auto add_light = [&radiance, &throughput](const math::float3& light) {
        radiance += throughput * light;
    };
    auto cosine_direction = [](const math::float2& uv) {
        const auto phi = uv.v * std::numbers::pi_v<float> * 2.f;
        const auto sq = std::sqrt(uv.u);
        const auto x = std::cos(phi) * sq;
        const auto y = std::sin(phi) * sq;
        const auto z = std::sqrt(1.f - uv.u);
        return math::float3(x, y, z);
    };
    math::float3 org = ray_org, dir = ray_dir;
    const int max_depth = 500;
    for (int depth = 0; depth < max_depth; depth++) {
        if (const auto hit = scene.intersect(org, dir)) {
            if (hit->emission.has_value()) {
                add_light(*hit->emission);
                break;
            }
            const math::ortho_normal_base ortho(hit->normal_g);
            const auto l = math::dot(hit->normal_s, dir);
            throughput *= l * hit->albedo;
            const std::uniform_real_distribution<float> uni(0, 1);
            const math::float2 uv(rng.sample());
            dir = ortho.to_world(cosine_direction(uv));
            org = hit->position;
            continue;
        }
        break;
    }
    return radiance;
}
} // namespace luc
