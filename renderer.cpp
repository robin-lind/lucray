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

#include "renderer.h"
#include <chrono>
#include "aixlog.hpp"
#include <cstdint>
#include <random>
#include <cmath>
#include <vector>
#include "math/vector.h"
#include "math/utils.h"

namespace luc {
void render(const settings& s, framebuffer<float>& fb, abort_token& aborter, const scene& scene)
{
    auto render_start = std::chrono::steady_clock::now();
    int samples_completed = 0;
    int pass = 0;
    while (samples_completed < s.samples_per_pixel && !aborter.aborted) {
        const int samples_left = s.samples_per_pixel - samples_completed;
        const int samples_this_loop = samples_left < s.samples_per_loop * 2 ? samples_left : s.samples_per_loop;
        auto update_start = std::chrono::steady_clock::now();
        std::random_device rd_outer;
        std::mt19937 rng_outer(rd_outer());
        const auto samples_sqrt = (int)std::floor(std::sqrt(samples_this_loop));
        std::vector<math::float4> samples;
        for (size_t v = 0; v < samples_sqrt; v++)
            for (size_t u = 0; u < samples_sqrt; u++)
                samples.emplace_back(
                  (((float)u + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_sqrt) - .5f,
                  (((float)v + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_sqrt) - .5f,
                  1.f / (float)samples_this_loop,
                  ((float)samples.size() + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_this_loop);
        for (auto i = samples.size(); i < samples_this_loop; i++)
            samples.emplace_back(
              std::uniform_real_distribution<float>(-.5f, .5f)(rng_outer),
              std::uniform_real_distribution<float>(-.5f, .5f)(rng_outer),
              1.f / (float)samples_this_loop,
              ((float)samples.size() + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_this_loop);
        auto tile_func = [&](const work_block<int>& block) {
            std::random_device rd_inner;
            std::mt19937 rng_inner(rd_inner());
            auto item_func = [&](const int x, const int y, auto&& transform) {
                auto variance = fb.variance.get(x, y);
                math::vector<float, 3> combined;
                math::vector<float, 3> diffuse_light;
                math::vector<float, 3> albedo;
                math::vector<float, 3> shading_normal;
                math::vector<float, 3> geometry_normal;
                math::vector<float, 3> position;
                math::vector<float, 3> emission;
                math::vector<float, 1> specular;
                math::vector<float, 1> metallic;
                math::vector<float, 1> roughness;
                math::vector<float, 1> eta;
                math::vector<float, 1> transmission;
                size_t hit_count = 0;
                for (auto& sample : samples) {
                    math::float3 ray_org, ray_dir;
                    std::tie(ray_org, ray_dir) = scene.cameras[s.camera_id].ray(transform(sample.uv));
                    const auto hit = scene_light(scene, rng_inner, ray_org, ray_dir, s.max_depth);
                    hit_count++;
                    if (hit.has_value()) {
                        const math::float3& light = hit->first;
                        const scene::intersection& inter = hit->second;
                        const auto final_l = math::sanitize(light);
                        combined += final_l;
                        const auto e = inter.emission.has_value() ? *inter.emission : math::float3();
                        emission += e;
                        diffuse_light += math::sanitize((light - e) / inter.albedo);
                        albedo += inter.albedo;
                        shading_normal += inter.normal_s;
                        geometry_normal += inter.normal_g;
                        position += inter.position;
                        specular += inter.specular;
                        metallic += inter.metallic;
                        roughness += inter.roughness;
                        eta += inter.eta;
                        transmission += inter.transmission;
                        variance.t.push(math::collapse(final_l) * (1.f / 3.f));
                    }
                }
                fb.variance.set(x, y, variance);
                const auto o_spp = fb.count.get(x, y);
                const auto n_spp = o_spp + hit_count;
                fb.count.set(x, y, (uint32_t)n_spp);
                const auto o_spp_f = static_cast<float>(o_spp);
                const auto inv_n_spp_f = static_cast<float>(1. / static_cast<double>(n_spp));
                fb.combined.update(x, y, o_spp_f, inv_n_spp_f, combined);
                fb.diffuse_light.update(x, y, o_spp_f, inv_n_spp_f, diffuse_light);
                fb.albedo.update(x, y, o_spp_f, inv_n_spp_f, albedo);
                fb.shading_normal.update(x, y, o_spp_f, inv_n_spp_f, shading_normal);
                fb.geometry_normal.update(x, y, o_spp_f, inv_n_spp_f, geometry_normal);
                fb.position.update(x, y, o_spp_f, inv_n_spp_f, position);
                fb.emission.update(x, y, o_spp_f, inv_n_spp_f, emission);
                fb.specular.update(x, y, o_spp_f, inv_n_spp_f, specular);
                fb.metallic.update(x, y, o_spp_f, inv_n_spp_f, metallic);
                fb.roughness.update(x, y, o_spp_f, inv_n_spp_f, roughness);
                fb.eta.update(x, y, o_spp_f, inv_n_spp_f, eta);
                fb.transmission.update(x, y, o_spp_f, inv_n_spp_f, transmission);
            };
            iterate_over_tile(block, aborter, item_func);
        };
        const auto domain = generate_parallel_for_domain_rows(0, s.width, 0, s.height);
        parallel_for<int, true>(domain, tile_func, &aborter);
        if (!aborter.aborted) {
            save_framebuffer_exr(fb, s.image_name);
            samples_completed += samples_this_loop;
        }
        const auto update_end = std::chrono::steady_clock::now();
        const std::chrono::duration<double> update_elapsed = update_end - update_start;
        LOG(INFO) << "pass(" << pass << ") " << samples_this_loop << ">" << samples_completed << "/" << s.samples_per_pixel << " in: " << update_elapsed << " etr: " << (update_elapsed / samples_this_loop) * (s.samples_per_pixel - samples_completed) << "\n";
        pass++;
    }
    const auto render_end = std::chrono::steady_clock::now();
    const std::chrono::duration<double> render_elapsed = render_end - render_start;
    LOG(INFO) << "complete(" << samples_completed << "/" << s.samples_per_pixel << ") in: " << render_elapsed << "\n";
    aborter.abort();
}
} // namespace luc
