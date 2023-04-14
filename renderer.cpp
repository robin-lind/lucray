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
            const float current_spp(pass);
            const float inv_next_spp(1. / double(pass + 1));
            auto item_func = [&](const int x, const int y, auto&& transform) {
                math::vector<float, 3> combined = fb.combined.get(x, y) * current_spp;
                math::vector<float, 3> diffuse_light = fb.diffuse_light.get(x, y) * current_spp;
                math::vector<float, 3> albedo = fb.albedo.get(x, y) * current_spp;
                math::vector<float, 3> shading_normal = fb.shading_normal.get(x, y) * current_spp;
                math::vector<float, 3> geometry_normal = fb.geometry_normal.get(x, y) * current_spp;
                math::vector<float, 3> position = fb.position.get(x, y) * current_spp;
                math::vector<float, 3> emission = fb.emission.get(x, y) * current_spp;
                math::vector<float, 1> specular = fb.specular.get(x, y) * current_spp;
                math::vector<float, 1> metallic = fb.metallic.get(x, y) * current_spp;
                math::vector<float, 1> roughness = fb.roughness.get(x, y) * current_spp;
                math::vector<float, 1> eta = fb.eta.get(x, y) * current_spp;
                math::vector<float, 1> transmission = fb.transmission.get(x, y) * current_spp;
                for (auto& sample : samples) {
                    math::float3 ray_org, ray_dir;
                    std::tie(ray_org, ray_dir) = scene.cameras[s.camera_id].ray(transform(sample.uv));
                    const auto hit = scene_light(scene, rng_inner, ray_org, ray_dir, s.max_depth);
                    if (hit.has_value()) {
                        const math::float3& light = hit->first;
                        const scene::intersection& inter = hit->second;
                        combined += math::sanitize(light) * sample.z;
                        const auto e = inter.emission.has_value() ? *inter.emission : math::float3();
                        emission += e * sample.z;
                        diffuse_light += math::sanitize((light - e) / inter.albedo) * sample.z;
                        albedo += inter.albedo * sample.z;
                        shading_normal += inter.normal_s * sample.z;
                        geometry_normal += inter.normal_g * sample.z;
                        position += inter.position * sample.z;
                        specular += inter.specular * sample.z;
                        metallic += inter.metallic * sample.z;
                        roughness += inter.roughness * sample.z;
                        eta += inter.eta * sample.z;
                        transmission += inter.transmission * sample.z;
                    }
                }
                fb.combined.set(x, y, combined * inv_next_spp);
                fb.diffuse_light.set(x, y, diffuse_light * inv_next_spp);
                fb.albedo.set(x, y, albedo * inv_next_spp);
                fb.shading_normal.set(x, y, shading_normal * inv_next_spp);
                fb.geometry_normal.set(x, y, geometry_normal * inv_next_spp);
                fb.position.set(x, y, position * inv_next_spp);
                fb.emission.set(x, y, emission * inv_next_spp);
                fb.specular.set(x, y, specular * inv_next_spp);
                fb.metallic.set(x, y, metallic * inv_next_spp);
                fb.roughness.set(x, y, roughness * inv_next_spp);
                fb.eta.set(x, y, eta * inv_next_spp);
                fb.transmission.set(x, y, transmission * inv_next_spp);
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
