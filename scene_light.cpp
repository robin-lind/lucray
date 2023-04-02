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
#include "scene.h"

namespace luc {

math::float3 scene_light(const luc::scene& scene, const math::float3& ray_org, const math::float3& ray_dir)
{
    math::float3 throughput(1.f);
    math::float3 radiance(0.f);
    auto add_light = [&radiance, &throughput](const math::float3& light) {
        radiance += throughput * light;
    };
    const int max_depth = 5;
    for (int depth = 0; depth < max_depth; depth++) {
        const auto hit = scene.intersect(ray_org, ray_dir);
        if (hit) {
            const auto l = math::dot(hit->normal_s, ray_dir);
            radiance += hit->albedo * l;
            if (hit->emission.has_value())
                radiance += *hit->emission;
        }
        break;
    }
    return radiance;
}
} // namespace luc
