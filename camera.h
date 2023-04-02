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

#ifndef CAMERA_H
#define CAMERA_H

#include "math/vector.h"
#include <numbers>
#include <utility>
#include <numeric>

namespace luc {

template<typename T>
struct ray_camera {
    math::vector<T, 3> pos;
    math::vector<T, 3> X, Y, Z;
    float backfocus;
    float inv_aspect;

    ray_camera() = default;

    ray_camera(float aspect, math::vector<T, 3> eye, math::vector<T, 3> target, math::vector<T, 3> up, const float yfov)
    {
        pos = eye;
        Z = math::normalize(target - eye);
        X = math::normalize(math::cross(Z, up));
        Y = math::normalize(math::cross(Z, X));
        const auto pi = std::numbers::pi_v<float>;
        const auto true_fov_x = 70.f * .5f * pi / 180.f;
        const auto true_backfocus = .5f / std::tan(true_fov_x);
        const auto half_pi = std::numbers::pi_v<float> * .5f;
        const auto half_fov = .5f * yfov;

        const auto sensor_height = 1.f;
        const auto half_sensor_height = sensor_height * .5f;
        const auto sensor_width = aspect;
        const auto half_sensor_width = sensor_width * .5f;

        auto A = half_fov;
        auto B = half_pi;
        auto C = pi - A - B;

        auto c = half_sensor_height / std::tan(A);

        auto R = half_pi;
        auto q = half_sensor_width;
        auto p = c;

        auto r = std::sqrt((p * p) + (q * q) - 2.f * p * q * std::cos(R));
        auto P = std::asin((std::sin(R) * p) / r);
        auto Q = pi - P - R;

        backfocus = half_sensor_height / std::tan(Q);
        inv_aspect = 1.f / aspect;
    }

    std::pair<math::vector<T, 3>, math::vector<T, 3>> ray(math::vector<T, 2> uv) const
    {
        const auto image_plane = math::normalize(math::vector<T, 3>(uv.u, uv.v * inv_aspect, backfocus));
        const auto dir = (X * image_plane.x) + (Y * image_plane.y) + (Z * image_plane.z);
        return std::make_pair(pos, dir);
    }
};

} // namespace luc
#endif