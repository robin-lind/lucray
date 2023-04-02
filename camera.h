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

    ray_camera() = default;

    ray_camera(const math::vector<T, 3>& eye, const math::vector<T, 3>& dir, const math::vector<T, 3>& up, T aspect, T yfov) : pos(eye)
    {
        constexpr auto half_pi = std::numbers::pi_v<double> * .5;
        const auto half_sh = .5;
        const auto half_sw = static_cast<double>(aspect) * .5;
        const auto c = half_sh / std::tan(static_cast<double>(yfov) * .5);
        const auto r = std::sqrt(c * c + half_sw * half_sw);
        const auto xfov = half_pi - std::asin(c / r);
        const auto bf = half_sh / std::tan(xfov);

        Z = math::normalize(dir) * static_cast<T>(bf);
        X = math::normalize(math::cross(Z, up));
        Y = math::normalize(math::cross(Z, X)) / aspect;
    }

    std::pair<math::vector<T, 3>, math::vector<T, 3>> ray(math::vector<T, 2> uv) const
    {
        const auto dir = X * uv.u + Y * uv.v + Z;
        return std::make_pair(pos, dir);
    }
};

} // namespace luc
#endif