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

#ifndef DIFFUSE_H
#define DIFFUSE_H

#include "../math/vector.h"
#include <limits>
#include <numbers>
#include <cmath>
#include <utility>
#include "../scene.h"

namespace luc {
template<typename T>
struct lambertian_reflection {
    static auto sample(const material_sample& material, const math::vector<T, 3>& wo, const std::array<T, 3>& rand)
    {
        const auto z = std::sqrt(rand[0]);
        const auto r = std::sqrt(T(1) - z * z);
        const auto phi = T(2) * std::numbers::pi_v<T> * rand[1];
        const math::vector<T, 3> wi(r * std::cos(phi), r * std::sin(phi), z);
        return wi;
    }

    static auto eval(const material_sample& material, const math::vector<T, 3>& wo, const math::vector<T, 3>& wi)
    {
        const auto diffuse = material.albedo / std::numbers::pi_v<T> * wi.z;
        return diffuse;
    }

    static auto pdf(const material_sample& material, const math::vector<T, 3>& wo, const math::vector<T, 3>& wi)
    {
        const auto p = wi.z / std::numbers::pi_v<T>;
        return p;
    }
};
} // namespace luc
#endif
