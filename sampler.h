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

#ifndef SAMPLER_H
#define SAMPLER_H

#include "math/vector.h"
#include <random>
#include <vector>

namespace luc {
template<typename T>
struct sampler {
    std::vector<math::vector<T, 2>> samples;
    int current = 0;

    sampler(int samples_sqrt, std::mt19937& rng)
    {
        const auto inv_samples_sqrt_f = static_cast<T>(1) / static_cast<T>(samples_sqrt);
        samples.reserve(samples_sqrt * samples_sqrt);
        for (size_t v = 0; v < samples_sqrt; v++)
            for (size_t u = 0; u < samples_sqrt; u++)
                samples.emplace_back(
                  (((T)u + std::uniform_real_distribution<T>(0, 1)(rng)) * inv_samples_sqrt_f),
                  (((T)v + std::uniform_real_distribution<T>(0, 1)(rng)) * inv_samples_sqrt_f));
        const auto count = samples_sqrt * samples_sqrt;
        std::uniform_int_distribution<int> rid(0, count - 1);
        for (int i = 0; i < count; i++)
            std::swap(samples[i], samples[rid(rng)]);
    }

    auto sample()
    {
        if (current >= samples.size())
            current = 0;
        return samples[current++];
    }
};
} // namespace luc

#endif