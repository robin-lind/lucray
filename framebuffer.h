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

#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <vector>
#include "image.h"
#include "math/vector.h"

namespace luc {
template<typename T, size_t N>
struct image_n {
    int width, height;
    std::array<image<T>, N> channels;

    image_n(int _width, int _height) :
      width(_width), height(_height)
    {
        for (int c = 0; c < N; c++)
            channels[c] = image<T>(width, height);
    }

    void set(int x, int y, int c, T value)
    {
        channels[c].pixels[x + y * width] = value;
    }

    auto get(int x, int y, int c)
    {
        return channels[c].pixels[x + y * width];
    }

    void set(int x, int y, math::vector<T, N> value)
    {
        for (int c = 0; c < N; c++)
            channels[c].pixels[x + y * width] = value.values[c];
    }

    auto get(int x, int y)
    {
        math::vector<T, N> result;
        for (int c = 0; c < N; c++)
            result.values[c] = channels[c].pixels[x + y * width];
        return result;
    }
};

template<typename T>
struct framebuffer {
    int width, height;
    image_n<T, 3> combined;
    image_n<T, 3> diffuse_light;
    image_n<T, 3> albedo;
    image_n<T, 3> shading_normal;
    image_n<T, 3> geometry_normal;
    image_n<T, 3> position;
    image_n<T, 3> emission;
    image_n<T, 1> specular;
    image_n<T, 1> metallic;
    image_n<T, 1> roughness;
    image_n<T, 1> eta;
    image_n<T, 1> transmission;

    framebuffer() = default;

    framebuffer(int _width, int _height) :
      width(_width), height(_height), combined(width, height), diffuse_light(width, height), albedo(width, height), shading_normal(width, height), geometry_normal(width, height), position(width, height), emission(width, height), specular(width, height), metallic(width, height), roughness(width, height), eta(width, height), transmission(width, height) {}
};
} // namespace luc
#endif
