#pragma once
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
    std::vector<T> pixels;

    image_n<float, 3> combined;
    image_n<float, 3> albedo;
    image_n<float, 3> shading_normal;
    image_n<float, 3> geometry_normal;
    image_n<float, 3> position;
    image_n<float, 3> emission;
    image_n<float, 3> specular;
    image_n<float, 1> metallic;
    image_n<float, 1> roughness;
    image_n<float, 1> ior;
    image_n<float, 1> transmission;

    framebuffer() = default;

    framebuffer(int _width, int _height) :
      width(_width), height(_height), pixels(width * height), combined(width, height), albedo(width, height), shading_normal(width, height), geometry_normal(width, height), position(width, height), emission(width, height), specular(width, height), metallic(width, height), roughness(width, height), ior(width, height), transmission(width, height) {}

    T& pixel(int x, int y)
    {
        return pixels[x + y * width];
    }
};
} // namespace luc