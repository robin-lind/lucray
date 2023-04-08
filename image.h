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

#ifndef IMAGE_H
#define IMAGE_H

#include "math/utils.h"
#include "math/vector.h"
#include "tinygltf/tiny_gltf.h"
#include <cstdint>
#include <limits>
#include <type_traits>

namespace luc {

template<typename TColor>
struct image {
    int width, height;
    std::vector<TColor> pixels;
    image() = default;

    image(int _width, int _height) :
      width(_width), height(_height), pixels(width * height) {}

    TColor pixel(int x, int y) const
    {
        return pixels[x + y * width];
    }

    void pixel(int x, int y, const TColor& color)
    {
        pixels[x + y * width] = color;
    }
};

struct SamplerNearest {
    template<typename T, size_t N>
    static auto sample(const image<math::vector<T, N>>& buffer, const math::float2& uv)
    {
        const auto x = (int)std::floor(math::map<float,float>(uv.u, 0, 1, 0, (float)buffer.width));
        const auto y = (int)std::floor(math::map<float,float>(uv.v, 0, 1, 0, (float)buffer.height));
        const auto xc = math::wrap(x, 0, buffer.width);
        const auto yc = math::wrap(y, 0, buffer.height);
        const auto result = buffer.pixel(xc, yc);
        return result;
    }
};

struct SamplerBilinear {
    template<typename T, size_t N>
    static auto sample(const image<math::vector<T, N>>& buffer, const math::float2& uv)
    {
        const auto x = (int)std::floor(math::map<float,float>(uv.u, 0, 1, 0, (float)buffer.width));
        const auto y = (int)std::floor(math::map<float,float>(uv.v, 0, 1, 0, (float)buffer.height));
        const auto minx = math::wrap(x, 0, buffer.width);
        const auto maxx = math::wrap(x + 1, 0, buffer.width);
        const auto miny = math::wrap(y, 0, buffer.height);
        const auto maxy = math::wrap(y + 1, 0, buffer.height);
        const auto top_l = buffer.pixel(minx, miny);
        const auto top_r = buffer.pixel(maxx, miny);
        const auto bot_l = buffer.pixel(minx, maxy);
        const auto bot_r = buffer.pixel(maxx, maxy);
        const auto top = math::lerp(uv.u, top_l, top_r);
        const auto bot = math::lerp(uv.u, bot_l, bot_r);
        const auto result = math::lerp(uv.v, top, bot);
        return result;
    }
};

template<typename T, size_t N, typename Sampler = SamplerBilinear>
struct texture {
    image<math::vector<T, N>> buffer;
    texture() = default;

    texture(int width, int height) :
      buffer(width, height) {}

    auto sample(const math::float2& uv)
    {
        const auto result = Sampler::sample(buffer, uv);
        return result;
    }
};

template<typename T, typename S>
T convert_pixel_type(S v)
{
    if constexpr (std::is_integral_v<S> && !std::is_integral_v<T>) {
        static constexpr auto source_max = static_cast<T>(std::numeric_limits<S>::max());
        static constexpr auto source_min = static_cast<T>(std::numeric_limits<S>::lowest());
        const auto result = math::map<T>(T(v), source_min, source_max, T(0), T(1));
        return result;
    }
    else if constexpr (!std::is_integral_v<S> && std::is_integral_v<T>) {
        static constexpr auto target_max = static_cast<S>(std::numeric_limits<T>::max());
        static constexpr auto target_min = static_cast<S>(std::numeric_limits<T>::lowest());
        const auto result = (T)math::map<S>(S(v), S(0), S(1), target_min, target_max);
        return result;
    }
    else {
        const auto result = static_cast<T>(v);
        return result;
    }
}

template<typename T, typename S, size_t N>
math::vector<T, N> convert_pixel(const math::vector<S, N>& v)
{
    math::vector<T, N> result;
    result.values = [&]<std::size_t... I>(std::index_sequence<I...>)
    {
        return std::array<T, N>{ convert_pixel_type<T, S>(std::get<I>(v.values))... };
    }
    (std::make_index_sequence<N>{});
    return result;
}

template<typename S, typename T, size_t N, bool sRGB = true>
void load_raw_into_image(image<math::vector<T, N>>& image, size_t channels, const void *data)
{
    auto inv_srbg = [&](T n) {
        return n < 0.04045f ? (n / 12.92f) : std::pow(((n + 0.055f) / 1.055f), 2.4f);
    };
    const auto c_max = std::min(N, channels);
    auto *read = (S *)data;
    for (int y = 0; y < image.height; y++) {
        for (int x = 0; x < image.width; x++) {
            math::vector<T, N> result;
            for (int c = 0; c < c_max; c++) {
                result.values[c] = convert_pixel_type<T>(read[c]);
                if constexpr (sRGB)
                    result.values[c] = inv_srbg(result.values[c]);
            }
            image.pixel(x, y, result);
            read += channels;
        }
    }
}
} // namespace luc
#endif