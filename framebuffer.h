#pragma once
#include <vector>

namespace luc
{
template<typename TColor>
struct framebuffer
{
    int width, height;
    std::vector<TColor> pixels;
    framebuffer() = default;

    framebuffer(int _width, int _height) :
      width(_width), height(_height), pixels(width * height) {}

    TColor& pixel(int x, int y)
    {
        return pixels[x + y * width];
    }
};
} // namespace luc