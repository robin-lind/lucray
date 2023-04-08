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

#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <filesystem>
#include "model.h"
#include "framebuffer.h"
#include "exr.h"

namespace luc {

namespace inner {
luc::model load_obj(const std::filesystem::path& path);
} // namespace inner

luc::model load_file(const std::filesystem::path& path);

template<typename T, size_t N>
void save_exr(image_n<T, N>& image, const std::filesystem::path& path)
{
    const auto new_path = (path.parent_path() / path.filename()).replace_extension("exr");
    exr_image<T, 3> img(image);
    const char *err = nullptr; // or nullptr in C++11 or later.
    const auto ret = SaveEXRImageToFile(&img.image, &img.header, new_path.c_str(), &err);
    if (ret != TINYEXR_SUCCESS)
        FreeEXRErrorMessage(err);
}

template<typename T>
void save_framebuffer_exr(framebuffer<T>& fb, const std::filesystem::path& path)
{
    luc::save_exr(fb.albedo, path);
}
} // namespace luc

#endif