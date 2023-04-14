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
    const char *err = nullptr;
    const auto ret = SaveEXRImageToFile(&img.image, &img.header, new_path.c_str(), &err);
    if (ret != TINYEXR_SUCCESS)
        FreeEXRErrorMessage(err);
}

template<typename T>
void save_framebuffer_exr(framebuffer<T>& fb, const std::filesystem::path& path)
{
    exr_image<T, 3> combined(fb.combined);
    exr_image<T, 3> diffuse_light(fb.diffuse_light);
    exr_image<T, 3> albedo(fb.albedo);
    exr_image<T, 3> shading_normal(fb.shading_normal);
    exr_image<T, 3> geometry_normal(fb.geometry_normal);
    exr_image<T, 3> position(fb.position);
    exr_image<T, 3> emission(fb.emission);
    exr_image<T, 1> specular(fb.specular);
    exr_image<T, 1> metallic(fb.metallic);
    exr_image<T, 1> roughness(fb.roughness);
    exr_image<T, 1> eta(fb.eta);
    exr_image<T, 1> transmission(fb.transmission);
    std::vector<const EXRHeader *> headers;
    std::vector<EXRImage> images;
    headers.reserve(12);
    images.reserve(12);
    strncpy(combined.header.name, "combined\0", 255);
    headers.push_back(&combined.header);
    images.push_back(combined.image);
    strncpy(diffuse_light.header.name, "diffuse_light\0", 255);
    headers.push_back(&diffuse_light.header);
    images.push_back(diffuse_light.image);
    strncpy(albedo.header.name, "albedo\0", 255);
    headers.push_back(&albedo.header);
    images.push_back(albedo.image);
    strncpy(shading_normal.header.name, "shading_normal\0", 255);
    headers.push_back(&shading_normal.header);
    images.push_back(shading_normal.image);
    strncpy(geometry_normal.header.name, "geometry_normal\0", 255);
    headers.push_back(&geometry_normal.header);
    images.push_back(geometry_normal.image);
    strncpy(position.header.name, "position\0", 255);
    headers.push_back(&position.header);
    images.push_back(position.image);
    strncpy(emission.header.name, "emission\0", 255);
    headers.push_back(&emission.header);
    images.push_back(emission.image);
    strncpy(specular.header.name, "specular\0", 255);
    headers.push_back(&specular.header);
    images.push_back(specular.image);
    strncpy(metallic.header.name, "metallic\0", 255);
    headers.push_back(&metallic.header);
    images.push_back(metallic.image);
    strncpy(roughness.header.name, "roughness\0", 255);
    headers.push_back(&roughness.header);
    images.push_back(roughness.image);
    strncpy(eta.header.name, "eta\0", 255);
    headers.push_back(&eta.header);
    images.push_back(eta.image);
    strncpy(transmission.header.name, "transmission\0", 255);
    headers.push_back(&transmission.header);
    images.push_back(transmission.image);
    const auto new_path = (path.parent_path() / path.filename()).replace_extension("exr");
    const char *err = nullptr;
    const auto ret = SaveEXRMultipartImageToFile(images.data(), headers.data(), headers.size(), new_path.c_str(), &err);
    if (ret != TINYEXR_SUCCESS)
        FreeEXRErrorMessage(err);
}
} // namespace luc

#endif