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

#ifndef EXR_H
#define EXR_H

#include "framebuffer.h"
#include "tinyexr/tinyexr.h"
#include <stdlib.h>
#include <string.h>
#include <array>

namespace luc {
template<typename T, size_t N>
struct exr_image {
    EXRHeader header;
    EXRImage image;

    exr_image(image_n<T, N>& img)
    {
        InitEXRHeader(&header);
        InitEXRImage(&image);
        header.num_channels = N;
        header.channels = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
        header.pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
        header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
        image.images = (unsigned char **)malloc(sizeof(T *) * header.num_channels);
        for (int i = 0; i < N; i++) {
            header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
            header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
        }
        if constexpr (N == 1) {
            strncpy(header.channels[0].name, "R\0", 255);
        }
        else if constexpr (N == 2) {
            strncpy(header.channels[0].name, "G\0", 255);
            strncpy(header.channels[1].name, "R\0", 255);
        }
        else if constexpr (N == 3) {
            strncpy(header.channels[0].name, "B\0", 255);
            strncpy(header.channels[1].name, "G\0", 255);
            strncpy(header.channels[2].name, "R\0", 255);
        }
        else if constexpr (N == 4) {
            strncpy(header.channels[0].name, "A\0", 255);
            strncpy(header.channels[1].name, "B\0", 255);
            strncpy(header.channels[2].name, "G\0", 255);
            strncpy(header.channels[2].name, "R\0", 255);
        }
        for (int i = 0; i < N; i++)
            ((T**)image.images)[i] = img.channels[N - 1 - i].pixels.data();
        image.width = img.width;
        image.height = img.height;
    }

    ~exr_image()
    {
        free(image.images);
        free(header.requested_pixel_types);
        free(header.pixel_types);
        free(header.channels);
    }
};
} // namespace luc
#endif