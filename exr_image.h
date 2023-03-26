#pragma once
#include "tinyexr/tinyexr.h"

void save_exr()
{
    const std::string file_name = "out.exr";
    EXRHeader header;
    InitEXRHeader(&header);

    EXRImage image;
    InitEXRImage(&image);
    image.num_channels = 3;

    std::array<float *, 3> image_ptr;
    image_ptr[0] = images[2].data(); // B
    image_ptr[1] = images[1].data(); // G
    image_ptr[2] = images[0].data(); // R

    image.images = (unsigned char **)image_ptr.data();
    image.width = width;
    image.height = height;

    header.num_channels = 3;
    header.channels = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
    // Must be (A)BGR order, since most of EXR viewers expect this channel order.
    strncpy(header.channels[0].name, "B", 255);
    header.channels[0].name[strlen("B")] = '\0';
    strncpy(header.channels[1].name, "G", 255);
    header.channels[1].name[strlen("G")] = '\0';
    strncpy(header.channels[2].name, "R", 255);
    header.channels[2].name[strlen("R")] = '\0';

    header.pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
    header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
    for (int i = 0; i < header.num_channels; i++)
    {
        header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;          // pixel type of input image
        header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_HALF; // pixel type of output image to be stored in .EXR
    }

    const char *err = nullptr; // or nullptr in C++11 or later.
    const int ret = SaveEXRImageToFile(&image, &header, file_name.c_str(), &err);
    if (ret != TINYEXR_SUCCESS)
    {
        FreeEXRErrorMessage(err); // free's buffer for an error message
        return ret;
    }

    free(header.channels);
    free(header.pixel_types);
    free(header.requested_pixel_types);
}