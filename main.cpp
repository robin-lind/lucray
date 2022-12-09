// main.cpp

// MIT License
//
// Copyright (c) 2022 Robin Lind
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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <fstream>
#include <limits>
#include <ostream>
#include <iterator>
#include <string>
#include <filesystem>
#include <vector>
#include "argh/argh.h"
#include "aixlog.hpp"
#include "lucmath_gen.h"
#include "tinyobjloader/tiny_obj_loader.h"
#include "tinyexr/tinyexr.h"
#include "lucmath.h"
#include "dacrt.h"
#include "camera.h"

int main(int argc, char *argv[])
{
    auto sink_cout = std::make_shared<AixLog::SinkCout>(AixLog::Severity::trace);
    auto sink_file = std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace, "logfile.log");
    AixLog::Log::init({ sink_cout, sink_file });

    argh::parser cmdl(argc, argv);

    int spp = 1, width = 1, height = 1;
    if (!(cmdl("w") >> width))
        LOG(WARNING) << "No image width provided! Defaulting to: " << width << " (-w)!" << std::endl;
    if (!(cmdl("h") >> height))
        LOG(WARNING) << "No image height provided! Defaulting to: " << height << " (-h)!" << std::endl;
    if (!(cmdl("s") >> spp))
        LOG(WARNING) << "No samples per pixel provided! Defaulting to: " << spp << " (-s)!" << std::endl;
    bool infinite_rendering = false;
    if (spp < 0)
    {
        infinite_rendering = true;
        spp                = 1;
    }

    auto                     working_directory = std::filesystem::current_path();
    std::vector<std::string> positional(std::begin(cmdl.pos_args()) + 1, std::end(cmdl.pos_args()));
    if (positional.empty())
    {
        LOG(ERROR) << "No input files!" << std::endl;
        return 1;
    }
    std::vector<std::string> input_files;
    input_files.reserve(positional.size());
    for (const auto& pos : positional)
        input_files.emplace_back(working_directory / std::filesystem::path(pos));

    if (input_files.size() > 1)
        LOG(INFO) << "More than one input file is currently not supported!" << std::endl;

    std::string            & inputfile = input_files.front();
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = std::filesystem::path(input_files.front()).parent_path();

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(inputfile, reader_config))
    {
        if (!reader.Error().empty())
            LOG(ERROR) << "TinyObjReader: " << reader.Error() << std::endl;
        else
            LOG(ERROR) << "TinyObjReader: Unknown parse error!" << std::endl;
        return 1;
    }
    if (!reader.Warning().empty())
        LOG(WARNING) << "TinyObjReader: " << reader.Warning() << std::endl;
    const auto& attrib    = reader.GetAttrib();
    const auto& shapes    = reader.GetShapes();
    const auto& materials = reader.GetMaterials();

    std::vector<luc::Vector3> vertices;
    vertices.reserve(attrib.vertices.size() / 3);
    size_t emplaced = 0;
    for (size_t i = 0; i < attrib.vertices.size(); i += 3)
    {
        const auto& a = attrib.vertices[i + 0];
        const auto& b = attrib.vertices[i + 1];
        const auto& c = attrib.vertices[i + 2];
        vertices.emplace_back(a, b, c);
    }
    std::vector<TriangleI>    triangles;
    for (const auto& shape : shapes)
    {
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
        {
            const auto& a = shape.mesh.indices[f * 3 + 0].vertex_index;
            const auto& b = shape.mesh.indices[f * 3 + 1].vertex_index;
            const auto& c = shape.mesh.indices[f * 3 + 2].vertex_index;
            triangles.emplace_back(a, b, c);
        }
    }

    const float        fov_x = 4.5f;
    const luc::Vector3 eye{ 100.0f, 66.0f, 100.0f };
    const luc::Vector3 target{ 0.f, 0.f, 0.f };
    RayCamera              camera(width, height, eye, target, { 0.f, 1.f, 0.f }, fov_x);
    Sampler                sampler(0);
    size_t                 ray_count = static_cast<size_t>(width) * height;
    std::vector<Ray>       rays;
    std::vector<HitRecord> records;

    std::array<std::vector<float>, 3> images;
    for (auto& image : images)
        image.resize(static_cast<size_t>(width) * height);
    std::vector<luc::Vector4> framebuffer;
    framebuffer.resize(static_cast<size_t>(width) * height);

    for (int s = 0; s < spp || infinite_rendering; s++)
    {
        auto start = std::chrono::system_clock::now();
        rays.reserve(ray_count);
        records.reserve(ray_count);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                rays.emplace_back(camera.CameraRay(x, y, sampler));
                HitRecord record;
                record.idx = { x, y };
                record.hit = false;
                records.emplace_back(record);
            }
        }
        Intersect(rays, triangles, vertices, records);

        for (const auto& record : records)
        {
            luc::Vector3 color;
            if (record.hit)
                color = luc::Vector3(190.f, 33.f, 55.f) * std::max(0.f, luc::Dot(record.n, luc::Normalize(record.p - luc::Vector3(0.f, 100.f, 0.f)))); //((record.n * .5f) + .5f)
            else
                color = { 0.f };
            size_t idx   = record.idx.x + record.idx.y * width;
            auto & pixel = framebuffer[idx];
            pixel        = pixel + luc::Vector4(color, 1.f);
        }

        rays.clear();
        records.clear();

        for (size_t i = 0; i < framebuffer.size(); i++)
        {
            auto& pixel  = framebuffer[i];
            images[0][i] = pixel.r / pixel.w;
            images[1][i] = pixel.g / pixel.w;
            images[2][i] = pixel.b / pixel.w;
        }

        std::string file_name = "out.exr";
        EXRHeader   header;
        InitEXRHeader(&header);

        EXRImage image;
        InitEXRImage(&image);

        image.num_channels = 3;

        std::array<float *, 3> image_ptr;
        // float *image_ptr[3];
        image_ptr[0] = images[2].data(); // B
        image_ptr[1] = images[1].data(); // G
        image_ptr[2] = images[0].data(); // R

        image.images = (unsigned char **)image_ptr.data();
        image.width  = width;
        image.height = height;

        header.num_channels = 3;
        header.channels     = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
        // Must be (A)BGR order, since most of EXR viewers expect this channel order.
        strncpy(header.channels[0].name, "B", 255);
        header.channels[0].name[strlen("B")] = '\0';
        strncpy(header.channels[1].name, "G", 255);
        header.channels[1].name[strlen("G")] = '\0';
        strncpy(header.channels[2].name, "R", 255);
        header.channels[2].name[strlen("R")] = '\0';

        header.pixel_types           = (int *)malloc(sizeof(int) * header.num_channels);
        header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
        for (int i = 0; i < header.num_channels; i++)
        {
            header.pixel_types[i]           = TINYEXR_PIXELTYPE_FLOAT; // pixel type of input image
            header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_HALF;  // pixel type of output image to be stored in .EXR
        }

        const char *err = nullptr; // or nullptr in C++11 or later.
        int         ret = SaveEXRImageToFile(&image, &header, file_name.c_str(), &err);
        if (ret != TINYEXR_SUCCESS)
        {
            FreeEXRErrorMessage(err); // free's buffer for an error message
            return ret;
        }

        free(header.channels);
        free(header.pixel_types);
        free(header.requested_pixel_types);

        if (infinite_rendering) spp = s + 1;
        auto end  = std::chrono::system_clock::now();
        auto span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (span.count() < 9999)
        {
            LOG(INFO) << "Sample: " << (s + 1) << "/" << spp << " in " << span.count() << "ms" << std::endl;
        }
        else
        {
            span = std::chrono::duration_cast<std::chrono::seconds>(end - start);
            LOG(INFO) << "Sample: " << (s + 1) << "/" << spp << " in " << span.count() << "s" << std::endl;
        }
    }

    return 0;
}