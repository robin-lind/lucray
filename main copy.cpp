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
#include <random>
#include <ostream>
#include <iterator>
#include <string>
#include <filesystem>
#include <vector>
#include "argh/argh.h"
#include "raylib.hpp"
#include "aixlog.hpp"
#include "math/vector.h"
#include "camera.h"
#include "load_model.h"
#include "framebuffer.h"
#include "parallel_for.h"
#include "traversal.h"

int main(int argc, char *argv[])
{
    auto sink_cout = std::make_shared<AixLog::SinkCout>(AixLog::Severity::trace);
    auto sink_file = std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace, "logfile.log");
    AixLog::Log::init({ sink_cout, sink_file });

    const argh::parser cmdl(argc, argv);

    int spp = 1, width = 1, height = 1;
    if (!(cmdl("w") >> width))
        LOG(WARNING) << "No image width provided! Defaulting to: " << width << " (-w=N)!" << std::endl;
    if (!(cmdl("h") >> height))
        LOG(WARNING) << "No image height provided! Defaulting to: " << height << " (-h=N)!" << std::endl;
    if (!(cmdl("s") >> spp))
        LOG(WARNING) << "No samples per pixel provided! Defaulting to: " << spp << " (-s=N)!" << std::endl;
    bool infinite_rendering = false;
    if (spp < 0) {
        infinite_rendering = true;
        spp = 1;
    }

    const std::vector<std::string> positional(std::begin(cmdl.pos_args()) + 1, std::end(cmdl.pos_args()));
    if (positional.empty()) {
        LOG(ERROR) << "No input files!" << std::endl;
        return 1;
    }
    std::vector<std::string> input_files;
    input_files.reserve(positional.size());
    auto working_directory = std::filesystem::current_path();
    for (const auto& pos : positional)
        input_files.emplace_back(working_directory / std::filesystem::path(pos));

    if (input_files.size() > 1)
        LOG(INFO) << "More than one input file is currently not supported!" << std::endl;

    std::vector<std::tuple<int, int, int>> triangles;
    std::vector<math::float3> vertices;
    std::vector<math::float3> normals;
    std::tie(triangles, vertices, normals) = load_model(input_files.front());
    auto get_tri = [&](size_t i) {
        auto get_vec = [&](const math::float3 v) {
            return *(bvh::Vec<float, 3> *)(&v);
        };
        const auto& tri = triangles[i];
        const auto v0 = get_vec(vertices[std::get<0>(tri)]);
        const auto v1 = get_vec(vertices[std::get<1>(tri)]);
        const auto v2 = get_vec(vertices[std::get<2>(tri)]);
        return bvh::Tri<float, 3>(v0, v1, v2);
    };
    traversal<float> trav(get_tri, triangles.size());
    LOG(INFO) << "Rendering: " << width << "x" << height << " " << spp << "spp" << std::endl;

    const float fov_x = 70.f;
    // const math::float3     eye{ 100.0f, 66.0f, 100.0f };
    // const math::float3     target{ 0.f, 0.f, 0.f };
    const math::float3 eye{ 0.f, 1.31f, 4.7f };
    const math::float3 target{ 0.f, 1.f, 2.94f };
    const ray_camera<float> camera(width, height, eye, target, { 0.f, 1.f, 0.f }, fov_x);
    luc::framebuffer<math::float3> framebuffer(width, height);
    bool done = false;
    abort_token aborter;
    std::mutex preview_mutex;
    std::vector<work_range<int> *> active_ranges;
    auto render_worker = [&]() {
        int frame = 0;
        while (!aborter.aborted) {
            std::random_device rd_outer;
            std::mt19937 rng_outer(rd_outer());
            const auto sample_count = spp;
            const auto samples_sqrt = (int)std::ceil(std::sqrt(sample_count));
            const auto sample_count_true = samples_sqrt * samples_sqrt;
            std::vector<math::float4> samples;
            for (size_t v = 0; v < samples_sqrt; v++)
                for (size_t u = 0; u < samples_sqrt; u++)
                    samples.emplace_back(
                      (((float)u + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_sqrt) - .5f,
                      (((float)v + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_sqrt) - .5f,
                      1.f / (float)sample_count_true,
                      ((float)samples.size() + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)sample_count_true);
            auto start = std::chrono::steady_clock::now();
            std::cout << "frame(" << frame << ")";
            // memset(framebuffer.pixels.data(), 0, framebuffer.pixels.size() * sizeof(math::float3));
            const auto domain = generate_parallel_for_domain(width, height);
            auto tile_func = [&](const work_block<int>& block) {
                static thread_local work_range<int> *active_range = nullptr;
                if (!active_range) {
                    const std::scoped_lock preview_lock(preview_mutex);
                    active_range = new work_range<int>(block.tile);
                    active_ranges.push_back(active_range);
                }
                else {
                    *active_range = block.tile;
                }
                // std::random_device rd_inner;
                // std::mt19937 rng_inner(rd_inner());
                auto item_func = [&](const int x, const int y, auto&& transform) {
                    math::float3 color;
                    for (auto& sample : samples) {
                        math::float3 ray_org, ray_dir;
                        std::tie(ray_org, ray_dir) = camera.ray(transform(sample.xy));
                        // const auto c = render_ray(ray_org,ray_dir,trav);
                        const bool hit = trav.traverse(ray_org, ray_dir);
                        const math::float3 c = hit ? math::float3(0.f, 1.f, 0.f) : math::float3(0.f, 0.f, 0.f);
                        color += c * sample.z;
                    }
                    framebuffer.pixel(x, y) = color;
                };
                iterate_over_tile(block, aborter, item_func);
            };
            parallel_for<int, true>(domain, tile_func, aborter);
            frame++;
            const auto end = std::chrono::steady_clock::now();
            const std::chrono::duration<double> elapsed_seconds = end - start;
            std::cout << " in: " << elapsed_seconds.count() << "s" << std::endl;
            for (auto *range : active_ranges)
                delete range;
            active_ranges.clear();
        }
        done = true;
    };

    raylib::SetConfigFlags(raylib::FLAG_WINDOW_UNDECORATED);
    raylib::InitWindow(width, height, "");
    raylib::SetTargetFPS(60);
    raylib::SetWindowPosition(0, 0);
    const auto frame_texture_id = raylib::rlLoadTexture(framebuffer.pixels.data(), width, height, raylib::PIXELFORMAT_UNCOMPRESSED_R32G32B32, 1);
    auto update_and_draw_fullscreen_texture = [width, height, frame_texture_id, &framebuffer]() {
        raylib::rlUpdateTexture(frame_texture_id, 0, 0, width, height, raylib::PIXELFORMAT_UNCOMPRESSED_R32G32B32, framebuffer.pixels.data());
        raylib::rlSetTexture(frame_texture_id);
        raylib::rlBegin(RL_QUADS);
        raylib::rlColor4ub(255, 255, 255, 255);
        raylib::rlNormal3f(0.f, 0.f, 1.f);
        raylib::rlTexCoord2f(0.f, 0.f);
        raylib::rlVertex2f(0.f, 0.f);
        raylib::rlTexCoord2f(0.f, 1.f);
        raylib::rlVertex2f(0.f, (float)height);
        raylib::rlTexCoord2f(1.f, 1.f);
        raylib::rlVertex2f((float)width, (float)height);
        raylib::rlTexCoord2f(1.f, 0.f);
        raylib::rlVertex2f((float)width, 0.f);
        raylib::rlEnd();
        raylib::rlSetTexture(0);
    };
    const std::thread render_thread(render_worker);
    while (!done) {
        raylib::BeginDrawing();
        raylib::ClearBackground(raylib::BLACK);
        update_and_draw_fullscreen_texture();
        {
            const std::scoped_lock preview_lock(preview_mutex);
            for (auto *range : active_ranges)
                raylib::DrawRectangleLines(range->minx, range->miny, range->maxx - range->minx, range->maxy - range->miny, raylib::WHITE);
        }
        raylib::EndDrawing();
        if (raylib::IsKeyPressed(raylib::KeyboardKey::KEY_ESCAPE))
            aborter.abort();
    }
    raylib::CloseWindow();
    return 0;
}
