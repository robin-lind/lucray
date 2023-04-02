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
#include <cstdint>
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
#include "math/math.h"
#include "raylib.hpp"
#include "aixlog.hpp"
#include "camera.h"
#include "framebuffer.h"
#include "parallel_for.h"
#include "filesystem.h"
#include "scene.h"

int main(int argc, char *argv[])
{
    auto sink_cout = std::make_shared<AixLog::SinkCout>(AixLog::Severity::trace);
    auto sink_file = std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace, "logfile.log");
    AixLog::Log::init({ sink_cout, sink_file });

    const argh::parser cmdl(argc, argv);

    int width = 1, height = 1, spp = 1, camera_id = 0;
    if (!(cmdl("w") >> width))
        LOG(WARNING) << "No image width provided! Defaulting to: " << width << " (-w=N)!\n";
    if (!(cmdl("h") >> height))
        LOG(WARNING) << "No image height provided! Defaulting to: " << height << " (-h=N)!\n";
    if (!(cmdl("s") >> spp))
        LOG(WARNING) << "No samples per pixel provided! Defaulting to: " << spp << " (-s=N)!\n";
    spp = std::max(spp, 1);
    if (!(cmdl("c") >> camera_id))
        LOG(WARNING) << "No camera selected! Defaulting to: " << camera_id << " (-c=N)!\n";

    const std::vector<std::string> positional(std::begin(cmdl.pos_args()) + 1, std::end(cmdl.pos_args()));
    if (positional.empty()) {
        LOG(ERROR) << "No input files!\n";
        return 1;
    }
    std::vector<std::string> input_files;
    input_files.reserve(positional.size());
    auto working_directory = std::filesystem::current_path();
    for (const auto& pos : positional)
        input_files.emplace_back(working_directory / std::filesystem::path(pos));

    luc::scene scene;
    for (auto& file_path : input_files)
        scene.append_model(luc::load_file(file_path));
    scene.commit();

    if (scene.cameras.empty()) {
        LOG(ERROR) << "No cameras in scene!\n";
        const float fov_x = 43.f * 0.0174532793f;
        const math::float3 root_max(scene.accelerator.get_root().get_bbox().max.values);
        const math::float3 eye = root_max * 2.f;
        const math::float3 up(0.f, 1.f, 0.f);
        scene.cameras.emplace_back(eye, -eye, up, (float)width / (float)height, fov_x);
    }

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
            LOG(INFO) << "frame(" << frame << ")";
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
                auto item_func = [&](const int x, const int y, auto&& transform) {
                    math::float3 color;
                    for (auto& sample : samples) {
                        math::float3 ray_org, ray_dir;
                        std::tie(ray_org, ray_dir) = scene.cameras[camera_id].ray(transform(sample.uv));
                        const auto hit = scene.intersect(ray_org, ray_dir);
                        if (hit.has_value())
                            color += hit->color * sample.z;
                        else
                            color += math::float3(.7f);
                    }
                    framebuffer.pixel(x, y) = color;
                };
                iterate_over_tile(block, aborter, item_func);
            };
            parallel_for<int, true>(domain, tile_func, aborter);
            frame++;
            const auto end = std::chrono::steady_clock::now();
            const std::chrono::duration<double> elapsed_seconds = end - start;
            LOG(INFO) << " in: " << elapsed_seconds.count() << "s\n";
            for (auto *range : active_ranges)
                delete range;
            active_ranges.clear();
        }
        done = true;
    };

    raylib::SetConfigFlags(raylib::FLAG_WINDOW_UNDECORATED);
    raylib::InitWindow(width, height, "");
    raylib::SetTargetFPS(60);
    raylib::SetWindowPosition(1920, 0);
    const auto frame_texture_id = raylib::rlLoadTexture(framebuffer.pixels.data(), width, height, raylib::PIXELFORMAT_UNCOMPRESSED_R32G32B32, 1);
    const std::thread render_thread(render_worker);
    while (!done) {
        raylib::BeginDrawing();
        raylib::ClearBackground(raylib::BLACK);
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
        {
            const std::scoped_lock preview_lock(preview_mutex);
            for (auto *range : active_ranges)
                raylib::DrawRectangleLines(range->minx, range->miny, range->maxx - range->minx, range->maxy - range->miny, raylib::WHITE);
        }
        raylib::DrawRectangleLines(0, 0, width, height, raylib::RED);
        raylib::EndDrawing();
        if (raylib::IsKeyPressed(raylib::KeyboardKey::KEY_ESCAPE))
            aborter.abort();
    }
    raylib::CloseWindow();
    return 0;
}
