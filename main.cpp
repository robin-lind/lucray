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
#include "math/math.h"
#include "raylib.hpp"
#include "aixlog.hpp"
#include "camera.h"
#include "load_model.h"
#include "framebuffer.h"
#include "parallel_for.h"
#include "traversal.h"
#include "filesystem.h"
#include "scene.h"

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
    spp = std::max(spp, 1);

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

    luc::scene scene;
    for (auto& file_path : input_files)
        scene.append_model(luc::load_file(file_path));
    scene.commit();

    return 0;
}
