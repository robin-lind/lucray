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

#include "filesystem.h"
#include "framebuffer.h"
#include "gltf.h"

namespace luc {

namespace inner {
luc::model load_obj(const std::filesystem::path& path)
{
    return luc::model();
}
} // namespace inner

luc::model load_file(const std::filesystem::path& path)
{
    const auto ext = path.extension();
    luc::model model;
    if (ext == ".obj")
        model = inner::load_obj(path);
    else if (ext == ".glb" || ext == ".gltf")
        model = inner::load_gltf(path);
    return model;
}
} // namespace luc
