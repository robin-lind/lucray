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

#ifndef SCENE_H
#define SCENE_H

#include "model.h"
#include "math/vector.h"
#include "tinygltf/tiny_gltf.h"
#include <filesystem>

namespace luc::inner {
std::vector<uint32_t> indices_from_prim(const tinygltf::Model& gmodel, const tinygltf::Primitive& gprim);
std::vector<math::float3> vertices_from_attributes(const tinygltf::Model& gmodel, const int accessor_key);
std::vector<math::float3> normals_from_attributes(const tinygltf::Model& gmodel, const int accessor_key, std::vector<uint32_t>& indices);
std::vector<math::float2> texcoords_from_attributes(const tinygltf::Model& gmodel, const int accessor_key, std::vector<uint32_t>& indices);
luc::model::mesh process_mesh(const tinygltf::Model& gmodel, const tinygltf::Mesh& gmesh);
luc::model load_gltf(std::filesystem::path& path);
} // namespace luc::inner

#endif