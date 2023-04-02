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

#ifndef MODEL_H
#define MODEL_H

#include <memory>
#include <optional>
#include <vector>
#include "math/math.h"
#include "image.h"
#include "camera.h"

namespace luc {
struct model {
    struct mesh {
        struct submesh {
            std::vector<unsigned int> indices;
            std::vector<math::float3> vertices;
            std::optional<std::vector<math::float3>> normals;
            std::optional<std::vector<math::float2>> texcoords;
            int material;
        };

        std::vector<submesh> meshes;
    };

    struct material {
        template<typename T, size_t N>
        struct color {
            math::vector<T, N> value;
            std::optional<std::shared_ptr<texture<T, N>>> texture;
        };

        color<float,3> albedo;
        color<float,3> emission;
        float emissive_strength = 1.f;
        color<float,1> metallic;
        color<float,1> roughness;
        color<float,1> ior;
        color<float,3> specular;
        color<float,1> transmission;
    };

    struct instance {
        math::matrix4 transform;
        int id;
    };

    std::vector<luc::model::mesh> meshes;
    std::vector<luc::model::instance> instances;
    std::vector<luc::model::material> materials;
    std::vector<luc::ray_camera<float>> cameras;
};
} // namespace luc

#endif