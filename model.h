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

#include <optional>
#include <vector>
#include "math/math.h"
#include "image.h"

namespace luc {
struct model {
    struct mesh {
        struct submesh {
            std::vector<unsigned int> indices;
            std::vector<math::float3> vertices;
            std::vector<math::float3> normals;
            std::vector<math::float2> texcoords;
            int material;
        };

        std::vector<submesh> meshes;
    };

    struct material {
        template<typename TColor>
        struct color {
            TColor c;
            std::optional<int> texture;
        };

        color<math::float3> albedo;
        color<math::float3> emission;
        float emissive_strength = 1.f;
        color<float> metallic;
        color<float> roughness;
        color<float> ior;
        color<math::float3> specular;
        color<float> transmission;
    };

    enum instance_type : int {
        mesh_instance,
        camera_instance
    };

    struct instance {
        math::matrix4 transform;
        instance_type type;
        int id;
    };

    std::vector<luc::model::mesh> meshes;
    std::vector<luc::model::instance> instances;
    std::vector<luc::model::material> materials;
    std::vector<luc::texture<float,3>> textures;
};
} // namespace luc

#endif