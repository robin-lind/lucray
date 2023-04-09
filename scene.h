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

#include <optional>
#include "math/vector.h"
#include "math/matrix.h"
#include "filesystem.h"
#include "model.h"
#include "bvh.h"
#include "camera.h"

namespace luc {

template<typename T>
struct triangle {
    math::vector<T, 3> p0, e1, e2, n;

    triangle() = default;

    triangle(const math::vector<T, 3>& p0, const math::vector<T, 3>& p1, const math::vector<T, 3>& p2) :
      p0(p0), e1(p0 - p1), e2(p2 - p0), n(math::cross(e1, e2)) {}

    struct intersection {
        math::vector<T, 2> uv;
        T distance = std::numeric_limits<T>::max();
    };

    std::optional<intersection> intersect(const math::vector<T, 3>& org, const math::vector<T, 3>& dir) const;
};

template<typename T>
union triplet {
    triplet() :
      p0(0), e1(0), e2(0) {}

    triplet(const T& p0, const T& p1, const T& p2) :
      p0(p0), e1(p0 - p1), e2(p2 - p0) {}

    std::array<T, 3> E;

    struct {
        T p0, e1, e2;
    };
};

struct scene {
    struct intersection {
        math::float3 position;
        math::float3 normal_g;
        math::float3 normal_s;
        math::float3 albedo;
        std::optional<math::float3> emission;
        std::optional<math::vector<float, 1>> specular;
        std::optional<math::vector<float, 1>> metallic;
        std::optional<math::vector<float, 1>> roughness;
        std::optional<math::vector<float, 1>> ior;
        std::optional<math::vector<float, 1>> transmission;
    };

    struct subscene {
        math::matrix4 transform;
        math::matrix4 inverse;
        model::material material;
        std::vector<triangle<float>> triangles;
        std::optional<std::vector<triplet<math::float3>>> normals;
        std::optional<std::vector<triplet<math::float2>>> texcoords;
        bvh::Bvh<bvh::Node<float, 3>> accelerator;

        struct intersection {
            math::float3 position;
            math::float3 normal_g;
            math::float3 normal_s;
            math::float2 texcoord;
            float distance = std::numeric_limits<float>::max();
        };

        std::optional<intersection> intersect(const math::float3& org, const math::float3& dir) const;
    };

    std::vector<subscene> scenes;
    std::vector<luc::texture<float, 3>> textures;
    std::vector<ray_camera<float>> cameras;
    bvh::Bvh<bvh::Node<float, 3>> accelerator;

    void append_model(luc::model&& model);
    void commit();
    std::optional<intersection> intersect(const math::float3& org, const math::float3& dir) const;
};
} // namespace luc

#endif