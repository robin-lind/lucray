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
#include "math/bounds.h"
#include "math/math.h"
#include "filesystem.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "model.h"
#include "bvh.h"

namespace luc {
struct scene {
    struct intersection {
        math::float3 color;
    };

    struct subscene {
        model::material material;
        math::matrix4 transform;
        math::float3 center;
        math::bounds3 bounds;
        std::vector<bvh::PrecomputedTri<float>> triangles;
        bvh::Bvh<bvh::Node<float, 3>> accelerator;
        std::optional<scene::intersection> intersect(const math::float3& org, const math::float3& dir);
    };

    std::vector<subscene> scenes;
    bvh::Bvh<bvh::Node<float, 3>> accelerator;

    void append_model(luc::model&& model);
    void commit();
    std::optional<intersection> intersect(const math::float3& org, const math::float3& dir);
};
} // namespace luc

#endif