// dacrt.h

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

#ifndef LUC_DACRT_H
#define LUC_DACRT_H

#include "lucmath.h"
#include "lucmath_gen.h"
#include "ray.h"
#include <vector>

struct TriangleI
{
    int A, B, C;
    TriangleI() = default;

    TriangleI(int a, int b, int c) :
      A(a), B(b), C(c) {}
};

struct HitRecord
{
    bool hit;
    luc::Int2 idx;
    luc::Vector3 p;
    luc::Vector3 n;
};

struct Intersection
{
    float distance;
    luc::Vector3 position;
    luc::Vector3 normal_geometric;
};

void Intersect(const std::vector<Ray>& rays, const std::vector<TriangleI>& triangles, const std::vector<luc::Vector3>& vertices, std::vector<HitRecord>& records);

#endif /* LUC_DACRT_H */
