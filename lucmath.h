// lucmath.h

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

#ifndef LUCRAY_MATH_H
#define LUCRAY_MATH_H

#include <cmath>
#include <array>
#include <functional>
#include <limits>
#include "lucmath_gen.h"

namespace luc
{

template<typename T, size_t N>
union MatrixTN
{
    MatrixTN() :
      MatrixTN(static_cast<T>(0)) {}

    MatrixTN(T t)
    {
        std::fill(std::begin(E), std::end(E), t);
    }

    MatrixTN(const std::array<T, N * N>& a) :
      E(a) {}

    MatrixTN(const std::array<VectorTN<T, N>, N>& c) :
      C(c) {}

    std::array<VectorTN<T, N>, N> C;
    std::array<T, N * N>          E{};
};

template<typename T>
union AffineT
{
    AffineT() :
      AffineT(static_cast<T>(0)) {}

    AffineT(T t)
    {
        std::fill(std::begin(E), std::end(E), t);
    }

    AffineT(const std::array<VectorTN<T, 3>, 4>& columns_) :
      columns(columns_) {}

    AffineT(const MatrixTN<T, 3>& transform_, const VectorTN<T, 3>& translation_) :
      transform(transform_), translation(translation_) {}

    AffineT(const VectorTN<T, 3>& tangent_, const VectorTN<T, 3>& bi_tangent_, const VectorTN<T, 3>& normal_, const VectorTN<T, 3>& offset_) :
      columns({ tangent_, bi_tangent_, normal_, offset_ }) {}

    std::array<VectorTN<T, 3>, 4> columns;
    VectorTN<T, 3>                tangent, bi_tangent, normal, offset;

    struct
    {
        MatrixTN<T, 3> transform;
        VectorTN<T, 3> translation;
    };

    std::array<T, 12> E{};
};

template<typename Op = std::plus<void>, typename T, size_t N>
auto Collapse(const VectorTN<T, N>&& a)
{
    if constexpr (N == 2)
        return Op{}(a.x, a.y);
    else if constexpr (N == 3)
        return Op{}(Op{}(a.x, a.y), a.z);
    else if constexpr (N == 4)
        return Op{}(Op{}(Op{}(a.x, a.y), a.z), a.w);
    else if constexpr (N > 4)
    {
        T result = a.E[0];
        for (size_t i = 1; i < N; i++)
            result = Op{}(result, a.E[i]);
        return result;
    }
}

template<typename T>
auto Cross(const VectorTN<T, 3>& a, const VectorTN<T, 3>& b) -> VectorTN<T, 3>
{
    const VectorTN<T, 3> result{
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)
    };
    return result;
}

template<typename T, size_t N>
auto Dot(const VectorTN<T, N>& a, const VectorTN<T, N>& b)
{
    const auto result = Collapse(a * b);
    return result;
}

template<typename T, size_t N>
auto LengthSquared(const VectorTN<T, N>& a)
{
    const auto result = Dot(a, a);
    return result;
}

template<typename T, size_t N>
auto Length(const VectorTN<T, N>& a)
{
    const auto result = std::sqrt(LengthSquared(a));
    return result;
}

template<typename T, size_t N>
auto DistanceSquared(const VectorTN<T, N>& a, const VectorTN<T, N>& b)
{
    const auto result = LengthSquared(a - b);
    return result;
}

template<typename T, size_t N>
auto Distance(const VectorTN<T, N>& a, const T& b)
{
    const auto result = Length(a - b);
    return result;
}

template<typename T, size_t N>
auto Normalize(const VectorTN<T, N>& t)
{
    const auto result = t / Length(t);
    return result;
}

template<typename T, size_t N>
auto NormalizedWithLength(const VectorTN<T, N>& a)
{
    const auto length = Length(a);
    return std::make_tuple(a / length, length);
}

template<typename T>
auto MakeOrthoNormalBase(const VectorTN<T, 3>& normal)
{
    // pixar technique
    // do not use sign(n.z), it can produce 0.0
    const auto sign_z     = normal.z >= 0.f ? static_cast<T>(1) : static_cast<T>(-1);
    const auto a          = static_cast<T>(-1) / (sign_z + normal.z);
    const auto b          = normal.x * normal.y * a;
    const auto tangent    = VectorTN<T, 3>(static_cast<T>(1) + sign_z * normal.x * normal.x * a, sign_z * b, -sign_z * normal.x);
    const auto bi_tangent = VectorTN<T, 3>(b, sign_z + normal.y * normal.y * a, -normal.y);
    return MatrixTN<T, 3>({ tangent, bi_tangent, normal });
}

template<typename T, size_t N>
auto Min(const VectorTN<T, N>& t, const VectorTN<T, N>& u)
{
    auto result = [&]<std::size_t... I>(std::index_sequence<I...>)
    {
        return VectorTN<T, N>(std::min(std::get<I>(t.E), std::get<I>(u.E))...);
    }
    (std::make_index_sequence<N>{});
    return result;
}

template<typename T, size_t N>
auto Max(const VectorTN<T, N>& t, const VectorTN<T, N>& u)
{
    auto result = [&]<std::size_t... I>(std::index_sequence<I...>)
    {
        return VectorTN<T, N>(std::max(std::get<I>(t.E), std::get<I>(u.E))...);
    }
    (std::make_index_sequence<N>{});
    return result;
}

template<size_t N>
bool AllTrue(VectorTN<bool, N> t)
{
    bool result = std::apply([](auto&&...v)
                             { return (v && ...); },
                             t.E);
    return result;
}

template<size_t N>
bool AnyTrue(VectorTN<bool, N> t)
{
    bool result = std::apply([](auto&&...v)
                             { return (v || ...); },
                             t.E);
    return result;
}

template<typename T, size_t N>
struct Bounds
{
    Bounds() :
      min(std::numeric_limits<T>::max()), max(-std::numeric_limits<T>::max()) {}

    template<typename... Ts>
    Bounds(const VectorTN<T, N>& t, const Ts&...args) :
      min(t), max(t)
    {
        constexpr auto                         Size = sizeof...(args);
        const std::array<VectorTN<T, N>, Size> items{ VectorTN<T, N>(args)... };
        for (const auto& v : items)
            Union(v);
        // {
        //     for (size_t i = 0; i < N; i++)
        //     {
        //         min.E[i] = std::min(min.E[i], t.E[i]);
        //         max.E[i] = std::max(max.E[i], t.E[i]);
        //     }
        // }
    }

    // auto Union(const Bounds<T, N>& t)
    // {
    //     min = luc::Min(min, t.min);
    //     max = luc::Max(max, t.max);
    //     return *this;
    // }

    // auto Union(const VectorTN<T, N>& t)
    // {
    //     min = luc::Min(min, t);
    //     max = luc::Max(max, t);
    //     return *this;
    // }

    void Union(VectorTN<T, N> t)
    {
        for (size_t i = 0; i < N; i++)
        {
            min.E[i] = std::min(min.E[i], t.E[i]);
            max.E[i] = std::max(max.E[i], t.E[i]);
        }
    }

    VectorTN<T, N> min, max;
};

using Bounds2 = Bounds<float, 2>;
using Bounds3 = Bounds<float, 3>;
using Bounds4 = Bounds<float, 4>;

}; // namespace luc

#endif /* LUCRAY_MATH_H */