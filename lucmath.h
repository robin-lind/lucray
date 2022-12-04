// math.h

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
#include <numeric>

namespace luc
{

template<typename T, size_t N>
union VectorTN
{
    VectorTN() = default;

    VectorTN(T t)
    {
        std::fill(std::begin(E), std::end(E), t);
    }

    VectorTN(const std::array<T, N>& a) :
      E(a) {}

    std::array<T, N> E{};
};

template<typename T>
union VectorTN<T, 2>
{
    VectorTN() = default;

    VectorTN(T t) :
      E{ t, t } {}

    VectorTN(T x_, T y_) :
      E{ x_, y_ } {}

    VectorTN(const std::array<T, 2>& a) :
      E(a) {}

    struct
    {
        T x, y;
    };

    struct
    {
        T u, v;
    };

    struct
    {
        T w, h;
    };

    std::array<T, 2> E{};
};

template<typename T>
union VectorTN<T, 3>
{
    VectorTN() = default;

    VectorTN(T t) :
      E{ t, t, t } {}

    VectorTN(const VectorTN<T, 2>& v, T s) :
      E{ v.x, v.y, s } {}

    VectorTN(T s, const VectorTN<T, 2>& v) :
      E{ s, v.x, v.y } {}

    VectorTN(const VectorTN<T, 2>& v) :
      VectorTN<T, 3>(v, static_cast<T>(0)) {}

    VectorTN(T x_, T y_, T z_) :
      E{ x_, y_, z_ } {}

    VectorTN(const std::array<T, 3>& a) :
      E(a) {}

    struct
    {
        T x, y, z;
    };

    struct
    {
        T u, v, w;
    };

    struct
    {
        T r, g, b;
    };

    struct
    {
        VectorTN<T, 2> xy;
        T              z0;
    };

    struct
    {
        T              x0;
        VectorTN<T, 2> yz;
    };

    struct
    {
        VectorTN<T, 2> uv;
        T              w0;
    };

    struct
    {
        T              u0;
        VectorTN<T, 2> vw;
    };

    struct
    {
        VectorTN<T, 2> rg;
        T              b0;
    };

    struct
    {
        T              r0;
        VectorTN<T, 2> gb;
    };

    std::array<T, 3> E{};
};

template<typename T>
union VectorTN<T, 4>
{
    VectorTN() = default;

    VectorTN(T t) :
      E{ t, t, t, t } {}

    VectorTN(const VectorTN<T, 2>& u, const VectorTN<T, 2>& v) :
      E{ u.x, u.y, v.x, v.y } {}

    VectorTN(const VectorTN<T, 2>& v, T s, T t) :
      E{ v.x, v.y, s, t } {}

    VectorTN(const VectorTN<T, 2>& v) :
      VectorTN(v, static_cast<T>(0), static_cast<T>(0)) {}

    VectorTN(const VectorTN<T, 3>& v, T s) :
      E{ v.x, v.y, v.z, s } {}

    VectorTN(T s, const VectorTN<T, 3>& v) :
      E{ s, v.x, v.y, v.z } {}

    VectorTN(const VectorTN<T, 3>& v) :
      VectorTN(v, static_cast<T>(0)) {}

    VectorTN(T x_, T y_, T z_, T w_) :
      E{ x_, y_, z_, w_ } {}

    VectorTN(const std::array<T, 4>& a) :
      E(a) {}

    struct
    {
        T x, y, z, w;
    };

    struct
    {
        T r, g, b, a;
    };

    struct
    {
        VectorTN<T, 3> xyz;
        T              w0;
    };

    struct
    {
        T              x0;
        VectorTN<T, 3> yzw;
    };

    struct
    {
        VectorTN<T, 2> xy;
        VectorTN<T, 2> zw;
    };

    struct
    {
        T              x1;
        VectorTN<T, 2> yz;
        T              w1;
    };

    struct
    {
        VectorTN<T, 3> rgb;
        T              a0;
    };

    struct
    {
        T              r0;
        VectorTN<T, 3> gba;
    };

    struct
    {
        VectorTN<T, 2> rg;
        VectorTN<T, 2> ba;
    };

    struct
    {
        T              r1;
        VectorTN<T, 2> gb;
        T              a1;
    };

    std::array<T, 4> E{};
};

// template <typename T>
// union Normal2T
// {
//     Normal2T() = default;
//     Normal2T(T x_) : e{ x_, x_ } {}
//     Normal2T(T x_, T y_) : e{ x_, y_ } {}
//     Normal2T(const VectorTN<T,2>& v) : e{ v.x, v.y } {}
//     Normal2T(const VectorTN<T,3>& v) : e{ v.x, v.y } {}
//     struct
//     {
//         T x, y;
//     };
//     std::array<T, 2> e{};
//     typedef T value_type;
// };
template<typename T, size_t N>
union MatrixTN
{
    MatrixTN() = default;
    ;

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
    AffineT() = default;
    ;

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
auto Collapse(const VectorTN<T, N>& a)
{
    if constexpr (N == 2)
        return Op{}(a.x, a.y);
    else if constexpr (N == 3)
        return Op{}(Op{}(a.x, a.y), a.z);
    else if constexpr (N == 4)
        return Op{}(Op{}(Op{}(a.x, a.y), a.z), a.w);
    else if constexpr (N > 4)
        return std::accumulate(std::begin(a.E), std::end(a.E), static_cast<T>(0));
}

template<typename Op, typename T, size_t N>
auto Reduce(const std::array<T, N>& a, const std::array<T, N>& b)
{
    auto result = [&]<std::size_t... I>(std::index_sequence<I...>)
    {
        return std::array<T, N>{ Op{}(std::get<I>(a), std::get<I>(b))... };
    }
    (std::make_index_sequence<N>{});
    return result;
}

template<typename Op, typename T, size_t N>
auto Reduce(const VectorTN<T, N>& a, const VectorTN<T, N>& b)
{
    const auto result = VectorTN<T, N>(Reduce<Op, T, N>(a.E, b.E));
    return result;
}

template<typename Op, typename T, typename U>
auto Reduce(const T& a, const U& s) requires std::is_arithmetic_v<U>
{
    const auto result = Reduce<Op>(a, T{ s });
    return result;
}

template<typename Op, typename T, typename U>
auto Reduce(const U& s, const T& a) requires std::is_arithmetic_v<U>
{
    const auto result = Reduce<Op>(a, s);
    return result;
}

template<typename Op = std::multiplies<void>, typename T, size_t C>
auto Reduce(const MatrixTN<T, C>& mat, const VectorTN<T, C>& vec)
{
    const auto result = [&]<std::size_t... I>(std::index_sequence<I...>)->VectorTN<T, C>
    {
        return (Op{}(std::get<I>(mat.C), std::get<I>(vec.E)) + ...);
    }
    (std::make_index_sequence<C>{});
    return result;
}

template<typename Op = std::multiplies<void>, typename T, size_t C>
auto Reduce(const MatrixTN<T, C>& left, const MatrixTN<T, C>& right)
{
    const auto result = [&]<std::size_t... I>(std::index_sequence<I...>)->MatrixTN<T, C>
    {
        return { { Reduce<Op>(left, std::get<I>(right.C))... } };
    }
    (std::make_index_sequence<C>{});
    return result;
}

template<typename Op = std::multiplies<void>, typename T>
auto Reduce(const AffineT<T>& affine, const VectorTN<T, 3>& vec)
{
    const auto result = Reduce<Op>(affine.transform, vec);
    return result + affine.translation;
}

template<typename T>
auto Dot(const T& a, const T& b)
{
    const auto result = Collapse(a * b);
    return result;
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

template<typename T>
auto LengthSquared(const T& a)
{
    const auto result = Dot(a, a);
    return result;
}

template<typename T>
auto Length(const T& a)
{
    const auto result = std::sqrt(LengthSquared(a));
    return result;
}

template<typename T>
auto DistanceSquared(const VectorTN<T, 3>& a, const VectorTN<T, 3>& b) -> T
{
    const auto result = LengthSquared(a - b);
    return result;
}

template<typename T>
auto Distance(const VectorTN<T, 3>& a, const VectorTN<T, 3>& b) -> T
{
    const auto result = Length(a - b);
    return result;
}

template<typename T>
auto Normalize(T&& a)
{
    const auto result = a / Length(a);
    return result;
}

template<typename T, size_t N>
auto NormalizedWithLength(const T& a)
{
    const auto length = Length(a);
    if constexpr (N == 2)
        return VectorTN<T, 3>(a / length, length);
    else if constexpr (N == 3)
        return VectorTN<T, 4>(a / length, length);
}

template<typename T, typename U>
auto operator+(T&& a, U&& b)
{
    return Reduce<std::plus<void>>(std::forward<T>(a), std::forward<U>(b));
}

template<typename T, typename U>
auto operator-(T&& a, U&& b)
{
    return Reduce<std::minus<void>>(std::forward<T>(a), std::forward<U>(b));
}

template<typename T, typename U>
auto operator*(T&& a, U&& b)
{
    return Reduce<std::multiplies<void>>(std::forward<T>(a), std::forward<U>(b));
}

template<typename T, typename U>
auto operator/(T&& a, U&& b)
{
    return Reduce<std::divides<void>>(std::forward<T>(a), std::forward<U>(b));
}

template<typename T, typename U>
void operator+=(T& a, U&& b)
{
    a = a + b;
}

template<typename T, typename U>
void operator-=(T& a, U&& b)
{
    a = a - b;
}

template<typename T, typename U>
void operator*=(T& a, U&& b)
{
    a = a * b;
}

template<typename T, typename U>
void operator/=(T& a, U&& b)
{
    a = a / b;
}

using Vector2 = VectorTN<float, 2>;
using Vector3 = VectorTN<float, 3>;
using Vector4 = VectorTN<float, 4>;
// typedef Normal2T<float> Normal2;
using Matrix3 = MatrixTN<float, 3>;
using Matrix4 = MatrixTN<float, 4>;
using Affine  = AffineT<float>;

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

}; // namespace luc

#endif /* LUCRAY_MATH_H */