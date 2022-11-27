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
    VectorTN() {}
    VectorTN(T x_) { std::fill(std::begin(e), std::end(e), x_); }
    VectorTN(const std::array<T, N>& a) : e(a) {}
    std::array<T, N> e{};
};

template <typename T>
union VectorTN<T,2>
{
    VectorTN() {}
    VectorTN(T x_) : e{ x_, x_ } {}
    VectorTN(T x_, T y_) : e{ x_, y_ } {}
    VectorTN(const std::array<T, 2>& a) : e(a) {}
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
    std::array<T, 2> e{};
};
template <typename T>
union VectorTN<T,3>
{
    VectorTN() {}
    VectorTN(T x_) : e{ x_, x_, x_ } {}
    VectorTN(const VectorTN<T,2>& v, T s) : e{ v.x, v.y, s } {}
    VectorTN(T s, const VectorTN<T,2>& v) : e{ s, v.x, v.y } {}
    VectorTN(const VectorTN<T,2>& v) : VectorTN<T,3>(v, static_cast<T>(0)) {}
    VectorTN(T x_, T y_, T z_) : e{ x_, y_, z_ } {}
    VectorTN(const std::array<T, 3>& a) : e(a) {}
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
        VectorTN<T,2> xy;
        T z0;
    };
    struct
    {
        T x0;
        VectorTN<T,2> yz;
    };
    struct
    {
        VectorTN<T,2> uv;
        T w0;
    };
    struct
    {
        T u0;
        VectorTN<T,2> vw;
    };
    struct
    {
        VectorTN<T,2> rg;
        T b0;
    };
    struct
    {
        T r0;
        VectorTN<T,2> gb;
    };
    std::array<T, 3> e{};
};
template <typename T>
union VectorTN<T,4>
{
    VectorTN() {}
    VectorTN(T x_) : e{ x_, x_, x_, x_ } {}
    VectorTN(const VectorTN<T,2>& u, const VectorTN<T,2>& v) : e{ u.x, u.y, v.x, v.y } {}
    VectorTN(const VectorTN<T,2>& v, T s, T t) : e{ v.x, v.y, s, t } {}
    VectorTN(const VectorTN<T,2>& v) : VectorTN(v, static_cast<T>(0), static_cast<T>(0)) {}
    VectorTN(const VectorTN<T,3>& v, T s) : e{ v.x, v.y, v.z, s } {}
    VectorTN(T s, const VectorTN<T,3>& v) : e{ s, v.x, v.y, v.z } {}
    VectorTN(const VectorTN<T,3>& v) : VectorTN(v, static_cast<T>(0)) {}
    VectorTN(T x_, T y_, T z_, T w_) : e{ x_, y_, z_, w_ } {}
    VectorTN(const std::array<T, 4>& a) : e(a) {}
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
        VectorTN<T,3> xyz;
        T w0;
    };
    struct
    {
        T x0;
        VectorTN<T,3> yzw;
    };
    struct
    {
        VectorTN<T,2> xy;
        VectorTN<T,2> zw;
    };
    struct
    {
        T x1;
        VectorTN<T,2> yz;
        T w1;
    };
    struct
    {
        VectorTN<T,3> rgb;
        T a0;
    };
    struct
    {
        T r0;
        VectorTN<T,3> gba;
    };
    struct
    {
        VectorTN<T,2> rg;
        VectorTN<T,2> ba;
    };
    struct
    {
        T r1;
        VectorTN<T,2> gb;
        T a1;
    };
    std::array<T, 4> e{};
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

template<typename T, size_t C, size_t R>
union MatrixTNM
{
    MatrixTNM() {};
    MatrixTNM(T x_) { std::fill(std::begin(e), std::end(e), x_); }
    MatrixTNM(const std::array<T, C*R>& a) : e(a) {}
    MatrixTNM(const std::array<VectorTN<T,R>, C>& a) : c(a) {}
    std::array<VectorTN<T,R>, C> c;

    std::array<VectorTN<T,C>, R> r;

    std::array<T, C*R> e{};
};

template <typename Op = std::plus<void>, typename T, size_t N>
auto Collapse(const VectorTN<T,N>& a)
{
    /**/ if constexpr (N == 2) return Op{} (a.x, a.y);
    else if constexpr (N == 3) return Op{} (Op{} (a.x, a.y), a.z);
    else if constexpr (N == 4) return Op{} (Op{} (Op{} (a.x, a.y), a.z), a.w);
    else if constexpr (N > 4) return std::accumulate(std::begin(a.e), std::end(a.e), static_cast<T>(0));
}

template <typename Op, typename T, size_t N>
auto Reduce(const std::array<T,N>& a, const std::array<T,N>& b)
{
    auto result = [&]<std::size_t... I>(std::index_sequence<I...>)
    {
        return std::array<T,N>{ Op{}(std::get<I>(a), std::get<I>(b)) ... };
    } (std::make_index_sequence<N>{});
    return result;
}

template <typename Op, typename T, size_t N>
auto Reduce(const VectorTN<T,N>& a, const VectorTN<T,N>& b)
{
    return VectorTN<T,N>(Reduce<Op, T, N>(a.e, b.e));
}

template <typename Op, typename T, typename U>
auto Reduce(const T& a, const U& s) requires std::is_arithmetic_v<U>
{
    return Reduce<Op>(a, T { s });
}

template <typename Op, typename T, typename U>
auto Reduce(const U& s, const T& a) requires std::is_arithmetic_v<U>
{
    return Reduce<Op>(a, s);
}

template <typename T, typename U> auto operator+(T&& a, U&& b) { return Reduce<std::plus<void>>(std::forward<T>(a), std::forward<U>(b)); }
template <typename T, typename U> auto operator-(T&& a, U&& b) { return Reduce<std::minus<void>>(std::forward<T>(a), std::forward<U>(b)); }
template <typename T, typename U> auto operator*(T&& a, U&& b) { return Reduce<std::multiplies<void>>(std::forward<T>(a), std::forward<U>(b)); }
template <typename T, typename U> auto operator/(T&& a, U&& b) { return Reduce<std::divides<void>>(std::forward<T>(a), std::forward<U>(b)); }
template <typename T, typename U> void operator+=(T& a, U&& b) { a = a + b; }
template <typename T, typename U> void operator-=(T& a, U&& b) { a = a - b; }
template <typename T, typename U> void operator*=(T& a, U&& b) { a = a * b; }
template <typename T, typename U> void operator/=(T& a, U&& b) { a = a / b; }

template <typename T>
auto Dot(const T& a, const T& b)
{
    return Collapse(a * b);
}
template <typename T>
VectorTN<T,3> Cross(const VectorTN<T,3>& a, const VectorTN<T,3>& b)
{
    const VectorTN<T,3> result {
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)
    };
    return result;
}
template <typename T>
auto LengthSquared(const T& a)
{
    return Dot(a, a);
}
template <typename T>
auto Length(const T& a)
{
    return std::sqrt(LengthSquared(a));
}
template <typename T>
T DistanceSquared(const VectorTN<T,3>& a, const VectorTN<T,3>& b)
{
    return LengthSquared(a - b);
}
template <typename T>
T Distance(const VectorTN<T,3>& a, const VectorTN<T,3>& b)
{
    return Length(a - b);
}
template <typename T>
auto Normalize(T&& a)
{
    return a / Length(a);
}
template <typename T, size_t N>
auto NormalizedWithLength(const T& a)
{
    const auto length = Length(a);
    /**/ if constexpr (N == 2) return VectorTN<T,3>(a / length, length);
    else if constexpr (N == 3) return VectorTN<T,4>(a / length, length);
}

typedef VectorTN<float,2> Vector2;
typedef VectorTN<float,3> Vector3;
typedef VectorTN<float,4> Vector4;
// typedef Normal2T<float> Normal2;
typedef MatrixTNM<float,3,3> Matrix3;
typedef MatrixTNM<float,4,4> Matrix4;
typedef MatrixTNM<float,3,4> Affine3;
typedef MatrixTNM<float,4,3> Affine4;

template <typename T, size_t C, size_t R>
auto Mul(const MatrixTNM<T,C,R>& mat, const VectorTN<T,C>& vec)
{
    auto result = [&]<std::size_t... I>(std::index_sequence<I...>) -> VectorTN<T,R>
    {
        return ((std::get<I>(mat.c) * std::get<I>(vec.e)) + ...);
    } (std::make_index_sequence<C>{});
    return result;
}

template <typename T, size_t C, size_t R>
auto Mul(const MatrixTNM<T,C,R>& Left, const MatrixTNM<T,C,R>& Right)
{
    auto result = [&]<std::size_t... I>(std::index_sequence<I...>) -> MatrixTNM<T,C,R>
    {
        return { { Mul(Left, std::get<I>(Right.r)) ... } };
    } (std::make_index_sequence<R>{});
    return result;
}

};

#endif /* LUCRAY_MATH_H */