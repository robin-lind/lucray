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

template <typename T>
union Vector2T
{
    Vector2T() {}
    Vector2T(T x_) : e{ x_, x_ } {}
    Vector2T(T x_, T y_) : e{ x_, y_ } {}
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
    typedef T value_type;
};
template <typename T>
union Vector3T
{
    Vector3T() {}
    Vector3T(T x_) : e{ x_, x_, x_ } {}
    Vector3T(const Vector2T<T>& v, T s) : e{ v.x, v.y, s } {}
    Vector3T(T s, const Vector2T<T>& v) : e{ s, v.x, v.y } {}
    Vector3T(const Vector2T<T>& v) : Vector3T(v, static_cast<T>(0)) {}
    Vector3T(T x_, T y_, T z_) : e{ x_, y_, z_ } {}
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
        Vector2T<T> xy;
        T z0;
    };
    struct
    {
        T x0;
        Vector2T<T> yz;
    };
    struct
    {
        Vector2T<T> uv;
        T w0;
    };
    struct
    {
        T u0;
        Vector2T<T> vw;
    };
    struct
    {
        Vector2T<T> rg;
        T b0;
    };
    struct
    {
        T r0;
        Vector2T<T> gb;
    };
    std::array<T, 3> e{};
    typedef T value_type;
};
template <typename T>
union Vector4T
{
    Vector4T() {}
    Vector4T(T x_) : e{ x_, x_, x_, x_ } {}
    Vector4T(const Vector2T<T>& u, const Vector2T<T>& v) : e{ u.x, u.y, v.x, v.y } {}
    Vector4T(const Vector2T<T>& v, T s, T t) : e{ v.x, v.y, s, t } {}
    Vector4T(const Vector2T<T>& v) : Vector4T(v, static_cast<T>(0), static_cast<T>(0)) {}
    Vector4T(const Vector3T<T>& v, T s) : e{ v.x, v.y, v.z, s } {}
    Vector4T(T s, const Vector3T<T>& v) : e{ s, v.x, v.y, v.z } {}
    Vector4T(const Vector3T<T>& v) : Vector4T(v, static_cast<T>(0)) {}
    Vector4T(T x_, T y_, T z_, T w_) : e{ x_, y_, z_, w_ } {}
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
        Vector3T<T> xyz;
        T w0;
    };
    struct
    {
        T x0;
        Vector3T<T> yzw;
    };
    struct
    {
        Vector2T<T> xy;
        Vector2T<T> zw;
    };
    struct
    {
        T x1;
        Vector2T<T> yz;
        T w1;
    };
    struct
    {
        Vector3T<T> rgb;
        T a0;
    };
    struct
    {
        T r0;
        Vector3T<T> gba;
    };
    struct
    {
        Vector2T<T> rg;
        Vector2T<T> ba;
    };
    struct
    {
        T r1;
        Vector2T<T> gb;
        T a1;
    };
    std::array<T, 4> e{};
    typedef T value_type;
};
template <typename T>
union Normal2T
{
    Normal2T() = default;
    Normal2T(T x_) : e{ x_, x_ } {}
    Normal2T(T x_, T y_) : e{ x_, y_ } {}
    Normal2T(const Vector2T<T>& v) : e{ v.x, v.y } {}
    Normal2T(const Vector3T<T>& v) : e{ v.x, v.y } {}
    struct
    {
        T x, y;
    };
    std::array<T, 2> e{};
    typedef T value_type;
};
template <typename T>
union Matrix3T
{
    Matrix3T() = default;
    Matrix3T(T x_) : Matrix3T({ x_, x_, x_ }) {}
    Matrix3T(Vector3T<T> x_) : x(x_), y(x_), z(x_) {}
    Matrix3T(Vector3T<T> x_, Vector3T<T> y_, Vector3T<T> z_) : x(x_), y(y_), z(z_) {}
    Matrix3T(const std::array<T, 9>& a) : e(a) {}
    struct
    {
        Vector3T<T> x, y, z;
    };
    std::array<Vector3T<T>, 3> r;
    std::array<T, 9> e{};
    typedef T value_type;
};
template <typename T>
union Matrix4T
{
    Matrix4T() = default;
    Matrix4T(T x_) : Matrix4T({ x_, x_, x_, x_ }) {}
    Matrix4T(Vector4T<T> x_) : x(x_), y(x_), z(x_), w(x_) {}
    Matrix4T(Vector4T<T> x_, Vector4T<T> y_, Vector4T<T> z_, Vector4T<T> w_) : x(x_), y(y_), z(z_), w(w_) {}
    Matrix4T(const std::array<T, 16>& a) : e(a) {}
    struct
    {
        Vector4T<T> x, y, z, w;
    };
    std::array<Vector4T<T>, 4> r;
    std::array<T, 16> e{};
    typedef T value_type;
};
template <typename T>
union Affine3T
{
    Affine3T() = default;
    Affine3T(T x_) : Affine3T({ x_, x_, x_ }) {}
    Affine3T(Vector3T<T> x_) : x(x_), y(x_), z(x_), w(x_) {}
    Affine3T(Vector3T<T> x_, Vector3T<T> y_, Vector3T<T> z_, Vector3T<T> w_) : x(x_), y(y_), z(z_), w(w_) {}
    Affine3T(const std::array<T, 12>& a) : e(a) {}
    struct
    {
        Vector3T<T> x, y, z, w;
    };
    std::array<Vector3T<T>, 4> r;
    std::array<T, 12> e{};
    typedef T value_type;
};
template <typename T>
union Affine4T
{
    Affine4T() = default;
    Affine4T(T x_) : Affine4T({ x_, x_, x_, x_ }) {}
    Affine4T(Vector4T<T> x_) : x(x_), y(x_), z(x_) {}
    Affine4T(Vector4T<T> x_, Vector4T<T> y_, Vector4T<T> z_) : x(x_), y(y_), z(z_) {}
    Affine4T(const std::array<T, 12>& a) : e(a) {}
    struct
    {
        Vector4T<T> x, y, z;
    };
    std::array<Vector4T<T>, 3> r;
    std::array<T, 12> e{};
    typedef T value_type;
};

template <typename T>
auto Sum(const T& a)
{
    /**/ if constexpr (std::is_same_v<T, Vector2T<typename T::value_type>>) return a.x + a.y;
    else if constexpr (std::is_same_v<T, Vector3T<typename T::value_type>>) return a.x + a.y + a.z;
    else if constexpr (std::is_same_v<T, Vector4T<typename T::value_type>>) return a.x + a.y + a.z + a.w;
    else return std::accumulate(std::begin(a.e), std::end(a.e), static_cast<typename T::value_type>(0));
}

template <typename Op, typename T>
auto Reduce(const T& a, const T& b)
{
    if constexpr (std::is_same_v<T, Vector2T<typename T::value_type>>)
    {
        return T {
            Op{}(a.x, b.x),
            Op{}(a.y, b.y)
        };
    }
    else if constexpr (std::is_same_v<T, Vector3T<typename T::value_type>>)
    {
        return T {
            Op{}(a.x, b.x),
            Op{}(a.y, b.y),
            Op{}(a.z, b.z)
        };
    }
    else if constexpr (std::is_same_v<T, Vector4T<typename T::value_type>>)
    {
        return T {
            Op{}(a.x, b.x),
            Op{}(a.y, b.y),
            Op{}(a.z, b.z),
            Op{}(a.w, b.w)
        };
    }
}

template <typename Op, typename T, typename U>
auto Reduce(const T& a, const U& scalar) requires std::is_arithmetic_v<std::remove_cvref_t<U>>
{
    return Reduce<Op>(a, T { scalar });
}

template <typename Op, typename T, typename U>
auto Reduce(const U& scalar, const T& a) requires std::is_arithmetic_v<std::remove_cvref_t<U>>
{
    return Reduce<Op>(a, T { scalar });
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
    return Sum(a * b);
}
template <typename T>
Vector3T<T> Cross(const Vector3T<T>& a, const Vector3T<T>& b)
{
    const Vector3T<T> result {
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
T DistanceSquared(const Vector3T<T>& a, const Vector3T<T>& b)
{
    return LengthSquared(a - b);
}
template <typename T>
T Distance(const Vector3T<T>& a, const Vector3T<T>& b)
{
    return Length(a - b);
}
template <typename T>
auto Normalize(const T& a)
{
    return a / Length(a);
}
template <typename T>
auto NormalizedWithLength(const T& a)
{
    const auto length = Length(a);
    /**/ if constexpr (std::is_same_v<T, Vector2T<typename T::value_type>>) return Vector3T<typename T::value_type>(a / length, length);
    else if constexpr (std::is_same_v<T, Vector3T<typename T::value_type>>) return Vector4T<typename T::value_type>(a / length, length);
}

typedef Vector2T<float> Vector2;
typedef Vector3T<float> Vector3;
typedef Vector4T<float> Vector4;
typedef Normal2T<float> Normal2;
typedef Matrix3T<float> Matrix3;
typedef Matrix4T<float> Matrix4;
typedef Affine3T<float> Affine3;
typedef Affine4T<float> Affine4;

};

#endif /* LUCRAY_MATH_H */