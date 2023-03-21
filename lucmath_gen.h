// lucmath_gen.h

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

#ifndef LUCRAY_MATH_GEN_H
#define LUCRAY_MATH_GEN_H

#include <cmath>
#include <array>

namespace luc
{

template<typename T, size_t N>
union VectorTN
{
    VectorTN() : VectorTN(static_cast<T>(0)) {}
    VectorTN(const T& t) { std::fill(std::begin(E), std::end(E), t); }
    VectorTN(const std::array<T, N>& a) : E(a) {}
    std::array<T, N> E{};
};
template<typename T>
union VectorTN<T, 2>
{
	VectorTN() : VectorTN(static_cast<T>(0)) {}
	VectorTN(const T& t) : E{ t, t } {}
	VectorTN(const T& _x, const T& _y) : E{ _x, _y } {}
	struct
	{
		T x;
		T y;
	};
	struct
	{
		T r;
		T g;
	};
	struct
	{
		T u;
		T v;
	};
	struct
	{
		T w;
		T h;
	};
	std::array<T, 2> E{};
};

template<typename T>
union VectorTN<T, 3>
{
	VectorTN() : VectorTN(static_cast<T>(0)) {}
	VectorTN(const T& t) : E{ t, t, t } {}
	VectorTN(const T& _x, const T& _y, const T& _z) : E{ _x, _y, _z } {}
	VectorTN(const VectorTN<T,2>& _xy, const T& _z) : E{ _xy.x, _xy.y, _z } {}
	VectorTN(const T& _x, const VectorTN<T,2>& _yz) : E{ _x, _yz.x, _yz.y } {}
	struct
	{
		T x;
		T y;
		T z;
	};
	struct
	{
		VectorTN<T,2> xy0;
		T z0;
	};
	struct
	{
		T x1;
		VectorTN<T,2> yz1;
	};
	struct
	{
		T r;
		T g;
		T b;
	};
	struct
	{
		VectorTN<T,2> rg0;
		T b0;
	};
	struct
	{
		T r1;
		VectorTN<T,2> gb1;
	};
	struct
	{
		T u;
		T v;
		T w;
	};
	struct
	{
		VectorTN<T,2> uv0;
		T w0;
	};
	struct
	{
		T u1;
		VectorTN<T,2> vw1;
	};
	std::array<T, 3> E{};
};

template<typename T>
union VectorTN<T, 4>
{
	VectorTN() : VectorTN(static_cast<T>(0)) {}
	VectorTN(const T& t) : E{ t, t, t, t } {}
	VectorTN(const T& _x, const T& _y, const T& _z, const T& _w) : E{ _x, _y, _z, _w } {}
	VectorTN(const VectorTN<T,2>& _xy, const T& _z, const T& _w) : E{ _xy.x, _xy.y, _z, _w } {}
	VectorTN(const VectorTN<T,3>& _xyz, const T& _w) : E{ _xyz.x, _xyz.y, _xyz.z, _w } {}
	VectorTN(const VectorTN<T,2>& _xy, const VectorTN<T,2>& _zw) : E{ _xy.x, _xy.y, _zw.x, _zw.y } {}
	VectorTN(const T& _x, const VectorTN<T,3>& _yzw) : E{ _x, _yzw.x, _yzw.y, _yzw.z } {}
	VectorTN(const T& _x, const VectorTN<T,2>& _yz, const T& _w) : E{ _x, _yz.x, _yz.y, _w } {}
	VectorTN(const T& _x, const T& _y, const VectorTN<T,2>& _zw) : E{ _x, _y, _zw.x, _zw.y } {}
	struct
	{
		T x;
		T y;
		T z;
		T w;
	};
	struct
	{
		VectorTN<T,2> xy0;
		T z0;
		T w0;
	};
	struct
	{
		VectorTN<T,3> xyz1;
		T w1;
	};
	struct
	{
		VectorTN<T,2> xy;
		VectorTN<T,2> zw;
	};
	struct
	{
		T x2;
		VectorTN<T,3> yzw2;
	};
	struct
	{
		T x3;
		VectorTN<T,2> yz3;
		T w3;
	};
	struct
	{
		T x4;
		T y4;
		VectorTN<T,2> zw4;
	};
	struct
	{
		T r;
		T g;
		T b;
		T a;
	};
	struct
	{
		VectorTN<T,2> rg0;
		T b0;
		T a0;
	};
	struct
	{
		VectorTN<T,3> rgb1;
		T a1;
	};
	struct
	{
		VectorTN<T,2> rg;
		VectorTN<T,2> ba;
	};
	struct
	{
		T r2;
		VectorTN<T,3> gba2;
	};
	struct
	{
		T r3;
		VectorTN<T,2> gb3;
		T a3;
	};
	struct
	{
		T r4;
		T g4;
		VectorTN<T,2> ba4;
	};
	std::array<T, 4> E{};
};

template<typename T>
auto operator+(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t.x + u.x, t.y + u.y);
	return result;
}
template<typename T>
auto operator+(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<T, 2> result(t.x + u, t.y + u);
	return result;
}
template<typename T>
auto operator+(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t + u.x, t + u.y);
	return result;
}
template<typename T>
auto operator-(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t.x - u.x, t.y - u.y);
	return result;
}
template<typename T>
auto operator-(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<T, 2> result(t.x - u, t.y - u);
	return result;
}
template<typename T>
auto operator-(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t - u.x, t - u.y);
	return result;
}
template<typename T>
auto operator*(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t.x * u.x, t.y * u.y);
	return result;
}
template<typename T>
auto operator*(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<T, 2> result(t.x * u, t.y * u);
	return result;
}
template<typename T>
auto operator*(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t * u.x, t * u.y);
	return result;
}
template<typename T>
auto operator/(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t.x / u.x, t.y / u.y);
	return result;
}
template<typename T>
auto operator/(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<T, 2> result(t.x / u, t.y / u);
	return result;
}
template<typename T>
auto operator/(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<T, 2> result(t / u.x, t / u.y);
	return result;
}
template<typename T>
auto operator<(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t.x < u.x, t.y < u.y);
	return result;
}
template<typename T>
auto operator<(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<bool, 2> result(t.x < u, t.y < u);
	return result;
}
template<typename T>
auto operator<(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t < u.x, t < u.y);
	return result;
}
template<typename T>
auto operator<=(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t.x <= u.x, t.y <= u.y);
	return result;
}
template<typename T>
auto operator<=(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<bool, 2> result(t.x <= u, t.y <= u);
	return result;
}
template<typename T>
auto operator<=(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t <= u.x, t <= u.y);
	return result;
}
template<typename T>
auto operator>(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t.x > u.x, t.y > u.y);
	return result;
}
template<typename T>
auto operator>(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<bool, 2> result(t.x > u, t.y > u);
	return result;
}
template<typename T>
auto operator>(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t > u.x, t > u.y);
	return result;
}
template<typename T>
auto operator>=(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t.x >= u.x, t.y >= u.y);
	return result;
}
template<typename T>
auto operator>=(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<bool, 2> result(t.x >= u, t.y >= u);
	return result;
}
template<typename T>
auto operator>=(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t >= u.x, t >= u.y);
	return result;
}
template<typename T>
auto operator==(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t.x == u.x, t.y == u.y);
	return result;
}
template<typename T>
auto operator==(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<bool, 2> result(t.x == u, t.y == u);
	return result;
}
template<typename T>
auto operator==(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t == u.x, t == u.y);
	return result;
}
template<typename T>
auto operator!=(const VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t.x != u.x, t.y != u.y);
	return result;
}
template<typename T>
auto operator!=(const VectorTN<T, 2>& t, const T& u)
{
	VectorTN<bool, 2> result(t.x != u, t.y != u);
	return result;
}
template<typename T>
auto operator!=(const T& t, const VectorTN<T, 2>& u)
{
	VectorTN<bool, 2> result(t != u.x, t != u.y);
	return result;
}
template<typename T>
auto operator+=(VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	t = t + u;
}
template<typename T>
auto operator+=(VectorTN<T, 2>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator-=(VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	t = t + u;
}
template<typename T>
auto operator-=(VectorTN<T, 2>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator*=(VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	t = t + u;
}
template<typename T>
auto operator*=(VectorTN<T, 2>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator/=(VectorTN<T, 2>& t, const VectorTN<T, 2>& u)
{
	t = t + u;
}
template<typename T>
auto operator/=(VectorTN<T, 2>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator-(const VectorTN<T, 2>& t)
{
	VectorTN<T, 2> result(-t.x, -t.y);
	return result;
}
template<typename T>
auto operator+(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t.x + u.x, t.y + u.y, t.z + u.z);
	return result;
}
template<typename T>
auto operator+(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<T, 3> result(t.x + u, t.y + u, t.z + u);
	return result;
}
template<typename T>
auto operator+(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t + u.x, t + u.y, t + u.z);
	return result;
}
template<typename T>
auto operator-(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t.x - u.x, t.y - u.y, t.z - u.z);
	return result;
}
template<typename T>
auto operator-(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<T, 3> result(t.x - u, t.y - u, t.z - u);
	return result;
}
template<typename T>
auto operator-(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t - u.x, t - u.y, t - u.z);
	return result;
}
template<typename T>
auto operator*(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t.x * u.x, t.y * u.y, t.z * u.z);
	return result;
}
template<typename T>
auto operator*(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<T, 3> result(t.x * u, t.y * u, t.z * u);
	return result;
}
template<typename T>
auto operator*(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t * u.x, t * u.y, t * u.z);
	return result;
}
template<typename T>
auto operator/(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t.x / u.x, t.y / u.y, t.z / u.z);
	return result;
}
template<typename T>
auto operator/(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<T, 3> result(t.x / u, t.y / u, t.z / u);
	return result;
}
template<typename T>
auto operator/(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<T, 3> result(t / u.x, t / u.y, t / u.z);
	return result;
}
template<typename T>
auto operator<(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t.x < u.x, t.y < u.y, t.z < u.z);
	return result;
}
template<typename T>
auto operator<(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<bool, 3> result(t.x < u, t.y < u, t.z < u);
	return result;
}
template<typename T>
auto operator<(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t < u.x, t < u.y, t < u.z);
	return result;
}
template<typename T>
auto operator<=(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t.x <= u.x, t.y <= u.y, t.z <= u.z);
	return result;
}
template<typename T>
auto operator<=(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<bool, 3> result(t.x <= u, t.y <= u, t.z <= u);
	return result;
}
template<typename T>
auto operator<=(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t <= u.x, t <= u.y, t <= u.z);
	return result;
}
template<typename T>
auto operator>(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t.x > u.x, t.y > u.y, t.z > u.z);
	return result;
}
template<typename T>
auto operator>(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<bool, 3> result(t.x > u, t.y > u, t.z > u);
	return result;
}
template<typename T>
auto operator>(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t > u.x, t > u.y, t > u.z);
	return result;
}
template<typename T>
auto operator>=(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t.x >= u.x, t.y >= u.y, t.z >= u.z);
	return result;
}
template<typename T>
auto operator>=(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<bool, 3> result(t.x >= u, t.y >= u, t.z >= u);
	return result;
}
template<typename T>
auto operator>=(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t >= u.x, t >= u.y, t >= u.z);
	return result;
}
template<typename T>
auto operator==(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t.x == u.x, t.y == u.y, t.z == u.z);
	return result;
}
template<typename T>
auto operator==(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<bool, 3> result(t.x == u, t.y == u, t.z == u);
	return result;
}
template<typename T>
auto operator==(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t == u.x, t == u.y, t == u.z);
	return result;
}
template<typename T>
auto operator!=(const VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t.x != u.x, t.y != u.y, t.z != u.z);
	return result;
}
template<typename T>
auto operator!=(const VectorTN<T, 3>& t, const T& u)
{
	VectorTN<bool, 3> result(t.x != u, t.y != u, t.z != u);
	return result;
}
template<typename T>
auto operator!=(const T& t, const VectorTN<T, 3>& u)
{
	VectorTN<bool, 3> result(t != u.x, t != u.y, t != u.z);
	return result;
}
template<typename T>
auto operator+=(VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	t = t + u;
}
template<typename T>
auto operator+=(VectorTN<T, 3>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator-=(VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	t = t + u;
}
template<typename T>
auto operator-=(VectorTN<T, 3>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator*=(VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	t = t + u;
}
template<typename T>
auto operator*=(VectorTN<T, 3>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator/=(VectorTN<T, 3>& t, const VectorTN<T, 3>& u)
{
	t = t + u;
}
template<typename T>
auto operator/=(VectorTN<T, 3>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator-(const VectorTN<T, 3>& t)
{
	VectorTN<T, 3> result(-t.x, -t.y, -t.z);
	return result;
}
template<typename T>
auto operator+(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t.x + u.x, t.y + u.y, t.z + u.z, t.w + u.w);
	return result;
}
template<typename T>
auto operator+(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<T, 4> result(t.x + u, t.y + u, t.z + u, t.w + u);
	return result;
}
template<typename T>
auto operator+(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t + u.x, t + u.y, t + u.z, t + u.w);
	return result;
}
template<typename T>
auto operator-(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t.x - u.x, t.y - u.y, t.z - u.z, t.w - u.w);
	return result;
}
template<typename T>
auto operator-(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<T, 4> result(t.x - u, t.y - u, t.z - u, t.w - u);
	return result;
}
template<typename T>
auto operator-(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t - u.x, t - u.y, t - u.z, t - u.w);
	return result;
}
template<typename T>
auto operator*(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t.x * u.x, t.y * u.y, t.z * u.z, t.w * u.w);
	return result;
}
template<typename T>
auto operator*(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<T, 4> result(t.x * u, t.y * u, t.z * u, t.w * u);
	return result;
}
template<typename T>
auto operator*(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t * u.x, t * u.y, t * u.z, t * u.w);
	return result;
}
template<typename T>
auto operator/(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t.x / u.x, t.y / u.y, t.z / u.z, t.w / u.w);
	return result;
}
template<typename T>
auto operator/(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<T, 4> result(t.x / u, t.y / u, t.z / u, t.w / u);
	return result;
}
template<typename T>
auto operator/(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<T, 4> result(t / u.x, t / u.y, t / u.z, t / u.w);
	return result;
}
template<typename T>
auto operator<(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t.x < u.x, t.y < u.y, t.z < u.z, t.w < u.w);
	return result;
}
template<typename T>
auto operator<(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<bool, 4> result(t.x < u, t.y < u, t.z < u, t.w < u);
	return result;
}
template<typename T>
auto operator<(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t < u.x, t < u.y, t < u.z, t < u.w);
	return result;
}
template<typename T>
auto operator<=(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t.x <= u.x, t.y <= u.y, t.z <= u.z, t.w <= u.w);
	return result;
}
template<typename T>
auto operator<=(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<bool, 4> result(t.x <= u, t.y <= u, t.z <= u, t.w <= u);
	return result;
}
template<typename T>
auto operator<=(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t <= u.x, t <= u.y, t <= u.z, t <= u.w);
	return result;
}
template<typename T>
auto operator>(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t.x > u.x, t.y > u.y, t.z > u.z, t.w > u.w);
	return result;
}
template<typename T>
auto operator>(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<bool, 4> result(t.x > u, t.y > u, t.z > u, t.w > u);
	return result;
}
template<typename T>
auto operator>(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t > u.x, t > u.y, t > u.z, t > u.w);
	return result;
}
template<typename T>
auto operator>=(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t.x >= u.x, t.y >= u.y, t.z >= u.z, t.w >= u.w);
	return result;
}
template<typename T>
auto operator>=(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<bool, 4> result(t.x >= u, t.y >= u, t.z >= u, t.w >= u);
	return result;
}
template<typename T>
auto operator>=(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t >= u.x, t >= u.y, t >= u.z, t >= u.w);
	return result;
}
template<typename T>
auto operator==(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t.x == u.x, t.y == u.y, t.z == u.z, t.w == u.w);
	return result;
}
template<typename T>
auto operator==(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<bool, 4> result(t.x == u, t.y == u, t.z == u, t.w == u);
	return result;
}
template<typename T>
auto operator==(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t == u.x, t == u.y, t == u.z, t == u.w);
	return result;
}
template<typename T>
auto operator!=(const VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t.x != u.x, t.y != u.y, t.z != u.z, t.w != u.w);
	return result;
}
template<typename T>
auto operator!=(const VectorTN<T, 4>& t, const T& u)
{
	VectorTN<bool, 4> result(t.x != u, t.y != u, t.z != u, t.w != u);
	return result;
}
template<typename T>
auto operator!=(const T& t, const VectorTN<T, 4>& u)
{
	VectorTN<bool, 4> result(t != u.x, t != u.y, t != u.z, t != u.w);
	return result;
}
template<typename T>
auto operator+=(VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	t = t + u;
}
template<typename T>
auto operator+=(VectorTN<T, 4>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator-=(VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	t = t + u;
}
template<typename T>
auto operator-=(VectorTN<T, 4>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator*=(VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	t = t + u;
}
template<typename T>
auto operator*=(VectorTN<T, 4>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator/=(VectorTN<T, 4>& t, const VectorTN<T, 4>& u)
{
	t = t + u;
}
template<typename T>
auto operator/=(VectorTN<T, 4>& t, const T& u)
{
	t = t + u;
}
template<typename T>
auto operator-(const VectorTN<T, 4>& t)
{
	VectorTN<T, 4> result(-t.x, -t.y, -t.z, -t.w);
	return result;
}

using Vector2 = VectorTN<float,2>;
using Double2 = VectorTN<double,2>;
using Int2 = VectorTN<int32_t,2>;
using Long2 = VectorTN<int64_t,2>;
using Bool2 = VectorTN<bool,2>;

using Vector3 = VectorTN<float,3>;
using Double3 = VectorTN<double,3>;
using Int3 = VectorTN<int32_t,3>;
using Long3 = VectorTN<int64_t,3>;
using Bool3 = VectorTN<bool,3>;

using Vector4 = VectorTN<float,4>;
using Double4 = VectorTN<double,4>;
using Int4 = VectorTN<int32_t,4>;
using Long4 = VectorTN<int64_t,4>;
using Bool4 = VectorTN<bool,4>;

}; // namespace luc

#endif /* LUCRAY_MATH_GEN_H */
