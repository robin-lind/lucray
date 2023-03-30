// https://github.com/madmann91/bvh
// Copyright 2022 Arsène Pérard-Gayot

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef BVH_H
#define BVH_H

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <istream>
#include <iterator>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <ostream>
#include <queue>
#include <span>
#include <stack>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#if defined(__clang__)
#define BVH_CLANG_ENABLE_FP_CONTRACT                            \
    _Pragma("clang diagnostic push")                            \
      _Pragma("clang diagnostic ignored \"-Wunknown-pragmas\"") \
        _Pragma("STDC FP_CONTRACT ON")                          \
          _Pragma("clang diagnostic pop")
#else
#define BVH_CLANG_ENABLE_FP_CONTRACT
#endif

#if defined(__GNUC__) || defined(__clang__)
#define BVH_ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined(_MSC_VER)
#define BVH_ALWAYS_INLINE __forceinline
#else
#define BVH_ALWAYS_INLINE inline
#endif

namespace bvh {

/// Helper type that gives an unsigned integer type with the given number of bits.
template<size_t Bits>
struct UnsignedInt {
};

template<>
struct UnsignedInt<8> {
    using Type = uint8_t;
};

template<>
struct UnsignedInt<16> {
    using Type = uint16_t;
};

template<>
struct UnsignedInt<32> {
    using Type = uint32_t;
};

template<>
struct UnsignedInt<64> {
    using Type = uint64_t;
};

template<size_t Bits>
using UnsignedIntType = typename UnsignedInt<Bits>::Type;

/// Helper callable object that just ignores its arguments and returns nothing.
struct IgnoreArgs {
    template<typename... Args>
    void operator()(Args&&...) const
    {
    }
};

/// Generates a bitmask with the given number of bits.
template<typename T, std::enable_if_t<std::is_unsigned_v<T>, bool> = true>
BVH_ALWAYS_INLINE constexpr T make_bitmask(size_t bits)
{
    return bits >= std::numeric_limits<T>::digits ? static_cast<T>(-1) : (static_cast<T>(1) << bits) - 1;
}

// These two functions are designed to return the second argument if the first one is a NaN.
template<typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
BVH_ALWAYS_INLINE T robust_min(T a, T b)
{
    return a < b ? a : b;
}

template<typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
BVH_ALWAYS_INLINE T robust_max(T a, T b)
{
    return a > b ? a : b;
}

/// Adds the given number of ULPs (Units in the Last Place) to the given floating-point number.
template<typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
BVH_ALWAYS_INLINE T add_ulp_magnitude(T t, unsigned ulp)
{
    UnsignedIntType<sizeof(T) * CHAR_BIT> u;
    std::memcpy(&u, &t, sizeof(T));
    u += ulp;
    std::memcpy(&t, &u, sizeof(T));
    return t;
}

/// Computes the inverse of the given value, always returning a finite value.
template<typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
BVH_ALWAYS_INLINE T safe_inverse(T x)
{
    return std::fabs(x) <= std::numeric_limits<T>::epsilon() ? std::copysign(std::numeric_limits<T>::max(), x) : static_cast<T>(1.) / x;
}

/// Fast multiply-add operation. Should translate into an FMA for architectures that support it.
#if defined(_MSC_VER) && !defined(__clang__)
#pragma float_control(push)
#pragma float_control(precise, off)
#pragma fp_contract(on)
#endif
template<typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
BVH_ALWAYS_INLINE T fast_mul_add(T a, T b, T c)
{
#ifdef FP_FAST_FMAF
    return std::fma(a, b, c);
#elif defined(__clang__)
    BVH_CLANG_ENABLE_FP_CONTRACT
#endif
    return a * b + c;
}
#if defined(_MSC_VER) && !defined(__clang__)
#pragma float_control(pop)
#endif

/// Executes the given function once for every integer in the range `[Begin, End)`.
template<size_t Begin, size_t End, typename F>
BVH_ALWAYS_INLINE void static_for(F&& f)
{
    if constexpr (Begin < End) {
        f(Begin);
        static_for<Begin + 1, End>(std::forward<F>(f));
    }
}

/// Computes the (rounded-up) compile-time log in base-2 of an unsigned integer.
template<typename T, std::enable_if_t<std::is_unsigned_v<T>, bool> = true>
inline constexpr T round_up_log2(T i, T p = 0)
{
    return (static_cast<T>(1) << p) >= i ? p : round_up_log2(i, p + 1);
}

/// Split an unsigned integer such that its bits are spaced by 2 zeros.
/// For instance, split_bits(0b00110010) = 0b000000001001000000001000.
template<typename T, std::enable_if_t<std::is_unsigned_v<T>, bool> = true>
BVH_ALWAYS_INLINE T split_bits(T x)
{
    constexpr size_t bit_count = sizeof(T) * CHAR_BIT;
    constexpr size_t log_bits = round_up_log2(bit_count);
    auto mask = static_cast<T>(-1) >> (bit_count / 2);
    x &= mask;
    for (size_t i = log_bits - 1, n = size_t{ 1 } << i; i > 0; --i, n >>= 1) {
        mask = (mask | (mask << n)) & ~(mask << (n / 2));
        x = (x | (x << n)) & mask;
    }
    return x;
}

/// Morton-encode three unsigned integers into one.
template<typename T, std::enable_if_t<std::is_unsigned_v<T>, bool> = true>
BVH_ALWAYS_INLINE T morton_encode(T x, T y, T z)
{
    return split_bits(x) | (split_bits(y) << 1) | (split_bits(z) << 2);
}

/// Computes the maximum between an atomic variable and a value, and returns the value previously
/// held by the atomic variable.
template<typename T>
BVH_ALWAYS_INLINE T atomic_max(std::atomic<T>& atomic, const T& value)
{
    auto prev_value = atomic;
    while (prev_value < value && !atomic.compare_exchange_weak(prev_value, value))
        ;
    return prev_value;
}

template<typename T, size_t N>
struct Vec {
    std::array<T, N> values;

    Vec() = default;

    template<typename... Args>
    BVH_ALWAYS_INLINE Vec(T x, T y, Args&&...args) :
      values{ x, y, static_cast<T>(std::forward<Args>(args))... }
    {
    }

    BVH_ALWAYS_INLINE explicit Vec(T x)
    {
        std::fill(std::begin(values), std::end(values), x);
    }

    template<typename Compare>
    BVH_ALWAYS_INLINE size_t get_best_axis(Compare&& compare) const
    {
        size_t axis = 0;
        static_for<1, N>([&](size_t i) { 
            if (compare(values[i], values[axis]))
                axis = i; });
        return axis;
    }

    // Note: These functions are designed to be robust to NaNs
    BVH_ALWAYS_INLINE size_t get_largest_axis() const
    {
        return get_best_axis(std::greater<T>());
    }

    BVH_ALWAYS_INLINE size_t get_smallest_axis() const
    {
        return get_best_axis(std::less<T>());
    }

    BVH_ALWAYS_INLINE T& operator[](size_t i)
    {
        return values[i];
    }

    BVH_ALWAYS_INLINE T operator[](size_t i) const
    {
        return values[i];
    }

    template<typename F>
    BVH_ALWAYS_INLINE static Vec<T, N> generate(F&& f)
    {
        Vec<T, N> v;
        static_for<0, N>([&](size_t i) { v[i] = f(i); });
        return v;
    }
};

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator+(const Vec<T, N>& a, const Vec<T, N>& b)
{
    return Vec<T, N>::generate([&](size_t i) { return a[i] + b[i]; });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator-(const Vec<T, N>& a, const Vec<T, N>& b)
{
    return Vec<T, N>::generate([&](size_t i) { return a[i] - b[i]; });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator-(const Vec<T, N>& a)
{
    return Vec<T, N>::generate([&](size_t i) { return -a[i]; });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator*(const Vec<T, N>& a, const Vec<T, N>& b)
{
    return Vec<T, N>::generate([&](size_t i) { return a[i] * b[i]; });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator/(const Vec<T, N>& a, const Vec<T, N>& b)
{
    return Vec<T, N>::generate([&](size_t i) { return a[i] / b[i]; });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator*(const Vec<T, N>& a, T b)
{
    return Vec<T, N>::generate([&](size_t i) { return a[i] * b; });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator*(T a, const Vec<T, N>& b)
{
    return b * a;
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> operator/(T a, const Vec<T, N>& b)
{
    return Vec<T, N>::generate([&](size_t i) { return a / b[i]; });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> robust_min(const Vec<T, N>& a, const Vec<T, N>& b)
{
    return Vec<T, N>::generate([&](size_t i) { return robust_min(a[i], b[i]); });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> robust_max(const Vec<T, N>& a, const Vec<T, N>& b)
{
    return Vec<T, N>::generate([&](size_t i) { return robust_max(a[i], b[i]); });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE T dot(const Vec<T, N>& a, const Vec<T, N>& b)
{
    return std::transform_reduce(std::begin(a.values), std::end(a.values), std::begin(b.values), T(0));
}

template<typename T>
BVH_ALWAYS_INLINE Vec<T, 3> cross(const Vec<T, 3>& a, const Vec<T, 3>& b)
{
    return Vec<T, 3>(
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]);
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> fast_mul_add(const Vec<T, N>& a, const Vec<T, N>& b, const Vec<T, N>& c)
{
    return Vec<T, N>::generate([&](size_t i) { return fast_mul_add(a[i], b[i], c[i]); });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> safe_inverse(const Vec<T, N>& v)
{
    return Vec<T, N>::generate([&](size_t i) { return safe_inverse(v[i]); });
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE T length(const Vec<T, N>& v)
{
    return std::sqrt(dot(v, v));
}

template<typename T, size_t N>
BVH_ALWAYS_INLINE Vec<T, N> normalize(const Vec<T, N>& v)
{
    return v * (static_cast<T>(1.) / length(v));
}

template<typename T, size_t N>
struct BBox {
    Vec<T, N> min, max;

    BBox() = default;

    BVH_ALWAYS_INLINE BBox(const Vec<T, N>& min, const Vec<T, N>& max) :
      min(min), max(max) {}

    BVH_ALWAYS_INLINE explicit BBox(const Vec<T, N>& point) :
      BBox(point, point) {}

    BVH_ALWAYS_INLINE BBox& extend(const Vec<T, N>& point)
    {
        return extend(BBox(point));
    }

    BVH_ALWAYS_INLINE BBox& extend(const BBox& other)
    {
        min = robust_min(min, other.min);
        max = robust_max(max, other.max);
        return *this;
    }

    BVH_ALWAYS_INLINE Vec<T, N> get_diagonal() const
    {
        return max - min;
    }

    BVH_ALWAYS_INLINE Vec<T, N> get_center() const
    {
        return (max + min) * static_cast<T>(0.5);
    }

    BVH_ALWAYS_INLINE T get_half_area() const
    {
        auto d = get_diagonal();
        static_assert(N == 2 || N == 3);
        if constexpr (N == 3) return (d[0] + d[1]) * d[2] + d[0] * d[1];
        if constexpr (N == 2) return d[0] + d[1];
        return static_cast<T>(0.);
    }

    BVH_ALWAYS_INLINE static constexpr BBox make_empty()
    {
        return BBox(
          Vec<T, N>(+std::numeric_limits<T>::max()),
          Vec<T, N>(-std::numeric_limits<T>::max()));
    }
};

struct Octant {
    uint32_t value = 0;
    static constexpr size_t max_dim = sizeof(value) * CHAR_BIT;

    uint32_t operator[](size_t i) const
    {
        return (value >> i) & uint32_t{ 1 };
    }
};

template<typename T, size_t N>
struct Ray {
    Vec<T, N> org, dir;
    T tmin, tmax;

    Ray() = default;

    BVH_ALWAYS_INLINE Ray(
      const Vec<T, N>& org,
      const Vec<T, N>& dir,
      T tmin = 0,
      T tmax = std::numeric_limits<T>::max()) :
      org(org), dir(dir), tmin(tmin), tmax(tmax)
    {
    }

    BVH_ALWAYS_INLINE Vec<T, N> get_inv_dir() const
    {
        return Vec<T, N>::generate([&](size_t i) { return safe_inverse(dir[i]); });
    }

    BVH_ALWAYS_INLINE Octant get_octant() const
    {
        static_assert(N <= Octant::max_dim);
        Octant octant;
        static_for<0, N>([&](size_t i) { octant.value |= std::signbit(dir[i]) * (uint32_t{ 1 } << i); });
        return octant;
    }

    // Pads the inverse direction according to T. Ize's "Robust BVH ray traversal"
    BVH_ALWAYS_INLINE static Vec<T, N> pad_inv_dir(const Vec<T, N>& inv_dir)
    {
        return Vec<T, N>::generate([&](size_t i) { return add_ulp_magnitude(inv_dir[i], 2); });
    }
};

/// Stream of data that can be used to deserialize data structures.
class InputStream {
public:
    template<typename T>
    T read(T&& default_val = {})
    {
        T data;
        if (read_raw(&data, sizeof(T)) != sizeof(T))
            data = std::move(default_val);
        return data;
    }

protected:
    virtual size_t read_raw(void *, size_t) = 0;
};

/// Stream of data that can be used to serialize data structures.
class OutputStream {
public:
    template<typename T>
    bool write(const T& data)
    {
        return write_raw(&data, sizeof(T));
    }

protected:
    virtual bool write_raw(const void *, size_t) = 0;
};

/// Stream adapter for standard library input streams.
class StdInputStream : public InputStream {
public:
    StdInputStream(std::istream& stream) :
      stream_(stream)
    {
    }

    using InputStream::read;

protected:
    std::istream& stream_;

    size_t read_raw(void *data, size_t size) override
    {
        stream_.read(reinterpret_cast<char *>(data), static_cast<std::streamsize>(size));
        return static_cast<size_t>(stream_.gcount());
    }
};

/// Stream adapter for standard library output streams.
class StdOutputStream : public OutputStream {
public:
    StdOutputStream(std::ostream& stream) :
      stream_(stream)
    {
    }

    using OutputStream::write;

protected:
    std::ostream& stream_;

    bool write_raw(const void *data, size_t size) override
    {
        stream_.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
        return stream_.good();
    }
};

template<
  typename T,
  size_t Dim,
  size_t IndexBits = sizeof(T) * CHAR_BIT,
  size_t PrimCountBits = 4>
struct Node {
    using Scalar = T;
    static constexpr size_t dimension = Dim;
    static constexpr size_t prim_count_bits = PrimCountBits;
    static constexpr size_t index_bits = IndexBits;
    static constexpr size_t max_prim_count = make_bitmask<size_t>(prim_count_bits);

    std::array<T, Dim * 2> bounds;

    struct Index {
        using Type = UnsignedIntType<IndexBits>;
        Type first_id : std::numeric_limits<Type>::digits - prim_count_bits;
        Type prim_count : prim_count_bits;

        BVH_ALWAYS_INLINE bool operator==(const Index& other) const
        {
            return first_id == other.first_id && prim_count == other.prim_count;
        }

        bool operator!=(const Index&) const = default;
    } index;

    static_assert(sizeof(Index) == sizeof(typename Index::Type));

    Node() = default;

    bool operator==(const Node&) const = default;
    bool operator!=(const Node&) const = default;

    BVH_ALWAYS_INLINE bool is_leaf() const
    {
        return index.prim_count != 0;
    }

    static BVH_ALWAYS_INLINE bool is_left_sibling(size_t node_id)
    {
        return node_id % 2 == 1;
    }

    static BVH_ALWAYS_INLINE size_t get_sibling_id(size_t node_id)
    {
        return is_left_sibling(node_id) ? node_id + 1 : node_id - 1;
    }

    static BVH_ALWAYS_INLINE size_t get_left_sibling_id(size_t node_id)
    {
        return is_left_sibling(node_id) ? node_id : node_id - 1;
    }

    static BVH_ALWAYS_INLINE size_t get_right_sibling_id(size_t node_id)
    {
        return is_left_sibling(node_id) ? node_id + 1 : node_id;
    }

    BVH_ALWAYS_INLINE void make_leaf(size_t first_prim, size_t prim_count)
    {
        assert(prim_count != 0);
        assert(prim_count <= max_prim_count);
        index.prim_count = static_cast<typename Index::Type>(prim_count);
        index.first_id = static_cast<typename Index::Type>(first_prim);
    }

    BVH_ALWAYS_INLINE void make_inner(size_t first_child)
    {
        index.prim_count = 0;
        index.first_id = static_cast<typename Index::Type>(first_child);
    }

    BVH_ALWAYS_INLINE BBox<T, Dim> get_bbox() const
    {
        return BBox<T, Dim>(
          Vec<T, Dim>::generate([&](size_t i) { return bounds[i * 2]; }),
          Vec<T, Dim>::generate([&](size_t i) { return bounds[i * 2 + 1]; }));
    }

    BVH_ALWAYS_INLINE void set_bbox(const BBox<T, Dim>& bbox)
    {
        static_for<0, Dim>([&](size_t i) {
            bounds[i * 2 + 0] = bbox.min[i];
            bounds[i * 2 + 1] = bbox.max[i]; });
    }

    BVH_ALWAYS_INLINE Vec<T, Dim> get_min_bounds(const Octant& octant) const
    {
        return Vec<T, Dim>::generate([&](size_t i) { return bounds[2 * static_cast<uint32_t>(i) + octant[i]]; });
    }

    BVH_ALWAYS_INLINE Vec<T, Dim> get_max_bounds(const Octant& octant) const
    {
        return Vec<T, Dim>::generate([&](size_t i) { return bounds[2 * static_cast<uint32_t>(i) + 1 - octant[i]]; });
    }

    /// Robust ray-node intersection routine. See "Robust BVH Ray Traversal", by T. Ize.
    BVH_ALWAYS_INLINE std::pair<T, T> intersect_robust(
      const Ray<T, Dim>& ray,
      const Vec<T, Dim>& inv_dir,
      const Vec<T, Dim>& inv_dir_pad,
      const Octant& octant) const
    {
        auto tmin = (get_min_bounds(octant) - ray.org) * inv_dir;
        auto tmax = (get_max_bounds(octant) - ray.org) * inv_dir_pad;
        return make_intersection_result(ray, tmin, tmax);
    }

    BVH_ALWAYS_INLINE std::pair<T, T> intersect_fast(
      const Ray<T, Dim>& ray,
      const Vec<T, Dim>& inv_dir,
      const Vec<T, Dim>& inv_org,
      const Octant& octant) const
    {
        auto tmin = fast_mul_add(get_min_bounds(octant), inv_dir, inv_org);
        auto tmax = fast_mul_add(get_max_bounds(octant), inv_dir, inv_org);
        return make_intersection_result(ray, tmin, tmax);
    }

    BVH_ALWAYS_INLINE void serialize(OutputStream& stream) const
    {
        for (auto&& bound : bounds)
            stream.write(bound);
        stream.write(static_cast<size_t>(index.first_id));
        stream.write(static_cast<size_t>(index.prim_count));
    }

    static inline Node deserialize(InputStream& stream)
    {
        Node node;
        for (auto& bound : node.bounds)
            bound = stream.read<T>();
        node.index.first_id = stream.read<size_t>();
        node.index.prim_count = stream.read<size_t>();
        return node;
    }

private:
    BVH_ALWAYS_INLINE static std::pair<T, T> make_intersection_result(
      const Ray<T, Dim>& ray,
      const Vec<T, Dim>& tmin,
      const Vec<T, Dim>& tmax)
    {
        auto t0 = ray.tmin;
        auto t1 = ray.tmax;
        static_for<0, Dim>([&](size_t i) {
            t0 = robust_max(tmin[i], t0);
            t1 = robust_min(tmax[i], t1); });
        return std::pair<T, T>{ t0, t1 };
    }
};

template<typename Node>
struct Bvh {
    using Index = typename Node::Index;
    using Scalar = typename Node::Scalar;

    std::vector<Node> nodes;
    std::vector<size_t> prim_ids;

    Bvh() = default;
    Bvh(Bvh&&) = default;

    Bvh& operator=(Bvh&&) = default;

    bool operator==(const Bvh& other) const = default;
    bool operator!=(const Bvh& other) const = default;

    /// Returns the root node of this BVH.
    const Node& get_root() const
    {
        return nodes[0];
    }

    /// Extracts the BVH rooted at the given node index.
    inline Bvh extract_bvh(size_t root_id) const;

    /// Intersects the BVH with a single ray, using the given function to intersect the contents
    /// of a leaf. The algorithm starts at the node index `top` and uses the given stack object.
    /// When `IsAnyHit` is true, the function stops at the first intersection (useful for shadow
    /// rays), otherwise it finds the closest intersection. When `IsRobust` is true, a slower but
    /// numerically robust ray-box test is used, otherwise a fast, but less precise test is used.
    template<bool IsAnyHit, bool IsRobust, typename Stack, typename LeafFn, typename InnerFn = IgnoreArgs>
    inline void intersect(Ray<Scalar, Node::dimension>& ray, Index top, Stack&, LeafFn&&, InnerFn&& = {}) const;

    inline void serialize(OutputStream&) const;
    static inline Bvh deserialize(InputStream&);
};

template<typename Node>
auto Bvh<Node>::extract_bvh(size_t root_id) const -> Bvh
{
    assert(root_id != 0);

    Bvh bvh;
    bvh.nodes.emplace_back();

    std::stack<std::pair<size_t, size_t>> stack;
    stack.emplace(root_id, 0);
    while (!stack.empty()) {
        auto [src_id, dst_id] = stack.top();
        stack.pop();
        auto& src_node = nodes[src_id];
        auto& dst_node = bvh.nodes[dst_id];
        dst_node = src_node;
        if (src_node.is_leaf()) {
            dst_node.index.first_id = static_cast<typename Index::Type>(bvh.prim_ids.size());
            std::copy_n(
              prim_ids.begin() + src_node.index.first_id,
              src_node.index.prim_count,
              std::back_inserter(bvh.prim_ids));
        }
        else {
            size_t first_id = bvh.nodes.size();
            dst_node.index.first_id = static_cast<typename Index::Type>(first_id);
            bvh.nodes.emplace_back();
            bvh.nodes.emplace_back();
            stack.emplace(src_node.index.first_id + 0, first_id + 0);
            stack.emplace(src_node.index.first_id + 1, first_id + 1);
        }
    }
    return bvh;
}

template<typename Node>
template<bool IsAnyHit, bool IsRobust, typename Stack, typename LeafFn, typename InnerFn>
void Bvh<Node>::intersect(Ray<Scalar, Node::dimension>& ray, Index start, Stack& stack, LeafFn&& leaf_fn, InnerFn&& inner_fn) const
{
    auto inv_dir = ray.get_inv_dir();
    auto inv_org = -inv_dir * ray.org;
    auto inv_dir_pad = Ray<Scalar, Node::dimension>::pad_inv_dir(inv_dir);
    auto octant = ray.get_octant();

    auto intersect_node = [&](const Node& node) {
        return IsRobust ? node.intersect_robust(ray, inv_dir, inv_dir_pad, octant) : node.intersect_fast(ray, inv_dir, inv_org, octant);
    };

    stack.push(start);
restart:
    while (!stack.is_empty()) {
        auto top = stack.pop();
        while (top.prim_count == 0) {
            auto& left = nodes[top.first_id];
            auto& right = nodes[top.first_id + 1];

            inner_fn(left, right);

            auto intr_left = intersect_node(left);
            auto intr_right = intersect_node(right);

            bool hit_left = intr_left.first <= intr_left.second;
            bool hit_right = intr_right.first <= intr_right.second;

            if (hit_left) {
                auto near = left.index;
                if (hit_right) {
                    auto far = right.index;
                    if (!IsAnyHit && intr_left.first > intr_right.first)
                        std::swap(near, far);
                    stack.push(far);
                }
                top = near;
            }
            else if (hit_right)
                top = right.index;
            else [[unlikely]]
                goto restart;
        }

        [[maybe_unused]] auto was_hit = leaf_fn(top.first_id, top.first_id + top.prim_count);
        if constexpr (IsAnyHit) {
            if (was_hit) return;
        }
    }
}

template<typename Node>
void Bvh<Node>::serialize(OutputStream& stream) const
{
    stream.write(nodes.size());
    stream.write(prim_ids.size());
    for (auto&& node : nodes)
        node.serialize(stream);
    for (auto&& prim_id : prim_ids)
        stream.write(prim_id);
}

template<typename Node>
Bvh<Node> Bvh<Node>::deserialize(InputStream& stream)
{
    Bvh bvh;
    bvh.nodes.resize(stream.read<size_t>());
    bvh.prim_ids.resize(stream.read<size_t>());
    for (auto& node : bvh.nodes)
        node = Node::deserialize(stream);
    for (auto& prim_id : bvh.prim_ids)
        prim_id = stream.read<size_t>();
    return bvh;
}

template<typename T, size_t N>
struct Tri {
    Vec<T, N> p0, p1, p2;

    Tri() = default;

    BVH_ALWAYS_INLINE Tri(const Vec<T, N>& p0, const Vec<T, N>& p1, const Vec<T, N>& p2) :
      p0(p0), p1(p1), p2(p2)
    {
    }

    BVH_ALWAYS_INLINE BBox<T, 3> get_bbox() const
    {
        return BBox(p0).extend(p1).extend(p2);
    }

    BVH_ALWAYS_INLINE Vec<T, 3> get_center() const
    {
        return (p0 + p1 + p2) * static_cast<T>(1. / 3.);
    }
};

/// A 3d triangle, represented as two edges and a point, with an (unnormalized, left-handed) normal.
template<typename T>
struct PrecomputedTri {
    Vec<T, 3> p0, e1, e2, n;

    PrecomputedTri() = default;

    BVH_ALWAYS_INLINE PrecomputedTri(const Vec<T, 3>& p0, const Vec<T, 3>& p1, const Vec<T, 3>& p2) :
      p0(p0), e1(p0 - p1), e2(p2 - p0), n(cross(e1, e2))
    {
    }

    BVH_ALWAYS_INLINE PrecomputedTri(const Tri<T, 3>& triangle) :
      PrecomputedTri(triangle.p0, triangle.p1, triangle.p2)
    {
    }

    BVH_ALWAYS_INLINE Tri<T, 3> convert_to_tri() const
    {
        return Tri<T, 3>(p0, p0 - e1, e2 + p0);
    }

    BVH_ALWAYS_INLINE BBox<T, 3> get_bbox() const
    {
        return convert_to_tri().get_bbox();
    }

    BVH_ALWAYS_INLINE Vec<T, 3> get_center() const
    {
        return convert_to_tri().get_center();
    }

    /// Returns a pair containing the barycentric coordinates of the hit point if the given ray
    /// intersects the triangle, otherwise returns nothing. The distance at which the ray intersects
    /// the triangle is set in `ray.tmax`. The tolerance can be adjusted to account for numerical
    /// precision issues.
    BVH_ALWAYS_INLINE std::optional<std::pair<T, T>> intersect(
      Ray<T, 3>& ray,
      T tolerance = -std::numeric_limits<T>::epsilon()) const;
};

template<typename T>
std::optional<std::pair<T, T>> PrecomputedTri<T>::intersect(Ray<T, 3>& ray, T tolerance) const
{
    auto c = p0 - ray.org;
    auto r = cross(ray.dir, c);
    auto inv_det = static_cast<T>(1.) / dot(n, ray.dir);

    auto u = dot(r, e2) * inv_det;
    auto v = dot(r, e1) * inv_det;
    auto w = static_cast<T>(1.) - u - v;

    // These comparisons are designed to return false
    // when one of t, u, or v is a NaN
    if (u >= tolerance && v >= tolerance && w >= tolerance) {
        auto t = dot(n, c) * inv_det;
        if (t >= ray.tmin && t <= ray.tmax) {
            ray.tmax = t;
            return std::make_optional(std::pair<T, T>{ u, v });
        }
    }

    return std::nullopt;
}

/// Fixed-size stack that can be used for a BVH traversal.
template<typename T, unsigned Capacity>
struct SmallStack {
    static constexpr unsigned capacity = Capacity;

    std::array<T, capacity> elems;
    unsigned size = 0;

    bool is_empty() const
    {
        return size == 0;
    }

    bool is_full() const
    {
        return size >= capacity;
    }

    void push(const T& t)
    {
        assert(!is_full());
        elems[size++] = t;
    }

    T pop()
    {
        assert(!is_empty());
        return elems[--size];
    }
};

/// Growing stack that can be used for BVH traversal. Its performance may be lower than a small,
/// fixed-size stack, depending on the architecture.
template<typename T>
struct GrowingStack {
    std::vector<T> elems;

    bool is_empty() const
    {
        return elems.empty();
    }

    void push(const T& t)
    {
        elems.push_back(t);
    }

    T pop()
    {
        assert(!is_empty());
        auto top = std::move(elems.back());
        elems.pop_back();
        return top;
    }
};

class ThreadPool {
public:
    using Task = std::function<void(size_t)>;

    /// Creates a thread pool with the given number of threads (a value of 0 tries to autodetect
    /// the number of threads and uses that as a thread count).
    ThreadPool(size_t thread_count = 0)
    {
        start(thread_count);
    }

    ~ThreadPool()
    {
        wait();
        stop();
        join();
    }

    inline void push(Task&& fun);
    inline void wait();

    size_t get_thread_count() const
    {
        return threads_.size();
    }

private:
    static inline void worker(ThreadPool *, size_t);

    inline void start(size_t);
    inline void stop();
    inline void join();

    int busy_count_ = 0;
    bool should_stop_ = false;
    std::mutex mutex_;
    std::vector<std::thread> threads_;
    std::condition_variable avail_;
    std::condition_variable done_;
    std::queue<Task> tasks_;
};

void ThreadPool::push(Task&& task)
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        tasks_.emplace(std::move(task));
    }
    avail_.notify_one();
}

void ThreadPool::wait()
{
    std::unique_lock<std::mutex> lock(mutex_);
    done_.wait(lock, [this] { return busy_count_ == 0 && tasks_.empty(); });
}

void ThreadPool::worker(ThreadPool *pool, size_t thread_id)
{
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(pool->mutex_);
            pool->avail_.wait(lock, [pool] { return pool->should_stop_ || !pool->tasks_.empty(); });
            if (pool->should_stop_ && pool->tasks_.empty())
                break;
            task = std::move(pool->tasks_.front());
            pool->tasks_.pop();
            pool->busy_count_++;
        }
        task(thread_id);
        {
            std::unique_lock<std::mutex> lock(pool->mutex_);
            pool->busy_count_--;
        }
        pool->done_.notify_one();
    }
}

void ThreadPool::start(size_t thread_count)
{
    if (thread_count == 0)
        thread_count = std::max(1u, std::thread::hardware_concurrency());
    for (size_t i = 0; i < thread_count; ++i)
        threads_.emplace_back(worker, this, i);
}

void ThreadPool::stop()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        should_stop_ = true;
    }
    avail_.notify_all();
}

void ThreadPool::join()
{
    for (auto& thread : threads_)
        thread.join();
}

/// Helper object that provides iteration and reduction over one-dimensional ranges.
template<typename Derived>
struct Executor {
    template<typename Loop>
    inline void for_each(size_t begin, size_t end, const Loop& loop)
    {
        return static_cast<Derived *>(this)->for_each(begin, end, loop);
    }

    template<typename T, typename Reduce, typename Join>
    inline T reduce(size_t begin, size_t end, const T& init, const Reduce& reduce, const Join& join)
    {
        return static_cast<Derived *>(this)->reduce(begin, end, init, reduce, join);
    }
};

/// Executor that executes serially.
struct SequentialExecutor : Executor<SequentialExecutor> {
    template<typename Loop>
    void for_each(size_t begin, size_t end, const Loop& loop)
    {
        loop(begin, end);
    }

    template<typename T, typename Reduce, typename Join>
    T reduce(size_t begin, size_t end, const T& init, const Reduce& reduce, const Join&)
    {
        T result(init);
        reduce(result, begin, end);
        return result;
    }
};

/// Executor that executes in parallel using the given thread pool.
struct ParallelExecutor : Executor<ParallelExecutor> {
    ThreadPool& thread_pool;
    size_t parallel_threshold;

    ParallelExecutor(ThreadPool& thread_pool, size_t parallel_threshold = 1024) :
      thread_pool(thread_pool), parallel_threshold(parallel_threshold)
    {
    }

    template<typename Loop>
    void for_each(size_t begin, size_t end, const Loop& loop)
    {
        if (end - begin < parallel_threshold)
            return loop(begin, end);

        auto chunk_size = std::max(size_t{ 1 }, (end - begin) / thread_pool.get_thread_count());
        for (size_t i = begin; i < end; i += chunk_size) {
            size_t next = std::min(end, i + chunk_size);
            thread_pool.push([=](size_t) { loop(i, next); });
        }
        thread_pool.wait();
    }

    template<typename T, typename Reduce, typename Join>
    T reduce(size_t begin, size_t end, const T& init, const Reduce& reduce, const Join& join)
    {
        if (end - begin < parallel_threshold) {
            T result(init);
            reduce(result, begin, end);
            return result;
        }

        auto chunk_size = std::max(size_t{ 1 }, (end - begin) / thread_pool.get_thread_count());
        std::vector<T> per_thread_result(thread_pool.get_thread_count(), init);
        for (size_t i = begin; i < end; i += chunk_size) {
            size_t next = std::min(end, i + chunk_size);
            thread_pool.push([&, i, next](size_t thread_id) {
                auto& result = per_thread_result[thread_id];
                reduce(result, i, next); });
        }
        thread_pool.wait();
        for (size_t i = 1; i < thread_pool.get_thread_count(); ++i)
            join(per_thread_result[0], std::move(per_thread_result[i]));
        return per_thread_result[0];
    }
};

template<typename T>
class SplitHeuristic {
public:
    /// Creates an SAH evaluator object, used by top-down builders to determine where to split.
    /// The two parameters are the log of the size of primitive clusters in base 2, and the ratio of
    /// the cost of intersecting a node (a ray-box intersection) over the cost of intersecting a
    /// primitive.
    BVH_ALWAYS_INLINE SplitHeuristic(
      size_t log_cluster_size = 0,
      T cost_ratio = static_cast<T>(1.)) :
      log_cluster_size_(log_cluster_size), prim_offset_(make_bitmask<size_t>(log_cluster_size)), cost_ratio_(cost_ratio)
    {
    }

    BVH_ALWAYS_INLINE size_t get_prim_count(size_t size) const
    {
        return (size + prim_offset_) >> log_cluster_size_;
    }

    template<size_t N>
    BVH_ALWAYS_INLINE T get_leaf_cost(size_t begin, size_t end, const BBox<T, N>& bbox) const
    {
        return bbox.get_half_area() * static_cast<T>(get_prim_count(end - begin));
    }

    template<size_t N>
    BVH_ALWAYS_INLINE T get_non_split_cost(size_t begin, size_t end, const BBox<T, N>& bbox) const
    {
        return bbox.get_half_area() * (static_cast<T>(get_prim_count(end - begin)) - cost_ratio_);
    }

private:
    size_t log_cluster_size_;
    size_t prim_offset_;
    T cost_ratio_;
};

/// Base class for all SAH-based, top-down builders.
template<typename Node>
class TopDownSahBuilder {
protected:
    using Scalar = typename Node::Scalar;
    using Vec = bvh::Vec<Scalar, Node::dimension>;
    using BBox = bvh::BBox<Scalar, Node::dimension>;

public:
    struct Config {
        /// SAH heuristic parameters that control how primitives are partitioned.
        SplitHeuristic<Scalar> sah;

        /// Nodes containing less than this amount of primitives will not be split.
        /// This is mostly to speed up BVH construction, and using large values may lead to lower
        /// quality BVHs.
        size_t min_leaf_size = 1;

        /// Nodes that cannot be split based on the SAH and have a number of primitives larger than
        /// this will be split using a fallback strategy. This should not happen often, but may
        /// happen in worst-case scenarios or poorly designed scenes.
        size_t max_leaf_size = 8;
    };

protected:
    struct WorkItem {
        size_t node_id;
        size_t begin;
        size_t end;

        BVH_ALWAYS_INLINE size_t size() const
        {
            return end - begin;
        }
    };

    std::span<const BBox> bboxes_;
    std::span<const Vec> centers_;
    const Config& config_;

    BVH_ALWAYS_INLINE TopDownSahBuilder(
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config) :
      bboxes_(bboxes), centers_(centers), config_(config)
    {
        assert(bboxes.size() == centers.size());
    }

    virtual std::vector<size_t>& get_prim_ids() = 0;
    virtual std::optional<size_t> try_split(const BBox& bbox, size_t begin, size_t end) = 0;

    BVH_ALWAYS_INLINE const std::vector<size_t>& get_prim_ids() const
    {
        return const_cast<TopDownSahBuilder *>(this)->get_prim_ids();
    }

    Bvh<Node> build()
    {
        const auto prim_count = bboxes_.size();

        Bvh<Node> bvh;
        bvh.nodes.reserve((2 * prim_count) / config_.min_leaf_size);
        bvh.nodes.emplace_back();
        bvh.nodes.back().set_bbox(compute_bbox(0, prim_count));

        std::stack<WorkItem> stack;
        stack.push(WorkItem{ 0, 0, prim_count });
        while (!stack.empty()) {
            auto item = stack.top();
            stack.pop();

            auto& node = bvh.nodes[item.node_id];
            if (item.size() > config_.min_leaf_size) {
                if (auto split_pos = try_split(node.get_bbox(), item.begin, item.end)) {
                    auto first_child = bvh.nodes.size();
                    node.make_inner(first_child);

                    bvh.nodes.resize(first_child + 2);

                    auto first_bbox = compute_bbox(item.begin, *split_pos);
                    auto second_bbox = compute_bbox(*split_pos, item.end);
                    auto first_range = std::make_pair(item.begin, *split_pos);
                    auto second_range = std::make_pair(*split_pos, item.end);

                    // For "any-hit" queries, the left child is chosen first, so we make sure that
                    // it is the child with the largest area, as it is more likely to contain an
                    // an occluder. See "SATO: Surface Area Traversal Order for Shadow Ray Tracing",
                    // by J. Nah and D. Manocha.
                    if (first_bbox.get_half_area() < second_bbox.get_half_area()) {
                        std::swap(first_bbox, second_bbox);
                        std::swap(first_range, second_range);
                    }

                    auto first_item = WorkItem{ first_child + 0, first_range.first, first_range.second };
                    auto second_item = WorkItem{ first_child + 1, second_range.first, second_range.second };
                    bvh.nodes[first_child + 0].set_bbox(first_bbox);
                    bvh.nodes[first_child + 1].set_bbox(second_bbox);

                    // Process the largest child item first, in order to minimize the stack size.
                    if (first_item.size() < second_item.size())
                        std::swap(first_item, second_item);

                    stack.push(first_item);
                    stack.push(second_item);
                    continue;
                }
            }

            node.make_leaf(item.begin, item.size());
        }

        bvh.prim_ids = std::move(get_prim_ids());
        bvh.nodes.shrink_to_fit();
        return bvh;
    }

    BVH_ALWAYS_INLINE BBox compute_bbox(size_t begin, size_t end) const
    {
        const auto& prim_ids = get_prim_ids();
        auto bbox = BBox::make_empty();
        for (size_t i = begin; i < end; ++i)
            bbox.extend(bboxes_[prim_ids[i]]);
        return bbox;
    }
};

/// Single-threaded top-down builder that partitions primitives based on the Surface
/// Area Heuristic (SAH). Primitives are only sorted once along each axis.
template<typename Node>
class SweepSahBuilder : public TopDownSahBuilder<Node> {
    using typename TopDownSahBuilder<Node>::Scalar;
    using typename TopDownSahBuilder<Node>::Vec;
    using typename TopDownSahBuilder<Node>::BBox;

    using TopDownSahBuilder<Node>::build;
    using TopDownSahBuilder<Node>::config_;
    using TopDownSahBuilder<Node>::bboxes_;

public:
    using typename TopDownSahBuilder<Node>::Config;

    BVH_ALWAYS_INLINE static Bvh<Node> build(
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config = {})
    {
        SweepSahBuilder builder(bboxes, centers, config);
        return builder.build();
    }

protected:
    struct Split {
        size_t pos;
        Scalar cost;
        size_t axis;
    };

    std::vector<bool> marks_;
    std::vector<Scalar> accum_;
    std::vector<size_t> prim_ids_[Node::dimension];

    BVH_ALWAYS_INLINE SweepSahBuilder(
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config) :
      TopDownSahBuilder<Node>(bboxes, centers, config)
    {
        marks_.resize(bboxes.size());
        accum_.resize(bboxes.size());
        for (size_t axis = 0; axis < Node::dimension; ++axis) {
            prim_ids_[axis].resize(bboxes.size());
            std::iota(prim_ids_[axis].begin(), prim_ids_[axis].end(), 0);
            std::sort(prim_ids_[axis].begin(), prim_ids_[axis].end(), [&](size_t i, size_t j) { return centers[i][axis] < centers[j][axis]; });
        }
    }

    std::vector<size_t>& get_prim_ids() override
    {
        return prim_ids_[0];
    }

    void find_best_split(size_t axis, size_t begin, size_t end, Split& best_split)
    {
        size_t first_right = begin;

        // Sweep from the right to the left, computing the partial SAH cost
        auto right_bbox = BBox::make_empty();
        for (size_t i = end - 1; i > begin;) {
            static constexpr size_t chunk_size = 32;
            size_t next = i - std::min(i - begin, chunk_size);
            auto right_cost = static_cast<Scalar>(0.);
            for (; i > next; --i) {
                right_bbox.extend(bboxes_[prim_ids_[axis][i]]);
                accum_[i] = right_cost = config_.sah.get_leaf_cost(i, end, right_bbox);
            }
            // Every `chunk_size` elements, check that we are not above the maximum cost
            if (right_cost > best_split.cost) {
                first_right = i;
                break;
            }
        }

        // Sweep from the left to the right, computing the full cost
        auto left_bbox = BBox::make_empty();
        for (size_t i = begin; i < first_right; ++i)
            left_bbox.extend(bboxes_[prim_ids_[axis][i]]);
        for (size_t i = first_right; i < end - 1; ++i) {
            left_bbox.extend(bboxes_[prim_ids_[axis][i]]);
            auto left_cost = config_.sah.get_leaf_cost(begin, i + 1, left_bbox);
            auto cost = left_cost + accum_[i + 1];
            if (cost < best_split.cost)
                best_split = Split{ i + 1, cost, axis };
            else if (left_cost > best_split.cost)
                break;
        }
    }

    BVH_ALWAYS_INLINE void mark_primitives(size_t axis, size_t begin, size_t split_pos, size_t end)
    {
        for (size_t i = begin; i < split_pos; ++i) marks_[prim_ids_[axis][i]] = true;
        for (size_t i = split_pos; i < end; ++i) marks_[prim_ids_[axis][i]] = false;
    }

    std::optional<size_t> try_split(const BBox& bbox, size_t begin, size_t end) override
    {
        // Find the best split over all axes
        auto leaf_cost = config_.sah.get_non_split_cost(begin, end, bbox);
        auto best_split = Split{ (begin + end + 1) / 2, leaf_cost, 0 };
        for (size_t axis = 0; axis < Node::dimension; ++axis)
            find_best_split(axis, begin, end, best_split);

        // Make sure that the split is good before proceeding with it
        if (best_split.cost >= leaf_cost) {
            if (end - begin <= config_.max_leaf_size)
                return std::nullopt;

            // If the number of primitives is too high, fallback on a split at the
            // median on the largest axis.
            best_split.pos = (begin + end + 1) / 2;
            best_split.axis = bbox.get_diagonal().get_largest_axis();
        }

        // Partition primitives (keeping the order intact so that the next recursive calls do not
        // need to sort primitives again).
        mark_primitives(best_split.axis, begin, best_split.pos, end);
        for (size_t axis = 0; axis < Node::dimension; ++axis) {
            if (axis == best_split.axis)
                continue;
            std::stable_partition(
              prim_ids_[axis].begin() + begin,
              prim_ids_[axis].begin() + end,
              [&](size_t i) { return marks_[i]; });
        }

        return std::make_optional(best_split.pos);
    }
};

/// Single-threaded top-down builder that partitions primitives based on a binned approximation of
/// the Surface Area Heuristic (SAH). This builder is inspired by
/// "On Fast Construction of SAH-based Bounding Volume Hierarchies", by I. Wald.
template<typename Node, size_t BinCount = 8>
class BinnedSahBuilder : public TopDownSahBuilder<Node> {
    using typename TopDownSahBuilder<Node>::Scalar;
    using typename TopDownSahBuilder<Node>::Vec;
    using typename TopDownSahBuilder<Node>::BBox;

    using TopDownSahBuilder<Node>::build;
    using TopDownSahBuilder<Node>::config_;
    using TopDownSahBuilder<Node>::bboxes_;
    using TopDownSahBuilder<Node>::centers_;

public:
    using typename TopDownSahBuilder<Node>::Config;

    BVH_ALWAYS_INLINE static Bvh<Node> build(
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config = {})
    {
        BinnedSahBuilder builder(bboxes, centers, config);
        return builder.build();
    }

protected:
    struct Split {
        size_t bin_id;
        Scalar cost;
        size_t axis;
    };

    struct Bin {
        BBox bbox = BBox::make_empty();
        size_t prim_count = 0;

        Bin() = default;

        BVH_ALWAYS_INLINE Scalar get_cost(const SplitHeuristic<Scalar>& sah) const
        {
            return sah.get_leaf_cost(0, prim_count, bbox);
        }

        BVH_ALWAYS_INLINE void add(const BBox& bbox, size_t prim_count = 1)
        {
            this->bbox.extend(bbox);
            this->prim_count += prim_count;
        }

        BVH_ALWAYS_INLINE void add(const Bin& bin)
        {
            add(bin.bbox, bin.prim_count);
        }
    };

    using Bins = std::array<Bin, BinCount>;
    using PerAxisBins = std::array<Bins, Node::dimension>;

    std::vector<size_t> prim_ids_;

    BVH_ALWAYS_INLINE BinnedSahBuilder(
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config) :
      TopDownSahBuilder<Node>(bboxes, centers, config), prim_ids_(bboxes.size())
    {
        std::iota(prim_ids_.begin(), prim_ids_.end(), 0);
    }

    std::vector<size_t>& get_prim_ids() override
    {
        return prim_ids_;
    }

    BVH_ALWAYS_INLINE void fill_bins(
      PerAxisBins& per_axis_bins,
      const BBox& bbox,
      size_t begin,
      size_t end)
    {
        auto bin_scale = Vec(BinCount) / bbox.get_diagonal();
        auto bin_offset = -bbox.min * bin_scale;

        for (size_t i = begin; i < end; ++i) {
            auto pos = fast_mul_add(centers_[prim_ids_[i]], bin_scale, bin_offset);
            static_for<0, Node::dimension>([&](size_t axis) {
                size_t index = std::min(BinCount - 1,
                    static_cast<size_t>(robust_max(pos[axis], static_cast<Scalar>(0.))));
                per_axis_bins[axis][index].add(bboxes_[prim_ids_[i]]); });
        }
    }

    void find_best_split(size_t axis, const Bins& bins, Split& best_split)
    {
        Bin right_accum;
        std::array<Scalar, BinCount> right_costs;
        for (size_t i = BinCount - 1; i > 0; --i) {
            right_accum.add(bins[i]);
            right_costs[i] = right_accum.get_cost(config_.sah);
        }

        Bin left_accum;
        for (size_t i = 0; i < BinCount - 1; ++i) {
            left_accum.add(bins[i]);
            auto cost = left_accum.get_cost(config_.sah) + right_costs[i + 1];
            if (cost < best_split.cost)
                best_split = Split{ i + 1, cost, axis };
        }
    }

    size_t fallback_split(size_t axis, size_t begin, size_t end)
    {
        size_t mid = (begin + end + 1) / 2;
        std::partial_sort(
          prim_ids_.begin() + begin,
          prim_ids_.begin() + mid,
          prim_ids_.begin() + end,
          [&](size_t i, size_t j) { return centers_[i][axis] < centers_[j][axis]; });
        return mid;
    }

    std::optional<size_t> try_split(const BBox& bbox, size_t begin, size_t end) override
    {
        PerAxisBins per_axis_bins;
        fill_bins(per_axis_bins, bbox, begin, end);

        auto largest_axis = bbox.get_diagonal().get_largest_axis();
        auto best_split = Split{ BinCount / 2, std::numeric_limits<Scalar>::max(), largest_axis };
        for (size_t axis = 0; axis < Node::dimension; ++axis)
            find_best_split(axis, per_axis_bins[axis], best_split);

        // Make sure that the split is good before proceeding with it
        auto leaf_cost = config_.sah.get_non_split_cost(begin, end, bbox);
        if (best_split.cost >= leaf_cost) {
            if (end - begin <= config_.max_leaf_size)
                return std::nullopt;
            return fallback_split(largest_axis, begin, end);
        }

        auto split_pos = fast_mul_add(
          bbox.get_diagonal()[best_split.axis] / static_cast<Scalar>(BinCount),
          static_cast<Scalar>(best_split.bin_id),
          bbox.min[best_split.axis]);

        size_t index = std::partition(prim_ids_.begin() + begin, prim_ids_.begin() + end,
                                      [&](size_t i) { return centers_[i][best_split.axis] < split_pos; }) -
                       prim_ids_.begin();
        if (index == begin || index == end)
            return fallback_split(largest_axis, begin, end);

        return std::make_optional(index);
    }
};

/// Multi-threaded top-down builder that partitions primitives using a grid. Multiple instances
/// of a single-threaded builder are run in parallel on that partition, generating many small
/// trees. Finally, a top-level tree is built on these smaller trees to form the final BVH.
/// This builder is inspired by
/// "Rapid Bounding Volume Hierarchy Generation using Mini Trees", by P. Ganestam et al.
template<typename Node, typename MortonCode = uint32_t>
class MiniTreeBuilder {
    using Scalar = typename Node::Scalar;
    using Vec = bvh::Vec<Scalar, Node::dimension>;
    using BBox = bvh::BBox<Scalar, Node::dimension>;

public:
    struct Config : TopDownSahBuilder<Node>::Config {
        /// Flag that turns on/off mini-tree pruning.
        bool enable_pruning = true;

        /// Threshold on the area of a mini-tree node above which it is pruned, expressed in
        /// fraction of the area of bounding box around the entire set of primitives.
        Scalar pruning_area_ratio = static_cast<Scalar>(0.01);

        /// Minimum number of primitives per parallel task.
        size_t parallel_threshold = 1024;

        /// Log of the dimension of the grid used to split the workload horizontally.
        size_t log2_grid_dim = 4;
    };

    /// Starts building a BVH with the given primitive data. The build algorithm is multi-threaded,
    /// and runs on the given thread pool.
    BVH_ALWAYS_INLINE static Bvh<Node> build(
      ThreadPool& thread_pool,
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config = {})
    {
        MiniTreeBuilder builder(thread_pool, bboxes, centers, config);
        auto mini_trees = builder.build_mini_trees();
        if (config.enable_pruning)
            mini_trees = builder.prune_mini_trees(std::move(mini_trees));
        return builder.build_top_bvh(mini_trees);
    }

private:
    friend struct BuildTask;

    struct Bin {
        std::vector<size_t> ids;

        BVH_ALWAYS_INLINE void add(size_t id)
        {
            ids.push_back(id);
        }

        BVH_ALWAYS_INLINE void merge(Bin&& other)
        {
            if (ids.empty())
                ids = std::move(other.ids);
            else {
                ids.insert(ids.end(), other.ids.begin(), other.ids.end());
                other.ids.clear();
            }
        }
    };

    struct LocalBins {
        std::vector<Bin> bins;

        BVH_ALWAYS_INLINE Bin& operator[](size_t i)
        {
            return bins[i];
        }

        BVH_ALWAYS_INLINE const Bin& operator[](size_t i) const
        {
            return bins[i];
        }

        BVH_ALWAYS_INLINE void merge_small_bins(size_t threshold)
        {
            for (size_t i = 0; i < bins.size();) {
                size_t j = i + 1;
                for (; j < bins.size() && bins[j].ids.size() + bins[i].ids.size() <= threshold; ++j)
                    bins[i].merge(std::move(bins[j]));
                i = j;
            }
        }

        BVH_ALWAYS_INLINE void remove_empty_bins()
        {
            bins.resize(std::remove_if(bins.begin(), bins.end(),
                                       [](const Bin& bin) { return bin.ids.empty(); }) -
                        bins.begin());
        }

        BVH_ALWAYS_INLINE void merge(LocalBins&& other)
        {
            bins.resize(std::max(bins.size(), other.bins.size()));
            for (size_t i = 0, n = std::min(bins.size(), other.bins.size()); i < n; ++i)
                bins[i].merge(std::move(other[i]));
        }
    };

    struct BuildTask {
        MiniTreeBuilder *builder;
        Bvh<Node>& bvh;
        std::vector<size_t> prim_ids;

        std::vector<BBox> bboxes;
        std::vector<Vec> centers;

        BuildTask(
          MiniTreeBuilder *builder,
          Bvh<Node>& bvh,
          std::vector<size_t>&& prim_ids) :
          builder(builder), bvh(bvh), prim_ids(std::move(prim_ids))
        {
        }

        BVH_ALWAYS_INLINE void run()
        {
            // Make sure that rebuilds produce the same BVH
            std::sort(prim_ids.begin(), prim_ids.end());

            // Extract bounding boxes and centers for this set of primitives
            bboxes.resize(prim_ids.size());
            centers.resize(prim_ids.size());
            for (size_t i = 0; i < prim_ids.size(); ++i) {
                bboxes[i] = builder->bboxes_[prim_ids[i]];
                centers[i] = builder->centers_[prim_ids[i]];
            }

            bvh = BinnedSahBuilder<Node>::build(bboxes, centers, builder->config_);

            // Permute primitive indices so that they index the proper set of primitives
            for (size_t i = 0; i < bvh.prim_ids.size(); ++i)
                bvh.prim_ids[i] = prim_ids[bvh.prim_ids[i]];
        }
    };

    ParallelExecutor executor_;
    std::span<const BBox> bboxes_;
    std::span<const Vec> centers_;
    const Config& config_;

    BVH_ALWAYS_INLINE MiniTreeBuilder(
      ThreadPool& thread_pool,
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config) :
      executor_(thread_pool), bboxes_(bboxes), centers_(centers), config_(config)
    {
        assert(bboxes.size() == centers.size());
    }

    std::vector<Bvh<Node>> build_mini_trees()
    {
        // Compute the bounding box of all centers
        auto center_bbox = executor_.reduce(
          0, bboxes_.size(), BBox::make_empty(),
          [this](BBox& bbox, size_t begin, size_t end) {
              for (size_t i = begin; i < end; ++i)
                  bbox.extend(centers_[i]);
          },
          [](BBox& bbox, const BBox& other) { bbox.extend(other); });

        assert(config_.log2_grid_dim <= std::numeric_limits<MortonCode>::digits / Node::dimension);
        auto bin_count = size_t{ 1 } << (config_.log2_grid_dim * Node::dimension);
        auto grid_dim = size_t{ 1 } << config_.log2_grid_dim;
        auto grid_scale = Vec(static_cast<Scalar>(grid_dim)) * safe_inverse(center_bbox.get_diagonal());
        auto grid_offset = -center_bbox.min * grid_scale;

        // Place primitives in bins
        auto final_bins = executor_.reduce(
          0, bboxes_.size(), LocalBins{},
          [&](LocalBins& local_bins, size_t begin, size_t end) {
              local_bins.bins.resize(bin_count);
              for (size_t i = begin; i < end; ++i) {
                  auto p = robust_max(fast_mul_add(centers_[i], grid_scale, grid_offset), Vec(0));
                  auto x = std::min(grid_dim - 1, static_cast<size_t>(p[0]));
                  auto y = std::min(grid_dim - 1, static_cast<size_t>(p[1]));
                  auto z = std::min(grid_dim - 1, static_cast<size_t>(p[2]));
                  local_bins[morton_encode(x, y, z) & (bin_count - 1)].add(i);
              }
          },
          [&](LocalBins& result, LocalBins&& other) { result.merge(std::move(other)); });

        // Note: Merging small bins will deteriorate the quality of the top BVH if there is no
        // pruning, since it will then produce larger mini-trees. For this reason, it is only enabled
        // when mini-tree pruning is enabled.
        if (config_.enable_pruning)
            final_bins.merge_small_bins(config_.parallel_threshold);
        final_bins.remove_empty_bins();

        // Iterate over bins to collect groups of primitives and build BVHs over them in parallel
        std::vector<Bvh<Node>> mini_trees(final_bins.bins.size());
        for (size_t i = 0; i < final_bins.bins.size(); ++i) {
            auto task = new BuildTask(this, mini_trees[i], std::move(final_bins[i].ids));
            executor_.thread_pool.push([task](size_t) { task->run(); delete task; });
        }
        executor_.thread_pool.wait();

        return mini_trees;
    }

    std::vector<Bvh<Node>> prune_mini_trees(std::vector<Bvh<Node>>&& mini_trees)
    {
        // Compute the area threshold based on the area of the entire set of primitives
        auto avg_area = static_cast<Scalar>(0.);
        for (auto& mini_tree : mini_trees)
            avg_area += mini_tree.get_root().get_bbox().get_half_area();
        avg_area /= static_cast<Scalar>(mini_trees.size());
        auto threshold = avg_area * config_.pruning_area_ratio;

        // Cull nodes whose area is above the threshold
        std::stack<size_t> stack;
        std::vector<std::pair<size_t, size_t>> pruned_roots;
        for (size_t i = 0; i < mini_trees.size(); ++i) {
            stack.push(0);
            auto& mini_tree = mini_trees[i];
            while (!stack.empty()) {
                auto node_id = stack.top();
                auto& node = mini_tree.nodes[node_id];
                stack.pop();
                if (node.get_bbox().get_half_area() < threshold || node.is_leaf()) {
                    pruned_roots.emplace_back(i, node_id);
                }
                else {
                    stack.push(node.index.first_id);
                    stack.push(node.index.first_id + 1);
                }
            }
        }

        // Extract the BVHs rooted at the previously computed indices
        std::vector<Bvh<Node>> pruned_trees(pruned_roots.size());
        executor_.for_each(0, pruned_roots.size(),
                           [&](size_t begin, size_t end) {
                               for (size_t i = begin; i < end; ++i)
                                   if (pruned_roots[i].second == 0)
                                       pruned_trees[i] = std::move(mini_trees[pruned_roots[i].first]);
                                   else
                                       pruned_trees[i] = mini_trees[pruned_roots[i].first]
                                                           .extract_bvh(pruned_roots[i].second);
                           });
        return pruned_trees;
    }

    Bvh<Node> build_top_bvh(std::vector<Bvh<Node>>& mini_trees)
    {
        // Build a BVH using the mini trees as leaves
        std::vector<Vec> centers(mini_trees.size());
        std::vector<BBox> bboxes(mini_trees.size());
        for (size_t i = 0; i < mini_trees.size(); ++i) {
            bboxes[i] = mini_trees[i].get_root().get_bbox();
            centers[i] = bboxes[i].get_center();
        }

        typename SweepSahBuilder<Node>::Config config = config_;
        config.max_leaf_size = 1; // Needs to have only one mini-tree in each leaf
        auto bvh = SweepSahBuilder<Node>::build(bboxes, centers, config);

        // Compute the offsets to apply to primitive and node indices
        std::vector<size_t> node_offsets(mini_trees.size());
        std::vector<size_t> prim_offsets(mini_trees.size());
        size_t node_count = bvh.nodes.size();
        size_t prim_count = 0;
        for (size_t i = 0; i < mini_trees.size(); ++i) {
            node_offsets[i] = node_count - 1; // Skip root node
            prim_offsets[i] = prim_count;
            node_count += mini_trees[i].nodes.size() - 1; // idem
            prim_count += mini_trees[i].prim_ids.size();
        }

        // Helper function to copy and fix the child/primitive index of a node
        auto copy_node = [&](size_t i, Node& dst_node, const Node& src_node) {
            dst_node = src_node;
            dst_node.index.first_id += static_cast<typename Node::Index::Type>(
              src_node.is_leaf() ? prim_offsets[i] : node_offsets[i]);
        };

        // Make the leaves of the top BVH point to the right internal nodes
        for (auto& node : bvh.nodes) {
            if (!node.is_leaf())
                continue;
            assert(node.index.prim_count == 1);
            size_t tree_id = bvh.prim_ids[node.index.first_id];
            copy_node(tree_id, node, mini_trees[tree_id].get_root());
        }

        bvh.nodes.resize(node_count);
        bvh.prim_ids.resize(prim_count);
        executor_.for_each(0, mini_trees.size(),
                           [&](size_t begin, size_t end) {
                               for (size_t i = begin; i < end; ++i) {
                                   auto& mini_tree = mini_trees[i];

                                   // Copy the nodes of the mini tree with the offsets applied, without copying
                                   // the root node (since it is already copied to the top-level part of the BVH).
                                   for (size_t j = 1; j < mini_tree.nodes.size(); ++j)
                                       copy_node(i, bvh.nodes[node_offsets[i] + j], mini_tree.nodes[j]);

                                   std::copy(
                                     mini_tree.prim_ids.begin(),
                                     mini_tree.prim_ids.end(),
                                     bvh.prim_ids.begin() + prim_offsets[i]);
                               }
                           });

        return bvh;
    }
};

template<typename Node>
class ReinsertionOptimizer {
    using Scalar = typename Node::Scalar;
    using BBox = bvh::BBox<Scalar, Node::dimension>;

public:
    struct Config {
        /// Fraction of the number of nodes to optimize per iteration.
        Scalar batch_size_ratio = static_cast<Scalar>(0.05);

        /// Maximum number of iterations.
        size_t max_iter_count = 3;
    };

    static void optimize(ThreadPool& thread_pool, Bvh<Node>& bvh, const Config& config = {})
    {
        ParallelExecutor executor(thread_pool);
        optimize(executor, bvh, config);
    }

    static void optimize(Bvh<Node>& bvh, const Config& config = {})
    {
        SequentialExecutor executor;
        optimize(executor, bvh, config);
    }

private:
    struct Candidate {
        size_t node_id = 0;
        Scalar cost = -std::numeric_limits<Scalar>::max();

        BVH_ALWAYS_INLINE bool operator>(const Candidate& other) const
        {
            return cost > other.cost;
        }
    };

    struct Reinsertion {
        size_t from = 0;
        size_t to = 0;
        Scalar area_diff = static_cast<Scalar>(0);

        BVH_ALWAYS_INLINE bool operator>(const Reinsertion& other) const
        {
            return area_diff > other.area_diff;
        }
    };

    Bvh<Node>& bvh_;
    std::vector<size_t> parents_;

    ReinsertionOptimizer(Bvh<Node>& bvh, std::vector<size_t>&& parents) :
      bvh_(bvh), parents_(std::move(parents))
    {
    }

    template<typename Derived>
    static void optimize(Executor<Derived>& executor, Bvh<Node>& bvh, const Config& config)
    {
        auto parents = compute_parents(executor, bvh);
        ReinsertionOptimizer<Node>(bvh, std::move(parents)).optimize(executor, config);
    }

    template<typename Derived>
    static std::vector<size_t> compute_parents(Executor<Derived>& executor, const Bvh<Node>& bvh)
    {
        std::vector<size_t> parents(bvh.nodes.size());
        parents[0] = 0;
        executor.for_each(0, bvh.nodes.size(),
                          [&](size_t begin, size_t end) {
                              for (size_t i = begin; i < end; ++i) {
                                  auto& node = bvh.nodes[i];
                                  if (!node.is_leaf()) {
                                      parents[node.index.first_id + 0] = i;
                                      parents[node.index.first_id + 1] = i;
                                  }
                              }
                          });
        return parents;
    }

    BVH_ALWAYS_INLINE std::vector<Candidate> find_candidates(size_t target_count)
    {
        // Gather the `target_count` nodes that have the highest cost.
        // Note that this may produce fewer nodes if the BVH has fewer than `target_count` nodes.
        const auto node_count = std::min(bvh_.nodes.size(), target_count + 1);
        std::vector<Candidate> candidates;
        for (size_t i = 1; i < node_count; ++i)
            candidates.push_back(Candidate{ i, bvh_.nodes[i].get_bbox().get_half_area() });
        std::make_heap(candidates.begin(), candidates.end(), std::greater<>{});
        for (size_t i = node_count; i < bvh_.nodes.size(); ++i) {
            auto cost = bvh_.nodes[i].get_bbox().get_half_area();
            if (candidates.front().cost < cost) {
                std::pop_heap(candidates.begin(), candidates.end(), std::greater<>{});
                candidates.back() = Candidate{ i, cost };
                std::push_heap(candidates.begin(), candidates.end(), std::greater<>{});
            }
        }
        return candidates;
    }

    Reinsertion find_reinsertion(size_t node_id)
    {
        assert(node_id != 0);

        /*
         * Here is an example that explains how the cost of a reinsertion is computed. For the
         * reinsertion from A to C, in the figure below, we need to remove P1, replace it by B,
         * and create a node that holds A and C and place it where C was.
         *
         *             R
         *            / \
         *          Pn   Q1
         *          /     \
         *        ...     ...
         *        /         \
         *       P1          C
         *      / \
         *     A   B
         *
         * The resulting area *decrease* is (SA(x) means the surface area of x):
         *
         *     SA(P1) +                                                : P1 was removed
         *     SA(P2) - SA(B) +                                        : P2 now only contains B
         *     SA(P3) - SA(B U sibling(P2)) +                          : Same but for P3
         *     ... +
         *     SA(Pn) - SA(B U sibling(P2) U ... U sibling(P(n - 1)) + : Same but for Pn
         *     0 +                                                     : R does not change
         *     SA(Q1) - SA(Q1 U A) +                                   : Q1 now contains A
         *     SA(Q2) - SA(Q2 U A) +                                   : Q2 now contains A
         *     ... +
         *     -SA(A U C)                                              : For the parent of A and C
         */

        Reinsertion best_reinsertion{ .from = node_id };
        auto node_area = bvh_.nodes[node_id].get_bbox().get_half_area();
        auto parent_area = bvh_.nodes[parents_[node_id]].get_bbox().get_half_area();
        auto area_diff = parent_area;
        auto sibling_id = Node::get_sibling_id(node_id);
        auto pivot_bbox = bvh_.nodes[sibling_id].get_bbox();
        auto parent_id = parents_[node_id];
        auto pivot_id = parent_id;

        std::vector<std::pair<Scalar, size_t>> stack;
        do {
            // Try to find a reinsertion in the sibling at the current level of the tree
            stack.emplace_back(area_diff, sibling_id);
            while (!stack.empty()) {
                auto top = stack.back();
                stack.pop_back();
                if (top.first - node_area <= best_reinsertion.area_diff)
                    continue;

                auto& dst_node = bvh_.nodes[top.second];
                auto merged_area = dst_node.get_bbox().extend(bvh_.nodes[node_id].get_bbox()).get_half_area();
                auto reinsert_area = top.first - merged_area;
                if (reinsert_area > best_reinsertion.area_diff) {
                    best_reinsertion.to = top.second;
                    best_reinsertion.area_diff = reinsert_area;
                }

                if (!dst_node.is_leaf()) {
                    auto child_area = reinsert_area + dst_node.get_bbox().get_half_area();
                    stack.emplace_back(child_area, dst_node.index.first_id + 0);
                    stack.emplace_back(child_area, dst_node.index.first_id + 1);
                }
            }

            // Compute the bounding box on the path from the node to the root, and record the
            // corresponding decrease in area.
            if (pivot_id != parent_id) {
                pivot_bbox.extend(bvh_.nodes[sibling_id].get_bbox());
                area_diff += bvh_.nodes[pivot_id].get_bbox().get_half_area() - pivot_bbox.get_half_area();
            }

            sibling_id = Node::get_sibling_id(pivot_id);
            pivot_id = parents_[pivot_id];
        } while (pivot_id != 0);

        if (best_reinsertion.to == Node::get_sibling_id(best_reinsertion.from) ||
            best_reinsertion.to == parents_[best_reinsertion.from])
            best_reinsertion = {};
        return best_reinsertion;
    }

    BVH_ALWAYS_INLINE void reinsert_node(size_t from, size_t to)
    {
        auto sibling_id = Node::get_sibling_id(from);
        auto parent_id = parents_[from];
        auto sibling_node = bvh_.nodes[sibling_id];
        auto dst_node = bvh_.nodes[to];

        bvh_.nodes[to].make_inner(Node::get_left_sibling_id(from));
        bvh_.nodes[sibling_id] = dst_node;
        bvh_.nodes[parent_id] = sibling_node;

        if (!dst_node.is_leaf()) {
            parents_[dst_node.index.first_id + 0] = sibling_id;
            parents_[dst_node.index.first_id + 1] = sibling_id;
        }
        if (!sibling_node.is_leaf()) {
            parents_[sibling_node.index.first_id + 0] = parent_id;
            parents_[sibling_node.index.first_id + 1] = parent_id;
        }

        parents_[sibling_id] = to;
        parents_[from] = to;
        refit_from(to);
        refit_from(parent_id);
    }

    BVH_ALWAYS_INLINE void refit_from(size_t index)
    {
        do {
            auto& node = bvh_.nodes[index];
            if (!node.is_leaf())
                node.set_bbox(
                  bvh_.nodes[node.index.first_id + 0].get_bbox().extend(
                    bvh_.nodes[node.index.first_id + 1].get_bbox()));
            index = parents_[index];
        } while (index != 0);
    }

    BVH_ALWAYS_INLINE auto get_conflicts(size_t from, size_t to)
    {
        return std::array<size_t, 5>{
            to, from,
            Node::get_sibling_id(from),
            parents_[to],
            parents_[from]
        };
    }

    template<typename Derived>
    void optimize(Executor<Derived>& executor, const Config& config)
    {
        auto batch_size = std::max(size_t{ 1 },
                                   static_cast<size_t>(static_cast<Scalar>(bvh_.nodes.size()) * config.batch_size_ratio));
        std::vector<Reinsertion> reinsertions;
        std::vector<bool> touched(bvh_.nodes.size());

        for (size_t iter = 0; iter < config.max_iter_count; ++iter) {
            auto candidates = find_candidates(batch_size);

            std::fill(touched.begin(), touched.end(), false);
            reinsertions.resize(candidates.size());
            executor.for_each(0, candidates.size(),
                              [&](size_t begin, size_t end) {
                                  for (size_t i = begin; i < end; ++i)
                                      reinsertions[i] = find_reinsertion(candidates[i].node_id);
                              });

            reinsertions.erase(std::remove_if(reinsertions.begin(), reinsertions.end(),
                                              [](auto& r) { return r.area_diff <= 0; }),
                               reinsertions.end());
            std::sort(reinsertions.begin(), reinsertions.end(), std::greater<>{});

            for (auto& reinsertion : reinsertions) {
                auto conflicts = get_conflicts(reinsertion.from, reinsertion.to);
                if (std::any_of(conflicts.begin(), conflicts.end(), [&](size_t i) { return touched[i]; }))
                    continue;
                for (auto conflict : conflicts)
                    touched[conflict] = true;
                reinsert_node(reinsertion.from, reinsertion.to);
            }
        }
    }
};

/// This builder is only a wrapper around all the other builders, which selects the best builder
/// depending on the desired BVH quality and whether a multi-threaded build is desired.
template<typename Node>
class DefaultBuilder {
    using Scalar = typename Node::Scalar;
    using Vec = bvh::Vec<Scalar, Node::dimension>;
    using BBox = bvh::BBox<Scalar, Node::dimension>;

public:
    enum class Quality {
        Low,
        Medium,
        High
    };

    struct Config : TopDownSahBuilder<Node>::Config {
        /// The quality of the BVH produced by the builder. The higher the quality the faster the
        /// BVH is to traverse, but the slower it is to build.
        Quality quality = Quality::High;

        /// Threshold, in number of primitives, under which the builder operates in a single-thread.
        size_t parallel_threshold = 1024;
    };

    /// Build a BVH in parallel using the given thread pool.
    BVH_ALWAYS_INLINE static Bvh<Node> build(
      ThreadPool& thread_pool,
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config = {})
    {
        if (bboxes.size() < config.parallel_threshold)
            return build(bboxes, centers, config);
        auto bvh = MiniTreeBuilder<Node>::build(
          thread_pool, bboxes, centers, make_mini_tree_config(config));
        if (config.quality == Quality::High)
            ReinsertionOptimizer<Node>::optimize(thread_pool, bvh);
        return bvh;
    }

    /// Build a BVH in a single-thread.
    BVH_ALWAYS_INLINE static Bvh<Node> build(
      std::span<const BBox> bboxes,
      std::span<const Vec> centers,
      const Config& config = {})
    {
        if (config.quality == Quality::Low)
            return BinnedSahBuilder<Node>::build(bboxes, centers, config);
        else {
            auto bvh = SweepSahBuilder<Node>::build(bboxes, centers, config);
            if (config.quality == Quality::High)
                ReinsertionOptimizer<Node>::optimize(bvh);
            return bvh;
        }
    }

private:
    BVH_ALWAYS_INLINE static auto make_mini_tree_config(const Config& config)
    {
        typename MiniTreeBuilder<Node>::Config mini_tree_config;
        static_cast<typename TopDownSahBuilder<Node>::Config&>(mini_tree_config) = config;
        mini_tree_config.enable_pruning = config.quality == Quality::Low ? false : true;
        mini_tree_config.pruning_area_ratio =
          config.quality == Quality::High ? static_cast<Scalar>(0.01) : static_cast<Scalar>(0.1);
        mini_tree_config.parallel_threshold = config.parallel_threshold;
        return mini_tree_config;
    }
};

} // namespace bvh

#endif
