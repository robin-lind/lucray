#pragma once

#include "math/vector.h"

template<typename T>
struct animated {
    T t0, t1;
    animated() = default;

    animated(T _t0, T _t1) :
      t0(_t0), t1(_t1) {}

    T lerp(float t)
    {
        return math::lerp<T>(t, t0, t1);
    }
};