#pragma once
#include "math/vector.h"
#include <utility>

template<typename TFloat = float>
struct ray_camera {
    math::vector<TFloat, 3> pos;
    math::vector<TFloat, 3> X, Y, Z;
    float inv_aspect;
    float backfocus;

    ray_camera(int width, int height, math::vector<TFloat, 3> eye, math::vector<TFloat, 3> target, math::vector<TFloat, 3> up, float fov)
    {
        pos = eye;
        Z = math::normalize(target - eye);
        X = math::normalize(math::cross(Z, up));
        Y = math::normalize(math::cross(Z, X));
        inv_aspect = (float)height / (float)width; // image width is 1
        backfocus = .5f / std::tan(fov * .5f * 3.14159f / 180.f);
    }

    std::pair<math::vector<TFloat, 3>, math::vector<TFloat, 3>> ray(math::vector<TFloat, 2> uv) const
    {
        const auto image_plane = math::normalize(math::vector<TFloat, 3>(uv.u, uv.v * inv_aspect, backfocus));
        const auto dir = (X * image_plane.x) + (Y * image_plane.y) + (Z * image_plane.z);
        return std::make_pair(pos, dir);
    }
};
