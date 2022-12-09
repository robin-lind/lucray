#pragma once
#include "lucmath.h"
#include "lucmath_gen.h"
#include "sampler.h"
#include "ray.h"

struct RayCamera
{
    RayCamera(int width, int height, luc::Vector3 eye, luc::Vector3 target, luc::Vector3 up, float fov);
    Ray          CameraRay(int x, int y, Sampler         &sampler) const;
    luc::Vector3 pos;
    luc::Vector3 X, Y, Z;
    luc::Vector2 resolution;
    float        aspect;
    float        backfocus;
};