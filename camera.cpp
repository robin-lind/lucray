#include "camera.h"
#include "lucmath_gen.h"
#include "sampler.h"

RayCamera::RayCamera(int width, int height, luc::Vector3 eye, luc::Vector3 target, luc::Vector3 up, float fov)
{
    pos = eye;
    Z = luc::Normalize(target - eye);
    X = luc::Normalize(luc::Cross(Z, up));
    Y = luc::Normalize(luc::Cross(Z, X));
    resolution = { static_cast<float>(width), static_cast<float>(height) };
    aspect = resolution.w / resolution.h;
    backfocus = .5f / std::tan(fov * .5f * 3.14159f / 180.f);
}

Ray RayCamera::CameraRay(int x, int y, Sampler& sampler) const
{
    const luc::Vector2 sample = sampler.Get2D();
    const luc::Vector2 as(1.f, 1.f / aspect);
    const luc::Vector2 xy(static_cast<float>(x), static_cast<float>(y));
    const luc::Vector2 uv = ((xy + sample + .5f) / resolution) - .5f;
    const luc::Vector3 image_plane = luc::Normalize(luc::Vector3{ uv * as, backfocus });
    const luc::Vector3 dir = X * image_plane.x + Y * image_plane.y + Z * image_plane.z;
    return { pos, dir };
}