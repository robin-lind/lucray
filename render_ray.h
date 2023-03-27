#include "math/vector.h"
#include "lucmath_gen.h"
#include "traversal.h"

auto render_ray(math::float3 ray_org, math::float3 ray_dir, traversal<float>& trav)
{
    struct intersection {
        math::float3 point;
        math::float3 normal;
        math::float2 texcoord;
        math::float3 wi;
        int material;
        int shader;
        unsigned int prim;
    };

    auto intersect = [&](const math::vector_tn<float, 3>& org, const math::vector_tn<float, 3>& dir) -> std::optional<intersection> {
        const auto hit = trav.traverse(org, dir);
        if (hit) {
            intersection in{};
            return in;
        }
        return {};
    };

    auto sample_light = [&]() {

    };

    math::float3 throughput(1.f);
    math::float3 radiance(0.f);
    auto add_light = [&radiance, &throughput](const math::float3& light) {
        radiance += throughput * light;
    };
    const int max_depth = 5;
    for (int depth = 0; depth < max_depth; depth++) {
        const auto hit = intersect(ray_org, ray_dir);
        if (hit) {
        }
        break;
    }
    // return radiance;

    return math::float3();
}