#pragma once
#include <random>
#include "lucmath.h"

struct Sampler
{
    Sampler(int seed = 0) :
      seed(seed), mt((unsigned int)seed), rand_float(0.f, 1.f) {}

    float Get1D()
    {
        return rand_float(mt);
    }

    luc::Vector2 Get2D()
    {
        return { rand_float(mt), rand_float(mt) };
    }

private:
    int                                   seed;
    std::mt19937                          mt;
    std::uniform_real_distribution<float> rand_float;
};