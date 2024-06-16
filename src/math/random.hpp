#ifndef YART_RANDOM_HPP
#define YART_RANDOM_HPP

#include <random>
#include <xoshiro.hpp>

#include "vec.hpp"

namespace yart::math::random {

[[nodiscard]] float2 pixelJitterSquare(float2 uv);

[[nodiscard]] float3 randomCosineVec(float2 uv) noexcept;

}

#endif //YART_RANDOM_HPP
