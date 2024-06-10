#ifndef YART_RANDOM_HPP
#define YART_RANDOM_HPP

#include <random>
#include <xoshiro.hpp>

#include "vec.hpp"

namespace yart::math::random {

[[nodiscard]] float2 pixelJitterSquare(Xoshiro::Xoshiro256PP& rng);

[[nodiscard]] float2 pixelJitterGaussian(Xoshiro::Xoshiro256PP& rng);

[[nodiscard]] float3 randomCosineVec(Xoshiro::Xoshiro256PP& rng) noexcept;

}

#endif //YART_RANDOM_HPP
