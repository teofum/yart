#ifndef YART_SAMPLING_HPP
#define YART_SAMPLING_HPP

#include <random>
#include <xoshiro.hpp>

#include "vec.hpp"

namespace yart::math::samplers {

[[nodiscard]] float2 pixelJitterSquare(float2 u);

[[nodiscard]] float2 pixelJitterGaussian(float2 uv);

[[nodiscard]] float3 sampleCosineHemisphere(float2 u) noexcept;

}

#endif //YART_SAMPLING_HPP
