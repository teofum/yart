#include "sampling.hpp"

namespace yart::math::samplers {

float2 pixelJitterSquare(float2 u) {
  return {
    u.x() - 0.5f,
    u.y() - 0.5f
  };
}

float2 pixelJitterGaussian(float2 u) {
  float a = std::sqrt(-2.0f * log(u.x())) * 0.5f;
  float b = 2.0f * float(pi) * u.y();

  return {a * std::cos(b), a * std::sin(b)};
}

float3 sampleCosineHemisphere(float2 u) noexcept {
  const float phi = u.x() * 2.0f * float(pi);
  const float sqrtr2 = std::sqrt(u.y());
  const float x = std::cos(phi) * sqrtr2;
  const float y = std::sin(phi) * sqrtr2;
  const float z = std::sqrt(1.0f - u.y());

  return {x, y, z};
}

}
