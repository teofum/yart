#include "random.hpp"

namespace yart::math::random {

float2 pixelJitterSquare(float2 uv) {
  return {
    uv.x() - 0.5f,
    uv.y() - 0.5f
  };
}

float3 randomCosineVec(float2 uv) noexcept {
  const float phi = uv.x() * 2.0f * float(pi);
  const float sqrtr2 = std::sqrt(uv.y());
  const float x = std::cos(phi) * sqrtr2;
  const float y = std::sin(phi) * sqrtr2;
  const float z = std::sqrt(1.0f - uv.y());

  return {x, y, z};
}

}
