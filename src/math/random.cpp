#include "random.hpp"

namespace yart::math::random {

std::uniform_real_distribution<float> uniform;
std::normal_distribution<float> normal(0.0f, 0.5f);

float2 pixelJitterSquare(Xoshiro::Xoshiro256PP* e) {
  return {
    uniform(*e) - 0.5f,
    uniform(*e) - 0.5f
  };
}

float2 pixelJitterGaussian(Xoshiro::Xoshiro256PP* e) {
  return {normal(*e), normal(*e)};
}

float3 randomCosineVec(Xoshiro::Xoshiro256PP* e) noexcept {
  const float r1 = uniform(*e);
  const float r2 = uniform(*e);

  const float phi = r1 * 2.0f * float(pi);
  const float sqrtr2 = std::sqrt(r2);
  const float x = std::cos(phi) * sqrtr2;
  const float y = std::sin(phi) * sqrtr2;
  const float z = std::sqrt(1.0f - r2);

  return {x, y, z};
}

}
