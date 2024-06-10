#include "random.hpp"
#include "math.hpp"

namespace yart::math::random {

std::uniform_real_distribution<float> uniform;
std::normal_distribution<float> normal(0.0f, 0.5f);
std::normal_distribution<float> stdNormal(0.0f, 1.0f);

static float myStdNormal(Xoshiro::Xoshiro256PP& e) noexcept {
  static float cached;
  static bool hasCached = false;

  if (hasCached) {
    hasCached = false;
    return cached;
  }
  hasCached = true;

  float v1, v2, sx;
  for (int i = 0;; ++i) {
    v1 = 2.0f * uniform(e) - 1.0f;
    v2 = 2.0f * uniform(e) - 1.0f;
    sx = v1 * v1 + v2 * v2;
    if (sx && sx < 1.0f) break;
  }

  float fx = std::sqrt(-2.0f * std::log(sx) / sx);
  cached = fx * v2;
  return fx * v1;
}

float2 pixelJitterSquare(Xoshiro::Xoshiro256PP& rng) {
  return {
    uniform(rng) - 0.5f,
    uniform(rng) - 0.5f
  };
}

float2 pixelJitterGaussian(Xoshiro::Xoshiro256PP& rng) {
  return {normal(rng), normal(rng)};
}

float3 randomCosineVec(Xoshiro::Xoshiro256PP& rng) noexcept {
  const float r1 = uniform(rng);
  const float r2 = uniform(rng);

  const float phi = r1 * 2.0f * float(pi);
  const float sqrtr2 = std::sqrt(r2);
  const float x = std::cos(phi) * sqrtr2;
  const float y = std::sin(phi) * sqrtr2;
  const float z = std::sqrt(1.0f - r2);

  return {x, y, z};
}

}
