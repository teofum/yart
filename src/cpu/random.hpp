#ifndef YART_RANDOM_HPP
#define YART_RANDOM_HPP

#include <random>

#include <math/math.hpp>

namespace yart::cpu {
using namespace math;

float3 randomCosineVec(
  std::mt19937* e,
  std::uniform_real_distribution<float>& rand
) {
  const auto r1 = float(rand(*e));
  const auto r2 = float(rand(*e));

  const float phi = r1 * 2.0f * float(pi);
  const float sqrtr2 = std::sqrt(r2);
  const float x = std::cos(phi) * sqrtr2;
  const float y = std::sin(phi) * sqrtr2;
  const float z = std::sqrt(1.0f - r2);

  return {x, y, z};
}

}

#endif //YART_RANDOM_HPP
