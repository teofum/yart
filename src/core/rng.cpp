#include "rng.hpp"

namespace yart {

RNG::RNG(uint64_t seed) noexcept: m_rng(seed) {}

float RNG::uniform() noexcept {
  return m_dist(m_rng);
}

void RNG::seed(uint64_t seed) noexcept {
  m_rng.seed(seed);
}

}