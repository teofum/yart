#ifndef YART_RNG_HPP
#define YART_RNG_HPP

#include <random>
#include <xoshiro.hpp>

namespace yart {

class RNG {
public:
  RNG() noexcept = default;

  explicit RNG(uint64_t seed) noexcept;

  [[nodiscard]] float uniform() noexcept;

  void seed(uint64_t seed) noexcept;

private:
  Xoshiro::Xoshiro256PP m_rng;
  std::uniform_real_distribution<float> m_dist;
};

}

#endif //YART_RNG_HPP
