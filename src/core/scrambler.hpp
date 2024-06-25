#ifndef YART_SCRAMBLER_HPP
#define YART_SCRAMBLER_HPP

#include <math/math.hpp>
#include "rng.hpp"

namespace yart {
using namespace math;

/**
 * Abstract scrambler class. Runtime polymorphism isn't actually required,
 * but inheriting a common base class lets us use the derived_from concept
 * in templated classes
 */
class Scrambler {
public:
  virtual constexpr uint32_t operator()(uint32_t v) noexcept = 0;
};

/**
 * Null scrambler: does nothing!
 */
class NullScrambler : public Scrambler {
public:
  [[nodiscard]] constexpr uint32_t operator()(uint32_t v) noexcept override {
    return v;
  }
};

/**
 * Applies a random permutation to the sample digits.
 * In base two, there are only two possible permutations for a digit, so
 * this can be done with a simple xor.
 */
class BinaryPermuteScrambler : public Scrambler {
public:
  constexpr explicit BinaryPermuteScrambler(uint32_t p) noexcept
    : m_permutation(p) {}

  [[nodiscard]] constexpr uint32_t operator()(uint32_t v) noexcept override {
    return m_permutation ^ v;
  }

private:
  uint32_t m_permutation;
};

/**
 * Fast approximation of Owen scrambling, lifted from pbrt
 * How does it work? No clue, but Owen scrambling defines digit permutations
 * that depend on multiple digit indices.
 */
class FastOwenScrambler : public Scrambler {
public:
  constexpr explicit FastOwenScrambler(uint32_t seed) noexcept: m_seed(seed) {}

  [[nodiscard]] constexpr uint32_t operator()(uint32_t v) noexcept override {
    v = reverseBits32(v);
    v ^= v * 0x3d20adea;
    v += m_seed;
    v *= (m_seed >> 16) | 1;
    v ^= v * 0x05526c56;
    v ^= v * 0x53a22864;
    return reverseBits32(v);
  }

private:
  uint32_t m_seed;
};

class OwenScrambler : public Scrambler {
public:
  constexpr explicit OwenScrambler(uint32_t seed) noexcept: m_seed(seed) {}

  [[nodiscard]] constexpr uint32_t operator()(uint32_t v) noexcept override {
    if (m_seed & 1) v ^= 1u << 31;
    for (uint32_t b = 1; b < 32; b++) {
      uint32_t mask = (~0u) << (32 - b); // Mask the b high bits
      if (uint32_t(mixBits(v & mask) ^ m_seed) & (1u << b))
        v ^= 1u << (31 - b);
    }
    return v;
  }

private:
  uint32_t m_seed;
};


}

#endif //YART_SCRAMBLER_HPP
