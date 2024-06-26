#ifndef YART_SAMPLER_HPP
#define YART_SAMPLER_HPP

#include <math/math.hpp>
#include "rng.hpp"
#include "scrambler.hpp"
#include "sobol-matrices.hpp"

namespace yart {
using namespace math;

class Sampler {
public:
  virtual void startPixelSample(uint2 p, uint32_t sample) noexcept = 0;

  virtual float get1D() noexcept = 0;

  virtual float2 get2D() noexcept = 0;

  virtual float2 getPixel2D() noexcept = 0;

protected:
  RNG m_rng;
};

/**
 * Simple naive sampler, returns independent random variables for each sampled
 * dimension.
 */
class NaiveSampler : public Sampler {
public:
  NaiveSampler(size_t samplesPerPixel, const uint2& renderSize) noexcept {};

  void startPixelSample(uint2 p, uint32_t sample) noexcept override;

  float get1D() noexcept override;

  float2 get2D() noexcept override;

  float2 getPixel2D() noexcept override;
};

/**
 * Stratified sampler, uses stratification to ensure a better distribution of
 * samples across dimensions.
 */
class StratifiedSampler : public Sampler {
public:
  StratifiedSampler(size_t samplesPerPixel, const uint2& renderSize) noexcept
    : m_xSamples(uint32_t(std::ceil(std::sqrt(samplesPerPixel)))),
      m_ySamples(uint32_t(std::ceil(std::sqrt(samplesPerPixel)))) {};

  void startPixelSample(uint2 p, uint32_t sample) noexcept override;

  float get1D() noexcept override;

  float2 get2D() noexcept override;

  float2 getPixel2D() noexcept override;

private:
  uint2 m_pixel;
  uint32_t m_dim = 0, m_sampleIdx = 0;
  uint32_t m_xSamples = 256, m_ySamples = 256;
};

/**
 * An implementation of the ZSobolSampler from pbrt (see pbr-book 4ed, 8.7)
 * @tparam R scrambler to use for randomization
 */
template<typename R> requires std::derived_from<R, Scrambler>
class SobolSampler : public Sampler {
public:
  SobolSampler(
    size_t samplesPerPixel,
    const uint2& renderSize
  ) noexcept {
    m_log2spp = log2Int(float(samplesPerPixel));
    uint32_t res = roundUpPow2(int32_t(maxComponent(renderSize)));
    uint32_t log4spp = (m_log2spp + 1) / 2;
    m_nBase4Digits = log2Int(float(res)) + log4spp;
  }

  void startPixelSample(uint2 p, uint32_t sample) noexcept override {
    m_dim = 0;
    m_mortonIndex = (encodeMorton2(p.x(), p.y()) << m_log2spp) | sample;
  }

  float get1D() noexcept override {
    uint64_t sampleIdx = getSampleIndex();
    m_dim++;
    uint32_t sampleHash = hash(m_dim);
    return sobolSample(sampleIdx, 0, R(sampleHash));
  }

  float2 get2D() noexcept override {
    uint64_t sampleIdx = getSampleIndex();
    m_dim += 2;
    uint64_t sampleHashBits = hash(m_dim);
    uint32_t sampleHash[2]{uint32_t(sampleHashBits),
                           uint32_t(sampleHashBits >> 32)};

    return {
      sobolSample(sampleIdx, 0, R(sampleHash[0])),
      sobolSample(sampleIdx, 1, R(sampleHash[1]))
    };
  }

  float2 getPixel2D() noexcept override { return get2D(); }

private:
  uint32_t m_log2spp, m_nBase4Digits, m_dim = 0;
  uint64_t m_mortonIndex = 0;

  static constexpr uint8_t permutations[24][4] = {
    {0, 1, 2, 3},
    {0, 1, 3, 2},
    {0, 2, 1, 3},
    {0, 2, 3, 1},
    {0, 3, 2, 1},
    {0, 3, 1, 2},
    {1, 0, 2, 3},
    {1, 0, 3, 2},
    {1, 2, 0, 3},
    {1, 2, 3, 0},
    {1, 3, 2, 0},
    {1, 3, 0, 2},
    {2, 1, 0, 3},
    {2, 1, 3, 0},
    {2, 0, 1, 3},
    {2, 0, 3, 1},
    {2, 3, 0, 1},
    {2, 3, 1, 0},
    {3, 1, 2, 0},
    {3, 1, 0, 2},
    {3, 2, 1, 0},
    {3, 2, 0, 1},
    {3, 0, 2, 1},
    {3, 0, 1, 2}
  };

  static constexpr float sobolSample(uint64_t d, uint32_t dim, R scrambler) {
    uint32_t v = 0;
    if (dim == 0) {
      v ^= reverseBits32(d);
    } else {
      for (uint32_t i = dim * sobol::sobolMatrixSize; d != 0; d >>= 1, i++) {
        v ^= (d & 1) * sobol::matrices[i];
      }
    }
    v = scrambler(v);
    return std::min(float(v) * 0x1p-32f, oneMinusEpsilon);
  }

  constexpr uint64_t getSampleIndex() noexcept {
    uint64_t index = 0;
    bool pow2Samples = m_log2spp & 1;
    int lastDigit = pow2Samples ? 1 : 0;
    for (int i = m_nBase4Digits - 1; i >= lastDigit; i--) {
      uint32_t digitShift = 2 * i - lastDigit;
      uint32_t digit = (m_mortonIndex >> digitShift) & 3;
      uint64_t higherDigits = m_mortonIndex >> (digitShift + 2);
      uint32_t p = (mixBits(higherDigits ^ (0x55555555u * m_dim)) >> 24) % 24;
      digit = permutations[p][digit];
      index |= uint64_t(digit) << digitShift;
    }
    if (pow2Samples) {
      uint32_t digit = m_mortonIndex & 1;
      index |=
        digit ^ (mixBits((m_mortonIndex >> 1) ^ (0x55555555u * m_dim)) & 1);
    }
    return index;
  }
};

}

#endif //YART_SAMPLER_HPP
