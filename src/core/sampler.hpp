#ifndef YART_SAMPLER_HPP
#define YART_SAMPLER_HPP

#include <math/math.hpp>
#include "rng.hpp"

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

class NaiveSampler : public Sampler {
public:
  NaiveSampler() noexcept = default;

  void startPixelSample(uint2 p, uint32_t sample) noexcept override;

  float get1D() noexcept override;

  float2 get2D() noexcept override;

  float2 getPixel2D() noexcept override;
};

class StratifiedSampler : public Sampler {
public:
  StratifiedSampler() noexcept = default;

  void startPixelSample(uint2 p, uint32_t sample) noexcept override;

  float get1D() noexcept override;

  float2 get2D() noexcept override;

  float2 getPixel2D() noexcept override;

private:
  uint2 m_pixel;
  uint32_t m_dim = 0, m_sampleIdx = 0;
  uint32_t m_xSamples = 4, m_ySamples = 4;
};

}

#endif //YART_SAMPLER_HPP
