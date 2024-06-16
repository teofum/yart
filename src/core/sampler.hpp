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
  NaiveSampler() noexcept;

  void startPixelSample(uint2 p, uint32_t sample) noexcept override;

  float get1D() noexcept override;

  float2 get2D() noexcept override;

  float2 getPixel2D() noexcept override;
};

}

#endif //YART_SAMPLER_HPP
