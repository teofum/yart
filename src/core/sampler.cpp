#include "sampler.hpp"

namespace yart {

NaiveSampler::NaiveSampler() noexcept {
  std::random_device rd;
  m_rng.seed(rd());
}

void NaiveSampler::startPixelSample(uint2 p, uint32_t sample) noexcept {
  m_rng.seed(hash(p, sample));
}

float NaiveSampler::get1D() noexcept {
  return m_rng.uniform();
}

float2 NaiveSampler::get2D() noexcept {
  return {m_rng.uniform(), m_rng.uniform()};
}

float2 NaiveSampler::getPixel2D() noexcept {
  return get2D();
}

}