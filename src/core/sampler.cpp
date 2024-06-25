#include "sampler.hpp"

namespace yart {

void NaiveSampler::startPixelSample(uint2 p, uint32_t sample) noexcept {
  m_rng.seed(hash(p, sample));
}

float NaiveSampler::get1D() noexcept {
  return m_rng.uniform();
}

float2 NaiveSampler::get2D() noexcept {
  return {m_rng.uniform(), m_rng.uniform()};
}

float2 NaiveSampler::getPixel2D() noexcept { return get2D(); }

void StratifiedSampler::startPixelSample(uint2 p, uint32_t sample) noexcept {
  m_pixel = p;
  m_sampleIdx = sample;
  m_dim = 0;
  m_rng.seed(hash(p, sample));
}

float StratifiedSampler::get1D() noexcept {
  uint64_t sampleHash = hash(m_pixel, m_dim);
  uint32_t stratum = permel(m_sampleIdx, m_xSamples * m_ySamples, sampleHash);
  m_dim++;
  float delta = m_rng.uniform();
  return (float(stratum) + delta) / float(m_xSamples * m_ySamples);
}

float2 StratifiedSampler::get2D() noexcept {
  uint64_t sampleHash = hash(m_pixel, m_dim);
  uint32_t stratum = permel(m_sampleIdx, m_xSamples * m_ySamples, sampleHash);
  m_dim += 2;
  uint32_t x = stratum % m_xSamples, y = stratum / m_xSamples;
  float dx = m_rng.uniform(), dy = m_rng.uniform();
  return {(float(x) + dx) / float(m_xSamples),
          (float(y) + dy) / float(m_ySamples)};
}

float2 StratifiedSampler::getPixel2D() noexcept { return get2D(); }

}