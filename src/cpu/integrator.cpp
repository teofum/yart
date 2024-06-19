#include "integrator.hpp"

namespace yart::cpu {

void Integrator::render() {
  if (!scene) return;

  m_rayCounter = 0;
  setup();

  uint32_t left = samplingBounds.min.x(), top = samplingBounds.min.y();
  uint32_t right = samplingBounds.max.x(), bottom = samplingBounds.max.y();

  uint32_t ox = samplingOffset.x(), oy = samplingOffset.y();

  for (size_t j = top; j < bottom; j++) {
    for (size_t i = left; i < right; i++) {
      for (uint32_t s = 0; s < samples; s++) {
        m_sampler.startPixelSample({i, j}, s);

        float3 sampled = sample(i + ox, j + oy) / float(samples);

        if (s == 0) m_target(i, j) = float4(sampled, 1.0f);
        else m_target(i, j) += float4(sampled, 1.0f);
      }
    }
  }
}

void Integrator::setup() {}

}