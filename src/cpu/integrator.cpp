#include "integrator.hpp"

namespace yart::cpu {

void Integrator::render() {
  if (!scene) return;

  m_rayCounter = 0;

  uint32_t left = samplingBounds.min.x(), top = samplingBounds.min.y();
  uint32_t right = samplingBounds.max.x(), bottom = samplingBounds.max.y();

  uint32_t ox = samplingOffset.x(), oy = samplingOffset.y();

  std::uniform_real_distribution<float> uniform;

  for (size_t j = top; j < bottom; j++) {
    for (size_t i = left; i < right; i++) {
      for (uint32_t s = 0; s < samples; s++) {
        Wavelengths w = Wavelengths::sampleUniform(uniform(m_rng));
        SpectrumSample sampled = sample(i + ox, j + oy, w);
        RGB rgbSampled = spectrumSampleToRGB(sampled, w, colorspace::sRGB());
        rgbSampled /= float(samples);

        if (s == 0) m_target(i, j) = float4(rgbSampled, 1.0f);
        else m_target(i, j) += float4(rgbSampled, 1.0f);
      }
    }
  }
}

}