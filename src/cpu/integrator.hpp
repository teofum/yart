#ifndef YART_INTEGRATOR_HPP
#define YART_INTEGRATOR_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart::cpu {
using namespace math;

class Integrator {
public:
  ubounds2 samplingBounds; // Sampling rect as written to buffer
  uint2 samplingOffset; // Pixel offset for rays

  uint32_t samples = 1; // Total # of samples
  uint32_t sampleOffset = 0; // First sample, used to seed RNG deterministically

  const Scene* scene = nullptr;
  float3 backgroundColor;

  Integrator(Buffer& buffer, const Camera& camera, Sampler& sampler) noexcept
    : m_camera(camera), m_target(buffer), m_sampler(sampler) {}

  void render();

  [[nodiscard]] constexpr uint64_t rayCount() const noexcept {
    return m_rayCounter;
  }

protected:
  const Camera& m_camera;
  Buffer& m_target;
  Sampler& m_sampler;

  // Perf counter
  uint64_t m_rayCounter = 0;

  [[nodiscard]] virtual float3 sample(uint32_t sx, uint32_t sy) = 0;

  virtual void setup();
};

}

#endif //YART_INTEGRATOR_HPP
