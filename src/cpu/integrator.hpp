#ifndef YART_INTEGRATOR_HPP
#define YART_INTEGRATOR_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart::cpu {
using namespace math;

class Integrator {
public:
  ubounds2 samplingBounds;
  uint2 samplingOffset;
  uint32_t samples = 1;
  const Scene* scene = nullptr;

  Integrator(Buffer& buffer, const Camera& camera) noexcept
    : m_camera(camera), m_target(buffer) {}

  void render();

  [[nodiscard]] constexpr uint64_t rayCount() const noexcept {
    return m_rayCounter;
  }

protected:
  const Camera& m_camera;
  Buffer& m_target;
  Xoshiro::Xoshiro256PP m_rng;

  // Perf counter
  uint64_t m_rayCounter = 0;

  [[nodiscard]] virtual SpectrumSample sample(
    uint32_t sx,
    uint32_t sy,
    Wavelengths& w
  ) = 0;
};

}

#endif //YART_INTEGRATOR_HPP
