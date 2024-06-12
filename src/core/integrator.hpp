#ifndef YART_INTEGRATOR_HPP
#define YART_INTEGRATOR_HPP

namespace yart {

class Integrator {
public:
  uint2 samplingOffset;

  Integrator(Buffer& buffer, const Camera& camera) noexcept
    : m_camera(camera), m_target(buffer) {}

  virtual void render(const Node& node) = 0;

  [[nodiscard]] constexpr uint64_t rayCount() const noexcept {
    return m_rayCounter;
  }

protected:
  const Camera& m_camera;
  Buffer& m_target;
  Xoshiro::Xoshiro256PP m_rng;

  // Perf counter
  uint64_t m_rayCounter = 0;
};

}

#endif //YART_INTEGRATOR_HPP
