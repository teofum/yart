#ifndef YART_NAIVE_INTEGRATOR_HPP
#define YART_NAIVE_INTEGRATOR_HPP

#include <core/core.hpp>
#include <math/math.hpp>

#include "ray-integrator.hpp"

namespace yart::cpu {

class NaiveIntegrator : public RayIntegrator {
public:
  NaiveIntegrator(Buffer& buffer, const Camera& camera) noexcept;

private:
  [[nodiscard]] SpectrumSample Li(const Ray& ray) override;

  [[nodiscard]] SpectrumSample LiImpl(
    const Ray& ray,
    const Node& root,
    uint32_t depth = 0
  );

  [[nodiscard]] ScatterResult scatter(const Ray& ray, const Hit& hit);
};

}

#endif //YART_NAIVE_INTEGRATOR_HPP
