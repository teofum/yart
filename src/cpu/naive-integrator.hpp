#ifndef YART_NAIVE_INTEGRATOR_HPP
#define YART_NAIVE_INTEGRATOR_HPP

#include <core/core.hpp>
#include <math/math.hpp>

#include "ray-integrator.hpp"

namespace yart::cpu {

class NaiveIntegrator : public RayIntegrator {
public:
  NaiveIntegrator(
    Buffer& buffer,
    const Camera& camera,
    Sampler& sampler
  ) noexcept;

private:
  [[nodiscard]] float3 Li(const Ray& ray) override;

  [[nodiscard]] float3 LiImpl(
    const Ray& ray,
    const Node& root,
    uint32_t depth = 0
  );

  [[nodiscard]] ScatterResult scatter(const Ray& ray, const Hit& hit);
};

}

#endif //YART_NAIVE_INTEGRATOR_HPP
