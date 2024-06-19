#ifndef YART_MIS_INTEGRATOR_HPP
#define YART_MIS_INTEGRATOR_HPP

#include <core/core.hpp>
#include <math/math.hpp>

#include "ray-integrator.hpp"

namespace yart::cpu {

class MISIntegrator : public RayIntegrator {
public:
  MISIntegrator(
    Buffer& buffer,
    const Camera& camera,
    Sampler& sampler
  ) noexcept;

private:
  UniformLightSampler m_lightSampler;

  void setup() override;

  [[nodiscard]] float3 Li(const Ray& ray) override;

  [[nodiscard]] float3 Ld(const float3& wo, const Hit& hit);

  [[nodiscard]] float lightPdf(
    const float3& p,
    const float3& wi,
    const Light& light
  ) const;

  [[nodiscard]] bool unoccluded(const float3& from, const float3& to) const;
};

}
#endif //YART_MIS_INTEGRATOR_HPP
