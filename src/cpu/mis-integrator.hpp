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
  PowerLightSampler m_lightSampler;
  std::vector<const Light*> m_infiniteLights;

  void setup() override;

  [[nodiscard]] float3 Li(const Ray& ray) override;

  [[nodiscard]] float3 Ld(const float3& wo, const Hit& hit);

  [[nodiscard]] bool unoccluded(const float3& from, const float3& to) const;
};

}
#endif //YART_MIS_INTEGRATOR_HPP
