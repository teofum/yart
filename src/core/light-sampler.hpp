#ifndef YART_LIGHT_SAMPLER_HPP
#define YART_LIGHT_SAMPLER_HPP

#include "light.hpp"

namespace yart {

struct SampledLight {
  const Light& light;
  float p;
};

class LightSampler {
public:
  virtual void init(const Scene* scene) noexcept;

  [[nodiscard]] virtual SampledLight sample(
    const float3& p,
    const float3& n,
    float u
  ) const = 0;

  [[nodiscard]] virtual float p(
    const float3& p,
    const float3& n,
    const Light& light
  ) const = 0;

protected:
  const Scene* m_scene = nullptr;
};

class UniformLightSampler : public LightSampler {
public:
  [[nodiscard]] SampledLight sample(
    const float3& p,
    const float3& n,
    float u
  ) const override;

  [[nodiscard]] float p(
    const float3& p,
    const float3& n,
    const Light& light
  ) const override;
};

class PowerLightSampler : public LightSampler {
public:
  [[nodiscard]] SampledLight sample(
    const float3& p,
    const float3& n,
    float u
  ) const override;

  [[nodiscard]] float p(
    const float3& p,
    const float3& n,
    const Light& light
  ) const override;

private:
  
};

}

#endif //YART_LIGHT_SAMPLER_HPP
