#include "core.hpp"
#include "light-sampler.hpp"


namespace yart {

void LightSampler::init(const Scene* scene) noexcept {
  m_scene = scene;
}

SampledLight UniformLightSampler::sample(
  const float3& p,
  const float3& n,
  float u
) const {
  auto lightIdx = size_t(u * (float(m_scene->nLights()) - 0.01f));
  const Light& light = m_scene->light(lightIdx);
  const float pl = 1.0f / float(m_scene->nLights());

  return {light, pl};
}

float UniformLightSampler::p(
  const float3& p,
  const float3& n,
  const Light& light
) const {
  return 1.0f / float(m_scene->nLights());
}

}
