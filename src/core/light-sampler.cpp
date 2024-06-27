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
  size_t lightIdx
) const {
  return 1.0f / float(m_scene->nLights());
}

void PowerLightSampler::init(const Scene* scene) noexcept {
  LightSampler::init(scene);

  m_lightPowers.clear();
  m_totalPower = 0.0f;
  for (const Light& light: scene->lights()) {
    m_lightPowers.push_back(m_totalPower + light.power());
    m_totalPower += light.power();
  }
}

SampledLight PowerLightSampler::sample(
  const float3& p,
  const float3& n,
  float u
) const {
  u *= m_totalPower;
  int64_t i = findFirst(
    int64_t(m_lightPowers.size()),
    [&](int64_t i) { return m_lightPowers[i] < u; }
  );

  float pl = m_scene->light(i).power() / m_totalPower;
  return {m_scene->light(i), pl};
}

float PowerLightSampler::p(
  const float3& p,
  const float3& n,
  size_t lightIdx
) const {
  return m_scene->light(lightIdx).power() / m_totalPower;
}

}
