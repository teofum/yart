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
  size_t nl = m_scene->nLights();
  size_t lightIdx = min(size_t(u * float(nl) - 0.01f), nl - 1);
  const Light& light = m_scene->light(lightIdx);
  const float pl = 1.0f / float(nl);

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
  m_lights.clear();
  m_infiniteLights.clear();
  m_totalPower = 0.0f;

  // Separate lists for infinite and area lights
  for (const Light& light: scene->lights()) {
    if (light.type() == Light::Type::Infinite) {
      m_infiniteLights.push_back(&light);
    } else {
      m_lights.push_back(&light);
      m_lightPowers.push_back(m_totalPower + light.power());
      m_totalPower += light.power();
    }
  }
}

SampledLight PowerLightSampler::sample(
  const float3& p,
  const float3& n,
  float u
) const {
  // Handle infinite light sampling, makes sure environment and area lights
  // are both sufficiently sampled
  size_t infCount = m_infiniteLights.size();
  float pInfinite = m_lights.empty() ? 1.0f
                                     : float(infCount) / float(infCount + 1);

  if (u < pInfinite) {
    u /= pInfinite;
    size_t idx = min(infCount - 1, size_t(u * infCount));
    return {*m_infiniteLights[idx], pInfinite / float(infCount)};
  }

  u = (u - pInfinite) / (1.0f - pInfinite);
  u *= m_totalPower;
  int64_t i = findFirst(
    int64_t(m_lightPowers.size()),
    [&](int64_t i) { return m_lightPowers[i] < u; }
  );

  float pl = m_lights[i]->power() / m_totalPower * (1.0f - pInfinite);
  return {*m_lights[i], pl};
}

float PowerLightSampler::p(
  const float3& p,
  const float3& n,
  size_t lightIdx
) const {
  size_t infCount = m_infiniteLights.size();
  float pInfinite = m_lights.empty() ? 1.0f
                                     : float(infCount) / float(infCount + 1);

  const Light& light = m_scene->light(lightIdx);
  if (light.type() == Light::Type::Infinite)
    return pInfinite / float(infCount);
  return light.power() / m_totalPower * (1.0f - pInfinite);
}

}
