#ifndef YART_LIGHT_SAMPLER_HPP
#define YART_LIGHT_SAMPLER_HPP

#include "light.hpp"

namespace yart {

struct SampledLight {
  const Light& light;
  float p;
};

/**
 * Base light sampler. Provides methods to sample a single light from the scene.
 */
class LightSampler {
public:
  /**
   * Initialize the sampler, should be called once at render start.
   */
  virtual void init(const Scene* scene) noexcept;

  /**
   * Sample a light from a point in a surface.
   * @param p Surface position
   * @param n Surface normal
   * @param u Sampled 1D value
   */
  [[nodiscard]] virtual SampledLight sample(
    const float3& p,
    const float3& n,
    float u
  ) const = 0;

  /**
   * Get the probability of sampling a given light for MIS
   * @param p Surface position
   * @param n Surface normal
   * @param lightIdx Light index
   */
  [[nodiscard]] virtual float p(
    const float3& p,
    const float3& n,
    size_t lightIdx
  ) const = 0;

protected:
  const Scene* m_scene = nullptr;
};

/**
 * Samples lights with a uniform probability.
 */
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
    size_t lightIdx
  ) const override;
};

/**
 * Samples lights with a probability based on total emission power. Better
 * convergence than uniform sampling.
 */
class PowerLightSampler : public LightSampler {
public:
  void init(const Scene* scene) noexcept override;

  [[nodiscard]] SampledLight sample(
    const float3& p,
    const float3& n,
    float u
  ) const override;

  [[nodiscard]] float p(
    const float3& p,
    const float3& n,
    size_t lightIdx
  ) const override;

private:
  float m_totalPower;
  std::vector<float> m_lightPowers;
  std::vector<const Light*> m_lights, m_infiniteLights;
};

}

#endif //YART_LIGHT_SAMPLER_HPP
