#ifndef YART_METAL_HPP
#define YART_METAL_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {
using namespace math;

class MetalBSDF : public BSDF {
public:
  explicit MetalBSDF(
    const float3& reflectance,
    float roughness = 0.0f,
    float ior = 1.5f
  ) noexcept;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return nullptr;
  }

private:
  // Magic fresnel constant
  static constexpr const float kFresnel = 10.0f;

  float3 m_reflectance;
  float m_ior;
  GGX m_microfacets;

  [[nodiscard]] float3 fImpl(const float3& wo, const float3& wi) const override;

  [[nodiscard]] float pdfImpl(
    const float3& wo,
    const float3& wi
  ) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc
  ) const override;
};

}

#endif //YART_METAL_HPP
