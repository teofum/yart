#ifndef YART_GLOSSY_HPP
#define YART_GLOSSY_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {
using namespace math;

class GlossyBSDF : public BSDF {
public:
  explicit GlossyBSDF(
    const float3& baseColor,
    float roughness = 0.0f,
    float ior = 1.5f,
    float anisotropic = 0.0f,
    const float3& emission = float3()
  ) noexcept;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return m_hasEmission ? &m_emission : nullptr;
  }

  friend class ParametricBSDF;

private:
  float3 m_base, m_emission;
  bool m_hasEmission;
  float m_ior, m_roughness;
  GGX m_microfacets, m_mfRoughened;

  [[nodiscard]] float3 fImpl(const float3& wo, const float3& wi) const override;

  [[nodiscard]] float pdfImpl(
    const float3& wo,
    const float3& wi
  ) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc,
    float uc2,
    bool regularized
  ) const override;
};

}

#endif //YART_GLOSSY_HPP
