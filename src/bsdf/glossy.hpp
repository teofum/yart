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
    const Texture<float3>* baseTexture = nullptr,
    float roughness = 0.0f,
    float ior = 1.5f,
    float anisotropic = 0.0f,
    float anisoRotation = 0.0f,
    const float3& emission = float3()
  ) noexcept;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return length2(m_emission) > 0.0f ? &m_emission : nullptr;
  }

  friend class ParametricBSDF;

private:
  float3 m_base, m_emission;
  float m_ior, m_roughness;

  const Texture<float3>* m_baseTexture;

  GGX m_microfacets, m_mfRoughened;
  float3x3 m_localRotation, m_invRotation;

  [[nodiscard]] float3 fImpl(
    const float3& wo,
    const float3& wi,
    const float2& uv
  ) const override;

  [[nodiscard]] float pdfImpl(
    const float3& wo,
    const float3& wi,
    const float2& uv
  ) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& uv,
    const float2& u,
    float uc,
    float uc2,
    bool regularized
  ) const override;
};

}

#endif //YART_GLOSSY_HPP
