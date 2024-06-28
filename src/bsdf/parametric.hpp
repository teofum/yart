#ifndef YART_PARAMETRIC_HPP
#define YART_PARAMETRIC_HPP

#include <math/math.hpp>
#include <core/core.hpp>

namespace yart {
using namespace math;

class ParametricBSDF : public BSDF {
public:
  explicit ParametricBSDF(
    const float3& baseColor,
    const Texture* baseTexture = nullptr,
    const Texture* mrTexture = nullptr,
    const Texture* transmissionTexture = nullptr,
    float metallic = 0.0f,
    float roughness = 0.0f,
    float transmission = 0.0f,
    float ior = 1.5f,
    float anisotropic = 0.0f,
    float anisoRotation = 0.0f,
    const float3& emission = float3()
  ) noexcept;

  [[nodiscard]] float alpha(const float2& uv) const override;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return length2(m_emission) > 0.0f ? &m_emission : nullptr;
  }

private:
  // Lobe weights
  float m_cTrans, m_cMetallic;

  // Common BSDF prameters
  float3 m_base, m_emission;
  float m_ior, m_roughness, m_anisotropic;

  // Textures
  const Texture* m_baseTexture;         // Base color
  const Texture* m_mrTexture;           // Metallic + roughness
  const Texture* m_transmissionTexture; // Transmission

  // Anisotropy data
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

  [[nodiscard]] float3 fMetallic(
    const float3& wo,
    const float3& wi,
    const float3& base,
    const GGX& mf
  ) const;

  [[nodiscard]] float pdfMetallic(
    const float3& wo,
    const float3& wi,
    const GGX& mf
  ) const;

  [[nodiscard]] BSDFSample sampleMetallic(
    const float3& wo,
    const float3& base,
    const GGX& mf,
    const float2& u,
    float uc
  ) const;

  [[nodiscard]] float3 fDielectric(
    const float3& wo,
    const float3& wi,
    const float3& base,
    const GGX& microfacets
  ) const;

  [[nodiscard]] float pdfDielectric(
    const float3& wo,
    const float3& wi,
    const GGX& mf
  ) const;

  [[nodiscard]] BSDFSample sampleDielectric(
    const float3& wo,
    const float3& base,
    const GGX& mf,
    const float2& u,
    float uc
  ) const;

  [[nodiscard]] float3 fGlossy(
    const float3& wo,
    const float3& wi,
    const float3& base,
    const GGX& mf
  ) const;

  [[nodiscard]] float pdfGlossy(
    const float3& wo,
    const float3& wi,
    const GGX& mf
  ) const;

  [[nodiscard]] BSDFSample sampleGlossy(
    const float3& wo,
    const float3& base,
    const GGX& mf,
    const float2& u,
    float uc
  ) const;
};

}

#endif //YART_PARAMETRIC_HPP
