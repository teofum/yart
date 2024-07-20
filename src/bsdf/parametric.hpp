#ifndef YART_PARAMETRIC_HPP
#define YART_PARAMETRIC_HPP

#include <math/math.hpp>
#include <core/core.hpp>

namespace yart {
using namespace math;

/**
 * Parametric PBR material that allows texture-driven control over most properties.
 * Loosely based on Enterprise PBR, OpenPBR and Blender's Principled BSDF.
 */
class ParametricBSDF : public BSDF {
public:
  explicit ParametricBSDF(
    const float3& baseColor,
    const RGBATexture* baseTexture = nullptr,
    const SDRTexture<2>* mrTexture = nullptr,
    const MonoTexture* transmissionTexture = nullptr,
    const RGBTexture* normalTexture = nullptr,
    const MonoTexture* clearcoatTexture = nullptr,
    const RGBTexture* emissionTexture = nullptr,
    float metallic = 0.0f,
    float roughness = 0.0f,
    float transmission = 0.0f,
    float ior = 1.5f,
    float anisotropic = 0.0f,
    float anisoRotation = 0.0f,
    float clearcoat = 0.0f,
    float clearcoatRoughness = 0.0f,
    const float3& emission = float3(),
    float normalScale = 1.0f,
    bool thinTransmission = false
  ) noexcept;

  [[nodiscard]] float alpha(const float2& uv) const override;

  [[nodiscard]] float3 base(const float2& uv) const override;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return m_hasEmission > 0.0f ? &m_emission : nullptr;
  }

  [[nodiscard]] bool transparent() const override;

private:
  // Lobe weights
  float m_cTrans, m_cMetallic;

  // Common BSDF prameters
  float3 m_base, m_emission;
  float m_ior, m_roughness, m_anisotropic;
  float m_clearcoat, m_clearcoatRoughness;
  bool m_thinTransmission, m_hasAlpha = false, m_hasEmission = false;

  // Textures
  const RGBATexture* m_baseTexture;         // Base color
  const SDRTexture<2>* m_mrTexture;         // Metallic + roughness
  const MonoTexture* m_transmissionTexture; // Transmission
  const MonoTexture* m_clearcoatTexture;    // Clearcoat
  const RGBTexture* m_emissionTexture;      // Emission

  // Anisotropy data
  float3x3 m_localRotation, m_invRotation;

  // BSDF implementation
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

  // Functions for different material lobes
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
    const float3& emission,
    const GGX& mf,
    const float2& u,
    float uc
  ) const;

  [[nodiscard]] float3 fClearcoat(
    const float3& wo,
    const float3& wi,
    const GGX& mf,
    float* Fc
  ) const;

  [[nodiscard]] float pdfClearcoat(
    const float3& wo,
    const float3& wi,
    const GGX& mf,
    float* Fc
  ) const;

  [[nodiscard]] BSDFSample sampleClearcoat(
    const float3& wo,
    const GGX& mf,
    const float2& u,
    float uc
  ) const;
};

}

#endif //YART_PARAMETRIC_HPP
