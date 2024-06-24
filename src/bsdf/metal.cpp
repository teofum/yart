#include "metal.hpp"
#include "luts.hpp"

namespace yart {

MetalBSDF::MetalBSDF(
  const float3& reflectance,
  float roughness,
  float anisotropic,
  float ior
) noexcept
  : m_baseColor(reflectance), m_ior(ior), m_roughness(roughness),
    m_microfacets(roughness, anisotropic),
    m_mfRoughened(
      max(roughness, std::clamp(roughness * 2.0f, 0.1f, 0.3f)),
      anisotropic
    ) {}

float3 MetalBSDF::fImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float3 Fss = fresnelSchlick(m_baseColor, absDot(wo, wm));
  const float3 fss = Fss * m_microfacets.mdf(wm) * m_microfacets.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  const float Ess = lut::E_ms(cosTheta_o, m_roughness);
  const float3 fms = fss * m_baseColor * (1.0f - Ess) / Ess;

  return fss + fms;
}

float MetalBSDF::pdfImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return 0;

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  return m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm));
}

BSDFSample MetalBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc,
  float uc2,
  bool regularized
) const {
  if (m_microfacets.smooth()) {
    const float3 F = fresnelSchlick(m_baseColor, wo.z());

    return {
      BSDFSample::Reflected | BSDFSample::Specular,
      F / std::abs(wo.z()),
      float3(),
      float3(-wo.x(), -wo.y(), wo.z()),
      1.0f,
      0.0f
    };
  }

  const GGX& mfd = regularized ? m_mfRoughened : m_microfacets;

  float3 wm = mfd.sampleVisibleMicrofacet(wo, u);
  float3 wi = reflect(wo, wm);
  if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

  const float pdf = mfd.vmdf(wo, wm) / (4 * absDot(wo, wm));

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  const float3 Fss = fresnelSchlick(m_baseColor, absDot(wo, wm));
  const float3 Mss = mfd.mdf(wm) * Fss * mfd.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  const float Ess = lut::E_ms(cosTheta_o, m_roughness);
  const float3 Mms = Mss * m_baseColor * (1.0f - Ess) / Ess;

  return {
    BSDFSample::Reflected | BSDFSample::Glossy,
    Mss + Mms,
    float3(),
    wi,
    pdf,
    m_roughness
  };
}

}
