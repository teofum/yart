#include "glossy.hpp"
#include "luts.hpp"

namespace yart {

GlossyBSDF::GlossyBSDF(
  const float3& baseColor,
  float roughness,
  float ior,
  float anisotropic,
  const float3& emission
) noexcept: m_base(baseColor),
            m_emission(emission),
            m_hasEmission(length2(emission) > 0.0f),
            m_ior(ior),
            m_roughness(roughness),
            m_microfacets(roughness, anisotropic),
            m_mfRoughened(
              max(roughness, std::clamp(roughness * 2.0f, 0.1f, 0.3f)),
              anisotropic
            ) {}

float3 GlossyBSDF::fImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Dielectric single scattering component
  const float Fss = fresnelSchlickDielectric(dot(wo, wm), m_ior);
  const float Mss = m_microfacets.mdf(wm) * m_microfacets.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  // Dielectric multiscatter component
  const float r = (1.0f - m_ior) / (1.0f + m_ior);
  const float F0 = r * r;
  const float fAvg = (1.0f + 20.0f * F0) / 21.0f;
  const float EmsAvg = lut::E_msAvg(m_roughness);
  const float Mms = (1.0f - lut::E_ms(cosTheta_o, m_roughness)) *
                    (1.0f - lut::E_ms(cosTheta_i, m_roughness)) /
                    (float(pi) * (1.0f - EmsAvg));
  const float Fms = fAvg * fAvg * EmsAvg / (1.0f - fAvg * (1.0f - EmsAvg));

  // Diffuse component
  const float cDiffuse =
    (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_o)) *
    (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_i)) /
    (float(pi) * (1.0f - lut::Eb_msAvg(F0, m_roughness)));
  const float3 diffuse = m_base * cDiffuse;

  // Final BSDF
  return float3(Fss * Mss + Mms * Fms) + diffuse;
}

float GlossyBSDF::pdfImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return 0;

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float Fss = fresnelSchlickDielectric(dot(wo, wm), m_ior);

  const float r = (1.0f - m_ior) / (1.0f + m_ior);
  const float F0 = r * r;
  const float fAvg = (1.0f + 20.0f * F0) / 21.0f;
  const float EmsAvg = lut::E_msAvg(m_roughness);
  const float Fms = fAvg * fAvg * EmsAvg / (1.0f - fAvg * (1.0f - EmsAvg));
  const float FAvg = (m_ior - 1.0f) / (4.08567f + 1.00071f * m_ior);
  const float Ems_o = lut::E_ms(cosTheta_o, m_roughness);
  const float kappa = 1.0f - (FAvg * Ems_o + Fms * (1.0f - Ems_o));

  return (Fss + Fms) * m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm)) +
         cosTheta_i * kappa;
}

BSDFSample GlossyBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc,
  float uc2,
  bool regularized
) const {
  const float r = (1.0f - m_ior) / (1.0f + m_ior);
  const float F0 = r * r;

  const float cosTheta_o = wo.z();

  const float fAvg = (1.0f + 20.0f * F0) / 21.0f;
  const float EmsAvg = lut::E_msAvg(m_roughness);
  const float Fms = fAvg * fAvg * EmsAvg / (1.0f - fAvg * (1.0f - EmsAvg));

  const float FAvg = (m_ior - 1.0f) / (4.08567f + 1.00071f * m_ior);
  const float Ems_o = lut::E_ms(cosTheta_o, m_roughness);
  const float kappa = 1.0f - (FAvg * Ems_o + Fms * (1.0f - Ems_o));

  // Diffuse scattering
  if (uc < kappa) {
    float3 wi = samplers::sampleCosineHemisphere(u);
    if (wo.z() < 0) wi *= -1;

    const float cosTheta_i = wi.z();
    const float cDiffuse =
      (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_o)) *
      (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_i)) /
      (float(pi) * (1.0f - lut::Eb_msAvg(F0, m_roughness)));

    return {
      BSDFSample::Reflected | BSDFSample::Diffuse |
      (m_hasEmission ? BSDFSample::Emitted : 0),
      m_base * cDiffuse,
      m_emission,
      wi,
      std::abs(wi.z()) * cDiffuse,
      1.0f
    };
  }

  // Handle perfect specular case
  if (m_microfacets.smooth()) {
    const float F = fresnelSchlickDielectric(wo.z(), m_ior);
    float3 wi(-wo.x(), -wo.y(), wo.z());

    return {
      BSDFSample::Reflected | BSDFSample::Specular,
      float3(F / std::abs(wi.z())),
      float3(),
      wi,
      F,
      0.0f
    };
  }

  const GGX& mfd = regularized ? m_mfRoughened : m_microfacets;

  // Rough (glossy) reflection
  float3 wm = mfd.sampleVisibleMicrofacet(wo, u);
  const float3 wi = reflect(wo, wm);
  const float cosTheta_i = wi.z();
  if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

  const float Fss = fresnelSchlickDielectric(dot(wo, wm), m_ior);
  const float Mss = mfd.mdf(wm) * mfd.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  const float Mms = (1.0f - Ems_o) *
                    (1.0f - lut::E_ms(cosTheta_i, m_roughness)) /
                    (float(pi) * (1.0f - EmsAvg));

  const float pdf = mfd.vmdf(wo, wm) / (4 * absDot(wo, wm)) * Fss;

  return {
    BSDFSample::Reflected | BSDFSample::Glossy,
    float3(Fss * Mss + Mms * Fms),
    float3(),
    wi,
    pdf,
    m_roughness
  };
}

}
