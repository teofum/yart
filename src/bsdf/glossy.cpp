#include "glossy.hpp"
#include "luts.hpp"

namespace yart {

GlossyBSDF::GlossyBSDF(
  const float3& baseColor,
  float roughness,
  float ior,
  const float3& emission
) noexcept: m_base(baseColor),
            m_emission(emission),
            m_hasEmission(length2(emission) > 0.0f),
            m_ior(ior),
            m_roughness(roughness),
            m_microfacets(roughness) {}

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

  const float cosTheta_i = std::abs(wi.z());
  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float F = fresnelSchlickDielectric(dot(wo, wm), m_ior);
  const float D = 1.0f - F;

  return F * m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm)) +
         D * cosTheta_i * float(invPi);
}

BSDFSample GlossyBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc,
  float uc2
) const {
  const float r = (1.0f - m_ior) / (1.0f + m_ior);
  const float F0 = r * r;

  // Handle perfect specular case
  if (m_microfacets.smooth()) {
    const float F = fresnelSchlickDielectric(wo.z(), m_ior);
    const float D = 1.0f - F;

    if (uc < F) {
      float3 wi(-wo.x(), -wo.y(), wo.z());

      return {
        BSDFSample::Reflected | BSDFSample::Specular,
        float3(F / std::abs(wi.z())),
        float3(),
        wi,
        F
      };
    } else {
      float3 wi = samplers::sampleCosineHemisphere(u);
      if (wo.z() < 0) wi *= -1;

      const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
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
        std::abs(wi.z()) * float(invPi) * D
      };
    }
  }

  float3 wm = m_microfacets.sampleVisibleMicrofacet(wo, u);
  const float F = fresnelSchlickDielectric(dot(wo, wm), m_ior);
  const float D = 1.0f - F;

  const float fAvg = (1.0f + 20.0f * F0) / 21.0f;
  const float EmsAvg = lut::E_msAvg(m_roughness);
  const float Fms = fAvg * fAvg * EmsAvg / (1.0f - fAvg * (1.0f - EmsAvg));

  if (uc < F + Fms) {
    const float3 wi = reflect(wo, wm);
    const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
    if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

    const float pdf = m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm)) * F;
    const float Mss = m_microfacets.mdf(wm) * m_microfacets.g(wo, wi) /
                      (4 * cosTheta_o * cosTheta_i);

    const float Mms = (1.0f - lut::E_ms(cosTheta_o, m_roughness)) *
                      (1.0f - lut::E_ms(cosTheta_i, m_roughness)) /
                      (float(pi) * (1.0f - EmsAvg));

    return {
      BSDFSample::Reflected | BSDFSample::Glossy,
      float3(F * Mss + Mms * Fms),
      float3(),
      wi,
      pdf
    };
  } else {
    float3 wi = samplers::sampleCosineHemisphere(u);
    if (wo.z() < 0) wi *= -1;

    const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
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
      std::abs(wi.z()) * float(invPi) * D
    };
  }
}

}
