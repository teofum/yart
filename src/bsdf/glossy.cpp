#include "glossy.hpp"

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
            m_microfacets(roughness) {}

float3 GlossyBSDF::fImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float F = fresnelDielectric(dot(wo, wm), m_ior);
  const float D = 1.0f - F;

  const float specular =
    m_microfacets.mdf(wm) * F * m_microfacets.g(wo, wi) /
    (4 * cosTheta_o * cosTheta_i);

  const float3 diffuse = m_base * float(invPi) * D;
  return float3(specular) + diffuse;
}

float GlossyBSDF::pdfImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return 0;

  const float cosTheta_i = std::abs(wi.z());
  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float F = fresnelDielectric(dot(wo, wm), m_ior);
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
  // Handle perfect specular case
  if (m_microfacets.smooth()) {
    const float F = fresnelDielectric(wo.z(), m_ior);
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

      return {
        BSDFSample::Reflected | BSDFSample::Diffuse |
        (m_hasEmission ? BSDFSample::Emitted : 0),
        m_base * float(invPi),
        m_emission,
        wi,
        std::abs(wi.z()) * float(invPi) * D
      };
    }
  }

  float3 wm = m_microfacets.sampleVisibleMicrofacet(wo, u);
  const float F = fresnelDielectric(dot(wo, wm), m_ior);
  const float D = 1.0f - F;

  if (uc < F) {
    const float3 wi = reflect(wo, wm);
    const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
    if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

    const float pdf = m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm)) * F;
    const float reflectionFactor =
      m_microfacets.mdf(wm) * F * m_microfacets.g(wo, wi) /
      (4 * cosTheta_o * cosTheta_i);

    return {
      BSDFSample::Reflected | BSDFSample::Glossy,
      float3(reflectionFactor),
      float3(),
      wi,
      pdf
    };
  } else {
    float3 wi = samplers::sampleCosineHemisphere(u);
    if (wo.z() < 0) wi *= -1;

    return {
      BSDFSample::Reflected | BSDFSample::Diffuse |
      (m_hasEmission ? BSDFSample::Emitted : 0),
      m_base * float(invPi),
      m_emission,
      wi,
      std::abs(wi.z()) * float(invPi) * D
    };
  }
}

}
