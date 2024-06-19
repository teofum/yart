#include "dielectric.hpp"

namespace yart {

DielectricBSDF::DielectricBSDF(float roughness, float ior) noexcept
  : m_ior(ior), m_microfacets(roughness * roughness) {}

float3 DielectricBSDF::fImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return {};

  const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
  const bool isReflection = cosTheta_o * cosTheta_i > 0.0f;
  float ior = 1.0f;
  if (!isReflection) ior = cosTheta_o > 0.0f ? m_ior : 1.0f / m_ior;

  float3 wm = ior * wi + wo;
  if (cosTheta_i == 0.0f || cosTheta_o == 0.0f || length2(wm) == 0.0f)
    return {};

  wm = normalized(wm.z() < 0.0f ? -wm : wm);
  if (dot(wm, wi) * cosTheta_i < 0.0f || dot(wm, wo) * cosTheta_o < 0.0f)
    return {}; // Discard back-facing microsurfaces

  const float F = fresnelDielectric(dot(wo, wm), m_ior);
  const float T = 1.0f - F;

  if (isReflection) {
    const float reflected =
      m_microfacets.mdf(wm) * F * m_microfacets.g(wo, wi) /
      (4 * cosTheta_o * cosTheta_i);
    return float3(reflected);
  } else {
    const float temp = dot(wi, wm) + dot(wo, wm) / ior;
    const float dwm_dwi = absDot(wi, wm) * absDot(wo, wm) / (temp * temp);

    const float transmitted =
      m_microfacets.mdf(wm) * T * m_microfacets.g(wo, wi) * dwm_dwi /
      (std::abs(cosTheta_i * cosTheta_o));

    return float3(transmitted);
  }
}

float DielectricBSDF::pdfImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return 0;

  const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
  const bool isReflection = cosTheta_o * cosTheta_i > 0.0f;
  float ior = 1.0f;
  if (!isReflection) ior = cosTheta_o > 0.0f ? m_ior : 1.0f / m_ior;

  float3 wm = ior * wi + wo;
  if (cosTheta_i == 0.0f || cosTheta_o == 0.0f || length2(wm) == 0.0f) return 0;

  wm = normalized(wm.z() < 0.0f ? -wm : wm);
  if (dot(wm, wi) * cosTheta_i < 0.0f || dot(wm, wo) * cosTheta_o < 0.0f)
    return 0; // Discard back-facing microsurfaces

  const float F = fresnelDielectric(dot(wo, wm), m_ior);
  const float T = 1.0f - F;

  float pdf;
  if (isReflection) {
    pdf = m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm)) * F;
  } else {
    const float temp = dot(wi, wm) + dot(wo, wm) / ior;
    const float dwm_dwi = absDot(wo, wm) / (temp * temp);
    pdf = m_microfacets.vmdf(wo, wm) * dwm_dwi * T;
  }
  return pdf;
}

BSDFSample DielectricBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc
) const {
  // Handle perfect specular case
  if (m_microfacets.smooth()) {
    float F = fresnelDielectric(wo.z(), m_ior);
    float T = 1.0f - F;

    if (uc < F) {
      float3 wi(-wo.x(), -wo.y(), wo.z());

      return {
        Scatter::Reflected,
        float3(F / std::abs(wi.z())),
        float3(),
        wi,
        F
      };
    } else {
      float3 wi;
      if (!refract(wo, axis_z<float>, m_ior, wi)) return {Scatter::Absorbed};

      return {
        Scatter::Transmitted,
        float3(T / std::abs(wi.z())),
        float3(),
        wi,
        T
      };
    }
  }

  float3 wm = m_microfacets.sampleVisibleMicrofacet(wo, u);
  const float F = fresnelDielectric(dot(wo, wm), m_ior);
  const float T = 1.0f - F;

  if (uc < F) {
    const float3 wi = reflect(wo, wm);
    const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
    if (wo.z() * wi.z() < 0.0f) return {Scatter::Absorbed};

    const float pdf = m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm)) * F;
    const float reflectionFactor =
      m_microfacets.mdf(wm) * F * m_microfacets.g(wo, wi) /
      (4 * cosTheta_o * cosTheta_i);

    return {
      Scatter::Reflected,
      float3(reflectionFactor),
      float3(),
      wi,
      pdf
    };
  } else {
    float3 wi;
    const bool tir = !refract(wo, wm, m_ior, wi);
    if (tir || wo.z() * wi.z() > 0.0f || wi.z() == 0.0f)
      return {Scatter::Absorbed};

    const float ior = wo.z() > 0.0f ? m_ior : 1.0f / m_ior;
    const float temp = dot(wi, wm) + dot(wo, wm) / ior;
    const float dwm_dwi = absDot(wi, wm) / (temp * temp);
    const float pdf = m_microfacets.vmdf(wo, wm) * dwm_dwi * T;
    const float transmissionFactor =
      m_microfacets.mdf(wm) * T * m_microfacets.g(wo, wi) *
      absDot(wi, wm) * absDot(wo, wm) / (wi.z() * wo.z() * temp * temp);

    return {
      Scatter::Transmitted,
      float3(transmissionFactor),
      float3(),
      wi,
      pdf
    };
  }
}

}
