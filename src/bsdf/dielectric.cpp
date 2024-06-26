#include "dielectric.hpp"
#include "luts.hpp"

namespace yart {

DielectricBSDF::DielectricBSDF(
  float roughness,
  float ior,
  float anisotropic
) noexcept: m_ior(ior), m_roughness(roughness),
            m_microfacets(roughness, anisotropic),
            m_mfRoughened(roughen(roughness), anisotropic) {}

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

  // Single-scattering fresnel
  const float Fss = fresnelDielectric(absDot(wo, wm), ior);
  const float T = 1.0f - Fss;

  const float E_o = lut::ggxGlassE(ior, m_roughness, std::abs(cosTheta_o));

  if (isReflection) {
    // Single-scattering term
    const float Mss = m_microfacets.mdf(wm) * m_microfacets.g(wo, wi) /
                      (4 * cosTheta_o * cosTheta_i);

    return float3(Fss * Mss / E_o);
  } else {
    const float temp = dot(wi, wm) * ior + dot(wo, wm);
    const float dwm_dwi = absDot(wi, wm) * absDot(wo, wm) / (temp * temp);

    const float Tss =
      m_microfacets.mdf(wm) * m_microfacets.g(wo, wi) * dwm_dwi /
      (std::abs(cosTheta_i * cosTheta_o));

    return float3(T * Tss / E_o);
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
  float uc,
  float uc2,
  bool regularized
) const {
  const float ior = wo.z() > 0.0f ? m_ior : 1.0f / m_ior;

  // Handle perfect specular case
  if (m_microfacets.smooth()) {
    float F = fresnelDielectric(std::abs(wo.z()), ior);
    float T = 1.0f - F;

    if (uc < F) {
      float3 wi(-wo.x(), -wo.y(), wo.z());

      return {
        BSDFSample::Reflected | BSDFSample::Specular,
        float3(F / std::abs(wi.z())),
        float3(),
        wi,
        F,
        0.0f
      };
    } else {
      float3 wi;
      if (!refract(wo, axis_z<float>, m_ior, wi)) return {BSDFSample::Absorbed};

      return {
        BSDFSample::Transmitted | BSDFSample::Specular,
        float3(T / std::abs(wi.z())),
        float3(),
        wi,
        T,
        0.0f
      };
    }
  }

  const GGX& mfd = regularized ? m_mfRoughened : m_microfacets;

  float3 wm = mfd.sampleVisibleMicrofacet(wo, u);
  const float Fss = fresnelDielectric(absDot(wo, wm), ior);

  const float cosTheta_o = std::abs(wo.z());
  const float E_o = lut::ggxGlassE(ior, m_roughness, cosTheta_o);

  if (uc < Fss) {
    const float3 wi = reflect(wo, wm);
    if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

    const float cosTheta_i = std::abs(wi.z());
    const float Mss = mfd.mdf(wm) * mfd.g(wo, wi) /
                      (4 * cosTheta_o * cosTheta_i);

    const float pdf = mfd.vmdf(wo, wm) / (4 * absDot(wo, wm)) * Fss;

    return {
      BSDFSample::Reflected | BSDFSample::Glossy,
      float3(Fss * Mss / E_o),
      float3(),
      wi,
      pdf,
      m_roughness
    };
  } else {
    float3 wi;
    const bool tir = !refract(wo, wm, m_ior, wi);
    if (tir || wo.z() * wi.z() > 0.0f || wi.z() == 0.0f)
      return {BSDFSample::Absorbed};

    const float temp = dot(wi, wm) * ior + dot(wo, wm);
    const float dwm_dwi = absDot(wi, wm) / (temp * temp);
    const float pdf = mfd.vmdf(wo, wm) * dwm_dwi * (1.0f - Fss);

    const float Tss = mfd.mdf(wm) * mfd.g(wo, wi) * std::abs(
      dot(wi, wm) * dot(wo, wm) / (wi.z() * wo.z() * temp * temp)
    );

    return {
      BSDFSample::Transmitted | BSDFSample::Glossy,
      float3((1.0f - Fss) * Tss / E_o),
      float3(),
      wi,
      pdf,
      m_roughness
    };
  }
}

}
