#include "metal.hpp"

namespace yart {

MetalBSDF::MetalBSDF(
  const float3& reflectance,
  float roughness,
  float ior
) noexcept
  : m_reflectance(reflectance), m_ior(ior),
    m_microfacets(roughness * roughness) {}

float3 MetalBSDF::fImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float r = fresnelComplex(absDot(wo, wm), m_ior, kFresnel);
  const float reflectionFactor =
    m_microfacets.mdf(wm) * r * m_microfacets.g(wo, wi) /
    (4 * cosTheta_o * cosTheta_i);

  return m_reflectance * reflectionFactor;
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
  float uc
) const {
  if (m_microfacets.smooth()) {
    const float r = fresnelComplex(wo.z(), m_ior, kFresnel);

    return {
      Scatter::Reflected,
      float3(m_reflectance * r / std::abs(wo.z())),
      float3(),
      float3(-wo.x(), -wo.y(), wo.z()),
      r
    };
  }

  float3 wm = m_microfacets.sampleVisibleMicrofacet(wo, u);
  float3 wi = reflect(wo, wm);
  if (wo.z() * wi.z() < 0.0f) return {Scatter::Absorbed};

  const float pdf = m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm));

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  const float r = fresnelComplex(absDot(wo, wm), m_ior, kFresnel);
  const float reflectionFactor =
    m_microfacets.mdf(wm) * r * m_microfacets.g(wo, wi) /
    (4 * cosTheta_o * cosTheta_i);

  return {
    Scatter::Reflected,
    m_reflectance * reflectionFactor,
    float3(),
    wi,
    pdf
  };
}

}