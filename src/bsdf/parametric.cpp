#include "parametric.hpp"

namespace yart {

ParametricBSDF::ParametricBSDF(
  const float3& baseColor,
  float metallic,
  float roughness,
  float transmission,
  float ior,
  const float3& emission
) noexcept
  : m_diffuse(baseColor, emission),
    m_dielectric(roughness, ior),
    m_metallic(baseColor, roughness),
    m_cTrans(transmission), m_cMetallic(metallic) {}

float3 ParametricBSDF::fImpl(const float3& wo, const float3& wi) const {
  return m_cMetallic * m_metallic.fImpl(wo, wi) +
         (1.0f - m_cMetallic) * m_cTrans * m_dielectric.fImpl(wo, wi) +
         (1.0f - m_cMetallic) * (1.0f - m_cTrans) * m_diffuse.fImpl(wo, wi);
}

float ParametricBSDF::pdfImpl(const float3& wo, const float3& wi) const {
  return m_cMetallic * m_metallic.pdfImpl(wo, wi) +
         (1.0f - m_cMetallic) * m_cTrans * m_dielectric.pdfImpl(wo, wi) +
         (1.0f - m_cMetallic) * (1.0f - m_cTrans) * m_diffuse.pdfImpl(wo, wi);
}

BSDFSample ParametricBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc,
  float uc2
) const {
  BSDFSample sample;

  if (uc2 < m_cMetallic) {
    sample = m_metallic.sampleImpl(wo, u, uc, 0.0f);
  } else {
    sample = m_dielectric.sampleImpl(wo, u, uc, 0.0f);
    if (sample.is(BSDFSample::Transmitted)) {
      float pDiff = m_cMetallic + (1.0f - m_cMetallic) * (1.0f - m_cTrans);
      if (uc2 < pDiff) sample = m_diffuse.sampleImpl(wo, u, uc, 0.0f);
    }
  }

  return sample;
}


}
