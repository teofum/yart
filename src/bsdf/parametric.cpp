#include "parametric.hpp"

namespace yart {

ParametricBSDF::ParametricBSDF(
  const float3& baseColor,
  float metallic,
  float roughness,
  float transmission,
  float ior,
  float anisotropic,
  float anisoRotation,
  const float3& emission
) noexcept
  : m_glossy(baseColor, roughness, ior, anisotropic, emission),
    m_dielectric(roughness, ior, anisotropic),
    m_metallic(baseColor, roughness, anisotropic, anisoRotation, ior),
    m_cTrans(transmission), m_cMetallic(metallic) {}

float3 ParametricBSDF::fImpl(const float3& wo, const float3& wi) const {
  const float cMetallic = m_cMetallic;
  const float cDielectric = (1.0f - m_cMetallic) * m_cTrans;
  const float cGlossy = (1.0f - m_cMetallic) * (1.0f - m_cTrans);

  float3 val;
  if (cMetallic > 0.0f) val += cMetallic * m_metallic.fImpl(wo, wi);
  if (cDielectric > 0.0f) val += cDielectric * m_dielectric.fImpl(wo, wi);
  if (cGlossy > 0.0f) val += cGlossy * m_glossy.fImpl(wo, wi);

  return val;
}

float ParametricBSDF::pdfImpl(const float3& wo, const float3& wi) const {
  const float pMetallic = m_cMetallic;
  const float pDielectric = (1.0f - m_cMetallic) * m_cTrans;
  const float pGlossy = (1.0f - m_cMetallic) * (1.0f - m_cTrans);

  float pdf = 0.0f;
  if (pMetallic > 0.0f) pdf += pMetallic * m_metallic.pdfImpl(wo, wi);
  if (pDielectric > 0.0f) pdf += pDielectric * m_dielectric.pdfImpl(wo, wi);
  if (pGlossy > 0.0f) pdf += pGlossy * m_glossy.pdfImpl(wo, wi);

  return pdf;
}

BSDFSample ParametricBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc,
  float uc2,
  bool regularized
) const {
  BSDFSample sample;
  const float pMetallic = m_cMetallic;
  const float pDielectric = m_cMetallic + (1.0f - m_cMetallic) * m_cTrans;

  if (uc2 < pMetallic) {
    sample = m_metallic.sampleImpl(wo, u, uc, 0.0f, regularized);
  } else if (uc2 < pDielectric) {
    sample = m_dielectric.sampleImpl(wo, u, uc, 0.0f, regularized);
  } else {
    sample = m_glossy.sampleImpl(wo, u, uc, 0.0f, regularized);
  }

  return sample;
}


}
