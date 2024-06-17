#include "diffuse.hpp"

namespace yart {

DiffuseBSDF::DiffuseBSDF(
  const float3& reflectance,
  const float3& emissive
) noexcept: m_reflectance(reflectance),
            m_rOverPi(reflectance * invPi),
            m_emissive(emissive),
            m_hasEmission(length2(emissive) > 0.0f) {}

float3 DiffuseBSDF::fImpl(const float3& wo, const float3& wi) const {
  return m_rOverPi;
}

float DiffuseBSDF::pdf(const float3& wo, const float3& wi) const {
  if (wo.z() * wi.z() < 0) return 0;

  return std::abs(wi.z()) * float(invPi); // wi.z = cos(theta)
}

BSDFSample DiffuseBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc
) const {
  float3 wi = samplers::sampleCosineHemisphere(u);
  if (wo.z() < 0) wi *= -1;

  return {
    m_hasEmission ? Scatter::Emitted : Scatter::Reflected,
    m_rOverPi,
    m_emissive,
    wi,
    std::abs(wi.z()) * float(invPi)
  };
}

}
