#include "bsdf.hpp"

namespace yart {

float3 BSDF::f(const float3& wo, const float3& wi, const float3& n) const {
  Frame localFrame(n);

  return fImpl(localFrame.wtl(wo), localFrame.wtl(wi));
}

BSDFSample BSDF::sample(
  const float3& wo,
  const float3& n,
  const float2& u,
  float uc
) const {
  Frame localFrame(n);

  auto sample = sampleImpl(localFrame.wtl(wo), u, uc);
  sample.wi = localFrame.ltw(sample.wi);
  return sample;
}

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
    m_hasEmission ? (Scatter::Reflected | Scatter::Emitted)
                  : Scatter::Reflected,
    m_rOverPi,
    m_emissive,
    wi,
    std::abs(wi.z()) * float(invPi)
  };
}

}