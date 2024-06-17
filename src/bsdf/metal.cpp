#include "metal.hpp"

namespace yart {

MetalBSDF::MetalBSDF(const float3& reflectance, float ior) noexcept
  : m_reflectance(reflectance), m_ior(ior) {}

float3 MetalBSDF::fImpl(const float3& wo, const float3& wi) const {
  return {};
}

float MetalBSDF::pdf(const float3& wo, const float3& wi) const {
  return 0;
}

BSDFSample MetalBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc
) const {
  float r = fresnelComplex(wo.z(), m_ior, 100.0f);

  return {
    Scatter::Reflected,
    float3(m_reflectance * r / std::abs(wo.z())),
    float3(),
    float3(-wo.x(), -wo.y(), wo.z()),
    r
  };
}

}
