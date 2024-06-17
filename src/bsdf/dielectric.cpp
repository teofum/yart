#include "dielectric.hpp"

namespace yart {

DielectricBSDF::DielectricBSDF(float ior) noexcept: m_ior(ior) {}

float3 DielectricBSDF::fImpl(const float3& wo, const float3& wi) const {
  return {};
}

float DielectricBSDF::pdf(const float3& wo, const float3& wi) const {
  return 0;
}

BSDFSample DielectricBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc
) const {
  float reflectance = fresnelDielectric(wo.z(), m_ior);
  float transmittance = 1.0f - reflectance;

  if (uc >= reflectance) {
    float3 wi;
    if (!refract(wo, axis_z<float>, m_ior, wi)) return {Scatter::Absorbed};
    return {
      Scatter::Transmitted,
      float3(transmittance / std::abs(wi.z())),
      float3(),
      wi,
      transmittance
    };
  } else {
    return {
      Scatter::Reflected,
      float3(reflectance / std::abs(wo.z())),
      float3(),
      float3(-wo.x(), -wo.y(), wo.z()),
      reflectance
    };
  }
}

}
