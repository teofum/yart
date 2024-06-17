#ifndef YART_METAL_HPP
#define YART_METAL_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {
using namespace math;

class MetalBSDF : public BSDF {
public:
  explicit MetalBSDF(const float3& reflectance, float ior) noexcept;

private:
  float3 m_reflectance;
  float m_ior;

  [[nodiscard]] float3 fImpl(const float3& wo, const float3& wi) const override;

  [[nodiscard]] float pdf(const float3& wo, const float3& wi) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc
  ) const override;
};

}

#endif //YART_METAL_HPP
