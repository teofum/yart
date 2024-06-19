#ifndef YART_DIFFUSE_HPP
#define YART_DIFFUSE_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {
using namespace math;

class DiffuseBSDF : public BSDF {
public:
  explicit DiffuseBSDF(
    const float3& reflectance,
    const float3& emissive = float3()
  ) noexcept;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return m_hasEmission ? &m_emissive : nullptr;
  }

private:
  float3 m_reflectance, m_rOverPi, m_emissive;
  bool m_hasEmission;

  [[nodiscard]] float3 fImpl(const float3& wo, const float3& wi) const override;

  [[nodiscard]] float pdfImpl(
    const float3& wo,
    const float3& wi
  ) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc
  ) const override;
};

}

#endif //YART_DIFFUSE_HPP
