#ifndef YART_DIELECTRIC_HPP
#define YART_DIELECTRIC_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {
using namespace math;

class DielectricBSDF : public BSDF {
public:
  explicit DielectricBSDF(float roughness = 0.0f, float ior = 1.5f) noexcept;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return nullptr;
  }

  friend class ParametricBSDF;

private:
  float m_ior;
  GGX m_microfacets;

  [[nodiscard]] float3 fImpl(const float3& wo, const float3& wi) const override;

  [[nodiscard]] float pdfImpl(
    const float3& wo,
    const float3& wi
  ) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc,
    float uc2,
    bool regularized
  ) const override;
};

}

#endif //YART_DIELECTRIC_HPP
