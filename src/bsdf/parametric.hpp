#ifndef YART_PARAMETRIC_HPP
#define YART_PARAMETRIC_HPP

#include <math/math.hpp>
#include <core/core.hpp>
#include "diffuse.hpp"
#include "dielectric.hpp"
#include "metal.hpp"

namespace yart {
using namespace math;

class ParametricBSDF : public BSDF {
public:
  explicit ParametricBSDF(
    const float3& baseColor,
    float metallic = 0.0f,
    float roughness = 0.0f,
    float transmission = 0.0f,
    float ior = 1.5f,
    const float3& emission = float3()
  ) noexcept;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return m_diffuse.emission();
  }

private:
  DiffuseBSDF m_diffuse;
  DielectricBSDF m_dielectric;
  MetalBSDF m_metallic;
  float m_cTrans, m_cMetallic;

  [[nodiscard]] float3 fImpl(const float3& wo, const float3& wi) const override;

  [[nodiscard]] float pdfImpl(
    const float3& wo,
    const float3& wi
  ) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc,
    float uc2
  ) const override;
};

}

#endif //YART_PARAMETRIC_HPP