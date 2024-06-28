#ifndef YART_METAL_HPP
#define YART_METAL_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {
using namespace math;

class MetalBSDF : public BSDF {
public:
  explicit MetalBSDF(
    const float3& reflectance,
    float roughness = 0.0f,
    float anisotropic = 0.0f,
    float anisoRotation = 0.0f,
    float ior = 1.5f
  ) noexcept;

  [[nodiscard]] constexpr const float3* emission() const noexcept override {
    return nullptr;
  }

  friend class ParametricBSDF;

private:
  float3 m_baseColor;
  float m_ior, m_roughness;
  GGX m_microfacets, m_mfRoughened;
  float3x3 m_localRotation, m_invRotation;

  [[nodiscard]] float3 fImpl(
    const float3& wo,
    const float3& wi,
    const float2& uv
  ) const override;

  [[nodiscard]] float pdfImpl(
    const float3& wo,
    const float3& wi,
    const float2& uv
  ) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& uv,
    const float2& u,
    float uc,
    float uc2,
    bool regularized
  ) const override;
};

}

#endif //YART_METAL_HPP
