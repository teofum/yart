#ifndef YART_BSDF_HPP
#define YART_BSDF_HPP

#include <variant>

#include <math/math.hpp>
#include "ray.hpp"

namespace yart {
using namespace math;

enum class Scatter : uint8_t {
  Absorbed = 0,
  Emitted = 1,
  Reflected = 2,
  Transmitted = 4
};

constexpr Scatter operator|(Scatter lhs, Scatter rhs) noexcept {
  return static_cast<Scatter>(static_cast<uint8_t>(lhs) |
                              static_cast<uint8_t>(lhs));
}

constexpr Scatter operator&(Scatter lhs, Scatter rhs) noexcept {
  return static_cast<Scatter>(static_cast<uint8_t>(lhs) &
                              static_cast<uint8_t>(lhs));
}

struct BSDFSample {
  Scatter scatter;
  float3 f;
  float3 Le;
  float3 wi;
  float pdf;
};

class BSDF {
public:
  [[nodiscard]] float3 f(
    const float3& wo,
    const float3& wi,
    const float3& n
  ) const;

  [[nodiscard]] BSDFSample sample(
    const float3& wo,
    const float3& n,
    const float2& u,
    float uc
  ) const;

protected:
  [[nodiscard]] virtual float3 fImpl(
    const float3& wo,
    const float3& wi
  ) const = 0;

  [[nodiscard]] virtual float pdf(const float3& wo, const float3& wi) const = 0;

  [[nodiscard]] virtual BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc
  ) const = 0;
};

class DiffuseBSDF : public BSDF {
public:
  explicit DiffuseBSDF(
    const float3& reflectance,
    const float3& emissive = float3()
  ) noexcept;

private:
  float3 m_reflectance, m_rOverPi, m_emissive;
  bool m_hasEmission;

  [[nodiscard]] float3 fImpl(const float3& wo, const float3& wi) const override;

  [[nodiscard]] float pdf(const float3& wo, const float3& wi) const override;

  [[nodiscard]] BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc
  ) const override;
};

}

#endif //YART_BSDF_HPP
