#ifndef YART_MATH_HPP
#define YART_MATH_HPP

#include "math_base.hpp"
#include "vec.hpp"
#include "mat.hpp"
#include "bounds.hpp"
#include "transform.hpp"
#include "frame.hpp"
#include "sampling.hpp"

#include <complex>

namespace yart::math {

using complex = std::complex<float>;

[[nodiscard]] constexpr float3 reflect(
  const float3& wo,
  const float3& normal
) noexcept {
  return -wo + normal * 2.0 * dot(wo, normal);
}

[[nodiscard]] constexpr bool refract(
  const float3& wi,
  float3 normal,
  float ior,
  float3& wt
) noexcept {
  float cosTheta = dot(wi, normal);
  if (cosTheta < 0.0f) {
    ior = 1.0f / ior;
    cosTheta *= -1.0f;
    normal *= -1.0f;
  }

  float sin2Theta = (1.0f - cosTheta * cosTheta);
  float sin2Theta_t = sin2Theta / ior * ior;
  if (sin2Theta_t >= 1.0f) return false;

  float cosTheta_t = std::sqrt(1.0f - sin2Theta_t);
  wt = -wi / ior + (cosTheta / ior - cosTheta_t) * normal;
  return true;
}

[[nodiscard]] constexpr float fresnelDielectric(
  float cosTheta,
  float ior
) noexcept {
  if (cosTheta < 0.0f) {
    ior = 1.0f / ior;
    cosTheta *= -1.0f;
  }

  float sin2Theta = (1.0f - cosTheta * cosTheta);
  float sin2Theta_t = sin2Theta / ior * ior;
  if (sin2Theta_t >= 1.0f) return 1.0f;

  float cosTheta_t = std::sqrt(1.0f - sin2Theta_t);

  float r_prl = (ior * cosTheta - cosTheta_t) / (ior * cosTheta + cosTheta_t);
  float r_per = (cosTheta - ior * cosTheta_t) / (cosTheta + ior * cosTheta_t);
  return (r_prl * r_prl + r_per * r_per) * 0.5f;
}

[[nodiscard]] constexpr float fresnelComplex(
  float cosTheta,
  float ior,
  float k
) noexcept {
  complex ik(ior, k);
  
  cosTheta = std::clamp(cosTheta, 0.0f, 1.0f);
  float sin2Theta = (1.0f - cosTheta * cosTheta);
  complex sin2Theta_t = sin2Theta / (ik * ik);
  complex cosTheta_t = std::sqrt(1.0f - sin2Theta_t);

  complex r_prl = (ik * cosTheta - cosTheta_t) / (ik * cosTheta + cosTheta_t);
  complex r_per = (cosTheta - ik * cosTheta_t) / (cosTheta + ik * cosTheta_t);
  return (std::norm(r_prl) + std::norm(r_per)) * 0.5f;
}

}

#endif //YART_MATH_HPP
