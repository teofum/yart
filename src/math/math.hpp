#ifndef YART_MATH_HPP
#define YART_MATH_HPP

#include "math_base.hpp"
#include "complex.hpp"
#include "vec.hpp"
#include "mat.hpp"
#include "bounds.hpp"
#include "transform.hpp"
#include "frame.hpp"
#include "sampling.hpp"

namespace yart::math {

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
  float sin2Theta_t = sin2Theta / (ior * ior);
  if (sin2Theta_t >= 1.0f) return false;

  float cosTheta_t = std::sqrt(1.0f - sin2Theta_t);
  wt = -wi / ior + (cosTheta / ior - cosTheta_t) * normal;
  return true;
}

[[nodiscard]] constexpr float fresnelDielectric(
  float cosTheta,
  float ior
) noexcept {
  cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
  if (cosTheta < 0.0f) {
    ior = 1.0f / ior;
    cosTheta = -cosTheta;
  }

  float sin2Theta = (1.0f - cosTheta * cosTheta);
  float sin2Theta_t = sin2Theta / (ior * ior);
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
  complex cosTheta_t = sqrt(1.0f - sin2Theta_t);

  complex r_prl = (ik * cosTheta - cosTheta_t) / (ik * cosTheta + cosTheta_t);
  complex r_per = (cosTheta - ik * cosTheta_t) / (cosTheta + ik * cosTheta_t);
  return (norm(r_prl) + norm(r_per)) * 0.5f;
}

[[nodiscard]] constexpr float3 fresnelSchlick(
  const float3& r,
  float cosTheta
) noexcept {
  const float k = 1.0f - cosTheta;
  const float k2 = k * k;
  return r + (float3(1.0f) - r) * (k2 * k2 * k);
}

[[nodiscard]] constexpr float fresnelSchlickDielectric(
  float cosTheta,
  float ior
) noexcept {
  const float k = 1.0f - cosTheta;
  const float k2 = k * k;
  float r = (1.0f - ior) / (1.0f + ior);
  r = r * r;

  return r + (1.0f - r) * k2 * k2 * k;
}

[[nodiscard]] constexpr uint32_t reverseBits32(uint32_t n) noexcept {
  n = (n << 16) | (n >> 16);
  n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
  n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
  n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
  n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
  return n;
}

[[nodiscard]] constexpr uint32_t multSobolGen(
  const std::array<uint32_t, 32>& C,
  uint32_t d
) noexcept {
  uint32_t v = 0;
  for (uint32_t i = 0; d != 0; i++, d >>= 1) {
    if (d & 1) v ^= C[i];
  }
  return v;
}

[[nodiscard]] constexpr uint64_t leftShift2(uint64_t x) {
  x &= 0xffffffff;
  x = (x ^ (x << 16)) & 0x0000ffff0000ffff;
  x = (x ^ (x << 8)) & 0x00ff00ff00ff00ff;
  x = (x ^ (x << 4)) & 0x0f0f0f0f0f0f0f0f;
  x = (x ^ (x << 2)) & 0x3333333333333333;
  x = (x ^ (x << 1)) & 0x5555555555555555;
  return x;
}

[[nodiscard]] constexpr uint64_t encodeMorton2(uint32_t x, uint32_t y) {
  return (leftShift2(y) << 1) | leftShift2(x);
}

}

#endif //YART_MATH_HPP
