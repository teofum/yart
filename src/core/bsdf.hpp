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
  Transmitted = 4,
  Diffuse = 8,
  Glossy = 16,
  Specular = 32
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

  [[nodiscard]] constexpr bool is(Scatter flag) const noexcept {
    return bool(scatter & flag);
  }
};

class BSDF {
public:
  [[nodiscard]] float3 f(
    const float3& wo,
    const float3& wi,
    const float3& n
  ) const;

  [[nodiscard]] float pdf(
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

  [[nodiscard]] constexpr virtual const float3* emission() const noexcept = 0;

  [[nodiscard]] constexpr virtual bool specular() const noexcept = 0;

protected:
  [[nodiscard]] virtual float3 fImpl(
    const float3& wo,
    const float3& wi
  ) const = 0;

  [[nodiscard]] virtual float pdfImpl(
    const float3& wo,
    const float3& wi
  ) const = 0;

  [[nodiscard]] virtual BSDFSample sampleImpl(
    const float3& wo,
    const float2& u,
    float uc
  ) const = 0;
};

class GGX {
public:
  constexpr explicit GGX(float alpha) noexcept
    : m_alphaX(alpha), m_alphaY(alpha) {}

  constexpr GGX(float alphaX, float alphaY) noexcept
    : m_alphaX(alphaX), m_alphaY(alphaY) {}

  // Microfacet distribution function
  [[nodiscard]] constexpr float mdf(const float3& w) const noexcept {
    const float cos2Theta = w.z() * w.z();
    const float sin2Theta = std::max(0.0f, 1.0f - cos2Theta);
    const float tan2Theta = sin2Theta / cos2Theta;
    if (isinf(tan2Theta)) return 0;

    const float cos4Theta = cos2Theta * cos2Theta;
    const float cos2Phi = sin2Theta == 0.0f ? 1.0f : w.x() * w.x() / sin2Theta;
    const float sin2Phi = sin2Theta == 0.0f ? 1.0f : w.y() * w.y() / sin2Theta;

    const float k = tan2Theta * (cos2Phi / (m_alphaX * m_alphaX) +
                                 sin2Phi / (m_alphaY * m_alphaY));
    const float k2 = (1.0f + k) * (1.0f + k);
    return 1.0f / (float(pi) * m_alphaX * m_alphaY * cos4Theta * k2);
  }

  // Masking function
  [[nodiscard]] constexpr float g1(const float3& w) const noexcept {
    return 1.0f / (1.0f + lambda(w));
  }

  // Masking + shadowing function
  [[nodiscard]] constexpr float g(
    const float3& wo,
    const float3& wi
  ) const noexcept {
    return 1.0f / (1.0f + lambda(wo) + lambda(wi));
  }

  // Visible microfacet distribution function
  [[nodiscard]] constexpr float vmdf(
    const float3& w,
    const float3& wm
  ) const noexcept {
    return g1(w) / std::abs(w.z()) * mdf(wm) * absDot(w, wm);
  }

  [[nodiscard]] constexpr bool smooth() const noexcept {
    return m_alphaX < 1e-3f && m_alphaY < 1e-3f;
  }

  [[nodiscard]] constexpr float3 sampleVisibleMicrofacet(
    const float3& w,
    const float2& u
  ) const noexcept {
    // Transform w from ellipsoid to hemispherical space
    float3 wh = normalized(float3(m_alphaX * w.x(), m_alphaY * w.y(), w.z()));
    if (wh.z() < 0) wh *= -1;

    const float3 b = (wh.z() < 0.9999f) ? normalized(cross(axis_z<float>, wh))
                                        : axis_x<float>;
    const float3 t = cross(wh, b);

    // Sample a disk and transform to the truncated hemisphere projection
    float2 p = samplers::sampleDiskUniform(u);
    const float h = std::sqrt(1.0f - p.x() * p.x());
    p.y() = lerp(h, p.y(), 0.5f * wh.z() + 0.5f);

    // Project onto the hemisphere and transform back to ellipsoid space
    const float pz = std::sqrt(std::max(0.0f, 1.0f - length2(p)));
    float3 nh = p.x() * b + p.y() * t + pz * wh;

    return normalized(
      float3(
        m_alphaX * nh.x(),
        m_alphaY * nh.y(),
        std::max(1e-6f, nh.z())
      )
    );
  }

private:
  float m_alphaX, m_alphaY;

  [[nodiscard]] constexpr float lambda(const float3& w) const noexcept {
    const float cos2Theta = w.z() * w.z();
    const float sin2Theta = (1.0f - cos2Theta);
    const float tan2Theta = sin2Theta / cos2Theta;
    if (isinf(tan2Theta)) return 0;

    const float cos2Phi = sin2Theta == 0.0f ? 1.0f : w.x() * w.x() / sin2Theta;
    const float sin2Phi = sin2Theta == 0.0f ? 0.0f : w.y() * w.y() / sin2Theta;

    const float alpha2 = m_alphaX * m_alphaX * cos2Phi
                         + m_alphaY * m_alphaY * sin2Phi;
    return (std::sqrt(1.0f + alpha2 * tan2Theta) - 1.0f) * 0.5f;
  }
};

}

#endif //YART_BSDF_HPP