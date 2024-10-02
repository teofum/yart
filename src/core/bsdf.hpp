#ifndef YART_BSDF_HPP
#define YART_BSDF_HPP

#include <variant>

#include <math/math.hpp>
#include "ray.hpp"
#include "texture.hpp"

namespace yart {
using namespace math;

/**
 * Calculate material roughness for regularized paths
 */
[[nodiscard]] constexpr float roughen(float roughness) noexcept {
  return max(roughness, std::clamp(roughness * 2.0f, 0.1f, 0.3f));
}

struct BSDFSample {
  enum Scatter {
    Absorbed = 0,
    Emitted = 1,
    Reflected = 2,
    Transmitted = 4,
    Diffuse = 8,
    Glossy = 16,
    Specular = 32
  };

  int scatter;      // Scatter flags
  float3 f;         // BSDF value
  float3 Le;        // Emission at sampled point
  float3 wi;        // Sampled incident light direction
  float pdf;        // Sampled direction PDF
  float roughness;  // Sampled point roughness, used for path regularization

  [[nodiscard]] constexpr bool is(int flag) const noexcept {
    return scatter & flag;
  }
};

class BSDF {
public:
  /**
   * Evaluate the BSDF at a point for a reflected and incident light direction
   * @param wo Reflected light direction
   * @param wi Incident light direction
   * @param n Surface (shading) normal
   * @param t Surface (shading) tangent
   * @param uv Texture coordinates
   * @return BSDF value
   */
  [[nodiscard]] float3 f(
    const float3& wo,
    const float3& wi,
    const float3& n,
    const float3& t,
    const float2& uv
  ) const;

  /**
   * Evaluate the PDF at a point for a reflected and incident light direction
   * @param wo Reflected light direction
   * @param wi Incident light direction
   * @param n Surface (shading) normal
   * @param t Surface (shading) tangent
   * @param uv Texture coordinates
   * @return PDF value
   */
  [[nodiscard]] float pdf(
    const float3& wo,
    const float3& wi,
    const float3& n,
    const float3& t,
    const float2& uv
  ) const;

  /**
   * Sample the BSDF
   * @param wo Reflected light direction
   * @param n Surface (shading) normal
   * @param t Surface (shading) tangent
   * @param uv Texture coordinates
   * @param u 2D sampled random value
   * @param uc 1D sampled random value
   * @param uc2 Aux. 1D sampled random value
   * @param regularized true if path is regularized
   * @return BSDF sample data including incident direction, value and pdf
   */
  [[nodiscard]] BSDFSample sample(
    const float3& wo,
    const float3& n,
    const float3& t,
    const float2& uv,
    const float2& u,
    float uc,
    float uc2,
    bool regularized = false
  ) const;

  /**
   * Get the surface shading normal at a point
   * @param n Geometric normal
   * @param t Geometric tangent
   * @param uv Texture coordinates
   * @return Surface normal
   */
  [[nodiscard]] float3 normal(
    const float3& n,
    const float4& t,
    const float2& uv
  ) const noexcept;

  /**
   * Get alpha value at a point
   * @param uv Texture coordinates
   * @return Alpha value
   */
  [[nodiscard]] virtual float alpha(const float2& uv) const = 0;

  /**
   * Get base color (albedo) at a point
   * @param uv Texture coordinates
   * @return Base color
   */
  [[nodiscard]] virtual float3 base(const float2& uv) const = 0;

  /**
   * Check if material is transparent to NEE/occlusion rays
   */
  [[nodiscard]] virtual bool transparent() const = 0;

  /**
   * Get material emission strength
   * @return Emission strength (no texture), nullptr if not emissive
   */
  [[nodiscard]] constexpr virtual const float3* emission() const noexcept = 0;

  /**
   * Returns volumetric attenuation factor for a given distance traveled
   * inside the medium
   */
  [[nodiscard]] virtual float3 attenuation(float d) const;

protected:
  const RGBTexture* m_normalTexture = nullptr;
  float m_normalScale = 1.0f;

  [[nodiscard]] virtual float3 fImpl(
    const float3& wo,
    const float3& wi,
    const float2& uv
  ) const = 0;

  [[nodiscard]] virtual float pdfImpl(
    const float3& wo,
    const float3& wi,
    const float2& uv
  ) const = 0;

  [[nodiscard]] virtual BSDFSample sampleImpl(
    const float3& wo,
    const float2& uv,
    const float2& u,
    float uc,
    float uc2,
    bool regularized
  ) const = 0;
};

/**
 * GGX microfacet distribution
 */
class GGX {
public:
  constexpr explicit GGX(float roughness) noexcept {
    m_alphaX = m_alphaY = roughness * roughness;
  }

  constexpr GGX(float roughness, float anisotropic) noexcept {
    float alpha = roughness * roughness;
    float aspect = std::sqrt(1.0f - 0.9f * anisotropic);
    m_alphaX = alpha / aspect;
    m_alphaY = alpha * aspect;
  }

  // Microfacet distribution function
  [[nodiscard]] constexpr float mdf(const float3& w) const noexcept {
    const float cos2Theta = w.z() * w.z();
    const float sin2Theta = std::max(0.0f, 1.0f - cos2Theta);
    const float tan2Theta = sin2Theta / cos2Theta;

    // Checking for infinity is surprisingly slow, and unlikely enough we can
    // just let it blow up and throw away the sample
//    if (isinf(tan2Theta)) return 0;

    const float cos4Theta = cos2Theta * cos2Theta;
    float k = tan2Theta;

    if (m_alphaX != m_alphaY) {
      float cos2Phi = sin2Theta == 0.0f ? 1.0f : w.x() * w.x() / sin2Theta;
      float sin2Phi = sin2Theta == 0.0f ? 1.0f : w.y() * w.y() / sin2Theta;
      k *= (cos2Phi / (m_alphaX * m_alphaX) + sin2Phi / (m_alphaY * m_alphaY));
    } else {
      k /= (m_alphaX * m_alphaX);
    }

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
  float m_alphaX = 0.0f, m_alphaY = 0.0f;

  [[nodiscard]] constexpr float lambda(const float3& w) const noexcept {
    const float cos2Theta = w.z() * w.z();
    const float sin2Theta = (1.0f - cos2Theta);
    const float tan2Theta = sin2Theta / cos2Theta;

    float alpha2 = m_alphaX * m_alphaX;

    if (m_alphaX != m_alphaY) {
      float cos2Phi = sin2Theta == 0.0f ? 1.0f : w.x() * w.x() / sin2Theta;
      float sin2Phi = sin2Theta == 0.0f ? 0.0f : w.y() * w.y() / sin2Theta;
      alpha2 = alpha2 * cos2Phi + m_alphaY * m_alphaY * sin2Phi;
    }

    return (std::sqrt(1.0f + alpha2 * tan2Theta) - 1.0f) * 0.5f;
  }
};

}

#endif //YART_BSDF_HPP
