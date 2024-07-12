#ifndef YART_SAMPLING_HPP
#define YART_SAMPLING_HPP

#include <span>
#include <utility>

#include "vec.hpp"
#include "mat.hpp"
#include "bounds.hpp"

namespace yart::math::samplers {

[[nodiscard]] constexpr float2 pixelJitterSquare(const float2& u) noexcept {
  return {
    u.x() - 0.5f,
    u.y() - 0.5f
  };
}

[[nodiscard]] constexpr float2 pixelJitterGaussian(
  const float2& u,
  float stdDev
) noexcept {
  float a = std::sqrt(-2.0f * log(u.x())) * stdDev;
  float b = 2.0f * float(pi) * u.y();

  return {a * std::cos(b), a * std::sin(b)};
}

[[nodiscard]] constexpr float3 sampleCosineHemisphere(const float2& u) noexcept {
  const float phi = u.x() * 2.0f * float(pi);
  const float sqrtr2 = std::sqrt(u.y());
  const float x = std::cos(phi) * sqrtr2;
  const float y = std::sin(phi) * sqrtr2;
  const float z = std::sqrt(1.0f - u.y());

  return {x, y, z};
}

[[nodiscard]] constexpr float2 sampleDiskUniform(const float2& u) noexcept {
  const float r = std::sqrt(u.x());
  const float theta = 2.0f * float(pi) * u.y();

  return {r * std::cos(theta), r * std::sin(theta)};
}

[[nodiscard]] constexpr float3 sampleSphereUniform(const float2& u) noexcept {
  const float z = 1.0f - 2.0f * u[0];
  const float r = std::sqrt(1.0f - z * z);
  const float phi = 2.0f * float(pi) * u[1];
  return {r * std::cos(phi), r * std::sin(phi), z};
}

[[nodiscard]] constexpr float3 sampleTriUniform(const float2& u) noexcept {
  float b0, b1;
  if (u.x() < u.y()) {
    b0 = u.x() * 0.5f;
    b1 = u.y() - b0;
  } else {
    b1 = u.y() * 0.5f;
    b0 = u.x() - b1;
  }
  return {b0, b1, 1.0f - b0 - b1};
}

/**
 * Sample a regular polygon with outer radius 1 uniformly.
 * @param u random float2
 * @param sides number of sides (>= 4)
 * @return a uniformly sampled point in the polygon
 */
[[nodiscard]] constexpr float2 samplePolyUniform(
  float2 u,
  uint32_t sides
) noexcept {
  u.x() *= sides;
  uint32_t side = min(sides - 1, uint32_t(u.x()));
  u.x() -= float(side);

  float3 b = sampleTriUniform(u);
  float theta1 = float(side) / float(sides) * 2.0f * float(pi);
  float theta2 = float(side + 1) / float(sides) * 2.0f * float(pi);
  float c1 = std::cos(theta1), s1 = std::sin(theta1);
  float c2 = std::cos(theta2), s2 = std::sin(theta2);

  return float2(0, 0) * b[0] +
         float2(-s1, c1) * b[1] +
         float2(-s2, c2) * b[2];
}

[[nodiscard]] constexpr float sampleLinear(float u, float a, float b) noexcept {
  if (u == 0.0f && a == 0.0f) return 0.0f;
  return u * (a + b) / (a + std::sqrt(lerp(a * a, b * b, u)));
}

[[nodiscard]] constexpr float2 sampleBilinear(
  const float2& u,
  const float4& w
) noexcept {
  float2 p;
  p.y() = sampleLinear(u.y(), w[0] + w[1], w[2] + w[3]);
  p.x() = sampleLinear(u.x(), lerp(w[0], w[2], p.y()), lerp(w[1], w[3], p.y()));
  return p;
}

[[nodiscard]] constexpr float bilinearPdf(
  const float2& u,
  const float4& w
) noexcept {
  if (u.x() < 0 || u.x() > 1 || u.y() < 0 || u.y() > 1) return 0.0f;
  if (w[0] + w[1] + w[2] + w[3] == 0) return 1;
  return 4 *
         ((1 - u[0]) * (1 - u[1]) * w[0] + u[0] * (1 - u[1]) * w[1] +
          (1 - u[0]) * u[1] * w[2] + u[0] * u[1] * w[3]) /
         (w[0] + w[1] + w[2] + w[3]);
}

class PiecewiseConstant1D {
public:
  constexpr PiecewiseConstant1D() = default;

  constexpr PiecewiseConstant1D(const std::span<float>& f, float min, float max)
    : m_func(f.begin(), f.end()), m_cdf(f.size() + 1), m_min(min), m_max(max) {
    // Take absolute value of f
    for (float& x: m_func) x = std::abs(x);

    // Compute the function integral
    m_cdf[0] = 0.0f;
    size_t n = m_func.size();
    for (size_t i = 1; i < n + 1; i++)
      m_cdf[i] = m_cdf[i - 1] + m_func[i - 1] * (max - min) / float(n);

    // Initialize the CDF
    m_integral = m_cdf[n];
    if (m_integral == 0.0f) {
      // Handle the zero integral case by sampling uniformly
      for (size_t i = 1; i < n + 1; i++) m_cdf[i] = float(i) / float(n);
    } else {
      for (size_t i = 1; i < n + 1; i++) m_cdf[i] /= m_integral;
    }
  }

  [[nodiscard]] constexpr float integral() const noexcept { return m_integral; }

  [[nodiscard]] constexpr float f(size_t i) const noexcept { return m_func[i]; }

  [[nodiscard]] constexpr size_t size() const noexcept { return m_func.size(); }

  [[nodiscard]] float sample(
    float u,
    float* pdf = nullptr,
    uint32_t* offset = nullptr
  ) const noexcept;

private:
  std::vector<float> m_func, m_cdf;
  float m_min = 0.0f, m_max = 0.0f, m_integral = 0.0f;
};

class PiecewiseConstant2D {
public:
  constexpr PiecewiseConstant2D() = default;

  constexpr PiecewiseConstant2D(
    const std::span<float>& f,
    fbounds2 domain,
    uint32_t nu,
    uint32_t nv
  ) : m_domain(std::move(domain)) {
    for (uint32_t v = 0; v < nv; v++)
      m_conditional
        .emplace_back(f.subspan(v * nu, nu), m_domain.min[0], m_domain.max[0]);

    std::vector<float> marginalFunc(nv);
    for (uint32_t v = 0; v < nv; v++)
      marginalFunc[v] = m_conditional[v].integral();
    m_marginal = PiecewiseConstant1D(
      marginalFunc,
      m_domain.min[1],
      m_domain.max[1]
    );
  }

  [[nodiscard]] constexpr float integral() const noexcept {
    return m_marginal.integral();
  }

  [[nodiscard]] float2 sample(float2 u, float* pdf = nullptr) const noexcept;

  [[nodiscard]] float pdf(const float2& uv) const noexcept;

private:
  fbounds2 m_domain;
  std::vector<PiecewiseConstant1D> m_conditional;
  PiecewiseConstant1D m_marginal;
};

}

#endif //YART_SAMPLING_HPP
