#ifndef YART_SAMPLING_HPP
#define YART_SAMPLING_HPP

#include "vec.hpp"

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

[[nodiscard]] constexpr float3 sampleTriSpherical(
  const float3& v0,
  const float3& v1,
  const float3& v2,
  const float3& p,
  const float2& u,
  float& pdf
) noexcept {
  // Do some trig magic to uniformly sample a direction w
  // See pbrt, section 6.5.4
  float3 a = normalized(v0 - p), b = normalized(v1 - p), c = normalized(v2 - p);
  float3 n_ab = cross(a, b), n_bc = cross(b, c), n_ca = cross(c, a);
  if (length2(n_ab) == 0 || length2(n_bc) == 0 || length2(n_ca) == 0) return {};
  n_ab = normalized(n_ab);
  n_bc = normalized(n_bc);
  n_ca = normalized(n_ca);

  const float
    alpha = angleBetween(n_ab, -n_ca),
    beta = angleBetween(n_bc, -n_ab),
    gamma = angleBetween(n_ca, -n_bc);

  const float Api = alpha + beta + gamma;
  const float Ap = lerp(float(pi), Api, u.x());
  const float A = Api - float(pi);
  pdf = (A <= 0) ? 0 : 1.0f / A;

  float cosAlpha = std::cos(alpha), sinAlpha = std::sin(alpha);
  float sinAp = std::sin(Ap), cosAp = std::cos(Ap);
  float sinPhi = sinAp * cosAlpha - cosAp * sinAlpha;
  float cosPhi = cosAp * cosAlpha + sinAp * sinAlpha;
  float k1 = cosPhi + cosAlpha;
  float k2 = sinPhi - sinAlpha * dot(a, b);
  float cosBp = (k2 + (k2 * cosPhi - k1 * sinPhi) * cosAlpha) /
                ((k2 * sinPhi + k1 * cosPhi) * sinAlpha);
  cosBp = std::clamp(cosBp, -1.0f, 1.0f);
  float sinBp = std::sqrt(1.0f - cosBp);
  float3 cp = cosBp * a + sinBp * normalized(gramSchmidt(c, a));

  float cosTheta = 1.0f - u.y() * (1.0f - dot(cp, b));
  float sinTheta = std::sqrt(1.0f - cosTheta);
  float3 w = cosTheta * b + sinTheta * normalized(gramSchmidt(cp, b));

  // Find the barycentric coords from w using a partial Möller-Trumbore
  float3 e1 = v1 - v0, e2 = v2 - v0;
  float3 s1 = cross(w, e2);
  float det = dot(s1, e1);
  float invDet = 1.0f / det;
  float3 s = p - v0;
  float b1 = dot(s, s1) * invDet;
  float b2 = dot(w, cross(s, e1)) * invDet;
  b1 = std::clamp(b1, 0.0f, 1.0f);
  b2 = std::clamp(b2, 0.0f, 1.0f);
  if (b1 + b2 > 1.0f) {
    b1 /= b1 + b2;
    b2 /= b1 + b2;
  }

  return {1.0f - b1 - b2, b1, b2};
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

// Stolen from pbrt
[[nodiscard]] constexpr float2 invertSphericalSample(
  const float3& v0,
  const float3& v1,
  const float3& v2,
  const float3& p,
  const float3& wi
) noexcept {
  float3 a = normalized(v0 - p), b = normalized(v1 - p), c = normalized(v2 - p);

  // Compute normalized cross products of all direction pairs
  float3 n_ab = cross(a, b), n_bc = cross(b, c), n_ca = cross(c, a);
  if (length2(n_ab) == 0 || length2(n_bc) == 0 || length2(n_ca) == 0) return {};
  n_ab = normalized(n_ab);
  n_bc = normalized(n_bc);
  n_ca = normalized(n_ca);

  float alpha = angleBetween(n_ab, -n_ca);
  float beta = angleBetween(n_bc, -n_ab);
  float gamma = angleBetween(n_ca, -n_bc);

  float3 cp = normalized(cross(cross(b, wi), cross(c, a)));
  if (dot(cp, a + c) < 0)
    cp = -cp;

  float u0;
  if (dot(a, cp) > 0.99999847691f /* 0.1 degrees */)
    u0 = 0;
  else {
    float3 n_cpb = cross(cp, b), n_acp = cross(a, cp);
    if (length2(n_cpb) == 0 || length2(n_acp) == 0) return {0.5, 0.5};
    n_cpb = normalized(n_cpb);
    n_acp = normalized(n_acp);
    float Ap =
      alpha + angleBetween(n_ab, n_cpb) + angleBetween(n_acp, -n_cpb) -
      float(pi);

    float A = alpha + beta + gamma - float(pi);
    u0 = Ap / A;
  }

  float u1 = (1 - dot(wi, b)) / (1 - dot(cp, b));
  return {std::clamp(u0, 0.0f, 1.0f), std::clamp(u1, 0.0f, 1.0f)};
}

}

#endif //YART_SAMPLING_HPP
