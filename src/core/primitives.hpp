#ifndef YART_PRIMITIVES_HPP
#define YART_PRIMITIVES_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

struct TriSample {
  float3 pos, normal;
  float pdf;
};

struct Vertex {
  float3 position, normal;
  float2 textureCoords;
};

struct Face {
  size_t i0, i1, i2;
};

struct Triangle {
  Vertex v0, v1, v2;
  float3 centroid;

  constexpr Triangle(
    const Face& face,
    const std::vector<Vertex>& vertices
  ) noexcept {
    v0 = vertices[face.i0];
    v1 = vertices[face.i1];
    v2 = vertices[face.i2];
    centroid = (v0.position + v1.position + v2.position) / 3.0f;
  }

  [[nodiscard]] constexpr float area() const noexcept {
    const float3 edge1 = v1.position - v0.position;
    const float3 edge2 = v2.position - v0.position;
    return length(cross(edge1, edge2)) * 0.5f;
  }

  [[nodiscard]] constexpr TriSample sample(const float2& u) const noexcept {
    const float3 b = samplers::sampleTriUniform(u);
    float3 pos = b[0] * v0.position + b[1] * v1.position + b[2] * v2.position;
    float3 normal = b[0] * v0.normal + b[1] * v1.normal + b[2] * v2.normal;

    return {pos, normal, 1.0f / area()};
  }

  [[nodiscard]] constexpr TriSample sample(
    const float3& p,
    const float3& n,
    const float2& u
  ) const noexcept {
    float sa = solidAngle(p);
    // Handle numerically unstable corner cases
    if (sa < 3e-4f || sa > 6.22f) {
      auto s = sample(u);
      float3 wi = normalized(s.pos - p);
      s.pdf /= absDot(s.normal, -wi) / length2(p - s.pos);
      return s;
    }

    const float3& p0 = v0.position, p1 = v1.position, p2 = v2.position;
    float3
      w0 = normalized(p0 - p),
      w1 = normalized(p1 - p),
      w2 = normalized(p2 - p);
    float4 w(
      std::max(0.01f, absDot(n, w1)),
      std::max(0.01f, absDot(n, w1)),
      std::max(0.01f, absDot(n, w0)),
      std::max(0.01f, absDot(n, w2))
    );
    float2 u2 = samplers::sampleBilinear(u, w);
    float pdf = samplers::bilinearPdf(u, w);

    float triPdf;
    float3 b = samplers::sampleTriSpherical(p0, p1, p2, p, u2, triPdf);
    pdf *= triPdf;

    float3 pos = b[0] * p0 + b[1] * p1 + b[2] * p2;
    float3 normal = b[0] * v0.normal + b[1] * v1.normal + b[2] * v2.normal;

    return {pos, normal, pdf};
  }

  [[nodiscard]] constexpr float pdf(
    const float3& p,
    const float3& n,
    const float3& wi
  ) const noexcept {
    float sa = solidAngle(p);
    float pdf = 1.0f / sa;

    const float3& p0 = v0.position, p1 = v1.position, p2 = v2.position;
    float2 u = samplers::invertSphericalSample(p0, p1, p2, p, wi);
    float3
      w0 = normalized(p0 - p),
      w1 = normalized(p1 - p),
      w2 = normalized(p2 - p);
    float4 w(
      std::max(0.01f, absDot(n, w1)),
      std::max(0.01f, absDot(n, w1)),
      std::max(0.01f, absDot(n, w0)),
      std::max(0.01f, absDot(n, w2))
    );
    pdf *= samplers::bilinearPdf(u, w);

    return pdf;
  }

  [[nodiscard]] constexpr float solidAngle(const float3& p) const noexcept {
    float3 a = normalized(v0.position - p);
    float3 b = normalized(v1.position - p);
    float3 c = normalized(v2.position - p);
    return std::abs(
      2.0f * std::atan2(
        dot(a, cross(b, c)),
        (1.0f + dot(a, b) + dot(b, c) + dot(c, a))
      )
    );
  }

  [[nodiscard]] constexpr float pdf() const noexcept {
    return 1.0f / area();
  }
};

}

#endif //YART_PRIMITIVES_HPP
