#include "core.hpp"
#include "light.hpp"

#include <utility>


namespace yart {

Light::Light(Transform transform) noexcept
  : m_transform(std::move(transform)) {}

const Transform& Light::transform() const noexcept {
  return m_transform;
}

AreaLight::AreaLight(
  const Triangle* tri,
  const Mesh* mesh,
  const float3& emission,
  const Transform& transform
) noexcept
  : Light(transform), m_tri(tri), m_mesh(mesh), m_emission(emission) {
  // Make sure to apply transforms before calculating area! This is necessary
  // to calculate the area correctly when triangles are scaled
  float3 t0 = transform(m_mesh->vertex(tri->i0), Transform::Type::Point);
  float3 t1 = transform(m_mesh->vertex(tri->i1), Transform::Type::Point);
  float3 t2 = transform(m_mesh->vertex(tri->i2), Transform::Type::Point);

  m_area = triangleArea(t0, t1, t2);
}

Light::Type AreaLight::type() const noexcept {
  return Light::Type::Area;
}

float AreaLight::power() const noexcept {
  return length(m_emission) * m_area * float(pi) * (twoSided ? 2.0f : 1.0f);
}

float AreaLight::pdf(const float3& wi) const noexcept {
  return 1.0f / m_area;
}

LightSample AreaLight::sample(
  const float3& p,
  const float3& n,
  const float2& u,
  float uc
) const noexcept {
  // Uniformly sample barycentric coordinates, then get position and normal
  const float3 b = samplers::sampleTriUniform(u);
  float3 pos = b[0] * m_mesh->vertex(m_tri->i0) +
               b[1] * m_mesh->vertex(m_tri->i1) +
               b[2] * m_mesh->vertex(m_tri->i2);
  float3 normal = b[0] * m_mesh->vertexData(m_tri->i0).normal +
                  b[1] * m_mesh->vertexData(m_tri->i1).normal +
                  b[2] * m_mesh->vertexData(m_tri->i2).normal;

  // Transform to world space
  pos = m_transform(pos, Transform::Type::Point);
  normal = m_transform(normal, Transform::Type::Normal);

  float3 wi = normalized(pos - p);
  float pdf = 1.0f / m_area;

  return {
    m_emission,
    wi,
    pos,
    normal,
    pdf
  };
}

float3 AreaLight::Le(const float2& uv) const noexcept {
  return m_emission;
}

float3 AreaLight::Lavg() const noexcept {
  return m_emission;
}

UniformInfiniteLight::UniformInfiniteLight(
  float sceneRadius,
  const float3& emission
) noexcept
  : Light({}),
    m_sceneRadius(sceneRadius),
    m_emission(emission) {}

Light::Type UniformInfiniteLight::type() const noexcept {
  return Light::Type::Infinite;
}

float3 UniformInfiniteLight::Le(const float2& uv) const noexcept {
  return m_emission;
}

float UniformInfiniteLight::power() const noexcept {
  // For power calculation, assume the infinite light is a sphere containing the
  // entire scene
  return 4.0f * float(pi) * float(pi) * m_sceneRadius * m_sceneRadius *
         length(m_emission);
}

float UniformInfiniteLight::pdf(const float3& wi) const noexcept {
  return 0.25f * float(invPi); // 1 / 4pi
}

LightSample UniformInfiniteLight::sample(
  const float3& p,
  const float3& n,
  const float2& u,
  float uc
) const noexcept {
  float3 wi = samplers::sampleSphereUniform(u);
  float pdf = 0.25f * float(invPi);
  return {
    m_emission,
    wi,
    wi * m_sceneRadius,
    -wi,
    pdf
  };
}

float3 UniformInfiniteLight::Lavg() const noexcept {
  return m_emission;
}

ImageInfiniteLight::ImageInfiniteLight(
  float sceneRadius,
  const HDRTexture* emissionTexture,
  fbounds2 bounds
) noexcept
  : Light({}),
    m_sceneRadius(sceneRadius),
    m_bounds(std::move(bounds)),
    m_emissionTexture(emissionTexture) {
  // Portal light support. TODO: Doesn't really work with octahedral projection,
  // should do something with this.
  uint32_t w = m_emissionTexture->width(), h = m_emissionTexture->height();
  uint32_t x0 = uint32_t(float(w) * bounds.min.x());
  uint32_t y0 = uint32_t(float(h) * bounds.min.y());
  uint32_t x1 = uint32_t(float(w) * bounds.max.x());
  uint32_t y1 = uint32_t(float(h) * bounds.max.y());
  uint32_t wTexture = w;
  w = x1 - x0, h = y1 - y0;

  // Build pixel-by-pixel probability distribution for importance sampling
  std::vector<float> d(w * h);
  for (uint32_t y = 0; y < h; y++) {
    float v = (float(y) + 0.5f) / float(h);
    float z = 1.0f - v * 2.0f;
    float sinTheta = std::sqrt(1.0f - z * z);

    for (uint32_t x = 0; x < w; x++) {
      float3 sampled = getValue(
        *m_emissionTexture,
        x + x0 + (y + y0) * wTexture
      );
      float value = sum(sampled) / 3.0f;
      d[y * w + x] = value * sinTheta; // TODO: do we need the sine term?
      m_Lavg += sampled;
    }
  }

  m_Lavg /= float(w * h);

  m_distribution = samplers::PiecewiseConstant2D(
    d, m_bounds, w, h
  );

  float avg = std::accumulate(d.begin(), d.end(), 0.0f) / float(d.size());
  for (float& v: d) v = max(v - avg, 0.0f);

  m_compensatedDistribution = samplers::PiecewiseConstant2D(
    d, m_bounds, w, h
  );

  // Calculate surface area
  float phi0 = m_bounds.min.x() * 2.0f * float(pi);
  float phi1 = m_bounds.max.x() * 2.0f * float(pi);
  float theta0 = m_bounds.min.y() * float(pi);
  float theta1 = m_bounds.max.y() * float(pi);
  m_surfaceArea = (phi1 - phi0) * (std::cos(theta0) - std::cos(theta1));
}

Light::Type ImageInfiniteLight::type() const noexcept {
  return Light::Type::Infinite;
}

float3 ImageInfiniteLight::Le(const float2& uv) const noexcept {
  if (!m_bounds.includes(uv))
    return {};

  return float3(m_emissionTexture->sample(uv));
}

float ImageInfiniteLight::power() const noexcept {
  return m_surfaceArea * float(pi) * m_sceneRadius * m_sceneRadius *
         sum(m_Lavg) / 3.0f;
}

float ImageInfiniteLight::pdf(const float3& wi) const noexcept {
  float2 uv = octahedralUV(transform.inverse(wi, Transform::Type::Vector));
  if (!m_bounds.includes(uv))return 0;

  float pdf = m_distribution.pdf(uv) / (4.0f * float(pi));
  return pdf;
}

LightSample ImageInfiniteLight::sample(
  const float3& p,
  const float3& n,
  const float2& u,
  float uc
) const noexcept {
  float pdf;
  float2 uv = m_distribution.sample(u, &pdf);
  if (pdf == 0.0f) return {};

  float3 wi = transform(invOctahedralUV(uv));
  pdf /= m_surfaceArea;
  return {
    Le(uv),
    wi,
    wi * 2.0f * m_sceneRadius,
    -wi,
    pdf
  };
}

float3 ImageInfiniteLight::Lavg() const noexcept {
  return m_Lavg;
}

}
