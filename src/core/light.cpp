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
  const TrianglePositions* tri,
  const TriangleData* data,
  const float3& emission,
  const Transform& transform
) noexcept
  : Light(transform), m_tri(tri), m_data(data), m_emission(emission) {
  // Get triangle areas and total surface area

  TrianglePositions transformed;
  transformed.p0 = transform(m_tri->p0, Transform::Type::Point);
  transformed.p1 = transform(m_tri->p1, Transform::Type::Point);
  transformed.p2 = transform(m_tri->p2, Transform::Type::Point);

  m_area = transformed.area();
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
  const float3 b = samplers::sampleTriUniform(u);
  float3 pos = b[0] * m_tri->p0 + b[1] * m_tri->p1 + b[2] * m_tri->p2;
  float3 normal =
    b[0] * m_data->n[0] + b[1] * m_data->n[1] + b[2] * m_data->n[2];

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
  const Texture* emissionTexture,
  fbounds2 bounds
) noexcept
  : Light({}),
    m_sceneRadius(sceneRadius),
    m_bounds(std::move(bounds)),
    m_emissionTexture(emissionTexture) {
  uint32_t w = m_emissionTexture->width(), h = m_emissionTexture->height();
  uint32_t x0 = uint32_t(float(w) * bounds.min.x());
  uint32_t y0 = uint32_t(float(h) * bounds.min.y());
  uint32_t x1 = uint32_t(float(w) * bounds.max.x());
  uint32_t y1 = uint32_t(float(h) * bounds.max.y());
  w = x1 - x0, h = y1 - y0;

  std::vector<float> d(w * h);
  for (uint32_t y = 0; y < h; y++) {
    float v = (float(y) + 0.5f) / float(h);
    float z = 1.0f - v * 2.0f;
    float sinTheta = std::sqrt(1.0f - z * z);
    for (uint32_t x = 0; x < w; x++) {
      float3 sampled = float3((*m_emissionTexture)(x + x0, y + y0));
      float value = sum(sampled) / 3.0f;
      d[y * w + x] = value * sinTheta;
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
  float2 uv = sphericalUV(wi);
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

  float3 wi = invSphericalUV(uv);
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
