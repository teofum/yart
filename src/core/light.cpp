#include "core.hpp"
#include "light.hpp"


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

float AreaLight::pdf() const noexcept {
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

float3 AreaLight::Le(const float3& wi) const noexcept {
  return m_emission;
}

InfiniteLight::InfiniteLight(float sceneRadius, const float3& emission) noexcept
  : Light({}),
    m_sceneRadius(sceneRadius),
    m_emission(emission),
    m_emissionTexture(nullptr) {}

InfiniteLight::InfiniteLight(
  float sceneRadius,
  const Texture* emissionTexture
) noexcept
  : Light({}),
    m_sceneRadius(sceneRadius),
    m_emission(float3{}),
    m_emissionTexture(emissionTexture) {}

Light::Type InfiniteLight::type() const noexcept {
  return Light::Type::Infinite;
}

float InfiniteLight::power() const noexcept {
  return 4.0f * float(pi) * float(pi) * m_sceneRadius * m_sceneRadius *
         length(m_emission);
}

float InfiniteLight::pdf() const noexcept {
  return 0.25f * float(invPi); // 1 / 4pi
}

LightSample InfiniteLight::sample(
  const float3& p,
  const float3& n,
  const float2& u,
  float uc
) const noexcept {
  float3 wi = samplers::sampleSphereUniform(u);
  float pdf = 0.25f * float(invPi);
  return {
    Le(wi),
    wi,
    wi * m_sceneRadius,
    -wi,
    pdf
  };
}

float3 InfiniteLight::Le(const float3& wi) const noexcept {
  if (m_emissionTexture)
    return float3(m_emissionTexture->sample(sphericalUV(wi)));

  return m_emission;
}

}
