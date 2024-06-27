#include "core.hpp"

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

}
