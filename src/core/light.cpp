#include "core.hpp"

namespace yart {

Light::Light(Transform transform) noexcept
  : m_transform(std::move(transform)) {}

const Mesh* Light::mesh() const noexcept {
  return nullptr;
}

const Transform& Light::transform() const noexcept {
  return m_transform;
}

AreaLight::AreaLight(
  const Mesh* mesh,
  const float3& emission,
  const Transform& transform
) noexcept
  : Light(transform), m_mesh(mesh), m_emission(emission) {
  // Get triangle areas and total surface area
  const auto& triangles = mesh->triangles();
  m_triAreas.reserve(triangles.size());
  m_triCumulativeAreas.reserve(triangles.size());

  const auto transformedTriangles = std::views::transform(
    triangles, [&](const Triangle& tri) {
      Triangle transformed(tri);
      transformed.v0.p = transform(transformed.v0.p, Transform::Type::Point);
      transformed.v1.p = transform(transformed.v1.p, Transform::Type::Point);
      transformed.v2.p = transform(transformed.v2.p, Transform::Type::Point);
      return transformed;
    }
  );

  m_area = 0.0f;
  for (const Triangle& tri: transformedTriangles) {
    const float triArea = tri.area();
    m_triAreas.push_back(triArea);
    m_triCumulativeAreas.push_back(m_area + triArea);
    m_area += triArea;
  }
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
  uc *= m_area;
  size_t triIdx = findInterval(
    int64_t(m_triCumulativeAreas.size()),
    [&](size_t i) { return m_triCumulativeAreas[i] < uc; }
  );

  const Triangle& tri = m_mesh->triangles()[triIdx];
  auto ts = tri.sample(u);
  ts.pos = m_transform(ts.pos, Transform::Type::Point);
  ts.normal = m_transform(ts.normal, Transform::Type::Normal);

  float3 wi = normalized(ts.pos - p);
  float pdf = 1.0f / m_area;

  return {
    m_emission,
    wi,
    ts.pos,
    ts.normal,
    pdf
  };
}

const Mesh* AreaLight::mesh() const noexcept {
  return m_mesh;
}

}
