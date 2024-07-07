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

  std::vector<float> radiance(w * h);
  float L = 0.0f, Lmax = 0.0f;

  for (uint32_t y = 0; y < h; y++) {
    for (uint32_t x = 0; x < w; x++) {
      float3 sampled = float3((*m_emissionTexture)(x, y));
      float value = sum(sampled) / 3.0f;
      radiance[y * w + x] = value;
      L += value;
      Lmax = max(Lmax, value);
      m_Lavg += sampled;
    }
  }

  m_Lavg /= float(w * h);

  float Lavg = sum(m_Lavg) / 3.0f, Lstd = 0.0f;
  for (const float& Lp: radiance) {
    float k = std::abs(Lp - Lavg);
    Lstd += k * k;
  }
  Lstd /= float(w * h);
  Lstd = std::sqrt(Lstd);

  float Lmin = Lmax + Lavg * 1e4f * Lstd / max(Lmax - Lavg, 1e-8f);
  subdivide(radiance, L, Lmin, fbounds2({0, 0}, {1, 1}));
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
  float2 uv = octahedralUV(wi);
  if (!m_bounds.includes(uv)) return 0;

  for (const fbounds2& bin: m_bins) {
    if (bin.includes(uv)) {
      float binArea = bin.size().x() * bin.size().y();
      return binArea / (4.0f * float(pi));
    }
  }

  return 0;
}

LightSample ImageInfiniteLight::sample(
  const float3& p,
  const float3& n,
  const float2& u,
  float uc
) const noexcept {
  uint32_t nBins = m_bins.size();
  uint32_t idx = min(nBins - 1, uint32_t(uc * nBins));
  const fbounds2& bin = m_bins[idx];
  float2 size = bin.size();
  float2 uv = bin.min + u * size;
  float pdf = 1.0f / (nBins * size.x() * size.y());

  float3 wi = invOctahedralUV(uv);
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

void ImageInfiniteLight::subdivide(
  const std::vector<float>& Le,
  float L,
  float Lmin,
  const fbounds2& b
) {
  uint32_t w = m_emissionTexture->width(), h = m_emissionTexture->height();

  float2 size = b.size();
  if (L <= Lmin || (size.x() * w) * (size.y() * h) <= 1) {
    m_bins.push_back(b);
    return;
  }

  float2 center = b.min + size / 2.0f;
  fbounds2 b0(b), b1(b);

  uint32_t splitAxis = size.x() > size.y() ? 0 : 1;
  b0.max[splitAxis] = center[splitAxis];
  b1.min[splitAxis] = center[splitAxis];

  float L0 = 0.0f;
  uint32_t x0 = b0.min.x() * w, x1 = b0.max.x() * w;
  uint32_t y0 = b0.min.y() * h, y1 = b0.max.y() * h;

  for (uint32_t y = y0; y < y1; y++) {
    for (uint32_t x = x0; x < x1; x++) {
      float3 sampled = float3((*m_emissionTexture)(x, y));
      float value = sum(sampled) / 3.0f;
      L0 += value;
    }
  }
  float L1 = L - L0;

  subdivide(Le, L0, Lmin, b0);
  subdivide(Le, L1, Lmin, b1);
}

}
