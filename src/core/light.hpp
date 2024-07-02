#ifndef YART_LIGHT_HPP
#define YART_LIGHT_HPP

#include <math/math.hpp>
#include "mesh.hpp"

namespace yart {
using namespace math;

struct LightSample {
  float3 Li;
  float3 wi;
  float3 p, n;
  float pdf;
};

class Light {
public:
  explicit Light(Transform transform) noexcept;

  enum class Type {
    Area,
    Infinite
  };

  [[nodiscard]] virtual Type type() const noexcept = 0;

  [[nodiscard]] virtual float3 Le(const float2& uv) const noexcept = 0;

  [[nodiscard]] virtual float3 Lavg() const noexcept = 0;

  [[nodiscard]] virtual float power() const noexcept = 0;

  [[nodiscard]] virtual float pdf(const float3& wi) const noexcept = 0;

  [[nodiscard]] virtual LightSample sample(
    const float3& p,
    const float3& n,
    const float2& u,
    float uc
  ) const noexcept = 0;

  [[nodiscard]] const Transform& transform() const noexcept;

protected:
  Transform m_transform;
};

class AreaLight : public Light {
public:
  bool twoSided = false;

  AreaLight(
    const TrianglePositions* tri,
    const TriangleData* data,
    const float3& emission,
    const Transform& transform
  ) noexcept;

  [[nodiscard]] Type type() const noexcept override;

  [[nodiscard]] float3 Le(const float2& uv) const noexcept override;

  [[nodiscard]] float3 Lavg() const noexcept override;

  [[nodiscard]] float power() const noexcept override;

  [[nodiscard]] float pdf(const float3& wi) const noexcept override;

  [[nodiscard]] LightSample sample(
    const float3& p,
    const float3& n,
    const float2& u,
    float uc
  ) const noexcept override;

private:
  const TrianglePositions* m_tri;
  const TriangleData* m_data;

  float m_area;
  float3 m_emission;
};

class UniformInfiniteLight : public Light {
public:
  UniformInfiniteLight(float sceneRadius, const float3& emission) noexcept;

  [[nodiscard]] Type type() const noexcept override;

  [[nodiscard]] float3 Le(const float2& uv) const noexcept override;

  [[nodiscard]] float3 Lavg() const noexcept override;

  [[nodiscard]] float power() const noexcept override;

  [[nodiscard]] float pdf(const float3& wi) const noexcept override;

  [[nodiscard]] LightSample sample(
    const float3& p,
    const float3& n,
    const float2& u,
    float uc
  ) const noexcept override;

private:
  float m_sceneRadius;
  float3 m_emission;
};

class ImageInfiniteLight : public Light {
public:
  ImageInfiniteLight(
    float sceneRadius,
    const Texture* emissionTexture,
    fbounds2 bounds = {{0, 0},
                       {1, 1}}
  ) noexcept;

  [[nodiscard]] Type type() const noexcept override;

  [[nodiscard]] float3 Le(const float2& uv) const noexcept override;

  [[nodiscard]] float3 Lavg() const noexcept override;

  [[nodiscard]] float power() const noexcept override;

  [[nodiscard]] float pdf(const float3& wi) const noexcept override;

  [[nodiscard]] LightSample sample(
    const float3& p,
    const float3& n,
    const float2& u,
    float uc
  ) const noexcept override;

private:
  float m_sceneRadius, m_surfaceArea = 4.0f * float(pi);
  float3 m_Lavg;
  fbounds2 m_bounds;
  const Texture* m_emissionTexture;
  samplers::PiecewiseConstant2D m_distribution, m_compensatedDistribution;
};

}

#endif //YART_LIGHT_HPP
