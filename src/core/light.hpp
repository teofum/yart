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

/**
 * Light base class, provides basic interface for lights.
 */
class Light {
public:
  explicit Light(Transform transform) noexcept;

  enum class Type {
    Area,     // Physical lights attached to geometry
    Infinite  // Environment lights at (effective) infinity
  };

  [[nodiscard]] virtual Type type() const noexcept = 0;

  /**
   * Returns emitted light intensity at a given point.
   */
  [[nodiscard]] virtual float3 Le(const float2& uv) const noexcept = 0;

  /**
   * Average light intensity over the light's surface
   */
  [[nodiscard]] virtual float3 Lavg() const noexcept = 0;

  /**
   * Total emitted light power
   */
  [[nodiscard]] virtual float power() const noexcept = 0;

  /**
   * Light sampling PDF for a given direction
   */
  [[nodiscard]] virtual float pdf(const float3& wi) const noexcept = 0;

  /**
   * Sample a light source
   * @param p Origin position
   * @param n Origin normal
   * @param u Sampled 2D value
   * @param uc Sampled 1D value
   * @return
   */
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

/**
 * Physical light corresponding to a shape (triangle) somewhere in the world.
 * Emits light uniformly from the triangle's surface.
 */
class AreaLight : public Light {
public:
  bool twoSided = false;

  AreaLight(
    const Triangle* tri,
    const Mesh* mesh,
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
  const Triangle* m_tri;
  const Mesh* m_mesh;

  float m_area;
  float3 m_emission;
};

/**
 * Uniform light at infinity.
 */
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

/**
 * Light at infinity with an image (environment map) using octahedral projection.
 */
class ImageInfiniteLight : public Light {
public:
  Transform transform;

  ImageInfiniteLight(
    float sceneRadius,
    const HDRTexture* emissionTexture,
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
  const HDRTexture* m_emissionTexture;
  samplers::PiecewiseConstant2D m_distribution, m_compensatedDistribution;
};

}

#endif //YART_LIGHT_HPP
