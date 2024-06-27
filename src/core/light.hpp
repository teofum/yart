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

  [[nodiscard]] virtual float power() const noexcept = 0;

  [[nodiscard]] virtual float pdf() const noexcept = 0;

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

  [[nodiscard]] float power() const noexcept override;

  [[nodiscard]] float pdf() const noexcept override;

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

}

#endif //YART_LIGHT_HPP
