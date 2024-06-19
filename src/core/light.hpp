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

  [[nodiscard]] virtual const Mesh* mesh() const noexcept;

  [[nodiscard]] const Transform& transform() const noexcept;

protected:
  Transform m_transform;
};

class AreaLight : public Light {
public:
  bool twoSided = false;

  AreaLight(
    const Mesh* mesh,
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

  [[nodiscard]] const Mesh* mesh() const noexcept override;

private:
  const Mesh* m_mesh;
  float3 m_emission;

  float m_area;
  std::vector<float> m_triAreas;
  std::vector<float> m_triCumulativeAreas;
};

}

#endif //YART_LIGHT_HPP
