#ifndef YART_BASIC_INTEGRATOR_HPP
#define YART_BASIC_INTEGRATOR_HPP

#include <core/core.hpp>
#include <math/math.hpp>

#include "hit.hpp"

namespace yart::cpu {

class BasicIntegrator : public Integrator {
public:
  uint32_t samples = 1;
  uint32_t maxDepth = 20;

  BasicIntegrator(Buffer& buffer, const Camera& camera) noexcept;

  void render(const Node& root) override;

private:
  Xoshiro::Xoshiro256PP m_rng;

  [[nodiscard]] float3 rayColor(
    const Ray& ray,
    const Node& root,
    uint32 depth = 0
  );

  [[nodiscard]] bool testNode(
    const Ray& ray,
    float tMin,
    Hit& hit,
    const Node& node
  ) const;

  [[nodiscard]] bool testMesh(
    const Ray& ray,
    float tMin,
    Hit& hit,
    const Mesh& mesh
  ) const;

  [[nodiscard]] bool testTriangle(
    const Ray& ray,
    float tMin,
    Hit& hit,
    const Triangle& tri,
    const Mesh& mesh
  ) const;

  [[nodiscard]] bool testBoundingBox(
    const Ray& ray,
    const interval<float>& tInt,
    const fbounds3& bounds
  ) const;

  [[nodiscard]] ScatterResult scatter(const Ray& ray, const Hit& hit);

  [[nodiscard]] ScatterResult scatterImpl(
    const Lambertian& mat,
    const Ray& ray,
    const Hit& hit
  );

  [[nodiscard]] ScatterResult scatterImpl(
    const Emissive& mat,
    const Ray& ray,
    const Hit& hit
  );
};

}

#endif //YART_BASIC_INTEGRATOR_HPP
