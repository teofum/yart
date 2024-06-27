#ifndef YART_RAY_INTEGRATOR_HPP
#define YART_RAY_INTEGRATOR_HPP

#include <core/core.hpp>
#include <math/math.hpp>

#include "hit.hpp"
#include "integrator.hpp"

namespace yart::cpu {

class RayIntegrator : public Integrator {
public:
  uint32_t maxDepth = 10;

  RayIntegrator(
    Buffer& buffer,
    const Camera& camera,
    Sampler& sampler
  ) noexcept;

protected:
  [[nodiscard]] float3 sample(uint32_t sx, uint32_t sy) override;

  [[nodiscard]] virtual float3 Li(const Ray& ray) = 0;

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

  [[nodiscard]] bool testBVH(
    const Ray& ray,
    float tMin,
    Hit& hit,
    const BVH& bvh
  ) const;

  [[nodiscard]] bool testTriangle(
    const Ray& ray,
    float tMin,
    Hit& hit,
    const TrianglePositions& tri,
    uint32_t idx
  ) const;

  [[nodiscard]] bool testBoundingBox(
    const Ray& ray,
    const interval<float>& tInt,
    const fbounds3& bounds,
    float* d
  ) const;
};

}

#endif //YART_RAY_INTEGRATOR_HPP
