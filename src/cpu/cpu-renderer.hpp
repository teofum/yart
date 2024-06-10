#ifndef YART_CPU_RENDERER_HPP
#define YART_CPU_RENDERER_HPP

#include <vector>
#include <random>

#include <core/core.hpp>
#include <math/math.hpp>

#include "hit.hpp"

namespace yart::cpu {

class CpuRenderThread : public Renderer {
private:
  const uint32 m_samples;
  const uint32 m_maxDepth;
  const uint32 m_threadId;

  std::unique_ptr<Xoshiro::Xoshiro256PP> m_threadRng;

  [[nodiscard]] float3 rayColor(
    const Ray& ray,
    const Node& root,
    uint32 depth = 0
  );

  [[nodiscard]] std::optional<Hit> testNode(
    const Ray& ray,
    const interval<float>& tInt,
    const Node& node
  ) const;

  [[nodiscard]] std::optional<Hit> testMesh(
    const Ray& ray,
    const interval<float>& tInt,
    const Mesh& mesh
  ) const;

  [[nodiscard]] std::optional<Hit> testTriangle(
    const Ray& ray,
    const interval<float>& tInt,
    const Triangle& tri,
    const Mesh& mesh
  ) const;

  [[nodiscard]] bool testBoundingBox(
    const Ray& ray,
    const interval<float>& tInt,
    const BoundingBox& bounds
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

public:
  CpuRenderThread(
    Buffer& buffer,
    const Camera& camera,
    uint32 threadId,
    uint32 samples = 10,
    uint32 maxDepth = 10
  ) noexcept;

  void render(const Node& root) override;
};

class CpuRenderer : public Renderer {
private:
  uint32 m_threadCount;
  uint32 m_samples = 100;
  uint32 m_maxDepth = 20;

public:
  constexpr CpuRenderer(
    Buffer& buffer,
    const Camera& camera,
    uint32 threadCount = 1
  ) noexcept: Renderer(buffer, camera), m_threadCount(threadCount) {

  }

  void render(const Node& root) override;
};

}

#endif //YART_CPU_RENDERER_HPP
