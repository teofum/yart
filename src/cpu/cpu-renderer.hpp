#ifndef YART_CPU_RENDERER_HPP
#define YART_CPU_RENDERER_HPP

#include <vector>
#include <random>
#include <shared_mutex>

#include <core/core.hpp>
#include <math/math.hpp>

#include "hit.hpp"

namespace yart::cpu {

class CpuRenderer : public Renderer {
public:
  CpuRenderer(
    const Buffer& buffer,
    const Camera& camera,
    uint32_t threadCount = 1
  ) noexcept;

  CpuRenderer(
    Buffer&& buffer,
    const Camera& camera,
    uint32_t threadCount = 1
  ) noexcept;

  void render(const Node& root) override;

  friend class CpuRenderThread;

private:
  uint32_t m_threadCount;
  uint32_t m_samples = 100;
  uint32_t m_maxDepth = 20;
  uint32_t m_samplesPerThread;
  uint32_t m_currentWave = 0, m_currentWaveFinishedThreads = 0;

  std::shared_mutex m_bufferMutex;

  void writeBuffer(const Buffer& src, size_t wave) noexcept;
};

class CpuRenderThread {
public:
  CpuRenderThread(CpuRenderer* renderer, std::random_device& rd) noexcept;

  void operator()(const Node& root) noexcept;

private:
  CpuRenderer* m_renderer;

  uint32_t m_samples, m_maxDepth;
  Xoshiro::Xoshiro256PP m_threadRng;
  Buffer m_threadBuffer;

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
};

}

#endif //YART_CPU_RENDERER_HPP
