#include "naive-integrator.hpp"

namespace yart::cpu {

NaiveIntegrator::NaiveIntegrator(
  Buffer& buffer,
  const Camera& camera,
  Sampler& sampler
) noexcept: RayIntegrator(buffer, camera, sampler) {}

float3 NaiveIntegrator::Li(const Ray& ray) {
  return LiImpl(ray, scene->root());
}

float3 NaiveIntegrator::LiImpl(
  const Ray& ray,
  const Node& root,
  uint32_t depth
) {
  if (depth > m_maxDepth) return {};
  m_rayCounter++;

  Hit hit;
  bool didHit = testNode(ray, 0.001f, hit, root);
  if (!didHit) {
    float3 L;
    for (const auto& light: scene->lights()) {
      if (light.type() == Light::Type::Infinite) {
        float3 Le = light.Le(octahedralUV(ray.dir));
//          Le = min(Le, float3(1.0f));
        L += Le;
      }
    }

    L += backgroundColor;
    return L;
  }

  BSDFSample res = hit.bsdf->sample(
    -ray.dir,
    hit.n,
    hit.tg,
    hit.uv,
    m_sampler.get2D(),
    m_sampler.get1D(),
    m_sampler.get1D()
  );

  float3 Li;
  if (res.is(BSDFSample::Emitted)) {
    Li += res.Le;
  }
  if (res.is(BSDFSample::Reflected | BSDFSample::Transmitted)) {
    float3 fcos = res.f * absDot(res.wi, hit.n);

    Ray scattered(hit.p, res.wi);
    Li += LiImpl(scattered, root, depth + 1) * fcos / res.pdf;
  }

  return Li;
}

}