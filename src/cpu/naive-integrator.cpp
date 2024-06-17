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
  if (depth > maxDepth) return {};
  m_rayCounter++;

  Hit hit;
  bool didHit = testNode(ray, 0.001f, hit, root);
  if (!didHit) return {}; // TODO background color

  BSDFSample res = hit.material->sample(
    -ray.dir,
    hit.normal,
    m_sampler.get2D(),
    m_sampler.get1D()
  );

  float3 Li;
  if (res.scatter == Scatter::Emitted) {
    Li += res.Le;
  }
  if (res.scatter == Scatter::Reflected) {
    float3 fcos = res.f * std::abs(dot(res.wi, hit.normal));

    Ray scattered(hit.position, res.wi);
    Li += LiImpl(scattered, root, depth + 1) * fcos / res.pdf;
  }
  if (res.scatter == Scatter::Transmitted) {
    float3 fcos = res.f * std::abs(dot(res.wi, hit.normal));

    Ray scattered(hit.position, res.wi);
    Li += LiImpl(scattered, root, depth + 1) * fcos / res.pdf;
  }

  return Li;
}

}