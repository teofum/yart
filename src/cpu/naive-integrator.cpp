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

  ScatterResult res = scatter(ray, hit);
  float rand = m_sampler.get1D();

  if (const Scattered* r = std::get_if<Scattered>(&res)) {
    float pTerm = std::max(1.0f - sum(r->attenuation), 0.0f);
    if (rand < pTerm) return {};

    return LiImpl(r->scattered, root, depth + 1) * r->attenuation / (1 - pTerm);
  } else if (const Emitted* e = std::get_if<Emitted>(&res)) {
    return e->emission;
  } else {
    // Absorbed
    return {};
  }
}

ScatterResult NaiveIntegrator::scatter(const Ray& ray, const Hit& hit) {
  float4x4 basis = normalToTBN(hit.normal);
  float3 scatterDir = float3(
    basis * float4(random::randomCosineVec(m_sampler.get2D()), 0.0f)
  );
  Ray scattered(hit.position, scatterDir);

  auto emission = hit.material->emissive;
  if (sum(emission) > 0.0f) {
    return Emitted{emission};
  }

  return Scattered{
    hit.material->diffuse,
    float3(),
    scattered
  };
}

}