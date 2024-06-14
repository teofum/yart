#include "naive-integrator.hpp"

namespace yart::cpu {

NaiveIntegrator::NaiveIntegrator(Buffer& buffer, const Camera& camera) noexcept
  : RayIntegrator(buffer, camera) {}

float3 NaiveIntegrator::Li(
  const Ray& ray,
  const Node& root
) {
  return LiImpl(ray, root);
}

float3 NaiveIntegrator::LiImpl(
  const Ray& ray,
  const Node& root,
  uint32_t depth
) {
  if (depth > maxDepth) return {0.0f, 0.0f, 0.0f};
  m_rayCounter++;

  Hit hit;
  bool didHit = testNode(ray, 0.001f, hit, root);
  if (!didHit) return {0.0f, 0.0f, 0.0f}; // TODO background color

  ScatterResult res = scatter(ray, hit);

  if (const Scattered* r = std::get_if<Scattered>(&res)) {
    float3 reflected = LiImpl(r->scattered, root, depth + 1) * r->attenuation;
    return reflected + r->emission;
  } else if (const Emitted* e = std::get_if<Emitted>(&res)) {
    return e->emission;
  } else {
    // Absorbed
    return {0.0f, 0.0f, 0.0f};
  }
}

ScatterResult NaiveIntegrator::scatter(const Ray& ray, const Hit& hit) {
  return std::visit(
    [&](const auto& mat) {
      return scatterImpl(mat, ray, hit);
    },
    *(hit.material)
  );
}

ScatterResult NaiveIntegrator::scatterImpl(
  const Lambertian& mat,
  const Ray& ray,
  const Hit& hit
) {
  float4x4 basis = normalToTBN(hit.normal);
  float3 scatterDir = float3(
    basis * float4(random::randomCosineVec(m_rng), 0.0f)
  );
  Ray scattered(hit.position, scatterDir);

  return Scattered{
    mat.albedo,
    {0.0f, 0.0f, 0.0f},
    scattered
  };
}

ScatterResult NaiveIntegrator::scatterImpl(
  const Emissive& mat,
  const Ray& ray,
  const Hit& hit
) {
  return Emitted{mat.emission};
}

}