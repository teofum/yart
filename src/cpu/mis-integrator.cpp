#include "mis-integrator.hpp"

#define REG_ROUGHNESS_THRESHOLD 0.5f;

namespace yart::cpu {

MISIntegrator::MISIntegrator(
  Buffer& buffer,
  const Camera& camera,
  Sampler& sampler
) noexcept: RayIntegrator(buffer, camera, sampler) {}

float3 MISIntegrator::Li(const Ray& r) {
  Hit lastHit;
  float3 L(0.0f), attenuation(1.0f);
  uint32_t depth = 0;
  bool specularBounce = false, regularized = false;
  float lastPdf = 0.0f, accRoughness = 0.0f;
  Ray ray(r);

  while (depth < maxDepth) {
    m_rayCounter++;

    // Test scene for intersection and return if none
    Hit hit;
    bool didHit = testNode(ray, 0.001f, hit, scene->root());
    if (!didHit) {
      for (const auto& light: scene->lights()) {
        if (light.type() == Light::Type::Infinite) {
          float3 Le = light.Le(sphericalUV(ray.dir));

          if (depth > 0) {
            float pdfLight = light.pdf(ray.dir) / absDot(lastHit.n, -ray.dir);
            float wBSDF = lastPdf / (lastPdf + pdfLight);
            Le *= wBSDF;
          }

//          Le = min(Le, float3(1.0f));
          L += attenuation * Le;
        }
      }

      L += attenuation * backgroundColor;
      break;
    }

    // Sample the BSDF to get the next ray direction
    float2 u = m_sampler.get2D();
    float uc = m_sampler.get1D();
    float uc2 = m_sampler.get1D();
    BSDFSample res = hit.bsdf->sample(
      -ray.dir,
      hit.n,
      hit.tg,
      hit.uv,
      u,
      uc,
      uc2,
      regularized
    );

    // Calculate indirect lighting (ie, if the ray happens to hit a light)
    if (res.is(BSDFSample::Emitted)) {
      if (depth == 0 || specularBounce) {
        L += attenuation * res.Le * absDot(res.wi, lastHit.n);
      } else if (hit.lightIdx != -1) {
        const Light& light = scene->light(hit.lightIdx);
        float pdfLight = light.pdf(-ray.dir) * length2(lastHit.p - hit.p) *
                         m_lightSampler.p(lastHit.p, lastHit.n, hit.lightIdx) /
                         absDot(hit.n, -ray.dir);

        float wBSDF = lastPdf / (lastPdf + pdfLight);
        L += attenuation * wBSDF * res.Le * absDot(res.wi, lastHit.n);
      }
    }

    if (!res.is(BSDFSample::Reflected | BSDFSample::Transmitted))
      break;

    // Calculate direct lighting
    if (!res.is(BSDFSample::Emitted | BSDFSample::Specular))
      L += attenuation * Ld(-ray.dir, hit);

    // Update state variables
    float3 fcos = res.f * absDot(res.wi, hit.n);
    attenuation *= fcos / res.pdf;
    ray = Ray(hit.p, res.wi);

    specularBounce = res.is(BSDFSample::Specular);
    accRoughness += res.roughness;
    regularized = accRoughness > REG_ROUGHNESS_THRESHOLD;
    lastPdf = res.pdf;
    lastHit = hit;
    depth++;

    // Russian roulette
    if (depth > 1 && maxComponent(attenuation) < 1.0f) {
      float q = std::max(0.0f, 1.0f - maxComponent(attenuation));
      if (m_sampler.get1D() < q) break;
      attenuation /= 1.0f - q;
    }
  }

//  L = min(L, float3(1.0f));
//  L /= L + 1.0f;
  return L;
}

/**
 * Calculate direct lighting at a point
 */
float3 MISIntegrator::Ld(const float3& wo, const Hit& hit) {
  if (scene->nLights() == 0) return {};

  m_rayCounter++;
  float uc = m_sampler.get1D();
  float2 u = m_sampler.get2D();

  // Pick a random light for sampling
  SampledLight l = m_lightSampler.sample(hit.p, hit.n, uc);

  // Sample the light
  LightSample ls = l.light.sample(hit.p, hit.n, u, 0.0f);

  float3 f = hit.bsdf->f(wo, ls.wi, hit.n, hit.tg, hit.uv);
  if (length2(f) == 0.0f || !unoccluded(hit.p, ls.p)) return {};

  float pdfBSDF = hit.bsdf->pdf(wo, ls.wi, hit.n, hit.tg, hit.uv);
  float pdfLight = l.p * ls.pdf / absDot(ls.n, -ls.wi);
  if (l.light.type() == Light::Type::Area) pdfLight *= length2(hit.p - ls.p);

  return ls.Li * f * absDot(ls.wi, hit.n) / (pdfBSDF + pdfLight);
}

bool MISIntegrator::unoccluded(const float3& from, const float3& to) const {
  Ray occlusionRay(from, normalized(to - from));
  Hit occlusionHit;
  occlusionHit.t = length(to - from) - 0.001f;
  return !testNode(occlusionRay, 0.001f, occlusionHit, scene->root());
}

void MISIntegrator::setup() {
  m_lightSampler.init(scene);
}

}
