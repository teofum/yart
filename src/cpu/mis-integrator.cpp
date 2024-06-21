#include "mis-integrator.hpp"

namespace yart::cpu {

MISIntegrator::MISIntegrator(
  Buffer& buffer,
  const Camera& camera,
  Sampler& sampler
) noexcept: RayIntegrator(buffer, camera, sampler) {}

float3 MISIntegrator::Li(const Ray& r) {
  float3 L(0.0f), attenuation(1.0f), lastHit;
  uint32_t depth = 0;
  bool specularBounce = false;
  float lastPdf = 0.0f;
  Ray ray(r);

  while (depth < maxDepth) {
    m_rayCounter++;

    // Test scene for intersection and return if none
    Hit hit;
    bool didHit = testNode(ray, 0.001f, hit, scene->root());
    if (!didHit) {
      L += float3(0.5f);
      break;
    } // TODO background color

    // Sample the BSDF to get the next ray direction
    float2 u = m_sampler.get2D();
    float uc = m_sampler.get1D();
    float uc2 = m_sampler.get1D();
    BSDFSample res = hit.bsdf->sample(-ray.dir, hit.n, u, uc, uc2);

    // Calculate indirect lighting (ie, if the ray happens to hit a light)
    if (res.is(BSDFSample::Emitted)) {
      if (depth == 0 || specularBounce) {
        L += attenuation * res.Le * absDot(res.wi, hit.n);
      } else if (hit.lightIdx != -1) {
        float pdfLight = lightPdf(lastHit, ray.dir, *hit.light) /
                         m_lightSampler.p(hit.p, hit.n, hit.lightIdx);
        float wBSDF = lastPdf / (lastPdf + pdfLight);
        L += attenuation * wBSDF * res.Le * absDot(res.wi, hit.n);
      }
    }

    if (!res.is(BSDFSample::Reflected | BSDFSample::Transmitted))
      break;

    // Calculate direct lighting
    if (!res.is(BSDFSample::Emitted | BSDFSample::Specular))
      L += attenuation * Ld(-ray.dir, hit);

    // Update state variables
    float3 fcos = res.f * absDot(res.wi, hit.n);
    specularBounce = res.is(BSDFSample::Specular);
    attenuation *= fcos / res.pdf;
    ray = Ray(hit.p, res.wi);
    lastPdf = res.pdf;
    lastHit = hit.p;
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
  float uLight = m_sampler.get1D();
  SampledLight l = m_lightSampler.sample(hit.p, hit.n, uLight);

  // Sample the light
  LightSample ls = l.light.sample(hit.p, hit.n, u, uc);

  float3 f = hit.bsdf->f(wo, ls.wi, hit.n);
  if (length2(f) == 0.0f || !unoccluded(hit.p, ls.p)) return {};

  float pdfBSDF = hit.bsdf->pdf(wo, ls.wi, hit.n);
  float pdfLight = l.p * ls.pdf * length2(hit.p - ls.p) / absDot(ls.n, -ls.wi);

  return ls.Li * f * absDot(ls.wi, hit.n) / (pdfBSDF + pdfLight);
}

float MISIntegrator::lightPdf(
  const float3& p,
  const float3& wi,
  const Light& light
) const {
  Hit lh;
  Ray lr(
    light.transform().inverse(p, Transform::Type::Point),
    light.transform().inverse(wi, Transform::Type::Vector)
  );

  if (testMesh(lr, 0.001f, lh, *light.mesh())) {
    return light.pdf() * length2(lr.origin - lh.p) / absDot(lh.n, -lr.dir);
  } else {
    return 0.0f;
  }
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
