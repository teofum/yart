#include "bsdf.hpp"

namespace yart {

float3 BSDF::f(const float3& wo, const float3& wi, const float3& n) const {
  Frame localFrame(n);
  return fImpl(localFrame.wtl(wo), localFrame.wtl(wi));
}

BSDFSample BSDF::sample(
  const float3& wo,
  const float3& n,
  const float2& u,
  float uc
) const {
  Frame localFrame(n);

  auto sample = sampleImpl(localFrame.wtl(wo), u, uc);
  sample.wi = localFrame.ltw(sample.wi);
  return sample;
}

float BSDF::pdf(const float3& wo, const float3& wi, const float3& n) const {
  Frame localFrame(n);
  return pdfImpl(localFrame.wtl(wo), localFrame.wtl(wi));
}

}