#include "bsdf.hpp"

namespace yart {

float3 BSDF::f(
  const float3& wo,
  const float3& wi,
  const float3& n,
  const float3& t,
  const float2& uv
) const {
  Frame localFrame = length2(t) > 0 ? Frame(n, t) : Frame(n);
  return fImpl(localFrame.wtl(wo), localFrame.wtl(wi), uv);
}

float BSDF::pdf(
  const float3& wo,
  const float3& wi,
  const float3& n,
  const float3& t,
  const float2& uv
) const {
  Frame localFrame = length2(t) > 0 ? Frame(n, t) : Frame(n);
  return pdfImpl(localFrame.wtl(wo), localFrame.wtl(wi), uv);
}

BSDFSample BSDF::sample(
  const float3& wo,
  const float3& n,
  const float3& t,
  const float2& uv,
  const float2& u,
  float uc,
  float uc2,
  bool regularized
) const {
  Frame localFrame = length2(t) > 0 ? Frame(n, t) : Frame(n);

  auto sample = sampleImpl(localFrame.wtl(wo), uv, u, uc, uc2, regularized);
  sample.wi = localFrame.ltw(sample.wi);
  return sample;
}

}