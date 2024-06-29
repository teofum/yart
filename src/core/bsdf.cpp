#include "bsdf.hpp"

namespace yart {

float3 BSDF::f(
  const float3& wo,
  const float3& wi,
  const float3& n,
  const float3& t,
  const float3& st,
  const float2& uv
) const {
  float3 sn = n;

  if (m_normalTexture) {
    float3 sampledNormal = float3(m_normalTexture->sample(uv)) * 2.0f - 1.0f;
    Frame normalMapFrame = Frame(sn, st);
    sn = normalized(normalMapFrame.ltw(sampledNormal));
  }

  Frame localFrame = length2(t) > 0 ? Frame(sn, t) : Frame(sn);
  return fImpl(localFrame.wtl(wo), localFrame.wtl(wi), uv);
}

float BSDF::pdf(
  const float3& wo,
  const float3& wi,
  const float3& n,
  const float3& t,
  const float3& st,
  const float2& uv
) const {
  float3 sn = n;

  if (m_normalTexture) {
    float3 sampledNormal = float3(m_normalTexture->sample(uv)) * 2.0f - 1.0f;
    Frame normalMapFrame = Frame(sn, st);
    sn = normalized(normalMapFrame.ltw(sampledNormal));
  }

  Frame localFrame = length2(t) > 0 ? Frame(sn, t) : Frame(sn);
  return pdfImpl(localFrame.wtl(wo), localFrame.wtl(wi), uv);
}

BSDFSample BSDF::sample(
  const float3& wo,
  const float3& n,
  const float3& t,
  const float3& st,
  const float2& uv,
  const float2& u,
  float uc,
  float uc2,
  bool regularized
) const {
  float3 sn = n;

  if (m_normalTexture) {
    float3 sampledNormal = float3(m_normalTexture->sample(uv)) * 2.0f - 1.0f;
    Frame normalMapFrame = Frame(sn, st);
    sn = normalized(normalMapFrame.ltw(sampledNormal));
  }

  Frame localFrame = length2(t) > 0 ? Frame(sn, t) : Frame(sn);

  auto sample = sampleImpl(localFrame.wtl(wo), uv, u, uc, uc2, regularized);
  sample.wi = localFrame.ltw(sample.wi);
  return sample;
}

}