#include "parametric.hpp"
#include "luts.hpp"

namespace yart {

// Average Fresnel term over all angles for a given IOR, curve fit from KC2017
static constexpr float FavgFit(float ior) noexcept {
  return (ior - 1.0f) / (4.08567f + 1.00071f * ior);
}

ParametricBSDF::ParametricBSDF(
  const float3& baseColor,
  const RGBATexture* baseTexture,
  const SDRTexture<2>* mrTexture,
  const MonoTexture* transmissionTexture,
  const RGBTexture* normalTexture,
  const MonoTexture* clearcoatTexture,
  const RGBTexture* emissionTexture,
  float metallic,
  float roughness,
  float transmission,
  float ior,
  float anisotropic,
  float anisoRotation,
  float clearcoat,
  float clearcoatRoughness,
  const float3& emission,
  float normalScale,
  bool thinTransmission
) noexcept: m_cTrans(transmission),
            m_cMetallic(metallic),
            m_base(baseColor),
            m_emission(emission),
            m_ior(ior),
            m_roughness(roughness),
            m_anisotropic(anisotropic),
            m_clearcoat(clearcoat),
            m_clearcoatRoughness(clearcoatRoughness),
            m_thinTransmission(thinTransmission),
            m_baseTexture(baseTexture),
            m_mrTexture(mrTexture),
            m_transmissionTexture(transmissionTexture),
            m_clearcoatTexture(clearcoatTexture),
            m_emissionTexture(emissionTexture) {
  // Create rotation matrices for anisotropy
  m_localRotation = float3x3(float4x4::rotation(-anisoRotation, axis_z<float>));
  m_invRotation = float3x3(float4x4::rotation(anisoRotation, axis_z<float>));

  // Set base BSDF properties
  m_normalTexture = normalTexture;
  m_normalScale = normalScale;

  // Check base texture and set alpha flag, this lets us skip expensive texture
  // sampling in hot intersection code for most materials
  if (m_baseTexture)
    for (uint32_t i = 3; i < m_baseTexture->data.size(); i += 4)
      if (m_baseTexture->data[i] < 255) m_hasAlpha = true;

  // Set emission flag based on emission strength
  // Note partially emissive materials (ie with an emission texture) are considered
  // fully emissive for sampling purposes, and some samples will be wasted
  m_hasEmission = length2(m_emission) > 0.0f;
}

float ParametricBSDF::alpha(const float2& uv) const {
  // Skip texture sampling if we know the material has no alpha blending
  if (m_hasAlpha && m_baseTexture) return m_baseTexture->sample(uv).w();
  return 1.0f;
}

float3 ParametricBSDF::base(const float2& uv) const {
  if (m_baseTexture) return m_base * float3(m_baseTexture->sample(uv));
  return m_base;
}

bool ParametricBSDF::transparent() const {
  return m_thinTransmission && m_cTrans > 0.0f;
}

float3 ParametricBSDF::fImpl(
  const float3& _wo,
  const float3& _wi,
  const float2& uv
) const {
  // Sample textures
  float3 base = m_base;
  if (m_baseTexture) base *= float3(m_baseTexture->sample(uv));

  float r = m_roughness, m = m_cMetallic, t = m_cTrans;
  float c = m_clearcoat, cr = m_clearcoatRoughness;
  if (m_mrTexture) {
    float2 mr = m_mrTexture->sample(uv);
    r *= mr.x();
    m *= mr.y();
  }
  if (m_transmissionTexture) t *= m_transmissionTexture->sample(uv);
  if (m_clearcoatTexture) {
    float2 ccr = float2(m_clearcoatTexture->sample(uv));
    c *= ccr.x();
    cr *= ccr.y();
  }

  // Init GGX microfacet distribution for sampled roughness
  GGX mf(r, m_anisotropic);

  // Calculate coeffs
  const float cMetallic = m;
  const float cDielectric = (1.0f - m) * t;
  const float cGlossy = (1.0f - m) * (1.0f - t);

  // Rotate wo/wi for anisotropy rotation
  float3 wo = m_localRotation * _wo, wi = m_localRotation * _wi;

  // Evaluate the BSDF
  float3 val;
  if (cMetallic > 0.0f) val += cMetallic * fMetallic(wo, wi, base, mf);
  if (cDielectric > 0.0f) val += cDielectric * fDielectric(wo, wi, base, mf);
  if (cGlossy > 0.0f) val += cGlossy * fGlossy(wo, wi, base, mf);

  // Add clearcoat, if any
  if (c > 0.0f) {
    GGX mfClearcoat(cr);
    float Fc = 0.0f;
    float3 valClear = fClearcoat(wo, wi, mfClearcoat, &Fc);
    val = (1.0f - c * Fc) * val + c * valClear;
  }

  return val;
}

float ParametricBSDF::pdfImpl(
  const float3& wo,
  const float3& wi,
  const float2& uv
) const {
  // Sample textures
  float r = m_roughness, m = m_cMetallic, t = m_cTrans;
  float c = m_clearcoat, cr = m_clearcoatRoughness;
  if (m_mrTexture) {
    float2 mr = m_mrTexture->sample(uv);
    r *= mr.x();
    m *= mr.y();
  }
  if (m_transmissionTexture) t *= m_transmissionTexture->sample(uv);
  if (m_clearcoatTexture) {
    float2 ccr = float2(m_clearcoatTexture->sample(uv));
    c *= ccr.x();
    cr *= ccr.y();
  }

  // Init GGX microfacet distribution for sampled roughness
  GGX mf(r, m_anisotropic);

  // Calculate probabilities of sampling each lobe
  const float pMetallic = m;
  const float pDielectric = (1.0f - m) * t;
  const float pGlossy = (1.0f - m) * (1.0f - t);

  float pdf = 0.0f;
  if (pMetallic > 0.0f) pdf += pMetallic * pdfMetallic(wo, wi, mf);
  if (pDielectric > 0.0f) pdf += pDielectric * pdfDielectric(wo, wi, mf);
  if (pGlossy > 0.0f) pdf += pGlossy * pdfGlossy(wo, wi, mf);

  // Mix clearcoat pdf, if any
  if (c > 0.0f) {
    GGX mfClearcoat(cr);
    float Fc = 0.0f;
    float pdfClear = pdfClearcoat(wo, wi, mfClearcoat, &Fc);
    pdf = (1.0f - c * Fc) * pdf + c * pdfClear;
  }

  return pdf;
}

BSDFSample ParametricBSDF::sampleImpl(
  const float3& _wo,
  const float2& uv,
  const float2& u,
  float uc,
  float uc2,
  bool regularized
) const {
  // Sample textures
  float3 base = m_base;
  if (m_baseTexture) base *= float3(m_baseTexture->sample(uv));

  float r = m_roughness, m = m_cMetallic, t = m_cTrans;
  float c = m_clearcoat, cr = m_clearcoatRoughness;
  if (m_mrTexture) {
    float2 mr = m_mrTexture->sample(uv);
    r *= mr.x();
    m *= mr.y();
  }
  if (m_transmissionTexture) t *= m_transmissionTexture->sample(uv);
  if (m_clearcoatTexture) {
    float2 ccr = float2(m_clearcoatTexture->sample(uv));
    c *= ccr.x();
    cr *= ccr.y();
  }

  // Increase roughness for near-specular materials for regularized paths
  // Helps reduce noise on diffuse -> specular -> light source bounces
  if (regularized) {
    r = roughen(r);
    cr = roughen(cr);
  }

  // Calculate probabilities of sampling each lobe
  const float pClearcoat = c * fresnelDielectric(std::abs(_wo.z()), 1.5f);
  const float pMetallic = (1.0f - c) * m;
  const float pDielectric = (1.0f - c) * (m + (1.0f - m) * t);

  // Sample the BSDF
  BSDFSample sample;
  if (uc2 < pClearcoat) {
    // Init GGX microfacet distribution for sampled clearcoat roughness
    GGX mfClearcoat(cr);

    sample = sampleClearcoat(_wo, mfClearcoat, u, uc);
  } else {
    // Init GGX microfacet distribution for sampled roughness
    GGX mf(r, m_anisotropic);

    // Rotate wo for anisotropy rotation
    float3 wo = m_localRotation * _wo;

    if (uc2 < pMetallic) {
      sample = sampleMetallic(wo, base, mf, u, uc);
    } else if (uc2 < pDielectric) {
      sample = sampleDielectric(wo, base, mf, u, uc);
    } else {
      float3 emission = m_emission;
      if (m_hasEmission && m_emissionTexture) {
        emission *= float3(m_emissionTexture->sample(uv));
      }
      sample = sampleGlossy(wo, base, emission, mf, u, uc);
    }

    // Rotate wi for anisotropy rotation
    sample.wi = m_invRotation * sample.wi;
  }

  return sample;
}

float3 ParametricBSDF::fMetallic(
  const float3& wo,
  const float3& wi,
  const float3& base,
  const GGX& mf
) const {
  // Handle perfect specular case, f = 0 (delta dirac distrib.)
  if (mf.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  // Get microfacet normal wm and make sure it's outward facing
  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Single-scattering fresnel term and BSDF (Cook-Torrance)
  const float3 Fss = fresnelSchlick(base, absDot(wo, wm));
  const float3 Mss = Fss * mf.mdf(wm) * mf.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  // Multi-scattering BSDF term (E. Turquin)
  const float Ess = lut::ggxE(cosTheta_o, m_roughness);
  const float3 Mms = Mss * base * (1.0f - Ess) / Ess;

  return Mss + Mms;
}

float ParametricBSDF::pdfMetallic(
  const float3& wo,
  const float3& wi,
  const GGX& mf
) const {
  // Handle perfect specular case, p = 0 (delta dirac distrib.)
  if (mf.smooth()) return 0;

  // Get microfacet normal wm and make sure it's outward facing
  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Eval visible microfacet distribution
  return mf.vmdf(wo, wm) / (4 * absDot(wo, wm));
}

BSDFSample ParametricBSDF::sampleMetallic(
  const float3& wo,
  const float3& base,
  const GGX& mf,
  const float2& u,
  float uc
) const {
  // Handle perfect specular case
  if (mf.smooth()) {
    const float3 F = fresnelSchlick(base, wo.z());

    return {
      BSDFSample::Reflected | BSDFSample::Specular,
      F / std::abs(wo.z()),
      float3(),
      float3(-wo.x(), -wo.y(), wo.z()),
      1.0f,
      0.0f
    };
  }

  // Sample GGX VMDF and calculate incident light direction
  float3 wm = mf.sampleVisibleMicrofacet(wo, u);
  float3 wi = reflect(wo, wm);
  if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

  const float pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm));

  // Single-scattering fresnel term and BSDF (Cook-Torrance)
  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  const float3 Fss = fresnelSchlick(base, absDot(wo, wm));
  const float3 Mss = Fss * mf.mdf(wm) * mf.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  // Multi-scattering BSDF term (E. Turquin)
  const float Ess = lut::ggxE(cosTheta_o, m_roughness);
  const float3 Mms = Mss * base * (1.0f - Ess) / Ess;

  return {
    BSDFSample::Reflected | BSDFSample::Glossy,
    Mss + Mms,
    float3(),
    wi,
    pdf,
    m_roughness
  };
}

float3 ParametricBSDF::fDielectric(
  const float3& wo,
  const float3& wi,
  const float3& base,
  const GGX& mf
) const {
  // Handle perfect specular case, f = 0 (delta dirac distrib.)
  if (mf.smooth()) return {};

  // Check for reflection/refraction, set relative ior if refraction
  const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
  const bool isReflection = cosTheta_o * cosTheta_i > 0.0f;
  float ior = 1.0f;
  // TODO: assumes outside interface is air, should consider volumes
  if (!isReflection) ior = cosTheta_o > 0.0f ? m_ior : 1.0f / m_ior;

  // Calculate microfacet normal
  float3 wm = ior * wi + wo;
  if (cosTheta_i == 0.0f || cosTheta_o == 0.0f || length2(wm) == 0.0f)
    return {};

  wm = normalized(wm.z() < 0.0f ? -wm : wm);
  if (dot(wm, wi) * cosTheta_i < 0.0f || dot(wm, wo) * cosTheta_o < 0.0f)
    return {}; // Discard back-facing microsurfaces

  // Single-scattering fresnel
  const float Fss = fresnelDielectric(absDot(wo, wm), ior);
  const float T = 1.0f - Fss;

  // Multi-scatter compensation term (E. Turquin)
  const float E_o = lut::ggxGlassE(ior, m_roughness, std::abs(cosTheta_o));

  if (isReflection) {
    // Single-scattering term
    const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                      (4 * cosTheta_o * cosTheta_i);

    return float3(Fss * Mss / E_o);
  } else if (m_thinTransmission) {
    // Handle thin transmission case
    float3 wip = reflect(-wi, axis_z<float>);
    wm = normalized(wip + wo);
    const float cosTheta_ip = std::abs(wip.z());

    // Single-scattering term
    const float Tss = mf.mdf(wm) * mf.g(wo, wip) /
                      (4 * cosTheta_o * cosTheta_ip);

    return T * base * Tss / E_o;
  } else {
    const float temp = dot(wi, wm) * ior + dot(wo, wm);
    const float dwm_dwi = absDot(wi, wm) * absDot(wo, wm) / (temp * temp);

    // Single-scattering term
    const float Tss = mf.mdf(wm) * mf.g(wo, wi) * dwm_dwi /
                      (std::abs(cosTheta_i * cosTheta_o));

    return T * base * Tss / E_o;
  }
}

float ParametricBSDF::pdfDielectric(
  const float3& wo,
  const float3& wi,
  const GGX& mf
) const {
  // Handle perfect specular case, p = 0 (delta dirac distrib.)
  if (mf.smooth()) return 0;

  // Check for reflection/refraction, set relative ior if refraction
  const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
  const bool isReflection = cosTheta_o * cosTheta_i > 0.0f;
  float ior = 1.0f;
  if (!isReflection) ior = cosTheta_o > 0.0f ? m_ior : 1.0f / m_ior;

  // Calculate microfacet normal
  float3 wm = ior * wi + wo;
  if (cosTheta_i == 0.0f || cosTheta_o == 0.0f || length2(wm) == 0.0f) return 0;

  wm = normalized(wm.z() < 0.0f ? -wm : wm);
  if (dot(wm, wi) * cosTheta_i < 0.0f || dot(wm, wo) * cosTheta_o < 0.0f)
    return 0; // Discard back-facing microsurfaces

  // Single-scattering fresnel term
  const float F = fresnelDielectric(dot(wo, wm), m_ior);
  const float T = 1.0f - F;

  float pdf;
  if (isReflection) {
    pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) * F;
  } else if (m_thinTransmission) {
    // TODO: check this is correct
    float3 wip = reflect(-wi, axis_z<float>);
    wm = normalized(wip + wo);

    pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) * T;
  } else {
    const float temp = dot(wi, wm) + dot(wo, wm) / ior;
    const float dwm_dwi = absDot(wo, wm) / (temp * temp);
    pdf = mf.vmdf(wo, wm) * dwm_dwi * T;
  }
  return pdf;
}

BSDFSample ParametricBSDF::sampleDielectric(
  const float3& wo,
  const float3& base,
  const GGX& mf,
  const float2& u,
  float uc
) const {
  // Get relative IOR
  const float ior = m_thinTransmission || wo.z() > 0.0f ? m_ior : 1.0f / m_ior;

  // Handle perfect specular case
  if (mf.smooth()) {
    float F = fresnelDielectric(std::abs(wo.z()), ior);
    float T = 1.0f - F;

    if (uc < F) {
      // Specular reflection
      float3 wi(-wo.x(), -wo.y(), wo.z());

      return {
        BSDFSample::Reflected | BSDFSample::Specular,
        float3(F / std::abs(wi.z())),
        float3(),
        wi,
        F,
        0.0f
      };
    } else {
      // Specular refraction
      float3 wi;
      if (m_thinTransmission) wi = -wo;
      else if (!refract(wo, axis_z<float>, m_ior, wi))
        return {BSDFSample::Absorbed};

      return {
        BSDFSample::Transmitted | BSDFSample::Specular,
        T * base / std::abs(wi.z()),
        float3(),
        wi,
        T,
        0.0f
      };
    }
  }

  // Sample GGX VMDF, calculate single-scattering fresnel term
  float3 wm = mf.sampleVisibleMicrofacet(wo, u);
  const float Fss = fresnelDielectric(absDot(wo, wm), ior);

  // Multi-scatter compensation term (E. Turquin)
  const float cosTheta_o = std::abs(wo.z());
  const float E_o = lut::ggxGlassE(ior, m_roughness, cosTheta_o);

  if (uc < Fss) {
    // Calculate reflected light direction
    const float3 wi = reflect(wo, wm);
    if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

    // Single-scattering BSDF term
    const float cosTheta_i = std::abs(wi.z());
    const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                      (4 * cosTheta_o * cosTheta_i);

    const float pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) * Fss;

    return {
      BSDFSample::Reflected | BSDFSample::Glossy,
      float3(Fss * Mss / E_o),
      float3(),
      wi,
      pdf,
      m_roughness
    };
  } else if (m_thinTransmission) {
    // Handle thin transmission case
    // TODO: check this is correct
    const float3 wi = reflect(wo, wm) * float3(1, 1, -1);

    const float cosTheta_i = std::abs(wi.z());
    const float Tss = mf.mdf(wm) * mf.g(wo, wi) / (4 * cosTheta_o * cosTheta_i);

    const float pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) * (1.0f - Fss);

    return {
      BSDFSample::Transmitted | BSDFSample::Glossy,
      (1.0f - Fss) * Tss * base / E_o,
      float3(),
      wi,
      pdf,
      m_roughness
    };
  } else {
    // Calculate refracted light direction and handle total internal reflection
    float3 wi;
    const bool tir = !refract(wo, wm, m_ior, wi);
    if (tir || wo.z() * wi.z() > 0.0f || wi.z() == 0.0f)
      return {BSDFSample::Absorbed};

    const float temp = dot(wi, wm) * ior + dot(wo, wm);
    const float dwm_dwi = absDot(wi, wm) / (temp * temp);
    const float pdf = mf.vmdf(wo, wm) * dwm_dwi * (1.0f - Fss);

    // Single-scattering BSDF
    const float Tss =
      mf.mdf(wm) * mf.g(wo, wi) * std::abs(
        dot(wi, wm) * dot(wo, wm) / (wi.z() * wo.z() * temp * temp)
      );

    return {
      BSDFSample::Transmitted | BSDFSample::Glossy,
      (1.0f - Fss) * Tss * base / E_o,
      float3(),
      wi,
      pdf,
      m_roughness
    };
  }
}

float3 ParametricBSDF::fGlossy(
  const float3& wo,
  const float3& wi,
  const float3& base,
  const GGX& mf
) const {
  // Handle perfect specular case, f = 0 (delta dirac distrib.)
  if (mf.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Dielectric single-scattering component (Cook-Torrance)
  const float Fss = fresnelDielectric(dot(wo, wm), m_ior);
  const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  // Dielectric multi-scattering component (C. Kulla, A. Conty)
  const float Favg = FavgFit(m_ior);
  const float Eavg = lut::ggxEavg(m_roughness);
  const float Mms = (1.0f - lut::ggxE(cosTheta_o, m_roughness)) *
                    (1.0f - lut::ggxE(cosTheta_i, m_roughness)) /
                    (float(pi) * (1.0f - Eavg));
  const float Fms = Favg * Favg * Eavg / (1.0f - Favg * (1.0f - Eavg));

  // Diffuse component, compensated for energy conservation (C. Kulla, A. Conty)
  const float r = (1.0f - m_ior) / (1.0f + m_ior);
  const float F0 = r * r;
  const float cDiffuse =
    (1.0f - lut::ggxBaseE(F0, m_roughness, cosTheta_o)) *
    (1.0f - lut::ggxBaseE(F0, m_roughness, cosTheta_i)) /
    (float(pi) * (1.0f - lut::ggxBaseEavg(F0, m_roughness)));
  const float3 diffuse = base * cDiffuse;

  // Final BSDF
  return float3(Fss * Mss + Mms * Fms) + diffuse;
}

float ParametricBSDF::pdfGlossy(
  const float3& wo,
  const float3& wi,
  const GGX& mf
) const {
  // Handle perfect specular case, p = 0 (delta dirac distrib.)
  if (mf.smooth()) return 0;

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Single-scattering fresnel
  const float Fss = fresnelDielectric(dot(wo, wm), m_ior);

  // Diffuse term, compensated for energy conservation
  const float Favg = FavgFit(m_ior);
  const float EmsAvg = lut::ggxEavg(m_roughness);
  const float Fms = Favg * Favg * EmsAvg / (1.0f - Favg * (1.0f - EmsAvg));
  const float Ems_o = lut::ggxE(cosTheta_o, m_roughness);
  const float kappa = 1.0f - (Favg * Ems_o + Fms * (1.0f - Ems_o));

  return (Fss + Fms) * mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) +
         cosTheta_i * kappa;
}

BSDFSample ParametricBSDF::sampleGlossy(
  const float3& wo,
  const float3& base,
  const float3& emission,
  const GGX& mf,
  const float2& u,
  float uc
) const {
  const float cosTheta_o = wo.z();

  // Multi-scattering fresnel (C. Kulla, A. Conty)
  const float Favg = FavgFit(m_ior);
  const float Eavg = lut::ggxEavg(m_roughness);
  const float Fms = Favg * Favg * Eavg / (1.0f - Favg * (1.0f - Eavg));

  // Diffuse scattering compensation term
  const float E_o = lut::ggxE(cosTheta_o, m_roughness);
  const float kappa = 1.0f - (Favg * E_o + Fms * (1.0f - E_o));

  // Diffuse scattering
  if (uc < kappa) {
    float3 wi = samplers::sampleCosineHemisphere(u);
    if (wo.z() < 0) wi *= -1;

    const float cosTheta_i = wi.z();

    const float r = (1.0f - m_ior) / (1.0f + m_ior);
    const float F0 = r * r;
    const float cDiffuse =
      (1.0f - lut::ggxBaseE(F0, m_roughness, cosTheta_o)) *
      (1.0f - lut::ggxBaseE(F0, m_roughness, cosTheta_i)) /
      (float(pi) * (1.0f - lut::ggxBaseEavg(F0, m_roughness)));

    return {
      BSDFSample::Reflected | BSDFSample::Diffuse |
      (length2(emission) > 0.0f ? BSDFSample::Emitted : 0),
      base * cDiffuse,
      emission,
      wi,
      std::abs(wi.z()) * cDiffuse,
      1.0f
    };
  }

  // Handle perfect specular case
  if (mf.smooth()) {
    const float F = fresnelDielectric(wo.z(), m_ior);
    float3 wi(-wo.x(), -wo.y(), wo.z());

    return {
      BSDFSample::Reflected | BSDFSample::Specular,
      float3(F / std::abs(wi.z())),
      float3(),
      wi,
      F,
      0.0f
    };
  }

  // Rough (glossy) reflection
  float3 wm = mf.sampleVisibleMicrofacet(wo, u);
  const float3 wi = reflect(wo, wm);
  const float cosTheta_i = wi.z();
  if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

  // Single-scattering fresnel and BSDF
  const float Fss = fresnelDielectric(dot(wo, wm), m_ior);
  const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  // Multi-scattering BSDF (C. Kulla, A. Conty)
  const float Mms = (1.0f - E_o) * (1.0f - lut::ggxE(cosTheta_i, m_roughness)) /
                    (float(pi) * (1.0f - Eavg));

  const float pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) * Fss;

  return {
    BSDFSample::Reflected | BSDFSample::Glossy,
    float3(Fss * Mss + Fms * Mms),
    float3(),
    wi,
    pdf,
    m_roughness
  };
}

float3 ParametricBSDF::fClearcoat(
  const float3& wo,
  const float3& wi,
  const GGX& mf,
  float* Fc
) const {
  // Handle perfect specular case, f = 0 (delta dirac distrib.)
  if (mf.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Dielectric single-scattering component
  const float Fss = fresnelDielectric(dot(wo, wm), 1.5f);
  const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  // Underlying material attenuation fresnel factor
  *Fc = max(
    fresnelDielectric(cosTheta_o, 1.5f),
    fresnelDielectric(cosTheta_i, 1.5f)
  );

  // Final BSDF
  return float3(Fss * Mss);
}

float ParametricBSDF::pdfClearcoat(
  const float3& wo,
  const float3& wi,
  const GGX& mf,
  float* Fc
) const {
  // Handle perfect specular case, p = 0 (delta dirac distrib.)
  if (mf.smooth()) return 0;

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Dielectric single-scattering component
  const float Fss = fresnelDielectric(dot(wo, wm), 1.5f);

  // Attenuation fresnel factor
  *Fc = max(
    fresnelDielectric(std::abs(wo.z()), 1.5f),
    fresnelDielectric(std::abs(wi.z()), 1.5f)
  );

  return Fss * mf.vmdf(wo, wm) / (4 * absDot(wo, wm));
}

BSDFSample ParametricBSDF::sampleClearcoat(
  const float3& wo,
  const GGX& mf,
  const float2& u,
  float uc
) const {
  const float cosTheta_o = wo.z();

  // Handle perfect specular case
  if (mf.smooth()) {
    const float F = fresnelDielectric(wo.z(), m_ior);
    float3 wi(-wo.x(), -wo.y(), wo.z());

    return {
      BSDFSample::Reflected | BSDFSample::Specular,
      float3(F / std::abs(wi.z())),
      float3(),
      wi,
      F,
      0.0f
    };
  }

  // Rough (glossy) reflection
  float3 wm = mf.sampleVisibleMicrofacet(wo, u);
  const float3 wi = reflect(wo, wm);
  const float cosTheta_i = wi.z();
  if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

  // Single-scattering fresnel term and BSDF
  // Clearcoat is not energy compensated for multiple scattering
  const float Fss = fresnelDielectric(dot(wo, wm), 1.5f);
  const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  const float pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) *
                    fresnelDielectric(cosTheta_o, 1.5f);

  return {
    BSDFSample::Reflected | BSDFSample::Glossy,
    float3(Fss * Mss),
    float3(),
    wi,
    pdf,
    m_roughness
  };
}

}
