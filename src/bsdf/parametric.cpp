#include "parametric.hpp"
#include "luts.hpp"

namespace yart {

ParametricBSDF::ParametricBSDF(
  const float3& baseColor,
  const Texture* baseTexture,
  const Texture* mrTexture,
  const Texture* transmissionTexture,
  const Texture* normalTexture,
  const Texture* clearcoatTexture,
  float metallic,
  float roughness,
  float transmission,
  float ior,
  float anisotropic,
  float anisoRotation,
  float clearcoat,
  float clearcoatRoughness,
  const float3& emission,
  float normalScale
) noexcept: m_cTrans(transmission),
            m_cMetallic(metallic),
            m_base(baseColor),
            m_emission(emission),
            m_ior(ior),
            m_roughness(roughness),
            m_anisotropic(anisotropic),
            m_clearcoat(clearcoat),
            m_clearcoatRoughness(clearcoatRoughness),
            m_baseTexture(baseTexture),
            m_mrTexture(mrTexture),
            m_transmissionTexture(transmissionTexture),
            m_clearcoatTexture(clearcoatTexture) {
  m_localRotation = float3x3(float4x4::rotation(-anisoRotation, axis_z<float>));
  m_invRotation = float3x3(float4x4::rotation(anisoRotation, axis_z<float>));
  m_normalTexture = normalTexture;
  m_normalScale = normalScale;
}

float ParametricBSDF::alpha(const float2& uv) const {
  if (m_baseTexture) return m_baseTexture->sample(uv).w();
  return 1.0f;
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
    float3 mr = float3(m_mrTexture->sample(uv));
    r *= mr.y();
    m *= mr.z();
  }
  if (m_transmissionTexture) t *= m_transmissionTexture->sample(uv).x();
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
    float3 mr = float3(m_mrTexture->sample(uv));
    r *= mr.y();
    m *= mr.z();
  }
  if (m_transmissionTexture) t *= m_transmissionTexture->sample(uv).x();
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
    float3 mr = float3(m_mrTexture->sample(uv));
    r *= mr.y();
    m *= mr.z();
  }
  if (m_transmissionTexture) t *= m_transmissionTexture->sample(uv).x();
  if (m_clearcoatTexture) {
    float2 ccr = float2(m_clearcoatTexture->sample(uv));
    c *= ccr.x();
    cr *= ccr.y();
  }

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
      sample = sampleGlossy(wo, base, mf, u, uc);
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
  if (mf.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float3 Fss = fresnelSchlick(base, absDot(wo, wm));
  const float3 fss = Fss * mf.mdf(wm) * mf.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  const float Ess = lut::E_ms(cosTheta_o, m_roughness);
  const float3 fms = fss * base * (1.0f - Ess) / Ess;

  return fss + fms;
}

float ParametricBSDF::pdfMetallic(
  const float3& wo,
  const float3& wi,
  const GGX& mf
) const {
  if (mf.smooth()) return 0;

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  return mf.vmdf(wo, wm) / (4 * absDot(wo, wm));
}

BSDFSample ParametricBSDF::sampleMetallic(
  const float3& wo,
  const float3& base,
  const GGX& mf,
  const float2& u,
  float uc
) const {
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

  float3 wm = mf.sampleVisibleMicrofacet(wo, u);
  float3 wi = reflect(wo, wm);
  if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

  const float pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm));

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  const float3 Fss = fresnelSchlick(base, absDot(wo, wm));
  const float3 Mss = mf.mdf(wm) * Fss * mf.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  const float Ess = lut::E_ms(cosTheta_o, m_roughness);
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
  if (mf.smooth()) return {};

  const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
  const bool isReflection = cosTheta_o * cosTheta_i > 0.0f;
  float ior = 1.0f;
  if (!isReflection) ior = cosTheta_o > 0.0f ? m_ior : 1.0f / m_ior;

  float3 wm = ior * wi + wo;
  if (cosTheta_i == 0.0f || cosTheta_o == 0.0f || length2(wm) == 0.0f)
    return {};

  wm = normalized(wm.z() < 0.0f ? -wm : wm);
  if (dot(wm, wi) * cosTheta_i < 0.0f || dot(wm, wo) * cosTheta_o < 0.0f)
    return {}; // Discard back-facing microsurfaces

  // Single-scattering fresnel
  const float Fss = fresnelDielectric(absDot(wo, wm), ior);
  const float T = 1.0f - Fss;

  const float E_o = lut::ggxGlassE(ior, m_roughness, std::abs(cosTheta_o));

  if (isReflection) {
    // Single-scattering term
    const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                      (4 * cosTheta_o * cosTheta_i);

    return float3(Fss * Mss / E_o);
  } else {
    const float temp = dot(wi, wm) * ior + dot(wo, wm);
    const float dwm_dwi = absDot(wi, wm) * absDot(wo, wm) / (temp * temp);

    const float Tss =
      mf.mdf(wm) * mf.g(wo, wi) * dwm_dwi /
      (std::abs(cosTheta_i * cosTheta_o));

    return T * base * Tss / E_o;
  }
}

float ParametricBSDF::pdfDielectric(
  const float3& wo,
  const float3& wi,
  const GGX& mf
) const {
  if (mf.smooth()) return 0;

  const float cosTheta_o = wo.z(), cosTheta_i = wi.z();
  const bool isReflection = cosTheta_o * cosTheta_i > 0.0f;
  float ior = 1.0f;
  if (!isReflection) ior = cosTheta_o > 0.0f ? m_ior : 1.0f / m_ior;

  float3 wm = ior * wi + wo;
  if (cosTheta_i == 0.0f || cosTheta_o == 0.0f || length2(wm) == 0.0f) return 0;

  wm = normalized(wm.z() < 0.0f ? -wm : wm);
  if (dot(wm, wi) * cosTheta_i < 0.0f || dot(wm, wo) * cosTheta_o < 0.0f)
    return 0; // Discard back-facing microsurfaces

  const float F = fresnelDielectric(dot(wo, wm), m_ior);
  const float T = 1.0f - F;

  float pdf;
  if (isReflection) {
    pdf = mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) * F;
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
  const float ior = wo.z() > 0.0f ? m_ior : 1.0f / m_ior;

  // Handle perfect specular case
  if (mf.smooth()) {
    float F = fresnelDielectric(std::abs(wo.z()), ior);
    float T = 1.0f - F;

    if (uc < F) {
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
      float3 wi;
      if (!refract(wo, axis_z<float>, m_ior, wi)) return {BSDFSample::Absorbed};

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

  float3 wm = mf.sampleVisibleMicrofacet(wo, u);
  const float Fss = fresnelDielectric(absDot(wo, wm), ior);

  const float cosTheta_o = std::abs(wo.z());
  const float E_o = lut::ggxGlassE(ior, m_roughness, cosTheta_o);

  if (uc < Fss) {
    const float3 wi = reflect(wo, wm);
    if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

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
  } else {
    float3 wi;
    const bool tir = !refract(wo, wm, m_ior, wi);
    if (tir || wo.z() * wi.z() > 0.0f || wi.z() == 0.0f)
      return {BSDFSample::Absorbed};

    const float temp = dot(wi, wm) * ior + dot(wo, wm);
    const float dwm_dwi = absDot(wi, wm) / (temp * temp);
    const float pdf = mf.vmdf(wo, wm) * dwm_dwi * (1.0f - Fss);

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
  if (mf.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Dielectric single scattering component
  const float Fss = fresnelDielectric(dot(wo, wm), m_ior);
  const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  // Dielectric multiscatter component
  const float Favg = (m_ior - 1.0f) / (4.08567f + 1.00071f * m_ior);
  const float Eavg = lut::E_msAvg(m_roughness);
  const float Mms = (1.0f - lut::E_ms(cosTheta_o, m_roughness)) *
                    (1.0f - lut::E_ms(cosTheta_i, m_roughness)) /
                    (float(pi) * (1.0f - Eavg));
  const float Fms = Favg * Favg * Eavg / (1.0f - Favg * (1.0f - Eavg));

  // Diffuse component
  const float r = (1.0f - m_ior) / (1.0f + m_ior);
  const float F0 = r * r;
  const float cDiffuse =
    (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_o)) *
    (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_i)) /
    (float(pi) * (1.0f - lut::Eb_msAvg(F0, m_roughness)));
  const float3 diffuse = base * cDiffuse;

  // Final BSDF
  return float3(Fss * Mss + Mms * Fms) + diffuse;
}

float ParametricBSDF::pdfGlossy(
  const float3& wo,
  const float3& wi,
  const GGX& mf
) const {
  if (mf.smooth()) return 0;

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float Fss = fresnelDielectric(dot(wo, wm), m_ior);

  const float FAvg = (m_ior - 1.0f) / (4.08567f + 1.00071f * m_ior);
  const float EmsAvg = lut::E_msAvg(m_roughness);
  const float Fms = FAvg * FAvg * EmsAvg / (1.0f - FAvg * (1.0f - EmsAvg));
  const float Ems_o = lut::E_ms(cosTheta_o, m_roughness);
  const float kappa = 1.0f - (FAvg * Ems_o + Fms * (1.0f - Ems_o));

  return (Fss + Fms) * mf.vmdf(wo, wm) / (4 * absDot(wo, wm)) +
         cosTheta_i * kappa;
}

BSDFSample ParametricBSDF::sampleGlossy(
  const float3& wo,
  const float3& base,
  const GGX& mf,
  const float2& u,
  float uc
) const {
  const float cosTheta_o = wo.z();

  const float Favg = (m_ior - 1.0f) / (4.08567f + 1.00071f * m_ior);
  const float Eavg = lut::E_msAvg(m_roughness);
  const float Fms = Favg * Favg * Eavg / (1.0f - Favg * (1.0f - Eavg));

  const float E_o = lut::E_ms(cosTheta_o, m_roughness);
  const float kappa = 1.0f - (Favg * E_o + Fms * (1.0f - E_o));

  // Diffuse scattering
  if (uc < kappa) {
    float3 wi = samplers::sampleCosineHemisphere(u);
    if (wo.z() < 0) wi *= -1;

    const float cosTheta_i = wi.z();

    const float r = (1.0f - m_ior) / (1.0f + m_ior);
    const float F0 = r * r;
    const float cDiffuse =
      (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_o)) *
      (1.0f - lut::Eb_ms(F0, m_roughness, cosTheta_i)) /
      (float(pi) * (1.0f - lut::Eb_msAvg(F0, m_roughness)));

    return {
      BSDFSample::Reflected | BSDFSample::Diffuse |
      (length2(m_emission) > 0.0f ? BSDFSample::Emitted : 0),
      base * cDiffuse,
      m_emission,
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

  const float Fss = fresnelDielectric(dot(wo, wm), m_ior);
  const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  const float Mms = (1.0f - E_o) *
                    (1.0f - lut::E_ms(cosTheta_i, m_roughness)) /
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
  if (mf.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  // Dielectric single scattering component
  const float Fss = fresnelDielectric(dot(wo, wm), 1.5f);
  const float Mss = mf.mdf(wm) * mf.g(wo, wi) /
                    (4 * cosTheta_o * cosTheta_i);

  // Attenuation fresnel factor
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
  if (mf.smooth()) return 0;

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

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
