#ifndef YART_LUTS_HPP
#define YART_LUTS_HPP

#include <iostream>

#include <math/math.hpp>

namespace yart::lut {
using namespace math;

// LUTs for metallic and glossy GGX materials, from Enterprise PBR spec
// https://dassaultsystemes-technology.github.io/EnterprisePBRShadingModel/spec-2022x.md.html
// Adapted by Sam-Izdat: https://gist.github.com/Sam-Izdat/2bf55f611cd2b02e8b1817a134b040f2
extern const float table_ggx_E[32][32];
extern const float table_ggx_Eavg[32];
extern const float table_ggx_base_E[16][16][16];
extern const float table_ggx_base_Eavg[16][16];

// LUTs for dielectric (transmissive) GGX materials, adapted from Cycles source
// https://projects.blender.org/blender/cycles/src/src/scene/shader.tables
extern const float table_ggx_glass_E[16][16][16];
extern const float table_ggx_glass_Eavg[16][16];
extern const float table_ggx_glass_inv_E[16][16][16];
extern const float table_ggx_glass_inv_Eavg[16][16];

/**
 * LUT lookup for GGX E term for multiscattering: hemisphere integral of
 * irradiance for a given outgoing direction and roughness, no fresnel term
 * @param cosTheta cosine of angle between outgoing direction and surf. normal [0-1]
 * @param r material roughness [0-1]
 * @return irradiance for a perfectly reflective surface over the hemisphere
 */
[[nodiscard]] constexpr float ggxE(float cosTheta, float r) noexcept {
  float ro = r * 31.0f, co = cosTheta * 31.0f;
  size_t ri = min(size_t(ro), 30), cosi = min(size_t(co), 30);
  ro -= float(ri);
  co -= float(cosi);

  const float
    d00 = table_ggx_E[ri][cosi], d01 = table_ggx_E[ri][cosi + 1],
    d10 = table_ggx_E[ri + 1][cosi], d11 = table_ggx_E[ri + 1][cosi + 1];

  return bilerp(d00, d01, d10, d11, ro, co);
}

/**
 * LUT lookup for GGX E_avg term for multiscattering: hemisphere integral of E
 * term over all outgoing directions, no fresnel term
 * @param r material roughness [0-1]
 * @return avg. irradiance for a perfectly reflective surface
 */
[[nodiscard]] constexpr float ggxEavg(float r) noexcept {
  size_t ri = min(size_t(r * 31), 30);
  float ro = r * 31.0f - float(ri);

  return lerp(table_ggx_Eavg[ri], table_ggx_Eavg[ri + 1], ro);
}

/**
 * LUT lookup for GGX "base" E term: hemisphere integral of specular reflection
 * irradiance over all incident directions for a given outgoing light direction,
 * material roughness and fresnel term F0 (ior dependent)
 * @param f0 fresnel term at 0 degrees angle [0-1]
 * @param r material roughness [0-1]
 * @param cosTheta cosine of angle between outgoing direction and surf. normal [0-1]
 * @return specular irradiance over the hemisphere
 */
[[nodiscard]] constexpr float ggxBaseE(
  float f0,
  float r,
  float cosTheta
) noexcept {
  float f0o = f0 * 15.0f;
  float ro = r * 15.0f;
  float co = cosTheta * 15.0f;

  size_t f0i = min(size_t(f0o), 14);
  size_t ri = min(size_t(ro), 14);
  size_t ci = min(size_t(co), 14);

  f0o -= float(f0i);
  ro -= float(ri);
  co -= float(ci);

  std::array<float, 8> vals = {
    table_ggx_base_E[f0i][ri][ci],
    table_ggx_base_E[f0i][ri][ci + 1],
    table_ggx_base_E[f0i][ri + 1][ci],
    table_ggx_base_E[f0i][ri + 1][ci + 1],
    table_ggx_base_E[f0i + 1][ri][ci],
    table_ggx_base_E[f0i + 1][ri][ci + 1],
    table_ggx_base_E[f0i + 1][ri + 1][ci],
    table_ggx_base_E[f0i + 1][ri + 1][ci + 1]
  };

  return trilerp(vals, f0o, ro, co);
}

/**
 * LUT lookup for GGX "base" E_avg term: hemisphere integral of E term over all
 * outgoing directions
 * @param f0 fresnel term at 0 degrees angle [0-1]
 * @param r material roughness [0-1]
 * @return avg. specular irradiance
 */
[[nodiscard]] constexpr float ggxBaseEavg(float f0, float r) noexcept {
  size_t f0i = min(size_t(f0 * 15), 14), ri = min(size_t(r * 15), 14);
  float f0o = f0 * 15.0f - float(f0i), ro = r * 15.0f - float(ri);

  const float
    d00 = table_ggx_base_Eavg[f0i][ri], d01 = table_ggx_base_Eavg[f0i][ri + 1],
    d10 = table_ggx_base_Eavg[f0i + 1][ri], d11 = table_ggx_base_Eavg[f0i + 1][
    ri + 1];

  return bilerp(d00, d01, d10, d11, f0o, ro);
}

/**
 * LUT lookup for GGX dielectric E term: sphere integral of specular reflection
 * irradiance for a given outgoing direction
 * @param ior relative IOR (eta_i / eta_o)
 * @param r material roughness [0-1]
 * @param cosTheta cosine of angle between outgoing direction and surf. normal [0-1]
 * @return specular irradiance over the unit sphere
 */
[[nodiscard]] constexpr float ggxGlassE(
  float ior,
  float r,
  float cosTheta
) noexcept {
  bool inv = ior < 1.0f;
  if (inv) ior = 1.0f / ior;
  float f0 = std::sqrt(std::abs((1.0f - ior) / (1.0f + ior)));

  size_t f0i = min(size_t(f0 * 15), 14);
  size_t ri = min(size_t(r * 15), 14);
  size_t ci = min(size_t(cosTheta * 15), 14);
  float f0o = f0 * 15.0f - float(f0i);
  float ro = r * 15.0f - float(ri);
  float co = cosTheta * 15.0f - float(ci);

  const float(& lut)[16][16][16] = inv ? table_ggx_glass_inv_E
                                       : table_ggx_glass_E;

  std::array<float, 8> vals = {
    lut[f0i][ci][ri],
    lut[f0i][ci][ri + 1],
    lut[f0i][ci + 1][ri],
    lut[f0i][ci + 1][ri + 1],
    lut[f0i + 1][ci][ri],
    lut[f0i + 1][ci][ri + 1],
    lut[f0i + 1][ci + 1][ri],
    lut[f0i + 1][ci + 1][ri + 1]
  };

  float val = trilerp(vals, f0o, co, ro);
  return val;
}

/**
 * LUT lookup for GGX dielectric E_avg term: sphere integral of E term over all
 * outgoing directions
 * @param ior relative IOR (eta_i / eta_o)
 * @param r material roughness [0-1]
 * @return avg. specular irradiance
 */
[[nodiscard]] constexpr float ggxGlassEavg(
  float ior,
  float r
) noexcept {
  bool inv = ior < 1.0f;
  if (inv) ior = 1.0f / ior;
  float f0 = std::sqrt(std::abs((1.0f - ior) / (1.0f + ior)));

  size_t f0i = min(size_t(f0 * 15), 14);
  size_t ri = min(size_t(r * 15), 14);
  float f0o = f0 * 15.0f - float(f0i);
  float ro = r * 15.0f - float(ri);

  const float(& lut)[16][16] = inv ? table_ggx_glass_inv_Eavg
                                   : table_ggx_glass_Eavg;

  float d00 = lut[f0i][ri], d01 = lut[f0i][ri + 1],
    d10 = lut[f0i + 1][ri], d11 = lut[f0i + 1][ri + 1];

  return bilerp(d00, d01, d10, d11, f0o, ro);
}

}

#endif //YART_LUTS_HPP
