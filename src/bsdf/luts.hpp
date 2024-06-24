#ifndef YART_LUTS_HPP
#define YART_LUTS_HPP

#include <iostream>

#include <math/math.hpp>

namespace yart::lut {
using namespace math;

extern const float E_msLut[32][32];
extern const float E_msAvgLut[32];

extern const float Eb_msLut[16][16][16];
extern const float Eb_msAvgLut[16][16];

extern const float table_ggx_glass_E[16][16][16];
extern const float table_ggx_glass_Eavg[16][16];
extern const float table_ggx_glass_inv_E[16][16][16];
extern const float table_ggx_glass_inv_Eavg[16][16];

[[nodiscard]] constexpr float E_ms(float cosTheta, float r) noexcept {
  size_t ri = min(size_t(r * 31), 30), cosi = min(size_t(cosTheta * 31), 30);
  float ro = r * 31.0f - float(ri), co = cosTheta * 31.0f - float(cosi);

  const float
    d00 = E_msLut[ri][cosi], d01 = E_msLut[ri][cosi + 1],
    d10 = E_msLut[ri + 1][cosi], d11 = E_msLut[ri + 1][cosi + 1];

  return bilerp(d00, d01, d10, d11, ro, co);
}

[[nodiscard]] constexpr float E_msAvg(float r) noexcept {
  size_t ri = min(size_t(r * 31), 30);
  float ro = r * 31.0f - float(ri);

  return lerp(E_msAvgLut[ri], E_msAvgLut[ri + 1], ro);
}

[[nodiscard]] constexpr float Eb_ms(
  float f0,
  float r,
  float cosTheta
) noexcept {
  size_t f0i = min(size_t(f0 * 15), 14);
  size_t ri = min(size_t(r * 15), 14);
  size_t ci = min(size_t(cosTheta * 15), 14);
  float f0o = f0 * 15.0f - float(f0i);
  float ro = r * 15.0f - float(ri);
  float co = cosTheta * 15.0f - float(ci);

  std::array<float, 8> vals = {
    Eb_msLut[f0i][ri][ci],
    Eb_msLut[f0i][ri][ci + 1],
    Eb_msLut[f0i][ri + 1][ci],
    Eb_msLut[f0i][ri + 1][ci + 1],
    Eb_msLut[f0i + 1][ri][ci],
    Eb_msLut[f0i + 1][ri][ci + 1],
    Eb_msLut[f0i + 1][ri + 1][ci],
    Eb_msLut[f0i + 1][ri + 1][ci + 1]
  };

  return trilerp(vals, f0o, ro, co);
}

[[nodiscard]] constexpr float Eb_msAvg(float f0, float r) noexcept {
  size_t f0i = min(size_t(f0 * 15), 14), ri = min(size_t(r * 15), 14);
  float f0o = f0 * 15.0f - float(f0i), ro = r * 15.0f - float(ri);

  const float
    d00 = Eb_msAvgLut[f0i][ri], d01 = Eb_msAvgLut[f0i][ri + 1],
    d10 = Eb_msAvgLut[f0i + 1][ri], d11 = Eb_msAvgLut[f0i + 1][ri + 1];

  return bilerp(d00, d01, d10, d11, f0o, ro);
}

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
