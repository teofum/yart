#ifndef YART_COLOR_UTILS_HPP
#define YART_COLOR_UTILS_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

constexpr float sRGB_Gamma = 2.4f;
constexpr float sRGB_InvGamma = 1.0f / sRGB_Gamma;

/**
 * sRGB decode. Doesn't affect the alpha channel for RGBA (float4) vectors.
 */
template<std::size_t N>
[[nodiscard]] constexpr vec<float, N> sRGBDecode(const vec<float, N>& val) noexcept {
  vec<float, N> ret(val);
  for (uint32_t i = 0; i < min(3, N); i++) {
    if (val[i] <= 0.04045f) ret[i] = val[i] / 12.92f;
    else ret[i] = std::pow((val[i] + 0.055f) / 1.055f, sRGB_Gamma);
  }
  return ret;
}

/**
 * sRGB encode. Doesn't affect the alpha channel for RGBA (float4) vectors.
 */
template<std::size_t N>
[[nodiscard]] constexpr vec<float, N> sRGBEncode(const vec<float, N>& val) noexcept {
  vec<float, N> ret(val);
  for (uint32_t i = 0; i < min(3, N); i++) {
    if (val[i] < 0.0031308) ret[i] = val[i] * 12.92f;
    else ret[i] = std::pow(val[i], sRGB_InvGamma) * 1.055 - 0.055;
  }
  return ret;
}

}

#endif //YART_COLOR_UTILS_HPP
