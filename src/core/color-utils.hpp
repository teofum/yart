#ifndef YART_COLOR_UTILS_HPP
#define YART_COLOR_UTILS_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

constexpr float sRGB_Gamma = 2.4f;
constexpr float sRGB_InvGamma = 1.0f / sRGB_Gamma;

[[nodiscard]] constexpr float sRGBDecode(float val) noexcept {
  if (val <= 0.04045f) return val / 12.92f;
  return std::pow((val + 0.055f) / 1.055f, sRGB_Gamma);
}

[[nodiscard]] constexpr float sRGBEncode(float val) noexcept {
  if (val < 0.0031308) return val * 12.92f;
  return std::pow(val, sRGB_InvGamma) * 1.055 - 0.055;
}

/**
 * sRGB decode. Doesn't affect the alpha channel for RGBA (float4) vectors.
 */
template<std::size_t N>
[[nodiscard]] constexpr vec<float, N> sRGBDecode(const vec<float, N>& val) noexcept {
  vec<float, N> ret(val);
  for (uint32_t i = 0; i < min(3, N); i++) ret[i] = sRGBDecode(val[i]);
  return ret;
}

/**
 * sRGB encode. Doesn't affect the alpha channel for RGBA (float4) vectors.
 */
template<std::size_t N>
[[nodiscard]] constexpr vec<float, N> sRGBEncode(const vec<float, N>& val) noexcept {
  vec<float, N> ret(val);
  for (uint32_t i = 0; i < min(3, N); i++) ret[i] = sRGBEncode(val[i]);
  return ret;
}

}

#endif //YART_COLOR_UTILS_HPP
