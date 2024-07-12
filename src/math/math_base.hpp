#ifndef YART_MATH_BASE_HPP
#define YART_MATH_BASE_HPP

#include <array>
#include <string>
#include <sstream>
#include <functional>

namespace yart::math {

constexpr double epsilon = 1e-12;
constexpr double pi = M_PI;
constexpr double invPi = 1.0 / pi;

constexpr float oneMinusEpsilon = 0x1.fffffep-1;

template<typename T>
concept numeric = std::integral<T> || std::floating_point<T>;

template<numeric T>
struct interval {
  T min, max;
};

template<std::floating_point T>
[[nodiscard]] constexpr T degrees(T radians) noexcept {
  return radians / T(pi) * T(180.0);
}

template<std::floating_point T>
[[nodiscard]] constexpr T radians(T degrees) noexcept {
  return degrees / T(180.0) * T(pi);
}

template<typename T, std::floating_point T_t>
[[nodiscard]] constexpr T lerp(const T& a, const T& b, T_t t) noexcept {
  return (T_t(1.0) - t) * a + t * b;
}

template<typename T, std::floating_point T_t>
[[nodiscard]] constexpr T invLerp(const T& a, const T& b, T_t t) noexcept {
  return (t - a) / (b - a);
}

template<typename T, std::floating_point T_uv>
[[nodiscard]] constexpr T bilerp(
  const T& a0,
  const T& a1,
  const T& b0,
  const T& b1,
  T_uv u,
  T_uv v
) noexcept {
  return a0 * (T_uv(1.0) - u) * (T_uv(1.0) - v) +
         a1 * (T_uv(1.0) - u) * v +
         b0 * u * (T_uv(1.0) - v) +
         b1 * u * v;
}

template<typename T, std::floating_point T_uv>
[[nodiscard]] constexpr T trilerp(
  const std::array<T, 8>& x,
  T_uv u,
  T_uv v,
  T_uv w
) noexcept {
  float up = T_uv(1.0) - u, vp = T_uv(1.0) - v, wp = T_uv(1.0) - w;

  return x[0] * up * vp * wp +
         x[1] * up * vp * w +
         x[2] * up * v * wp +
         x[3] * up * v * w +
         x[4] * u * vp * wp +
         x[5] * u * vp * w +
         x[6] * u * v * wp +
         x[7] * u * v * w;
}

template<std::integral T, std::integral U>
[[nodiscard]] constexpr T ceilDiv(const T& m, const U& n) {
  return (m / n) + T(m % n != 0);
}

template<numeric T, numeric U>
[[nodiscard]] constexpr T min(const T& m, const U& n) noexcept {
  return m < n ? m : n;
}

template<numeric T, numeric U>
[[nodiscard]] constexpr T max(const T& m, const U& n) noexcept {
  return m > n ? m : n;
}

template<numeric T, numeric C>
[[nodiscard]] constexpr T evalPolynomial(T t, C c) noexcept { return c; }

template<numeric T, numeric C, numeric... Args>
[[nodiscard]] constexpr T evalPolynomial(T t, C c, Args... args) noexcept {
  return t * evalPolynomial(t, args...) + c;
}

/**
 * Given a size and a predicate f, returns the first value in the range
 * [0; size) for which f(x) is false
 */
[[nodiscard]] constexpr int64_t findFirst(
  int64_t size,
  const std::function<bool(int64_t)>& f
) noexcept {
  int64_t sz = int64_t(size - 1), first = 0;

  while (sz > 0) {
    int64_t half = sz >> 1, middle = first + half;
    bool res = f(middle);
    first = res ? (middle + 1) : first;
    sz = res ? sz - (half + 1) : half;
  }
  return std::clamp(first, int64_t(0), size - 1);
}

/**
 * Given a size and a predicate f, returns the first value in the range
 * [0; size - 1) for which f(x) is true and f(x + 1) is false
 */
[[nodiscard]] constexpr int64_t findInterval(
  int64_t size,
  const std::function<bool(int64_t)>& f
) noexcept {
  int64_t sz = int64_t(size - 2), first = 1;

  while (sz > 0) {
    int64_t half = sz >> 1, middle = first + half;
    bool res = f(middle);
    first = res ? middle + 1 : first;
    sz = res ? sz - (half + 1) : half;
  }
  return std::clamp(first - 1, int64_t(0), size - 2);
}

[[nodiscard]] constexpr uint32_t bits(float f) {
  return std::bit_cast<uint32_t>(f);
}

[[nodiscard]] constexpr int32_t exponent(float v) {
  return int32_t(bits(v) >> 23) - 127;
}

[[nodiscard]] constexpr uint32_t significand(float v) {
  return bits(v) & ((1 << 23) - 1);
}

[[nodiscard]] constexpr uint32_t signBit(float v) {
  return bits(v) & 0x80000000;
}

[[nodiscard]] constexpr int32_t log2Int(float v) {
  if (v < 1) return -log2Int(1 / v);
  const uint32_t midsignif = 0b00000000001101010000010011110011;
  return exponent(v) + ((significand(v) >= midsignif) ? 1 : 0);
}

[[nodiscard]] constexpr int32_t roundUpPow2(int32_t v) {
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return v + 1;
}


}

#endif //YART_MATH_BASE_HPP
