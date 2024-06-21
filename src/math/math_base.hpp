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

}

#endif //YART_MATH_BASE_HPP
