#ifndef YART_MATH_BASE_HPP
#define YART_MATH_BASE_HPP

#include <array>
#include <string>
#include <sstream>

namespace yart::math {

constexpr double epsilon = 1e-12;
constexpr double pi = M_PI;

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

}

#endif //YART_MATH_BASE_HPP
