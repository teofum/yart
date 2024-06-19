#ifndef YART_COMPLEX_HPP
#define YART_COMPLEX_HPP

#include "math_base.hpp"

namespace yart::math {

template<numeric T>
struct complex {
  T re, im;

  constexpr explicit complex(T re) noexcept: re(re), im(0) {}

  constexpr complex(T re, T im) noexcept: re(re), im(im) {}

  [[nodiscard]] constexpr bool operator==(const complex& rhs) const noexcept {
    return re == rhs.re && im == rhs.im;
  }

  [[nodiscard]] constexpr bool operator!=(const complex& rhs) const noexcept {
    return re != rhs.re || im != rhs.im;
  }

  [[nodiscard]] constexpr complex operator-() const noexcept {
    return {-re, -im};
  }

  [[nodiscard]] constexpr complex operator+(const complex& rhs) const noexcept {
    return {re + rhs.re, im + rhs.im};
  }

  [[nodiscard]] constexpr complex operator-(const complex& rhs) const noexcept {
    return {re - rhs.re, im - rhs.im};
  }

  [[nodiscard]] constexpr complex operator*(const complex& rhs) const noexcept {
    return {re * rhs.re - im * rhs.im, re * rhs.im + im * rhs.re};
  }

  [[nodiscard]] constexpr complex operator/(const complex& rhs) const noexcept {
    T s = T(1.0) / (rhs.re * rhs.re + rhs.im * rhs.im);
    return {s * (re * rhs.re + im * rhs.im), s * (im * rhs.re - re * rhs.im)};
  }

  [[nodiscard]] constexpr complex operator+(T rhs) const noexcept {
    return {re + rhs, im};
  }

  [[nodiscard]] constexpr complex operator-(T rhs) const noexcept {
    return {re - rhs, im};
  }

  [[nodiscard]] constexpr complex operator*(T rhs) const noexcept {
    return {re * rhs, im * rhs};
  }

  [[nodiscard]] constexpr complex operator/(T rhs) const noexcept {
    return {re / rhs, im / rhs};
  }

  [[nodiscard]] friend constexpr complex operator+(
    T lhs,
    const complex& rhs
  ) noexcept {
    return {lhs + rhs.re, rhs.im};
  }

  [[nodiscard]] friend constexpr complex operator-(
    T lhs,
    const complex& rhs
  ) noexcept {
    return {lhs - rhs.re, rhs.im};
  }

  [[nodiscard]] friend constexpr complex operator*(
    T lhs,
    const complex& rhs
  ) noexcept {
    return {lhs * rhs.re, lhs * rhs.im};
  }

  [[nodiscard]] friend constexpr complex operator/(
    T lhs,
    const complex& rhs
  ) noexcept {
    T s = lhs / (rhs.re * rhs.re + rhs.im * rhs.im);
    return {s * rhs.re, s * -rhs.im};
  }
};

template<numeric T>
[[nodiscard]] constexpr T real(const complex<T>& z) noexcept { return z.re; }

template<numeric T>
[[nodiscard]] constexpr T imag(const complex<T>& z) noexcept { return z.im; }

template<numeric T>
[[nodiscard]] constexpr T norm(const complex<T>& z) noexcept {
  return z.re * z.re + z.im * z.im;
}

template<numeric T>
[[nodiscard]] constexpr T abs(const complex<T>& z) noexcept {
  return std::sqrt(norm(z));
}

template<numeric T>
[[nodiscard]] constexpr complex<T> sqrt(const complex<T>& z) noexcept {
  T n = abs(z), t1 = std::sqrt(T(0.5) * (n + std::abs(z.re)));
  T t2 = T(0.5) * z.im / t1;

  if (n == 0) return {0, 0};
  if (z.re >= 0) return {t1, t2};
  return {std::abs(t2), std::copysign(t1, z.im)};
}

}

#endif //YART_COMPLEX_HPP
