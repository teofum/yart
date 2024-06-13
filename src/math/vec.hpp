#ifndef YART_VEC_HPP
#define YART_VEC_HPP

#include "math_base.hpp"

namespace yart::math {

/**
 * Vector for math operations
 * @tparam T Data type stored in the vector
 * @tparam N Dimension of the vector, 2-4
 */
template<numeric T, std::size_t N>
class vec {
  static_assert(N >= 2 && N <= 4, "Vector may only have 2, 3 or 4 components");

private:
  std::array<T, N> m_data;

public:
  /*
   * Constructors
   */

  // Vector components are initialized to 0
  constexpr vec() noexcept {
    m_data.fill(T(0));
  }

  // All components are initialized to a scalar
  constexpr explicit vec(auto scalar) noexcept {
    m_data.fill(T(scalar));
  }

  constexpr explicit vec(std::array<T, N> values) noexcept: m_data(values) {
  }

  template<numeric... V>
  requires (sizeof...(V) == N)
  constexpr vec(V... values) noexcept : m_data{T(values)...} {} // NOLINT(*)

  template<numeric T_lhs, std::size_t N_lhs, typename... V>
  requires (N_lhs < N && N_lhs + sizeof...(V) == N)
  constexpr explicit vec(const vec<T_lhs, N_lhs>& lhs, V... values) noexcept {
    std::array<T, sizeof...(V)> temp{T(values)...};
    for (size_t i = 0; i < N; i++)
      m_data[i] = i < N_lhs ? T(lhs[i]) : temp[i - N_lhs];
  }

  constexpr vec(const vec<T, N>& other) noexcept: m_data(other.m_data) {}

  constexpr auto& operator=(const vec<T, N>& other) noexcept {
    m_data = other.m_data;
    return *this;
  }

  template<numeric T_other>
  requires (!std::same_as<T_other, T>)
  constexpr explicit vec(const vec<T_other, N>& other) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] = T(other[i]);
  }

  template<numeric T_other>
  requires (!std::same_as<T_other, T>)
  constexpr auto& operator=(const vec<T_other, N>& other) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] = T(other[i]);
    return *this;
  }

  template<numeric T_other, std::size_t N_other>
  requires (N_other >= N)
  constexpr explicit vec(const vec<T_other, N_other>& other) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] = T(other[i]);
  }

  template<numeric T_other, std::size_t N_other>
  requires (N_other >= N)
  constexpr auto& operator=(const vec<T_other, N_other>& other) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] = T(other[i]);
    return *this;
  }

  /*
   * Access operators
   */

  [[nodiscard]] constexpr T& operator[](std::size_t idx) {
    return m_data[idx];
  }

  [[nodiscard]] constexpr const T& operator[](std::size_t idx) const {
    return m_data[idx];
  }

  [[nodiscard]] constexpr T* data() const noexcept {
    return m_data.data();
  }

  [[nodiscard]] constexpr T& x() noexcept {
    return m_data[0];
  }

  [[nodiscard]] constexpr T& y() noexcept {
    return m_data[1];
  }

  [[nodiscard]] constexpr T& z() noexcept {
    static_assert(N >= 3);
    return m_data[2];
  }

  [[nodiscard]] constexpr T& w() noexcept {
    static_assert(N == 4);
    return m_data[3];
  }

  [[nodiscard]] constexpr const T& x() const noexcept {
    return m_data[0];
  }

  [[nodiscard]] constexpr const T& y() const noexcept {
    return m_data[1];
  }

  [[nodiscard]] constexpr const T& z() const noexcept {
    static_assert(N >= 3);
    return m_data[2];
  }

  [[nodiscard]] constexpr const T& w() const noexcept {
    static_assert(N == 4);
    return m_data[3];
  }

  /*
   * Arithmetic operators
   */
  [[nodiscard]] constexpr bool operator==(const vec<T, N>& rhs) const noexcept {
    for (size_t i = 0; i < N; i++) {
      if (m_data[i] != rhs.m_data[i]) return false;
    }
    return true;
  }

  [[nodiscard]] constexpr bool operator!=(const vec<T, N>& rhs) const noexcept {
    return !(*this == rhs);
  }

  [[nodiscard]] constexpr auto operator+(const T& rhs) const noexcept {
    return vec<T, N>(*this) += rhs;
  }

  [[nodiscard]] constexpr auto operator+(const vec<T, N>& rhs) const noexcept {
    return vec<T, N>(*this) += rhs;
  }

  constexpr auto operator+=(const T& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] += rhs;
    return *this;
  }

  constexpr auto operator+=(const vec<T, N>& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] += rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator-(const T& rhs) const noexcept {
    return vec<T, N>(*this) -= rhs;
  }

  [[nodiscard]] constexpr auto operator-(const vec<T, N>& rhs) const noexcept {
    return vec<T, N>(*this) -= rhs;
  }

  constexpr auto operator-=(const T& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] -= rhs;
    return *this;
  }

  constexpr auto operator-=(const vec<T, N>& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] -= rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator*(const T& rhs) const noexcept {
    return vec<T, N>(*this) *= rhs;
  }

  [[nodiscard]] constexpr auto operator*(const vec<T, N>& rhs) const noexcept {
    return vec<T, N>(*this) *= rhs;
  }

  constexpr auto operator*=(const T& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] *= rhs;
    return *this;
  }

  constexpr auto operator*=(const vec<T, N>& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] *= rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator/(const T& rhs) const noexcept {
    return vec<T, N>(*this) /= rhs;
  }

  [[nodiscard]] constexpr auto operator/(const vec<T, N>& rhs) const noexcept {
    return vec<T, N>(*this) /= rhs;
  }

  constexpr auto operator/=(const T& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] /= rhs;
    return *this;
  }

  constexpr auto operator/=(const vec<T, N>& rhs) noexcept {
    for (size_t i = 0; i < N; i++) m_data[i] /= rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator-() const noexcept {
    vec<T, N> ret;
    for (size_t i = 0; i < N; i++) ret.m_data[i] = -m_data[i];
    return ret;
  }

  /*
   * Other
   */
  [[nodiscard]] std::string toString() const {
    std::stringstream ss;

    ss << "vec" << N << "(";
    for (std::size_t i = 0; i < N; i++)
      ss << m_data[i] << (i < N - 1 ? ", " : ")");

    return ss.str();
  }
};

/*
 * Debug print operator
 */
template<numeric T, std::size_t N>
std::ostream& operator<<(std::ostream& ostream, const vec <T, N> vec) {
  ostream << vec.toString();
  return ostream;
}

/*
 * Type aliases
 */

template<numeric T>
using vec2 = vec<T, 2>;

template<numeric T>
using vec3 = vec<T, 3>;

template<numeric T>
using vec4 = vec<T, 4>;

using float2 = vec2<float>;
using float3 = vec3<float>;
using float4 = vec4<float>;

using double2 = vec2<double>;
using double3 = vec3<double>;
using double4 = vec4<double>;

using int2 = vec2<int>;
using int3 = vec3<int>;
using int4 = vec4<int>;

using uint2 = vec2<unsigned>;
using uint3 = vec3<unsigned>;
using uint4 = vec4<unsigned>;

/*
 * Additional operators
 */
template<numeric T, std::size_t N>
[[nodiscard]] constexpr auto operator*(
  const auto& lhs,
  const vec <T, N>& rhs
) noexcept {
  return rhs * T(lhs);
}

template<numeric T>
[[nodiscard]] constexpr auto operator/(
  const auto& lhs,
  const vec2<T>& rhs
) noexcept {
  return vec2<T>(lhs / rhs[0], lhs / rhs[1]);
}

template<numeric T>
[[nodiscard]] constexpr auto operator/(
  const auto& lhs,
  const vec3<T>& rhs
) noexcept {
  return vec3<T>(lhs / rhs[0], lhs / rhs[1], lhs / rhs[2]);
}

template<numeric T>
[[nodiscard]] constexpr auto operator/(
  const auto& lhs,
  const vec4<T>& rhs
) noexcept {
  return vec4<T>(lhs / rhs[0], lhs / rhs[1], lhs / rhs[2], lhs / rhs[3]);
}

/*
 * Useful math functions
 */

template<numeric T, std::size_t N>
[[nodiscard]] constexpr T sum(const vec <T, N>& vec) noexcept {
  T sum = T(0);
  for (std::size_t i = 0; i < N; i++) sum += vec[i];
  return sum;
}

template<numeric T, std::size_t N>
[[nodiscard]] constexpr T length2(const vec <T, N>& vec) noexcept {
  T sum = T(0);
  for (std::size_t i = 0; i < N; i++) sum += vec[i] * vec[i];
  return sum;
}

template<numeric T, std::size_t N>
[[nodiscard]] constexpr T length(const vec <T, N>& vec) noexcept {
  return std::sqrt(length2(vec));
}

template<numeric T, std::size_t N>
[[nodiscard]] constexpr vec<T, N> normalized(const vec <T, N>& vec) noexcept {
  return vec / length(vec);
}

template<numeric T, std::size_t N>
[[nodiscard]] constexpr bool nearZero(const vec <T, N>& vec) noexcept {
  for (std::size_t i = 0; i < N; i++) {
    if (vec[i] > epsilon) return false;
  }
  return true;
}

template<numeric T>
[[nodiscard]] constexpr T dot(const vec3<T>& lhs, const vec3<T>& rhs) noexcept {
  return lhs.x() * rhs.x() + lhs.y() * rhs.y() + lhs.z() * rhs.z();
}

template<numeric T>
[[nodiscard]] constexpr vec3<T> cross(
  const vec3<T>& lhs,
  const vec3<T>& rhs
) noexcept {
  return vec3<T>(
    lhs.y() * rhs.z() - lhs.z() * rhs.y(),
    lhs.z() * rhs.x() - lhs.x() * rhs.z(),
    lhs.x() * rhs.y() - lhs.y() * rhs.x()
  );
}

template<std::floating_point T>
[[nodiscard]] constexpr vec3<T> reflect(
  const vec3<T>& vec,
  const vec3<T>& normal
) noexcept {
  return vec - normal * 2.0 * dot(vec, normal);
}

template<std::floating_point T>
[[nodiscard]] constexpr vec3<T> refract(
  const vec3<T>& vec,
  const vec3<T>& normal,
  T iorRatio
) noexcept {
  T cosTheta = std::min(T(1.0), dot(-vec, normal));
  vec3<T> refractedPerp = (vec + normal * cosTheta) * iorRatio;
  vec3<T> refractedParallel =
    normal * -std::sqrt(1.0 - refractedPerp.lengthSquared());

  return refractedPerp + refractedParallel;
}

template<numeric T, std::size_t N>
[[nodiscard]] constexpr vec<T, N> min(
  const vec<T, N>& lhs,
  const vec<T, N>& rhs
) noexcept {
  vec<T, N> ret;
  for (size_t i = 0; i < N; i++) ret[i] = std::min(lhs[i], rhs[i]);
  return ret;
}

template<numeric T, std::size_t N>
[[nodiscard]] constexpr vec<T, N> max(
  const vec<T, N>& lhs,
  const vec<T, N>& rhs
) noexcept {
  vec<T, N> ret;
  for (size_t i = 0; i < N; i++) ret[i] = std::max(lhs[i], rhs[i]);
  return ret;
}

}

#endif //YART_VEC_HPP
