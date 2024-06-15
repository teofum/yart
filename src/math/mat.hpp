#ifndef YART_MAT_HPP
#define YART_MAT_HPP

#include <functional>

#include "math_base.hpp"
#include "vec.hpp"

namespace yart::math {

template<numeric T, std::size_t N, std::size_t M>
class mat {
  static_assert(N >= 2 && N <= 4, "Matrix may only have 2, 3 or 4 rows");
  static_assert(M >= 2 && M <= 4, "Matrix may only have 2, 3 or 4 columns");

private:
  std::array<T, N * M> m_data;

public:
  /*
   * Constructors
   */

  [[nodiscard]] constexpr static mat<T, N, M> identity() requires (N == M) {
    return mat<T, N, M>(1);
  }

  template<numeric... Ts>
  [[nodiscard]] constexpr static mat<T, N, M> fromColumns(
    const vec<Ts, N>... cols
  ) requires (sizeof...(Ts) == M) {
    mat<T, N, M> mat;
    for (size_t i = 0; i < N; i++) {
      size_t j = 0;
      for (auto& col: {cols...}) {
        mat(i, j++) = col[i];
      }
    }
    return mat;
  }

  [[nodiscard]] constexpr static mat<T, 4, 4> translation(const vec3<T>& t) //
  requires (N == 4 && M == 4) {
    return {
      1.0, 0.0, 0.0, t.x(),
      0.0, 1.0, 0.0, t.y(),
      0.0, 0.0, 1.0, t.z(),
      0.0, 0.0, 0.0, 1.0,
    };
  }

  [[nodiscard]] constexpr static mat<T, 4, 4> rotation(
    const T& angle,
    const vec3<T>& axis
  ) requires (std::floating_point<T> && N == 4 && M == 4) {
    const T a = angle;
    const T c = std::cos(a);
    const T s = std::sin(a);

    const vec3<T> nAxis = normalized(axis);
    const vec3<T> temp = (1.0 - c) * nAxis;
    return {
      c + temp.x() * nAxis.x(),
      temp.y() * nAxis.x() - s * nAxis.z(),
      temp.z() * nAxis.x() + s * nAxis.y(), 0.0f,
      temp.x() * nAxis.y() + s * nAxis.z(),
      c + temp.y() * nAxis.y(),
      temp.z() * nAxis.y() - s * nAxis.x(), 0.0f,
      temp.x() * nAxis.z() - s * nAxis.y(),
      temp.y() * nAxis.z() + s * nAxis.x(),
      c + temp.z() * nAxis.z(), 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f
    };
  }

  [[nodiscard]] constexpr static mat<T, 4, 4> scaling(const vec3<T>& s) //
  requires (N == 4 && M == 4) {
    return {
      s.x(), 0.0, 0.0, 0.0,
      0.0, s.y(), 0.0, 0.0,
      0.0, 0.0, s.z(), 0.0,
      0.0, 0.0, 0.0, 1.0,
    };
  }

  [[nodiscard]] constexpr static mat<T, 4, 4> scaling(const T& u) //
  requires (N == 4 && M == 4) {
    return {
      u, 0.0, 0.0, 0.0,
      0.0, u, 0.0, 0.0,
      0.0, 0.0, u, 0.0,
      0.0, 0.0, 0.0, 1.0,
    };
  }

  [[nodiscard]] constexpr static mat<T, 4, 4> perspective(
    T fov, T aspect, T near, T far
  ) requires (N == 4 && M == 4) {
    const T sy = 1.0 / std::tan(fov * T(0.5));
    const T sx = sy / aspect;
    const T zRange = near - far;
    const T sz = (far + near) / zRange;
    const T tz = 2 * far * near / zRange;

    return {
      sx, 0.0, 0.0, 0.0,
      0.0, sy, 0.0, 0.0,
      0.0, 0.0, sz, tz,
      0.0, 0.0, -1.0, 0.0
    };
  }

  // Matrix components are initialized to 0
  constexpr mat() noexcept {
    m_data.fill(T(0));
  }

  // Diagonal components are initialized to a scalar
  constexpr explicit mat(auto scalar) noexcept: mat() {
    for (std::size_t i = 0; i < std::min(N, M); i++)
      m_data[i * N + i] = T(scalar);
  }

  constexpr explicit mat(std::array<T, N * M> values) noexcept
    : m_data(values) {}

  template<numeric... V>
  requires (sizeof...(V) == std::min(N, M))
  constexpr explicit mat(V... values) noexcept {
    m_data.fill(T(0));
    size_t i = 0;
    ((m_data[M * i + i] = values, i++), ...);
  }

  template<numeric... V>
  requires (sizeof...(V) == N * M)
  constexpr mat(V... values) noexcept : m_data{T(values)...} {} // NOLINT(*)

  constexpr mat(const mat<T, N, M>& other) noexcept: m_data(other.m_data) {}

  constexpr auto& operator=(const mat<T, N, M>& other) noexcept {
    m_data = other.m_data;
    return *this;
  }

  template<numeric T_other>
  requires (!std::same_as<T_other, T>)
  constexpr explicit mat(const mat<T_other, N, M>& other) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] = T(other[i]);
  }

  template<numeric T_other>
  requires (!std::same_as<T_other, T>)
  constexpr auto& operator=(const mat<T_other, N, M>& other) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] = T(other[i]);
    return *this;
  }

  template<numeric T_other, std::size_t N_other, std::size_t M_other>
  requires (N_other > N && M_other > M)
  constexpr explicit mat(const mat<T_other, N_other, M_other>& other) noexcept {
    for (size_t i = 0; i < N; i++)
      for (size_t j = 0; j < M; j++)
        m_data[i * N + j] = T(other[i * N_other + j]);
  }

  template<numeric T_other, std::size_t N_other, std::size_t M_other>
  requires (N_other > N && M_other > M)
  constexpr auto& operator=(const mat<T_other, N_other, M_other>& other) noexcept {
    for (size_t i = 0; i < N; i++)
      for (size_t j = 0; j < M; j++)
        m_data[i * N + j] = T(other[i * N_other + j]);
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

  [[nodiscard]] constexpr T& operator()(std::size_t i, std::size_t j) noexcept {
    return m_data[i * N + j];
  }

  [[nodiscard]] constexpr const T& operator()(
    std::size_t i,
    std::size_t j
  ) const noexcept {
    return m_data[i * N + j];
  }

  /*
   * Arithmetic operators
   */
  [[nodiscard]] constexpr bool operator==(const mat<T, N, M>& rhs) const noexcept {
    for (size_t i = 0; i < N * M; i++) {
      if (m_data[i] != rhs.m_data[i]) return false;
    }
    return true;
  }

  [[nodiscard]] constexpr bool operator!=(const mat<T, N, M>& rhs) const noexcept {
    return !(*this == rhs);
  }

  [[nodiscard]] constexpr auto operator+(const T& rhs) const noexcept {
    return mat<T, N, M>(*this) += rhs;
  }

  [[nodiscard]] constexpr auto operator+(const mat<T, N, M>& rhs) const noexcept {
    return mat<T, N, M>(*this) += rhs;
  }

  constexpr auto operator+=(const T& rhs) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] += rhs;
    return *this;
  }

  constexpr auto operator+=(const mat<T, N, M>& rhs) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] += rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator-(const T& rhs) const noexcept {
    return mat<T, N, M>(*this) -= rhs;
  }

  [[nodiscard]] constexpr auto operator-(const mat<T, N, M>& rhs) const noexcept {
    return mat<T, N, M>(*this) -= rhs;
  }

  constexpr auto operator-=(const T& rhs) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] -= rhs;
    return *this;
  }

  constexpr auto operator-=(const mat<T, N, M>& rhs) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] -= rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator*(const T& rhs) const noexcept {
    return mat<T, N, M>(*this) *= rhs;
  }

  constexpr auto operator*=(const T& rhs) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] *= rhs;
    return *this;
  }

  template<std::size_t R>
  [[nodiscard]] constexpr auto operator*(const mat<T, M, R>& rhs) const noexcept {
    mat<T, N, R> res;
    for (std::size_t i = 0; i < N; i++) {
      for (std::size_t j = 0; j < R; j++) {
        for (std::size_t k = 0; k < M; k++) {
          res(i, j) += (*this)(i, k) * rhs(k, j);
        }
      }
    }
    return res;
  }

  [[nodiscard]] constexpr auto operator/(const T& rhs) const noexcept {
    return mat<T, N, M>(*this) /= rhs;
  }

  constexpr auto operator/=(const T& rhs) noexcept {
    for (size_t i = 0; i < N * M; i++) m_data[i] /= rhs;
    return *this;
  }

  [[nodiscard]] constexpr auto operator-() const noexcept {
    mat<T, N, M> ret;
    for (size_t i = 0; i < N * M; i++) ret.m_data[i] = -m_data[i];
    return ret;
  }
};

/*
 * Type aliases
 */

template<numeric T>
using mat2x2 = mat<T, 2, 2>;
template<numeric T>
using mat2x3 = mat<T, 2, 3>;
template<numeric T>
using mat2x4 = mat<T, 2, 4>;

template<numeric T>
using mat3x2 = mat<T, 3, 2>;
template<numeric T>
using mat3x3 = mat<T, 3, 3>;
template<numeric T>
using mat3x4 = mat<T, 3, 4>;

template<numeric T>
using mat4x2 = mat<T, 4, 2>;
template<numeric T>
using mat4x3 = mat<T, 4, 3>;
template<numeric T>
using mat4x4 = mat<T, 4, 4>;

using float2x2 = mat2x2<float>;
using float2x3 = mat2x3<float>;
using float2x4 = mat2x4<float>;
using float3x2 = mat3x2<float>;
using float3x3 = mat3x3<float>;
using float3x4 = mat3x4<float>;
using float4x2 = mat4x2<float>;
using float4x3 = mat4x3<float>;
using float4x4 = mat4x4<float>;

using double2x2 = mat2x2<double>;
using double2x3 = mat2x3<double>;
using double2x4 = mat2x4<double>;
using double3x2 = mat3x2<double>;
using double3x3 = mat3x3<double>;
using double3x4 = mat3x4<double>;
using double4x2 = mat4x2<double>;
using double4x3 = mat4x3<double>;
using double4x4 = mat4x4<double>;

/*
 * Useful math functions
 */

template<numeric T, std::size_t N, std::size_t M>
[[nodiscard]] constexpr mat<T, M, N> transpose(const mat<T, N, M>& m) noexcept {
  mat<T, M, N> transposed;
  for (std::size_t i = 0; i < N; i++) {
    for (std::size_t j = 0; j < M; j++) {
      transposed(j, i) = m(i, j);
    }
  }
  return transposed;
}

template<numeric T>
[[nodiscard]] constexpr T determinant(const mat<T, 2, 2>& m) noexcept {
  return m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
}

template<std::floating_point T>
[[nodiscard]] constexpr std::optional<mat<T, 2, 2>> inverse(const mat<T, 2, 2>& m) noexcept {
  T det = determinant(m);
  if (det == T(0)) return std::nullopt;

  det = T(1.0) / det;
  mat<T, 2, 2> adj(m(1, 1), -m(0, 1), -m(1, 0), m(0, 0));
  return adj * det;
}

template<numeric T>
[[nodiscard]] constexpr T determinant(const mat<T, 3, 3>& m) noexcept {
  return m(0, 0) * m(1, 1) * m(2, 2)
         + m(0, 1) * m(1, 2) * m(2, 0)
         + m(0, 2) * m(1, 0) * m(2, 1)
         - m(0, 2) * m(1, 1) * m(2, 0)
         - m(0, 1) * m(1, 0) * m(2, 2)
         - m(0, 0) * m(1, 2) * m(2, 1);
}

template<std::floating_point T>
[[nodiscard]] constexpr std::optional<mat<T, 3, 3>> inverse(const mat<T, 3, 3>& m) noexcept {
  T det = determinant(m);
  if (det == T(0)) return std::nullopt;

  det = T(1.0) / det;
  return std::make_optional(
    mat<T, 3, 3>(
      (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) * det,
      (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * det,
      (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * det,
      (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2)) * det,
      (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * det,
      (m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2)) * det,
      (m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1)) * det,
      (m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1)) * det,
      (m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1)) * det
    ));
}

// Internal 4x4 matrix determinant/inverse implementation
namespace internal_pls_no_touchy {

template<numeric T>
[[nodiscard]] constexpr T det_impl(
  const mat<T, 4, 4>& m,
  std::array<T, 16>& inv
) noexcept {
  inv[0] = m[5] * m[10] * m[15] -
           m[5] * m[11] * m[14] -
           m[9] * m[6] * m[15] +
           m[9] * m[7] * m[14] +
           m[13] * m[6] * m[11] -
           m[13] * m[7] * m[10];

  inv[4] = -m[4] * m[10] * m[15] +
           m[4] * m[11] * m[14] +
           m[8] * m[6] * m[15] -
           m[8] * m[7] * m[14] -
           m[12] * m[6] * m[11] +
           m[12] * m[7] * m[10];

  inv[8] = m[4] * m[9] * m[15] -
           m[4] * m[11] * m[13] -
           m[8] * m[5] * m[15] +
           m[8] * m[7] * m[13] +
           m[12] * m[5] * m[11] -
           m[12] * m[7] * m[9];

  inv[12] = -m[4] * m[9] * m[14] +
            m[4] * m[10] * m[13] +
            m[8] * m[5] * m[14] -
            m[8] * m[6] * m[13] -
            m[12] * m[5] * m[10] +
            m[12] * m[6] * m[9];

  inv[1] = -m[1] * m[10] * m[15] +
           m[1] * m[11] * m[14] +
           m[9] * m[2] * m[15] -
           m[9] * m[3] * m[14] -
           m[13] * m[2] * m[11] +
           m[13] * m[3] * m[10];

  inv[5] = m[0] * m[10] * m[15] -
           m[0] * m[11] * m[14] -
           m[8] * m[2] * m[15] +
           m[8] * m[3] * m[14] +
           m[12] * m[2] * m[11] -
           m[12] * m[3] * m[10];

  inv[9] = -m[0] * m[9] * m[15] +
           m[0] * m[11] * m[13] +
           m[8] * m[1] * m[15] -
           m[8] * m[3] * m[13] -
           m[12] * m[1] * m[11] +
           m[12] * m[3] * m[9];

  inv[13] = m[0] * m[9] * m[14] -
            m[0] * m[10] * m[13] -
            m[8] * m[1] * m[14] +
            m[8] * m[2] * m[13] +
            m[12] * m[1] * m[10] -
            m[12] * m[2] * m[9];

  inv[2] = m[1] * m[6] * m[15] -
           m[1] * m[7] * m[14] -
           m[5] * m[2] * m[15] +
           m[5] * m[3] * m[14] +
           m[13] * m[2] * m[7] -
           m[13] * m[3] * m[6];

  inv[6] = -m[0] * m[6] * m[15] +
           m[0] * m[7] * m[14] +
           m[4] * m[2] * m[15] -
           m[4] * m[3] * m[14] -
           m[12] * m[2] * m[7] +
           m[12] * m[3] * m[6];

  inv[10] = m[0] * m[5] * m[15] -
            m[0] * m[7] * m[13] -
            m[4] * m[1] * m[15] +
            m[4] * m[3] * m[13] +
            m[12] * m[1] * m[7] -
            m[12] * m[3] * m[5];

  inv[14] = -m[0] * m[5] * m[14] +
            m[0] * m[6] * m[13] +
            m[4] * m[1] * m[14] -
            m[4] * m[2] * m[13] -
            m[12] * m[1] * m[6] +
            m[12] * m[2] * m[5];

  inv[3] = -m[1] * m[6] * m[11] +
           m[1] * m[7] * m[10] +
           m[5] * m[2] * m[11] -
           m[5] * m[3] * m[10] -
           m[9] * m[2] * m[7] +
           m[9] * m[3] * m[6];

  inv[7] = m[0] * m[6] * m[11] -
           m[0] * m[7] * m[10] -
           m[4] * m[2] * m[11] +
           m[4] * m[3] * m[10] +
           m[8] * m[2] * m[7] -
           m[8] * m[3] * m[6];

  inv[11] = -m[0] * m[5] * m[11] +
            m[0] * m[7] * m[9] +
            m[4] * m[1] * m[11] -
            m[4] * m[3] * m[9] -
            m[8] * m[1] * m[7] +
            m[8] * m[3] * m[5];

  inv[15] = m[0] * m[5] * m[10] -
            m[0] * m[6] * m[9] -
            m[4] * m[1] * m[10] +
            m[4] * m[2] * m[9] +
            m[8] * m[1] * m[6] -
            m[8] * m[2] * m[5];

  return m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
}

}

template<numeric T>
[[nodiscard]] constexpr T determinant(const mat<T, 4, 4>& m) noexcept {
  std::array<T, 16> inv;
  return internal_pls_no_touchy::det_impl(m, inv);
}

template<std::floating_point T>
[[nodiscard]] constexpr std::optional<mat<T, 4, 4>> inverse(const mat<T, 4, 4>& m) noexcept {
  std::array<T, 16> inv;
  T det = internal_pls_no_touchy::det_impl(m, inv);
  if (det == T(0)) return std::nullopt;

  det = T(1.0) / det;
  for (auto& i: inv) i *= det;
  return std::make_optional(mat<T, 4, 4>(inv));
}

/*
 * Matrix-vector math
 */
template<numeric T, std::size_t N, std::size_t M>
[[nodiscard]] constexpr vec<T, N> col(
  const mat<T, N, M>& m,
  std::size_t j
) noexcept {
  vec<T, N> col;
  for (std::size_t i = 0; i < N; i++) col[i] = m(i, j);
  return col;
}

template<numeric T, std::size_t N, std::size_t M>
[[nodiscard]] constexpr vec<T, M> row(
  const mat<T, N, M>& m,
  std::size_t i
) noexcept {
  vec<T, M> row;
  for (std::size_t j = 0; j < M; j++) row[j] = m(i, j);
  return row;
}

template<numeric T, std::size_t N, std::size_t M>
[[nodiscard]] constexpr vec<T, N> operator*(
  const mat<T, N, M>& lhs,
  const vec<T, M>& rhs
) noexcept {
  vec<T, N> res;
  for (std::size_t i = 0; i < N; i++) {
    for (std::size_t j = 0; j < M; j++) {
      res[i] += lhs(i, j) * rhs[j];
    }
  }
  return res;
}

template<numeric T, std::size_t N, std::size_t M>
[[nodiscard]] constexpr vec<T, M> operator*(
  const vec<T, N>& lhs,
  const mat<T, N, M>& rhs
) noexcept {
  vec<T, N> res;
  for (std::size_t j = 0; j < M; j++) {
    for (std::size_t i = 0; i < N; i++) {
      res[j] += lhs[i] * rhs(i, j);
    }
  }
  return res;
}

}

#endif //YART_MAT_HPP
