#ifndef YART_MATH_HPP
#define YART_MATH_HPP

#include "math_base.hpp"
#include "vec.hpp"
#include "mat.hpp"

namespace yart::math {

/*
 * Useful constants
 */
template<std::floating_point T>
static constexpr vec<T, 3> axis_x = vec<T, 3>(1.0, 0.0, 0.0);

template<std::floating_point T>
static constexpr vec<T, 3> axis_y = vec<T, 3>(0.0, 1.0, 0.0);

template<std::floating_point T>
static constexpr vec<T, 3> axis_z = vec<T, 3>(0.0, 0.0, 1.0);

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
//requires (N < 4 || M < 4)
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

//template<numeric T>
//[[nodiscard]] constexpr vec4<T> operator*(
//  const mat4x4<T>& lhs,
//  const vec4<T>& rhs
//) noexcept {
//  return {
//    lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2] + lhs[3] * rhs[3],
//    lhs[4] * rhs[0] + lhs[5] * rhs[1] + lhs[6] * rhs[2] + lhs[7] * rhs[3],
//    lhs[8] * rhs[0] + lhs[9] * rhs[1] + lhs[10] * rhs[2] + lhs[11] * rhs[3],
//    lhs[12] * rhs[0] + lhs[13] * rhs[1] + lhs[14] * rhs[2] + lhs[15] * rhs[3],
//  };
//}

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

/*
 * Utilities
 */
template<std::floating_point T>
[[nodiscard]] constexpr mat4x4<T> normalToTBN(const vec3<T>& n) noexcept {
  const vec3<T> a = std::abs(n.x()) > 0.5 ? axis_y<T> : axis_x<T>;

  const vec3<T> b = normalized(cross(n, a));
  const vec3<T> t = cross(n, b);

  return mat4x4<T>::fromColumns(
    vec4<T>(t, 0.0),
    vec4<T>(b, 0.0),
    vec4<T>(n, 0.0),
    vec4<T>(0.0, 0.0, 0.0, 1.0)
  );
}

}

#endif //YART_MATH_HPP
