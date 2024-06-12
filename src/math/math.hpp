#ifndef YART_MATH_HPP
#define YART_MATH_HPP

#include "math_base.hpp"
#include "vec.hpp"
#include "mat.hpp"
#include "bounds.hpp"
#include "transform.hpp"
#include "ray.hpp"
#include "random.hpp"

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
