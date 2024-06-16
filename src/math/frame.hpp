#ifndef YART_FRAME_HPP
#define YART_FRAME_HPP

#include "vec.hpp"
#include "math.hpp"

namespace yart::math {

/*
 * Useful constants
 */
template<std::floating_point T>
constexpr const vec<T, 3> axis_x = vec<T, 3>(1.0, 0.0, 0.0);

template<std::floating_point T>
constexpr const vec<T, 3> axis_y = vec<T, 3>(0.0, 1.0, 0.0);

template<std::floating_point T>
constexpr const vec<T, 3> axis_z = vec<T, 3>(0.0, 0.0, 1.0);

class Frame {
public:
  float3 x, y, z;

  constexpr Frame() noexcept: x(1, 0, 0), y(0, 1, 0), z(0, 0, 1) {}

  constexpr explicit Frame(const float3& n) noexcept: z(n) {
    const float3 a = std::abs(n.x()) > 0.5 ? axis_y<float> : axis_x<float>;

    y = normalized(cross(n, a));
    x = cross(n, y);
  }

  [[nodiscard]] constexpr float3 wtl(const float3& w) const noexcept {
    return {dot(w, x), dot(w, y), dot(w, z)};
  }

  [[nodiscard]] constexpr float3 ltw(const float3& l) const noexcept {
    return l.x() * x + l.y() * y + l.z() * z;
  }
};

}

#endif //YART_FRAME_HPP
