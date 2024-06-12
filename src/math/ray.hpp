#ifndef YART_RAY_HPP
#define YART_RAY_HPP

#include "math_base.hpp"
#include "vec.hpp"

namespace yart::math {

class Ray {
public:
  float3 origin, dir;

  constexpr Ray(const float3& origin, const float3& dir) noexcept
    : origin(origin), dir(dir) {}

  [[nodiscard]] constexpr float3 operator()(float t) const noexcept {
    return origin + (t * dir);
  }
};

}

#endif //YART_RAY_HPP
