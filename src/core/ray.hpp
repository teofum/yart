#ifndef YART_RAY_HPP
#define YART_RAY_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

class Ray {
public:
  float3 origin, dir, idir, odir;
  vec3<uint8_t> sign;
  bool nee = false;

  constexpr Ray(const float3& origin, const float3& dir) noexcept
    : origin(origin), dir(dir) {
    idir = 1.0 / dir;
    odir = -origin / dir;
    sign = {
      dir[0] < 0.0f ? 1 : 0,
      dir[1] < 0.0f ? 1 : 0,
      dir[2] < 0.0f ? 1 : 0
    };
  }

  [[nodiscard]] constexpr float3 operator()(float t) const noexcept {
    return origin + (t * dir);
  }
};

}

#endif //YART_RAY_HPP
