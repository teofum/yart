#ifndef YART_RAY_HPP
#define YART_RAY_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

class Ray {
private:
  float3 m_dir, m_invDir;
  vec3<uint8> m_sign;

public:
  float3 origin;

  constexpr Ray(const float3& origin, const float3& dir) noexcept
    : m_dir(dir), origin(origin) {
    m_invDir = 1.0f / dir;
    m_sign = vec3<uint8>(
      m_invDir.x() < 0.0f ? 1 : 0,
      m_invDir.y() < 0.0f ? 1 : 0,
      m_invDir.z() < 0.0f ? 1 : 0
    );
  }

  [[nodiscard]] constexpr const float3& dir() const noexcept {
    return m_dir;
  }

  [[nodiscard]] constexpr const float3& invDir() const noexcept {
    return m_invDir;
  }

  [[nodiscard]] constexpr const vec3<uint8>& sign() const noexcept {
    return m_sign;
  }

  constexpr void setDir(const float3& dir) noexcept {
    m_dir = dir;
    m_invDir = 1.0f / dir;
    m_sign = vec3<uint8>(
      dir.x() < 0.0f ? 1 : 0,
      dir.y() < 0.0f ? 1 : 0,
      dir.z() < 0.0f ? 1 : 0
    );
  }

  [[nodiscard]] constexpr float3 at(float t) const noexcept {
    return origin + (t * m_dir);
  }
};

}

#endif //YART_RAY_HPP
