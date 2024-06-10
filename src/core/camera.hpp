#ifndef YART_CAMERA_HPP
#define YART_CAMERA_HPP

#include <math/math.hpp>
#include <core/ray.hpp>

namespace yart {
using namespace math;

class Camera {
private:
  uint2 m_imageSize;
  float m_vfov;
  float3 m_position, m_forward, m_up;

  float m_aspect, m_focusDistance = 1.0;
  float3 m_topLeftPixel;
  float3 m_pixelDeltaU, m_pixelDeltaV;

  constexpr void calcDerivedProperties() noexcept {
    m_focusDistance = length(m_forward);
    float vh = 2.0f * m_focusDistance * std::tan(m_vfov * 0.5f);
    float vw = vh * m_aspect;

    m_up = normalized(m_up);
    float3 w = normalized(-m_forward);
    float3 u = cross(m_up, w);
    float3 v = cross(w, u);

    float3 viewportU = u * vw;
    float3 viewportV = -v * vh;

    m_pixelDeltaU = viewportU / float(m_imageSize.x());
    m_pixelDeltaV = viewportV / float(m_imageSize.y());

    float3 viewportTopLeft =
      m_position - w * m_focusDistance - (viewportU + viewportV) * 0.5f;
    m_topLeftPixel = viewportTopLeft + (m_pixelDeltaU + m_pixelDeltaV) * 0.5f;
  }

public:
  constexpr Camera(
    const uint2& imageSize,
    float vfov,
    const float3& position,
    const float3& forward = -axis_z<float>,
    const float3& up = axis_y<float>
  ) noexcept
    : m_imageSize(imageSize),
      m_vfov(vfov),
      m_position(position),
      m_forward(forward),
      m_up(up),
      m_aspect(float(m_imageSize.x()) / float(m_imageSize.y())) {
    calcDerivedProperties();
  }

  constexpr void setPosition(const float3& pos) noexcept {
    m_position = pos;
    calcDerivedProperties();
  }

  constexpr void setDirection(
    const float3& forward,
    const float3& up = float3()) noexcept {
    m_forward = forward;
    if (length2(up) != 0.0f) m_up = up;

    calcDerivedProperties();
  }

  constexpr void setFocusDistance(float d) noexcept {
    m_focusDistance = d;
    calcDerivedProperties();
  }

  constexpr void lookAt(
    const float3& target,
    const float3& up = float3()) noexcept {
    setDirection(target - m_position, up);
  }

  constexpr void moveAndLookAt(
    const float3& position,
    const float3& target,
    const float3& up = float3()) noexcept {
    m_position = position;
    setDirection(target - position, up);
  }

  [[nodiscard]] constexpr Ray getRay(
    const uint2& pixelCoords,
    Xoshiro::Xoshiro256PP& rng
  ) const noexcept {
    float2 jitter = random::pixelJitterGaussian(rng) + float2(pixelCoords);
    float3 pixel = m_topLeftPixel
                   + m_pixelDeltaU * jitter.x()
                   + m_pixelDeltaV * jitter.y();

    return {m_position, pixel - m_position};
  }
};

}

#endif //YART_CAMERA_HPP
