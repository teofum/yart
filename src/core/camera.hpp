#ifndef YART_CAMERA_HPP
#define YART_CAMERA_HPP

#include <math/math.hpp>
#include "ray.hpp"

namespace yart {
using namespace math;

class Camera {
private:
  uint2 m_imageSize;    // Image size in pixels, defines aspect ratio
  float m_focalLength;  // Lens focal length in mm
  float m_fNumber;      // Lens aperture as f-number (fraction of focal length)
  float2 m_sensorSize;  // Sensor/film size in mm

  float3 m_position, m_forward, m_up;
  Frame m_cameraFrame;

  float m_aspect, m_focusDistance = 1.0, m_apertureRadius = 0.0f;
  float3 m_topLeftPixel;
  float3 m_pixelDeltaU, m_pixelDeltaV;

  // Initialize the camera from basic properties
  constexpr void calcDerivedProperties() noexcept {
    // Calculate final (cropped) aspect ratio from sensor and image aspect
    float sensorAspect = m_sensorSize.x() / m_sensorSize.y();
    float croppedSensorHeight = m_sensorSize.x() / max(sensorAspect, m_aspect);

    // Calculate viewport size
    m_focusDistance = length(m_forward);
    float vh = m_focusDistance * croppedSensorHeight / m_focalLength;
    float vw = vh * m_aspect;

    // Define camera coordinate frame
    m_up = normalized(m_up);
    float3 w = normalized(-m_forward);
    float3 u = cross(m_up, w);
    float3 v = cross(w, u);
    m_cameraFrame = Frame(w, u);

    // Make the image plane match the focus plane, makes the math a lot easier
    // Not how a real camera works, but we're not constrained by the laws of physics!
    float3 viewportU = u * vw;
    float3 viewportV = -v * vh;
    float3 viewportTopLeft =
      m_position - w * m_focusDistance - (viewportU + viewportV) * 0.5f;

    m_pixelDeltaU = viewportU / float(m_imageSize.x());
    m_pixelDeltaV = viewportV / float(m_imageSize.y());

    // Top-left pixel, shifted half a pixel from the top left corner of the viewport
    m_topLeftPixel = viewportTopLeft + (m_pixelDeltaU + m_pixelDeltaV) * 0.5f;

    // Calculate aperture radius from f-number
    // f = (focal length) / (aperture radius)
    m_apertureRadius = m_fNumber ? (m_focalLength / 2000.0f) / m_fNumber
                                 : 0.0f;
  }

public:
  float exposure = 0.0f;      // Exposure compensation in stops (EV)
  uint32_t apertureSides = 0; // Number of sides for aperture shape, 0 = circle

  /**
   * Create a new camera
   * @param imageSize Image size in pixels, defines aspect ratio as crop of sensor aspect
   * @param focalLength Lens focal length in mm
   * @param fNumber Lens aperture as f-number (fraction of focal length), 0 disables DOF
   * @param sensorSize Sensor/film size in mm, defaults to 36x24mm (35mm full frame)
   * @param position Camera position
   * @param forward Camera forward vector, direction camera is looking towards
   * @param up Up direction vector, defines camera roll
   */
  constexpr Camera(
    const uint2& imageSize,
    float focalLength,
    float fNumber = 0.0f,
    const float2& sensorSize = {36, 24},
    const float3& position = {},
    const float3& forward = -axis_z<float>,
    const float3& up = axis_y<float>
  ) noexcept
    : m_imageSize(imageSize),
      m_focalLength(focalLength),
      m_fNumber(fNumber),
      m_sensorSize(sensorSize),
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
    const float3& up = float3()
  ) noexcept {
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
    const float3& up = float3()
  ) noexcept {
    setDirection(target - m_position, up);
  }

  constexpr void moveAndLookAt(
    const float3& position,
    const float3& target,
    const float3& up = float3()
  ) noexcept {
    m_position = position;
    setDirection(target - position, up);
  }

  /**
   * Spawn a ray from the camera towards some pixel
   * @param pixelCoords Pixel coordinates in the sensor, origin at top left
   * @param uvFilm Sampled 2D point for film (pixel) position
   * @param uvLens Sampled 2D point for lens position
   * @return Primary ray
   */
  [[nodiscard]] constexpr Ray getRay(
    const uint2& pixelCoords,
    const float2& uvFilm,
    const float2& uvLens
  ) const noexcept {
    // Get world space pixel position
    float2 jitter =
      samplers::pixelJitterGaussian(uvFilm, 0.3f) + float2(pixelCoords);
    float3 pixel = m_topLeftPixel
                   + m_pixelDeltaU * jitter.x()
                   + m_pixelDeltaV * jitter.y();

    float3 origin = m_position;

    // Get ray origin on lens from aperture
    if (m_apertureRadius > 0.0f) {
      float2 apertureSample =
        apertureSides == 0 ? samplers::sampleDiskUniform(uvLens)
                           : samplers::samplePolyUniform(uvLens, apertureSides);

      float3 lensPos(apertureSample, 0.0f);
      lensPos *= m_apertureRadius;
      origin += m_cameraFrame.ltw(lensPos);
    }

    return {origin, normalized(pixel - origin)};
  }
};

}

#endif //YART_CAMERA_HPP
