#ifndef YART_TRANSFORM_HPP
#define YART_TRANSFORM_HPP

#include "math_base.hpp"
#include "vec.hpp"
#include "mat.hpp"
#include "bounds.hpp"
#include "ray.hpp"

namespace yart::math {

class Transform {
public:
  enum class Type : int {
    Vector = 0,
    Point = 1,
    Normal = 2
  };

  static constexpr Transform translation(const float3& t) noexcept {
    return {float4x4::translation(t), float4x4::translation(-t)};
  }

  static constexpr Transform rotation(
    float angle,
    const float3& axis
  ) noexcept {
    return {float4x4::rotation(angle, axis), float4x4::rotation(-angle, axis)};
  }

  static constexpr Transform scaling(const float3& s) noexcept {
    return {float4x4::scaling(s), float4x4::scaling(1.0f / s)};
  }

  static constexpr Transform scaling(float s) noexcept {
    return {float4x4::scaling(s), float4x4::scaling(1.0f / s)};
  }

  constexpr Transform() noexcept
    : m_transform(float4x4::identity()),
      m_inverseTransform(float4x4::identity()),
      m_normalTransform(float3x3::identity()),
      m_inverseNormalTransform(float3x3::identity()) {}

  constexpr explicit Transform(const float4x4& m) noexcept
    : m_transform(m),
      m_inverseTransform(math::inverse(m).value()),
      m_normalTransform(transpose(float3x3(m_inverseTransform))),
      m_inverseNormalTransform(transpose(float3x3(m_transform))) {}

  constexpr Transform(const float4x4& m, const float4x4& inv) noexcept
    : m_transform(m),
      m_inverseTransform(inv),
      m_normalTransform(transpose(float3x3(inv))),
      m_inverseNormalTransform(transpose(float3x3(m))) {}

  [[nodiscard]] constexpr auto operator*(const Transform& rhs) const noexcept {
    return Transform(
      m_transform * rhs.m_transform,
      rhs.m_inverseTransform * m_inverseTransform
    );
  }

  [[nodiscard]] constexpr float4 operator()(const float4& rhs) const noexcept {
    return m_transform * rhs;
  }

  [[nodiscard]] constexpr float3 operator()(
    const float3& rhs,
    Type type = Type::Vector
  ) const noexcept {
    if (type == Type::Normal) return normalized(m_normalTransform * rhs);
    return float3(m_transform * float4(rhs, float(type)));
  }

  [[nodiscard]] constexpr fbounds3 operator()(const fbounds3& rhs) const noexcept {
    std::vector<float3> corners{
      {rhs.min.x(), rhs.min.y(), rhs.min.z()},
      {rhs.min.x(), rhs.min.y(), rhs.max.z()},
      {rhs.min.x(), rhs.max.y(), rhs.min.z()},
      {rhs.min.x(), rhs.max.y(), rhs.max.z()},
      {rhs.max.x(), rhs.min.y(), rhs.min.z()},
      {rhs.max.x(), rhs.min.y(), rhs.max.z()},
      {rhs.max.x(), rhs.max.y(), rhs.min.z()},
      {rhs.max.x(), rhs.max.y(), rhs.max.z()}
    };

    for (float3& corner: corners) corner = (*this)(corner, Type::Point);

    return fbounds3::fromPoints(corners);
  }

  [[nodiscard]] constexpr Ray operator()(const Ray& rhs) const noexcept {
    return {(*this)(rhs.origin, Type::Point), (*this)(rhs.dir)};
  }

  [[nodiscard]] constexpr float4 inverse(const float4& rhs) const noexcept {
    return m_inverseTransform * rhs;
  }

  [[nodiscard]] constexpr float3 inverse(
    const float3& rhs,
    Type type = Type::Vector
  ) const noexcept {
    if (type == Type::Normal) return normalized(m_inverseNormalTransform * rhs);
    return float3(m_inverseTransform * float4(rhs, float(type)));
  }

  [[nodiscard]] constexpr Ray inverse(const Ray& rhs) const noexcept {
    return {inverse(rhs.origin, Type::Point), inverse(rhs.dir)};
  }

private:
  float4x4 m_transform;
  float4x4 m_inverseTransform;
  float3x3 m_normalTransform;
  float3x3 m_inverseNormalTransform;
};

}

#endif //YART_TRANSFORM_HPP
