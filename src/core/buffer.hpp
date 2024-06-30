#ifndef YART_BUFFER_HPP
#define YART_BUFFER_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

class Buffer {
protected:
  uint32_t m_width, m_height;
  std::vector<float4> m_data;

public:
  constexpr Buffer(unsigned width, unsigned height) noexcept
    : m_width(width), m_height(height), m_data(width * height, float4()) {
  }

  [[nodiscard]] constexpr const uint32_t& width() const {
    return m_width;
  }

  [[nodiscard]] constexpr const uint32_t& height() const {
    return m_height;
  }

  [[nodiscard]] constexpr const float4& operator()(size_t x, size_t y) const {
    return m_data[y * m_width + x];
  }

  [[nodiscard]] constexpr float4& operator()(size_t x, size_t y) {
    return m_data[y * m_width + x];
  }

  [[nodiscard]] constexpr size_t size() const {
    return m_data.size();
  }

  [[nodiscard]] constexpr const std::vector<float4>& dataVec() const {
    return m_data;
  }

  [[nodiscard]] constexpr const float4* data() const {
    return m_data.data();
  }

  [[nodiscard]] constexpr const float4* data(uint2 offset) const {
    return m_data.data() + offset.y() * m_width + offset.x();
  }
};

}

#endif //YART_BUFFER_HPP
