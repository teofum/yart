#ifndef YART_TEXTURE_HPP
#define YART_TEXTURE_HPP

#include <stb_image.h>

#include <math/math.hpp>

namespace yart {
using namespace math;

template<typename T>
class Texture {
public:
  static Texture<float> loadMono(const uint8_t* data, int32_t len) {
    int32_t w, h;
    const uint8_t* pixels =
      stbi_load_from_memory(data, len, &w, &h, nullptr, 1);

    Texture<float> texture(w, h);
    for (uint32_t i = 0; i < w * h; i++) {
      texture.m_data[i] = float(pixels[i]) / 255.0f;
    }

    return texture;
  }

  static Texture<float3> loadRGB(const uint8_t* data, int32_t len) {
    int32_t w, h;
    const uint8_t* pixels =
      stbi_load_from_memory(data, len, &w, &h, nullptr, 3);

    Texture<float3> texture(w, h);
    for (uint32_t i = 0; i < w * h; i++) {
      uint32_t iPixel = i * 3;
      texture.m_data[i] = float3(
        pixels[iPixel + 0],
        pixels[iPixel + 1],
        pixels[iPixel + 2]
      ) / 255.0f;
    }

    return texture;
  }

  static Texture<float4> loadRGBA(const uint8_t* data, int32_t len) {
    int32_t w, h;
    const uint8_t* pixels =
      stbi_load_from_memory(data, len, &w, &h, nullptr, 4);

    Texture<float4> texture(w, h);
    for (uint32_t i = 0; i < w * h; i++) {
      uint32_t iPixel = i * 4;
      texture.m_data[i] = float4(
        pixels[iPixel + 0],
        pixels[iPixel + 1],
        pixels[iPixel + 2],
        pixels[iPixel + 3]
      ) / 255.0f;
    }

    return texture;
  }

  constexpr Texture(unsigned width, unsigned height) noexcept
    : m_width(width), m_height(height), m_data(width * height, T(0)) {
  }

  [[nodiscard]] constexpr const size_t& width() const {
    return m_width;
  }

  [[nodiscard]] constexpr const size_t& height() const {
    return m_height;
  }

  [[nodiscard]] constexpr const T& operator()(size_t x, size_t y) const {
    return m_data[y * m_width + x];
  }

  [[nodiscard]] constexpr T& operator()(size_t x, size_t y) {
    return m_data[y * m_width + x];
  }

  [[nodiscard]] constexpr size_t size() const {
    return m_data.size();
  }

  [[nodiscard]] constexpr const T* data() const {
    return m_data.data();
  }

  [[nodiscard]] constexpr const T* data(uint2 offset) const {
    return m_data.data() + offset.y() * m_width + offset.x();
  }

  [[nodiscard]] constexpr const T& sample(float2 uv) const {
    uv.x() -= std::floor(uv.x());
    uv.y() -= std::floor(uv.y());

    uint32_t x = min(m_width - 1, m_width * uv.x());
    uint32_t y = min(m_height - 1, m_height * uv.y());

    return m_data[y * m_width + x];
  }

private:
  size_t m_width, m_height;
  std::vector<T> m_data;
};

}

#endif //YART_TEXTURE_HPP
