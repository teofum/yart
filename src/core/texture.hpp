#ifndef YART_TEXTURE_HPP
#define YART_TEXTURE_HPP

#include <stb_image.h>

#include <math/math.hpp>

#include "buffer.hpp"
#include "color-utils.hpp"

namespace yart {
using namespace math;

class Texture : public Buffer {
public:
  enum class Type {
    LinearRGB = 0,
    sRGB,
    NonColor
  };

  static Texture load(const uint8_t* data, int32_t len, Type type) {
    int32_t w, h;
    const uint8_t* pixels =
      stbi_load_from_memory(data, len, &w, &h, nullptr, 4);

    Texture texture(w, h);
    for (uint32_t i = 0; i < w * h; i++) {
      uint32_t iPixel = i * 4;
      texture.m_data[i] = float4(
        pixels[iPixel + 0],
        pixels[iPixel + 1],
        pixels[iPixel + 2],
        pixels[iPixel + 3]
      ) / 255.0f;

      if (type == Type::sRGB)
        texture.m_data[i] = sRGBDecode(texture.m_data[i]);
    }

    return texture;
  }

  static Texture load(const char* filename, Type type) {
    int32_t w, h;
    const uint8_t* pixels = stbi_load(filename, &w, &h, nullptr, 4);

    Texture texture(w, h);
    for (uint32_t i = 0; i < w * h; i++) {
      uint32_t iPixel = i * 4;
      texture.m_data[i] = float4(
        pixels[iPixel + 0],
        pixels[iPixel + 1],
        pixels[iPixel + 2],
        pixels[iPixel + 3]
      ) / 255.0f;

      if (type == Type::sRGB)
        texture.m_data[i] = sRGBDecode(texture.m_data[i]);
    }

    return texture;
  }


  static Texture loadHDR(const char* filename) {
    int32_t w, h;
    const float* pixels = stbi_loadf(filename, &w, &h, nullptr, 4);

    Texture texture(w, h);
    for (uint32_t i = 0; i < w * h; i++) {
      uint32_t iPixel = i * 4;
      texture.m_data[i] = float4(
        pixels[iPixel + 0],
        pixels[iPixel + 1],
        pixels[iPixel + 2],
        pixels[iPixel + 3]
      );
    }

    return texture;
  }

  constexpr Texture(unsigned width, unsigned height) noexcept
    : Buffer(width, height) {}

  [[nodiscard]] constexpr const float4& sample(float2 uv) const {
    uv.x() -= std::floor(uv.x());
    uv.y() -= std::floor(uv.y());

    uint32_t x = min(m_width - 1, float(m_width) * uv.x());
    uint32_t y = min(m_height - 1, float(m_height) * uv.y());

    return m_data[y * m_width + x];
  }
};

}

#endif //YART_TEXTURE_HPP
