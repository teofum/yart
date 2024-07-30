#ifndef YART_TEXTURE_HPP
#define YART_TEXTURE_HPP

#include <stb_image.h>

#include <math/math.hpp>

#include "buffer.hpp"
#include "color-utils.hpp"

namespace yart {
using namespace math;

enum class TextureType {
  LinearRGB = 0,
  sRGB,
  NonColor
};

template<numeric T, std::size_t C>
class Texture {
public:
  std::vector<T> data;

  constexpr Texture(unsigned width, unsigned height, TextureType type) noexcept
    : data(width * height * C, T(0)),
      m_width(width),
      m_height(height),
      m_channels(C),
      m_type(type) {}

  [[nodiscard]] vec<float, C> sample(float2 uv) const requires (C > 1);

  [[nodiscard]] float sample(float2 uv) const requires (C == 1);

  [[nodiscard]] constexpr TextureType type() const noexcept { return m_type; }

  [[nodiscard]] constexpr const uint32_t& width() const {
    return m_width;
  }

  [[nodiscard]] constexpr const uint32_t& height() const {
    return m_height;
  }

private:
  uint32_t m_width, m_height, m_channels;
  TextureType m_type;
};

/**
 * Type aliases
 */
using HDRTexture = Texture<float, 3>;

template<size_t C>
using SDRTexture = Texture<uint8_t, C>;
using MonoTexture = SDRTexture<1>;
using RGBTexture = SDRTexture<3>;
using RGBATexture = SDRTexture<4>;

template<size_t C>
SDRTexture<C> loadTexture(
  const uint8_t* data,
  int32_t len,
  TextureType type,
  std::array<uint32_t, C> channels
) {
  int32_t w, h;
  const uint8_t* pixels = stbi_load_from_memory(data, len, &w, &h, nullptr, 4);

  SDRTexture<C> texture(w, h, type);
  for (uint32_t i = 0; i < w * h; i++) {
    uint32_t iPixel = i * 4, iData = i * C;

    for (uint32_t j = 0; j < C; j++) {
      uint8_t pixel = pixels[iPixel + channels[j]];
      if (type == TextureType::sRGB) {
        // Decode and re-encode with gamma 2. This preserves most detail, but is
        // faster to decode on sample than sRGB
        float val = sRGBDecode(float(pixel) / 255.0f);
        val = std::sqrt(val);
        pixel = uint8_t(val * 255.0f);
      }

      texture.data[iData + j] = pixel;
    }
  }

  stbi_image_free((void*) pixels);
  return texture;
}

template<size_t C>
SDRTexture<C> loadTexture(const uint8_t* data, int32_t len, TextureType type) {
  std::array<uint32_t, C> channels = {};
  for (uint32_t i = 0; i < C; i++) channels[i] = i;
  return loadTexture<C>(data, len, type, channels);
}

HDRTexture loadTextureHDR(const char* filename);

uint2 getXY(float2& uv, uint32_t w, uint32_t h);

template<std::integral T, std::size_t C>
vec<float, C> getValue(const Texture<T, C>& tex, size_t idx) {
  vec<float, C> val;
  for (uint32_t i = 0; i < C; i++)
    val[i] = float(tex.data[C * idx + i]) / 255.0f;

  // Decode gamma 2 encoding
  if (tex.type() == TextureType::sRGB)
    for (uint32_t i = 0; i < min(C, 3); i++) val[i] = val[i] * val[i];
  return val;
}

template<std::floating_point T, std::size_t C>
vec<float, C> getValue(const Texture<T, C>& tex, size_t idx) {
  vec<float, C> val;
  for (uint32_t i = 0; i < C; i++) val[i] = tex.data[C * idx + i];

  return val;
}

template<numeric T, std::size_t C>
vec<float, C> Texture<T, C>::sample(float2 uv) const requires (C > 1) {
  uint2 xy = getXY(uv, m_width, m_height);

  vec<float, C> samples[4] = {
    getValue<T, C>(*this, xy[1] * m_width + xy[0]),
    getValue<T, C>(*this, (xy[1] + 1) * m_width + xy[0]),
    getValue<T, C>(*this, xy[1] * m_width + (xy[0] + 1)),
    getValue<T, C>(*this, (xy[1] + 1) * m_width + (xy[0] + 1))
  };

  return bilerp(samples[0], samples[1], samples[2], samples[3], uv.x(), uv.y());
}

template<std::integral T>
float getValueF(const Texture<T, 1>& tex, size_t idx) {
  return float(tex.data[idx]) / 255.0f;
}

template<std::floating_point T>
float getValueF(const Texture<T, 1>& tex, size_t idx) {
  return tex.data[idx];
}

template<numeric T, std::size_t C>
float Texture<T, C>::sample(float2 uv) const requires (C == 1) {
  uint2 xy = getXY(uv, m_width, m_height);

  float samples[4] = {
    getValueF<T>(*this, C * (xy[1] * m_width + xy[0])),
    getValueF<T>(*this, C * ((xy[1] + 1) * m_width + xy[0])),
    getValueF<T>(*this, C * (xy[1] * m_width + (xy[0] + 1))),
    getValueF<T>(*this, C * ((xy[1] + 1) * m_width + (xy[0] + 1)))
  };

  return bilerp(samples[0], samples[1], samples[2], samples[3], uv.x(), uv.y());
}

}

#endif //YART_TEXTURE_HPP
