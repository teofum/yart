#include "texture.hpp"

namespace yart {

HDRTexture loadTextureHDR(const char* filename) {
  int32_t w, h;
  const float* pixels = stbi_loadf(filename, &w, &h, nullptr, 4);

  HDRTexture texture(w, h, TextureType::LinearRGB);
  for (uint32_t i = 0; i < w * h; i++) {
    uint32_t iPixel = i * 4, iData = i * 3;
    texture.data[iData + 0] = pixels[iPixel + 0];
    texture.data[iData + 1] = pixels[iPixel + 1];
    texture.data[iData + 2] = pixels[iPixel + 2];
  }

  stbi_image_free((void*) pixels);
  return texture;
}

uint2 getXY(float2& uv, uint32_t w, uint32_t h) {
  uv.x() -= std::floor(uv.x());
  uv.y() -= std::floor(uv.y());

  uv.x() *= float(w - 1);
  uv.y() *= float(h - 1);

  uint32_t x = min(w - 2, uv.x());
  uint32_t y = min(h - 2, uv.y());

  uv.x() -= x;
  uv.y() -= y;

  return {x, y};
}

}