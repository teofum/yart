#include <core/types.hpp>

#include "ppm.hpp"

namespace yart::output {

constexpr float gamma = 2.2f;

void writePPM(std::ostream& ostream, const Buffer& buffer) {
  size_t width = buffer.width(), height = buffer.height();

  ostream << "P6\n" << width << " " << height << "\n255\n";

  for (size_t y = 0; y < height; y++) {
    for (size_t x = 0; x < width; x++) {
      for (size_t c = 0; c < 3; c++) {
        float mapped = std::clamp(std::pow(buffer(x, y)[c], gamma), 0.0f, 1.0f);
        auto byte = uint8(mapped * 255.999f);
        ostream << byte;
      }
    }
  }
}

}
