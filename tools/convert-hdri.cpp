#include <core/core.hpp>
#include <math/math.hpp>

#include <stb_image_write.h>

using namespace yart;
using namespace math;

int main(int argc, char** argv) {
  if (argc < 2) return 1;
  // Load equirectangular mapped texture
  yart::Texture equi = yart::Texture::loadHDR(argv[1]);

  // Create octahedral texture
  uint32_t size = min(equi.width(), equi.height());
  yart::Texture octa(size, size);

  for (uint32_t y = 0; y < size; y++) {
    for (uint32_t x = 0; x < size; x++) {
      float2 uv(float(x) + 0.5f, float(y) + 0.5f);
      uv /= float(size);

      float3 dir = invOctahedralUV(uv);
      float2 equiUv = sphericalUV(dir);

      octa(x, y) = equi.sample(equiUv);
    }
  }

  stbi_write_hdr(argv[2], int(size), int(size), 4, (float*) (octa.data()));
}
