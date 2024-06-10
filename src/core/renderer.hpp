#ifndef YART_RENDERER_HPP
#define YART_RENDERER_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {

class Renderer {
public:
  Buffer& buffer;
  const Camera& camera;
  float3 backgroundColor;

  constexpr Renderer(Buffer& buffer, const Camera& camera) noexcept
    : buffer(buffer), camera(camera) {
  }

  virtual void render(const Node& root) = 0;
};

}

#endif //YART_RENDERER_HPP
