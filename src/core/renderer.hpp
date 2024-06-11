#ifndef YART_RENDERER_HPP
#define YART_RENDERER_HPP

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {

class Renderer {
public:
  float3 backgroundColor;
  std::optional<std::function<void(const Buffer&)>> onRenderComplete;
  std::optional<std::function<void(
    const Buffer&,
    size_t,
    size_t
  )>> onRenderWaveComplete;

  constexpr Renderer(const Buffer& buffer, const Camera& camera) noexcept
    : m_buffer(buffer), m_camera(camera) {}

  constexpr Renderer(Buffer&& buffer, const Camera& camera) noexcept
    : m_buffer(std::move(buffer)), m_camera(camera) {}

  [[nodiscard]] constexpr uint2 bufferSize() const {
    return {m_buffer.width(), m_buffer.height()};
  }

  [[nodiscard]] constexpr uint32_t bufferWidth() const {
    return m_buffer.width();
  }

  [[nodiscard]] constexpr uint32_t bufferHeight() const {
    return m_buffer.height();
  }

  virtual void render(const Node& root) = 0;

protected:
  Buffer m_buffer;
  const Camera& m_camera;
};

}

#endif //YART_RENDERER_HPP
