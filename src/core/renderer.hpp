#ifndef YART_RENDERER_HPP
#define YART_RENDERER_HPP

#include <shared_mutex>

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart {

template<typename... Ts>
using RenderCallback = typename std::optional<std::function<void(Ts...)>>;

/**
 * Abstract renderer class, defines the common interface for a renderer.
 */
class Renderer {
public:
  /**
   * General render data struct for progress reporting
   */
  struct RenderData {
    const Buffer& buffer;
    size_t samplesTaken, totalSamples;

    uint64_t totalRays;
    std::chrono::milliseconds totalTime;
  };

  /**
   * Wave data struct for progress reporting
   */
  struct WaveData {
    size_t wave, waveSamples;

    uint64_t rays;
    std::chrono::milliseconds time;
  };

  /**
   * Tile data for progress reporting
   */
  struct TileData {
    uint2 offset;
    uint2 size;
    size_t index, total;

    uint64_t rays;
    std::chrono::milliseconds time;
  };

  float3 backgroundColor;
  const Scene* scene = nullptr;

  RenderCallback<RenderData> onRenderComplete;
  RenderCallback<RenderData> onRenderAborted;
  RenderCallback<RenderData, WaveData> onRenderWaveComplete;
  RenderCallback<RenderData, TileData> onRenderTileComplete;

  Renderer(Buffer&& buffer, const Camera& camera) noexcept
    : m_camera(camera), m_buffer(std::move(buffer)) {}

  [[nodiscard]] constexpr uint32_t bufferWidth() const {
    return m_buffer.width();
  }

  [[nodiscard]] constexpr uint32_t bufferHeight() const {
    return m_buffer.height();
  }

  /**
   * Renders a scene asynchronously, without blocking the calling thread.
   * On completion, notifies the caller via the onRenderComplete callback.
   */
  virtual void render() = 0;

  /**
   * Stops a render in progress, if there is one. The render caller will be
   * notified via the onRenderAborted callback.
   * Note that depending on renderer implementation, calling abort() may not
   * stop the render immediately.
   */
  virtual void abort() = 0;

  /**
   * Waits for the renderer to finish and joins all threads. Should be called
   * when exiting and always after abort() is called.
   */
  virtual void wait() = 0;

  /**
   * Renders a scene synchronously, blocking execution on the caller until the
   * render is complete.
   * @param root Root node of the scene to be rendered
   * @return Data object including the filled buffer and data about the render
   */
  virtual RenderData renderSync() = 0;

protected:
  const Camera& m_camera;

  Buffer m_buffer;
  std::shared_mutex m_bufferMutex;
};

}

#endif //YART_RENDERER_HPP
