#ifndef YART_TILE_RENDERER_HPP
#define YART_TILE_RENDERER_HPP

#include <chrono>
#include <thread>

#include <core/core.hpp>
#include <math/math.hpp>
#include "integrator.hpp"

#define DEFAULT_TILE_SIZE 64
#define DEFAULT_SAMPLE_COUNT 64
#define MAX_WAVE_SAMPLES 64

namespace yart::cpu {

/**
 * A renderer that splits the image into tiles
 * @tparam T_Integrator
 */
template<typename T_Integrator> requires std::derived_from<T_Integrator, Integrator>
class TileRenderer : public Renderer {
public:
  uint32_t samples = DEFAULT_SAMPLE_COUNT;
  uint32_t tileSize = DEFAULT_TILE_SIZE;
  uint32_t threadCount;

  TileRenderer(Buffer&& buffer, const Camera& camera) noexcept
    : Renderer(std::move(buffer), camera) {
    threadCount = std::thread::hardware_concurrency();
  }

  void render(const Node& root) override {
    renderImpl(root);
  }

  void abort() override {
    m_shouldStopRenderInProgress = true;
  }

  RenderData renderSync(const Node& root) override {
    renderImpl(root);

    for (auto& thread: m_activeThreads) thread->join();

    TimePoint now = std::chrono::high_resolution_clock::now();
    auto totalRenderTime = toMillis(now - m_renderStart);
    return {
      m_buffer,
      m_totalSamples,
      m_totalSamples,
      m_totalRays,
      totalRenderTime
    };
  }

private:
  struct Tile {
    uint2 offset;
    uint32_t width, height;
    size_t index;
  };

  // Threading state
  std::vector<std::unique_ptr<std::thread>> m_activeThreads;
  bool m_shouldStopRenderInProgress = false;

  // Tile state
  std::vector<Tile> m_queuedTiles;
  size_t m_nextTile = 0, m_finishedTiles = 0;
  std::mutex m_tileMutex;

  // Wave and samples state
  size_t m_samplesRemaining = samples, m_totalSamples = samples;
  size_t m_currentWave = 0, m_waveSamples = 1;
  std::mutex m_waveMutex;
  std::condition_variable m_waveCv;

  // Perf counters
  using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
  TimePoint m_renderStart, m_waveStart;
  uint64_t m_totalRays = 0, m_waveRays = 0;

  /**
   * Render implementation for TileRenderer
   */
  void renderImpl(const Node& root) {
    // Reset renderer state
    m_activeThreads.clear();
    m_currentWave = 0;
    m_waveSamples = 1;
    m_totalSamples = samples;
    m_samplesRemaining = samples;

    // Create tiles
    uint32_t tilesX = ceilDiv(m_buffer.width(), tileSize);
    uint32_t tilesY = ceilDiv(m_buffer.height(), tileSize);

    m_queuedTiles.clear();
    m_queuedTiles.reserve(tilesX * tilesY);
    m_nextTile = 0;

    for (size_t y = 0; y < tilesY; y++) {
      for (size_t x = 0; x < tilesX; x++) {
        Tile tile;
        tile.offset = uint2(x * tileSize, y * tileSize);
        tile.width = min(tileSize, m_buffer.width() - tile.offset.x());
        tile.height = min(tileSize, m_buffer.height() - tile.offset.y());
        tile.index = y * tilesX + x;

        m_queuedTiles.push_back(tile);
      }
    }

    m_renderStart = std::chrono::high_resolution_clock::now();
    m_waveStart = std::chrono::high_resolution_clock::now();

    // Start the rendering threads
    for (uint32_t ti = 0; ti < threadCount; ti++) {
      auto threadFunc = [&]() {
        Buffer threadBuffer(tileSize, tileSize);
        T_Integrator integrator(threadBuffer, m_camera);

        size_t threadWave = 0;

        while (m_waveSamples > 0) {
          while (threadWave == m_currentWave) {
            std::unique_lock tileLock(m_tileMutex);
            if (m_nextTile >= m_queuedTiles.size()) break;

            const Tile& tile = m_queuedTiles[m_nextTile++];
            tileLock.unlock();

            TimePoint tileStart = std::chrono::high_resolution_clock::now();
            integrator.samples = m_waveSamples;
            integrator.samplingOffset = tile.offset;
            integrator.samplingBounds = ubounds2(
              {0, 0},
              {tile.width, tile.height}
            );

            integrator.render(root);

            finishTile(tile, threadBuffer, integrator.rayCount(), tileStart);
          }

          // Increment current thread wave and sync with other threads
          threadWave++;
          std::unique_lock waveLock(m_waveMutex);
          m_waveCv.wait(waveLock, [&] { return threadWave == m_currentWave; });
        }
      };

      m_activeThreads.push_back(
        std::make_unique<std::thread>(std::thread(std::move(threadFunc)))
      );
    }
  }

  /**
   * Called when a tile is completed, updates state and notifies events
   * @param tile
   * @param threadBuffer
   */
  void finishTile(
    const Tile& tile,
    const Buffer& threadBuffer,
    uint64_t tileRays,
    TimePoint tileStart
  ) noexcept {
    // Perf counters
    TimePoint now = std::chrono::high_resolution_clock::now();
    auto tileRenderTime = toMillis(now - tileStart);
    auto waveRenderTime = toMillis(now - m_waveStart);
    auto totalRenderTime = toMillis(now - m_renderStart);

    m_waveRays += tileRays;
    m_totalRays += tileRays;

    size_t takenBefore = m_totalSamples - m_samplesRemaining;
    size_t takenAfter = takenBefore + m_waveSamples;
    float wCurrent = float(takenBefore) / float(takenAfter);
    float wWave = float(m_waveSamples) / float(takenAfter);

    std::unique_lock bufferLock(m_bufferMutex);
    for (size_t y = 0; y < tile.height; y++) {
      for (size_t x = 0; x < tile.width; x++) {
        const float4& current = m_buffer(
          x + tile.offset.x(),
          y + tile.offset.y()
        );
        const float4& wave = threadBuffer(x, y);

        m_buffer(x + tile.offset.x(), y + tile.offset.y()) =
          current * wCurrent + wave * wWave;
      }
    }

    if (onRenderTileComplete) {
      const auto cb = onRenderTileComplete.value();
      cb(
        {
          m_buffer,
          m_totalSamples - m_samplesRemaining,
          m_totalSamples,
          m_totalRays,
          totalRenderTime
        },
        {
          tile.offset,
          uint2(tile.width, tile.height),
          m_finishedTiles + 1,
          m_queuedTiles.size(),
          tileRays,
          tileRenderTime,
        }
      );
    }

    if (++m_finishedTiles == m_queuedTiles.size()) {
      std::unique_lock waveLock(m_waveMutex);
      m_nextTile = 0;
      m_finishedTiles = 0;

      m_samplesRemaining -= m_waveSamples;
      if (onRenderWaveComplete) {
        const auto cb = onRenderWaveComplete.value();
        cb(
          {
            m_buffer,
            m_totalSamples - m_samplesRemaining,
            m_totalSamples,
            m_totalRays,
            totalRenderTime
          },
          {m_currentWave, m_waveSamples, m_waveRays, waveRenderTime}
        );
      }

      size_t nextWaveSamples = m_currentWave > 0 ? min(
        m_waveSamples * 2,
        MAX_WAVE_SAMPLES
      ) : 1;
      m_waveSamples = min(nextWaveSamples, m_samplesRemaining);
      m_currentWave++;

      if (m_waveSamples == 0 && onRenderComplete) {
        const auto cb = onRenderComplete.value();
        cb(
          {
            m_buffer,
            m_totalSamples,
            m_totalSamples,
            m_totalRays,
            totalRenderTime
          }
        );
      }

      m_waveStart = std::chrono::high_resolution_clock::now();
      m_waveRays = 0;
      waveLock.unlock();
      m_waveCv.notify_all();
    }
  }
};

}

#endif //YART_TILE_RENDERER_HPP
