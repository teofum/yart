#ifndef YART_MAIN_HPP
#define YART_MAIN_HPP

#include <SDL2/SDL.h>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include <core/core.hpp>
#include <math/math.hpp>

namespace yart::frontend {

class MetalSDLFrontend {
public:
  explicit MetalSDLFrontend(Renderer* renderer) noexcept;

  ~MetalSDLFrontend();

  void start() noexcept;

private:
  Renderer* m_renderer;
  uint2 m_viewportSize = {0, 0};
  float4x4 m_viewTransform = float4x4::scaling(2.0f);

  SDL_Window* m_sdlWindow = nullptr;
  SDL_Renderer* m_sdlRenderer = nullptr;

  CA::MetalLayer* m_layer = nullptr;
  MTL::Device* m_device = nullptr;
  MTL::CommandQueue* m_commandQueue = nullptr;
  MTL::RenderPassDescriptor* m_rpd = nullptr;
  MTL::RenderPipelineState* m_pso = nullptr;
  MTL::SamplerState* m_sso = nullptr;
  MTL::Buffer* m_vertexBuffer = nullptr;
  MTL::Texture* m_texture = nullptr;

  const uint8_t* m_keys = nullptr;

  static constexpr const float m_vertexData[][2] = {
    {0, 0},
    {1, 0},
    {1, 1},
    {0, 0},
    {1, 1},
    {0, 1},
  };

  void buildBuffers();

  void buildShaders();

  void drawFrame() noexcept;

  void startRenderer() noexcept;

  void handleInput(const SDL_Event& event) noexcept;
};

}

#endif //YART_MAIN_HPP
