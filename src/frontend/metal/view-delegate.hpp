#ifndef YART_VIEW_DELEGATE_HPP
#define YART_VIEW_DELEGATE_HPP

#include <Metal/Metal.hpp>
#include <AppKit/AppKit.hpp>
#include <MetalKit/MetalKit.hpp>

#include <math/math.hpp>
#include <core/core.hpp>

#include "utils.hpp"

namespace yart::frontend::metal {
using namespace math;

class ViewDelegate : public MTK::ViewDelegate {
public:
  ViewDelegate(Renderer* renderer, const Node* root);

  ~ViewDelegate() override;

  void init(MTL::Device* device, MTK::View* view);

  void drawInMTKView(MTK::View* view) override;

  void drawableSizeWillChange(MTK::View* view, CGSize size) override;

protected:
  MTL::Device* m_device = nullptr;
  MTL::CommandQueue* m_commandQueue = nullptr;
  MTK::View* m_view = nullptr;
  MTL::RenderPipelineState* m_pso = nullptr;
  MTL::SamplerState* m_sso = nullptr;
  MTL::Buffer* m_vertexBuffer = nullptr;
  MTL::Texture* m_texture = nullptr;

  uint2 m_viewportSize = {0, 0};
  Renderer* m_renderer;
  const Node* m_root;

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
};

}

#endif
