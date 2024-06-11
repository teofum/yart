#include "view-delegate.hpp"

namespace yart::frontend::metal {

ViewDelegate::ViewDelegate(Renderer* renderer, const Node* root)
  : MTK::ViewDelegate(), m_renderer(renderer), m_root(root) {}

ViewDelegate::~ViewDelegate() {
  m_commandQueue->release();
  m_device->release();
  m_pso->release();
  m_sso->release();
  m_vertexBuffer->release();
  m_texture->release();
}

void ViewDelegate::init(MTL::Device* device, MTK::View* view) {
  m_device = device->retain();
  m_commandQueue = m_device->newCommandQueue();
  m_view = view;

  m_viewportSize.x() = static_cast<uint32_t>(view->drawableSize().width);
  m_viewportSize.y() = static_cast<uint32_t>(view->drawableSize().height);
  buildBuffers();
  buildShaders();

  m_renderer->onRenderWaveComplete = [&](
    const Buffer& buf,
    size_t wave,
    size_t total
  ) {
    std::cout << "Samples: " << wave << "/" << total << "\n";

    m_texture->replaceRegion(
      {0, 0, buf.width(), buf.height()},
      0,
      buf.data(),
      buf.width() * sizeof(float4)
    );
  };

  m_renderer->render(*m_root);
}

void ViewDelegate::drawInMTKView(MTK::View* view) {
  {
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();

    MTL::CommandBuffer* cmd = m_commandQueue->commandBuffer();
    MTL::RenderPassDescriptor* rpd = view->currentRenderPassDescriptor();
    MTL::RenderCommandEncoder* enc = cmd->renderCommandEncoder(rpd);

    enc->setViewport(
      {0.0, 0.0,
       double(m_viewportSize.x()), double(m_viewportSize.y()),
       0.0, 1.0}
    );
    enc->setRenderPipelineState(m_pso);
    enc->setVertexBuffer(m_vertexBuffer, 0, 0);
    enc->setFragmentTexture(m_texture, 0);
    enc->setFragmentSamplerState(m_sso, 0);

    enc->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), 6);

    enc->endEncoding();

    cmd->presentDrawable(view->currentDrawable());
    cmd->commit();

    pool->release();
  }
}

void ViewDelegate::drawableSizeWillChange(MTK::View* view, CGSize size) {
  m_viewportSize.x() = static_cast<uint32_t>(size.width);
  m_viewportSize.y() = static_cast<uint32_t>(size.height);
}

void ViewDelegate::buildBuffers() {
  size_t bufferSize = 6 * 2 * sizeof(float);
  m_vertexBuffer = m_device
    ->newBuffer(bufferSize, MTL::ResourceStorageModeManaged);

  memcpy(m_vertexBuffer->contents(), m_vertexData, bufferSize);
  m_vertexBuffer
    ->didModifyRange(NS::Range::Make(0, m_vertexBuffer->length()));

  auto texDesc = MTL::TextureDescriptor::alloc()->init();
  texDesc->setPixelFormat(MTL::PixelFormatRGBA32Float);
  texDesc->setTextureType(MTL::TextureType2D);
  texDesc->setWidth(m_renderer->bufferWidth());
  texDesc->setHeight(m_renderer->bufferHeight());
  texDesc->setUsage(MTL::TextureUsageShaderRead);
  texDesc->setStorageMode(MTL::StorageModeShared);

  m_texture = m_device->newTexture(texDesc);

  auto smpDesc = MTL::SamplerDescriptor::alloc()->init();
  smpDesc->setNormalizedCoordinates(true);
  smpDesc->setMagFilter(MTL::SamplerMinMagFilterNearest);
  smpDesc->setMinFilter(MTL::SamplerMinMagFilterNearest);

  m_sso = m_device->newSamplerState(smpDesc);

  smpDesc->release();
  texDesc->release();
}

void ViewDelegate::buildShaders() {
  NS::Error* error = nullptr;
  MTL::Library* lib = m_device->newLibrary("yart.metallib"_ns, &error);
  if (!lib) {
    std::cerr << error->localizedDescription()->utf8String() << "\n";
    assert(false);
  }

  MTL::Function* vertexFunction = lib->newFunction("vertexShader"_ns);
  MTL::Function* fragmentFunction = lib->newFunction("fragmentShader"_ns);

  auto desc = MTL::RenderPipelineDescriptor::alloc()->init();
  desc->setVertexFunction(vertexFunction);
  desc->setFragmentFunction(fragmentFunction);
  desc->colorAttachments()->object(0)
      ->setPixelFormat(m_view->colorPixelFormat());

  m_pso = m_device->newRenderPipelineState(desc, &error);
  if (!m_pso) {
    std::cerr << error->localizedDescription()->utf8String() << "\n";
    assert(false);
  }

  vertexFunction->release();
  fragmentFunction->release();
  desc->release();
  lib->release();
}

}
