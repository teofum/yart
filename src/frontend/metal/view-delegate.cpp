#include <iomanip>

#include "view-delegate.hpp"

namespace yart::frontend::metal {

ViewDelegate::ViewDelegate(Renderer* renderer)
  : MTK::ViewDelegate(), m_renderer(renderer) {}

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

  m_renderer->onRenderTileComplete = [&](
    Renderer::RenderData renderData,
    Renderer::TileData tileData
  ) {
    m_texture->replaceRegion(
      {tileData.offset.x(), tileData.offset.y(),
       tileData.size.x(), tileData.size.y()},
      0,
      renderData.buffer.data(tileData.offset),
      renderData.buffer.width() * sizeof(float4)
    );
  };

  m_renderer->onRenderWaveComplete = [&](
    Renderer::RenderData renderData,
    Renderer::WaveData waveData
  ) {
    float wavePerf = float(waveData.rays) / float(1000 * waveData.time.count());
    float perf =
      float(renderData.totalRays) / float(1000 * renderData.totalTime.count());

    size_t nPixels = renderData.buffer.height() * renderData.buffer.width();
    size_t waveSamples = waveData.waveSamples * nPixels;
    float waveSamplePerf =
      float(waveSamples) / float(1000 * waveData.time.count());
    size_t totalSamples = renderData.samplesTaken * nPixels;
    float samplePerf =
      float(totalSamples) / float(1000 * renderData.totalTime.count());

    std::cout << "Finished wave " << waveData.wave + 1
              << " (" << renderData.samplesTaken
              << "/" << renderData.totalSamples
              << " samples, "
              << waveData.waveSamples << " wave samples)\n";

    std::cout << "  Wave:  "
              << std::setw(8) << waveData.time << ", "
              << std::setw(12) << waveData.rays << " rays ["
              << std::setw(6) << std::fixed << std::setprecision(3)
              << wavePerf << " Mrays/s, "
              << std::setw(6) << std::fixed << std::setprecision(3)
              << waveSamplePerf << " Msam/s]\n";

    std::cout << "  Total: "
              << std::setw(8) << renderData.totalTime << ", "
              << std::setw(12) << renderData.totalRays << " rays ["
              << std::setw(6) << std::fixed << std::setprecision(3)
              << perf << " Mrays/s, "
              << std::setw(6) << std::fixed << std::setprecision(3)
              << samplePerf << " Msam/s]\n";
  };

  m_renderer->onRenderComplete = [&](Renderer::RenderData renderData) {
    float perf =
      float(renderData.totalRays) / float(1000 * renderData.totalTime.count());
    std::cout << "Done!\n";
    std::cout << "  Total: "
              << std::setw(8) << renderData.totalTime << ", "
              << std::setw(12) << renderData.totalRays << " rays ["
              << std::setw(6) << std::fixed << std::setprecision(3)
              << perf << " Mrays/s]\n";
  };

  m_renderer->render();
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
