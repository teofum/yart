#include <iomanip>

#include "main.hpp"
#include "helpers.hpp"

namespace yart::frontend {

[[nodiscard]] static constexpr NS::String* operator ""_ns(
  const char* cStr,
  size_t len
) noexcept {
  return NS::String::string(cStr, NS::UTF8StringEncoding);
}

MetalSDLFrontend::MetalSDLFrontend(yart::Renderer* renderer) noexcept
  : m_renderer(renderer) {}

MetalSDLFrontend::~MetalSDLFrontend() {
  m_commandQueue->release();
  m_device->release();
  m_rpd->release();
  m_pso->release();
  m_sso->release();
  m_vertexBuffer->release();
  m_texture->release();
}

void MetalSDLFrontend::start() noexcept {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::cerr << "SDL Init failed: " << SDL_GetError() << "\n";
    return;
  }

  SDL_SetHint(SDL_HINT_RENDER_DRIVER, "metal");
  SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");

  m_sdlWindow = SDL_CreateWindow(
    "yart [SDL2 + Metal]",
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED,
    int(m_renderer->bufferWidth()),
    int(m_renderer->bufferHeight()),
    SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI
  );

  m_sdlRenderer = SDL_CreateRenderer(
    m_sdlWindow,
    -1,
    SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
  );

  m_keys = SDL_GetKeyboardState(nullptr);

  m_layer = static_cast<CA::MetalLayer*>(SDL_RenderGetMetalLayer(m_sdlRenderer));
  m_device = sdl2_metal::getDevice(m_layer);

  m_commandQueue = m_device->newCommandQueue();
  m_rpd = MTL::RenderPassDescriptor::alloc()->init();

  buildBuffers();
  buildShaders();

  startRenderer();

  bool exit = false;
  while (!exit) {
    NS::AutoreleasePool* autoreleasePool = NS::AutoreleasePool::alloc()->init();

    // Event polling
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT || (
        event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID == SDL_GetWindowID(m_sdlWindow)
      )) {
        m_renderer->abort();
        exit = true;
      } else {
        handleInput(event);
      }
    }

    // Handle resize
    int width, height;
    SDL_GetRendererOutputSize(m_sdlRenderer, &width, &height);
    sdl2_metal::setDrawableSize(m_layer, width, height);
    m_viewportSize.x() = width;
    m_viewportSize.y() = height;

    drawFrame();

    autoreleasePool->release();
  }

  m_renderer->wait();

  SDL_DestroyRenderer(m_sdlRenderer);
  SDL_DestroyWindow(m_sdlWindow);
  SDL_Quit();
}

void MetalSDLFrontend::drawFrame() noexcept {
  CA::MetalDrawable* drawable = sdl2_metal::nextDrawable(m_layer);

  MTL::CommandBuffer* cmd = m_commandQueue->commandBuffer();
  auto* colorAttachment = m_rpd->colorAttachments()->object(0);
  colorAttachment->setClearColor(MTL::ClearColor::Make(0.0, 0.0, 0.0, 1.0));
  colorAttachment->setTexture(drawable->texture());
  colorAttachment->setLoadAction(MTL::LoadActionClear);
  colorAttachment->setStoreAction(MTL::StoreActionStore);

  MTL::RenderCommandEncoder* enc = cmd->renderCommandEncoder(m_rpd);

  float2 bufferSize(m_renderer->bufferWidth(), m_renderer->bufferHeight());
  float2 viewportSize(m_viewportSize);
  float4x4 transformTr = transpose(m_viewTransform);

  enc->setViewport(
    {0.0, 0.0,
     double(m_viewportSize.x()), double(m_viewportSize.y()),
     0.0, 1.0}
  );
  enc->setRenderPipelineState(m_pso);
  enc->setVertexBuffer(m_vertexBuffer, 0, 0);
  enc->setVertexBytes(&viewportSize, sizeof(float2), 1);
  enc->setVertexBytes(&bufferSize, sizeof(float2), 2);
  enc->setVertexBytes(&transformTr, sizeof(float4x4), 3);

  enc->setFragmentTexture(m_texture, 0);
  enc->setFragmentSamplerState(m_sso, 0);

  enc->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), 6);

  enc->endEncoding();

  cmd->presentDrawable(drawable);
  cmd->commit();
}

void MetalSDLFrontend::buildBuffers() {
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

void MetalSDLFrontend::buildShaders() {
  NS::Error* error = nullptr;
  MTL::Library* lib = m_device->newLibrary("yart.metallib"_ns, &error);
  if (!lib) {
    std::cerr << error->localizedDescription()->utf8String() << "\n";
    abort();
  }

  MTL::Function* vertexFunction = lib->newFunction("vertexShader"_ns);
  MTL::Function* fragmentFunction = lib->newFunction("fragmentShader"_ns);

  auto desc = MTL::RenderPipelineDescriptor::alloc()->init();
  desc->setVertexFunction(vertexFunction);
  desc->setFragmentFunction(fragmentFunction);
  desc->colorAttachments()->object(0)
      ->setPixelFormat(MTL::PixelFormatRGBA8Unorm_sRGB);

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

void MetalSDLFrontend::startRenderer() noexcept {
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

  m_renderer->onRenderAborted = [&](Renderer::RenderData renderData) {
    std::cout << "Render aborted\n";
    std::cout << "  Total: "
              << std::setw(8) << renderData.totalTime << ", "
              << std::setw(12) << renderData.totalRays << " rays\n";
  };

  m_renderer->render();
}

void MetalSDLFrontend::handleInput(const SDL_Event& event) noexcept {
  if (event.type == SDL_MOUSEWHEEL) {
    if (m_keys[SDL_SCANCODE_LGUI] || m_keys[SDL_SCANCODE_RGUI]) {
      float2 mouseOffset(event.wheel.mouseX, event.wheel.mouseY);
      mouseOffset *= 2.0f;
      mouseOffset.y() = float(m_viewportSize.y()) - mouseOffset.y();
      mouseOffset -= float2(m_viewportSize) / 2.0f;
      mouseOffset *= 2.0f;

      float zoom = std::exp(event.wheel.preciseY * 0.05f);

      float4x4 t = float4x4::translation(float3(mouseOffset, 0.0f));
      float4x4 ti = float4x4::translation(float3(-mouseOffset, 0.0f));
      float4x4 s = float4x4::scaling(zoom);

      m_viewTransform = t * s * ti * m_viewTransform;
    } else {
      float2 amt(event.wheel.preciseX, event.wheel.preciseY);
      float4x4 t = float4x4::translation(float3(amt * -20.0f, 0.0f));

      m_viewTransform = t * m_viewTransform;
    }
  }
}

}
