#include "app.hpp"

namespace yart::frontend::metal {

MetalFrontend::MetalFrontend(Renderer* renderer) noexcept {
  // Autorelease pool used for reference counting
  // https://developer.apple.com/documentation/foundation/nsautoreleasepool
  NS::AutoreleasePool* autoreleasePool = NS::AutoreleasePool::alloc()->init();

  AppDelegate del(
    new ViewDelegate(renderer),
    {renderer->bufferWidth(), renderer->bufferHeight()}
  );

  // NSApplication object manages the main event loop and delegates
  // https://developer.apple.com/documentation/appkit/nsapplication
  NS::Application* sharedApplication = NS::Application::sharedApplication();
  sharedApplication->setDelegate(&del);
  sharedApplication->run();

  autoreleasePool->release();
}

}