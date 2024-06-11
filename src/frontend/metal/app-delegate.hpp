#ifndef YART_APP_DELEGATE_HPP
#define YART_APP_DELEGATE_HPP

#include <Metal/Metal.hpp>
#include <AppKit/AppKit.hpp>
#include <MetalKit/MetalKit.hpp>

#include "view-delegate.hpp"
#include "math/vec.hpp"

namespace yart::frontend::metal {
using namespace math;

/**
 * Delegate class: implements app functionality, receives notifications from the
 * NSApplication object
 */
class AppDelegate : public NS::ApplicationDelegate {
public:
  explicit AppDelegate(
    ViewDelegate* viewDelegate,
    const float2& size,
    const char* title = "yart"
  );

  ~AppDelegate() override;

  static NS::Menu* createMenuBar();

  void applicationDidFinishLaunching(NS::Notification* notification) override;

  void applicationWillFinishLaunching(NS::Notification* notification) override;

  bool applicationShouldTerminateAfterLastWindowClosed(NS::Application* sender) override;

private:
  const char* m_title;
  float2 m_size;
  NS::Window* m_window = nullptr;
  MTK::View* m_mtkView = nullptr;
  MTL::Device* m_device = nullptr;

  ViewDelegate* m_viewDelegate;
};

}

#endif
