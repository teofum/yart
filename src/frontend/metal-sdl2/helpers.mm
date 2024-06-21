#include <Metal/Metal.h>
#include <QuartzCore/QuartzCore.h>

#include "helpers.hpp"

namespace yart::frontend::sdl2_metal {

MTL::Device* getDevice(CA::MetalLayer* layer) noexcept {
  CAMetalLayer* metalLayer = (CAMetalLayer*) layer;
  MTL::Device* cppDevice = ( __bridge MTL::Device*) metalLayer.device;
  return cppDevice;
}

CA::MetalDrawable* nextDrawable(CA::MetalLayer* layer) noexcept {
  CAMetalLayer* metalLayer = (CAMetalLayer*) layer;
  id <CAMetalDrawable> metalDrawable = [metalLayer nextDrawable];
  CA::MetalDrawable* cppDrawable = ( __bridge CA::MetalDrawable*) metalDrawable;
  return cppDrawable;
}

void setupLayer(CA::MetalLayer* layer) noexcept {
  CAMetalLayer* metalLayer = (CAMetalLayer*) layer;
  metalLayer.pixelFormat = MTLPixelFormatRGBA8Unorm_sRGB;
}

void setDrawableSize(CA::MetalLayer* layer, int width, int height) noexcept {
  CAMetalLayer* metalLayer = (CAMetalLayer*) layer;
  metalLayer.drawableSize = CGSize{float(width), float(height)};
}

}
