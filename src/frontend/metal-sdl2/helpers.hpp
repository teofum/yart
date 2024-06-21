#ifndef YART_HELPERS_HPP
#define YART_HELPERS_HPP

#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

namespace yart::frontend::sdl2_metal {

MTL::Device* getDevice(CA::MetalLayer* layer) noexcept;

CA::MetalDrawable* nextDrawable(CA::MetalLayer* layer) noexcept;

void setupLayer(CA::MetalLayer* layer) noexcept;

void setDrawableSize(CA::MetalLayer* layer, int width, int height) noexcept;

}

#endif //YART_HELPERS_HPP
