#ifndef YART_APP_HPP
#define YART_APP_HPP

#include "app-delegate.hpp"

namespace yart::frontend::metal {

class MetalFrontend {
public:
  explicit MetalFrontend(Renderer* renderer) noexcept;
};

}

#endif //YART_APP_HPP
