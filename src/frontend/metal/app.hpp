#ifndef YART_APP_HPP
#define YART_APP_HPP

#include "app-delegate.hpp"

namespace yart::frontend::metal {

class MetalFrontend {
public:
  MetalFrontend(Renderer* renderer, const Node* root) noexcept;
};

}

#endif //YART_APP_HPP
