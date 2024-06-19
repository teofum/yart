#ifndef YART_HIT_HPP
#define YART_HIT_HPP

#include <core/core.hpp>

namespace yart::cpu {

class Hit {
public:
  float t = std::numeric_limits<float>::infinity();
  float3 p, n;
  const BSDF* bsdf = nullptr;
  const Light* light = nullptr;
};

}

#endif //YART_HIT_HPP
