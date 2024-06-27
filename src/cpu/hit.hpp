#ifndef YART_HIT_HPP
#define YART_HIT_HPP

#include <core/core.hpp>

namespace yart::cpu {

class Hit {
public:
  float t = std::numeric_limits<float>::infinity(), u, v;
  float3 p, n, tg;
  const BSDF* bsdf = nullptr;
  const Light* light = nullptr;
  int32_t lightIdx = -1;
  uint32_t idx;
};

}

#endif //YART_HIT_HPP
