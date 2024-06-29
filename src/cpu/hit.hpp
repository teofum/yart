#ifndef YART_HIT_HPP
#define YART_HIT_HPP

#include <core/core.hpp>

namespace yart::cpu {

class Hit {
public:
  float t = std::numeric_limits<float>::infinity();
  float2 uv;
  float3 p, n, tg;
  float4 st;
  const BSDF* bsdf = nullptr;
  const TrianglePositions* tri = nullptr;
  int32_t lightIdx = -1;
  uint32_t idx;
};

}

#endif //YART_HIT_HPP
