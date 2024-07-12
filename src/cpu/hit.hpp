#ifndef YART_HIT_HPP
#define YART_HIT_HPP

#include <core/core.hpp>

namespace yart::cpu {

class Hit {
public:
  float t = std::numeric_limits<float>::infinity();
  float2 uv;
  float3 p, n, tg, attenuation = float3(1.0f);
  const BSDF* bsdf = nullptr;
  const TriangleData* d = nullptr;
  int32_t lightIdx = -1;
  uint32_t idx;
};

}

#endif //YART_HIT_HPP
