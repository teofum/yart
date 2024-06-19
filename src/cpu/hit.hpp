#ifndef YART_HIT_HPP
#define YART_HIT_HPP

namespace yart::cpu {

class Hit {
public:
  float t = std::numeric_limits<float>::infinity();
  float3 p, n;
  const BSDF* bsdf;
};

}

#endif //YART_HIT_HPP
