#ifndef YART_HIT_HPP
#define YART_HIT_HPP

namespace yart::cpu {

class Hit {
public:
  float t;
  float3 position, normal;
  const Material* material;
};

}

#endif //YART_HIT_HPP
