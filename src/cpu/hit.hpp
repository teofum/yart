#ifndef YART_HIT_HPP
#define YART_HIT_HPP

namespace yart::cpu {

class Hit {
public:
  float t = std::numeric_limits<float>::infinity();
  float3 position, normal;
  const Material* material;
};

}

#endif //YART_HIT_HPP
