#ifndef YART_MATERIALS_HPP
#define YART_MATERIALS_HPP

#include <variant>

#include <math/math.hpp>

namespace yart {
using namespace math;

class Material {
public:
  float3 diffuse;
  float3 emissive;
};

struct Scattered {
  float3 attenuation;
  float3 emission;
  Ray scattered;
};

struct Emitted {
  float3 emission;
};

struct Absorbed {
};

using ScatterResult = std::variant<Scattered, Emitted, Absorbed>;

}

#endif //YART_MATERIALS_HPP
