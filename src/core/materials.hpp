#ifndef YART_MATERIALS_HPP
#define YART_MATERIALS_HPP

#include <variant>

#include <math/math.hpp>

namespace yart {
using namespace math;

struct Lambertian {
  float3 albedo;
};

struct Emissive {
  float3 emission;
};

using Material = std::variant<Lambertian, Emissive>;

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
