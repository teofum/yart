#ifndef YART_PRIMITIVES_HPP
#define YART_PRIMITIVES_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

struct Vertex {
  float3 position, normal;
  float2 textureCoords;
};

struct Face {
  size_t i0, i1, i2;
};

struct Triangle {
  Vertex v0, v1, v2;
  float3 centroid;

  constexpr Triangle(
    const Face& face,
    const std::vector<Vertex>& vertices
  ) noexcept {
    v0 = vertices[face.i0];
    v1 = vertices[face.i1];
    v2 = vertices[face.i2];
    centroid = (v0.position + v1.position + v2.position) / 3.0f;
  }
};

}

#endif //YART_PRIMITIVES_HPP
