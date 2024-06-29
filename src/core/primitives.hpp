#ifndef YART_PRIMITIVES_HPP
#define YART_PRIMITIVES_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

struct Vertex {
  float3 p, normal;
  float4 tangent;
  float2 texCoords;
};

struct Face {
  size_t i0, i1, i2;
  uint32_t materialIdx;
};

struct TrianglePositions {
  float3 p0, p1, p2;

  [[nodiscard]] constexpr float area() const noexcept {
    const float3 edge1 = p1 - p0;
    const float3 edge2 = p2 - p0;
    return length(cross(edge1, edge2)) * 0.5f;
  }
};

struct TriangleData {
  float3 n[3];
  float4 t[3];
  float2 texCoords[3];
};

constexpr void createTriangle(
  const Face& face,
  const std::vector<Vertex>& vertices,
  TrianglePositions* positions,
  TriangleData* data,
  float3* centroid,
  uint32_t* materialIdx
) {
  const Vertex& v0 = vertices[face.i0];
  const Vertex& v1 = vertices[face.i1];
  const Vertex& v2 = vertices[face.i2];

  positions->p0 = v0.p;
  positions->p1 = v1.p;
  positions->p2 = v2.p;

  data->n[0] = v0.normal;
  data->n[1] = v1.normal;
  data->n[2] = v2.normal;

  data->t[0] = v0.tangent;
  data->t[1] = v1.tangent;
  data->t[2] = v2.tangent;

  data->texCoords[0] = v0.texCoords;
  data->texCoords[1] = v1.texCoords;
  data->texCoords[2] = v2.texCoords;

  *materialIdx = face.materialIdx;
  *centroid = (v0.p + v1.p + v2.p) / 3.0f;
}

}

#endif //YART_PRIMITIVES_HPP
