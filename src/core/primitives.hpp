#ifndef YART_PRIMITIVES_HPP
#define YART_PRIMITIVES_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

struct VertexData {
  float3 normal;
  float4 tangent;
  float2 texCoords;
};

struct Face {
  uint32_t i0, i1, i2;
  uint32_t materialIdx;
};

struct Triangle {
  uint32_t i0, i1, i2;
};

[[nodiscard]] constexpr float triangleArea(
  const float3& p0,
  const float3& p1,
  const float3& p2
) noexcept {
  const float3 edge1 = p1 - p0;
  const float3 edge2 = p2 - p0;
  return length(cross(edge1, edge2)) * 0.5f;
}

constexpr void createTriangle(
  const Face& face,
  const std::vector<float3>& vertices,
  Triangle* tri,
  float3* centroid,
  uint32_t* materialIdx
) {
  const float3& v0 = vertices[tri->i0 = face.i0];
  const float3& v1 = vertices[tri->i1 = face.i1];
  const float3& v2 = vertices[tri->i2 = face.i2];

  *materialIdx = face.materialIdx;
  *centroid = (v0 + v1 + v2) / 3.0f;
}

}

#endif //YART_PRIMITIVES_HPP
