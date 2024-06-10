#ifndef YART_MESH_HPP
#define YART_MESH_HPP

#include <ranges>

#include <math/math.hpp>
#include "materials.hpp"

namespace yart {
using namespace math;

struct BoundingBox {
  float3 bounds[2] = {
    float3(std::numeric_limits<float>::infinity()),
    float3(-std::numeric_limits<float>::infinity()),
  };

  [[nodiscard]] constexpr const float3& min() const noexcept {
    return bounds[0];
  }

  [[nodiscard]] constexpr const float3& max() const noexcept {
    return bounds[1];
  }
};

struct Vertex {
  float3 position, normal;
  float2 textureCoords;
};

struct Face {
  size_t indices[3];
};

struct Triangle {
  Vertex vertices[3];
};

class Mesh {
private:
  std::vector<Vertex> m_vertices;
  std::vector<Face> m_triangles;

public:
  enum class FaceCulling : int {
    None = 0, Front, Back
  };

  FaceCulling faceCulling = FaceCulling::None;
  const Material* material;

  constexpr Mesh(
    const std::vector<Vertex>& vertices,
    const std::vector<Face>& triangles,
    const Material* mat
  ) noexcept
    : m_vertices(vertices), m_triangles(triangles), material(mat) {
  }

  constexpr Mesh(
    std::vector<Vertex>&& vertices,
    std::vector<Face>&& triangles,
    const Material* mat
  ) noexcept
    : m_vertices(std::move(vertices)), m_triangles(std::move(triangles)),
      material(mat) {
  }

  [[nodiscard]] constexpr auto triangles() const noexcept {
    return std::views::transform(
      m_triangles, [&](const Face& face) {
        return Triangle{
          m_vertices[face.indices[0]],
          m_vertices[face.indices[1]],
          m_vertices[face.indices[2]],
        };
      }
    );
  }

  [[nodiscard]] constexpr const std::vector<Vertex>& vertices() const noexcept {
    return m_vertices;
  }

  [[nodiscard]] constexpr auto vertexPositions() const noexcept {
    return std::views::transform(
      m_vertices, [&](const Vertex& vertex) {
        return vertex.position;
      }
    );
  }
};

template<typename range_T>
requires std::ranges::random_access_range<range_T>
[[nodiscard]] constexpr BoundingBox getBoundingBox(const range_T& vertices) noexcept {
  float3 min(std::numeric_limits<float>::infinity());
  float3 max = -min;

  for (const float3& vert: vertices) {
    for (size_t i = 0; i < 3; i++) {
      if (vert[i] < min[i]) min[i] = vert[i];
      if (vert[i] > max[i]) max[i] = vert[i];
    }
  }

  return {min - float3(0.001f), max + float3(0.001f)};
}

template<typename... Ts>
requires ((std::convertible_to<Ts, BoundingBox>) && ...)
[[nodiscard]] constexpr BoundingBox combineBoundingBoxes(const Ts& ...boxes) noexcept {
  float3 min(std::numeric_limits<float>::infinity());
  float3 max = -min;

  for (const BoundingBox& box: {boxes...}) {
    for (size_t i = 0; i < 3; i++) {
      if (box.min()[i] < min[i]) min[i] = box.min()[i];
      if (box.max()[i] > max[i]) max[i] = box.max()[i];
    }
  }

  return {min - float3(0.001f), max + float3(0.001f)};
}

}

#endif //YART_MESH_HPP
