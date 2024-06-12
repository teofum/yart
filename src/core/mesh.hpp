#ifndef YART_MESH_HPP
#define YART_MESH_HPP

#include <ranges>

#include <math/math.hpp>
#include "materials.hpp"

namespace yart {
using namespace math;

struct Vertex {
  float3 position, normal;
  float2 textureCoords;
};

struct Face {
  size_t indices[3];
};

struct Triangle {
  const Vertex* vertices[3];
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
  Material material;

  constexpr Mesh(
    const std::vector<Vertex>& vertices,
    const std::vector<Face>& triangles,
    const Material& mat
  ) noexcept
    : m_vertices(vertices), m_triangles(triangles), material(mat) {
  }

  constexpr Mesh(
    std::vector<Vertex>&& vertices,
    std::vector<Face>&& triangles,
    const Material& mat
  ) noexcept
    : m_vertices(std::move(vertices)), m_triangles(std::move(triangles)),
      material(mat) {
  }

  [[nodiscard]] constexpr auto triangles() const noexcept {
    return std::views::transform(
      m_triangles, [&](const Face& face) {
        return Triangle{
          &m_vertices[face.indices[0]],
          &m_vertices[face.indices[1]],
          &m_vertices[face.indices[2]],
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

}

#endif //YART_MESH_HPP
