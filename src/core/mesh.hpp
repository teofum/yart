#ifndef YART_MESH_HPP
#define YART_MESH_HPP

#include <ranges>

#include <math/math.hpp>
#include <utility>
#include "bsdf.hpp"
#include "primitives.hpp"
#include "bvh.hpp"

namespace yart {
using namespace math;

class Mesh {
private:
  using BVHType = SahBVH;

  std::vector<Vertex> m_vertices;
  std::vector<Triangle> m_triangles;
  BVHType m_bvh;
  float3 m_emission;

  constexpr std::vector<Triangle> buildTris(const std::vector<Face>& faces) noexcept {
    std::vector<Triangle> tris;
    tris.reserve(faces.size());

    for (const Face& face: faces)
      tris.emplace_back(face, m_vertices);

    return tris;
  }

public:
  enum class FaceCulling : int {
    None = 0, Front, Back
  };

  FaceCulling faceCulling = FaceCulling::None;
  int64_t lightIdx = -1;

  constexpr Mesh(
    const std::vector<Vertex>& vertices,
    const std::vector<Face>& faces,
    const float3& emission = {}
  ) noexcept
    : m_vertices(vertices),
      m_triangles(buildTris(faces)),
      m_bvh(m_triangles),
      m_emission(emission) {}

  constexpr Mesh(
    std::vector<Vertex>&& vertices,
    const std::vector<Face>& faces,
    const float3& emission = {}
  ) noexcept
    : m_vertices(std::move(vertices)),
      m_triangles(buildTris(faces)),
      m_bvh(m_triangles),
      m_emission(emission) {}

  constexpr Mesh(const Mesh& other) noexcept
    : m_vertices(other.m_vertices),
      m_triangles(other.m_triangles),
      m_bvh(other.m_bvh, m_triangles),
      m_emission(other.m_emission) {}

  constexpr Mesh& operator=(const Mesh& other) noexcept {
    m_vertices = other.m_vertices;
    m_triangles = other.m_triangles;
    m_bvh = BVHType(other.m_bvh, other.m_triangles);
    m_emission = other.m_emission;
    return *this;
  }

  constexpr Mesh(Mesh&& other) noexcept
    : m_vertices(std::move(other.m_vertices)),
      m_triangles(std::move(other.m_triangles)),
      m_bvh(std::move(other.m_bvh), m_triangles),
      m_emission(other.m_emission) {}

  constexpr Mesh& operator=(Mesh&& other) noexcept {
    m_vertices = std::move(other.m_vertices);
    m_triangles = std::move(other.m_triangles);
    m_bvh = BVHType(std::move(other.m_bvh), other.m_triangles);
    m_emission = other.m_emission;
    return *this;
  }

  [[nodiscard]] constexpr const BVH& bvh() const noexcept {
    return m_bvh;
  }

  [[nodiscard]] constexpr const std::vector<Triangle>& triangles() const noexcept {
    return m_triangles;
  }

  [[nodiscard]] constexpr const std::vector<Vertex>& vertices() const noexcept {
    return m_vertices;
  }

  [[nodiscard]] constexpr auto vertexPositions() const noexcept {
    return std::views::transform(
      m_vertices, [&](const Vertex& vertex) {
        return vertex.p;
      }
    );
  }

  [[nodiscard]] constexpr const float3* emission() const noexcept {
    return length2(m_emission) == 0.0f ? nullptr : &m_emission;
  }
};

}

#endif //YART_MESH_HPP
