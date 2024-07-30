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

  std::vector<float3> m_vertices;
  std::vector<VertexData> m_vertexData;

  std::vector<Triangle> m_triangles;
  std::vector<float3> m_centroids;
  std::vector<uint32_t> m_materials;
  std::vector<int32_t> m_lights;

  BVHType m_bvh;

  // Fill in triangle data from the I/O face structs
  constexpr void buildTris(
    const std::vector<float3>& vertices,
    const std::vector<Face>& faces
  ) noexcept {
    m_triangles.resize(faces.size());
    m_centroids.resize(faces.size());
    m_materials.resize(faces.size());
    m_lights.resize(faces.size());

    size_t i = 0;
    for (const Face& face: faces) {
      createTriangle(
        face,
        vertices,
        &m_triangles[i],
        &m_centroids[i],
        &m_materials[i]
      );
      m_lights[i] = -1;
      i++;
    }
  }

public:
  constexpr Mesh(
    const std::vector<float3>& vertices,
    const std::vector<VertexData>& vertexData,
    const std::vector<Face>& faces
  ) noexcept: m_vertices(vertices), m_vertexData(vertexData) {
    buildTris(vertices, faces);
    m_bvh.init(&m_vertices, &m_triangles, &m_centroids);
  }

  constexpr Mesh(Mesh&& other) noexcept
    : m_vertices(std::move(other.m_vertices)),
      m_vertexData(std::move(other.m_vertexData)),
      m_triangles(std::move(other.m_triangles)),
      m_centroids(std::move(other.m_centroids)),
      m_materials(std::move(other.m_materials)),
      m_lights(std::move(other.m_lights)) {
    m_bvh = std::move(other.m_bvh);
    m_bvh.m_tris = &m_triangles;
    m_bvh.m_centroids = &m_centroids;
  }

  constexpr Mesh& operator=(Mesh&& other) noexcept {
    m_vertices = std::move(other.m_vertices);
    m_vertexData = std::move(other.m_vertexData);
    m_triangles = std::move(other.m_triangles);
    m_centroids = std::move(other.m_centroids);
    m_materials = std::move(other.m_materials);
    m_lights = std::move(other.m_lights);

    m_bvh = std::move(other.m_bvh);
    m_bvh.m_tris = &m_triangles;
    m_bvh.m_centroids = &m_centroids;
    return *this;
  }

  [[nodiscard]] constexpr const BVH& bvh() const noexcept {
    return m_bvh;
  }

  [[nodiscard]] constexpr const std::vector<float3>& vertices() const noexcept {
    return m_vertices;
  }

  [[nodiscard]] constexpr const float3& vertex(uint32_t i) const noexcept {
    return m_vertices[i];
  }

  [[nodiscard]] constexpr const std::vector<Triangle>& triangles() const noexcept {
    return m_triangles;
  }

  [[nodiscard]] constexpr const Triangle& triangle(uint32_t i) const noexcept {
    return m_triangles[i];
  }

  [[nodiscard]] constexpr const std::vector<VertexData>& vertexData() const noexcept {
    return m_vertexData;
  }

  [[nodiscard]] constexpr const VertexData& vertexData(uint32_t i) const noexcept {
    return m_vertexData[i];
  }

  [[nodiscard]] constexpr uint32_t material(uint32_t i) const noexcept {
    return m_materials[i];
  }

  [[nodiscard]] constexpr int32_t& lightIdx(uint32_t i) noexcept {
    return m_lights[i];
  }

  [[nodiscard]] constexpr int32_t lightIdx(uint32_t i) const noexcept {
    return m_lights[i];
  }
};

}

#endif //YART_MESH_HPP
