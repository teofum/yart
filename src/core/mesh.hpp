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

  std::vector<TrianglePositions> m_triangles;
  std::vector<TriangleData> m_triangleData;
  std::vector<float3> m_centroids;
  std::vector<uint32_t> m_materials;
  std::vector<int32_t> m_lights;

  BVHType m_bvh;

  constexpr void buildTris(
    const std::vector<Vertex>& vertices,
    const std::vector<Face>& faces
  ) noexcept {
    m_triangles.resize(faces.size());
    m_triangleData.resize(faces.size());
    m_centroids.resize(faces.size());
    m_materials.resize(faces.size());
    m_lights.resize(faces.size());

    size_t i = 0;
    for (const Face& face: faces) {
      createTriangle(
        face,
        vertices,
        &m_triangles[i],
        &m_triangleData[i],
        &m_centroids[i],
        &m_materials[i]
      );
      m_lights[i] = -1;
      i++;
    }
  }

public:
  constexpr Mesh(
    const std::vector<Vertex>& vertices,
    const std::vector<Face>& faces
  ) noexcept {
    buildTris(vertices, faces);
    m_bvh.init(&m_triangles, &m_centroids);
  }

  constexpr Mesh(Mesh&& other) noexcept
    : m_triangles(std::move(other.m_triangles)),
      m_triangleData(std::move(other.m_triangleData)),
      m_centroids(std::move(other.m_centroids)),
      m_materials(std::move(other.m_materials)),
      m_lights(std::move(other.m_lights)) {
    m_bvh = std::move(other.m_bvh);
    m_bvh.m_tris = &m_triangles;
    m_bvh.m_centroids = &m_centroids;
  }

  constexpr Mesh& operator=(Mesh&& other) noexcept {
    m_triangles = std::move(other.m_triangles);
    m_triangleData = std::move(other.m_triangleData);
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

  [[nodiscard]] constexpr const TrianglePositions& triangle(uint32_t i) const noexcept {
    return m_triangles[i];
  }

  [[nodiscard]] constexpr const TriangleData& data(uint32_t i) const noexcept {
    return m_triangleData[i];
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

  [[nodiscard]] constexpr const std::vector<TrianglePositions>& triangles() const noexcept {
    return m_triangles;
  }
};

}

#endif //YART_MESH_HPP
