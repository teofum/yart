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

  BVHType m_bvh;
  float3 m_emission;

  constexpr void buildTris(
    const std::vector<Vertex>& vertices,
    const std::vector<Face>& faces
  ) noexcept {
    m_triangles.resize(faces.size());
    m_triangleData.resize(faces.size());
    m_centroids.resize(faces.size());
    m_materials.resize(faces.size());

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
      i++;
    }
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
  ) noexcept: m_emission(emission) {
    buildTris(vertices, faces);
    m_bvh.init(&m_triangles, &m_centroids);
  }

  constexpr Mesh(const Mesh& other) noexcept
    : m_triangles(other.m_triangles),
      m_triangleData(other.m_triangleData),
      m_centroids(other.m_centroids),
      m_materials(other.m_materials),
      m_emission(other.m_emission) {
    m_bvh.init(&m_triangles, &m_centroids);
  }

  constexpr Mesh& operator=(const Mesh& other) noexcept {
    m_triangles = other.m_triangles;
    m_triangleData = other.m_triangleData;
    m_centroids = other.m_centroids;
    m_materials = other.m_materials;
    m_emission = other.m_emission;

    m_bvh.init(&m_triangles, &m_centroids);
    return *this;
  }

  constexpr Mesh(Mesh&& other) noexcept
    : m_triangles(std::move(other.m_triangles)),
      m_triangleData(std::move(other.m_triangleData)),
      m_centroids(std::move(other.m_centroids)),
      m_materials(std::move(other.m_materials)),
      m_emission(other.m_emission) {
    m_bvh = std::move(other.m_bvh);
    m_bvh.m_tris = &m_triangles;
    m_bvh.m_centroids = &m_centroids;
  }

  constexpr Mesh& operator=(Mesh&& other) noexcept {
    m_triangles = std::move(other.m_triangles);
    m_triangleData = std::move(other.m_triangleData);
    m_centroids = std::move(other.m_centroids);
    m_materials = std::move(other.m_materials);
    m_emission = other.m_emission;

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

  [[nodiscard]] constexpr const std::vector<TrianglePositions>& triangles() const noexcept {
    return m_triangles;
  }

  [[nodiscard]] constexpr const float3* emission() const noexcept {
    return length2(m_emission) == 0.0f ? nullptr : &m_emission;
  }
};

}

#endif //YART_MESH_HPP
