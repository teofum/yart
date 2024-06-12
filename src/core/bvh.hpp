#ifndef YART_BVH_HPP
#define YART_BVH_HPP

#include <core/core.hpp>
#include <math/math.hpp>
#include "primitives.hpp"

namespace yart {

struct BVHNode {
  fbounds3 bounds;
  uint32_t left;
  uint32_t first, span;

  [[nodiscard]] constexpr bool isLeaf() const noexcept { return span > 0; }
};

class BVH {
public:
  constexpr explicit BVH(const std::vector<Triangle>& tris) noexcept
    : m_tris(&tris), m_triIdx(tris.size()), m_nodes(tris.size() * 2 - 1) {
    for (size_t i = 0; i < m_triIdx.size(); i++) {
      m_triIdx[i] = i;
    }

    BVHNode& root = m_nodes[m_rootIdx];
    root.left = 0;
    root.first = 0;
    root.span = m_tris->size();

    updateBounds(root);
    subdivide(root);
  }

  constexpr BVH(const BVH& other, const std::vector<Triangle>& tris) noexcept
    : m_tris(&tris), m_triIdx(other.m_triIdx), m_nodes(other.m_nodes),
      m_rootIdx(other.m_rootIdx), m_nodesUsed(other.m_nodesUsed) {}

  constexpr BVH(BVH&& other, const std::vector<Triangle>& tris) noexcept
    : m_tris(&tris),
      m_triIdx(std::move(other.m_triIdx)),
      m_nodes(std::move(other.m_nodes)),
      m_rootIdx(other.m_rootIdx),
      m_nodesUsed(other.m_nodesUsed) {}

  [[nodiscard]] constexpr const BVHNode& operator[](size_t i) const {
    return m_nodes[i];
  };

  [[nodiscard]] constexpr const Triangle& tri(size_t i) const {
    return (*m_tris)[m_triIdx[i]];
  };

  friend class Mesh;

private:
  const std::vector<Triangle>* m_tris;
  std::vector<size_t> m_triIdx;
  std::vector<BVHNode> m_nodes;
  size_t m_rootIdx = 0, m_nodesUsed = 1;

  constexpr void updateBounds(BVHNode& node) {
    size_t first = node.first;
    for (size_t i = 0; i < node.span; i++) {
      const Triangle& leafTri = (*m_tris)[m_triIdx[first + i]];
      node.bounds = fbounds3::join(
        node.bounds,
        fbounds3::fromPoints(
          leafTri.v0.position,
          leafTri.v1.position,
          leafTri.v2.position
        )
      );
    }
  }

  constexpr void subdivide(BVHNode& node) {
    if (node.span <= 2) return;

    float3 size = node.bounds.size();
    uint8_t axis = 0;
    if (size.y() > size.x()) axis = 1;
    if (size.z() > size[axis]) axis = 2;
    float splitPos = node.bounds.min[axis] + size[axis] * 0.5f;

    int64_t i = node.first, j = i + node.span - 1;
    while (i <= j) {
      if ((*m_tris)[m_triIdx[i]].centroid[axis] < splitPos)
        i++;
      else
        std::iter_swap(m_triIdx.begin() + i, m_triIdx.begin() + (j--));
    }

    size_t leftCount = i - node.first;
    if (leftCount == 0 || leftCount == node.span) return;

    size_t leftIdx = m_nodesUsed++;
    size_t rightIdx = m_nodesUsed++;
    node.left = leftIdx;

    BVHNode& left = m_nodes[leftIdx];
    BVHNode& right = m_nodes[rightIdx];
    left.first = node.first;
    left.span = leftCount;
    right.first = i;
    right.span = node.span - leftCount;
    node.span = 0;

    updateBounds(left);
    updateBounds(right);

    subdivide(left);
    subdivide(right);
  }
};

}

#endif //YART_BVH_HPP
