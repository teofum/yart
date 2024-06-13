#ifndef YART_BVH_HPP
#define YART_BVH_HPP

#include <core/core.hpp>
#include <math/math.hpp>
#include "primitives.hpp"
#include "utils.hpp"

namespace yart {

struct BVHNode {
  fbounds3 bounds;
  union {
    uint32_t left, first = 0;
  };
  uint32_t span = 0;
};

class BVH {
public:
  constexpr explicit BVH(const std::vector<Triangle>& tris) noexcept
    : m_tris(&tris), m_triIdx(tris.size()), m_nodes(tris.size() * 2 - 1) {
    m_buildStart = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < m_triIdx.size(); i++) {
      m_triIdx[i] = i;
    }

    BVHNode& root = m_nodes[m_rootIdx];
    root.first = 0;
    root.span = m_tris->size();

    updateBounds(root);
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

protected:
  const std::vector<Triangle>* m_tris;
  std::vector<size_t> m_triIdx;
  std::vector<BVHNode> m_nodes;
  size_t m_rootIdx = 0, m_nodesUsed = 1;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_buildStart;

  constexpr void updateBounds(BVHNode& node) {
    size_t first = node.first;
    for (size_t i = 0; i < node.span; i++) {
      const Triangle& leafTri = (*m_tris)[m_triIdx[first + i]];
      fbounds3 tribounds = fbounds3::fromPoints(
        leafTri.v0.position,
        leafTri.v1.position,
        leafTri.v2.position
      );
      node.bounds = fbounds3::join(
        node.bounds,
        tribounds
      );
    }
  }

  [[nodiscard]] constexpr fbounds3 getCentroidBounds(
    uint32_t first,
    uint32_t span
  ) const noexcept {
    fbounds3 bounds;
    for (size_t i = first; i < first + span; i++) {
      const Triangle& tri = (*m_tris)[m_triIdx[i]];
      bounds.expandToInclude(tri.centroid);
    }

    return bounds;
  }

  constexpr void subdivide(BVHNode& node) {
    uint8_t axis = 0;
    float splitPos = 0;
    if (!getSplit(node, axis, splitPos)) return;

    int64_t i = node.first;
    int64_t j = i + node.span - 1;
    while (i <= j) {
      float centroid = (*m_tris)[m_triIdx[i]].centroid[axis];
      if (centroid < splitPos) {
        i++;
      } else {
        std::iter_swap(m_triIdx.begin() + i, m_triIdx.begin() + (j--));
      }
    }

    size_t leftCount = i - node.first;
    if (leftCount == 0 || leftCount == node.span) return;

    size_t leftIdx = m_nodesUsed++;
    size_t rightIdx = m_nodesUsed++;

    BVHNode& left = m_nodes[leftIdx];
    BVHNode& right = m_nodes[rightIdx];
    left.first = node.first;
    left.span = leftCount;
    right.first = i;
    right.span = node.span - leftCount;

    node.left = leftIdx;
    node.span = 0;

    updateBounds(left);
    updateBounds(right);

    subdivide(left);
    subdivide(right);
  }

  void printStats() const {
    auto buildEnd = std::chrono::high_resolution_clock::now();
    auto buildTime = toMillis(buildEnd - m_buildStart);

    uint32_t maxSize = 0, leafCount = 0, totalCount = 0;
    float avgSize;

    for (size_t i = 0; i < m_nodesUsed; i++) {
      const BVHNode& node = m_nodes[i];
      if (node.span > 0) {
        maxSize = std::max(maxSize, node.span);
        totalCount += node.span;
        leafCount++;
      }
    }

    avgSize = float(totalCount) / float(leafCount);

    std::cout << "Built in " << buildTime << "\n";
    std::cout << "  Total primitives: " << totalCount << "\n";
    std::cout << "  Node count: " << m_nodesUsed << "\n";
    std::cout << "  Leaf count: " << leafCount << "\n";
    std::cout << "  Max leaf size: " << maxSize << "\n";
    std::cout << "  Avg leaf size: " << avgSize << "\n";
    std::cout << "\n";
  }

  virtual constexpr bool getSplit(
    const BVHNode& node,
    uint8_t& axis,
    float& splitPos
  ) const noexcept = 0;
};

class MedianSplitBVH : public BVH {
public:
  constexpr explicit MedianSplitBVH(const std::vector<Triangle>& tris) noexcept
    : BVH(tris) {
    BVHNode& root = m_nodes[m_rootIdx];
    subdivide(root);
    printStats();
  }

  constexpr MedianSplitBVH(
    const BVH& other,
    const std::vector<Triangle>& tris
  ) noexcept: BVH(other, tris) {}

  constexpr MedianSplitBVH(
    BVH&& other,
    const std::vector<Triangle>& tris
  ) noexcept: BVH(std::move(other), tris) {}

private:
  constexpr bool getSplit(
    const BVHNode& node,
    uint8_t& axis,
    float& splitPos
  ) const noexcept override {
    if (node.span <= 2) return false;
    fbounds3 centroidBounds = getCentroidBounds(node.first, node.span);

    axis = 0;
    float3 size = centroidBounds.size();
    if (size.y() > size.x()) axis = 1;
    if (size.z() > size[axis]) axis = 2;

    splitPos = centroidBounds.min[axis] + size[axis] * 0.5f;
    return true;
  }
};

class SahBVH : public BVH {
public:
  constexpr explicit SahBVH(const std::vector<Triangle>& tris) noexcept
    : BVH(tris) {
    BVHNode& root = m_nodes[m_rootIdx];
    subdivide(root);
    printStats();
  }

  constexpr SahBVH(
    const BVH& other,
    const std::vector<Triangle>& tris
  ) noexcept: BVH(other, tris) {}

  constexpr SahBVH(
    BVH&& other,
    const std::vector<Triangle>& tris
  ) noexcept: BVH(std::move(other), tris) {}

private:
  struct Bin {
    uint32_t count = 0;
    fbounds3 bounds;
  };

  constexpr bool getSplit(
    const BVHNode& node,
    uint8_t& axis,
    float& splitPos
  ) const noexcept override {
    float minCost = std::numeric_limits<float>::infinity();
    fbounds3 centroidBounds = getCentroidBounds(node.first, node.span);

    constexpr uint32_t nBins = 20;
    constexpr uint32_t nSplits = nBins - 1;

    for (uint8_t a = 0; a < 3; a++) {
      float bmin = centroidBounds.min[a], bsize = centroidBounds.size()[a];

      // Initialize bins
      Bin bins[nBins];
      float scale = float(nBins) / bsize;
      for (uint32_t i = 0; i < node.span; i++) {
        const Triangle& tri = (*m_tris)[m_triIdx[node.first + i]];
        auto triBounds = fbounds3::fromPoints(
          tri.v0.position,
          tri.v1.position,
          tri.v2.position
        );

        auto b = std::min(
          nBins - 1,
          uint32_t(scale * (tri.centroid[a] - bmin))
        );
        bins[b].count++;
        bins[b].bounds = fbounds3::join(bins[b].bounds, triBounds);
      }

      // Compute costs for splitting
      float costs[nSplits] = {0.0f};
      uint32_t countBelow = 0;
      fbounds3 boundsBelow;
      for (uint32_t i = 0; i < nSplits; i++) {
        boundsBelow = fbounds3::join(boundsBelow, bins[i].bounds);
        countBelow += bins[i].count;
        costs[i] += float(countBelow) * boundsBelow.area();
      }

      uint32_t countAbove = 0;
      fbounds3 boundsAbove;
      for (uint32_t i = nSplits; i > 0; i--) {
        boundsAbove = fbounds3::join(boundsAbove, bins[i].bounds);
        countAbove += bins[i].count;
        costs[i - 1] += float(countAbove) * boundsAbove.area();
      }

      for (uint32_t i = 0; i < nSplits; i++) {
        if (costs[i] < minCost) {
          minCost = costs[i];
          axis = a;
          splitPos = bmin + bsize * (float(i + 1) / nBins);
        }
      }
    }

    float leafCost = (float(node.span) - 0.5f) * node.bounds.area();
    if (leafCost < minCost) return false;

    return true;
  }
};

}

#endif //YART_BVH_HPP
