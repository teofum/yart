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
  void init(
    const std::vector<TrianglePositions>* tris,
    const std::vector<float3>* centroids
  ) noexcept {
    m_tris = tris;
    m_centroids = centroids;

    m_indices.resize(tris->size());
    m_nodes.resize(tris->size() * 2 - 1);

    m_buildStart = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < m_indices.size(); i++) {
      m_indices[i] = i;
    }

    BVHNode& root = m_nodes[m_rootIdx];
    root.first = 0;
    root.span = m_tris->size();

    updateBounds(root);
    subdivide(root);
    printStats();
  }

//  constexpr BVH(const BVH& other, const std::vector<Triangle>& tris) noexcept
//    : m_tris(&tris), m_triIdx(other.m_triIdx), m_nodes(other.m_nodes),
//      m_rootIdx(other.m_rootIdx), m_nodesUsed(other.m_nodesUsed) {}
//
//  constexpr BVH(BVH&& other, const std::vector<Triangle>& tris) noexcept
//    : m_tris(&tris),
//      m_triIdx(std::move(other.m_triIdx)),
//      m_nodes(std::move(other.m_nodes)),
//      m_rootIdx(other.m_rootIdx),
//      m_nodesUsed(other.m_nodesUsed) {}

  [[nodiscard]] constexpr const BVHNode& operator[](size_t i) const {
    return m_nodes[i];
  };

  [[nodiscard]] constexpr const TrianglePositions& tri(size_t i) const {
    return (*m_tris)[m_indices[i]];
  };

  [[nodiscard]] constexpr uint32_t idx(size_t i) const {
    return m_indices[i];
  };

  friend class Mesh;

protected:
  // Pointers to mesh data
  const std::vector<TrianglePositions>* m_tris = nullptr;
  const std::vector<float3>* m_centroids = nullptr;

  // Internal BVH data
  std::vector<size_t> m_indices;
  std::vector<BVHNode> m_nodes;
  size_t m_rootIdx = 0, m_nodesUsed = 1;

  // Perf data
  std::chrono::time_point<std::chrono::high_resolution_clock> m_buildStart;

  constexpr void updateBounds(BVHNode& node) {
    size_t first = node.first;
    for (size_t i = 0; i < node.span; i++) {
      const TrianglePositions& leafTri = (*m_tris)[m_indices[first + i]];
      fbounds3 tribounds = fbounds3::fromPoints(
        leafTri.p0,
        leafTri.p1,
        leafTri.p2
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
      const float3& centroid = (*m_centroids)[m_indices[i]];
      bounds.expandToInclude(centroid);
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
      float centroid = (*m_centroids)[m_indices[i]][axis];
      if (centroid < splitPos) {
        i++;
      } else {
        std::iter_swap(m_indices.begin() + i, m_indices.begin() + (j--));
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
        const TrianglePositions& tri = (*m_tris)[m_indices[node.first + i]];
        const float3& centroid = (*m_centroids)[m_indices[node.first + i]];
        auto triBounds = fbounds3::fromPoints(
          tri.p0,
          tri.p1,
          tri.p2
        );

        auto b = std::min(
          nBins - 1,
          uint32_t(scale * (centroid[a] - bmin))
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
