#ifndef YART_BVH_HPP
#define YART_BVH_HPP

#include <core/core.hpp>
#include <math/math.hpp>
#include "primitives.hpp"
#include "utils.hpp"

/**
 * Useful resources:
 * https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
 */

#define MAX_LEAF_SIZE 20

namespace yart {

/**
 * 32-byte BVH node, optimized for minimal memory footprint
 */
struct BVHNode {
  fbounds3 bounds;
  union {
    // If span = 0 node is internal and left is the left child index (right
    // child is always left + 1 and thus not needed)
    // If span > 0 node is a leaf and first is an offset into triangle array
    // These two properties are never needed at the same time so we can use a
    // union and save 4 bytes
    uint32_t left, first = 0;
  };
  // Number of tris in leaf, 0 if node is not a leaf
  uint32_t span = 0;
};

/**
 * Abstract BVH base class. Handles most logic, node splitting for BVH build is
 * handled in derived classes.
 */
class BVH {
public:
  void init(
    const std::vector<float3>* vertices,
    const std::vector<Triangle>* tris,
    const std::vector<float3>* centroids
  ) noexcept {
    m_vertices = vertices;
    m_tris = tris;
    m_centroids = centroids;

    m_indices.resize(tris->size());
    m_nodes.resize(tris->size() * 2 - 1); // Max node count

    m_buildStart = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < m_indices.size(); i++)
      m_indices[i] = i; // Init indices to sequential

    // Init root node
    BVHNode& root = m_nodes[m_rootIdx];
    root.first = 0;
    root.span = m_tris->size();

    // Recursively build BVH
    updateBounds(root);
    subdivide(root);

    printStats();
  }

  [[nodiscard]] constexpr const BVHNode& operator[](size_t i) const {
    return m_nodes[i];
  };

  [[nodiscard]] constexpr const Triangle& tri(size_t i) const {
    return (*m_tris)[m_indices[i]];
  };

  [[nodiscard]] constexpr uint32_t idx(size_t i) const {
    return m_indices[i];
  };

  friend class Mesh;

protected:
  // Pointers to mesh data
  const std::vector<float3>* m_vertices = nullptr;
  const std::vector<Triangle>* m_tris = nullptr;
  const std::vector<float3>* m_centroids = nullptr;

  // Internal BVH data
  std::vector<size_t> m_indices;
  std::vector<BVHNode> m_nodes;
  size_t m_rootIdx = 0, m_nodesUsed = 1;

  // Perf data
  std::chrono::time_point<std::chrono::high_resolution_clock> m_buildStart;

  /**
   * Update a node's bounding box to contain all of its triangles. Assumes the
   * bounding box is empty.
   */
  constexpr void updateBounds(BVHNode& node) {
    size_t first = node.first;
    for (size_t i = 0; i < node.span; i++) {
      const Triangle& leafTri = (*m_tris)[m_indices[first + i]];
      fbounds3 tribounds = fbounds3::fromPoints(
        (*m_vertices)[leafTri.i0],
        (*m_vertices)[leafTri.i1],
        (*m_vertices)[leafTri.i2]
      );
      node.bounds = fbounds3::join(
        node.bounds,
        tribounds
      );
    }
  }

  /**
   * Get bounding box containing the centroids of a group of triangles.
   * @param first Offset into triangles array
   * @param span Number of triangles
   * @return Bounding box
   */
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

  /**
   * Recursively split a BVH node into children. Stops when the heuristic used
   * decides not to split.
   */
  constexpr void subdivide(BVHNode& node) {
    // Get split axis and position, exit (leaf) if no split is necessary
    uint8_t axis = 0;
    float splitPos = 0;
    if (!getSplit(node, axis, splitPos)) return;

    // Sort triangle indices by left/right split
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

    // Check for degenerate split (all tris on left or right child), and return
    // as leaf if this is the case
    size_t leftCount = i - node.first;
    if (leftCount == 0 || leftCount == node.span) return;

    // Init child nodes
    size_t leftIdx = m_nodesUsed++;
    size_t rightIdx = m_nodesUsed++;

    BVHNode& left = m_nodes[leftIdx];
    BVHNode& right = m_nodes[rightIdx];
    left.first = node.first;
    left.span = leftCount;
    right.first = i;
    right.span = node.span - leftCount;

    // Update parent node
    node.left = leftIdx;
    node.span = 0; // Mark as non leaf

    // Recursively split children
    updateBounds(left);
    updateBounds(right);

    subdivide(left);
    subdivide(right);
  }

  /**
   * Print out some BVH build stats, useful for debugging and checking perf
   */
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

    std::cout << "Built in " << buildTime << "\n"
              << "  Total primitives: " << totalCount << "\n"
              << "  Node count: " << m_nodesUsed << "\n"
              << "  Leaf count: " << leafCount << "\n"
              << "  Max leaf size: " << maxSize << "\n"
              << "  Avg leaf size: " << avgSize << "\n"
              << "\n";
  }

  /**
   * Decides whether to split or not, and split axis and position
   * @param node Parent node
   * @param axis (out) Split axis [0-2]
   * @param splitPos (out) Split position
   * @return true if node should split, false if it should remain a leaf
   */
  virtual constexpr bool getSplit(
    const BVHNode& node,
    uint8_t& axis,
    float& splitPos
  ) const noexcept = 0;
};

/**
 * Simple median split BVH: on each split, pick the longest axis of the parent
 * node's bounds and split along the middle. If there are 2 or fewer triangles,
 * leave the node as a leaf.
 * Very fast, but produces a low quality BVH. Shouldn't be used other than as a
 * baseline for comparing performance.
 */
class MedianSplitBVH : public BVH {
private:
  constexpr bool getSplit(
    const BVHNode& node,
    uint8_t& axis,
    float& splitPos
  ) const noexcept override {
    if (node.span <= 2) return false;
    fbounds3 centroidBounds = getCentroidBounds(node.first, node.span);

    // Pick the longest axis from the node's centroid bounds
    // Very imoportant not to use triangle bounds, as it can result in a broken
    // BVH with many degenerate nodes if the mesh has large triangles
    axis = 0;
    float3 size = centroidBounds.size();
    if (size.y() > size.x()) axis = 1;
    if (size.z() > size[axis]) axis = 2;

    splitPos = centroidBounds.min[axis] + size[axis] * 0.5f;
    return true;
  }
};

/**
 * Binned SAH (Surface Area Heuristic) BVH: checks a number of potential splits
 * along each axis and chooses the best one according to the SAH.
 * Slower, but builds a much better BVH resulting in faster renders. Build time
 * vs quality can be tweaked by changing number of bins.
 */
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

    // Number of BVH bins for splitting. More bins is slower to build but gives
    // a better quality BVH, 20 seems to strike a good balance.
    constexpr uint32_t nBins = 20;
    constexpr uint32_t nSplits = nBins - 1;

    // Iterate over each axis
    for (uint8_t a = 0; a < 3; a++) {
      float bmin = centroidBounds.min[a], bsize = centroidBounds.size()[a];

      // Initialize bins
      Bin bins[nBins];
      float scale = float(nBins) / bsize;
      for (uint32_t i = 0; i < node.span; i++) {
        // Add each triangle in the node to a bin depending on where its
        // centroid lies along the relevant axis
        const Triangle& tri = (*m_tris)[m_indices[node.first + i]];
        const float3& centroid = (*m_centroids)[m_indices[node.first + i]];
        auto triBounds = fbounds3::fromPoints(
          (*m_vertices)[tri.i0],
          (*m_vertices)[tri.i1],
          (*m_vertices)[tri.i2]
        );

        auto b = std::min(
          nBins - 1,
          uint32_t(scale * (centroid[a] - bmin))
        );
        bins[b].count++;
        bins[b].bounds = fbounds3::join(bins[b].bounds, triBounds);
      }

      // Compute costs for splitting in two steps: for each possible split,
      // consider the count and bounding area of both resulting children
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

      // Pick the best bin and set its axis and split position
      for (uint32_t i = 0; i < nSplits; i++) {
        if (costs[i] < minCost) {
          minCost = costs[i];
          axis = a;
          splitPos = bmin + bsize * (float(i + 1) / nBins);
        }
      }
    }

    // Don't split if the cost of traversing both children would actually be
    // greater than just testing every triangle in the leaf
    float leafCost = (float(node.span) - 0.5f) * node.bounds.area();
    if (node.span <= MAX_LEAF_SIZE && leafCost < minCost) return false;

    return true;
  }
};

}

#endif //YART_BVH_HPP
