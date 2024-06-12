#ifndef YART_NODE_HPP
#define YART_NODE_HPP

#include <ranges>

#include "mesh.hpp"

namespace yart {

class Node {
private:
  std::unique_ptr<Mesh> m_mesh = nullptr;
  std::vector<std::unique_ptr<Node>> m_children;

  fbounds3 m_meshBounds;
  fbounds3 m_bounds;

  [[nodiscard]] constexpr fbounds3 transformedBounds() const noexcept {
    return transform(m_bounds);
  }

public:
  Transform transform;

  constexpr Node() noexcept = default;

  constexpr explicit Node(const Mesh& mesh) noexcept {
    m_meshBounds = fbounds3::fromPoints(mesh.vertexPositions());
    m_bounds = m_meshBounds;

    m_mesh = std::make_unique<Mesh>(mesh);
  }

  [[nodiscard]] constexpr const Mesh* mesh() const noexcept {
    return m_mesh.get();
  }

  [[nodiscard]] constexpr const fbounds3& boundingBox() const noexcept {
    return m_bounds;
  }

  [[nodiscard]] constexpr const fbounds3& meshBoundingBox() const noexcept {
    return m_meshBounds;
  }

  [[nodiscard]] constexpr auto children() const noexcept {
    return std::views::transform(
      m_children, [](const std::unique_ptr<Node>& child) -> const Node& {
        return *child;
      }
    );
  }

  constexpr void appendChild(Node&& child) noexcept {
    m_children.push_back(std::make_unique<Node>(std::move(child)));

    m_bounds = fbounds3::join(m_bounds, child.transformedBounds());
  }
};

}

#endif //YART_NODE_HPP
