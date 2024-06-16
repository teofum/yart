#ifndef YART_SCENE_HPP
#define YART_SCENE_HPP

#include <ranges>

#include "mesh.hpp"

namespace yart {

class Node {
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

private:
  std::unique_ptr<Mesh> m_mesh = nullptr;
  std::vector<std::unique_ptr<Node>> m_children;

  fbounds3 m_meshBounds;
  fbounds3 m_bounds;

  [[nodiscard]] constexpr fbounds3 transformedBounds() const noexcept {
    return transform(m_bounds);
  }
};

class Scene {
public:
  constexpr explicit Scene(Node&& root) noexcept
    : m_root(std::make_unique<Node>(std::move(root))) {}

  [[nodiscard]] constexpr const Node& root() const noexcept {
    return *m_root;
  }

  [[nodiscard]] constexpr const BSDF& material(size_t i) const noexcept {
    return *(m_materials[i]);
  }

  template<typename T>
  requires std::derived_from<T, BSDF>
  constexpr void addMaterial(T&& material) noexcept {
    m_materials.push_back(std::make_unique<T>(std::forward(material)));
  }

  constexpr void addMaterial(std::unique_ptr<BSDF>&& matPtr) noexcept {
    m_materials.push_back(std::move(matPtr));
  }

private:
  std::unique_ptr<Node> m_root = nullptr;
  std::vector<std::unique_ptr<BSDF>> m_materials;
};

}

#endif //YART_SCENE_HPP
