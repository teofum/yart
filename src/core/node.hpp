#ifndef YART_NODE_HPP
#define YART_NODE_HPP

#include <ranges>

#include "mesh.hpp"

namespace yart {

class Node {
private:
  std::unique_ptr<Mesh> m_mesh = nullptr;
  std::vector<std::unique_ptr<Node>> m_children;

  float4x4 m_transform = float4x4::identity();
  float4x4 m_inverseTransform = float4x4::identity();
  float3x3 m_normalTransform = float3x3::identity();

  BoundingBox m_meshBounds;
  BoundingBox m_bounds;
  BoundingBox m_transformedBounds;

  constexpr void updateTransform() {
    m_normalTransform = transpose(inverse(float3x3(m_transform)).value());

    std::vector<float3> corners{
      {m_bounds.min().x(), m_bounds.min().y(), m_bounds.min().z()},
      {m_bounds.min().x(), m_bounds.min().y(), m_bounds.max().z()},
      {m_bounds.min().x(), m_bounds.max().y(), m_bounds.min().z()},
      {m_bounds.min().x(), m_bounds.max().y(), m_bounds.max().z()},
      {m_bounds.max().x(), m_bounds.min().y(), m_bounds.min().z()},
      {m_bounds.max().x(), m_bounds.min().y(), m_bounds.max().z()},
      {m_bounds.max().x(), m_bounds.max().y(), m_bounds.min().z()},
      {m_bounds.max().x(), m_bounds.max().y(), m_bounds.max().z()}
    };

    for (float3& corner: corners)
      corner = float3(m_transform * float4(corner, 1.0f));

    m_transformedBounds = getBoundingBox(corners);
  }

public:
  constexpr Node() noexcept = default;

  constexpr explicit Node(const Mesh& mesh) noexcept {
    m_meshBounds = getBoundingBox(mesh.vertexPositions());
    m_bounds = m_meshBounds;
    m_transformedBounds = m_bounds;

    m_mesh = std::make_unique<Mesh>(mesh);
  }

  [[nodiscard]] constexpr const Mesh* mesh() const noexcept {
    return m_mesh.get();
  }

  [[nodiscard]] constexpr const BoundingBox& boundingBox() const noexcept {
    return m_bounds;
  }

  [[nodiscard]] constexpr const BoundingBox& meshBoundingBox() const noexcept {
    return m_meshBounds;
  }

  [[nodiscard]] constexpr const float4x4& transform() const noexcept {
    return m_transform;
  }

  [[nodiscard]] constexpr const float4x4& inverseTransform() const noexcept {
    return m_inverseTransform;
  }

  [[nodiscard]] constexpr const float3x3& normalTransform() const noexcept {
    return m_normalTransform;
  }

  constexpr bool setTransform(const float4x4& transform) noexcept {
    if (auto inv = inverse(transform)) {
      m_inverseTransform = inv.value();
      m_transform = transform;
      updateTransform();
      return true;
    }

    return false;
  }

  constexpr void translate(const float3& t) noexcept {
    m_transform = float4x4::translation(t) * m_transform;
    m_inverseTransform = m_inverseTransform * float4x4::translation(-t);
    updateTransform();
  }

  constexpr void rotate(float angle, const float3& axis) noexcept {
    m_transform = float4x4::rotation(angle, axis) * m_transform;
    m_inverseTransform = m_inverseTransform * float4x4::rotation(-angle, axis);
    updateTransform();
  }

  constexpr void scale(const float3& s) noexcept {
    m_transform = float4x4::scaling(s) * m_transform;
    m_inverseTransform = m_inverseTransform * float4x4::scaling(1.0f / s);
    updateTransform();
  }

  constexpr void scale(float s) noexcept {
    m_transform = float4x4::scaling(s) * m_transform;
    m_inverseTransform = m_inverseTransform * float4x4::scaling(1.0f / s);
    updateTransform();
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

    m_bounds = combineBoundingBoxes(m_bounds, child.m_transformedBounds);
    updateTransform();
  }
};

}

#endif //YART_NODE_HPP
