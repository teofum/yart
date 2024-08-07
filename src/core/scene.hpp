#ifndef YART_SCENE_HPP
#define YART_SCENE_HPP

#include <ranges>

#include "mesh.hpp"
#include "light.hpp"

namespace yart {

class Node {
public:
  Transform transform;

  constexpr Node() noexcept = default;

  constexpr explicit Node(Mesh* mesh) noexcept: m_mesh(mesh) {
    for (const float3& v: mesh->vertices()) {
      m_meshBounds.expandToInclude(v);
    }
    m_bounds = m_meshBounds;
  }

  [[nodiscard]] constexpr const Mesh* mesh() const noexcept {
    return m_mesh;
  }

  [[nodiscard]] constexpr Mesh* mesh() noexcept {
    return m_mesh;
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
  Mesh* m_mesh = nullptr;
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

  [[nodiscard]] constexpr Node& root() noexcept {
    return *m_root;
  }

  [[nodiscard]] constexpr const Mesh& mesh(size_t i) const noexcept {
    return *(m_meshes[i]);
  }

  [[nodiscard]] constexpr Mesh& mesh(size_t i) noexcept {
    return *(m_meshes[i]);
  }

  constexpr void addMesh(std::unique_ptr<Mesh>&& meshPtr) noexcept {
    m_meshes.push_back(std::move(meshPtr));
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

  constexpr MonoTexture* addTexture(std::unique_ptr<MonoTexture>&& tPtr) noexcept {
    uint32_t i = m_texturesMono.size();
    m_texturesMono.push_back(std::move(tPtr));

    return m_texturesMono[i].get();
  }

  constexpr SDRTexture<2>* addTexture(std::unique_ptr<SDRTexture<2>>&& tPtr) noexcept {
    uint32_t i = m_textures2Ch.size();
    m_textures2Ch.push_back(std::move(tPtr));

    return m_textures2Ch[i].get();
  }

  constexpr RGBTexture* addTexture(std::unique_ptr<RGBTexture>&& tPtr) noexcept {
    uint32_t i = m_texturesRGB.size();
    m_texturesRGB.push_back(std::move(tPtr));

    return m_texturesRGB[i].get();
  }

  constexpr RGBATexture* addTexture(std::unique_ptr<RGBATexture>&& tPtr) noexcept {
    uint32_t i = m_texturesRGBA.size();
    m_texturesRGBA.push_back(std::move(tPtr));

    return m_texturesRGBA[i].get();
  }

  [[nodiscard]] constexpr size_t nLights() const noexcept {
    return m_lights.size();
  }

  [[nodiscard]] constexpr const Light& light(size_t i) const noexcept {
    return *(m_lights[i]);
  }

  [[nodiscard]] constexpr auto lights() const noexcept {
    return std::views::transform(
      m_lights, [&](const std::unique_ptr<Light>& lightPtr) -> const Light& {
        return *lightPtr;
      }
    );
  }

  template<typename T>
  requires std::derived_from<T, Light>
  constexpr void addLight(T&& light) noexcept {
    m_lights.push_back(std::make_unique<T>(std::move(light)));
  }

  constexpr void addLight(std::unique_ptr<Light>&& lightPtr) noexcept {
    m_lights.push_back(std::move(lightPtr));
  }

private:
  std::unique_ptr<Node> m_root = nullptr;
  std::vector<std::unique_ptr<Mesh>> m_meshes;
  std::vector<std::unique_ptr<BSDF>> m_materials;
  std::vector<std::unique_ptr<Light>> m_lights;

  std::vector<std::unique_ptr<MonoTexture>> m_texturesMono;
  std::vector<std::unique_ptr<SDRTexture<2>>> m_textures2Ch;
  std::vector<std::unique_ptr<RGBTexture>> m_texturesRGB;
  std::vector<std::unique_ptr<RGBATexture>> m_texturesRGBA;
};

}

#endif //YART_SCENE_HPP
