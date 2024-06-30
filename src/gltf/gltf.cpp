#include "gltf.hpp"
#include <bsdf/parametric.hpp>

namespace yart::gltf {

static float4x4 rotationFromQuat(std::array<float, 4> q) noexcept {
  float qr = q[3], qi = q[0], qj = q[1], qk = q[2];

  float4x4 half{
    0.5f - (qj * qj + qk * qk), (qi * qj - qr * qk), (qi * qk + qr * qj), 0.0f,
    (qi * qj + qr * qk), 0.5f - (qi * qi + qk * qk), (qj * qk - qr * qi), 0.0f,
    (qi * qk - qr * qj), (qj * qk + qr * qi), 0.5f - (qi * qi + qj * qj), 0.0f,
    0.0f, 0.0f, 0.0f, 0.5f
  };

  return half * 2.0f;
}

static std::unique_ptr<Texture> loadTexture(
  const fastgltf::Asset& asset,
  size_t index,
  Texture::Type type
) {
  const fastgltf::Texture& gltfTex = asset.textures[index];
  if (!gltfTex.imageIndex) return nullptr;

  uint32_t imgIdx = gltfTex.imageIndex.value();
  const auto& image = asset.images[imgIdx];

  const auto* bvi = std::get_if<fastgltf::sources::BufferView>(&image.data);
  if (!bvi) return nullptr;

  const auto& bv = asset.bufferViews[bvi->bufferViewIndex];
  const auto& buf = asset.buffers[bv.bufferIndex];

  const auto* bytes = std::get_if<fastgltf::sources::ByteView>(&buf.data);
  if (!bytes) return nullptr;

  // Shut up, C++
  const uint8_t* data =
    reinterpret_cast<const uint8_t*>(&bytes->bytes[bv.byteOffset]);
  int32_t len = int32_t(bv.byteLength);

  return std::make_unique<Texture>(Texture::load(data, len, type));
}

static std::unique_ptr<BSDF> processMaterial(
  const fastgltf::Asset& asset,
  const fastgltf::Material& gltfMat,
  Scene& scene
) noexcept {
  // Base color
  const auto base = gltfMat.pbrData.baseColorFactor;
  const float3 baseColor = float3(base[0], base[1], base[2]);
  const Texture* baseTexture = nullptr;
  if (gltfMat.pbrData.baseColorTexture) {
    uint32_t idx = gltfMat.pbrData.baseColorTexture.value().textureIndex;
    baseTexture = scene.addTexture(
      loadTexture(asset, idx, Texture::Type::sRGB)
    );
  }

  // Roughness + metallic
  float roughness = gltfMat.pbrData.roughnessFactor;
  float metallic = gltfMat.pbrData.metallicFactor;
  const Texture* mrTexture = nullptr;
  if (gltfMat.pbrData.metallicRoughnessTexture) {
    uint32_t idx = gltfMat.pbrData.metallicRoughnessTexture.value()
                          .textureIndex;
    mrTexture = scene.addTexture(
      loadTexture(asset, idx, Texture::Type::NonColor)
    );
  }

  // Transmission
  float transmission = 0.0f;
  const Texture* transmissionTexture = nullptr;
  if (gltfMat.transmission) {
    transmission = gltfMat.transmission->transmissionFactor;
    if (gltfMat.transmission->transmissionTexture) {
      uint32_t idx = gltfMat.transmission->transmissionTexture.value()
                            .textureIndex;
      transmissionTexture = scene.addTexture(
        loadTexture(asset, idx, Texture::Type::NonColor)
      );
    }
  }

  // Anisotropy
  float anisotropic = 0.0f, anisoRotation = 0.0f;
  if (gltfMat.anisotropy) {
    anisotropic = gltfMat.anisotropy->anisotropyStrength;
    anisoRotation = gltfMat.anisotropy->anisotropyRotation;
  }

  // Emission
  const auto em = gltfMat.emissiveFactor;
  const float3 emission =
    float3(em[0], em[1], em[2]) * gltfMat.emissiveStrength;

  // Normal maps
  Texture* normalTexture = nullptr;
  float normalScale = 1.0f;
  if (gltfMat.normalTexture) {
    uint32_t idx = gltfMat.normalTexture.value().textureIndex;
    normalTexture = scene.addTexture(
      loadTexture(asset, idx, Texture::Type::NonColor)
    );

    normalScale = gltfMat.normalTexture.value().scale;
  }

  ParametricBSDF bsdf(
    baseColor,
    baseTexture,
    mrTexture,
    transmissionTexture,
    normalTexture,
    metallic,
    roughness,
    transmission,
    gltfMat.ior,
    anisotropic,
    anisoRotation,
    emission,
    normalScale
  );

  return std::make_unique<ParametricBSDF>(std::move(bsdf));
}

static std::unique_ptr<Mesh> processMesh(
  const fastgltf::Asset& asset,
  const fastgltf::Mesh& gltfMesh,
  Scene& scene
) noexcept {
  const auto primitives = gltfMesh.primitives;
  std::vector<Vertex> meshVertices;
  std::vector<Face> meshFaces;
  float3 emission;

  for (const auto& primitive: primitives) {
    const size_t idxOffset = meshVertices.size();
    const uint32_t materialIdx = primitive.materialIndex.value_or(0);

    const BSDF& material = scene.material(materialIdx);
    if (material.emission() && length2(emission) == 0.0f) {
      emission = *material.emission();
    }

    if (primitive.type != fastgltf::PrimitiveType::Triangles) continue;

    std::vector<Vertex> vertices;
    std::vector<size_t> indices;

    const auto* positionIt = primitive.findAttribute("POSITION");
    const auto& posAccesor = asset.accessors[positionIt->second];
    const auto& posIt = fastgltf::iterateAccessor<float3>(asset, posAccesor);
    vertices.reserve(posAccesor.count);
    for (const float3& pos: posIt)
      vertices.push_back(Vertex{pos, float3(), float4(), float2()});

    const auto* normalIt = primitive.findAttribute("NORMAL");
    const auto& normAccessor = asset.accessors[normalIt->second];
    const auto& normIt = fastgltf::iterateAccessor<float3>(asset, normAccessor);
    size_t nIdx = 0;
    for (const float3& norm: normIt) vertices[nIdx++].normal = norm;

    const auto* texIt = primitive.findAttribute("TEXCOORD_0");
    const auto& tcAccessor = asset.accessors[texIt->second];
    const auto& tcIt = fastgltf::iterateAccessor<float2>(asset, tcAccessor);
    size_t tcIdx = 0;
    for (const float2& tc: tcIt)
      vertices[tcIdx++].texCoords = tc;

    const auto* tangentIt = primitive.findAttribute("TANGENT");
    if (tangentIt) {
      const auto& tanAccessor = asset.accessors[tangentIt->second];
      const auto& tanIt = fastgltf::iterateAccessor<float4>(asset, tanAccessor);
      size_t tIdx = 0;
      for (const float4& tan: tanIt) vertices[tIdx++].tangent = tan;
    }

    meshVertices.reserve(meshVertices.size() + vertices.size());
    meshVertices.insert(meshVertices.end(), vertices.begin(), vertices.end());

    const size_t indexAccessorIdx = primitive.indicesAccessor.value();
    const auto& idxAccesor = asset.accessors[indexAccessorIdx];
    const auto& idxIt = fastgltf::iterateAccessor<uint32>(asset, idxAccesor);
    indices.reserve(idxAccesor.count);
    for (const uint32& idx: idxIt) indices.push_back(idx + idxOffset);

    std::vector<Face> faces;
    faces.reserve(indices.size() / 3);
    for (size_t i = 0; i < indices.size(); i += 3) {
      faces.push_back(
        {
          indices[i + 0],
          indices[i + 1],
          indices[i + 2],
          materialIdx
        }
      );
    }

    meshFaces.reserve(meshFaces.size() + faces.size());
    meshFaces.insert(meshFaces.end(), faces.begin(), faces.end());
  }

  return std::make_unique<Mesh>(Mesh(meshVertices, meshFaces));
}

static Node processNode(
  const fastgltf::Asset& asset,
  size_t nodeIdx,
  Scene& scene,
  const Transform& globalTransform
) noexcept {
  const auto gltfNode = asset.nodes[nodeIdx];

  const auto meshIdx = gltfNode.meshIndex;
  Node node = meshIdx ? Node(&scene.mesh(meshIdx.value())) : Node();

  auto trs = std::get_if<fastgltf::TRS>(&gltfNode.transform);
  if (trs) {
    const float4x4 transform =
      float4x4::translation(float3(trs->translation)) *
      rotationFromQuat(trs->rotation) *
      float4x4::scaling(float3(trs->scale));

    node.transform = Transform(transform);
  }

  Transform localTransform = node.transform * globalTransform;

  for (size_t childIdx: gltfNode.children) {
    node.appendChild(processNode(asset, childIdx, scene, localTransform));
  }

  if (node.mesh()) {
    uint32_t i = 0;
    int32_t li = 0;
    for (const auto& tri: node.mesh()->triangles()) {
      const BSDF& mat = scene.material(node.mesh()->material(i));
      const float3* emission = mat.emission();

      if (emission) {
        AreaLight light(&tri, &node.mesh()->data(i), *emission, localTransform);
        scene.addLight(std::move(light));
        node.mesh()->lightIdx(i) = li++;
      }

      i++;
    }
  }

  return node;
}

std::unique_ptr<Scene> load(const fs::path& path) noexcept {
  auto buffer = fastgltf::GltfDataBuffer();
  buffer.loadFromFile(path);

  auto parser = fastgltf::Parser(
    fastgltf::Extensions::KHR_materials_emissive_strength |
    fastgltf::Extensions::KHR_materials_transmission |
    fastgltf::Extensions::KHR_materials_ior |
    fastgltf::Extensions::KHR_materials_anisotropy
  );
  auto result = parser.loadGltf(
    &buffer,
    path.parent_path(),
    fastgltf::Options::GenerateMeshIndices |
    fastgltf::Options::DecomposeNodeMatrices |
    fastgltf::Options::LoadExternalBuffers
  );

  if (result.error() != fastgltf::Error::None) return nullptr;
  const fastgltf::Asset& asset = result.get();

  size_t sceneIdx = asset.defaultScene.value_or(0);
  auto gltfScene = asset.scenes[sceneIdx];

  Node root;
  Scene scene(std::move(root));

  for (const auto& material: asset.materials)
    scene.addMaterial(processMaterial(asset, material, scene));
  
  for (const auto& mesh: asset.meshes)
    scene.addMesh(processMesh(asset, mesh, scene));

  for (size_t nodeIdx: gltfScene.nodeIndices)
    scene.root().appendChild(processNode(asset, nodeIdx, scene, Transform()));

  return std::make_unique<Scene>(std::move(scene));
}

}

