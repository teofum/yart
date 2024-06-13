#include "gltf.hpp"

namespace yart::gltf {

Material defaultMaterial(Lambertian{float3(0.8f)});

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

static Material processMaterial(
  const fastgltf::Asset& asset,
  size_t materialIdx
) noexcept {
  const auto& gltfMat = asset.materials[materialIdx];

  const auto em = gltfMat.emissiveFactor;
  const float3 emission =
    float3(em[0], em[1], em[2]) * gltfMat.emissiveStrength * 10.0f;
  if (length(emission) > 0.0f) return Material(Emissive{emission});

  const auto base = gltfMat.pbrData.baseColorFactor;
  return Material(Lambertian{float3(base[0], base[1], base[2])});
}

static Mesh processMesh(const fastgltf::Asset& asset, size_t meshIdx) noexcept {
  const auto gltfMesh = asset.meshes[meshIdx];

  const auto primitives = gltfMesh.primitives;
  const auto materialIdx = primitives[0].materialIndex;

  Material material = materialIdx ? processMaterial(asset, materialIdx.value())
                                  : defaultMaterial;

  std::vector<Vertex> meshVertices;
  std::vector<size_t> meshIndices;
  for (const auto& primitive: primitives) {
    if (primitive.type != fastgltf::PrimitiveType::Triangles) continue;

    std::vector<Vertex> vertices;
    std::vector<size_t> indices;

    const auto* positionIt = primitive.findAttribute("POSITION");
    const auto& posAccesor = asset.accessors[positionIt->second];
    const auto& posIt = fastgltf::iterateAccessor<float3>(asset, posAccesor);
    vertices.reserve(posAccesor.count);
    for (const float3& pos: posIt)
      vertices.push_back(Vertex{pos, float3(), float2()});

    const auto* normalIt = primitive.findAttribute("NORMAL");
    const auto& normAccessor = asset.accessors[normalIt->second];
    const auto& normIt = fastgltf::iterateAccessor<float3>(asset, normAccessor);
    size_t nIdx = 0;
    for (const float3& norm: normIt) vertices[nIdx++].normal = norm;

    meshVertices.reserve(meshVertices.size() + vertices.size());
    meshVertices.insert(meshVertices.end(), vertices.begin(), vertices.end());

    const size_t indexAccessorIdx = primitive.indicesAccessor.value();
    const auto& idxAccesor = asset.accessors[indexAccessorIdx];
    const auto& idxIt = fastgltf::iterateAccessor<uint32>(asset, idxAccesor);
    indices.reserve(idxAccesor.count);
    for (const uint32& idx: idxIt) indices.push_back(idx);

    meshIndices.reserve(meshIndices.size() + indices.size());
    meshIndices.insert(meshIndices.end(), indices.begin(), indices.end());
  }

  std::vector<Face> faces;
  faces.reserve(meshIndices.size() / 3);
  for (size_t i = 0; i < meshIndices.size(); i += 3) {
    faces.push_back(
      {
        meshIndices[i + 0],
        meshIndices[i + 1],
        meshIndices[i + 2]
      }
    );
  }

  return {std::move(meshVertices), faces, material};
}

static Node processNode(const fastgltf::Asset& asset, size_t nodeIdx) noexcept {
  const auto gltfNode = asset.nodes[nodeIdx];

  const auto meshIdx = gltfNode.meshIndex;
  Node node = meshIdx ? Node(processMesh(asset, meshIdx.value())) : Node();

  auto trs = std::get_if<fastgltf::TRS>(&gltfNode.transform);
  if (trs) {
    const float4x4 transform =
      float4x4::translation(float3(trs->translation)) *
      rotationFromQuat(trs->rotation) *
      float4x4::scaling(float3(trs->scale));

    node.transform = Transform(transform);
  }

  for (size_t childIdx: gltfNode.children) {
    node.appendChild(processNode(asset, childIdx));
  }

  return node;
}

std::optional<Node> load(const fs::path& path) noexcept {
  auto buffer = fastgltf::GltfDataBuffer();
  buffer.loadFromFile(path);

  auto parser = fastgltf::Parser();
  auto result = parser.loadGltf(
    &buffer,
    path.parent_path(),
    fastgltf::Options::GenerateMeshIndices |
    fastgltf::Options::DecomposeNodeMatrices |
    fastgltf::Options::LoadExternalBuffers
  );

  if (result.error() != fastgltf::Error::None) return std::nullopt;
  const fastgltf::Asset& asset = result.get();

  size_t sceneIdx = asset.defaultScene.value_or(0);
  auto scene = asset.scenes[sceneIdx];

  Node root;
  for (size_t nodeIdx: scene.nodeIndices) {
    root.appendChild(processNode(asset, nodeIdx));
  }

  return root;
}

}

