#include "basic-integrator.hpp"

namespace yart::cpu {

BasicIntegrator::BasicIntegrator(Buffer& buffer, const Camera& camera) noexcept
  : Integrator(buffer, camera) {
  // TODO: take rng seed as parameter
  std::random_device rd;
  m_rng = Xoshiro::Xoshiro256PP(rd());
}

void BasicIntegrator::render(const Node& root) {
  m_rayCounter = 0;

  for (size_t i = 0; i < m_target.width(); i++) {
    for (size_t j = 0; j < m_target.height(); j++) {
      for (uint32_t sample = 0; sample < samples; sample++) {
        auto ray = m_camera
          .getRay({i + samplingOffset.x(), j + samplingOffset.y()}, m_rng);
        float3 color = rayColor(ray, root);

        if (sample == 0) m_target(i, j) = float4(color, 1.0f) / float(samples);
        else m_target(i, j) += float4(color, 1.0f) / float(samples);
      }
    }
  }
}

float3 BasicIntegrator::rayColor(
  const Ray& ray,
  const Node& root,
  uint32_t depth
) {
  if (depth > maxDepth) return {0.0f, 0.0f, 0.0f};
  m_rayCounter++;

  Hit hit;
  bool didHit = testNode(ray, 0.001f, hit, root);
  if (!didHit) return {0.0f, 0.0f, 0.0f}; // TODO background color

  ScatterResult res = scatter(ray, hit);

  if (const Scattered* r = std::get_if<Scattered>(&res)) {
    float3 reflected = rayColor(r->scattered, root, depth + 1) * r->attenuation;
    return reflected + r->emission;
  } else if (const Emitted* e = std::get_if<Emitted>(&res)) {
    return e->emission;
  } else {
    // Absorbed
    return {0.0f, 0.0f, 0.0f};
  }
}

bool BasicIntegrator::testNode(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Node& node
) const {
  const Ray rayObjSpace(
    float3(node.inverseTransform() * float4(ray.origin, 1.0f)),
    float3(node.inverseTransform() * float4(ray.dir(), 0.0f))
  );

  if (!testBoundingBox(rayObjSpace, {tMin, hit.t}, node.boundingBox()))
    return false;

  bool didHit = false;

  const Mesh* mesh;
  if ((mesh = node.mesh())) {
    didHit = testMesh(rayObjSpace, tMin, hit, *mesh);
  }

  for (const Node& child: node.children()) {
    didHit |= testNode(rayObjSpace, tMin, hit, child);
  }

  if (!didHit) return false;

  hit.position = float3(node.transform() * float4(hit.position, 1.0));
  hit.normal = normalized(node.normalTransform() * hit.normal);
  return true;
}

bool BasicIntegrator::testMesh(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Mesh& mesh
) const {
  bool didHit = false;

  for (const Triangle& tri: mesh.triangles()) {
    didHit |= testTriangle(ray, tMin, hit, tri, mesh);
  }

  return didHit;
}

// Möller–Trumbore intersection
bool BasicIntegrator::testTriangle(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Triangle& tri,
  const Mesh& mesh
) const {
  const float3&
    v0 = tri.vertices[0]->position,
    v1 = tri.vertices[1]->position,
    v2 = tri.vertices[2]->position;

  const float3 edge1 = v1 - v0;
  const float3 edge2 = v2 - v0;

  // Cramer's Rule
  const float3 rayEdge2 = cross(ray.dir(), edge2);

  const float det = dot(edge1, rayEdge2);
  // TODO: culling support
  if (std::abs(det) < epsilon) return false;

  const float invDet = 1.0f / det;
  const float3 b = ray.origin - v0; // We're solving for Ax = b

  const float u = dot(b, rayEdge2) * invDet;
  if (u < 0.0f || u > 1.0f) return false;

  // Uses the property a.(b×c) = b.(c×a) = c.(a×b)
  // and a×b = b×(-a)
  const float3 bEdge1 = cross(b, edge1);
  const float v = dot(ray.dir(), bEdge1) * invDet;
  if (v < 0.0f || u + v > 1.0f) return false;

  const float t = dot(edge2, bEdge1) * invDet;
  if (t <= tMin || hit.t <= t) return false;

  hit.t = t;
  hit.position = ray.at(t);

  const float w = 1.0f - u - v;
  hit.normal = w * tri.vertices[0]->normal + u * tri.vertices[1]->normal +
               v * tri.vertices[2]->normal;

  hit.material = &mesh.material;
  return true;
}

// "An Efficient and Robust Ray–Box Intersection Algorithm", Amy Williams
bool BasicIntegrator::testBoundingBox(
  const Ray& ray,
  const interval<float>& tInt,
  const BoundingBox& bounds
) const {
  const float3& invDir = ray.invDir();
  const vec3<uint8>& sign = ray.sign();

  float tMin = (bounds.bounds[sign.x()].x() - ray.origin.x()) * invDir.x();
  float tMax = (bounds.bounds[1 - sign.x()].x() - ray.origin.x()) * invDir.x();

  float tyMin = (bounds.bounds[sign.y()].y() - ray.origin.y()) * invDir.y();
  float tyMax = (bounds.bounds[1 - sign.y()].y() - ray.origin.y()) * invDir.y();

  if (tMin > tyMax || tyMin > tMax) return false;

  if (tyMin > tMin) tMin = tyMin;
  if (tyMax < tMax) tMax = tyMax;

  float tzMin = (bounds.bounds[sign.z()].z() - ray.origin.z()) * invDir.z();
  float tzMax = (bounds.bounds[1 - sign.z()].z() - ray.origin.z()) * invDir.z();

  if (tMin > tzMax || tzMin > tMax) return false;

  if (tzMin > tMin) tMin = tzMin;
  if (tzMax < tMax) tMax = tzMax;

  return tMin < tInt.max && tMax > tInt.min;
}

ScatterResult BasicIntegrator::scatter(const Ray& ray, const Hit& hit) {
  return std::visit(
    [&](const auto& mat) {
      return scatterImpl(mat, ray, hit);
    },
    *(hit.material)
  );
}

ScatterResult BasicIntegrator::scatterImpl(
  const Lambertian& mat,
  const Ray& ray,
  const Hit& hit
) {
  float4x4 basis = normalToTBN(hit.normal);
  float3 scatterDir = float3(
    basis * float4(random::randomCosineVec(m_rng), 0.0f)
  );
  Ray scattered(hit.position, scatterDir);

  return Scattered{
    mat.albedo,
    {0.0f, 0.0f, 0.0f},
    scattered
  };
}

ScatterResult BasicIntegrator::scatterImpl(
  const Emissive& mat,
  const Ray& ray,
  const Hit& hit
) {
  return Emitted{mat.emission};
}

}