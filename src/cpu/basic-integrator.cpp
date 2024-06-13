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
  const Ray rayObjSpace = node.transform.inverse(ray);

  if (isinf(testBoundingBox(rayObjSpace, {tMin, hit.t}, node.boundingBox())))
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

  hit.position = node.transform(hit.position, Transform::Type::Point);
  hit.normal = node.transform(hit.normal, Transform::Type::Normal);
  return true;
}

bool BasicIntegrator::testMesh(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Mesh& mesh
) const {
  bool didHit = testBVH(ray, tMin, hit, mesh.bvh());
  if (didHit) hit.material = &mesh.material;

  return didHit;
}

bool BasicIntegrator::testBVH(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const BVH& bvh
) const {
  const BVHNode* node = &bvh[0];
  const BVHNode* stack[64];
  uint32_t stackIdx = 0;
  bool didHit = false;

  while (true) {
    if (node->span > 0) {
      for (size_t i = 0; i < node->span; i++) {
        didHit |= testTriangle(ray, tMin, hit, bvh.tri(node->first + i));
      }
      if (stackIdx == 0) break;
      node = stack[--stackIdx];
    } else {
      const BVHNode* child1 = &bvh[node->left];
      const BVHNode* child2 = &bvh[node->left + 1];
      float d1 = testBoundingBox(ray, {tMin, hit.t}, child1->bounds);
      float d2 = testBoundingBox(ray, {tMin, hit.t}, child2->bounds);
      if (d1 > d2) {
        std::swap(d1, d2);
        std::swap(child1, child2);
      }
      if (isinf(d1)) {
        if (stackIdx == 0) break;
        node = stack[--stackIdx];
      } else {
        node = child1;
        if (!isinf(d2)) stack[stackIdx++] = child2;
      }
    }
  }

  return didHit;
}

// Möller–Trumbore intersection
bool BasicIntegrator::testTriangle(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Triangle& tri
) const {
  const float3&
    v0 = tri.v0.position,
    v1 = tri.v1.position,
    v2 = tri.v2.position;

  const float3 edge1 = v1 - v0;
  const float3 edge2 = v2 - v0;

  // Cramer's Rule
  const float3 rayEdge2 = cross(ray.dir, edge2);

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
  const float v = dot(ray.dir, bEdge1) * invDet;
  if (v < 0.0f || u + v > 1.0f) return false;

  const float t = dot(edge2, bEdge1) * invDet;
  if (t <= tMin || hit.t <= t) return false;

  hit.t = t;
  hit.position = ray(t);

  const float w = 1.0f - u - v;
  hit.normal = w * tri.v0.normal + u * tri.v1.normal + v * tri.v2.normal;

  return true;
}

float BasicIntegrator::testBoundingBox(
  const Ray& ray,
  const interval<float>& tInt,
  const fbounds3& bounds
) const {
  const float3 invDir = 1.0f / ray.dir;

  float tx1 = invDir.x() * (bounds.min.x() - ray.origin.x());
  float tx2 = invDir.x() * (bounds.max.x() - ray.origin.x());
  float tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);

  float ty1 = invDir.y() * (bounds.min.y() - ray.origin.y());
  float ty2 = invDir.y() * (bounds.max.y() - ray.origin.y());
  tmin = std::max(tmin, std::min(ty1, ty2));
  tmax = std::min(tmax, std::max(ty1, ty2));

  float tz1 = invDir.z() * (bounds.min.z() - ray.origin.z());
  float tz2 = invDir.z() * (bounds.max.z() - ray.origin.z());
  tmin = std::max(tmin, std::min(tz1, tz2));
  tmax = std::min(tmax, std::max(tz1, tz2));

  if (tmax >= tmin && tmin < tInt.max && tmax > tInt.min) return tmin;
  return std::numeric_limits<float>::infinity();
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