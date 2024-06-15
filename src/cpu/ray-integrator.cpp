#include "ray-integrator.hpp"

namespace yart::cpu {

RayIntegrator::RayIntegrator(Buffer& buffer, const Camera& camera) noexcept
  : Integrator(buffer, camera) {
  // TODO: take rng seed as parameter
  std::random_device rd;
  m_rng = Xoshiro::Xoshiro256PP(rd());
}

float3 RayIntegrator::sample(uint32_t sx, uint32_t sy) {
  auto ray = m_camera.getRay({sx, sy}, m_rng);
  return Li(ray);
}

bool RayIntegrator::testNode(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Node& node
) const {
  const Ray rayObjSpace(
    node.transform.inverse(ray.origin, Transform::Type::Point),
    node.transform.inverse(ray.dir, Transform::Type::Vector)
  );

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

bool RayIntegrator::testMesh(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Mesh& mesh
) const {
  bool didHit = testBVH(ray, tMin, hit, mesh.bvh());
  if (didHit) hit.material = &scene->material(mesh.materialIdx);

  return didHit;
}

bool RayIntegrator::testBVH(
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
bool RayIntegrator::testTriangle(
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

float RayIntegrator::testBoundingBox(
  const Ray& ray,
  const interval<float>& tInt,
  const fbounds3& bounds
) const {
  float3 t1 = fma(bounds.min, ray.idir, ray.odir);
  float3 t2 = fma(bounds.max, ray.idir, ray.odir);

  float tmin = min(t1.x(), t2.x()), tmax = max(t1.x(), t2.x());
  tmin = max(tmin, min(t1.y(), t2.y()));
  tmax = min(tmax, max(t1.y(), t2.y()));
  tmin = max(tmin, min(t1.z(), t2.z()));
  tmax = min(tmax, max(t1.z(), t2.z()));

  if (tmax >= tmin && tmin < tInt.max && tmax > tInt.min) return tmin;
  return std::numeric_limits<float>::infinity();
}

}