#include "ray-integrator.hpp"

namespace yart::cpu {

RayIntegrator::RayIntegrator(
  Buffer& buffer,
  const Camera& camera,
  Sampler& sampler
) noexcept: Integrator(buffer, camera, sampler) {}

float3 RayIntegrator::sample(uint32_t sx, uint32_t sy) {
  auto ray = m_camera.getRay({sx, sy}, m_sampler.getPixel2D());
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

  if (hit.t < testBoundingBox(rayObjSpace, {tMin, hit.t}, node.boundingBox()))
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

  hit.p = node.transform(hit.p, Transform::Type::Point);
  hit.n = node.transform(hit.n, Transform::Type::Normal);
  return true;
}

bool RayIntegrator::testMesh(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Mesh& mesh
) const {
  bool didHit = testBVH(ray, tMin, hit, mesh.bvh());
  if (didHit) hit.bsdf = &scene->material(mesh.materialIdx);

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
  float dStack[64] = {std::numeric_limits<float>::infinity()};
  float d = testBoundingBox(ray, {tMin, hit.t}, node->bounds);
  uint32_t stackIdx = 0;
  bool didHit = false;

  while (true) {
    if (d < hit.t) {
      if (node->span > 0) {
        for (size_t i = 0; i < node->span; i++) {
          didHit |= testTriangle(ray, tMin, hit, bvh.tri(node->first + i));
        }
        if (stackIdx == 0) break;
        node = stack[--stackIdx];
        d = dStack[stackIdx];
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
          d = dStack[stackIdx];
        } else {
          node = child1;
          d = d1;
          if (!isinf(d2)) {
            dStack[stackIdx] = d2;
            stack[stackIdx++] = child2;
          }
        }
      }
    } else {
      if (stackIdx == 0) break;
      node = stack[--stackIdx];
      d = dStack[stackIdx];
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
  const float3 edge1 = tri.v1.p - tri.v0.p;
  const float3 edge2 = tri.v2.p - tri.v0.p;

  // Cramer's Rule
  const float3 rayEdge2 = cross(ray.dir, edge2);

  const float det = dot(edge1, rayEdge2);
  // TODO: culling support
  if (std::abs(det) < epsilon) return false;

  const float invDet = 1.0f / det;
  const float3 b = ray.origin - tri.v0.p; // We're solving for Ax = b

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
  hit.p = ray(t);

  const float w = 1.0f - u - v;
  hit.n = w * tri.v0.normal + u * tri.v1.normal + v * tri.v2.normal;

  return true;
}

float RayIntegrator::testBoundingBox(
  const Ray& ray,
  const interval<float>& tInt,
  const fbounds3& bounds
) const {
  float3 bmin(
    bounds[ray.sign[0]][0],
    bounds[ray.sign[1]][1],
    bounds[ray.sign[2]][2]
  );
  float3 bmax(
    bounds[1 - ray.sign[0]][0],
    bounds[1 - ray.sign[1]][1],
    bounds[1 - ray.sign[2]][2]
  );

  float3 tmin = fma(bmin, ray.idir, ray.odir);
  float3 tmax = fma(bmax, ray.idir, ray.odir);

  float t0 = tInt.min, t1 = tInt.max;
  t0 = max(tmin[0], t0);
  t0 = max(tmin[1], t0);
  t0 = max(tmin[2], t0);
  t1 = min(tmax[0], t1);
  t1 = min(tmax[1], t1);
  t1 = min(tmax[2], t1);

  if (t1 >= t0) return t0;
  return std::numeric_limits<float>::infinity();
}

}