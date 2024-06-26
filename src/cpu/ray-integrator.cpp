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

  float d;
  if (!testBoundingBox(rayObjSpace, {tMin, hit.t}, node.boundingBox(), &d) ||
      hit.t < d)
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
  hit.tg = node.transform(hit.tg, Transform::Type::Vector);
  return true;
}

bool RayIntegrator::testMesh(
  const Ray& ray,
  float tMin,
  Hit& hit,
  const Mesh& mesh
) const {
  bool didHit = testBVH(ray, tMin, hit, mesh.bvh());
  if (didHit) {
    hit.bsdf = &scene->material(mesh.materialIdx);
    hit.lightIdx = mesh.lightIdx;
    if (mesh.lightIdx != -1) hit.light = &scene->light(mesh.lightIdx);
  }

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
  float d;
  uint32_t stackIdx = 0;
  bool didHit = false;

  if (!testBoundingBox(ray, {tMin, hit.t}, node->bounds, &d)) return false;

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
        float d1, d2;
        bool hit1 = testBoundingBox(ray, {tMin, hit.t}, child1->bounds, &d1);
        bool hit2 = testBoundingBox(ray, {tMin, hit.t}, child2->bounds, &d2);
        if (hit1) {
          if (hit2) {
            if (d1 > d2) {
              std::swap(d1, d2);
              std::swap(child1, child2);
            }
            dStack[stackIdx] = d2;
            stack[stackIdx++] = child2;
          }
          node = child1;
          d = d1;
        } else if (hit2) {
          node = child2;
          d = d2;
        } else {
          if (stackIdx == 0) break;
          node = stack[--stackIdx];
          d = dStack[stackIdx];
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
  hit.tg = w * tri.v0.tangent + u * tri.v1.tangent + v * tri.v2.tangent;

  return true;
}

bool RayIntegrator::testBoundingBox(
  const Ray& ray,
  const interval<float>& tInt,
  const fbounds3& bounds,
  float* d
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

  *d = t0;
  return t1 >= t0;
}

}