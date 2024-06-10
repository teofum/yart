#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <variant>
#include <vector>

#include "cpu-renderer.hpp"

namespace yart::cpu {
using namespace math;

CpuRenderThread::CpuRenderThread(
  Buffer& buffer,
  const Camera& camera,
  uint32 threadId,
  uint32 samples,
  uint32 maxDepth
) noexcept
  : Renderer(buffer, camera),
    m_samples(samples),
    m_maxDepth(maxDepth),
    m_threadId(threadId) {
  std::random_device rd; // Used to seed thread RNG
  m_threadRng = std::make_unique<Xoshiro::Xoshiro256PP>(Xoshiro::Xoshiro256PP(rd()));
}

void CpuRenderThread::render(const yart::Node& root) {
  auto start = std::chrono::high_resolution_clock::now();

  for (uint32 sample = 0; sample < m_samples; sample++) {
    for (size_t i = 0; i < buffer.width(); i++) {
      for (size_t j = 0; j < buffer.height(); j++) {
        auto ray = camera.getRay({i, j}, m_threadRng.get());
        float3 color = rayColor(ray, root);

        buffer(i, j) += float4(color, 1.0f);
      }
    }
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto delta = std::chrono::duration_cast<std::chrono::duration<float>>(
    end - start
  ).count();

  std::cout << "Thread " << m_threadId << " done in " << delta << " seconds\n";

  for (size_t i = 0; i < buffer.width(); i++) {
    for (size_t j = 0; j < buffer.height(); j++) {
      buffer(i, j) /= float(m_samples);
    }
  }
}

float3 CpuRenderThread::rayColor(
  const Ray& ray,
  const Node& root,
  uint32 depth
) {
  if (depth > m_maxDepth) return {0.0f, 0.0f, 0.0f};

  auto hit = testNode(
    ray,
    {0.001f, std::numeric_limits<float>::infinity()},
    root
  );
  if (!hit) return backgroundColor;

  ScatterResult res = scatter(ray, *hit);

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

std::optional<Hit> CpuRenderThread::testNode(
  const Ray& ray,
  const interval<float>& tInt,
  const Node& node
) const {
  const Ray rayObjSpace(
    float3(node.inverseTransform() * float4(ray.origin, 1.0f)),
    float3(node.inverseTransform() * float4(ray.dir(), 0.0f))
  );

  if (!testBoundingBox(rayObjSpace, tInt, node.boundingBox()))
    return std::nullopt;

  std::optional<Hit> closest = std::nullopt;
  float tMin = tInt.max;

  const Mesh* mesh;
  if ((mesh = node.mesh())) {
    auto hit = testMesh(rayObjSpace, tInt, *mesh);
    if (hit) {
      tMin = hit->t;
      closest = hit;
    }
  }

  for (const Node& child: node.children()) {
    auto hit = testNode(rayObjSpace, {tInt.min, tMin}, child);
    if (hit) {
      tMin = hit->t;
      closest = hit;
    }
  }

  closest->position = float3(node.transform() * float4(closest->position, 1.0));
  closest->normal = normalized(node.normalTransform() * closest->normal);

  return closest;
}

std::optional<Hit> CpuRenderThread::testMesh(
  const Ray& ray,
  const interval<float>& tInt,
  const Mesh& mesh
) const {
  std::optional<Hit> closest = std::nullopt;
  float tMin = tInt.max;

  for (const Triangle& tri: mesh.triangles()) {
    auto hit = testTriangle(ray, {tInt.min, tMin}, tri, mesh);
    if (hit) {
      tMin = hit->t;
      closest = hit;
    }
  }

  return closest;
}

// Möller–Trumbore intersection
std::optional<Hit> CpuRenderThread::testTriangle(
  const Ray& ray,
  const interval<float>& tInt,
  const Triangle& tri,
  const Mesh& mesh
) const {
  const float3&
    v0 = tri.vertices[0].position,
    v1 = tri.vertices[1].position,
    v2 = tri.vertices[2].position;

  const float3 edge1 = v1 - v0;
  const float3 edge2 = v2 - v0;

  // Cramer's Rule
  const float3 rayEdge2 = cross(ray.dir(), edge2);

  const float det = dot(edge1, rayEdge2);
  // TODO: culling support
  if (std::abs(det) < epsilon) return std::nullopt;

  const float invDet = 1.0f / det;
  const float3 b = ray.origin - v0; // We're solving for Ax = b

  const float u = dot(b, rayEdge2) * invDet;
  if (u < 0.0f || u > 1.0f) return std::nullopt;

  // Uses the property a.(b×c) = b.(c×a) = c.(a×b)
  // and a×b = b×(-a)
  const float3 bEdge1 = cross(b, edge1);
  const float v = dot(ray.dir(), bEdge1) * invDet;
  if (v < 0.0f || u + v > 1.0f) return std::nullopt;

  const float t = dot(edge2, bEdge1) * invDet;
  if (t <= tInt.min || tInt.max <= t) return std::nullopt;

  Hit hit;
  hit.t = t;
  hit.position = ray.at(t);

  const float w = 1.0f - u - v;
  hit.normal = w * tri.vertices[0].normal + u * tri.vertices[1].normal +
               v * tri.vertices[2].normal;

  hit.material = mesh.material;
  return std::make_optional(hit);
}

// "An Efficient and Robust Ray–Box Intersection Algorithm", Amy Williams
bool CpuRenderThread::testBoundingBox(
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

ScatterResult CpuRenderThread::scatter(const Ray& ray, const Hit& hit) {
  return std::visit(
    [&](const auto& mat) {
      return scatterImpl(mat, ray, hit);
    },
    *(hit.material)
  );
}

ScatterResult CpuRenderThread::scatterImpl(
  const Lambertian& mat,
  const Ray& ray,
  const Hit& hit
) {
  float4x4 basis = normalToTBN(hit.normal);
  float3 scatterDir = float3(
    basis * float4(random::randomCosineVec(m_threadRng.get()), 0.0f)
  );
  Ray scattered(hit.position, scatterDir);

  return Scattered{
    mat.albedo,
    {0.0f, 0.0f, 0.0f},
    scattered
  };
}

void CpuRenderer::render(const Node& root) {
  uint32 samplesPerThread = m_samples / m_threadCount;

  auto renderFunc = [&](
    Buffer* threadBuffer,
    uint32 threadId
  ) {
    CpuRenderThread threadRenderer(
      *threadBuffer,
      camera,
      threadId,
      samplesPerThread,
      m_maxDepth
    );
    threadRenderer.backgroundColor = backgroundColor;
    threadRenderer.render(root);
  };

  std::vector<Buffer> threadBuffers;
  threadBuffers.reserve(m_threadCount);
  std::vector<std::unique_ptr<std::thread>> threads;

  for (uint32 tid = 0; tid < m_threadCount; tid++) {
    std::packaged_task<void(Buffer*, uint32)> renderTask{renderFunc};
    threadBuffers.emplace_back(buffer.width(), buffer.height());

    threads.push_back(
      std::make_unique<std::thread>(
        std::thread(std::move(renderTask), &threadBuffers[tid], tid)
      )
    );
  }

  for (uint32 tid = 0; tid < m_threadCount; tid++) {
    threads[tid]->join();

    for (size_t i = 0; i < buffer.width(); i++) {
      for (size_t j = 0; j < buffer.height(); j++) {
        buffer(i, j) += threadBuffers[tid](i, j);
      }
    }
  }

  for (size_t i = 0; i < buffer.width(); i++) {
    for (size_t j = 0; j < buffer.height(); j++) {
      buffer(i, j) /= float(m_threadCount);
    }
  }
}

}

