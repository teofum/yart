#include "cpu-renderer.hpp"

namespace yart::cpu {
using namespace math;

CpuRenderer::CpuRenderer(
  const Buffer& buffer,
  const Camera& camera,
  uint32_t threadCount
) noexcept
  : Renderer(buffer, camera),
    m_threadCount(threadCount),
    m_samplesPerThread(m_samples / m_threadCount) {}

CpuRenderer::CpuRenderer(
  Buffer&& buffer,
  const Camera& camera,
  uint32_t threadCount
) noexcept
  : Renderer(buffer, camera),
    m_threadCount(threadCount),
    m_samplesPerThread(m_samples / m_threadCount) {}

void CpuRenderer::render(const Node& root) {
  std::random_device rd; // Used to initialize thread RNG

  for (uint32_t ti = 0; ti < m_threadCount; ti++) {
    std::packaged_task<void()> renderTask{[&]() {
      CpuRenderThread renderThread(this, rd);
      renderThread(root);
    }};

    m_threads.push_back(
      std::make_unique<std::thread>(std::thread(std::move(renderTask)))
    );
  }
}

void CpuRenderer::writeBuffer(const Buffer& src, size_t wave) noexcept {
  std::unique_lock lock(m_bufferMutex);

  if (wave < m_currentWave) return;
  if (wave > m_currentWave) {
    m_currentWave = wave;
    m_currentWaveFinishedThreads = 0;

    for (size_t i = 0; i < src.width(); i++) {
      for (size_t j = 0; j < src.height(); j++) {
        m_buffer(i, j) = src(i, j) / (float(m_threadCount) * float(wave));
      }
    }
  } else {
    for (size_t i = 0; i < src.width(); i++) {
      for (size_t j = 0; j < src.height(); j++) {
        m_buffer(i, j) += src(i, j) / (float(m_threadCount) * float(wave));
      }
    }
  }

  if (++m_currentWaveFinishedThreads == m_threadCount) {
    if (onRenderWaveComplete) {
      auto cb = onRenderWaveComplete.value();
      cb(m_buffer, wave, m_samplesPerThread);
    }

    if (m_currentWave == m_samplesPerThread && onRenderComplete) {
      auto cb = onRenderComplete.value();
      cb(m_buffer);
    }
  }
}


CpuRenderThread::CpuRenderThread(
  CpuRenderer* renderer,
  std::random_device& rd
) noexcept
  : m_renderer(renderer),
    m_threadRng(rd()),
    m_threadBuffer(renderer->m_buffer.width(), renderer->m_buffer.height()) {
  m_samples = renderer->m_samplesPerThread;
  m_maxDepth = renderer->m_maxDepth;
}

void CpuRenderThread::operator()(const Node& root) noexcept {
  auto start = std::chrono::high_resolution_clock::now();

  size_t nextWave = 1;

  for (uint32_t sample = 0; sample < m_samples;) {
    for (size_t i = 0; i < m_threadBuffer.width(); i++) {
      for (size_t j = 0; j < m_threadBuffer.height(); j++) {
        auto ray = m_renderer->m_camera.getRay({i, j}, m_threadRng);
        float3 color = rayColor(ray, root);

        m_threadBuffer(i, j) += float4(color, 1.0f);
      }
    }

    if (++sample == nextWave && nextWave != m_samples) {
      m_renderer->writeBuffer(m_threadBuffer, sample);
      nextWave *= 2;
    }
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto delta = std::chrono::duration_cast<std::chrono::duration<float>>(
    end - start
  );

  std::cout << "Thread " << std::this_thread::get_id() << " done in " << delta
            << "\n";

  m_renderer->writeBuffer(m_threadBuffer, m_samples);
}

float3 CpuRenderThread::rayColor(
  const Ray& ray,
  const Node& root,
  uint32_t depth
) {
  if (depth > m_maxDepth) return {0.0f, 0.0f, 0.0f};

  auto hit = testNode(
    ray,
    {0.001f, std::numeric_limits<float>::infinity()},
    root
  );
  if (!hit) return m_renderer->backgroundColor;

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
    v0 = tri.vertices[0]->position,
    v1 = tri.vertices[1]->position,
    v2 = tri.vertices[2]->position;

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
  hit.normal = w * tri.vertices[0]->normal + u * tri.vertices[1]->normal +
               v * tri.vertices[2]->normal;

  hit.material = &mesh.material;
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
    basis * float4(random::randomCosineVec(m_threadRng), 0.0f)
  );
  Ray scattered(hit.position, scatterDir);

  return Scattered{
    mat.albedo,
    {0.0f, 0.0f, 0.0f},
    scattered
  };
}

ScatterResult CpuRenderThread::scatterImpl(
  const Emissive& mat,
  const Ray& ray,
  const Hit& hit
) {
  return Emitted{mat.emission};
}

}

