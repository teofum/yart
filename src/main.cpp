#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <math/math.hpp>
#include <output/ppm.hpp>
#include <cpu/tile-renderer.hpp>
#include <cpu/mis-integrator.hpp>
#include <cpu/naive-integrator.hpp>
#include <gltf/gltf.hpp>
#include <frontend/metal-sdl2/main.hpp>

using namespace yart::math;
using namespace yart::gltf;

using Sampler = yart::SobolSampler<yart::FastOwenScrambler>;
using Integrator = yart::cpu::MISIntegrator;

int main() {
//  yart::Buffer buffer(1600, 1200);
//  yart::Buffer buffer(800, 400); // Material test
//  yart::Buffer buffer(1600, 800);
//  yart::Buffer buffer(400, 400); // Furnace test
//  yart::Buffer buffer(600, 400); // 3:2 small
  yart::Buffer buffer(900, 600); // 3:2 medium
//  yart::Buffer buffer(1920, 1200); // 16:10 large
//  yart::Buffer buffer(400, 400); // 1:1 small
//  yart::Buffer buffer(1280, 1280); // 1:1 large

  yart::Camera camera({buffer.width(), buffer.height()}, 24.0f, 8.0f);
//  camera.apertureSides = 7;
  camera.exposure = 1.0f;

//  camera.moveAndLookAt({0.0f, 5.0f, 15.0f}, {0.0f, 5.0f, 0.0f}); // Cornell Box
//  camera.moveAndLookAt({0.0f, 5.0f, 15.0f}, {0.0f, 1.0f, 0.0f}); // Mat test
//  camera.moveAndLookAt({5.28f, 0.96f, 0.0f}, {2.57f, 1.09f, 1.1f}); // Sponza
//  camera.moveAndLookAt({8.5f, 1.8f, 0}, {0, 3.2f, 0}); // New Sponza
  camera.moveAndLookAt({-32.2f, 3.5f, -14.1f}, {-8.2f, 6.1f, -0.95f}); // Bistro
//  camera.moveAndLookAt({-32.2f, 3.5f, -14.1f}, {-7.1f, 4.0f, -0.95f}); // Bistro
//  camera.moveAndLookAt({-11.2f, 2.0f, -1.7f}, {-8.1f, 2.2f, 0.33f}); // Bistro
//  camera.moveAndLookAt({6.1f, 0.4f, 0.46f}, {1.5f, 0.5f, 0.3f}); // Car
//  camera.moveAndLookAt({6.1f, 0.4f, 0.46f}, {1.5f, 0.5f, -0.25f}); // Cars
//  camera.moveAndLookAt({6.3f, 0.83f, 0.46f}, {4.0f, 0.66f, 0.1f}); // Porsche
//  camera.moveAndLookAt({-10.3f, 1.0f, 9.9f}, {0.14f, 0.67f, 0.96f}); // Porsche
//  camera.moveAndLookAt({2.16f, 0.68f, -0.72f}, {1.7f, 0.82f, 0.17f}); // Porsche
//  camera.moveAndLookAt({-1.9f, 0.68f, 0.37f}, {-0.11f, 0.64f, 0.3f}); // Porsche
//  camera.moveAndLookAt({-0.17f, 0.88f, 3.76f}, {-0.2f, 0.95f, 0.3f}); // JS
//  camera.moveAndLookAt({-1.07f, 0.68f, 3.35f}, {-0.91f, 0.64f, 0.85f}); // Wh
//  camera.moveAndLookAt({5.0f, 50.0f, 5.0f}, {}); // City
//  camera.moveAndLookAt({11.07f, -0.98f, 10.62f}, {0.0f, 0.28f, 1.55f});

  std::unique_ptr<yart::Scene> scene = load("models/bistro.glb");

  yart::HDRTexture hdri = yart::loadTextureHDR("hdris/rosendal_plains_2_oct.hdr");
  auto envLight = yart::ImageInfiniteLight(100.0f, &hdri);
  envLight.transform = Transform::rotation(radians(270.0f), axis_y<float>);
  scene->addLight(std::move(envLight));

  yart::tonemap::AgX tonemapper;
  tonemapper.look = yart::tonemap::AgX::none;

  yart::cpu::TileRenderer<Sampler, Integrator> renderer(
    std::move(buffer),
    camera
  );
  renderer.scene = scene.get();
  renderer.samples = 256;
  renderer.tonemapper = &tonemapper;

  yart::frontend::MetalSDLFrontend frontend(&renderer);
  frontend.start();

  return 0;
}
