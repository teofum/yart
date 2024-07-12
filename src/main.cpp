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
//  yart::Buffer buffer(800, 600);
//  yart::Buffer buffer(1600, 1200);
//  yart::Buffer buffer(800, 400); // Material test
//  yart::Buffer buffer(1600, 800);
//  yart::Buffer buffer(400, 400); // Furnace test
  yart::Buffer buffer(900, 600); // 3:2
//  yart::Buffer buffer(1920, 1200); // 16:10 large

  yart::Camera camera({buffer.width(), buffer.height()}, 35.0f, 2.8f);
//  camera.apertureSides = 7;
  camera.exposure = 1.0f;

//  camera.moveAndLookAt({0.0f, 5.0f, 15.0f}, {0.0f, 1.0f, 0.0f}); // Mat test
//  camera.moveAndLookAt({5.28f, 0.96f, 0.0f}, {2.57f, 1.09f, 1.1f}); // Sponza
  camera.moveAndLookAt({-32.2f, 3.5f, -14.1f}, {-8.2f, 6.1f, -0.95f}); // Bistro
//  camera.moveAndLookAt({6.1f, 0.4f, 0.46f}, {1.5f, 0.5f, 0.3f}); // Car
//  camera.moveAndLookAt({6.1f, 0.4f, 0.46f}, {1.5f, 0.5f, -0.25f}); // Cars
//  camera.moveAndLookAt({5.0f, 50.0f, 5.0f}, {}); // City
//  camera.moveAndLookAt({11.07f, -0.98f, 10.62f}, {0.0f, 0.28f, 1.55f});

//  std::unique_ptr<yart::Scene> scene = load("models/cornell_nee_tint.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/cornell_mat_coat_rough.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/furnace_glass_dl.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/sponza.glb");
  std::unique_ptr<yart::Scene> scene = load("models/bistro.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/deccer.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/hdri_test.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/small_city.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/cars.glb");

  yart::Texture hdri = yart::Texture::loadHDR("hdris/kloetzle_blei_oct.hdr");
  scene->addLight(yart::ImageInfiniteLight(100.0f, &hdri));

  yart::tonemap::AgX tonemapper;
  tonemapper.look = yart::tonemap::AgX::none;

  yart::cpu::TileRenderer<Sampler, Integrator> renderer(
    std::move(buffer),
    camera
  );
  renderer.scene = scene.get();
  renderer.samples = 128;
  renderer.tonemapper = &tonemapper;

  yart::frontend::MetalSDLFrontend frontend(&renderer);
  frontend.start();

  return 0;
}
