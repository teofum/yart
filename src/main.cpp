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
  yart::Buffer buffer(800, 400); // Material test
//  yart::Buffer buffer(1600, 800);
//  yart::Buffer buffer(400, 400); // Furnace test

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(20.0f),
    {0.0f, 5.0f, 15.0f} // Cornell box / Furnace test
  );

  camera.moveAndLookAt({0.0f, 5.0f, 15.0f}, {0.0f, 1.0f, 0.0f}); // Mat test
//  camera.moveAndLookAt({5.28f, 0.96f, 0.0f}, {2.57f, 1.09f, 1.1f}); // Sponza
//  camera.moveAndLookAt({5.0f, 50.0f, 5.0f}, {}); // City
//  camera.moveAndLookAt({11.07f, -0.98f, 10.62f}, {0.0f, 0.28f, 1.55f});

//  std::unique_ptr<yart::Scene> scene = load("models/cornell_metaldragon.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/cornell_mat_metal.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/furnace_glass_dl.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/sponza_unlit.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/deccer.glb");
  std::unique_ptr<yart::Scene> scene = load("models/hdri_test.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/small_city.glb");

  yart::Texture hdri = yart::Texture::loadHDR("hdris/autumn_park_4k.hdr");
  scene->addLight(yart::ImageInfiniteLight(100.0f, &hdri));

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
