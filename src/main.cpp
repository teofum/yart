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
  yart::Buffer buffer(800, 600);
//  yart::Buffer buffer(800, 400); // Material test
//  yart::Buffer buffer(400, 400); // Furnace test

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(55.0f),
    {0.0f, 5.0f, 15.0f} // Cornell box / Furnace test
  );

//  camera.moveAndLookAt({0.0f, 5.0f, 15.0f}, {0.0f, 1.0f, 0.0f}); // Mat test
  camera.moveAndLookAt({6.0f, 3.0f, 0.0f}, {0.0f, 3.0f, 0.0f}); // Sponza

//  std::unique_ptr<yart::Scene> scene = load("models/cornell_multimat_test_3.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/cornell_mat_metal.glb");
//  std::unique_ptr<yart::Scene> scene = load("models/furnace_glass_dl.glb");
  std::unique_ptr<yart::Scene> scene = load("models/sponza.glb");

  yart::tonemap::AgX tonemapper;
  tonemapper.look = yart::tonemap::AgX::none;

  yart::cpu::TileRenderer<Sampler, Integrator> renderer(
    std::move(buffer),
    camera
  );

//  renderer.backgroundColor = float3(0.5f);
  renderer.scene = scene.get();
  renderer.samples = 256;
  renderer.firstWaveSamples = 1;
  renderer.maxWaveSamples = 128;
  renderer.tonemapper = &tonemapper;

  yart::frontend::MetalSDLFrontend frontend(&renderer);
  frontend.start();

  return 0;
}
