#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <math/math.hpp>
#include <output/ppm.hpp>
#include <cpu/tile-renderer.hpp>
#include <cpu/mis-integrator.hpp>
#include <cpu/naive-integrator.hpp>
#include <gltf/gltf.hpp>
#include <frontend/metal/app.hpp>

using namespace yart::math;
using namespace yart::gltf;

int main() {
  yart::Buffer buffer(800, 600);

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(55.0f),
//    {-7.0f, 5.0f, 0.0f}, axis_x<float> // Sponza
    {0.0f, 5.0f, 15.0f} // Cornell box
  );

  yart::Scene scene = load("models/cornell_dragon_lights.glb").value();
//  yart::Scene scene = load("models/sponza_nomats.glb").value();

  yart::cpu::TileRenderer<yart::NaiveSampler, yart::cpu::MISIntegrator> renderer(
    std::move(buffer),
    camera
  );
  renderer.scene = &scene;
  renderer.backgroundColor = float3(0.0f, 0.0f, 0.0f);
  renderer.samples = 128;

  yart::frontend::metal::MetalFrontend app(&renderer);

  return 0;
}

