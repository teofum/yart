#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <math/math.hpp>
#include <output/ppm.hpp>
#include <cpu/tile-renderer.hpp>
#include <cpu/basic-integrator.hpp>
#include <gltf/gltf.hpp>
#include <frontend/metal/app.hpp>


using namespace yart::math;

int main() {
  yart::Buffer buffer(800, 600);

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(55.0f),
    {0.0f, 5.0f, 15.0f}
  );

  yart::Node root = yart::gltf::load("models/cornell_gtest.glb").value();

  yart::cpu::TileRenderer<yart::cpu::BasicIntegrator> renderer(
    std::move(buffer),
    camera
  );
  renderer.backgroundColor = float3(0.0f, 0.0f, 0.0f);
  renderer.samples = 100;

  yart::frontend::metal::MetalFrontend app(&renderer, &root);

  return 0;
}

