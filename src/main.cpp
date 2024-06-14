#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <math/math.hpp>
#include <output/ppm.hpp>
#include <cpu/tile-renderer.hpp>
#include <cpu/naive-integrator.hpp>
#include <gltf/gltf.hpp>
#include <frontend/metal/app.hpp>


using namespace yart::math;

int main() {
  yart::Buffer buffer(800, 600);

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(55.0f),
//    {-7.0f, 5.0f, 0.0f}, axis_x<float> // Sponza
    {0.0f, 5.0f, 15.0f} // Cornell box
  );

  yart::Node root = yart::gltf::load("models/cornell_dragon.glb").value();

  yart::cpu::TileRenderer<yart::cpu::NaiveIntegrator> renderer(
    std::move(buffer),
    camera
  );
  renderer.backgroundColor = float3(0.0f, 0.0f, 0.0f);
  renderer.samples = 100;

  yart::frontend::metal::MetalFrontend app(&renderer, &root);

  return 0;
}

