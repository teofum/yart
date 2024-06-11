#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <math/math.hpp>
#include <output/ppm.hpp>
#include <cpu/cpu-renderer.hpp>
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

  yart::Node root = yart::gltf::load("models/cornell.glb").value();

  yart::cpu::CpuRenderer renderer(std::move(buffer), camera, 10);
  renderer.backgroundColor = float3(0.0f, 0.0f, 0.0f);

  yart::frontend::metal::MetalFrontend app(&renderer, &root);

  return 0;
}

