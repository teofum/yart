#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <math/math.hpp>
#include <output/ppm.hpp>
#include <cpu/cpu-renderer.hpp>
#include <gltf/gltf.hpp>

using namespace yart::math;

int main() {
  yart::Buffer buffer(800, 600);

  yart::Node root = yart::gltf::load("models/cornell.glb").value();

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(55.0f),
    {0.0f, 5.0f, 15.0f}
  );

  yart::cpu::CpuRenderer renderer(buffer, camera, 10);
  renderer.backgroundColor = float3(0.0f, 0.0f, 0.0f);
  renderer.render(root);

  std::ofstream outFile("out.ppm");
  yart::output::writePPM(outFile, buffer);
  return 0;
}

