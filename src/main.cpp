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

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(55.0f),
    {0.0f, 5.0f, 15.0f}
  );

  yart::Node root = yart::gltf::load("models/cornell.glb").value();

  yart::cpu::CpuRenderer renderer(std::move(buffer), camera, 10);
  renderer.backgroundColor = float3(0.0f, 0.0f, 0.0f);

  renderer.onRenderWaveComplete = [](
    const yart::Buffer& buffer,
    size_t wave,
    size_t total
  ) {
    std::cout << "Rendered " << wave << "/" << total << " samples\n";
    std::stringstream filename;
    filename << "out_" << wave << ".ppm";
    std::ofstream outFile(filename.str());
    yart::output::writePPM(outFile, buffer);
  };

  renderer.onRenderComplete = [](const yart::Buffer& buffer) {
    std::cout << "Writing to file...\n";
    std::ofstream outFile("out.ppm");
    yart::output::writePPM(outFile, buffer);
  };

  renderer.render(root);

  return 0;
}

