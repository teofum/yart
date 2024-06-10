#include <iostream>
#include <fstream>

#include <core/core.hpp>
#include <math/math.hpp>
#include <output/ppm.hpp>
#include <cpu/cpu-renderer.hpp>

using namespace yart::math;

int main() {
  yart::Buffer buffer(800, 600);

  yart::Camera camera(
    {buffer.width(), buffer.height()},
    radians(90.0f),
    {0.0f, 1.0f, 3.0f}
  );

  yart::Material defaultMaterial(yart::Lambertian{float3(0.9f)});
  yart::Material redMaterial(yart::Lambertian{float3(1.0f, 0.3f, 0.4f)});

  std::vector<yart::Vertex> wallVerts{
    {{-1, -1, 0},   {0, 0, 1}, {0, 0}},
    {{1,  -1, 0},   {0, 0, 1}, {1, 0}},
    {{-1, 1,  0.5}, {0, 0, 1}, {0, 1}},
    {{1,  1,  0.5}, {0, 0, 1}, {1, 1}}
  };
  std::vector<yart::Face> wallFaces{
    {0, 1, 2},
    {1, 3, 2},
  };
  yart::Mesh wallMesh(
    std::move(wallVerts),
    std::move(wallFaces),
    &redMaterial
  );

  std::vector<yart::Vertex> floorVerts{
    {{-1, 0, 1},  {0, 1, 0}, {0, 0}},
    {{1,  0, 1},  {0, 1, 0}, {1, 0}},
    {{-1, 0, -1}, {0, 1, 0}, {0, 1}},
    {{1,  0, -1}, {0, 1, 0}, {1, 1}}
  };
  std::vector<yart::Face> floorFaces{
    {0, 1, 2},
    {1, 3, 2},
  };
  yart::Mesh floorMesh(
    std::move(floorVerts),
    std::move(floorFaces),
    &defaultMaterial
  );

  yart::Node wall(wallMesh);
  wall.translate({1.0f, 1.0f, 0.0f});
  wall.rotate(radians(-45.0f), axis_y<float>);

  yart::Node floor(floorMesh);
  floor.scale(10.0f);

  yart::Node root;
  root.appendChild(floor);
  root.appendChild(wall);

  yart::cpu::CpuRenderer renderer(buffer, camera, 10);
  renderer.backgroundColor = float3(0.7, 0.8, 1.0);
  renderer.render(root);

  std::ofstream outFile("out.ppm");
  yart::output::writePPM(outFile, buffer);
  return 0;
}

