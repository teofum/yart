cmake_minimum_required(VERSION 3.28)
project(yart)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-Wall -fno-objc-arc")

include_directories(src)

add_subdirectory(fastgltf)
include_directories(fastgltf/include)

add_executable(yart src/main.cpp
        src/math/math_base.hpp
        src/math/vec.hpp
        src/math/math.hpp
        src/math/mat.hpp
        src/output/ppm.cpp
        src/output/ppm.hpp
        src/core/buffer.hpp
        src/core/ray.hpp
        src/core/types.hpp
        src/core/camera.hpp
        src/core/core.hpp
        src/core/mesh.hpp
        src/core/renderer.hpp
        src/core/node.hpp
        src/cpu/cpu-renderer.cpp
        src/cpu/cpu-renderer.hpp
        src/cpu/hit.hpp
        src/core/materials.hpp
        src/cpu/random.hpp
)
target_link_libraries(yart)
