cmake_minimum_required(VERSION 3.28)
project(yart)

function(build_shaders LIB_NAME)
    message(STATUS "Building shader lib: ${LIB_NAME}")

    set(SHADER_AIRS "")
    foreach (SHADER_PATH IN LISTS ARGN)
        get_filename_component(SHADER_FILENAME ${SHADER_PATH} NAME_WLE)
        message(STATUS "  Compiling: ${SHADER_FILENAME}")

        set(SHADER_SRC "${CMAKE_SOURCE_DIR}/${SHADER_PATH}")
        get_filename_component(LIB_NAME_WLE ${LIB_NAME} NAME_WLE)
        set(SHADER_AIR "${LIB_NAME_WLE}_${SHADER_FILENAME}.air")

        list(APPEND SHADER_AIRS ${SHADER_AIR})
        add_custom_command(
                OUTPUT ${SHADER_AIR}
                COMMAND xcrun -sdk macosx metal -o ${SHADER_AIR} -c ${SHADER_SRC}
                DEPENDS ${SHADER_SRC}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        )
    endforeach ()

    message(STATUS "  Building: ${SHADER_AIRS}")
    add_custom_command(
            OUTPUT ${LIB_NAME}
            COMMAND xcrun -sdk macosx metallib -o ${CMAKE_SOURCE_DIR}/${LIB_NAME} ${SHADER_AIRS}
            DEPENDS ${SHADER_AIRS}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endfunction()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-Wall -fno-objc-arc")
set(CMAKE_EXE_LINKER_FLAGS "-framework Metal -framework Foundation -framework Cocoa -framework CoreGraphics -framework MetalKit")

add_subdirectory(metal-cmake)  # Library definition
include_directories(metal-cmake/metal-cpp)
include_directories(metal-cmake/metal-cpp-extensions)

include_directories(src xoshiro-rng)

add_subdirectory(fastgltf)
include_directories(fastgltf/include)

set(SRC_MATH
        src/math/math_base.hpp
        src/math/vec.hpp
        src/math/math.hpp
        src/math/mat.hpp
        src/math/sampling.hpp
        src/math/sampling.cpp
        src/math/bounds.hpp
        src/core/ray.hpp
        src/math/transform.hpp
        src/math/frame.hpp)

set(SRC_CORE
        src/core/buffer.hpp
        src/core/types.hpp
        src/core/camera.hpp
        src/core/core.hpp
        src/core/mesh.hpp
        src/core/renderer.hpp
        src/core/scene.hpp
        src/core/bsdf.hpp
        src/core/utils.hpp
        src/core/bvh.hpp
        src/core/primitives.hpp
        src/core/rng.hpp
        src/core/sampler.hpp
        src/core/sampler.cpp
        src/core/rng.cpp
        src/core/bsdf.cpp
)

set(SRC_IO
        src/output/ppm.cpp
        src/output/ppm.hpp
        src/gltf/gltf.cpp
        src/gltf/gltf.hpp)

set(SRC_CPU
        src/cpu/hit.hpp
        src/cpu/tile-renderer.hpp
        src/cpu/integrator.hpp
        src/cpu/integrator.cpp
        src/cpu/ray-integrator.cpp
        src/cpu/ray-integrator.hpp
        src/cpu/naive-integrator.cpp
        src/cpu/naive-integrator.hpp
)

set(SRC_FRONTEND_METAL
        src/frontend/metal/app-delegate.cpp
        src/frontend/metal/app-delegate.hpp
        src/frontend/metal/view-delegate.cpp
        src/frontend/metal/view-delegate.hpp
        src/frontend/metal/utils.hpp
        src/frontend/metal/app.hpp
        src/frontend/metal/app.cpp)

build_shaders(yart.metallib src/frontend/metal/display.metal)
add_executable(yart src/main.cpp
        ${SRC_CORE}
        ${SRC_MATH}
        ${SRC_IO}
        ${SRC_CPU}
        ${SRC_FRONTEND_METAL}
        xoshiro-rng/xoshiro.hpp
        yart.metallib
)
target_link_libraries(yart fastgltf metal_cpp)
