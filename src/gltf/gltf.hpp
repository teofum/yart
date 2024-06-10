#ifndef YART_GLTF_HPP
#define YART_GLTF_HPP

#include <filesystem>
#include <functional>

#include <fastgltf/core.hpp>
#include <fastgltf/tools.hpp>
#include <core/core.hpp>

namespace fs = std::filesystem;
using namespace yart::math;

template<>
struct fastgltf::ElementTraits<float3>
  : fastgltf::ElementTraitsBase<float3, AccessorType::Vec3, float> {
};

namespace yart::gltf {

std::optional<Node> load(const fs::path& path) noexcept;

}

#endif //YART_GLTF_HPP
