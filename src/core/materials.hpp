#ifndef YART_MATERIALS_HPP
#define YART_MATERIALS_HPP

#include <variant>

#include <math/math.hpp>

namespace yart {
using namespace math;

class Material {
public:
  template<typename D, typename E>
  requires (std::derived_from<D, Spectrum> && std::derived_from<E, Spectrum>)
  constexpr Material(D&& diffuse, E&& emissive)
    : m_diffuse(std::make_unique<D>(std::forward<D&&>(diffuse))),
      m_emissive(std::make_unique<E>(std::forward<E&&>(emissive))) {}

  [[nodiscard]] constexpr const Spectrum& diffuse() const noexcept {
    return *m_diffuse;
  }

  [[nodiscard]] constexpr const Spectrum& emissive() const noexcept {
    return *m_emissive;
  }

private:
  std::unique_ptr<Spectrum> m_diffuse = nullptr;
  std::unique_ptr<Spectrum> m_emissive = nullptr;
};

struct Scattered {
  SpectrumSample attenuation;
  SpectrumSample emission;
  Ray scattered;
};

struct Emitted {
  SpectrumSample emission;
};

struct Absorbed {
};

using ScatterResult = std::variant<Scattered, Emitted, Absorbed>;

}

#endif //YART_MATERIALS_HPP
