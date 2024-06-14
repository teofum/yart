#ifndef YART_BOUNDS_HPP
#define YART_BOUNDS_HPP

#include "math_base.hpp"
#include "vec.hpp"

namespace yart::math {

template<numeric T, std::size_t N>
class bounds {
public:
  vec<T, N> min = vec<T, N>(std::numeric_limits<T>::infinity());
  vec<T, N> max = vec<T, N>(-std::numeric_limits<T>::infinity());

  constexpr bounds<T, N>() noexcept = default;

  constexpr bounds<T, N>(vec<T, N> min, vec <T, N> max) noexcept
    : min(min), max(max) {}

  [[nodiscard]] constexpr vec<T, N>& operator[](std::size_t i) noexcept {
    return i == 0 ? min : max;
  }

  [[nodiscard]] constexpr const vec<T, N>& operator[](std::size_t i) const noexcept {
    return i == 0 ? min : max;
  }

  [[nodiscard]] constexpr vec<T, N> size() const noexcept {
    return max - min;
  }

  [[nodiscard]] constexpr float area() const noexcept {
    float3 size = max - min;
    return size.x() * size.y() + size.y() * size.z() + size.z() * size.x();
  }

  constexpr void expandToInclude(const vec<T, N>& point) noexcept {
    min = math::min(min, point);
    max = math::max(max, point);
  }

  template<numeric... Ts>
  [[nodiscard]] static constexpr bounds<T, N> join(bounds<Ts, N> ...a) noexcept {
    bounds<T, N> _union;
    for (const bounds<T, N>& b: {a...}) {
      for (std::size_t i = 0; i < N; i++) {
        _union.min[i] = math::min(_union.min[i], b.min[i]);
        _union.max[i] = math::max(_union.max[i], b.max[i]);
      }
    }

    return _union;
  }

  template<numeric U>
  [[nodiscard]] static constexpr bounds<T, N> intersection(
    bounds<T, N> a,
    bounds<U, N> b
  ) noexcept {
    bounds<T, N> intersection;
    for (std::size_t i = 0; i < N; i++) {
      intersection.min[i] = math::max(a.min[i], b.min[i]);
      intersection.max[i] = math::min(a.max[i], b.max[i]);
    }

    return intersection;
  }

  template<typename range_T>
  requires std::ranges::random_access_range<range_T>
  [[nodiscard]] static constexpr bounds<T, N> fromPoints(const range_T& points) noexcept {
    bounds<T, N> b;
    for (const vec<T, N>& p: points) {
      for (size_t i = 0; i < N; i++) {
        if (p[i] < b.min[i]) b.min[i] = p[i];
        if (p[i] > b.max[i]) b.max[i] = p[i];
      }
    }
    b.min -= float3(0.001);
    b.max += float3(0.001);

    return b;
  }

  template<typename... Ts>
  [[nodiscard]] static constexpr bounds<T, N> fromPoints(const Ts& ...points) noexcept {
    bounds<T, N> b;
    for (const vec<T, N>& p: {points...}) {
      for (size_t i = 0; i < N; i++) {
        if (p[i] < b.min[i]) b.min[i] = p[i];
        if (p[i] > b.max[i]) b.max[i] = p[i];
      }
    }
    b.min -= float3(0.001);
    b.max += float3(0.001);

    return b;
  }
};

template<numeric T>
using bounds2 = bounds<T, 2>;
template<numeric T>
using bounds3 = bounds<T, 3>;

using ubounds2 = bounds2<uint32_t>;
using ubounds3 = bounds3<uint32_t>;

using fbounds2 = bounds2<float>;
using fbounds3 = bounds3<float>;

using dbounds2 = bounds2<double>;
using dbounds3 = bounds3<double>;

}

#endif //YART_BOUNDS_HPP
