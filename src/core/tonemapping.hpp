#ifndef YART_TONEMAPPING_HPP
#define YART_TONEMAPPING_HPP

#include <math/math.hpp>

namespace yart::tonemap {
using namespace math;

class Tonemap {
public:
  [[nodiscard]] virtual constexpr float3 operator()(const float3& hdr) const noexcept = 0;
};

class AgX : public Tonemap {
public:
  struct Look {
    float3 offset, slope, power;
    float sat;
  };

  static constexpr const Look none = {
    float3(0.0f), float3(1.0f), float3(1.0f),
    1.0f
  };

  static constexpr const Look golden = {
    float3(0.0f), float3(1.0f, 0.9f, 0.5f), float3(0.8f),
    0.8f
  };

  static constexpr const Look punchy = {
    float3(0.0f), float3(1.0f), float3(1.35f),
    1.4f
  };

  Look look = none;

  [[nodiscard]] constexpr float3 operator()(const float3& hdr) const noexcept override {
    return end(applyLook(start(hdr)));
  }

private:
  [[nodiscard]] constexpr float3 contrast(const float3& x) const noexcept {
    float3 x2 = x * x;
    float3 x4 = x2 * x2;

    return +15.5 * x4 * x2
           - 40.14 * x4 * x
           + 31.96 * x4
           - 6.868 * x2 * x
           + 0.4298 * x2
           + 0.1191 * x
           - 0.00232;
  }

  [[nodiscard]] constexpr float3 start(float3 val) const noexcept {
    constexpr float3x3 agx(
      0.842479062253094, 0.0784335999999992, 0.0792237451477643,
      0.0423282422610123, 0.878468636469772, 0.0791661274605434,
      0.0423756549057051, 0.0784336, 0.879142973793104
    );

    constexpr float3 min_ev(-12.47393f);
    constexpr float3 max_ev(4.026069f);

    val = agx * val;
    val = clamp(log2(val), min_ev, max_ev);
    val = (val - min_ev) / (max_ev - min_ev);

    return contrast(val);
  }

  [[nodiscard]] constexpr float3 applyLook(float3 val) const noexcept {
    constexpr float3 lw(0.2126f, 0.7152f, 0.0722f);
    float luma = dot(val, lw);

    val = pow(val * look.slope + look.offset, look.power);
    return float3(luma) + look.sat * (val - luma);
  }

  [[nodiscard]] constexpr float3 end(float3 val) const noexcept {
    constexpr float3x3 agxInv(
      1.19687900512017, -0.0980208811401368, -0.0990297440797205,
      -0.0528968517574562, 1.15190312990417, -0.0989611768448433,
      -0.0529716355144438, -0.0980434501171241, 1.15107367264116
    );

    val = agxInv * val;
    val = clamp(val, float3(0.0f), float3(1.0f));
    return pow(val, float3(2.2f));
  }
};

}

#endif //YART_TONEMAPPING_HPP
