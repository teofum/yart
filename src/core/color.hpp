#ifndef YART_COLOR_HPP
#define YART_COLOR_HPP

#include "spectrum.hpp"

namespace yart {

using XYZ = float3;
using RGB = float3;

class RGBToSpectrumLUT {
  static constexpr uint32_t res = 64;

public:
  using CoeffArray = float[3][res][res][res][3];

  constexpr RGBToSpectrumLUT(
    const CoeffArray* coeffs,
    const float* zNodes
  ) noexcept: m_coeffs(coeffs), m_zNodes(zNodes) {}

  [[nodiscard]] RGBSigmoidPolynomial operator()(const RGB& rgb) const;

private:
  const CoeffArray* m_coeffs;
  const float* m_zNodes;
};

class RGBColorSpace {
public:
  RGBColorSpace(
    const float2& r,
    const float2& g,
    const float2& b,
    const Spectrum& illuminant,
    const RGBToSpectrumLUT& lut
  );

  [[nodiscard]] RGB fromXYZ(const XYZ& xyz) const noexcept;

  [[nodiscard]] XYZ toXYZ(const RGB& rgb) const noexcept;

  [[nodiscard]] RGBSigmoidPolynomial toRGBSpectrum(const RGB& rgb) const;

  [[nodiscard]] constexpr const DenseSpectrum& illuminant() const noexcept {
    return m_illuminant;
  }

  [[nodiscard]] static float3x3 convert(
    const RGBColorSpace& from,
    const RGBColorSpace& to
  ) noexcept;

private:
  float2 r, g, b, w; // RGB primitives + whitepoint xy chromaticity
  float3x3 m_rgb2xyz, m_xyz2rgb;
  DenseSpectrum m_illuminant;
  std::unique_ptr<RGBToSpectrumLUT> m_lut;
};

class RGBSpectrum : public Spectrum {
public:
  RGBSpectrum(const RGBColorSpace& cs, const RGB& rgb);

  [[nodiscard]] constexpr float operator()(float wl) const override {
    return m_rsp(wl);
  }

  [[nodiscard]] constexpr float maxValue() const override {
    return m_rsp.maxValue();
  }

private:
  RGBSigmoidPolynomial m_rsp;
};

class RGBUnboundedSpectrum : public Spectrum {
public:
  RGBUnboundedSpectrum(const RGBColorSpace& cs, const RGB& rgb);

  [[nodiscard]] constexpr float operator()(float wl) const override {
    return m_scale * m_rsp(wl);
  }

  [[nodiscard]] constexpr float maxValue() const override {
    return m_scale * m_rsp.maxValue();
  }

private:
  float m_scale;
  RGBSigmoidPolynomial m_rsp;
};

class RGBIlluminantSpectrum : public Spectrum {
public:
  RGBIlluminantSpectrum(const RGBColorSpace& cs, const RGB& rgb);

  [[nodiscard]] constexpr float operator()(float wl) const override {
    return m_scale * m_rsp(wl) * m_illuminant(wl);
  }

  [[nodiscard]] constexpr float maxValue() const override {
    return m_scale * m_rsp.maxValue() * m_illuminant.maxValue();
  }

private:
  float m_scale;
  RGBSigmoidPolynomial m_rsp;
  const DenseSpectrum& m_illuminant;
};

XYZ spectrumToXYZ(const Spectrum& s);

RGB spectrumToRGB(const Spectrum& s, const RGBColorSpace& cs);

XYZ spectrumSampleToXYZ(const SpectrumSample& s, const Wavelengths& w);

RGB spectrumSampleToRGB(
  const SpectrumSample& s,
  const Wavelengths& w,
  const RGBColorSpace& cs
);

float2 chromaticity(const XYZ& xyz) noexcept;

XYZ xyYToXYZ(float2 xy, float Y = 1.0f);

namespace colorspace {

void init();

const RGBColorSpace& sRGB();

const RGBColorSpace& DCI_P3();

const RGBColorSpace& Rec2020();

}


}

#endif //YART_COLOR_HPP
