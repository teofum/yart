#include "color.hpp"

namespace yart {

RGBSigmoidPolynomial RGBToSpectrumLUT::operator()(const RGB& rgb) const {
  // Handle uniform RGB values by returning a constant spectrum
  if (rgb[0] == rgb[1] && rgb[1] == rgb[2]) {
    // Invert the sigmoid function to find the constant factor
    return {0, 0, (rgb[0] - 0.5f) / std::sqrt(rgb[0] * (1.0f - rgb[0]))};
  }

  // Find the maximum component and remap values
  // Colors are tabularized according to the max component to better account
  // for discontinuities in the mapping
  uint32_t maxC = (rgb[0] > rgb[1])
                  ? (rgb[0] > rgb[2] ? 0 : 2)
                  : (rgb[1] > rgb[2] ? 1 : 2);
  float z = rgb[maxC];
  float x = rgb[(maxC + 1) % 3] * float(res - 1) / z;
  float y = rgb[(maxC + 2) % 3] * float(res - 1) / z;

  // Get indices and offsets for LUT interpolation
  uint32_t xi = std::min(uint32_t(x), res - 2);
  uint32_t yi = std::min(uint32_t(y), res - 2);
  uint32_t zi = findInterval(res, [&](int64_t i) { return m_zNodes[i] < z; });

  float dx = x - float(xi), dy = y - float(yi);
  float dz = (z - m_zNodes[zi]) / (m_zNodes[zi + 1] - m_zNodes[zi]);

  // Cubic lerp over the LUT
  std::array<float, 3> c{};
  for (uint32_t i = 0; i < 3; i++) {
    auto co = [&](uint32_t dxi, uint32_t dyi, uint32_t dzi) {
      return (*m_coeffs)[maxC][zi + dzi][yi + dyi][xi + dxi][i];
    };

    c[i] = lerp(
      lerp(
        lerp(co(0, 0, 0), co(1, 0, 0), dx),
        lerp(co(0, 1, 0), co(1, 1, 0), dx),
        dy
      ),
      lerp(
        lerp(co(0, 0, 1), co(1, 0, 1), dx),
        lerp(co(0, 1, 1), co(1, 1, 1), dx),
        dy
      ),
      dz
    );
  }

  return {c[0], c[1], c[2]};
}

XYZ spectrumToXYZ(const Spectrum& s) {
  return XYZ(
    Spectrum::innerProduct(spectra::X(), s),
    Spectrum::innerProduct(spectra::Y(), s),
    Spectrum::innerProduct(spectra::Z(), s)
  ) / spectra::CIE_Y_integral;
}

RGB spectrumToRGB(const Spectrum& s, const RGBColorSpace& cs) {
  return cs.fromXYZ(spectrumToXYZ(s));
}

XYZ spectrumSampleToXYZ(const SpectrumSample& s, const Wavelengths& w) {
  SpectrumSample x = spectra::X().sample(w);
  SpectrumSample y = spectra::Y().sample(w);
  SpectrumSample z = spectra::Z().sample(w);

  SpectrumSample pdf = w.pdf();
  XYZ xyz(
    (x * s / pdf).average(),
    (y * s / pdf).average(),
    (z * s / pdf).average()
  );
  return xyz / spectra::CIE_Y_integral;
}

RGB spectrumSampleToRGB(
  const SpectrumSample& s,
  const Wavelengths& w,
  const RGBColorSpace& cs
) {
  return cs.fromXYZ(spectrumSampleToXYZ(s, w));
}

float2 chromaticity(const XYZ& xyz) noexcept {
  return {xyz.x() / sum(xyz), xyz.y() / sum(xyz)};
}

XYZ xyYToXYZ(float2 xy, float Y) {
  if (xy.y() == 0.0f) return {};
  return {xy.x() * Y / xy.y(), Y, (1 - xy.x() - xy.y()) * Y / xy.y()};
}

RGBColorSpace::RGBColorSpace(
  const float2& r,
  const float2& g,
  const float2& b,
  const Spectrum& illuminant,
  const RGBToSpectrumLUT& lut
) : r(r), g(g), b(b), m_illuminant(illuminant),
    m_lut(std::make_unique<RGBToSpectrumLUT>(lut)) {
  const auto W = spectrumToXYZ(illuminant);
  w = chromaticity(W);
  XYZ R = xyYToXYZ(r), G = xyYToXYZ(g), B = xyYToXYZ(b);

  float3x3 rgb = float3x3::fromColumns(R, G, B);
  XYZ c = inverse(rgb).value() * W;
  m_rgb2xyz = rgb * float3x3(c.x(), c.y(), c.z());
  m_xyz2rgb = inverse(m_rgb2xyz).value();
}

RGB RGBColorSpace::fromXYZ(const XYZ& xyz) const noexcept {
  return m_xyz2rgb * xyz;
}

XYZ RGBColorSpace::toXYZ(const RGB& rgb) const noexcept {
  return m_rgb2xyz * rgb;
}

float3x3 RGBColorSpace::convert(
  const RGBColorSpace& from,
  const RGBColorSpace& to
) noexcept {
  return to.m_xyz2rgb * from.m_rgb2xyz;
}

RGBSigmoidPolynomial RGBColorSpace::toRGBSpectrum(const RGB& rgb) const {
  return (*m_lut)(rgb);
}

RGBSpectrum::RGBSpectrum(const RGBColorSpace& cs, const RGB& rgb)
  : m_rsp(cs.toRGBSpectrum(rgb)) {}

RGBUnboundedSpectrum::RGBUnboundedSpectrum(
  const RGBColorSpace& cs,
  const RGB& rgb
) : m_scale(2.0f * std::max({rgb.x(), rgb.y(), rgb.z()})),
    m_rsp(cs.toRGBSpectrum(m_scale > 0.0f ? rgb / m_scale : RGB())) {}

RGBIlluminantSpectrum::RGBIlluminantSpectrum(
  const RGBColorSpace& cs,
  const RGB& rgb
) : m_scale(2.0f * std::max({rgb.x(), rgb.y(), rgb.z()})),
    m_rsp(cs.toRGBSpectrum(m_scale > 0.0f ? rgb / m_scale : RGB())),
    m_illuminant(cs.illuminant()) {}


namespace lut {
extern const float sRGBLUTScale[64];
extern const RGBToSpectrumLUT::CoeffArray sRGBLUTData;
}

namespace colorspace {

const RGBColorSpace* cs_sRGB, * cs_DCI_P3, * cs_Rec2020;

const RGBColorSpace& sRGB() { return *cs_sRGB; }

const RGBColorSpace& DCI_P3() { return *cs_DCI_P3; }

const RGBColorSpace& Rec2020() { return *cs_Rec2020; }

void init() {
  cs_sRGB = new RGBColorSpace(
    {0.640, 0.330},
    {0.300, 0.600},
    {0.150, 0.060},
    spectra::D(6500, 1.0f),
    RGBToSpectrumLUT(&lut::sRGBLUTData, lut::sRGBLUTScale)
  );
//  DCI_P3 = new RGBColorSpace(
//    {0.680, 0.320},
//    {0.265, 0.690},
//    {0.150, 0.060},
//    spectra::D(6500, 1.0f),
//    RGBToSpectrumLUT(DCI_P3LUTData, DCI_P3LUTScale)
//  );
//  Rec2020 = new RGBColorSpace(
//    {0.708, 0.292},
//    {0.170, 0.797},
//    {0.131, 0.046},
//    spectra::D(6500, 1.0f),
//    RGBToSpectrumLUT(Rec2020LUTData, Rec2020LUTScale)
//  );
}

}

}
