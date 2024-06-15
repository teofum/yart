#ifndef YART_SPECTRUM_HPP
#define YART_SPECTRUM_HPP

#include <span>
#include <ranges>

#include <math/math.hpp>

namespace yart {
using namespace math;

static constexpr size_t nSpectrumSamples = 4;
static constexpr float minWavelength = 360;
static constexpr float maxWavelength = 830;
static constexpr size_t nWavelengths = size_t(
  maxWavelength - minWavelength + 1
);

/*
 * Useful functions
 */
// Planck's law
[[nodiscard]] constexpr float blackbody(float wavelength, float temp) noexcept {
  if (temp <= 0.0f) return 0.0f;

  constexpr float c = 299792458.0f; // Speed of light
  // Magic physics constants
  constexpr float h = 6.62606957e-34f;
  constexpr float kb = 1.3806488e-23f;

  const float l = wavelength * 1e-9f;
  const float emission =
    (2.0f * h * c * c)
    / (std::pow(l, 5.0f) * (std::exp((h * c) / (l * kb * temp)) - 1.0f));
  return emission;
}

/**
 * Spectrum samples
 */
class SpectrumSample {
public:
  constexpr explicit SpectrumSample(float s = 0.0f) noexcept: m_data() {
    m_data.fill(s);
  }

  constexpr explicit SpectrumSample(
    const std::array<float, nSpectrumSamples>& a
  ) noexcept: m_data(a) {}

  [[nodiscard]] constexpr float operator[](size_t i) const noexcept {
    return m_data[i];
  }

  [[nodiscard]] constexpr float& operator[](size_t i) noexcept {
    return m_data[i];
  }

  [[nodiscard]] constexpr float sum() const noexcept {
    float sum = 0.0f;
    for (float s: m_data) sum += s;
    return sum;
  }

  [[nodiscard]] constexpr float average() const noexcept {
    return sum() / float(nSpectrumSamples);
  }

  [[nodiscard]] constexpr bool operator==(const SpectrumSample& rhs) const noexcept {
    return m_data == rhs.m_data;
  }

  [[nodiscard]] constexpr bool operator!=(const SpectrumSample& rhs) const noexcept {
    return !(*this == rhs);
  }

  [[nodiscard]] constexpr auto operator+(const SpectrumSample& rhs) const noexcept {
    return SpectrumSample(*this) += rhs;
  }

  [[nodiscard]] constexpr SpectrumSample& operator+=(const SpectrumSample& rhs) noexcept {
    for (uint32_t i = 0; i < nSpectrumSamples; i++)
      m_data[i] += rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator-(const SpectrumSample& rhs) const noexcept {
    return SpectrumSample(*this) -= rhs;
  }

  [[nodiscard]] constexpr SpectrumSample& operator-=(const SpectrumSample& rhs) noexcept {
    for (uint32_t i = 0; i < nSpectrumSamples; i++)
      m_data[i] -= rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator*(const SpectrumSample& rhs) const noexcept {
    return SpectrumSample(*this) *= rhs;
  }

  [[nodiscard]] constexpr SpectrumSample& operator*=(const SpectrumSample& rhs) noexcept {
    for (uint32_t i = 0; i < nSpectrumSamples; i++)
      m_data[i] *= rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator*(float rhs) const noexcept {
    return SpectrumSample(*this) *= rhs;
  }

  [[nodiscard]] constexpr SpectrumSample& operator*=(float rhs) noexcept {
    for (uint32_t i = 0; i < nSpectrumSamples; i++)
      m_data[i] *= rhs;
    return *this;
  }

  [[nodiscard]] constexpr auto operator/(const SpectrumSample& rhs) const noexcept {
    return SpectrumSample(*this) /= rhs;
  }

  [[nodiscard]] constexpr SpectrumSample& operator/=(const SpectrumSample& rhs) noexcept {
    for (uint32_t i = 0; i < nSpectrumSamples; i++)
      m_data[i] /= rhs.m_data[i];
    return *this;
  }

  [[nodiscard]] constexpr auto operator/(float rhs) const noexcept {
    return SpectrumSample(*this) /= rhs;
  }

  [[nodiscard]] constexpr SpectrumSample& operator/=(float rhs) noexcept {
    for (uint32_t i = 0; i < nSpectrumSamples; i++)
      m_data[i] /= rhs;
    return *this;
  }

private:
  std::array<float, nSpectrumSamples> m_data;
};

// Additional operator
[[nodiscard]] constexpr auto operator*(
  float lhs,
  const SpectrumSample& rhs
) noexcept {
  return rhs * lhs;
}

/**
 * Wavelengths object: holds a number of sampled wavelengths and their PDF
 * values, used for sampling spectra.
 */
class Wavelengths {
public:
  [[nodiscard]] static constexpr Wavelengths sampleUniform(
    float u,
    float min = minWavelength,
    float max = maxWavelength
  ) {
    Wavelengths wls;
    const float range = max - min;
    const float delta = range / nSpectrumSamples;
    float wl = min + range * u;
    for (uint32_t i = 0; i < nSpectrumSamples; i++) {
      wls[i] = wl;
      wl += delta;
      if (wl > max) wl -= range;

      wls.m_pdfValues[i] = 1.0f / range;
    }
    return wls;
  }

  [[nodiscard]] constexpr bool operator==(const Wavelengths& rhs) const noexcept {
    return m_wavelengths == rhs.m_wavelengths && m_pdfValues == rhs.m_pdfValues;
  }

  [[nodiscard]] constexpr bool operator!=(const Wavelengths& rhs) const noexcept {
    return !(*this == rhs);
  }

  [[nodiscard]] constexpr float operator[](size_t i) const noexcept {
    return m_wavelengths[i];
  }

  [[nodiscard]] constexpr float& operator[](size_t i) noexcept {
    return m_wavelengths[i];
  }

  [[nodiscard]] constexpr SpectrumSample pdf() const noexcept {
    return SpectrumSample(m_pdfValues);
  }

private:
  constexpr Wavelengths() noexcept: m_wavelengths(), m_pdfValues() {
    m_wavelengths.fill(0.0f);
    m_pdfValues.fill(0.0f);
  }

  std::array<float, nSpectrumSamples> m_wavelengths, m_pdfValues;
};

/**
 * Spectrum interface: defines a function in the visible light spectrum that
 * can be sampled
 */
class Spectrum {
public:
  [[nodiscard]] virtual constexpr float operator()(float wl) const = 0;

  [[nodiscard]] virtual constexpr float maxValue() const = 0;

  [[nodiscard]] constexpr SpectrumSample sample(const Wavelengths& wls) const {
    SpectrumSample sample;
    for (uint32_t i = 0; i < nSpectrumSamples; i++)
      sample[i] = (*this)(wls[i]);
    return sample;
  }

  [[nodiscard]] static float innerProduct(
    const Spectrum& f,
    const Spectrum& g
  ) noexcept {
    float integral = 0.0f;
    for (float wl = minWavelength; wl <= maxWavelength; wl++) { // NOLINT(*)
      integral += f(wl) + g(wl);
    }
    return integral;
  }
};

class ConstantSpectrum : public Spectrum {
public:
  explicit ConstantSpectrum(float c) noexcept: c(c) {}

  [[nodiscard]] constexpr float operator()(float wl) const override { return c; }

  [[nodiscard]] constexpr float maxValue() const override { return c; };

private:
  float c;
};

class DenseSpectrum : public Spectrum {
public:
  explicit DenseSpectrum(const Spectrum& other) noexcept: m_data() {
    for (size_t i = 0; i < nWavelengths; i++) {
      const float wl = float(i) + minWavelength;
      m_data[i] = other(wl);
    }
  }

  [[nodiscard]] constexpr float operator()(float wl) const override {
    const auto i = size_t(wl - minWavelength);
    return m_data[i];
  }

  [[nodiscard]] constexpr float maxValue() const override {
    return *std::max_element(m_data.begin(), m_data.end());
  }


private:
  std::array<float, nWavelengths> m_data;
};

class PiecewiseLinearSpectrum : public Spectrum {
public:
  explicit PiecewiseLinearSpectrum(
    const float* wavelengths,
    const float* values,
    size_t n
  ) {
    m_wls.reserve(n);
    m_values.reserve(n);

    for (size_t i = 0; i < n; i++) {
      m_wls.push_back(wavelengths[i]);
      m_values.push_back(values[i]);
    }
  }

  template<std::ranges::random_access_range T, std::ranges::random_access_range U>
  explicit PiecewiseLinearSpectrum(
    T wavelengths,
    U values
  ) {
    m_wls.reserve(wavelengths.size());
    m_values.reserve(values.size());

    for (float wl: wavelengths) m_wls.push_back(wl);
    for (float val: values) m_values.push_back(val);
  }

  [[nodiscard]] constexpr float operator()(float wl) const override {
    if (m_wls.empty() || wl < m_wls.front() || wl > m_wls.back()) return 0.0f;

    size_t i = 1;
    while (m_wls[i] <= wl && i < m_wls.size() - 1) i++;
    const float t = (wl - m_wls[i - 1]) / (m_wls[i] - m_wls[i - 1]);
    return lerp(m_values[i - 1], m_values[i], t);
  }

  [[nodiscard]] constexpr float maxValue() const override {
    return *std::max_element(m_values.begin(), m_values.end());
  }

private:
  std::vector<float> m_wls, m_values;
};

class BlackbodySpectrum : public Spectrum {
public:
  constexpr explicit BlackbodySpectrum(float temp) noexcept
    : m_temp(temp) {
    const float maxEnergyWavelength = 2.8977721e-3f / temp;
    m_normFactor = 1.0f / blackbody(maxEnergyWavelength * 1e9f, temp);
  }

  [[nodiscard]] constexpr float operator()(float wl) const override {
    return blackbody(wl, m_temp) * m_normFactor;
  }

  [[nodiscard]] constexpr float maxValue() const override {
    return 1.0f;
  }

private:
  float m_temp;
  float m_normFactor;
};

class RGBSigmoidPolynomial : public Spectrum {
public:
  constexpr RGBSigmoidPolynomial(float c0, float c1, float c2) noexcept
    : c0(c0), c1(c1), c2(c2) {}

  [[nodiscard]] constexpr float operator()(float wl) const override {
    return sigmoid(evalPolynomial(wl, c2, c1, c0));
  }

  [[nodiscard]] constexpr float maxValue() const override {
    float res = std::max((*this)(minWavelength), (*this)(maxWavelength));

    // Find the maximum of the polynomial by its derivative
    float wlExt = -c1 / (2 * c0); // 0 = 2x * c0 + c1
    if (wlExt >= minWavelength && wlExt <= maxWavelength)
      res = std::max(res, (*this)(wlExt));

    return res;
  }

private:
  float c0, c1, c2;

  [[nodiscard]] constexpr static float sigmoid(float x) {
    if (isinf(x)) return x > 0.0f ? 1.0f : 0.0f;
    return 0.5f + x / (2.0f * std::sqrt(1.0f + x * x));
  }
};

namespace spectra {

static constexpr float CIE_Y_integral = 106.856895f;

void init();

DenseSpectrum D(float temp, float scale = 0.01f);

const DenseSpectrum& X();

const DenseSpectrum& Y();

const DenseSpectrum& Z();

}

}

#endif //YART_SPECTRUM_HPP
