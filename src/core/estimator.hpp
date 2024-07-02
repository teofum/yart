#ifndef YART_ESTIMATOR_HPP
#define YART_ESTIMATOR_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

class Estimator {
public:
  virtual constexpr void addSample(const float3& sample) noexcept = 0;

  [[nodiscard]] virtual constexpr float3 getValue() noexcept = 0;

protected:
  [[nodiscard]] static constexpr float luma(const float3& val) noexcept {
    constexpr float3 lw(0.2126f, 0.7152f, 0.0722f);
    return dot(val, lw);
  }
};

class MeanEstimator : public Estimator {
public:
  constexpr explicit MeanEstimator(size_t nSamples) noexcept
    : m_samples(nSamples) {}

  constexpr void addSample(const float3& sample) noexcept override {
    if (hasnan(sample)) return;
    m_acc += sample;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    return m_acc / float(m_samples);
  }

private:
  size_t m_samples;
  float3 m_acc;
};

class MoNEstimator : public Estimator {
public:
  constexpr explicit MoNEstimator(int32_t n, int32_t mMax) noexcept
    : m(min(mMax, max(1, 1 + 2 * ((n - 5) / 10)))),
      m_acc(m), m_smp(m) {}

  constexpr void addSample(const float3& sample) noexcept override {
    if (!hasnan(sample)) {
      m_acc[m_idx] += sample;
      m_smp[m_idx]++;
    }
    m_idx = (m_idx + 1) % m;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    if (m == 1) return m_acc[0] / float(m_smp[0]);

    // Divide buckets by number of samples
    for (size_t i = 0; i < m; i++) m_acc[i] /= float(m_smp[i]);

    // Sort means by total value
    std::sort(
      m_acc.begin(),
      m_acc.end(),
      [](const float3& a, const float3& b) { return luma(a) < luma(b); }
    );

    // Return the median of means
    return m_acc[m / 2];
  }

private:
  size_t m, m_idx = 0;
  std::vector<float3> m_acc;
  std::vector<size_t> m_smp;
};

class GMoNbEstimator : public Estimator {
public:
  constexpr explicit GMoNbEstimator(int32_t n, int32_t mMax) noexcept
    : m(min(mMax, max(1, 1 + 2 * ((n - 5) / 10)))),
      m_acc(m), m_smp(m) {}

  constexpr void addSample(const float3& sample) noexcept override {
    if (!hasnan(sample)) {
      m_acc[m_idx] += sample;
      m_smp[m_idx]++;
    }
    m_idx = (m_idx + 1) % m;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    if (m == 1) return m_acc[0] / float(m_smp[0]);

    // Divide buckets by number of samples
    for (size_t i = 0; i < m; i++) m_acc[i] /= float(m_smp[i]);

    // Sort means by total value
    std::sort(
      m_acc.begin(),
      m_acc.end(),
      [](const float3& a, const float3& b) { return luma(a) < luma(b); }
    );

    // Calculate the Gini function
    float3 sum, weightedSum;
    for (size_t i = 0; i < m; i++) {
      sum += m_acc[i];
      weightedSum += (i + 1) * m_acc[i];
    }
    float G = (2.0f * luma(weightedSum)) / (float(m) * luma(sum)) -
              float(m + 1) / float(m);

    // If G is below the threshold return the mean
    if (G <= 0.25f) return sum / float(m);

    // Return the median of means
    return m_acc[m / 2];
  }

private:
  size_t m, m_idx = 0;
  std::vector<float3> m_acc;
  std::vector<size_t> m_smp;
};

class GMoNEstimator : public Estimator {
public:
  constexpr explicit GMoNEstimator(int32_t n, int32_t mMax) noexcept
    : m(min(mMax, max(1, 1 + 2 * ((n - 5) / 10)))),
      m_acc(m), m_smp(m) {}

  constexpr void addSample(const float3& sample) noexcept override {
    if (!hasnan(sample)) {
      m_acc[m_idx] += sample;
      m_smp[m_idx]++;
    }
    m_idx = (m_idx + 1) % m;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    if (m == 1) return m_acc[0] / float(m_smp[0]);

    // Divide buckets by number of samples
    for (size_t i = 0; i < m; i++) m_acc[i] /= float(m_smp[i]);

    // Sort means by total value
    std::sort(
      m_acc.begin(),
      m_acc.end(),
      [](const float3& a, const float3& b) { return luma(a) < luma(b); }
    );

    // Calculate the Gini function
    float3 sum, weightedSum;
    for (size_t i = 0; i < m; i++) {
      sum += m_acc[i];
      weightedSum += (i + 1) * m_acc[i];
    }
    float G = (2.0f * luma(weightedSum)) / (float(m) * luma(sum)) -
              float(m + 1) / float(m);
    if (G > 1.0f) G = 1.0f;

    auto c = size_t(G * float(m / 2)); // NOLINT(*-integer-division)
    sum = float3(0.0f);
    for (size_t i = c; i < m - c; i++) sum += m_acc[i];

    return sum / float(m - 2 * c);
  }

private:
  size_t m, m_idx = 0;
  std::vector<float3> m_acc;
  std::vector<size_t> m_smp;
};

}

#endif //YART_ESTIMATOR_HPP
