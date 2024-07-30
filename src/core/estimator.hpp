#ifndef YART_ESTIMATOR_HPP
#define YART_ESTIMATOR_HPP

#include <math/math.hpp>

namespace yart {
using namespace math;

/**
 * Base estimator abstract class, manages adding up individual samples to a
 * final pixel value.
 */
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

/**
 * Simple estimator using the mean of all samples.
 */
class MeanEstimator : public Estimator {
public:
  constexpr explicit MeanEstimator(size_t nSamples) noexcept
    : m_samples(nSamples) {}

  constexpr void addSample(const float3& sample) noexcept override {
    if (hasnan(sample)) return; // Discard samples with NaN so they don't ruin the pixel
    m_acc += sample;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    return m_acc / float(m_samples);
  }

private:
  size_t m_samples;
  float3 m_acc;
};

/**
 * Median of means estimator. Splits samples into a number of "buckets", each
 * taking the mean of its samples, and uses the median bucket. Helps eliminate
 * outliers (fireflies) at the cost of increased variance.
 */
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

/**
 * Binary GMoN estimator. Variation on median of means, uses the Gini function
 * to detect outliers and uses mean or median of means depending on it.
 */
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

/**
 * GMoN estimator. Uses the Gini function to define a number of buckets, then
 * takes the mean of those buckets. This estimator has most of the benefit of
 * median of means, and converges much faster.
 */
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

    // Use more buckets when the Gini function is lower (higher confidence)
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
