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
    m_acc += sample;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    return m_acc / float(m_samples);
  }

private:
  size_t m_samples;
  float3 m_acc;
};

template<std::size_t M>
class MoNEstimator : public Estimator {
public:
  constexpr explicit MoNEstimator(size_t nSamples) noexcept
    : m_samples(nSamples) {
    m_acc.fill(float3());
  }

  constexpr void addSample(const float3& sample) noexcept override {
    m_acc[m_idx] += sample;
    m_idx = (m_idx + 1) % M;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    // Sort means by total value
    std::sort(
      m_acc.begin(),
      m_acc.end(),
      [](const float3& a, const float3& b) { return luma(a) - luma(b); }
    );

    // Return the median of means
    return m_acc[M / 2] / float(m_samples) * float(M);
  }

private:
  size_t m_samples, m_idx = 0;
  std::array<float3, M> m_acc;
};

template<std::size_t M>
class GMoNbEstimator : public Estimator {
public:
  constexpr explicit GMoNbEstimator(size_t nSamples) noexcept
    : m_samples(nSamples) {
    m_acc.fill(float3());
  }

  constexpr void addSample(const float3& sample) noexcept override {
    m_acc[m_idx] += sample;
    m_idx = (m_idx + 1) % M;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    // Sort means by total value
    std::sort(
      m_acc.begin(),
      m_acc.end(),
      [](const float3& a, const float3& b) { return luma(a) - luma(b); }
    );

    // Calculate the Gini function
    float3 sum, weightedSum;
    for (size_t i = 0; i < M; i++) {
      sum += m_acc[i];
      weightedSum += (i + 1) * m_acc[i];
    }
    float G = (2.0f * luma(weightedSum)) / (float(M) * luma(sum)) -
              float(M + 1) / float(M);

    // If G is below the threshold return the mean
    if (G <= 0.25f) return sum / float(m_samples);

    // Return the median of means
    return m_acc[M / 2] / float(m_samples) * float(M);
  }

private:
  size_t m_samples, m_idx = 0;
  std::array<float3, M> m_acc;
};

template<std::size_t M>
class GMoNEstimator : public Estimator {
public:
  constexpr explicit GMoNEstimator(size_t nSamples) noexcept
    : m_samples(nSamples) {
    m_acc.fill(float3());
  }

  constexpr void addSample(const float3& sample) noexcept override {
    m_acc[m_idx] += sample;
    m_idx = (m_idx + 1) % M;
  }

  [[nodiscard]] constexpr float3 getValue() noexcept override {
    // Sort means by total value
    std::sort(
      m_acc.begin(),
      m_acc.end(),
      [](const float3& a, const float3& b) { return luma(a) - luma(b); }
    );

    // Calculate the Gini function
    float3 sum, weightedSum;
    for (size_t i = 0; i < M; i++) {
      sum += m_acc[i];
      weightedSum += (i + 1) * m_acc[i];
    }
    float G = (2.0f * luma(weightedSum)) / (float(M) * luma(sum)) -
              float(M + 1) / float(M);

    auto c = size_t(G * float(M / 2));
    sum = float3(0.0f);
    for (size_t i = c; i < M - c; i++) sum += m_acc[i];

    return sum / float(m_samples) * float(M) / float(M - 2 * c);
  }

private:
  size_t m_samples, m_idx = 0;
  std::array<float3, M> m_acc;
};

}

#endif //YART_ESTIMATOR_HPP
