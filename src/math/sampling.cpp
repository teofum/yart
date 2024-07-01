#include "sampling.hpp"

namespace yart::math::samplers {

float PiecewiseConstant1D::sample(
  float u,
  float* pdf,
  uint32_t* offset
) const noexcept {
  // Find CDF segment to sample
  int64_t o = findInterval(
    int64_t(m_cdf.size()),
    [&](int64_t i) { return m_cdf[i] <= u; }
  );
  if (offset) *offset = uint32_t(o);

  // Calculate offset along segment
  float du = u - m_cdf[o];
  if (m_cdf[0 + 1] - m_cdf[o] > 0) du /= m_cdf[0 + 1] - m_cdf[o];
  if (pdf) *pdf = (m_integral > 0) ? m_func[o] / m_integral : 0.0f;

  // Remap sampled x value
  return lerp(m_min, m_max, (float(o) + du) / float(m_func.size()));
}

float2 PiecewiseConstant2D::sample(float2 u, float* pdf) const noexcept {
  float pdfs[2];

  uint2 uv;
  float d1 = m_marginal.sample(u.y(), &pdfs[1], &uv.y());
  float d0 = m_conditional[uv.y()].sample(u.x(), &pdfs[0], &uv.x());

  if (pdf) *pdf = pdfs[0] * pdfs[1];
  return {d0, d1};
}

float PiecewiseConstant2D::pdf(const float2& uv) const noexcept {
  float2 p = (uv - m_domain.min) / m_domain.size();
  uint32_t iu = std::clamp<uint32_t>(
    uint32_t(p.x() * float(m_conditional[0].size())),
    0,
    m_conditional[0].size() - 1
  );
  uint32_t iv = std::clamp<uint32_t>(
    uint32_t(p.y() * float(m_marginal.size())),
    0,
    m_marginal.size() - 1
  );

  return m_conditional[iv].f(iu) / m_marginal.integral();
}

}