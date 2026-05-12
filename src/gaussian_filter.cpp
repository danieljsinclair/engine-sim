#include "../include/gaussian_filter.h"

#include <cmath>

GaussianFilter::GaussianFilter() {
    m_cache = nullptr;

    m_cacheSteps = 0;
    m_radius = 0.0f;
    m_alpha = 0.0f;

    m_exp_s = 0.0f;
    m_inv_r = 0.0f;
}

GaussianFilter::~GaussianFilter() {
    if (m_cache != nullptr) delete[] m_cache;
}

void GaussianFilter::initialize(real_t alpha, real_t radius, int cacheSteps) {
    m_cacheSteps = cacheSteps;

    m_alpha = alpha;
    m_radius = radius;

    m_exp_s = std::exp(-m_alpha * m_radius * m_radius);
    m_inv_r = 1 / m_radius;

    generateCache();
}

real_t GaussianFilter::evaluate(real_t s) const {
    const int actualSteps = m_cacheSteps - 32;
    const real_t s_sample = actualSteps * std::abs(s) * m_inv_r;
    const real_t s0 = std::floor(s_sample);
    const real_t s1 = std::ceil(s_sample);
    const real_t d = s_sample - s0;

    return
        (1 - d) * m_cache[(int)s0]
        + d * m_cache[(int)s1];
}

real_t GaussianFilter::calculate(real_t s) const {
    return std::max(
            real_t(0.0),
            std::exp(-m_alpha * s * s) - m_exp_s);
}

void GaussianFilter::generateCache() {
    const int actualSteps = m_cacheSteps - 32;
    const real_t step = 1.0f / (real_t)actualSteps;

    m_cache = new real_t[m_cacheSteps];
    for (int i = 0; i <= actualSteps; ++i) {
        const real_t s = i * step * m_radius;
        m_cache[i] = calculate(s);
    }

    for (int i = actualSteps + 1; i < m_cacheSteps; ++i) {
        m_cache[i] = 0.0f;
    }
}
