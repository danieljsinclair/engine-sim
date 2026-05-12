#ifndef ATG_ENGINE_SIM_GAUSSIAN_FILTER_H
#define ATG_ENGINE_SIM_GAUSSIAN_FILTER_H

#include "scs.h"

#include "crankshaft.h"
#include "types.h"

class GaussianFilter {
    public:
        GaussianFilter();
        ~GaussianFilter();

        void initialize(real_t alpha, real_t radius, int cacheSteps=1024);
        real_t evaluate(real_t s) const;

        real_t getRadius() const { return m_radius; }
        real_t getAlpha() const { return m_alpha; }

    protected:
        real_t calculate(real_t s) const;
        void generateCache();

    protected:
        real_t *m_cache;

        int m_cacheSteps;
        real_t m_radius;
        real_t m_alpha;

        real_t m_exp_s;
        real_t m_inv_r;
};

#endif /* ATG_ENGINE_SIM_GAUSSIAN_FILTER_H */
