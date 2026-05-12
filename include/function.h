#ifndef ATG_ENGINE_SIM_FUNCTION_H
#define ATG_ENGINE_SIM_FUNCTION_H

#include "gaussian_filter.h"

class Function {
    protected:
        static GaussianFilter *DefaultGaussianFilter;

    public:
        Function();
        virtual ~Function();

        void initialize(int size, real_t filterRadius, GaussianFilter *filter = nullptr);
        void resize(int newCapacity);
        void destroy();

        void setInputScale(real_t s) { m_inputScale = s; }
        void setOutputScale(real_t s) { m_outputScale = s; }
        void addSample(real_t x, real_t y);

        real_t sampleTriangle(real_t x) const;
        real_t sampleGaussian(real_t x) const;
        real_t triangle(real_t x) const;
        int closestSample(real_t x) const;

        bool isOrdered() const;

        void getDomain(real_t *x0, real_t *x1);
        void getRange(real_t *y0, real_t *y1);

    protected:
        real_t *m_x;
        real_t *m_y;

        real_t m_yMin;
        real_t m_yMax;
        real_t m_inputScale;
        real_t m_outputScale;

        real_t m_filterRadius;

        int m_capacity;
        int m_size;

        GaussianFilter *m_gaussianFilter;
};

#endif /* ATG_ENGINE_SIM_FUNCTION_H */
