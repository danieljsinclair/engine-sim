#include "../include/fast_math.h"
#include "../include/constants.h"
#include <cmath>

namespace fast_math {
    real_t SinTable[TableSize + 1];

    void initialize() {
        for (int i = 0; i <= TableSize; ++i) {
            SinTable[i] = (real_t)std::sin((i / (real_t)TableSize) * 2.0 * (double)constants::pi);
        }
    }

    real_t sin(real_t x) {
        // Wrap to [0, 2pi]
        real_t angle = std::fmod(x, (real_t)(2.0 * constants::pi));
        if (angle < 0) angle += (real_t)(2.0 * constants::pi);

        const real_t index_f = (angle / (real_t)(2.0 * constants::pi)) * TableSize;
        const int index = (int)index_f;
        const real_t fraction = index_f - index;

        // Linear interpolation
        return SinTable[index] * (1.0f - fraction) + SinTable[index + 1] * fraction;
    }

    real_t cos(real_t x) {
        // cos(x) = sin(x + pi/2)
        return sin(x + (real_t)(constants::pi / 2.0));
    }

    real_t pow(real_t base, real_t exp) {
        // Fallback to std::pow for now, we can add a fast_pow later if needed
        // as it is much harder to LUT due to two variables.
        return std::pow(base, exp);
    }
}
