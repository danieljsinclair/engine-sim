#include "../include/utilities.h"

#include <cmath>

real_t modularDistance(real_t a0, real_t b0, real_t mod) {
    real_t a, b;
    if (a0 < b0) {
        a = a0;
        b = b0;
    }
    else {
        a = b0;
        b = a0;
    }

    return std::fmin(b - a, a + mod - b);
}

real_t positiveMod(real_t x, real_t mod) {
    if (x < 0) {
        x = std::ceil(-x / mod) * mod + x;
    }

    return std::fmod(x, mod);
}

real_t erfApproximation(real_t x) {
    const real_t a1 = 0.278393f;
    const real_t a2 = 0.230389f;
    const real_t a3 = 0.000972f;
    const real_t a4 = 0.078108f;

    const real_t x2 = x * x;
    const real_t x3 = x2 * x;
    const real_t x4 = x3 * x;

    const real_t q = 1 / (1 + a1 * x + a2 * x2 + a3 * x3 + a4 * x4);
    const real_t q2 = q * q;
    const real_t q4 = q2 * q2;

    return 1 - q4;
}
