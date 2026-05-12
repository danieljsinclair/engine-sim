#ifndef ATG_ENGINE_SIM_UTILITIES_H
#define ATG_ENGINE_SIM_UTILITIES_H

#include "types.h"

real_t modularDistance(real_t a, real_t b, real_t mod = 1.0);
real_t positiveMod(real_t x, real_t mod);
real_t erfApproximation(real_t x);

template <typename t>
inline t clamp(t x, t x0 = static_cast<t>(0.0), t x1 = static_cast<t>(1.0)) {
    if (x <= x0) return x0;
    else if (x >= x1) return x1;
    else return x;
}

#endif /* ATG_ENGINE_SIM_UTILITIES_H */
