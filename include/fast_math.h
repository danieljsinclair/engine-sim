#ifndef ATG_ENGINE_SIM_FAST_MATH_H
#define ATG_ENGINE_SIM_FAST_MATH_H

#include "types.h"
#include "constants.h"
#include <cmath>

namespace fast_math {
    void initialize();

    real_t sin(real_t x);
    real_t cos(real_t x);
    real_t pow(real_t base, real_t exp);

    // Internal LUTs
    constexpr int TableSize = 2048;
    extern real_t SinTable[TableSize + 1];
}

#endif /* ATG_ENGINE_SIM_FAST_MATH_H */
