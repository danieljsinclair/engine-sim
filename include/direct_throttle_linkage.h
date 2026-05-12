#ifndef ATG_ENGINE_SIM_DIRECT_THROTTLE_LINKAGE_H
#define ATG_ENGINE_SIM_DIRECT_THROTTLE_LINKAGE_H

#include "throttle.h"
#include "types.h"

class DirectThrottleLinkage : public Throttle {
public:
    struct Parameters {
        real_t gamma;
    };

public:
    DirectThrottleLinkage();
    virtual ~DirectThrottleLinkage();

    void initialize(const Parameters &params);

    virtual void setSpeedControl(real_t s);
    virtual void update(real_t dt, Engine *engine);

protected:
    real_t m_gamma;
    real_t m_throttlePosition;
};

#endif /* ATG_ENGINE_SIM_DIRECT_THROTTLE_LINKAGE_H */
