#ifndef ATG_ENGINE_SIM_GOVERNOR_H
#define ATG_ENGINE_SIM_GOVERNOR_H

#include "throttle.h"
#include "types.h"

class Governor : public Throttle {
public:
    struct Parameters {
        real_t minSpeed;
        real_t maxSpeed;
        real_t minVelocity;
        real_t maxVelocity;
        real_t k_s;
        real_t k_d;
        real_t gamma;
    };

public:
    Governor();
    virtual ~Governor();

    void initialize(const Parameters &params);

    virtual void setSpeedControl(real_t s);
    virtual void update(real_t dt, Engine *engine);

protected:
    real_t m_minSpeed;
    real_t m_maxSpeed;
    real_t m_minVelocity;
    real_t m_maxVelocity;
    real_t m_k_s;
    real_t m_k_d;
    real_t m_gamma;

    real_t m_targetSpeed;

    real_t m_currentThrottle;
    real_t m_velocity;
};

#endif /* ATG_ENGINE_SIM_GOVERNOR_H */
