#ifndef ATG_ENGINE_SIM_THROTTLE_H
#define ATG_ENGINE_SIM_THROTTLE_H

#include "part.h"
#include "types.h"

class Engine;
class Throttle {
public:
    Throttle();
    virtual ~Throttle();

    virtual void setSpeedControl(real_t s);
    virtual void update(real_t dt, Engine *engine);

    inline real_t getSpeedControl() const { return m_speedControl; }

protected:
    real_t m_speedControl;
};

#endif /* ATG_ENGINE_SIM_THROTTLE_H */
