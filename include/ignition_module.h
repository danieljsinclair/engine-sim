#ifndef ATG_ENGINE_SIM_IGNITION_MODULE_H
#define ATG_ENGINE_SIM_IGNITION_MODULE_H

#include "part.h"

#include "crankshaft.h"
#include "function.h"
#include "units.h"
#include "types.h"

class IgnitionModule : public Part {
    public:
        struct Parameters {
            int cylinderCount;
            Crankshaft *crankshaft;
            Function *timingCurve;
            real_t revLimit = units::rpm(6000.0);
            real_t limiterDuration = 0.5 * units::sec;
        };

        struct SparkPlug {
            real_t angle = 0;
            bool ignitionEvent = false;
            bool enabled = false;
        };

    public:
        IgnitionModule();
        virtual ~IgnitionModule();

        virtual void destroy();

        void initialize(const Parameters &params);
        void setFiringOrder(int cylinderIndex, real_t angle);
        void reset();
        void update(real_t dt);

        bool getIgnitionEvent(int index) const;
        void resetIgnitionEvents();

        real_t getTimingAdvance();

        bool m_enabled;

    protected:
        SparkPlug *getPlug(int i);

        Function *m_timingCurve;
        SparkPlug *m_plugs;
        Crankshaft *m_crankshaft;
        int m_cylinderCount;

        real_t m_lastCrankshaftAngle;
        real_t m_revLimit;
        real_t m_revLimitTimer;
        real_t m_limiterDuration;
};

#endif /* ATG_ENGINE_SIM_IGNITION_MODULE_H */
