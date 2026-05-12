#ifndef ATG_ENGINE_SIM_CRANKSHAFT_H
#define ATG_ENGINE_SIM_CRANKSHAFT_H

#include "part.h"
#include "types.h"

class Crankshaft : public Part {
    public:
        struct Parameters {
            double mass;
            double flywheelMass;
            double momentOfInertia;
            double crankThrow;
            double pos_x = 0;
            double pos_y = 0;
            double tdc = 0;
            double frictionTorque = 0;
            int rodJournals;
        };

    public:
        Crankshaft();
        virtual ~Crankshaft();

        void initialize(const Parameters &params);
        virtual void destroy();
        inline int getRodJournalCount() const { return m_rodJournalCount; }
        void setRodJournalAngle(int i, real_t angle);
        void getRodJournalPositionLocal(int i, real_t *x, real_t *y);
        void getRodJournalPositionGlobal(int i, real_t *x, real_t *y);
        real_t getRodJournalAngle(int i) { return m_rodJournalAngles[i]; }

        void resetAngle();

        real_t getAngle() const;
        real_t getCycleAngle(real_t offset = 0.0);

        inline real_t getTdc() const { return m_tdc; }
        inline real_t getThrow() const { return m_throw; }
        inline real_t getMass() const { return m_m; }
        inline real_t getMomentOfInertia() const { return m_I; }
        inline real_t getFlywheelMass() const { return m_flywheelMass; }
        inline real_t getPosX() const { return m_p_x; }
        inline real_t getPosY() const { return m_p_y; }
        inline real_t getFrictionTorque() const { return m_frictionTorque; }

    protected:
        real_t *m_rodJournalAngles;
        int m_rodJournalCount;

        real_t m_tdc;
        real_t m_throw;
        real_t m_m;
        real_t m_I;
        real_t m_flywheelMass;
        real_t m_p_x;
        real_t m_p_y;
        real_t m_frictionTorque;
};

#endif /* ATG_ENGINE_SIM_CRANKSHAFT_H */
