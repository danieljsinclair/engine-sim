#ifndef ATG_ENGINE_SIM_INTAKE_H
#define ATG_ENGINE_SIM_INTAKE_H

#include "part.h"

#include "gas_system.h"

class Intake : public Part {
    public:
        struct Parameters {
            // Plenum volume
            double volume;

            // Plenum dimensions
            double CrossSectionArea;

            // Input flow constant
            double InputFlowK;

            // Idle-circuit flow constant
            double IdleFlowK;

            // Flow rate from plenum to runner
            double RunnerFlowRate;

            // Molecular air fuel ratio (defaults to ideal for octane)
            double MolecularAfr = (25.0 / 2.0);

            // Throttle plate position at idle
            double IdleThrottlePlatePosition = 0.975;

            // Runner volume
            double RunnerLength = units::distance(4.0, units::inch);

            // Velocity decay factor
            double VelocityDecay = 0.5;
        };

    public:
        Intake();
        virtual ~Intake();

        void initialize(Parameters &params);
        virtual void destroy();

        void process(real_t dt);

        inline real_t getRunnerFlowRate() const { return m_runnerFlowRate; }
        inline real_t getThrottlePlatePosition() const { return m_idleThrottlePlatePosition * m_throttle; }
        inline real_t getRunnerLength() const { return m_runnerLength; }
        inline real_t getPlenumCrossSectionArea() const { return m_crossSectionArea; }
        inline real_t getVelocityDecay() const { return m_velocityDecay; }

        GasSystem m_system;
        real_t m_throttle;

        real_t m_flow;
        real_t m_flowRate;
        real_t m_totalFuelInjected;

    protected:
        real_t m_crossSectionArea;
        real_t m_inputFlowK;
        real_t m_idleFlowK;
        real_t m_runnerFlowRate;
        real_t m_molecularAfr;
        real_t m_idleThrottlePlatePosition;
        real_t m_runnerLength;
        real_t m_velocityDecay;

        GasSystem m_atmosphere;
};

#endif /* ATG_ENGINE_SIM_INTAKE_H */
