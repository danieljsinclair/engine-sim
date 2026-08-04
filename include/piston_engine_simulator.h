#ifndef ATG_ENGINE_SIM_PISTON_ENGINE_SIMULATOR_H
#define ATG_ENGINE_SIM_PISTON_ENGINE_SIMULATOR_H

#include "simulator.h"

#include "engine.h"
#include "transmission.h"
#include "combustion_chamber.h"
#include "vehicle.h"
#include "synthesizer.h"
#include "dynamometer.h"
#include "starter_motor.h"
#include "derivative_filter.h"
#include "vehicle_drag_constraint.h"
#include "delay_filter.h"

#include "scs.h"

class PistonEngineSimulator : public Simulator {
    public:
        PistonEngineSimulator();
        virtual ~PistonEngineSimulator() override;

        virtual void loadSimulation(Engine *engine, Vehicle *vehicle, Transmission *transmission) override;

        virtual double getTotalExhaustFlow() const override;
        void endFrame() override;
        virtual void destroy() override;

        void setFluidSimulationSteps(int steps) override { m_fluidSimulationSteps = steps; }
        int getFluidSimulationSteps() const override { return m_fluidSimulationSteps; }
        int getFluidSimulationFrequency() const { return m_fluidSimulationSteps * getSimulationFrequency(); }

        virtual double getAverageOutputSignal() const override;

        DerivativeFilter m_derivativeFilter;

#ifdef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    public:
        // Called once per simulateStep_() after the fluid substeps complete.
        // Owns the CROSS-CHAMBER concerns (decel window, global pop spacing) and
        // delegates the per-chamber decision to CombustionChamber::tickAfterfire,
        // so neither class duplicates the other's gating (SRP).
        void tickAfterfire(double dt, double throttle, double rpm);

    protected:
        // Global decel state is owned by the simulator instance — deliberately not
        // file-scope statics, so two simulators in one process cannot interfere.
        bool m_afterfireGlobalInDecel = false;
        double m_afterfireGlobalDecelElapsedMs = 0.0;
        double m_afterfireGlobalLastRpm = 0.0;
        int m_afterfireGlobalEventsInDecel = 0;
        // Minimum sim-time between any two pops across all chambers. Without this
        // every eligible chamber fires on the same step and the effect is a single
        // burst rather than a sequence of crackles.
        double m_afterfireGlobalPopCooldownMs = 0.0;
#endif /* ATG_ENGINE_SIM_AFTERFIRE_SPIKE */

    protected:
        virtual void simulateStep_() override;

    protected:
        void placeAndInitialize();
        void placeCylinder(int i);
        
    protected:
        virtual void writeToSynthesizer() override;

    protected:
        DelayFilter *m_delayFilters;

        atg_scs::FixedPositionConstraint *m_crankConstraints;
        atg_scs::ClutchConstraint *m_crankshaftLinks;
        atg_scs::RotationFrictionConstraint *m_crankshaftFrictionConstraints;
        atg_scs::LineConstraint *m_cylinderWallConstraints;
        atg_scs::LinkConstraint *m_linkConstraints;

        double *m_exhaustFlowStagingBuffer;

        int m_fluidSimulationSteps;
};

#endif /* ATG_ENGINE_SIM_PISTON_ENGINE_SIMULATOR_H */
