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
        // Called once per simulateStep_(), after the fluid substeps have updated
        // every runner's temperature and mixture. It only advances each chamber's
        // own auto-ignition chemistry — there is no cross-chamber scheduling,
        // decel window or pop spacing to own, because a pop is now a local
        // consequence of that runner's gas state (SRP: the chamber decides).
        void tickAfterfire(double dt);
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

        // Per-channel afterfire pop-rendering state.
        //
        // A fired pop MIXES a custom WAV onto its exhaust channel's audio output
        // (Synthesizer::triggerPop), alongside the engine's continuing exhaust
        // convolution — the channel's impulse response is never touched, so the
        // engine note is never interrupted. The synthesizer owns one
        // OneShotSampleMixer per channel (Synthesizer::m_popMixers); the
        // simulator's only bookkeeping here is the gain to mix each channel's pop
        // at (per exhaust channel, since a V-engine shares one channel across
        // several cylinders and we want the crack at a consistent level regardless
        // of which cylinder fired it).
        std::vector<float> m_customAfterfireGain;

        int m_fluidSimulationSteps;
};

#endif /* ATG_ENGINE_SIM_PISTON_ENGINE_SIMULATOR_H */
