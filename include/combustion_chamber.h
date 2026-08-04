#ifndef ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H
#define ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H

#include "scs.h"

#include "piston.h"
#include "gas_system.h"
#include "cylinder_head.h"
#include "units.h"
#include "fuel.h"

class Engine;
class CombustionChamber : public atg_scs::ForceGenerator {
    public:
        struct Parameters {
            Piston *Piston;
            CylinderHead *Head;
            Fuel *Fuel;
            Function *MeanPistonSpeedToTurbulence;

            double StartingPressure;
            double StartingTemperature;
            double CrankcasePressure;
        };

        struct FlameEvent {
            double lit_n = 0;
            double total_n = 0;
            double percentageLit = 0;
            double efficiency = 1.0;
            double flameSpeed = 0.0;

            double lastVolume = 0.0;
            double travel_x = 0.0;
            double travel_y = 0.0;
            GasSystem::Mix globalMix;
        };

        struct FrictionModelParams {
            double frictionCoeff = 0.06;
            double breakawayFriction = units::force(50, units::N);
            double breakawayFrictionVelocity = units::distance(0.1, units::m);
            double viscousFrictionCoefficient = units::force(20, units::N);
        };

    public:
        CombustionChamber();
        virtual ~CombustionChamber();

        void initialize(const Parameters &params);
        void destroy();
        void setEngine(Engine *engine) { m_engine = engine; }
        virtual void apply(atg_scs::SystemState *system);

        CylinderHead *getCylinderHead() const { return m_head; }
        Piston *getPiston() const { return m_piston; }

        double getFrictionForce() const;
        double getVolume() const;
        double pistonSpeed() const;
        double calculateMeanPistonSpeed() const;
        double calculateFiringPressure() const;

        bool isLit() const { return m_lit; }
        bool popLitLastFrame();

        void ignite();
        void update(double dt);
        void flow(double dt);

        double lastEventAfr() const;

        double getLastIterationExhaustFlow() const { return m_exhaustFlow; }

        double getCrankcasePressure() const { return m_crankcasePressure; }

        void resetLastTimestepExhaustFlow() { m_lastTimestepTotalExhaustFlow = 0; }
        double getLastTimestepExhaustFlow() const { return m_lastTimestepTotalExhaustFlow; }

        void resetLastTimestepIntakeFlow() { m_lastTimestepTotalIntakeFlow = 0; }
        double getLastTimestepIntakeFlow() const { return m_lastTimestepTotalIntakeFlow; }

        Function *m_meanPistonSpeedToTurbulence;
        GasSystem m_system;
        GasSystem m_intakeRunnerAndManifold;
        GasSystem m_exhaustRunnerAndPrimary;
        FlameEvent m_flameEvent;
        bool m_lit;

        FrictionModelParams m_frictionModel;

        double m_peakTemperature;
        double m_nBurntFuel;

#ifdef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    public:
        // Tuning for the exhaust afterfire ("pop on overrun") effect. All gating
        // thresholds live here so the firing decision stays data-driven.
        struct AfterfireParameters {
            bool enabled = false;
            double intensity = 0.18;
            double cooldownMs = 360.0;
            double throttleCutoff = 0.04;
            double rpmMin = 3000.0;
            double fuelFraction = 0.0018;
            double probability = 0.012;
            double decelWindowMs = 1200.0;
            int maxEventsPerDecel = 5;
            double rpmFallThreshold = 120.0;    // rpm drop per step that counts as overrun
            double globalPopIntervalMs = 500.0; // min sim-time between any two pops
            bool diagnostics = false;
        };

        // Observable counters. eventCount is the value the acceptance test asserts on;
        // the skipped* counters exist so a non-firing engine can be diagnosed by
        // reason rather than by guesswork.
        struct AfterfireDiagnostics {
            int eventCount = 0;
            int skippedCooldown = 0;
            int skippedLowRpm = 0;
            int skippedThrottle = 0;
            int skippedProbability = 0;
            int skippedMaxEvents = 0;
            int skippedCrankAngle = 0;
            int skippedNoOverrun = 0;
            int eventsInCurrentDecel = 0;
            double lastEventRpm = 0.0;
            double lastEventThrottle = 0.0;
            double lastEventPeakPressure = 0.0;
            double lastEventEnergyReleased = 0.0;
        };

        void setAfterfireParameters(const AfterfireParameters &parameters);
        AfterfireParameters getAfterfireParameters() const;
        void enableAfterfire(bool enabled);
        bool isAfterfireEnabled() const;

        // Force a pop, bypassing overrun detection (diagnostics / direct drive).
        void triggerAfterfire(double intensity);
        // Gated trigger: returns true only if a pop was actually generated.
        bool triggerAfterfire(double intensity, double throttle, double rpm);

        double getLastAfterfirePeakPressure() const;
        double getLastAfterfireEnergyReleased() const;
        int getAfterfireEventCount() const;
        double getLastAfterfireRpm() const;
        AfterfireDiagnostics getAfterfireDiagnostics() const;
        void resetAfterfireDiagnostics();

        // Per-chamber cooldown in SIMULATION time (not wall-clock), advanced by the
        // owning simulator once per step so results stay deterministic.
        double getAfterfireCooldownRemainingMs() const;
        void setAfterfireCooldownRemainingMs(double ms);
        void tickAfterfireCooldown(double dtMs);

        // Evaluate gating and fire if eligible. Called once per sim step per chamber
        // by PistonEngineSimulator::tickAfterfire. Returns true if a pop fired.
        bool tickAfterfire(double throttle, double rpm, int globalEventsInDecel);

        // Audible pulse window: true while a fired pop is still being mixed into
        // the exhaust signal.
        bool hasAfterfirePulse() const;
        void tickAfterfirePulse(double dtMs);

    protected:
        bool shouldTriggerAfterfire(double throttle, double rpm, int globalEventsInDecel);
        void recordAfterfireEvent(double throttle, double rpm, double peakPressure, double energyReleased);

        AfterfireParameters m_afterfireParameters;
        AfterfireDiagnostics m_afterfireDiagnostics;
        double m_afterfireCooldownRemainingMs = 0.0;
        double m_afterfireLastRpm = 0.0;
        double m_afterfireDecelElapsedMs = 0.0;
        double m_afterfirePulseRemainingMs = 0.0;
        double m_afterfirePulseDurationMs = 60.0;
        bool m_afterfireInDecel = false;
        bool m_afterfireEnabled = false;
        double m_lastAfterfirePeakPressure = 0.0;
        double m_lastAfterfireEnergyReleased = 0.0;
        int m_afterfireEventCount = 0;
#endif /* ATG_ENGINE_SIM_AFTERFIRE_SPIKE */

    protected:
        double calculateFrictionForce(double v) const;
        void updateCycleStates();

        double m_intakeFlowRate;
        double m_exhaustFlowRate;

        double m_manifoldToRunnerFlowRate;
        double m_primaryToCollectorFlowRate;
        double m_cylinderCrossSectionSurfaceArea;
        double m_cylinderWidthApproximation;

        double m_lastTimestepTotalExhaustFlow;
        double m_lastTimestepTotalIntakeFlow;
        double m_exhaustFlow;

        double m_crankcasePressure;

        double *m_pressure;
        double *m_pistonSpeed;
        static constexpr int StateSamples = 256;

        bool m_litLastFrame;

        Piston *m_piston;
        CylinderHead *m_head;
        Engine *m_engine;
        Fuel *m_fuel;
};

#endif /* ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H */
