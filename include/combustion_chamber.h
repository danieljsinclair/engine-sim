#ifndef ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H
#define ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H

#include "scs.h"

#include <string>

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

        // Reset all three chamber-local gas systems (cylinder charge, intake
        // runner+manifold, exhaust runner+primary) back to the initialize()
        // condition: 1 atm / 25 C ambient mix, zero momentum. The measured
        // exhaust-port flow is decided between m_system and
        // m_exhaustRunnerAndPrimary (updateGas flow chain) — a reset that
        // skips the chamber-local systems does not clear the standing state
        // that drives sustained reversion.
        void resetGasState();
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
        // Tuning for the exhaust afterfire ("pop on overrun") effect.
        //
        // These are PHYSICAL constants, not a schedule. There is deliberately no
        // throttle gate, no RPM floor, no probability roll, no cooldown and no
        // per-decel event cap: a pop happens when — and only when — unburnt fuel
        // sitting in a hot exhaust runner completes its auto-ignition induction
        // period before the gas is swept out of the pipe. Overrun satisfies that
        // by itself (weak flow => long residence), which is why no explicit
        // "are we coasting?" test is needed or wanted.
        struct AfterfireParameters {
            bool enabled = false;

            // --- Stage 1: how much RAW fuel reaches the pipe ------------------
            // Manifold pressure below which the trapped charge is so diluted by
            // residual exhaust gas that in-cylinder combustion breaks down and
            // fuel is pumped out raw. This is the CAUSE of overrun afterfire and
            // the only thing that distinguishes overrun from full power here:
            // measured on this engine, MAP is 98 kPa at WOT, 52 kPa at part
            // throttle and idle, but collapses to 25-29 kPa on a high-RPM coast.
            // Every other candidate (runner temperature, fuel fraction, oxygen
            // fraction, residence time) is at least as favourable at WIDE-OPEN
            // THROTTLE as on overrun, so none of them can discriminate.
            double misfireManifoldPressure = units::pressure(0.4, units::atm);

            // --- Stage 2: auto-ignition of that raw fuel ----------------------
            // Arrhenius induction time, written about a reference point:
            //     tau(T) = ignitionDelayRefS * exp(activationTempK * (1/T - 1/refTempK))
            // The exponential is what makes a hot pipe light off in milliseconds
            // and a cooling one take the best part of a second, so pops thin out
            // and stop by themselves as the exhaust cools during a long coast.
            double ignitionDelayRefS = 0.02;    // induction time at refTempK
            double activationTempK   = 8000.0;  // Arrhenius activation temperature
            double refTempK          = 1000.0;  // reference point for the above

            // Hard floor: below the auto-ignition temperature of gasoline vapour
            // no amount of residence time will light the mixture.
            double autoIgnitionTempK = 750.0;

            // Raw fuel must actually collect in the pipe, and there must be
            // oxygen left to burn it with, before anything can ignite.
            double minRawFuelFraction    = 0.0005;
            double minOxygenMoleFraction = 0.01;

            // Trim on released combustion energy. 1.0 = the fuel's own energy
            // density (the physical value); exists only for audibility tuning.
            double energyScale = 1.0;

            // Throttle position below which afterfire is allowed.
            // Speed control s where s=1 = wide open, s=0 = shut.
            // This prevents firing at steady part-throttle where MAP may also be low.
            // Overrun is physically: throttle CLOSED + high RPM + falling RPM.
            double throttleCutoff = 0.1;

            // Custom impulse response for afterfire pops. Empty = use the engine's
            // default exhaust impulse response, so the pop is coloured by the real
            // exhaust geometry (a real backfire is the pipe reacting to a pressure
            // spike, not a foreign sample). When set, the afterfire event injects
            // THIS waveform as its pressure spike instead — used to give pops a
            // distinct, library-sourced "crack". May be a single file path OR a
            // glob; if a glob, one matching file is chosen at random per pop so a
            // bank of cylinders does not fire the same sample every time.
            //
            // afterfireWavPath carries the raw config string (single path OR glob).
            // afterfireWavPaths carries the bridge-resolved, glob-expanded list of
            // absolute candidate files. The chamber loads from afterfireWavPaths;
            // if that list is empty the engine's default exhaust impulse response is
            // used (a real backfire = the pipe reacting to a pressure spike).
            std::string afterfireWavPath;
            std::vector<std::string> afterfireWavPaths;

            bool diagnostics = false;
        };

        // Observable state. The skipped* counters name the PHYSICAL precondition
        // that was missing, so a silent engine can be diagnosed ("never got hot
        // enough" vs "hot but always scavenged before it could light") instead of
        // guessed at. maxIgnitionProgress is the key one: it says how close the
        // induction integral ever came to completing.
        struct AfterfireDiagnostics {
            int eventCount = 0;
            int skippedTooCold = 0;     // runner below auto-ignition temperature
            int skippedNoFuel = 0;      // no raw (misfired) fuel in the runner
            int skippedNoOxygen = 0;    // not enough O2 left to react with
            int skippedThrottle = 0;     // pedal above cutoff: not overrun
            int skippedNotReady = 0;    // reactive, but induction period incomplete
            int misfireCycles = 0;      // cycles that pumped raw fuel into the pipe
            double maxIgnitionProgress = 0.0;
            double maxRunnerTempK = 0.0;
            double maxRawFuelFraction = 0.0;
            double minManifoldPressure = 0.0;
            double lastEventRpm = 0.0;
            double lastEventThrottle = 0.0;
            double lastEventPeakPressure = 0.0;
            double lastEventEnergyReleased = 0.0;
            double lastEventRunnerTempK = 0.0;
        };

        void setAfterfireParameters(const AfterfireParameters &parameters);
        AfterfireParameters getAfterfireParameters() const;
        void enableAfterfire(bool enabled);
        bool isAfterfireEnabled() const;

        // Advance the auto-ignition state of this chamber's exhaust runner by dt
        // and light it off if the induction period completed. This is the whole
        // effect: called once per simulation step, it reads the runner's real
        // temperature and mixture and decides nothing on a timer.
        // `throttle` is the pedal (Engine::getSpeedControl): 1 = wide open,
        // 0 = shut. It is used only for the overrun gate — above throttleCutoff
        // the pedal is down so this is not overrun and no pop can occur.
        // Returns true if a pop occurred this step.
        bool updateAfterfire(double dt, double throttle = 0.0);

        // Fraction of the auto-ignition induction period completed by the fuel
        // currently in the runner. 1.0 means it lights.
        double getAfterfireIgnitionProgress() const;

        double getLastAfterfirePeakPressure() const;
        double getLastAfterfireEnergyReleased() const;
        int getAfterfireEventCount() const;
        double getLastAfterfireRpm() const;
        AfterfireDiagnostics getAfterfireDiagnostics() const;
        void resetAfterfireDiagnostics();

        // Audible pulse window: true while a fired pop is still being mixed into
        // the exhaust signal.
        bool hasAfterfirePulse() const;
        void tickAfterfirePulse(double dtMs);

        // Custom pop sample, if a WAV was configured. After a pop, the chamber
        // picks one candidate (random for a glob) and exposes it here; the
        // simulator swaps it into the exhaust channel's convolution impulse
        // response. Returns nullptr when no custom WAV is configured (the engine's
        // default exhaust IR is then used, as a real backfire colours the pipe).
        bool hasAfterfireWavChoice() const;
        const int16_t* afterfireWavChoice(size_t& outCount) const;
        // Sample rate (Hz) of the most recently chosen pop sample, for resampling
        // to the synthesizer's audio rate at mix time. Returns the synth audio rate
        // (a no-op resample) when no custom WAV is configured.
        int afterfireWavChoiceSampleRate() const;

    protected:
        // Stage 1. Track raw fuel pumped into the runner by a misfiring cycle,
        // and let what is already there be scavenged away by exhaust flow.
        void updateRawExhaustFuel(double dt);
        // Induction time for the runner's current thermodynamic state, in seconds.
        // Returns infinity when the mixture cannot react at all. Not const: it
        // records WHICH precondition was missing, so a non-popping engine is
        // diagnosable by reason instead of by guesswork.
        double afterfireIgnitionDelay();
        // Burn the raw fuel pocket in the runner. Returns true if fuel actually
        // burned (an O2- or fuel-starved pipe reacts nothing and stays silent).
        bool igniteExhaustCharge(double runnerTempK);
        void recordAfterfireEvent(double peakPressure, double energyReleased, double runnerTempK);

        AfterfireParameters m_afterfireParameters;
        AfterfireDiagnostics m_afterfireDiagnostics;
        // Moles of RAW (never-burned) fuel currently sitting in this runner.
        // Tracked separately from the GasSystem's own p_fuel because that value
        // is dominated by ordinary combustion products and is actually HIGHER at
        // full throttle than on overrun — it cannot distinguish the two.
        double m_afterfireRawFuel_n = 0.0;
        // Induction integral: sum of dt/tau(T) over the raw fuel's residence in
        // the runner. Reaching 1.0 IS the ignition event. Reset whenever the raw
        // fuel is scavenged away, which is what makes the effect intermittent
        // without any timer.
        double m_afterfireIgnitionProgress = 0.0;
        // Edge detector for per-cycle fuel delivery: raw fuel is credited once
        // per exhaust event, not continuously every step.
        bool m_afterfireExhaustOpen = false;
        double m_afterfirePulseRemainingMs = 0.0;
        double m_afterfirePulseDurationMs = 60.0;
        bool m_afterfireEnabled = false;
        double m_lastAfterfirePeakPressure = 0.0;
        double m_lastAfterfireEnergyReleased = 0.0;
        int m_afterfireEventCount = 0;

        // Custom pop samples, loaded from afterfireWavPaths at configure time.
        // Empty => use the engine's default exhaust impulse response. Each entry
        // is one candidate WAV's full sample buffer (mono, 16-bit, at the WAV's own
        // sample rate). No 10000-sample IR clip is applied: the pop is mixed
        // one-shot at its native length instead of being stretched into a
        // convolution impulse response, so a 0.7 s crack plays as a 0.7 s crack.
        std::vector<std::vector<int16_t>> m_afterfireWavSamples;
        // Per-candidate sample rate (Hz), captured at load so the simulator can
        // resample the pop to the synthesizer's audio rate (44100) before mixing.
        std::vector<int> m_afterfireWavSampleRates;
        // Index into m_afterfireWavSamples for the most recent pop (-1 = none).
        int m_afterfireWavChosen = -1;
        size_t m_afterfireWavChosenCount = 0;
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
