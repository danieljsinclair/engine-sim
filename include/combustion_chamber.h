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

            real_t StartingPressure;
            real_t StartingTemperature;
            real_t CrankcasePressure;
        };

        struct FlameEvent {
            real_t lit_n = 0;
            real_t total_n = 0;
            real_t percentageLit = 0;
            real_t efficiency = 1.0;
            real_t flameSpeed = 0.0;

            real_t lastVolume = 0.0;
            real_t travel_x = 0.0;
            real_t travel_y = 0.0;
            GasSystem::Mix globalMix;
        };

        struct FrictionModelParams {
            real_t frictionCoeff = 0.06;
            real_t breakawayFriction = units::force(50, units::N);
            real_t breakawayFrictionVelocity = units::distance(0.1, units::m);
            real_t viscousFrictionCoefficient = units::force(20, units::N);
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

        real_t getFrictionForce() const;
        real_t getVolume() const;
        real_t pistonSpeed() const;
        real_t calculateMeanPistonSpeed() const;
        real_t calculateFiringPressure() const;

        bool isLit() const { return m_lit; }
        bool popLitLastFrame();

        void ignite();
        void update(real_t dt);
        void flow(real_t dt);

        real_t lastEventAfr() const;

        real_t getLastIterationExhaustFlow() const { return m_exhaustFlow; }

        void resetLastTimestepExhaustFlow() { m_lastTimestepTotalExhaustFlow = 0; }
        real_t getLastTimestepExhaustFlow() const { return m_lastTimestepTotalExhaustFlow; }

        void resetLastTimestepIntakeFlow() { m_lastTimestepTotalIntakeFlow = 0; }
        real_t getLastTimestepIntakeFlow() const { return m_lastTimestepTotalIntakeFlow; }

        Function *m_meanPistonSpeedToTurbulence;
        GasSystem m_system;
        GasSystem m_intakeRunnerAndManifold;
        GasSystem m_exhaustRunnerAndPrimary;
        FlameEvent m_flameEvent;
        bool m_lit;

        FrictionModelParams m_frictionModel;

        real_t m_peakTemperature;
        real_t m_nBurntFuel;

    protected:
        real_t calculateFrictionForce(real_t v) const;
        void updateCycleStates();

        real_t m_intakeFlowRate;
        real_t m_exhaustFlowRate;

        real_t m_manifoldToRunnerFlowRate;
        real_t m_primaryToCollectorFlowRate;
        real_t m_cylinderCrossSectionSurfaceArea;
        real_t m_cylinderWidthApproximation;

        real_t m_lastTimestepTotalExhaustFlow;
        real_t m_lastTimestepTotalIntakeFlow;
        real_t m_exhaustFlow;

        real_t m_crankcasePressure;

        real_t *m_pressure;
        real_t *m_pistonSpeed;
        static constexpr int StateSamples = 256;

        bool m_litLastFrame;

        Piston *m_piston;
        CylinderHead *m_head;
        Engine *m_engine;
        Fuel *m_fuel;
};

#endif /* ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H */
