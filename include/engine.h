#ifndef ATG_ENGINE_SIM_ENGINE_H
#define ATG_ENGINE_SIM_ENGINE_H

#include "part.h"

#include "piston.h"
#include "connecting_rod.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "cylinder_head.h"
#include "exhaust_system.h"
#include "ignition_module.h"
#include "intake.h"
#include "combustion_chamber.h"
#include "units.h"
#include "throttle.h"

#include <string>

class Simulator;
class Vehicle;
class Transmission;
class Engine : public Part {
    public:
        struct Parameters {
            int cylinderBanks;
            int cylinderCount;
            int crankshaftCount;
            int exhaustSystemCount;
            int intakeCount;

            std::string name;

            double starterTorque = units::torque(90.0, units::ft_lb);
            double starterSpeed = units::rpm(200);
            double redline = units::rpm(6500);
            double dynoMinSpeed = units::rpm(1000);
            double dynoMaxSpeed = units::rpm(6500);
            double dynoHoldStep = units::rpm(100);

            Throttle *throttle;

            double initialSimulationFrequency;
            double initialHighFrequencyGain;
            double initialNoise;
            double initialJitter;
        };

    public:
        Engine();
        virtual ~Engine();

        void initialize(const Parameters &params);
        virtual void destroy();

        std::string getName() const { return m_name; }

        virtual Crankshaft *getOutputCrankshaft() const;
        virtual void setSpeedControl(real_t s);
        virtual real_t getSpeedControl();
        virtual void setThrottle(real_t throttle);
        virtual real_t getThrottle() const;
        virtual real_t getThrottlePlateAngle() const;
        virtual void calculateDisplacement();
        real_t getDisplacement() const { return m_displacement; }
        virtual real_t getIntakeFlowRate() const;
        virtual void update(real_t dt);

        virtual real_t getManifoldPressure() const;
        virtual real_t getIntakeAfr() const;
        virtual real_t getExhaustO2() const;
        virtual real_t getRpm() const;
        virtual real_t getSpeed() const;
        virtual bool isSpinningCw() const;

        virtual void resetFuelConsumption();
        virtual real_t getTotalFuelMassConsumed() const;
        real_t getTotalVolumeFuelConsumed() const;

        inline real_t getStarterTorque() const { return m_starterTorque; }
        inline real_t getStarterSpeed() const { return m_starterSpeed; }
        inline real_t getRedline() const { return m_redline; }
        inline real_t getDynoMinSpeed() const { return m_dynoMinSpeed; }
        inline real_t getDynoMaxSpeed() const { return m_dynoMaxSpeed; }
        inline real_t getDynoHoldStep() const { return m_dynoHoldStep; }

        int getCylinderBankCount() const { return m_cylinderBankCount; }
        int getCylinderCount() const { return m_cylinderCount; }
        int getCrankshaftCount() const { return m_crankshaftCount; }
        int getExhaustSystemCount() const { return m_exhaustSystemCount; }
        int getIntakeCount() const { return m_intakeCount; }
        int getMaxDepth() const;

        Crankshaft *getCrankshaft(int i) const { return &m_crankshafts[i]; }
        CylinderBank *getCylinderBank(int i) const { return &m_cylinderBanks[i]; }
        CylinderHead *getHead(int i) const { return &m_heads[i]; }
        Piston *getPiston(int i) const { return &m_pistons[i]; }
        ConnectingRod *getConnectingRod(int i) const { return &m_connectingRods[i]; }
        IgnitionModule *getIgnitionModule() { return &m_ignitionModule; }
        ExhaustSystem *getExhaustSystem(int i) const { return &m_exhaustSystems[i]; }
        Intake *getIntake(int i) const { return &m_intakes[i]; }
        CombustionChamber *getChamber(int i) const { return &m_combustionChambers[i]; }
        Fuel *getFuel() { return &m_fuel; }

        real_t getSimulationFrequency() const { return m_initialSimulationFrequency; }
        real_t getInitialHighFrequencyGain() const { return m_initialHighFrequencyGain; }
        real_t getInitialNoise() const { return m_initialNoise; }
        real_t getInitialJitter() const { return m_initialJitter; }

        virtual Simulator *createSimulator(Vehicle *vehicle, Transmission *transmission);

    protected:
        std::string m_name;

        Crankshaft *m_crankshafts;
        int m_crankshaftCount;

        CylinderBank *m_cylinderBanks;
        CylinderHead *m_heads;
        int m_cylinderBankCount;

        Piston *m_pistons;
        ConnectingRod *m_connectingRods;
        CombustionChamber *m_combustionChambers;
        int m_cylinderCount;

        real_t m_starterTorque;
        real_t m_starterSpeed;
        real_t m_redline;
        real_t m_dynoMinSpeed;
        real_t m_dynoMaxSpeed;
        real_t m_dynoHoldStep;

        real_t m_initialSimulationFrequency;
        real_t m_initialHighFrequencyGain;
        real_t m_initialNoise;
        real_t m_initialJitter;

        ExhaustSystem *m_exhaustSystems;
        int m_exhaustSystemCount;

        Intake *m_intakes;
        int m_intakeCount;

        IgnitionModule m_ignitionModule;
        Fuel m_fuel;

        Throttle *m_throttle;

        real_t m_throttleValue;
        real_t m_displacement;
};

#endif /* ATG_ENGINE_SIM_ENGINE_H */
