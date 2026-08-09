#ifndef ATG_ENGINE_SIM_TRANSMISSION_H
#define ATG_ENGINE_SIM_TRANSMISSION_H

#include "vehicle.h"
#include "engine.h"
#include "scs.h"

class Transmission;

// TransmissionInputTorqueGenerator - MATCH-mode driving torque on the rotating
// mass. Mirrors how CombustionChamber applies torque: a ForceGenerator that
// accumulates into SystemState::t[body] during processForces(), so the rigid-
// body ODE integrates it (dω = T/I·dt) BEFORE the clutch constraint solves.
// Engine RPM then EMERGES from the clutch coupling (engine combustion torque vs
// the now-driven rotating-mass load) instead of being set by fiat. The torque
// flows input-shaft -> gearbox (ratio-scaled reflected inertia) -> diff ->
// wheels, so gearbox ratios + shifting affect road speed.
class TransmissionInputTorqueGenerator : public atg_scs::ForceGenerator {
public:
    void setOwner(Transmission* owner) { m_owner = owner; }
    void apply(atg_scs::SystemState* system) override;

private:
    Transmission* m_owner = nullptr;
};

class Transmission {
    public:
        struct Parameters {
            int GearCount;
            const double *GearRatios;
            double MaxClutchTorque;
        };

    public:
        Transmission();
        ~Transmission();

        void initialize(const Parameters &params);
        void update(double dt);
        void addToSystem(
            atg_scs::RigidBodySystem *system,
            atg_scs::RigidBody *rotatingMass,
            Vehicle *vehicle,
            Engine *engine);
        void changeGear(int newGear);

        // MATCH mode: inject recorded drivetrain torque (Nm) at the transmission
        // input / rotating-mass side, BEFORE the gearbox. The rotating-mass body's
        // inertia is already gear-ratio-scaled vehicle mass, so torque flows
        // clutch -> gearbox -> diff -> wheels through the existing solver path.
        void setInputTorque(double nm);
        // Torque actually applied this frame: the recorded input torque, but only
        // while a forward gear is engaged (neutral/open gearbox passes no torque
        // to the wheels). Read by the input-torque ForceGenerator during solve.
        inline double effectiveInputTorque() const {
            return (m_gear >= 0) ? m_inputTorque : 0.0;
        }
        // The rotating-mass body the input torque is applied to (or null).
        inline const atg_scs::RigidBody* inputTorqueBody() const { return m_rotatingMass; }
        inline int getGear() const { return m_gear; }
        inline double getGearRatio() const {
            return (m_gear >= 0 && m_gear < m_gearCount) ? m_gearRatios[m_gear] : 0.0;
        }
        inline void setClutchPressure(double pressure) { m_clutchPressure = pressure; }
        inline double getClutchPressure() const { return m_clutchPressure; }
        inline int getGearCount() const { return m_gearCount; }
        inline double getGearRatio(int i) const { return m_gearRatios[i]; }
        inline double getMaxClutchTorque() const { return m_maxClutchTorque; }
        inline const atg_scs::ClutchConstraint& getClutchConstraint() const { return m_clutchConstraint; }

    protected:
        atg_scs::ClutchConstraint m_clutchConstraint;
        atg_scs::RigidBody *m_rotatingMass;
        Vehicle *m_vehicle;

        int m_gear;
        int m_newGear;
        int m_gearCount;
        double *m_gearRatios;
        double m_maxClutchTorque;
        double m_clutchPressure;
        double m_inputTorque;
        TransmissionInputTorqueGenerator m_inputTorqueGenerator;
};

#endif /* ATG_ENGINE_SIM_TRANSMISSION_H */
