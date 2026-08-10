#ifndef ATG_ENGINE_SIM_TRANSMISSION_H
#define ATG_ENGINE_SIM_TRANSMISSION_H

#include "vehicle.h"
#include "engine.h"
#include "scs.h"
#include "torque_converter.h"

#include <memory>

class Transmission {
    public:
        struct Parameters {
            int GearCount;
            const double *GearRatios;
            double MaxClutchTorque;
            // Optional fluid coupling. Null (the default) keeps the historic
            // friction-clutch-only drivetrain, so existing callers are
            // unaffected. When supplied, the converter is added ALONGSIDE the
            // friction clutch, which stays on its real bodies for gear shifts.
            const TorqueConverter::Parameters *TorqueConverterParams = nullptr;
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
        inline bool hasTorqueConverter() const { return m_torqueConverter != nullptr; }
        inline TorqueConverter *getTorqueConverter() const { return m_torqueConverter.get(); }

    protected:
        atg_scs::ClutchConstraint m_clutchConstraint;
        std::unique_ptr<TorqueConverter> m_torqueConverter;
        atg_scs::RigidBody *m_rotatingMass;
        Vehicle *m_vehicle;

        int m_gear;
        int m_newGear;
        int m_gearCount;
        double *m_gearRatios;
        double m_maxClutchTorque;
        double m_clutchPressure;
};

#endif /* ATG_ENGINE_SIM_TRANSMISSION_H */
