#ifndef ATG_ENGINE_SIM_VEHICLE_SPEED_CONSTRAINT_H
#define ATG_ENGINE_SIM_VEHICLE_SPEED_CONSTRAINT_H

// Spike-A: a "vehicle dynamometer" — a velocity-bias constraint that drives the
// vehicle-mass body (body-1 of the transmission) to a target LINEAR road speed,
// instead of the crankshaft (body-0, which the regular Dynamometer pins).
//
// The vehicle's kinetic energy is stored as 0.5*I*v_theta^2 on the rotating
// virtual mass (see Vehicle::getSpeed). To target a linear speed we convert:
//     v_theta = speed * sqrt(m / I)
// using the body's CURRENT m and I (I is remapped on every gear change, so it is
// read live each frame in calculate()).
//
// Symmetric limits let the constraint both drive the body up to speed AND hold it
// there (unlike VehicleDragConstraint, which is one-sided).

#include "scs.h"

namespace atg_scs { struct RigidBody; }

class VehicleSpeedConstraint : public atg_scs::Constraint {
    public:
        VehicleSpeedConstraint();
        virtual ~VehicleSpeedConstraint();

        void connectVehicleMass(atg_scs::RigidBody *vehicleMass);
        virtual void calculate(Output *output, atg_scs::SystemState *state);

        // Target linear road speed in m/s. The constraint converts this to the
        // virtual angular velocity of the vehicle-mass body.
        double m_targetLinearSpeed;

        double m_ks;
        double m_kd;
        double m_maxForce;   // symmetric force limit (Newtons, in linear terms; converted internally)

        bool m_enabled;
};

#endif /* ATG_ENGINE_SIM_VEHICLE_SPEED_CONSTRAINT_H */
