#include "../include/vehicle_speed_constraint.h"

#include <cmath>

VehicleSpeedConstraint::VehicleSpeedConstraint() : atg_scs::Constraint(1, 1) {
    m_targetLinearSpeed = 0.0;
    // PIN-mode gains. The prior m_ks=2 / m_maxForce=5000N was so soft the engine
    // (M156 ~19000N at the wheels in 1st) overpowered it on every hard accel and
    // the simulated speed ran 3-7 mph AHEAD of the CSV target (the mph-overshoot
    // failure: |sim-tgt| up to 11, %>2 ~20%). These gains tighten the velocity
    // bias so PIN actually PINS (sim tracks the CSV within ~2 mph under load)
    // while staying below the combustion force scale so the engine-sim still
    // loads against the constraint (the clutch still couples engine↔road, the
    // rpm/sound still emerges from physics, not from a hard speed playback).
    m_ks = 15.0;    // tighter: clamp WOT-stomp overshoot toward the CSV target
    m_kd = 2.0;     // heavier damping: keep the stiff spring stable at buffer dt
    m_maxForce = 25000.0;  // brake authority exceeds a 1st-gear WOT torque spike

    m_enabled = false;
}

VehicleSpeedConstraint::~VehicleSpeedConstraint() {
    /* void */
}

void VehicleSpeedConstraint::connectVehicleMass(atg_scs::RigidBody *vehicleMass) {
    m_bodies[0] = vehicleMass;
}

void VehicleSpeedConstraint::calculate(Output *output, atg_scs::SystemState *state) {
    (void)state;

    output->J[0][0] = 0;
    output->J[0][1] = 0;
    output->J[0][2] = 1;       // rotation axis of the vehicle-mass body

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->ks[0] = m_ks;
    output->kd[0] = m_kd;

    output->C[0] = 0;

    // Convert target linear speed (m/s) to the virtual angular velocity of the
    // vehicle-mass body using the SAME energy identity Vehicle::getSpeed inverts:
    //     0.5*I*v_theta^2 = 0.5*m*v^2   ->   v_theta = v * sqrt(m / I)
    // Read m and I LIVE — I is remapped on every gear change (transmission.cpp).
    const atg_scs::RigidBody *body = m_bodies[0];
    const double ratio = (body->I > 0.0 && body->m > 0.0)
        ? std::sqrt(body->m / body->I)
        : 0.0;
    const double targetVtheta = m_targetLinearSpeed * ratio;

    // Convert the symmetric force limit (Newtons, linear) into a virtual torque
    // limit using Vehicle's identity: torque_virtual = force * sqrt(I / m).
    const double invRatio = (ratio > 0.0) ? (1.0 / ratio) : 0.0;  // sqrt(I/m)
    const double maxTorque = m_maxForce * invRatio;

    // Symmetric limits — can both drive the body up AND brake it down. The dyno
    // uses asymmetric limits because it only ever resists; we want a true
    // speed-tracking constraint on the wheels.
    if (m_enabled) {
        output->limits[0][0] = -maxTorque;
        output->limits[0][1] = maxTorque;
    } else {
        output->limits[0][0] = 0.0;
        output->limits[0][1] = 0.0;
    }

    // Sign convention matches the dyno: v_bias sign follows the body's current
    // spin direction so the constraint always pulls toward |targetVtheta|.
    output->v_bias[0] = (body->v_theta < 0) ? targetVtheta : -targetVtheta;
    if (body->v_theta < 0.0 && targetVtheta == 0.0) {
        output->v_bias[0] = 0.0;
    }
}
