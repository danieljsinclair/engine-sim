#include "../include/transmission.h"

#include "../include/units.h"

#include <cmath>

Transmission::Transmission() {
    m_gear = -1;
    m_newGear = -1;
    m_gearCount = 0;
    m_gearRatios = nullptr;
    m_maxClutchTorque = units::torque(1000.0, units::ft_lb);
    m_torqueConverter = nullptr;
    m_rotatingMass = nullptr;
    m_vehicle = nullptr;
    m_clutchPressure = 0.0;
}

Transmission::~Transmission() {
    if (m_gearRatios != nullptr) {
        delete[] m_gearRatios;
    }
    if (m_torqueConverter != nullptr) {
        delete m_torqueConverter;
    }
    m_gearRatios = nullptr;
    m_torqueConverter = nullptr;
}

void Transmission::initialize(const Parameters &params) {
    m_gearCount = params.GearCount;
    m_maxClutchTorque = params.MaxClutchTorque;
    m_gearRatios = new double[params.GearCount];
    memcpy(m_gearRatios, params.GearRatios, sizeof(double) * m_gearCount);

    // Create torque converter if parameters provided
    if (params.TorqueConverterParams != nullptr) {
        m_torqueConverter = new atg_scs::TorqueConverter();
        m_torqueConverter->initialize(*params.TorqueConverterParams);
    }
}

void Transmission::update(double dt) {
    if (m_gear == -1) {
        m_clutchConstraint.m_minTorque = 0;
        m_clutchConstraint.m_maxTorque = 0;
    }
    else {
        m_clutchConstraint.m_minTorque = -m_maxClutchTorque * m_clutchPressure;
        m_clutchConstraint.m_maxTorque = m_maxClutchTorque * m_clutchPressure;
    }

    // Update torque converter: set capacity from clutch pressure and update RPM
    if (m_torqueConverter != nullptr) {
        // TC capacity is proportional to clutch pressure.
        // At pressure=0: TC is open (no torque transfer, engine free-revs)
        // At pressure=1: TC at full capacity (K × N² law applies)
        // The TC's own calculate() handles the fluid coupling physics.
        constexpr double kMaxTcCapacity = 600.0;  // Nm at full pressure
        const double tcCapacity = kMaxTcCapacity * m_clutchPressure;
        m_torqueConverter->setMaxInputTorque(tcCapacity);

        // Update RPM from constraint bodies
        if (m_torqueConverter->m_bodies[0] != nullptr) {
            const double radPerSecToRpm = 30.0 / 3.14159265358979;
            double inputRpm = std::abs(m_torqueConverter->m_bodies[0]->v_theta) * radPerSecToRpm;
            double outputRpm = std::abs(m_torqueConverter->m_bodies[1]->v_theta) * radPerSecToRpm;
            m_torqueConverter->updateRpm(inputRpm, outputRpm);
        }
    }
}

void Transmission::addToSystem(
    atg_scs::RigidBodySystem *system,
    atg_scs::RigidBody *rotatingMass,
    Vehicle *vehicle,
    Engine *engine)
{
    m_rotatingMass = rotatingMass;
    m_vehicle = vehicle;

    if (m_torqueConverter != nullptr) {
        // TC replaces the friction clutch as the coupling between engine and
        // drivetrain. The TC's fluid coupling allows slip, preventing the
        // engine from redlining at light throttle.
        m_torqueConverter->m_bodies[0] = &engine->getOutputCrankshaft()->m_body;
        m_torqueConverter->m_bodies[1] = m_rotatingMass;
        system->addConstraint(m_torqueConverter);

        // Friction clutch remains for gear shifts (open during shift, closed after)
        m_clutchConstraint.setBody1(m_rotatingMass);
        m_clutchConstraint.setBody2(m_rotatingMass);  // Same body — acts as a brake
        system->addConstraint(&m_clutchConstraint);
    } else {
        // Legacy path: direct friction clutch coupling (unchanged)
        m_clutchConstraint.setBody1(&engine->getOutputCrankshaft()->m_body);
        m_clutchConstraint.setBody2(m_rotatingMass);
        system->addConstraint(&m_clutchConstraint);
    }
}

void Transmission::changeGear(int newGear) {
    if (newGear < -1 || newGear >= m_gearCount) return;
    else if (newGear != -1) {
        const double m_car = m_vehicle->getMass();
        const double gear_ratio = m_gearRatios[newGear];
        const double diff_ratio = m_vehicle->getDiffRatio();
        const double tire_radius = m_vehicle->getTireRadius();
        const double f = tire_radius / (diff_ratio * gear_ratio);

        const double new_I = m_car * f * f;
        const double E_r =
            0.5 * m_rotatingMass->I * m_rotatingMass->v_theta * m_rotatingMass->v_theta;
        const double new_v_theta = m_rotatingMass->v_theta < 0
            ? -std::sqrt(E_r * 2 / new_I)
            : std::sqrt(E_r * 2 / new_I);

        m_rotatingMass->I = new_I;
        m_rotatingMass->p_x = m_rotatingMass->p_y = 0;
        m_rotatingMass->m = m_car;
        m_rotatingMass->v_theta = new_v_theta;
    }

    m_gear = newGear;
}
