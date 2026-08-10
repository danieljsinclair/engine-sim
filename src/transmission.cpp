#include "../include/transmission.h"

#include "../include/units.h"

#include <cmath>

Transmission::Transmission() {
    m_gear = -1;
    m_newGear = -1;
    m_gearCount = 0;
    m_gearRatios = nullptr;
    m_maxClutchTorque = units::torque(1000.0, units::ft_lb);
    m_rotatingMass = nullptr;
    m_vehicle = nullptr;
    m_clutchPressure = 0.0;
}

Transmission::~Transmission() {
    if (m_gearRatios != nullptr) {
        delete[] m_gearRatios;
    }

    m_gearRatios = nullptr;

    // m_torqueConverter is a unique_ptr — released automatically.
}

void Transmission::initialize(const Parameters &params) {
    m_gearCount = params.GearCount;
    m_maxClutchTorque = params.MaxClutchTorque;
    m_gearRatios = new double[params.GearCount];
    memcpy(m_gearRatios, params.GearRatios, sizeof(double) * m_gearCount);

    if (params.TorqueConverterParams != nullptr) {
        m_torqueConverter = std::make_unique<TorqueConverter>();
        m_torqueConverter->initialize(*params.TorqueConverterParams);
    }
}

void Transmission::update(double dt) {
    (void)dt;

    const bool inNeutral = (m_gear == -1);

    if (inNeutral) {
        m_clutchConstraint.m_minTorque = 0;
        m_clutchConstraint.m_maxTorque = 0;
    }
    else {
        m_clutchConstraint.m_minTorque = -m_maxClutchTorque * m_clutchPressure;
        m_clutchConstraint.m_maxTorque = m_maxClutchTorque * m_clutchPressure;
    }

    if (m_torqueConverter != nullptr) {
        // The converter's rated capacity comes from its own K * N^2 law; the
        // driver's clutch pedal only gates how much of that is available, and
        // neutral opens it entirely so the engine can free-rev. The converter
        // reads its own body speeds inside calculate(), so nothing else needs
        // pushing in from here.
        m_torqueConverter->setCapacityScale(inNeutral ? 0.0 : m_clutchPressure);
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

    atg_scs::RigidBody *crankshaft = &engine->getOutputCrankshaft()->m_body;

    // The friction clutch keeps its real bodies in BOTH configurations — it is
    // what opens the driveline during a gear change. The converter is added
    // alongside it, spanning the same two bodies, so the pair act as parallel
    // torque paths: the clutch caps the rigid path, the converter supplies the
    // fluid path with its own capacity law.
    m_clutchConstraint.setBody1(crankshaft);
    m_clutchConstraint.setBody2(m_rotatingMass);
    system->addConstraint(&m_clutchConstraint);

    if (m_torqueConverter != nullptr) {
        m_torqueConverter->setImpeller(crankshaft);
        m_torqueConverter->setTurbine(m_rotatingMass);
        system->addConstraint(m_torqueConverter.get());
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
