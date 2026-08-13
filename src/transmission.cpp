#include "../include/transmission.h"

#include "../include/units.h"
#include "../include/torque_converter.h"

#include <cmath>
#include <cstdio>

Transmission::Transmission() {
    m_gear = -1;
    m_newGear = -1;
    m_gearCount = 0;
    m_gearRatios = nullptr;
    m_maxClutchTorque = units::torque(1000.0, units::ft_lb);
    m_rotatingMass = nullptr;
    m_vehicle = nullptr;
    m_clutchPressure = 0.0;
    m_inputTorque = 0.0;
    m_inputTorqueGenerator.setOwner(this);
}

Transmission::~Transmission() {
    if (m_gearRatios != nullptr) {
        delete[] m_gearRatios;
    }

    m_gearRatios = nullptr;
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
    static int dbgFrames = 0;
    if (dbgFrames < 30) {
        std::fprintf(stderr, "[TC-DBG] update#%d hasTC=%d gear=%d clutchP=%.3f capScale=%.3f\n",
                     dbgFrames, (m_torqueConverter != nullptr ? 1 : 0), m_gear,
                     m_clutchPressure,
                     (m_torqueConverter != nullptr ? m_torqueConverter->getCapacityScale() : -1.0));
        ++dbgFrames;
    }
    if (m_torqueConverter != nullptr) {
        // Fluid-coupling torque converter installed: it is the SOLE coupling
        // path. The friction clutch is held OPEN (zero torque) so it cannot
        // rigidly lock the engine to the (CSV-pinned) wheels and stall at
        // standstill. The converter's own K * N^2 pump law loads the engine
        // proportionally: at WOT/standstill the engine settles at its STALL SPEED
        // (turbine pinned, TR ~ 2.0, combustion torque balanced by K*N^2*TR) — a
        // few thousand rpm, well below the free-rev bar — and at cruise the
        // lockup clutch couples 1:1. No free-rev (always loaded), no stall (the
        // fluid slips). The converter capacity follows the clutch pedal so a
        // shift can briefly open the driveline; neutral opens it fully.
        m_clutchConstraint.m_minTorque = 0;
        m_clutchConstraint.m_maxTorque = 0;
        m_torqueConverter->setCapacityScale(m_gear == -1 ? 0.0 : m_clutchPressure);
    }
    else if (m_gear == -1) {
        m_clutchConstraint.m_minTorque = 0;
        m_clutchConstraint.m_maxTorque = 0;
    }
    else {
        m_clutchConstraint.m_minTorque = -m_maxClutchTorque * m_clutchPressure;
        m_clutchConstraint.m_maxTorque = m_maxClutchTorque * m_clutchPressure;
    }

    // NOTE: MATCH-mode driving torque is NOT applied here. update() runs AFTER
    // m_system->process() each frame (simulator.cpp:110 then :114), so anything
    // written here would miss the solve. The recorded input torque is applied
    // by the TransmissionInputTorqueGenerator (a ForceGenerator) during
    // processForces(), accumulating into SystemState::t[rotatingMass] so the ODE
    // integrates it before the clutch constraint solves — see apply() below.
    (void)dt;
}

void Transmission::setInputTorque(double nm) {
    m_inputTorque = nm;
}

void Transmission::addToSystem(
    atg_scs::RigidBodySystem *system,
    atg_scs::RigidBody *rotatingMass,
    Vehicle *vehicle,
    Engine *engine)
{
    m_rotatingMass = rotatingMass;
    m_vehicle = vehicle;

    m_clutchConstraint.setBody1(&engine->getOutputCrankshaft()->m_body);
    m_clutchConstraint.setBody2(m_rotatingMass);

    system->addConstraint(&m_clutchConstraint);

    // Fluid-coupling torque converter: added in parallel with the friction
    // clutch, spanning the same two bodies (engine crankshaft <-> rotating
    // mass). The clutch keeps its bodies and is only used to open the driveline
    // during a shift; the converter supplies the fluid (pump/turbine) torque
    // path directly in Nm via the SCS solver.
    if (m_torqueConverter != nullptr) {
        m_torqueConverter->setImpeller(&engine->getOutputCrankshaft()->m_body);
        m_torqueConverter->setTurbine(m_rotatingMass);
        system->addConstraint(m_torqueConverter.get());
    }

    // MATCH mode: register the input-torque generator so the recorded torque is
    // accumulated onto the rotating mass during processForces each frame.
    system->addForceGenerator(&m_inputTorqueGenerator);
}

void TransmissionInputTorqueGenerator::apply(atg_scs::SystemState* system) {
    if (m_owner == nullptr) return;

    const double torque = m_owner->effectiveInputTorque();
    if (torque == 0.0) return;  // no-op for FREE/PIN and neutral/idle frames

    const atg_scs::RigidBody* body = m_owner->inputTorqueBody();
    if (body != nullptr) {
        system->t[body->index] += torque;
    }
}

void Transmission::attachTorqueConverter(atg_scs::RigidBodySystem *system,
                                         Engine *engine,
                                         const TorqueConverter::Parameters &params) {
    if (m_torqueConverter != nullptr) {
        return;  // already installed — idempotent
    }
    if (system == nullptr || engine == nullptr || m_rotatingMass == nullptr) {
        return;  // not yet wired — caller must wire the transmission first
    }

    m_torqueConverter = std::make_unique<TorqueConverter>();
    m_torqueConverter->initialize(params);
    m_torqueConverter->setImpeller(&engine->getOutputCrankshaft()->m_body);
    m_torqueConverter->setTurbine(m_rotatingMass);
    system->addConstraint(m_torqueConverter.get());
}

void Transmission::ensureTorqueConverter(const TorqueConverter::Parameters &params) {
    if (m_torqueConverter != nullptr) {
        return;  // already installed — idempotent
    }
    // Create the object only; addToSystem() wires the bodies + adds the
    // constraint to the rigid-body system at the safe wiring time (before the
    // solver is ever stepped). Never add a constraint to an already-initialized
    // live system — the SCS Gauss-Seidel solver pre-sizes its per-constraint
    // buffers at initialize(), so a post-hoc addConstraint corrupts memory.
    m_torqueConverter = std::make_unique<TorqueConverter>();
    m_torqueConverter->initialize(params);
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
