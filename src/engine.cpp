#include "../include/engine.h"

#include "../include/constants.h"
#include "../include/units.h"
#include "../include/fuel.h"
#include "../include/piston_engine_simulator.h"

#include <cmath>
#include <assert.h>
#include <limits>

Engine::Engine() {
    m_name = "";

    m_crankshafts = nullptr;
    m_cylinderBanks = nullptr;
    m_heads = nullptr;
    m_pistons = nullptr;
    m_connectingRods = nullptr;
    m_exhaustSystems = nullptr;
    m_intakes = nullptr;
    m_combustionChambers = nullptr;

    m_crankshaftCount = 0;
    m_cylinderBankCount = 0;
    m_cylinderCount = 0;
    m_intakeCount = 0;
    m_exhaustSystemCount = 0;
    m_starterSpeed = 0;
    m_starterTorque = 0;
    m_dynoMinSpeed = 0;
    m_dynoMaxSpeed = 0;
    m_dynoHoldStep = 0;
    m_redline = 0;

    m_throttle = nullptr;
    m_throttleValue = 0.0;

    m_initialSimulationFrequency = 10000.0;
    m_initialHighFrequencyGain = 0.01;
    m_initialJitter = 0.5;
    m_initialNoise = 1.0;
}

Engine::~Engine() {
    assert(m_crankshafts == nullptr);
    assert(m_cylinderBanks == nullptr);
    assert(m_pistons == nullptr);
    assert(m_connectingRods == nullptr);
    assert(m_heads == nullptr);
    assert(m_exhaustSystems == nullptr);
    assert(m_intakes == nullptr);
    assert(m_combustionChambers == nullptr);
}

void Engine::initialize(const Parameters &params) {
    m_crankshaftCount = params.crankshaftCount;
    m_cylinderCount = params.cylinderCount;
    m_cylinderBankCount = params.cylinderBanks;
    m_exhaustSystemCount = params.exhaustSystemCount;
    m_intakeCount = params.intakeCount;
    m_starterTorque = params.starterTorque;
    m_starterSpeed = params.starterSpeed;
    m_dynoMinSpeed = params.dynoMinSpeed;
    m_dynoMaxSpeed = params.dynoMaxSpeed;
    m_dynoHoldStep = params.dynoHoldStep;
    m_redline = params.redline;
    m_name = params.name;
    m_throttle = params.throttle;
    m_initialHighFrequencyGain = params.initialHighFrequencyGain;
    m_initialSimulationFrequency = params.initialSimulationFrequency;
    m_initialJitter = params.initialJitter;
    m_initialNoise = params.initialNoise;

    m_crankshafts = new Crankshaft[m_crankshaftCount];
    m_cylinderBanks = new CylinderBank[m_cylinderBankCount];
    m_heads = new CylinderHead[m_cylinderBankCount];
    m_pistons = new Piston[m_cylinderCount];
    m_connectingRods = new ConnectingRod[m_cylinderCount];
    m_exhaustSystems = new ExhaustSystem[m_exhaustSystemCount];
    m_intakes = new Intake[m_intakeCount];
    m_combustionChambers = new CombustionChamber[m_cylinderCount];

    for (int i = 0; i < m_exhaustSystemCount; ++i) {
        m_exhaustSystems[i].m_index = i;
    }

    for (int i = 0; i < m_cylinderCount; ++i) {
        m_combustionChambers[i].setEngine(this);
    }
}

void Engine::destroy() {
    for (int i = 0; i < m_crankshaftCount; ++i) {
        m_crankshafts[i].destroy();
    }

    for (int i = 0; i < m_cylinderCount; ++i) {
        m_pistons[i].destroy();
        m_connectingRods[i].destroy();
        m_combustionChambers[i].destroy();
    }

    for (int i = 0; i < m_exhaustSystemCount; ++i) {
        m_exhaustSystems[i].destroy();
    }

    for (int i = 0; i < m_intakeCount; ++i) {
        m_intakes[i].destroy();
    }

    m_ignitionModule.destroy();

    if (m_throttle != nullptr) delete m_throttle;
    if (m_crankshafts != nullptr) delete[] m_crankshafts;
    if (m_cylinderBanks != nullptr) delete[] m_cylinderBanks;
    if (m_heads != nullptr) delete[] m_heads;
    if (m_pistons != nullptr) delete[] m_pistons;
    if (m_connectingRods != nullptr) delete[] m_connectingRods;
    if (m_exhaustSystems != nullptr) delete[] m_exhaustSystems;
    if (m_intakes != nullptr) delete[] m_intakes;
    if (m_combustionChambers != nullptr) delete[] m_combustionChambers;

    m_crankshafts = nullptr;
    m_cylinderBanks = nullptr;
    m_pistons = nullptr;
    m_connectingRods = nullptr;
    m_heads = nullptr;
    m_exhaustSystems = nullptr;
    m_intakes = nullptr;
    m_combustionChambers = nullptr;
    m_throttle = nullptr;
}

Crankshaft *Engine::getOutputCrankshaft() const {
    return &m_crankshafts[0];
}

void Engine::setSpeedControl(real_t s) {
    m_throttle->setSpeedControl(s);
}

real_t Engine::getSpeedControl() {
    return m_throttle->getSpeedControl();
}

void Engine::setThrottle(real_t throttle) {
    for (int i = 0; i < m_intakeCount; ++i) {
        m_intakes[i].m_throttle = throttle;
    }

    m_throttleValue = throttle;
}

real_t Engine::getThrottle() const {
    return m_throttleValue;
}

real_t Engine::getThrottlePlateAngle() const {
    return (1 - m_intakes[0].getThrottlePlatePosition()) * (constants::pi / 2);
}

bool placeRod(
    const ConnectingRod &rod,
    const CylinderBank &bank,
    const Crankshaft &crankshaft,
    real_t crankshaftAngle,
    real_t *p_x,
    real_t *p_y,
    real_t *theta,
    real_t *s)
{
    real_t p_x_0, p_y_0, l_x, l_y, theta_0;
    if (rod.getMasterRod() != nullptr) {
        real_t s;
        const bool succeeded = placeRod(
            *rod.getMasterRod(),
            *rod.getMasterRod()->getPiston()->getCylinderBank(),
            *rod.getCrankshaft(),
            crankshaftAngle,
            &p_x_0,
            &p_y_0,
            &theta_0,
            &s);

        if (!succeeded) {
            return false;
        }

        rod.getMasterRod()->getRodJournalPositionLocal(rod.getPiston()->getCylinderIndex(), &l_x, &l_y);
    }
    else {
        theta_0 = crankshaftAngle;
        p_x_0 = rod.getCrankshaft()->getPosX();
        p_y_0 = rod.getCrankshaft()->getPosY();
        rod.getCrankshaft()->getRodJournalPositionLocal(rod.getPiston()->getCylinderIndex(), &l_x, &l_y);
    }

    const real_t dx = std::cos(theta_0);
    const real_t dy = std::sin(theta_0);
    *p_x = p_x_0 + (dx * l_x - dy * l_y);
    *p_y = p_y_0 + (dy * l_x + dx * l_y);

    // (bank->m_x + bank->m_dx * s - p_x)^2 + (bank->m_y + bank->m_dy * s - p_y)^2 = (rod->m_length)^2
    const real_t a = bank.getDx() * bank.getDx() + bank.getDy() * bank.getDy();
    const real_t b = -2 * bank.getDx() * ((*p_x) - bank.getX()) - 2 * bank.getDy() * ((*p_y) - bank.getY());
    const real_t c =
        ((*p_x) - bank.getX()) * ((*p_x) - bank.getX())
        + ((*p_y) - bank.getY()) * ((*p_y) - bank.getY())
        - rod.getLength() * rod.getLength();

    const real_t det = b * b - 4 * a * c;
    if (det < 0) return false;

    const real_t sqrt_det = std::sqrt(det);
    const real_t s0 = (-b + sqrt_det) / (2 * a);
    const real_t s1 = (-b - sqrt_det) / (2 * a);

    *s = std::max(s0, s1);
    if (*s < 0) return false;
   
    if (s != nullptr) {
        const real_t dx = (bank.getX() + bank.getDx() * (*s)) - (*p_x);
        const real_t dy = (bank.getY() + bank.getDy() * (*s)) - (*p_y);

        *theta = (dy > 0)
            ? std::acos(dx)
            : -std::acos(dx);
    }

    return true;
}

void Engine::calculateDisplacement() {
    // There is a closed-form/correct way to do this which I really
    // don't feel like deriving right now, so I'm just going with this
    // numerical approximation.
    constexpr int Resolution = 1000;

    real_t *min_s = new real_t[m_cylinderCount];
    real_t *max_s = new real_t[m_cylinderCount];

    for (int i = 0; i < m_cylinderCount; ++i) {
        min_s[i] = std::numeric_limits<real_t>::max();
        max_s[i] = -std::numeric_limits<real_t>::max();
    }

    for (int j = 0; j < Resolution; ++j) {
        const real_t crankshaftAngle = 2 * (j / static_cast<real_t>(Resolution)) * constants::pi;

        for (int i = 0; i < m_cylinderCount; ++i) {
            const Piston &piston = m_pistons[i];
            const CylinderBank &bank = *piston.getCylinderBank();
            const ConnectingRod &rod = *piston.getRod();
            const Crankshaft &shaft = *rod.getCrankshaft();

            real_t p_x, p_y;
            real_t theta;
            real_t s;
            if (!placeRod(
                rod,
                bank,
                shaft,
                crankshaftAngle,
                &p_x,
                &p_y,
                &theta,
                &s))
            {
                continue;
            }

            min_s[i] = std::min(min_s[i], s);
            max_s[i] = std::max(max_s[i], s);
        }
    }

    real_t displacement = 0;
    for (int i = 0; i < m_cylinderCount; ++i) {
        const Piston &piston = m_pistons[i];
        const CylinderBank &bank = *piston.getCylinderBank();

        if (min_s[i] < max_s[i]) {
            const real_t r = bank.getBore() / 2.0;
            displacement += constants::pi * r * r * (max_s[i] - min_s[i]);
        }
    }

    m_displacement = displacement;
}

real_t Engine::getIntakeFlowRate() const {
    real_t airIntake = 0;
    for (int i = 0; i < m_intakeCount; ++i) {
        airIntake += m_intakes[i].m_flowRate;
    }

    return airIntake;
}

void Engine::update(real_t dt) {
    m_throttle->update(dt, this);
}

real_t Engine::getManifoldPressure() const {
    real_t pressureSum = 0.0;
    for (int i = 0; i < m_intakeCount; ++i) {
        pressureSum += m_intakes[i].m_system.pressure();
    }

    return pressureSum / m_intakeCount;
}

real_t Engine::getIntakeAfr() const {
    real_t totalOxygen = 0.0;
    real_t totalFuel = 0.0;
    for (int i = 0; i < m_intakeCount; ++i) {
        totalOxygen += m_intakes[i].m_system.n_o2();
        totalFuel += m_intakes[i].m_system.n_fuel();
    }

    constexpr real_t octaneMolarMass = units::mass(114.23, units::g);
    constexpr real_t oxygenMolarMass = units::mass(31.9988, units::g);

    if (totalFuel == 0) return 0;
    else {
        return
            (oxygenMolarMass * totalOxygen / 0.21)
            / (totalFuel * octaneMolarMass);
    }
}

real_t Engine::getExhaustO2() const {
    real_t totalInert = 0.0;
    real_t totalOxygen = 0.0;
    real_t totalFuel = 0.0;
    for (int i = 0; i < m_exhaustSystemCount; ++i) {
        totalInert += m_exhaustSystems[i].m_system.n_inert();
        totalOxygen += m_exhaustSystems[i].m_system.n_o2();
        totalFuel += m_exhaustSystems[i].m_system.n_fuel();
    }

    constexpr real_t octaneMolarMass = units::mass(114.23, units::g);
    constexpr real_t oxygenMolarMass = units::mass(31.9988, units::g);
    constexpr real_t nitrogenMolarMass = units::mass(28.014, units::g);

    if (totalFuel == 0) return 0;
    else {
        return
            (oxygenMolarMass * totalOxygen)
            / (
                totalFuel * octaneMolarMass
                + nitrogenMolarMass * totalInert
                + oxygenMolarMass * totalOxygen);
    }
}

void Engine::resetFuelConsumption() {
    for (int i = 0; i < m_intakeCount; ++i) {
        m_intakes[i].m_totalFuelInjected = 0;
    }
}

real_t Engine::getTotalFuelMassConsumed() const {
    real_t n_fuelConsumed = 0;
    for (int i = 0; i < m_intakeCount; ++i) {
        n_fuelConsumed += m_intakes[i].m_totalFuelInjected;
    }

    return n_fuelConsumed * m_fuel.getMolecularMass();
}

real_t Engine::getTotalVolumeFuelConsumed() const {
    return getTotalFuelMassConsumed() / m_fuel.getDensity();
}

int Engine::getMaxDepth() const {
    int maxDepth = 0;
    for (int i = 0; i < m_crankshaftCount; ++i) {
        maxDepth = std::max(m_crankshafts[i].getRodJournalCount(), maxDepth);
    }

    return maxDepth;
}

Simulator *Engine::createSimulator(Vehicle *vehicle, Transmission *transmission) {
    PistonEngineSimulator *simulator = new PistonEngineSimulator;
    Simulator::Parameters simulatorParams;
    simulatorParams.systemType = Simulator::SystemType::NsvOptimized;
    simulator->initialize(simulatorParams);

    simulator->loadSimulation(this, vehicle, transmission);
    simulator->setFluidSimulationSteps(8);

    return static_cast<Simulator *>(simulator);
}

real_t Engine::getRpm() const {
    if (m_crankshaftCount == 0) return 0;
    else return std::abs(units::toRpm(getCrankshaft(0)->m_body.v_theta));
}

real_t Engine::getSpeed() const {
    if (m_crankshaftCount == 0) return 0;
    else return std::abs(getCrankshaft(0)->m_body.v_theta);
}

bool Engine::isSpinningCw() const {
    return getOutputCrankshaft()->m_body.v_theta <= 0;
}
