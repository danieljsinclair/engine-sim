#include "../include/combustion_chamber.h"

#include "../include/constants.h"
#include "../include/units.h"
#include "../include/piston.h"
#include "../include/connecting_rod.h"
#include "../include/utilities.h"
#include "../include/exhaust_system.h"
#include "../include/cylinder_bank.h"
#include "../include/engine.h"

#include <algorithm>
#include <cmath>

CombustionChamber::CombustionChamber() {
    m_crankcasePressure = 0.0;
    m_piston = nullptr;
    m_head = nullptr;
    m_engine = nullptr;
    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
    m_lit = false;
    m_litLastFrame = false;
    m_peakTemperature = 0;

    m_meanPistonSpeedToTurbulence = nullptr;
    m_nBurntFuel = 0;

    m_manifoldToRunnerFlowRate = 0;
    m_primaryToCollectorFlowRate = 0;
    m_cylinderWidthApproximation = 0;
    m_cylinderCrossSectionSurfaceArea = 0;

    m_lastTimestepTotalExhaustFlow = 0;
    m_lastTimestepTotalIntakeFlow = 0;
    m_exhaustFlow = 0;
    m_exhaustFlowRate = 0;
    m_intakeFlowRate = 0;

    m_fuel = nullptr;
}

CombustionChamber::~CombustionChamber() {
    assert(m_pistonSpeed == nullptr);
    assert(m_pressure == nullptr);
}

void CombustionChamber::initialize(const Parameters &params) {
    m_piston = params.Piston;
    m_head = params.Head;
    m_fuel = params.Fuel;
    m_crankcasePressure = params.CrankcasePressure;
    m_meanPistonSpeedToTurbulence = params.MeanPistonSpeedToTurbulence;

    m_pistonSpeed = new double[StateSamples];
    m_pressure = new double[StateSamples];
    for (int i = 0; i < StateSamples; ++i) {
        m_pistonSpeed[i] = 0;
        m_pressure[i] = 0;
    }

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());

    m_manifoldToRunnerFlowRate = intake->getRunnerFlowRate();
    m_primaryToCollectorFlowRate = exhaust->getPrimaryFlowRate();

    const double bore_r = m_head->getCylinderBank()->getBore() / 2.0;
    m_cylinderCrossSectionSurfaceArea = constants::pi * bore_r * bore_r;
    m_cylinderWidthApproximation = std::sqrt(m_cylinderCrossSectionSurfaceArea);

    const double height = getVolume() / m_cylinderCrossSectionSurfaceArea;
    m_system.setGeometry(
        m_cylinderWidthApproximation,
        height,
        1.0,
        0.0);

    const double intakeRunnerCrossSection = m_head->getIntakeRunnerCrossSectionArea();
    const double intakeRunnerWidth = std::sqrt(intakeRunnerCrossSection);
    const double manifoldRunnerLength = intake->getRunnerLength();
    const double manifoldRunnerVolume = intakeRunnerCrossSection * manifoldRunnerLength;
    const double totalIntakeRunnerVolume = m_head->getIntakeRunnerVolume() + manifoldRunnerVolume;
    const double overallIntakeRunnerLength = totalIntakeRunnerVolume / intakeRunnerCrossSection;
    m_intakeRunnerAndManifold.initialize(
        units::pressure(1.0, units::atm),
        totalIntakeRunnerVolume,
        units::celcius(25.0));
    m_intakeRunnerAndManifold.setGeometry(
        overallIntakeRunnerLength,
        intakeRunnerWidth,
        1.0,
        0.0);

    const double exhaustRunnerCrossSection = m_head->getExhaustRunnerCrossSectionArea();
    const double exhaustRunnerWidth = std::sqrt(exhaustRunnerCrossSection);
    const double exhaustTubeLength =
        exhaust->getPrimaryTubeLength() + m_head->getHeaderPrimaryLength(m_piston->getCylinderIndex());
    const double exhaustTubeVolume = exhaustRunnerCrossSection * exhaustTubeLength;
    const double totalExhaustRunnerVolume = m_head->getExhaustRunnerVolume() + exhaustTubeVolume;
    const double overallExhaustRunnerLength = totalExhaustRunnerVolume / exhaustRunnerCrossSection;
    m_exhaustRunnerAndPrimary.initialize(
        units::pressure(1.0, units::atm),
        totalExhaustRunnerVolume,
        units::celcius(25.0));
    m_exhaustRunnerAndPrimary.setGeometry(
        overallExhaustRunnerLength,
        exhaustRunnerWidth,
        1.0,
        0.0);
}

void CombustionChamber::destroy() {
    if (m_pistonSpeed != nullptr) delete[] m_pistonSpeed;
    if (m_pressure != nullptr) delete[] m_pressure;

    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
}

double CombustionChamber::getVolume() const {
    const double combustionPortVolume = m_head->getCombustionChamberVolume();
    const CylinderBank *bank = m_head->getCylinderBank();

    const double area = bank->boreSurfaceArea();
    const double s =
        m_piston->relativeX() * bank->getDx()
        + m_piston->relativeY() * bank->getDy();
    const double sweep =
        area * (bank->getDeckHeight() - s - m_piston->getCompressionHeight());

    return sweep + combustionPortVolume - m_piston->getDisplacement();
}

double CombustionChamber::pistonSpeed() const {
    const CylinderBank *bank = m_head->getCylinderBank();
    return
        m_piston->m_body.v_x * bank->getDx()
        + m_piston->m_body.v_y * bank->getDy();
}

double CombustionChamber::calculateMeanPistonSpeed() const {
    double avg = 0;
    for (int i = 0; i < StateSamples; ++i) {
        avg += m_pistonSpeed[i];
    }

    avg /= StateSamples;
    return avg;
}

double CombustionChamber::calculateFiringPressure() const {
    double firingPressure = 0;
    for (int i = 0; i < StateSamples; ++i) {
        if (m_pressure[i] > firingPressure) {
            firingPressure = m_pressure[i];
        }
    }

    return firingPressure;
}

bool CombustionChamber::popLitLastFrame() {
    const bool lit = m_litLastFrame;
    m_litLastFrame = false;

    return lit;
}

void CombustionChamber::ignite() {
    if (!m_lit) {
        if (m_system.mix().p_fuel == 0) return;

        const double afr = m_system.mix().p_o2 / m_system.mix().p_fuel;
        const double equivalenceRatio = afr / m_fuel->getMolecularAfr();
        if (equivalenceRatio < 0.5) return;
        else if (equivalenceRatio > 1.9) return;

        const double idealInert = m_system.mix().p_o2 / 0.7;
        const double dilution = (m_system.mix().p_inert / idealInert) - 1;

        m_flameEvent.lastVolume = getVolume();
        m_flameEvent.travel_x = 0;
        m_flameEvent.travel_y = 0;
        m_flameEvent.lit_n = 0;
        m_flameEvent.total_n = m_system.n();
        m_flameEvent.percentageLit = 0;
        m_flameEvent.globalMix = m_system.mix();
        m_lit = true;
        m_litLastFrame = true;

        const double randomness =
            m_fuel->getBurningEfficiencyRandomness();
        const double lowEfficiencyAttenuation =
            m_fuel->getLowEfficiencyAttenuation();
        const double maxBurningEfficiency =
            m_fuel->getMaxBurningEfficiency();
        const double maxTurbulenceEffect =
            m_fuel->getMaxTurbulenceEffect();
        const double maxDilutionEffect =
            m_fuel->getMaxDilutionEffect();

        const double turbulence =
            m_meanPistonSpeedToTurbulence->sampleTriangle(
                calculateMeanPistonSpeed());
        const double mixingFactor =
            1.0 - (
                clamp(turbulence / maxTurbulenceEffect)
                * clamp(1 - dilution / maxDilutionEffect));
        const double rand_s =
            lowEfficiencyAttenuation
            * ((1 - randomness) + randomness * ((double)rand() / RAND_MAX));
        const double efficiencyAttenuation =
            (mixingFactor * rand_s + (1 - mixingFactor));
        m_flameEvent.efficiency =
            efficiencyAttenuation * maxBurningEfficiency;
        m_flameEvent.flameSpeed = m_fuel->flameSpeed(
            turbulence,
            afr,
            m_system.temperature(),
            m_system.pressure(),
            calculateFiringPressure(),
            units::pressure(160, units::psi));
    }
}

void CombustionChamber::update(double dt) {
    m_system.setVolume(getVolume());

    updateCycleStates();

    m_intakeFlowRate = m_head->intakeFlowRate(m_piston->getCylinderIndex());
    m_exhaustFlowRate = m_head->exhaustFlowRate(m_piston->getCylinderIndex());
}

void CombustionChamber::flow(double dt) {
    if (m_system.temperature() > m_peakTemperature) {
        m_peakTemperature = m_system.temperature();
    }

    const double volume = getVolume();
    const double cylinderHeight = volume / m_cylinderCrossSectionSurfaceArea;
    const double cylinderSurfaceArea =
        cylinderHeight * constants::pi * m_head->getCylinderBank()->getBore()
        + m_cylinderCrossSectionSurfaceArea * 2;

    const double dT = units::celcius(90.0) - m_system.temperature();

    m_system.changeEnergy(dT * cylinderSurfaceArea * 100 * dt);
    m_system.flow(m_piston->getBlowbyK(), dt, m_crankcasePressure, units::celcius(25.0));

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());

    GasSystem::FlowParameters flowParams;
    flowParams.dt = dt;

    flowParams.k_flow = m_manifoldToRunnerFlowRate;
    flowParams.crossSectionArea_0 = intake->getPlenumCrossSectionArea();
    flowParams.crossSectionArea_1 = m_head->getIntakeRunnerCrossSectionArea();
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &intake->m_system;
    flowParams.system_1 = &m_intakeRunnerAndManifold;
    GasSystem::flow(flowParams);

    m_intakeRunnerAndManifold.dissipateExcessVelocity();

    flowParams.k_flow = m_intakeFlowRate;
    flowParams.crossSectionArea_0 = m_head->getIntakeRunnerCrossSectionArea();
    flowParams.crossSectionArea_1 = volume / cylinderHeight;
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &m_intakeRunnerAndManifold;
    flowParams.system_1 = &m_system;
    const double intakeFlow = GasSystem::flow(flowParams);

    m_intakeRunnerAndManifold.dissipateExcessVelocity();
    m_system.dissipateExcessVelocity();

    flowParams.k_flow = m_exhaustFlowRate;
    flowParams.crossSectionArea_0 = volume / cylinderHeight;
    flowParams.crossSectionArea_1 = m_head->getExhaustRunnerCrossSectionArea();
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &m_system;
    flowParams.system_1 = &m_exhaustRunnerAndPrimary;
    const double exhaustFlow = GasSystem::flow(flowParams);

    m_system.dissipateExcessVelocity();
    m_exhaustRunnerAndPrimary.dissipateExcessVelocity();

    flowParams.k_flow = m_primaryToCollectorFlowRate;
    flowParams.crossSectionArea_0 = m_head->getExhaustRunnerCrossSectionArea();
    flowParams.crossSectionArea_1 = exhaust->getCollectorCrossSectionArea();
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &m_exhaustRunnerAndPrimary;
    flowParams.system_1 = exhaust->getSystem();
    GasSystem::flow(flowParams);

    m_intakeRunnerAndManifold.updateVelocity(dt, intake->getVelocityDecay());
    m_system.updateVelocity(dt, 0.5);
    m_exhaustRunnerAndPrimary.updateVelocity(dt, exhaust->getVelocityDecay());

    if (std::abs(intakeFlow) > 1E-9 && m_lit) {
        m_lit = false;
    }

    m_exhaustFlow = exhaustFlow;
    m_lastTimestepTotalExhaustFlow += exhaustFlow;
    m_lastTimestepTotalIntakeFlow += intakeFlow;

    if (m_lit) {
        CylinderBank *bank = m_head->getCylinderBank();
        const double totalTravel_x = bank->getBore() / 2;
        const double totalTravel_y = volume / bank->boreSurfaceArea();
        const double expansion = volume / m_flameEvent.lastVolume;
        const double lastTravel_x = m_flameEvent.travel_x;
        const double lastTravel_y = m_flameEvent.travel_y * expansion;
        const double flameSpeed = m_flameEvent.flameSpeed;

        m_flameEvent.travel_x =
            std::fmin(lastTravel_x + dt * flameSpeed, totalTravel_x);
        m_flameEvent.travel_y =
            std::fmin(lastTravel_y + dt * flameSpeed, totalTravel_y);

        if (lastTravel_x < m_flameEvent.travel_x || lastTravel_y < m_flameEvent.travel_y) {
            const double burnedVolume =
                m_flameEvent.travel_x * m_flameEvent.travel_x
                * constants::pi * m_flameEvent.travel_y;
            const double prevBurnedVolume =
                lastTravel_x * lastTravel_x * constants::pi * lastTravel_y;
            const double litVolume = burnedVolume - prevBurnedVolume;
            const double n = (litVolume / volume) * m_system.n();

            const double fuelBurned =
                m_system.react(n * m_flameEvent.efficiency, m_flameEvent.globalMix);
            const double massFuelBurned = fuelBurned * m_fuel->getMolecularMass();
            m_system.changeEnergy(
                massFuelBurned * m_fuel->getEnergyDensity());

            m_flameEvent.lit_n += n;
            m_flameEvent.percentageLit += litVolume / volume;

            m_nBurntFuel += massFuelBurned;
        }
        else {
            m_lit = false;
        }

        m_flameEvent.lastVolume = volume;
    }
}

double CombustionChamber::lastEventAfr() const {
    const double totalFuel = m_flameEvent.globalMix.p_fuel * m_flameEvent.total_n;
    const double totalOxygen = m_flameEvent.globalMix.p_o2 * m_flameEvent.total_n;
    const double totalInert = m_flameEvent.globalMix.p_inert * m_flameEvent.total_n;

    constexpr double octaneMolarMass = units::mass(114.23, units::g);
    constexpr double oxygenMolarMass = units::mass(31.9988, units::g);
    constexpr double nitrogenMolarMass = units::mass(28.014, units::g);

    if (totalFuel == 0) return 0;
    else {
        return
            (oxygenMolarMass * totalOxygen + totalInert * nitrogenMolarMass)
            / (totalFuel * octaneMolarMass);
    }
}

double CombustionChamber::calculateFrictionForce(double v_s) const {
    const double cylinderWallForce = m_piston->calculateCylinderWallForce();

    const double F_coul = m_frictionModel.frictionCoeff * cylinderWallForce;
    const double v_st = m_frictionModel.breakawayFrictionVelocity * constants::root_2;
    const double v_coul = m_frictionModel.breakawayFrictionVelocity / 10;
    const double F_brk = m_frictionModel.breakawayFriction;
    const double v = std::abs(v_s);

    const double F_0 = constants::root_2 * constants::e * (F_brk - F_coul);
    const double F_1 = v / v_st;
    const double F_2 = std::exp(-F_1 * F_1) * F_1;
    const double F_3 = F_coul * std::tanh(v / v_coul);
    const double F_4 = m_frictionModel.viscousFrictionCoefficient * v;

    return F_0 * F_2 + F_3 + F_4;
}

void CombustionChamber::updateCycleStates() {
    double crankAngle = m_engine->getOutputCrankshaft()->getCycleAngle();
    if (std::isnan(crankAngle) || std::isinf(crankAngle)) {
        crankAngle = 0.0;
    }

    const int i = (int)std::round((crankAngle / (4 * constants::pi)) * (StateSamples - 1.0));

    m_pistonSpeed[i] = std::abs(pistonSpeed());
    m_pressure[i] = m_system.pressure();
}

void CombustionChamber::apply(atg_scs::SystemState *system) {
    CylinderBank *bank = m_head->getCylinderBank();
    const double area = (bank->getBore() * bank->getBore() / 4.0) * constants::pi;
    const double v_x = system->v_x[m_piston->m_body.index];
    const double v_y = system->v_y[m_piston->m_body.index];

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    const double pressureDifferential = m_system.pressure() - m_crankcasePressure;
    const double force = -area * pressureDifferential;

    if (std::isnan(force) || std::isinf(force)) {
        assert(false);
    }

    constexpr double limit = 1E-3;
    const double abs_v_s = std::fmin(std::abs(v_s), limit);
    const double attenuation = abs_v_s / limit;

    const double F = calculateFrictionForce(v_s) * attenuation;
    const double F_fric = (v_s > 0)
        ? -F
        : F;

    system->applyForce(
        0.0,
        0.0,
        (force + F_fric) * bank->getDx(),
        (force + F_fric) * bank->getDy(),
        m_piston->m_body.index);
}

double CombustionChamber::getFrictionForce() const {
    CylinderBank *bank = m_head->getCylinderBank();
    const double v_x = m_piston->m_body.v_x;
    const double v_y = m_piston->m_body.v_y;

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    return calculateFrictionForce(v_s);
}

#ifdef ATG_ENGINE_SIM_AFTERFIRE_SPIKE

// Combustion energy released per mole of afterfire fuel burned (synthetic
// reference value used to size the exhaust pressure spike). Guarded effect:
// real combustion enthalpy depends on the charge; this is a fixed tunable.
namespace {
    constexpr double kEnergyPerMolFuel =
        units::energy(48.1, units::kJ) / units::mass(1.0, units::g)
        * units::mass(100.0, units::g);
}

// ============================================================================
// Afterfire — firing logic (GREEN phase).
//
// A pop fires on OVERRUN: the throttle is cut (closed pedal), RPM is above the
// auto-ignition floor, and the chamber is in a genuine deceleration window.
// Unburnt fuel reaching the hot exhaust is modelled by injecting a synthetic
// fuel charge into the exhaust runner and reacting it above auto-ignition,
// releasing energy as a pressure spike. Cooldown and probability gating apply.
// ============================================================================

void CombustionChamber::setAfterfireParameters(const AfterfireParameters &parameters) {
    m_afterfireParameters = parameters;
    m_afterfireEnabled = parameters.enabled;
}

CombustionChamber::AfterfireParameters CombustionChamber::getAfterfireParameters() const {
    return m_afterfireParameters;
}

void CombustionChamber::enableAfterfire(bool enabled) {
    m_afterfireEnabled = enabled;
    m_afterfireParameters.enabled = enabled;
}

bool CombustionChamber::isAfterfireEnabled() const {
    return m_afterfireEnabled;
}

double CombustionChamber::getAfterfireCooldownRemainingMs() const {
    return m_afterfireCooldownRemainingMs;
}

void CombustionChamber::setAfterfireCooldownRemainingMs(double ms) {
    m_afterfireCooldownRemainingMs = ms;
}

void CombustionChamber::tickAfterfireCooldown(double dtMs) {
    m_afterfireCooldownRemainingMs =
        std::max(0.0, m_afterfireCooldownRemainingMs - dtMs);
}

bool CombustionChamber::hasAfterfirePulse() const {
    return m_afterfirePulseRemainingMs > 0.0;
}

void CombustionChamber::tickAfterfirePulse(double dtMs) {
    m_afterfirePulseRemainingMs =
        std::max(0.0, m_afterfirePulseRemainingMs - dtMs);
}

// Overrun detection and gating. Returns true only when a pop is actually
// permitted this step for this chamber. Diagnostics counters record every
// declined gate so the test/diagnostics channel can explain a zero-event run.
bool CombustionChamber::shouldTriggerAfterfire(
    double throttle,
    double rpm,
    int globalEventsInDecel)
{
    if (!m_afterfireEnabled) return false;

    // Throttle not cut: pedal applied, not an overrun. Reset the per-chamber
    // decel window so a later cut starts a fresh window.
    if (throttle > m_afterfireParameters.throttleCutoff) {
        m_afterfireInDecel = false;
        m_afterfireDecelElapsedMs = 0.0;
        m_afterfireDiagnostics.eventsInCurrentDecel = 0;
        ++m_afterfireDiagnostics.skippedThrottle;
        return false;
    }

    // Below the auto-ignition RPM floor the exhaust cannot light off.
    if (rpm < m_afterfireParameters.rpmMin) {
        ++m_afterfireDiagnostics.skippedLowRpm;
        return false;
    }

    // Sustained-decel window: open it when we enter overrun or RPM rises again
    // (a genuine deceleration must be a falling-RPM trend, not per-step noise).
    const double rpmDelta = m_afterfireLastRpm - rpm;
    if (!m_afterfireInDecel || rpmDelta > 5.0) {
        m_afterfireInDecel = true;
        m_afterfireDecelElapsedMs = 0.0;
        m_afterfireDiagnostics.eventsInCurrentDecel = 0;
    }
    m_afterfireLastRpm = rpm;

    if (m_afterfireInDecel && rpmDelta < -2.0 && m_afterfireDecelElapsedMs > 0) {
        // RPM rising again: close the decel window.
        m_afterfireInDecel = false;
        m_afterfireDecelElapsedMs = 0.0;
        m_afterfireDiagnostics.eventsInCurrentDecel = 0;
    }

    if (m_afterfireDecelElapsedMs > m_afterfireParameters.decelWindowMs) {
        ++m_afterfireDiagnostics.skippedMaxEvents;
        return false;
    }
    if (m_afterfireDecelElapsedMs < 0.0) {
        ++m_afterfireDiagnostics.skippedNoOverrun;
        return false;
    }

    // Global (cross-chamber) cap for this decel.
    if (globalEventsInDecel >= m_afterfireParameters.maxEventsPerDecel) {
        ++m_afterfireDiagnostics.skippedMaxEvents;
        return false;
    }

    // Per-chamber simulation-time cooldown.
    if (m_afterfireCooldownRemainingMs > 0.0) {
        ++m_afterfireDiagnostics.skippedCooldown;
        return false;
    }

    // Probability gate (deterministic when probability == 1.0).
    const double p = clamp(m_afterfireParameters.probability, 0.0, 1.0);
    if (p < 1.0 && (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) > p) {
        ++m_afterfireDiagnostics.skippedProbability;
        return false;
    }

    return true;
}

// Bookkeeping records a fired pop and arms the per-chamber cooldown/pulse.
void CombustionChamber::recordAfterfireEvent(
    double throttle,
    double rpm,
    double peakPressure,
    double energyReleased)
{
    m_lastAfterfirePeakPressure = peakPressure;
    m_lastAfterfireEnergyReleased = energyReleased;
    m_afterfireDiagnostics.lastEventRpm = rpm;
    m_afterfireDiagnostics.lastEventThrottle = throttle;
    m_afterfireDiagnostics.lastEventPeakPressure = peakPressure;
    m_afterfireDiagnostics.lastEventEnergyReleased = energyReleased;
    m_afterfireDiagnostics.eventsInCurrentDecel = 1;
    m_afterfireCooldownRemainingMs = m_afterfireParameters.cooldownMs;
    m_afterfirePulseRemainingMs = m_afterfirePulseDurationMs;
    ++m_afterfireEventCount;
}

bool CombustionChamber::tickAfterfire(double throttle, double rpm, int globalEventsInDecel) {
    if (!shouldTriggerAfterfire(throttle, rpm, globalEventsInDecel)) return false;
    // Only count when a pop is actually generated — triggerAfterfire can still
    // abort on gating, and counting those aborts would inflate the global cap.
    return triggerAfterfire(m_afterfireParameters.intensity, throttle, rpm);
}

void CombustionChamber::triggerAfterfire(double intensity) {
    const double throttle = (m_engine != nullptr)
        ? m_engine->getThrottle()
        : 0.0;
    const double rpm = (m_engine != nullptr)
        ? m_engine->getSpeed() * 60.0 / (2.0 * constants::pi)
        : 0.0;

    triggerAfterfire(intensity, throttle, rpm);
}

// Inject a synthetic unburnt-fuel charge into the hot exhaust runner and react
// it (light-off above auto-ignition), releasing energy as a pressure spike.
bool CombustionChamber::triggerAfterfire(double intensity, double throttle, double rpm) {
    if (!shouldTriggerAfterfire(throttle, rpm, 0)) {
        return false;
    }

    const double clampedIntensity = clamp(intensity, 0.0, 1.0);
    const double clampedFuelFraction = clamp(m_afterfireParameters.fuelFraction, 0.0, 0.02);

    // Under closed-throttle overrun the exhaust runner is depleted (n() -> 0),
    // which would skip the pop or make it inaudible. Floor the reference mole
    // count so the effect stays consistent — this is a guarded synthetic charge.
    constexpr double nFloor = 0.01;  // mol
    const double nReference = std::max(m_exhaustRunnerAndPrimary.n(), nFloor);
    const double nFuel = nReference * clampedFuelFraction * clampedIntensity;
    if (nFuel <= 0.0) return false;

    m_exhaustRunnerAndPrimary.injectFuel(nFuel);

    GasSystem::Mix afterfireMix;
    afterfireMix.p_fuel = 0.08;
    afterfireMix.p_inert = 0.67;
    afterfireMix.p_o2 = 0.25;

    m_exhaustRunnerAndPrimary.react(nFuel, afterfireMix);

    const double energyReleased = nFuel * kEnergyPerMolFuel;
    m_exhaustRunnerAndPrimary.changeEnergy(energyReleased);

    const double postPressure = m_exhaustRunnerAndPrimary.pressure();
    recordAfterfireEvent(throttle, rpm, postPressure, energyReleased);
    return true;
}

double CombustionChamber::getLastAfterfirePeakPressure() const {
    return m_lastAfterfirePeakPressure;
}

double CombustionChamber::getLastAfterfireEnergyReleased() const {
    return m_lastAfterfireEnergyReleased;
}

int CombustionChamber::getAfterfireEventCount() const {
    return m_afterfireEventCount;
}

double CombustionChamber::getLastAfterfireRpm() const {
    return m_afterfireDiagnostics.lastEventRpm;
}

CombustionChamber::AfterfireDiagnostics CombustionChamber::getAfterfireDiagnostics() const {
    AfterfireDiagnostics diagnostics = m_afterfireDiagnostics;
    diagnostics.eventCount = m_afterfireEventCount;
    diagnostics.lastEventPeakPressure = m_lastAfterfirePeakPressure;
    diagnostics.lastEventEnergyReleased = m_lastAfterfireEnergyReleased;

    return diagnostics;
}

void CombustionChamber::resetAfterfireDiagnostics() {
    m_afterfireDiagnostics = AfterfireDiagnostics{};
    m_lastAfterfirePeakPressure = 0.0;
    m_lastAfterfireEnergyReleased = 0.0;
    m_afterfireEventCount = 0;
    m_afterfireInDecel = false;
    m_afterfireLastRpm = 0.0;
    m_afterfireDecelElapsedMs = 0.0;
    m_afterfireCooldownRemainingMs = 0.0;
    m_afterfirePulseRemainingMs = 0.0;
}

#endif /* ATG_ENGINE_SIM_AFTERFIRE_SPIKE */
