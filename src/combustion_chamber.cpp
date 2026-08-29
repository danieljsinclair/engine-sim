#include "../include/combustion_chamber.h"

#include "../include/constants.h"
#include "../include/units.h"
#include "../include/piston.h"
#include "../include/connecting_rod.h"
#include "../include/utilities.h"
#include "../include/exhaust_system.h"
#include "../include/cylinder_bank.h"
#include "../include/engine.h"
#include "../include/intake.h"
#include "../include/wav_loader.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <filesystem>
#include <random>
#include <set>

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

void CombustionChamber::resetGasState() {
    // Back to the initialize() condition for every chamber-local gas system.
    // m_system keeps its current (piston-tracked) volume — reset() derives
    // n_mol from it — so this is a fresh ambient charge at the cylinder's
    // present position. Combustion bookkeeping (m_lit, flame event) is left
    // alone: m_lit self-clears on the next intake flow.
    m_system.reset(
            units::pressure(1.0, units::atm),
            units::celcius(25.0));
    m_intakeRunnerAndManifold.reset(
            units::pressure(1.0, units::atm),
            units::celcius(25.0));
    m_exhaustRunnerAndPrimary.reset(
            units::pressure(1.0, units::atm),
            units::celcius(25.0));
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
    // volumeMoved: the same transfer as a VOLUME at the source side's
    // conditions (cylinder state on outflow, runner state on reversion) —
    // what the exhaust-flow readout accumulates. See GasSystem::flow.
    double exhaustVolumeMoved = 0.0;
    const double exhaustFlow = GasSystem::flow(flowParams, &exhaustVolumeMoved);

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
    // Accumulate the port flow as VOLUME (source-side conditions, signed:
    // positive = out the exhaust port, negative = reversion). The raw mole
    // sum was previously displayed as a volume rate — mol/s shown as m^3/s,
    // dimensionally wrong and ~70x off. m_exhaustFlow stays in moles for its
    // existing consumers.
    m_lastTimestepTotalExhaustFlow += exhaustVolumeMoved;
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

// ============================================================================
// Afterfire — auto-ignition of unburnt fuel in the exhaust runner.
//
// This is a PHYSICAL model, not a scheduler. Nothing here asks "is the driver
// coasting?", counts pops, rolls dice or waits out a cooldown. One question is
// asked, once per simulation step, of the chamber's own exhaust runner:
//
//     has the unburnt fuel sitting in this pipe been hot enough, for long
//     enough, to auto-ignite?
//
// Auto-ignition is not instantaneous. A fuel-air mixture held above its
// auto-ignition temperature reacts only after an INDUCTION PERIOD tau(T) — the
// time the pre-ignition chemistry needs to run away. tau is Arrhenius, so it
// collapses exponentially as the gas gets hotter. The model integrates residence
// against that clock:
//
//     progress += dt / tau(T)        ignition when progress reaches 1
//
// The competing process is SCAVENGING: exhaust flow sweeps the charge out of the
// runner before the chemistry finishes, resetting progress. So a pop requires
// hot gas AND weak flow, and that single race is what makes the effect select
// overrun on its own:
//
//   - Under power the pipe is hottest (measured 2413 K at WOT) but every exhaust
//     stroke blows the runner through, so the fuel never stays put long enough.
//     A naive "fuel + heat" test would fire hardest HERE, which is exactly the
//     wrong answer, and is why residence time — not temperature alone — has to
//     be the discriminator.
//   - On overrun the throttle is shut, so cylinder filling collapses and flow
//     through the runner nearly stops, while the pipe is still soaked at
//     1700 K+ from the preceding pull. Fuel now lingers, tau is short, and the
//     integral completes: it pops.
//   - As the pipe cools the exponential stretches tau out until the pops die
//     away by themselves, which is what a real engine does on a long coast.
//
// The irregularity is likewise physical, not an rand() sprinkle. Runner
// temperature, mixture and flow all oscillate with the firing cycle, so
// progress climbs in uneven bursts and crosses 1 at genuinely uneven intervals.
// ============================================================================

void CombustionChamber::setAfterfireParameters(const AfterfireParameters &parameters) {
    m_afterfireParameters = parameters;
    m_afterfireEnabled = parameters.enabled;

    // Resolve and load the custom afterfire pop samples, if any.
    // afterfireWavPaths is the bridge-resolved, glob-expanded list of absolute
    // candidate files (empty => use the engine's default exhaust IR). The pop is
    // MIXED on top of the engine's continuing exhaust sound, so the full sample is
    // kept at its native length and sample rate (the rate is remembered for
    // resampling at mix time). A multi-channel WAV is downmixed to mono by
    // averaging its channels; the synthesizer is mono per exhaust channel.
    m_afterfireWavSamples.clear();
    m_afterfireWavSampleRates.clear();
    m_afterfireWavChosen = -1;
    m_afterfireWavChosenCount = 0;
    for (const std::string& path : parameters.afterfireWavPaths) {
        if (path.empty()) continue;
        WavLoader::Result wav = WavLoader::load(path);
        if (!wav.valid || wav.getSampleCount() == 0) {
            if (parameters.diagnostics) {
                std::fprintf(stderr, "[AFTERFIRE] could not load afterfire WAV: %s\n", path.c_str());
            }
            continue;
        }
        const int channels = (wav.channels > 0) ? wav.channels : 1;
        const size_t frameCount = wav.getSampleCount() / static_cast<size_t>(channels);
        std::vector<int16_t> mono(frameCount);
        if (channels == 1) {
            std::copy(wav.getData(), wav.getData() + frameCount, mono.begin());
        } else {
            // Average the channels into mono (integer averaging, round to nearest).
            for (size_t f = 0; f < frameCount; ++f) {
                int32_t acc = 0;
                for (int c = 0; c < channels; ++c) {
                    acc += wav.getData()[f * channels + c];
                }
                mono[f] = static_cast<int16_t>((acc + channels / 2) / channels);
            }
        }
        m_afterfireWavSamples.push_back(std::move(mono));
        m_afterfireWavSampleRates.push_back(wav.sampleRate > 0 ? wav.sampleRate : 44100);
    }
    if (parameters.diagnostics && !m_afterfireWavSamples.empty()) {
        std::fprintf(stderr, "[AFTERFIRE] loaded %zu custom pop WAV candidate(s)\n",
                     m_afterfireWavSamples.size());
    }
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

double CombustionChamber::getAfterfireIgnitionProgress() const {
    return m_afterfireIgnitionProgress;
}

bool CombustionChamber::hasAfterfirePulse() const {
    return m_afterfirePulseRemainingMs > 0.0;
}

void CombustionChamber::tickAfterfirePulse(double dtMs) {
    m_afterfirePulseRemainingMs =
        std::max(0.0, m_afterfirePulseRemainingMs - dtMs);
}

// Stage 1 — raw fuel delivery and scavenging.
//
// A pop needs RAW, never-burned fuel in the pipe. That is NOT the same thing as
// the runner's p_fuel: measured on this engine, the runner's fuel fraction is
// 0.019 at wide-open throttle and only 0.0026 late in a coast, because ordinary
// combustion always leaves some fuel in the products. Keying off p_fuel would
// therefore fire hardest under power, which is precisely the wrong answer.
//
// What actually distinguishes overrun is MANIFOLD VACUUM. With the throttle
// shut the cylinder cannot fill: the trapped charge is mostly leftover exhaust
// gas, dilution passes the point where a flame will propagate, the cycle
// misfires, and its fuel is pumped into the exhaust unburned. Measured MAP on
// this engine separates the regimes cleanly and by a wide margin:
//
//     wide-open throttle   98 kPa
//     part throttle        52 kPa
//     idle                 53 kPa
//     OVERRUN           25-29 kPa
//
// so a threshold at ~40 kPa selects overrun and nothing else. Note idle sits
// with part throttle, not with overrun — an idling engine must not pop, and a
// vacuum test gets that right for free.
//
// Fuel is credited once per exhaust event (an edge, not every step), and is
// then eaten away by exhaust flow scavenging the pipe. Both processes are
// per-chamber and cycle-resolved, which is where the irregular spacing of real
// crackle comes from — no rand(), no interval timer.
void CombustionChamber::updateRawExhaustFuel(double dt) {
    const Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    const double manifoldPressure = intake->m_system.pressure();

    m_afterfireDiagnostics.minManifoldPressure =
        (m_afterfireDiagnostics.minManifoldPressure == 0.0)
            ? manifoldPressure
            : std::min(m_afterfireDiagnostics.minManifoldPressure, manifoldPressure);

    // Raw fuel is carried out with the gas, so it is credited for as long as
    // the cylinder is emptying — not once on an edge. (Crediting a single
    // step's worth per cycle while scavenging ran every step starved the pipe
    // by ~2 orders of magnitude: measured rawFuelFraction 8e-06 against a 5e-04
    // threshold.) The amount is the flow times the fuel fraction ACTUALLY still
    // unburned in the cylinder, so a cycle that burned cleanly contributes
    // almost nothing and no separate "intensity" term is needed.
    const double exhaustFlow = m_exhaustFlow;
    const bool exhaustOpen = exhaustFlow > 0.0;
    if (exhaustOpen && manifoldPressure < m_afterfireParameters.misfireManifoldPressure) {
        const double n_rawFuel = exhaustFlow * m_system.mix().p_fuel;
        if (n_rawFuel > 0.0) {
            m_afterfireRawFuel_n += n_rawFuel;
            // Count the CYCLE, not the step: only the opening edge is a new
            // misfire event, so this stays a meaningful per-cycle tally.
            if (!m_afterfireExhaustOpen) ++m_afterfireDiagnostics.misfireCycles;
        }
    }
    m_afterfireExhaustOpen = exhaustOpen;

    // Scavenging: exhaust throughput replaces the pipe's contents, carrying the
    // raw fuel away with it. This is what stops fuel accumulating indefinitely,
    // and (via the progress reset below) what makes a well-scavenged pipe under
    // power incapable of popping however hot it gets.
    const double n_runner = m_exhaustRunnerAndPrimary.n();
    if (n_runner > 0.0 && exhaustFlow > 0.0) {
        const double sweptFraction = clamp(exhaustFlow / n_runner, 0.0, 1.0);
        m_afterfireRawFuel_n *= (1.0 - sweptFraction);
    }

    (void)dt;
}

// Induction time tau(T) for the runner's current state, in seconds.
//
// Returns infinity — "will never light" — when a physical precondition for
// reaction is absent, so the caller's integral simply makes no progress rather
// than needing a parallel set of boolean gates. Each refusal is counted under
// the reason that caused it, because "no pops" is otherwise undiagnosable.
double CombustionChamber::afterfireIgnitionDelay() {
    const double n_runner = m_exhaustRunnerAndPrimary.n();
    const double rawFuelFraction = (n_runner > 0.0)
        ? m_afterfireRawFuel_n / n_runner
        : 0.0;
    const double T = m_exhaustRunnerAndPrimary.temperature();

    m_afterfireDiagnostics.maxRawFuelFraction =
        std::max(m_afterfireDiagnostics.maxRawFuelFraction, rawFuelFraction);

    double delay = std::numeric_limits<double>::infinity();
    if (rawFuelFraction < m_afterfireParameters.minRawFuelFraction) {
        ++m_afterfireDiagnostics.skippedNoFuel;
    }
    else if (m_exhaustRunnerAndPrimary.mix().p_o2 < m_afterfireParameters.minOxygenMoleFraction) {
        ++m_afterfireDiagnostics.skippedNoOxygen;
    }
    else if (T < m_afterfireParameters.autoIgnitionTempK) {
        ++m_afterfireDiagnostics.skippedTooCold;
    }
    else {
        // tau(T) = tau_ref * exp(Ta * (1/T - 1/T_ref)).
        // Above T_ref the exponent is negative and the delay shrinks; below it
        // the delay grows without any extra clamping.
        const double exponent = m_afterfireParameters.activationTempK
            * (1.0 / T - 1.0 / m_afterfireParameters.refTempK);
        delay = m_afterfireParameters.ignitionDelayRefS * std::exp(exponent);
    }

    return delay;
}

// Stage 2 — advance the induction integral and light off if it completes.
//
// Progress belongs to the raw fuel currently resident in the runner, so it is
// discarded the moment that fuel stops being able to react — swept out by
// scavenging, starved of oxygen, or sitting in a pipe that has fallen below the
// auto-ignition temperature. That reset is the mechanism that prevents a hot,
// hard-working engine from slowly accumulating its way to a pop.
bool CombustionChamber::updateAfterfire(double dt, double throttle) {
    bool fired = false;

    if (m_afterfireEnabled) {
        // Throttle GATE. `throttle` is the pedal (Engine::getSpeedControl):
        // 1 = wide open, 0 = shut. The manifold-vacuum test alone cannot
        // discriminate steady part-throttle (MAP ~52 kPa) from overrun (MAP
        // ~25 kPa) on this engine, because part-throttle MAP sits between idle
        // and overrun and a low MAP also occurs at light steady cruise. With the
        // pedal above the cutoff the driver is ON THE GAS, so whatever low MAP
        // exists is a fueling/load condition, NOT overrun — and the runner is
        // being scavenged hard every exhaust stroke, so it cannot pop anyway.
        // Refusing here is purely physical: no pedal => no overrun => no pop.
        // (Measured: 10% pedal @ 52 kPa MAP previously fired; that is partial
        // throttle, not overrun, so it must be gated out.)
        if (throttle >= m_afterfireParameters.throttleCutoff) {
            ++m_afterfireDiagnostics.skippedThrottle;
            return false;
        }

        updateRawExhaustFuel(dt);

        const double runnerTempK = m_exhaustRunnerAndPrimary.temperature();
        m_afterfireDiagnostics.maxRunnerTempK =
            std::max(m_afterfireDiagnostics.maxRunnerTempK, runnerTempK);

        const double delay = afterfireIgnitionDelay();
        if (std::isinf(delay)) {
            m_afterfireIgnitionProgress = 0.0;
        }
        else {
            m_afterfireIgnitionProgress += dt / delay;
            m_afterfireDiagnostics.maxIgnitionProgress =
                std::max(m_afterfireDiagnostics.maxIgnitionProgress, m_afterfireIgnitionProgress);

            if (m_afterfireIgnitionProgress >= 1.0) {
                m_afterfireIgnitionProgress = 0.0;
                fired = igniteExhaustCharge(runnerTempK);
            }
            else {
                ++m_afterfireDiagnostics.skippedNotReady;
            }
        }
    }

    return fired;
}

// Burn the raw fuel pocket and release its chemical energy into the runner.
//
// The pressure spike is a CONSEQUENCE of the combustion, not a hand-authored
// waveform: react() consumes fuel against the oxygen actually present and
// returns the moles burned, and that quantity times the fuel's own energy
// density is the heat added. A lean pocket therefore makes a weak pop and a
// rich one a loud crack, with no separate intensity schedule.
bool CombustionChamber::igniteExhaustCharge(double runnerTempK) {
    bool fired = false;

    const double n_runner = m_exhaustRunnerAndPrimary.n();
    if (m_afterfireRawFuel_n > 0.0 && n_runner > 0.0) {
        const double pressureBefore = m_exhaustRunnerAndPrimary.pressure();

        // Present the reaction with the raw fuel as fuel and the runner's own
        // oxygen as oxidiser, so an O2-starved pipe self-limits.
        GasSystem::Mix chargeMix;
        chargeMix.p_fuel = clamp(m_afterfireRawFuel_n / n_runner, 0.0, 1.0);
        chargeMix.p_o2 = m_exhaustRunnerAndPrimary.mix().p_o2;
        chargeMix.p_inert = clamp(1.0 - chargeMix.p_fuel - chargeMix.p_o2, 0.0, 1.0);

        const double n_fuelBurned = m_exhaustRunnerAndPrimary.react(n_runner, chargeMix);
        if (n_fuelBurned > 0.0) {
            const double energyReleased =
                n_fuelBurned * m_fuel->getMolecularMass() * m_fuel->getEnergyDensity()
                * m_afterfireParameters.energyScale;
            m_exhaustRunnerAndPrimary.changeEnergy(energyReleased);

            // The fuel that burned is gone from the pipe.
            m_afterfireRawFuel_n = std::max(0.0, m_afterfireRawFuel_n - n_fuelBurned);

            const double pressureAfter = m_exhaustRunnerAndPrimary.pressure();
            recordAfterfireEvent(pressureAfter, energyReleased, runnerTempK);

            if (m_afterfireParameters.diagnostics) {
                const double rpm = (m_engine != nullptr)
                    ? m_engine->getSpeed() * 60.0 / (2.0 * constants::pi)
                    : 0.0;
                printf("[AFTERFIRE] pop: rpm=%.0f runnerT=%.0fK dP=%.0fPa energy=%.2fJ nFuel=%.3e\n",
                    rpm, runnerTempK, pressureAfter - pressureBefore, energyReleased, n_fuelBurned);
                fflush(stdout);
            }

            // Choose which custom pop sample to inject this time. With a single
            // file the choice is fixed; with a glob the bank does not fire the
            // same sample every time, which keeps a multi-cylinder overrun from
            // sounding like one looped clip. The chosen samples are exposed to the
            // simulator, which owns the audio path and MIXES the pop onto the
            // exhaust channel's output (the chamber has no access to the
            // synthesizer — SRP: the simulator drives audio). The pop is summed
            // into the channel's audio alongside the engine's continuing exhaust
            // sound, so the engine note is never interrupted.
            if (!m_afterfireWavSamples.empty()) {
                static thread_local std::mt19937 rng(std::random_device{}());
                std::uniform_int_distribution<int> dist(0, static_cast<int>(m_afterfireWavSamples.size()) - 1);
                m_afterfireWavChosen = dist(rng);
                m_afterfireWavChosenCount = m_afterfireWavSamples[m_afterfireWavChosen].size();
            }
            else {
                m_afterfireWavChosen = -1;
                m_afterfireWavChosenCount = 0;
            }

            fired = true;
        }
    }

    return fired;
}

bool CombustionChamber::hasAfterfireWavChoice() const {
    return m_afterfireWavChosen >= 0
        && static_cast<size_t>(m_afterfireWavChosen) < m_afterfireWavSamples.size();
}

const int16_t* CombustionChamber::afterfireWavChoice(size_t& outCount) const {
    if (!hasAfterfireWavChoice()) {
        outCount = 0;
        return nullptr;
    }
    outCount = m_afterfireWavChosenCount;
    return m_afterfireWavSamples[m_afterfireWavChosen].data();
}

int CombustionChamber::afterfireWavChoiceSampleRate() const {
    if (!hasAfterfireWavChoice()) {
        return 44100;  // synthesizer audio rate: resampling becomes a no-op
    }
    return m_afterfireWavSampleRates[m_afterfireWavChosen];
}

void CombustionChamber::recordAfterfireEvent(
    double peakPressure,
    double energyReleased,
    double runnerTempK)
{
    const double rpm = (m_engine != nullptr)
        ? m_engine->getSpeed() * 60.0 / (2.0 * constants::pi)
        : 0.0;

    m_lastAfterfirePeakPressure = peakPressure;
    m_lastAfterfireEnergyReleased = energyReleased;
    m_afterfireDiagnostics.lastEventRpm = rpm;
    m_afterfireDiagnostics.lastEventThrottle = (m_engine != nullptr)
        ? m_engine->getThrottle()
        : 0.0;
    m_afterfireDiagnostics.lastEventPeakPressure = peakPressure;
    m_afterfireDiagnostics.lastEventEnergyReleased = energyReleased;
    m_afterfireDiagnostics.lastEventRunnerTempK = runnerTempK;
    m_afterfirePulseRemainingMs = m_afterfirePulseDurationMs;
    ++m_afterfireEventCount;
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
    m_afterfireIgnitionProgress = 0.0;
    m_afterfireRawFuel_n = 0.0;
    m_afterfireExhaustOpen = false;
    m_afterfirePulseRemainingMs = 0.0;
}

#endif /* ATG_ENGINE_SIM_AFTERFIRE_SPIKE */
