#include "../include/simulator.h"

Simulator::Simulator() {
    m_engine = nullptr;
    m_vehicle = nullptr;
    m_transmission = nullptr;
    m_system = nullptr;

    m_physicsProcessingTime = 0;

    m_simulationSpeed = 1.0;
    m_targetSynthesizerLatency = 0.1;
    m_simulationFrequency = 10000;
    m_steps = 0;

    m_currentIteration = 0;

    m_filteredEngineSpeed = 0.0;
    m_dynoTorqueSamples = nullptr;
    m_lastDynoTorqueSample = 0;

    m_frameExhaustVolume = 0.0;
    m_frameCouplingTorqueSum = 0.0;
    m_frameTurbineTorqueSum = 0.0;
    m_frameStepsTaken = 0;
}

Simulator::~Simulator() {
    assert(m_system == nullptr);
    if (m_dynoTorqueSamples != nullptr) {
        delete[] m_dynoTorqueSamples;
        m_dynoTorqueSamples = nullptr;
    }
}

void Simulator::initialize(const Parameters &params) {
    if (params.systemType == SystemType::NsvOptimized) {
        atg_scs::OptimizedNsvRigidBodySystem *system =
            new atg_scs::OptimizedNsvRigidBodySystem;
        system->initialize(
            new atg_scs::GaussSeidelSleSolver);
        m_system = system;
    }
    else {
        atg_scs::GenericRigidBodySystem *system =
            new atg_scs::GenericRigidBodySystem;
        system->initialize(
            new atg_scs::GaussianEliminationSleSolver,
            new atg_scs::NsvOdeSolver);
        m_system = system;
    }

    m_dynoTorqueSamples = new double[DynoTorqueSamples];
    for (int i = 0; i < DynoTorqueSamples; ++i) {
        m_dynoTorqueSamples[i] = 0.0;
    }
}

void Simulator::loadSimulation(Engine *engine, Vehicle *vehicle, Transmission *transmission) {
    m_engine = engine;
    m_vehicle = vehicle;
    m_transmission = transmission;
}

void Simulator::releaseSimulation() {
    m_synthesizer.endAudioRenderingThread();
    if (m_system != nullptr) m_system->reset();

    destroy();
}

void Simulator::startFrame(double dt) {
    if (m_engine == nullptr) {
        m_steps = 0;
        return;
    }

    m_simulationStart = std::chrono::steady_clock::now();
    m_currentIteration = 0;
    m_synthesizer.setInputSampleRate(m_simulationFrequency * m_simulationSpeed);

    const double timestep = getTimestep();
    m_steps = (int)std::round((dt * m_simulationSpeed) / timestep);

    const double targetLatency = getSynthesizerInputLatencyTarget();
    if (m_synthesizer.getLatency() < targetLatency) {
        m_steps = static_cast<int>((m_steps + 1) * 1.1);
    }
    else if (m_synthesizer.getLatency() > targetLatency) {
        m_steps = static_cast<int>((m_steps - 1) * 0.9);
        if (m_steps < 0) {
            m_steps = 0;
        }
    }

    if (m_steps > 0) {
        for (int i = 0; i < m_engine->getIntakeCount(); ++i) {
            m_engine->getIntake(i)->m_flowRate = 0;
        }

        // Reset the frame-integration accumulators ONLY when this frame will
        // actually step: a zero-step frame (the latency governor can produce
        // one) keeps the previous frame's means instead of reading as zero.
        m_frameExhaustVolume = 0.0;
        m_frameCouplingTorqueSum = 0.0;
        m_frameTurbineTorqueSum = 0.0;
        m_frameStepsTaken = 0;
    }
}

bool Simulator::simulateStep() {
    if (getCurrentIteration() >= simulationSteps()) {
        auto s1 = std::chrono::steady_clock::now();

        const long long lastFrame =
            std::chrono::duration_cast<std::chrono::microseconds>(s1 - m_simulationStart).count();
        m_physicsProcessingTime = m_physicsProcessingTime * 0.98 + 0.02 * lastFrame;

        return false;
    }

    const double timestep = getTimestep();
    m_system->process(timestep, 1);

    m_engine->update(timestep);
    m_vehicle->update(timestep);
    m_transmission->update(timestep);

    updateFilteredEngineSpeed(timestep);

    Crankshaft *outputShaft = m_engine->getOutputCrankshaft();
    outputShaft->resetAngle();

    for (int i = 0; i < m_engine->getCrankshaftCount(); ++i) {
        Crankshaft *shaft = m_engine->getCrankshaft(i);

        // Correct drift (temporary hack)
        shaft->m_body.theta = outputShaft->m_body.theta;
    }

    const int index =
        static_cast<int>(std::floor(DynoTorqueSamples * outputShaft->getCycleAngle() / (4 * constants::pi)));
    const int step = m_engine->isSpinningCw() ? 1 : -1;
    m_dynoTorqueSamples[index] = m_dyno.getTorque();

    if (m_lastDynoTorqueSample != index) {
        for (int i = m_lastDynoTorqueSample + step; i != index; i += step) {
            if (i >= DynoTorqueSamples) {
                i = -1;
                continue;
            }
            else if (i < 0) {
                i = DynoTorqueSamples;
                continue;
            }

            m_dynoTorqueSamples[i] = m_dyno.getTorque();
        }

        m_lastDynoTorqueSample = index;
    }

    simulateStep_();

    // Frame-integration for the display readouts (see the header). F_t was
    // repopulated by m_system->process() above and the chamber exhaust
    // accumulator by this step's engine update; grab both NOW, before the
    // next step overwrites them, so the frame getters can present means
    // instead of an aliased last-substep sample.
    m_frameExhaustVolume += getTotalExhaustFlow();
    if (m_transmission != nullptr) {
        if (m_transmission->hasTorqueConverter()) {
            const auto *converter = m_transmission->getTorqueConverter();
            // Row 0: J = [0 0 -1 | 0 0 TR] — column 0 is the impeller
            // (engine) side, column 1 the turbine side. Sign conventions
            // mirror the bridge readout: engine producing power positive,
            // turbine torque negated so "driving the wheels" is positive.
            m_frameCouplingTorqueSum += converter->F_t[0][0];
            m_frameTurbineTorqueSum += -converter->F_t[0][1];
        }
        else {
            const auto &clutch = m_transmission->getClutchConstraint();
            m_frameCouplingTorqueSum += clutch.F_t[0][0];
            m_frameTurbineTorqueSum += -clutch.F_t[0][1];
        }
    }
    ++m_frameStepsTaken;

    writeToSynthesizer();

    ++m_currentIteration;
    return true;
}

double Simulator::getTotalExhaustFlow() const {
    return 0.0;
}

double Simulator::getFrameExhaustFlowRate() const {
    if (m_frameStepsTaken == 0) return 0.0;

    // True volumetric rate: the frame's total exhaust volume over the frame's
    // actual simulated duration (steps x substep dt), NOT the substep dt —
    // the chambers accumulate volume per solver step, so dividing by the
    // substep timestep alone would overstate the rate by the step count.
    return m_frameExhaustVolume / (m_frameStepsTaken * getTimestep());
}

double Simulator::getFrameCouplingTorque() const {
    return (m_frameStepsTaken > 0)
        ? m_frameCouplingTorqueSum / m_frameStepsTaken
        : 0.0;
}

double Simulator::getFrameTurbineTorque() const {
    return (m_frameStepsTaken > 0)
        ? m_frameTurbineTorqueSum / m_frameStepsTaken
        : 0.0;
}

int Simulator::readAudioOutput(int samples, int16_t *target) {
    return m_synthesizer.readAudioOutput(samples, target);
}

void Simulator::endFrame() {
    m_synthesizer.endInputBlock();
}

void Simulator::destroy() {
    m_synthesizer.destroy();
    if (m_dynoTorqueSamples != nullptr) {
        delete[] m_dynoTorqueSamples;
        m_dynoTorqueSamples = nullptr;
    }
}

void Simulator::startAudioRenderingThread() {
    m_synthesizer.startAudioRenderingThread();
}

void Simulator::endAudioRenderingThread() {
    m_synthesizer.endAudioRenderingThread();
}

double Simulator::getSynthesizerInputLatencyTarget() const {
    return m_targetSynthesizerLatency;
}

double Simulator::getFilteredDynoTorque() const {
    if (m_dynoTorqueSamples == nullptr) return 0;

    double averageTorque = 0;
    for (int i = 0; i < DynoTorqueSamples; ++i) {
        averageTorque += m_dynoTorqueSamples[i];
    }

    return averageTorque / DynoTorqueSamples;
}

double Simulator::getDynoPower() const {
    return (m_engine != nullptr)
        ? getFilteredDynoTorque() * m_engine->getSpeed()
        : 0;
}

double Simulator::getAverageOutputSignal() const {
    return 0.0;
}

#include <iostream>
// ANSI color codes for terminal output
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_RESET   "\x1b[0m"

void Simulator::initializeSynthesizer() {
    // Skip if already initialized with proper parameters
    // (Bridge initializes synthesizer with correct config before calling loadSimulation)
    if (m_synthesizer.m_inputChannels != nullptr) {
        std::cout << ANSI_COLOR_YELLOW << "WARNING: Synthesizer already initialized, reinitializing" << ANSI_COLOR_RESET << std::endl;
        m_synthesizer.destroy();
    }

    Synthesizer::Parameters synthParams;
    synthParams.audioBufferSize = 44100;
    synthParams.audioSampleRate = 44100;
    synthParams.inputBufferSize = 44100;
    synthParams.inputChannelCount = m_engine->getExhaustSystemCount();
    synthParams.inputSampleRate = static_cast<float>(getSimulationFrequency());
    m_synthesizer.initialize(synthParams);
}

void Simulator::simulateStep_() {
}

void Simulator::updateFilteredEngineSpeed(double dt) {
    // First-order low-pass on engine rpm, tau in SECONDS. This models the
    // realistic tachometer/ECU sensor every real vehicle broadcasts: crank
    // ripple (combustion firing at 30-60+ Hz) is out-of-band for a tach and
    // is filtered before display/broadcast. The previous form,
    // alpha = dt / (100 + dt), was written for dt in MILLISECONDS but
    // receives SECONDS, making alpha ~2e-4 — effectively no filtering.
    // alpha = dt / (tau + dt) is the exact discrete equivalent of the
    // continuous first-order filter for a zero-order-held sample.
    constexpr double kEngineSpeedFilterTauS = 0.1;
    const double rpm = m_engine->getRpm();
    if (m_filteredEngineSpeed == 0.0 && rpm > 0.0) {
        // Seed with the first sample so the filter does not ramp from 0
        // through the cranking spin-up (a sensor reads instantly on power-up).
        m_filteredEngineSpeed = rpm;
        return;
    }
    const double alpha = dt / (kEngineSpeedFilterTauS + dt);
    m_filteredEngineSpeed += alpha * (rpm - m_filteredEngineSpeed);
}
