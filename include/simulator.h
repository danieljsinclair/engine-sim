#ifndef ATG_ENGINE_SIM_SIMULATOR_H
#define ATG_ENGINE_SIM_SIMULATOR_H

#include "engine.h"
#include "transmission.h"
#include "vehicle.h"
#include "synthesizer.h"
#include "dynamometer.h"
#include "starter_motor.h"
#include "derivative_filter.h"
#include "vehicle_drag_constraint.h"
#include "delay_filter.h"
#include "engine.h"

#include <chrono>

class Simulator {
public:
    enum class SystemType {
        NsvOptimized,
        Generic
    };

    struct Parameters {
        SystemType systemType = SystemType::NsvOptimized;
    };

    static constexpr int DynoTorqueSamples = 512;

public:
    Simulator();
    virtual ~Simulator();

    virtual void initialize(const Parameters &params);
    virtual void loadSimulation(Engine *engine, Vehicle *vehicle, Transmission *transmission);
    void releaseSimulation();

    virtual void startFrame(real_t dt);
    bool simulateStep();
    virtual real_t getTotalExhaustFlow() const;
    int readAudioOutput(int samples, int16_t *target);
    virtual void endFrame();
    virtual void destroy();

    void startAudioRenderingThread();
    void endAudioRenderingThread();

    int getFrameIterationCount() const { return m_steps; }

    Synthesizer &synthesizer() { return m_synthesizer; }

    Engine *getEngine() const { return m_engine; }
    Transmission *getTransmission() const { return m_transmission; }
    Vehicle *getVehicle() const { return m_vehicle; }
    atg_scs::RigidBodySystem *getSystem() { return m_system; }

    void setSimulationFrequency(int frequency) { m_simulationFrequency = frequency; }
    int getSimulationFrequency() const { return m_simulationFrequency; }

    virtual void setFluidSimulationSteps(int steps) { (void)steps; }
    virtual int getFluidSimulationSteps() const { return 0; }

    real_t getTimestep() const { return 1.0 / m_simulationFrequency; }

    void setTargetSynthesizerLatency(real_t latency) { m_targetSynthesizerLatency = latency; }
    real_t getTargetSynthesizerLatency() const { return m_targetSynthesizerLatency; }
    real_t getSynthesizerInputLatency() const { return m_synthesizer.getLatency(); }
    real_t getSynthesizerInputLatencyTarget() const;

    void setSimulationSpeed(real_t simSpeed) { m_simulationSpeed = simSpeed; }
    real_t getSimulationSpeed() const { return m_simulationSpeed; }
    int getCurrentIteration() const { return m_currentIteration; }
    real_t getAverageProcessingTime() const { return m_physicsProcessingTime; }

    int simulationSteps() const { return m_steps; }

    virtual real_t getFilteredDynoTorque() const;
    virtual real_t getDynoPower() const;
    virtual real_t getAverageOutputSignal() const;

    real_t filteredEngineSpeed() const { return m_filteredEngineSpeed; }

    Dynamometer m_dyno;
    StarterMotor m_starterMotor;

protected:
    void initializeSynthesizer();
    virtual void simulateStep_();
    virtual void writeToSynthesizer() = 0;

    atg_scs::RigidBodySystem *m_system;

    // Engine/vehicle/transmission stored via loadSimulation().
    // Ownership remains with the caller — Simulator does not delete these.
    Engine *m_engine;
    Transmission *m_transmission;
    Vehicle *m_vehicle;

    // Physics rigid body and drag constraint for vehicle simulation.
    // Used by subclasses in loadSimulation() for addToSystem() wiring.
    atg_scs::RigidBody m_vehicleMass;
    VehicleDragConstraint m_vehicleDrag;

private:
    void updateFilteredEngineSpeed(real_t dt);

    Synthesizer m_synthesizer;

    std::chrono::steady_clock::time_point m_simulationStart;
    std::chrono::steady_clock::time_point m_simulationEnd;
    int m_currentIteration;

    real_t m_physicsProcessingTime;

    int m_simulationFrequency;

    real_t m_targetSynthesizerLatency;
    real_t m_simulationSpeed;

    real_t *m_dynoTorqueSamples;
    int m_lastDynoTorqueSample;

    real_t m_filteredEngineSpeed;

    int m_steps;
};

#endif /* ATG_ENGINE_SIM_SIMULATOR_H */
