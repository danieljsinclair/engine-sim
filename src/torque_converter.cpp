#include "../include/torque_converter.h"
#include <cmath>
#include <algorithm>
#include <cassert>

// Speed ratio vector for TR lookup (matches MathWorks default)
static const double s_speedRatioTable[] = {
    0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.90, 0.92, 0.94, 0.96, 0.97, 1.0
};
static const double s_torqueRatioTable[] = {
    2.0, 1.85, 1.70, 1.55, 1.40, 1.25, 1.12, 1.02, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.0
};
static constexpr int s_tableSize = 16;

atg_scs::TorqueConverter::TorqueConverter() : Constraint(1, 1) {
    m_stallTorqueRatio = 2.0;
    m_capacityFactor = 1.8e-5;
    m_lockupRpm = 1500.0;
    m_maxInputTorque = 1250.0;
    m_inputRpm = 0.0;
    m_outputRpm = 0.0;
    buildTorqueRatioTable();
}

atg_scs::TorqueConverter::~TorqueConverter() {}

void atg_scs::TorqueConverter::initialize(const Parameters &params) {
    m_stallTorqueRatio = params.stallTorqueRatio;
    m_capacityFactor = params.capacityFactor;
    m_lockupRpm = params.lockupRpm;
    m_maxInputTorque = params.maxInputTorque;
    buildTorqueRatioTable();
}

void atg_scs::TorqueConverter::buildTorqueRatioTable() {
    for (int i = 0; i < 256; ++i) {
        double speedRatio = static_cast<double>(i) / 255.0;
        // Interpolate from the reference table
        double torqueRatio = 1.0;
        for (int j = 0; j < s_tableSize - 1; ++j) {
            if (speedRatio >= s_speedRatioTable[j] && speedRatio <= s_speedRatioTable[j + 1]) {
                double t = (speedRatio - s_speedRatioTable[j]) /
                           (s_speedRatioTable[j + 1] - s_speedRatioTable[j]);
                torqueRatio = s_torqueRatioTable[j] * (1.0 - t) + s_torqueRatioTable[j + 1] * t;
                break;
            }
        }
        // Scale by stall torque ratio (reference table assumes 2.0)
        torqueRatio = 1.0 + (torqueRatio - 1.0) * (m_stallTorqueRatio - 1.0) / 1.0;
        m_torqueRatioTable[i] = std::max(torqueRatio, 1.0);
    }
}

double atg_scs::TorqueConverter::lookupTorqueRatio(double speedRatio) const {
    double clamped = std::clamp(speedRatio, 0.0, 1.0);
    double index = clamped * 255.0;
    int i0 = static_cast<int>(index);
    int i1 = std::min(i0 + 1, 255);
    double frac = index - i0;
    return m_torqueRatioTable[i0] * (1.0 - frac) + m_torqueRatioTable[i1] * frac;
}

void atg_scs::TorqueConverter::updateRpm(double inputRpm, double outputRpm) {
    m_inputRpm = std::max(inputRpm, 0.0);
    m_outputRpm = std::max(outputRpm, 0.0);
}

double atg_scs::TorqueConverter::getTorqueRatio() const {
    double speedRatio = (m_inputRpm > 0.0) ? m_outputRpm / m_inputRpm : 0.0;
    return lookupTorqueRatio(speedRatio);
}

double atg_scs::TorqueConverter::getSpeedRatio() const {
    return (m_inputRpm > 0.0) ? m_outputRpm / m_inputRpm : 0.0;
}

double atg_scs::TorqueConverter::getSlip() const {
    return 1.0 - getSpeedRatio();
}

double atg_scs::TorqueConverter::getMaxInputTorque() const {
    if (m_inputRpm <= 0.0) return 0.0;
    double capacity = m_capacityFactor * m_inputRpm * m_inputRpm;
    return std::min(capacity, m_maxInputTorque);
}

void atg_scs::TorqueConverter::setStallTorqueRatio(double ratio) {
    m_stallTorqueRatio = std::max(ratio, 1.0);
    buildTorqueRatioTable();
}

void atg_scs::TorqueConverter::setCapacityFactor(double factor) { m_capacityFactor = factor; }
void atg_scs::TorqueConverter::setLockupRpm(double rpm) { m_lockupRpm = rpm; }
void atg_scs::TorqueConverter::setMaxInputTorque(double torque) { m_maxInputTorque = torque; }

void atg_scs::TorqueConverter::calculate(Output *output, SystemState *state) {
    (void)state;

    // Read angular velocities from constraint bodies
    double inputOmega = m_bodies[0]->v_theta;   // engine side (impeller)
    double outputOmega = m_bodies[1]->v_theta;  // transmission side (turbine)

    // Convert to RPM (magnitude only)
    const double radPerSecToRpm = 30.0 / 3.14159265358979;
    double inputRpm = std::abs(inputOmega) * radPerSecToRpm;
    double outputRpm = std::abs(outputOmega) * radPerSecToRpm;

    // Update internal state
    updateRpm(inputRpm, outputRpm);

    // Compute speed ratio and torque ratio
    double speedRatio = getSpeedRatio();
    double torqueRatio = getTorqueRatio();

    // Compute capacity (RPM-squared law)
    double capacity = getMaxInputTorque();

    // Set up the constraint as a velocity-bias constraint
    // The TC allows slip but limits the torque that can be transmitted
    output->J[0][0] = 0;
    output->J[0][1] = 0;
    output->J[0][2] = 1;   // rotation axis (impeller)
    output->J[0][3] = 0;
    output->J[0][4] = 0;
    output->J[0][5] = -torqueRatio;  // turbine side, scaled by torque ratio

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;
    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    output->J_dot[0][5] = 0;

    // Soft constraint: allow slip but resist it
    // At stall (speedRatio=0), the constraint is softest (most slip allowed)
    // At coupling (speedRatio~0.9), the constraint stiffens
    double slip = 1.0 - speedRatio;
    double ks = 1000.0 * slip;  // stiffness proportional to slip
    double kd = 100.0;

    output->ks[0] = ks;
    output->kd[0] = kd;
    output->C[0] = 0;

    // Torque limits: the TC can transmit up to `capacity` from impeller
    // and `capacity * torqueRatio` to turbine
    double maxTorque = capacity;
    output->limits[0][0] = -maxTorque;
    output->limits[0][1] = maxTorque;

    // Bias velocity: drive toward zero slip (but allow it)
    // The bias is proportional to the current slip
    double targetSlip = 0.0;  // ideally no slip
    output->v_bias[0] = (targetSlip - slip) * inputOmega * 0.1;
}
