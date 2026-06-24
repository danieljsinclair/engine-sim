#ifndef ATG_ENGINE_SIM_TORQUE_CONVERTER_H
#define ATG_ENGINE_SIM_TORQUE_CONVERTER_H

#include "scs.h"

namespace atg_scs {

class TorqueConverter : public Constraint {
public:
    struct Parameters {
        double stallTorqueRatio = 2.0;    // TR at λ=0 (stall)
        double capacityFactor = 1.8e-5;   // K at stall (Nm/RPM²)
        double lockupRpm = 1500.0;        // Min RPM for lockup
        double maxInputTorque = 1250.0;   // Max torque capacity (Nm)
    };

    TorqueConverter();
    virtual ~TorqueConverter();

    void initialize(const Parameters &params);
    void updateRpm(double inputRpm, double outputRpm);

    // Accessors for telemetry
    double getTorqueRatio() const;
    double getSpeedRatio() const;
    double getSlip() const;
    double getMaxInputTorque() const;
    double getInputRpm() const { return m_inputRpm; }
    double getOutputRpm() const { return m_outputRpm; }

    // Configuration
    void setStallTorqueRatio(double ratio);
    void setCapacityFactor(double factor);
    void setLockupRpm(double rpm);
    void setMaxInputTorque(double torque);

    // Constraint interface
    virtual void calculate(Output *output, SystemState *state) override;

protected:
    double m_stallTorqueRatio;
    double m_capacityFactor;
    double m_lockupRpm;
    double m_maxInputTorque;
    double m_inputRpm;
    double m_outputRpm;

    // Pre-computed torque ratio lookup table (256 entries)
    // Indexed by speedRatio * 255
    double m_torqueRatioTable[256];

    void buildTorqueRatioTable();
    double lookupTorqueRatio(double speedRatio) const;
};

} // namespace atg_scs

#endif
