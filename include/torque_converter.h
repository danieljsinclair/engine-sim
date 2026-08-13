#ifndef ATG_ENGINE_SIM_TORQUE_CONVERTER_H
#define ATG_ENGINE_SIM_TORQUE_CONVERTER_H

#include "scs.h"

// Fluid-coupling torque converter, modelled as a TWO-body constraint between
// the engine output crankshaft (impeller, body 0) and the transmission's
// rotating mass (turbine, body 1).
//
// Solver contract
// ---------------
// The simulator runs atg_scs::OptimizedNsvRigidBodySystem (see simulator.cpp).
// That system reads J / J_dot / C / v_bias / limits from Constraint::Output and
// NEVER reads ks / kd (only GenericRigidBodySystem does). Therefore the torque
// relation MUST be expressed through limits[], which the Gauss-Seidel solver
// projects onto every iteration. Any ks/kd based "soft constraint" is dead code
// under this solver.
//
// Formulation
// -----------
// Row 0 is the fluid coupling, written in the same style as ClutchConstraint
// (J = [-1, +1]) but with the torque ratio folded into the turbine column:
//
//     J = [ 0 0 -1 | 0 0 TR ]      =>   C_dot = -w_imp + TR * w_turb
//
// Virtual work then gives:
//
//     torque(impeller) = -lambda        (reaction drag on the engine)
//     torque(turbine)  = +TR * lambda   (multiplied output torque)
//
// so lambda IS the impeller-side torque, and power is conserved when the row is
// satisfied: lambda * (TR*w_turb - w_imp) = 0. Constraining lambda to the K*N^2
// capacity window is exactly the physical statement "the converter can only
// pump this much torque"; when the solver clamps lambda the row goes unsatisfied
// and the residual appears as slip. That is the same mechanism a slipping
// ClutchConstraint uses, which is why v_bias and C stay at zero here.

class TorqueConverter : public atg_scs::Constraint {
    public:
        struct Parameters {
            // Torque ratio at stall (speed ratio 0). Scales the reference curve.
            double StallTorqueRatio = 2.0;
            // Capacity coefficient K in T_max = K * N_impeller^2, Nm per RPM^2.
            double CapacityFactor = 1.8e-5;
            // Impeller speed at or above which the lockup clutch may engage, RPM.
            double LockupRpm = 1500.0;
            // Hard ceiling on transmitted impeller torque, Nm.
            double MaxInputTorque = 1250.0;
            // Speed ratio at or above which lockup may engage (near coupling).
            double LockupSpeedRatio = 0.85;
            // Release band below LockupRpm, RPM. Prevents lockup chatter.
            double LockupHysteresisRpm = 150.0;
            // Master enable for the lockup clutch.
            bool LockupEnabled = true;
        };

    public:
        TorqueConverter();
        virtual ~TorqueConverter();

        void initialize(const Parameters &params);

        // Body wiring. Impeller is the engine side, turbine the gearbox side.
        void setImpeller(atg_scs::RigidBody *body) { m_bodies[0] = body; }
        void setTurbine(atg_scs::RigidBody *body) { m_bodies[1] = body; }

        // Refresh cached impeller/turbine speeds (RPM, magnitude) and re-evaluate
        // the lockup state machine. Called from calculate() and may also be
        // called by the owner for telemetry between solver steps.
        void updateRpm(double inputRpm, double outputRpm);

        // Telemetry accessors.
        double getTorqueRatio() const;
        double getSpeedRatio() const;
        double getSlip() const;
        double getMaxInputTorque() const;
        double getInputRpm() const { return m_inputRpm; }
        double getOutputRpm() const { return m_outputRpm; }
        bool isLockupEngaged() const { return m_lockupEngaged; }

        // Fraction of rated capacity currently available, 0..1. The owning
        // Transmission drives this so the converter opens during a gear change.
        void setCapacityScale(double scale);
        double getCapacityScale() const { return m_capacityScale; }

        // Configuration.
        void setStallTorqueRatio(double ratio);
        void setCapacityFactor(double factor);
        void setLockupRpm(double rpm);
        void setMaxInputTorque(double torque);
        void setLockupEnabled(bool enabled);

        virtual void calculate(Output *output, atg_scs::SystemState *state) override;

    protected:
        void buildTorqueRatioTable();
        double lookupTorqueRatio(double speedRatio) const;
        void updateLockup();

        double m_stallTorqueRatio;
        double m_capacityFactor;
        double m_lockupRpm;
        double m_maxInputTorque;
        double m_lockupSpeedRatio;
        double m_lockupHysteresisRpm;
        bool m_lockupEnabled;

        double m_inputRpm;
        double m_outputRpm;
        double m_capacityScale;
        bool m_lockupEngaged;

        // Torque ratio resampled onto a uniform speed-ratio grid.
        static constexpr int TableResolution = 256;
        double m_torqueRatioTable[TableResolution];
};

#endif /* ATG_ENGINE_SIM_TORQUE_CONVERTER_H */
