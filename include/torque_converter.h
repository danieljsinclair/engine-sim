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
            // Turbine (output/road) speed at or above which the lockup clutch
            // may engage, RPM. Keyed on the TURBINE, not the impeller: the
            // turbine is the exogenous side, so the lock state cannot feed
            // back through the engine's torque balance (see updateLockup()).
            double LockupRpm = 1500.0;
            // Hard ceiling on transmitted impeller torque, Nm.
            double MaxInputTorque = 1250.0;
            // Speed ratio at or above which lockup may engage (near coupling).
            double LockupSpeedRatio = 0.85;
            // Release band below LockupRpm (turbine side), RPM. Prevents
            // lockup chatter.
            double LockupHysteresisRpm = 150.0;
            // Time over which the lockup clutch's rated torque ramps between the
            // fluid figure (K * N^2) and the locked ceiling (MaxInputTorque),
            // seconds. A real lockup clutch APPLIES progressively; stepping the
            // rated torque in one solver frame yanks a slipping engine to sync
            // instantly (a several-hundred-rpm one-frame snap).
            double LockupBlendTimeS = 0.25;
            // Release-side blend time, seconds. Release must be FAST: the
            // release fires exactly when the box wants the fluid back (a
            // downshift, a lug), and holding lockup rigidity through that
            // transition yanks the engine to the NEW road-implied speed in one
            // frame. Apply is progressive, release is near-immediate — the
            // asymmetry a real lockup clutch has.
            double LockupReleaseTimeS = 0.03;
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

        // Advance the lockup APPLY BLEND toward the engaged state at the rate
        // given by LockupBlendTimeS. The blend interpolates the rated torque
        // between the fluid capacity (K * N^2) and the locked ceiling, so the
        // lockup clutch applies progressively like real hardware instead of
        // stepping the capacity in one frame. Called by the owning Transmission
        // once per simulation step (it carries the dt the constraint interface
        // does not).
        void advanceLockupBlend(double dt);

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
        // Pair the converter with the engine's design rotation (see Engine::
        // getDesignRotationDirection). The owning Transmission calls this at
        // wiring time, before the first solver step; D = -1 pairs a CW
        // (negative-v_theta) engine with a forward (positive) driveline. The
        // default (+1) is the historical sign-blind pairing, kept for
        // stand-alone use without an engine.
        void setImpellerDirection(double direction);
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

        // Driveline direction pairing. The constraint row ties
        // w_imp = D * TR * w_turb; D is the sign that maps the engine's
        // design rotation onto the driveline's forward direction, seeded
        // from the engine at wiring time (setImpellerDirection). A CCW
        // (positive-v_theta) engine keeps D = +1 — the historical pairing.
        double m_impellerDirection = 1.0;

        // 0 = fluid capacity only, 1 = full locked ceiling. Tracks
        // m_lockupEngaged at the LockupBlendTimeS rate (see
        // advanceLockupBlend).
        double m_lockupBlend;
        double m_lockupBlendTimeS;
        double m_lockupReleaseTimeS;

        // Torque ratio resampled onto a uniform speed-ratio grid.
        static constexpr int TableResolution = 256;
        double m_torqueRatioTable[TableResolution];
};

#endif /* ATG_ENGINE_SIM_TORQUE_CONVERTER_H */
