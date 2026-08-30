#include "../include/torque_converter.h"

#include "../include/units.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace {
    // MathWorks-standard torque-converter characteristic (salvaged verbatim).
    // Speed ratio SR = w_turbine / w_impeller; torque ratio TR = T_out / T_in.
    // The curve is authored for a 2.0 stall ratio and rescaled in
    // buildTorqueRatioTable() for other stall ratios.
    constexpr int ReferenceTableSize = 16;

    constexpr double ReferenceSpeedRatio[ReferenceTableSize] = {
        0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
        0.8, 0.85, 0.90, 0.92, 0.94, 0.96, 0.97, 1.0
    };

    constexpr double ReferenceTorqueRatio[ReferenceTableSize] = {
        2.0, 1.85, 1.70, 1.55, 1.40, 1.25, 1.12, 1.02,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.0
    };

    // Stall ratio the reference curve above was authored against.
    constexpr double ReferenceStallTorqueRatio = 2.0;

    // Speed ratio below which lockup is force-released, independent of RPM.
    // Keeps the clutch out during launch even if the engine is spinning fast.
    constexpr double LockupReleaseSpeedRatioBand = 0.05;
}

TorqueConverter::TorqueConverter() : atg_scs::Constraint(1, 2) {
    m_stallTorqueRatio = 2.0;
    m_capacityFactor = 1.8e-5;
    m_lockupRpm = 1500.0;
    m_maxInputTorque = 1250.0;
    m_lockupSpeedRatio = 0.85;
    m_lockupHysteresisRpm = 150.0;
    m_lockupEnabled = true;

    m_inputRpm = 0.0;
    m_outputRpm = 0.0;
    m_capacityScale = 0.0;
    m_lockupEngaged = false;
    m_lockupBlend = 0.0;
    m_lockupBlendTimeS = 0.25;
    m_lockupReleaseTimeS = 0.03;

    // atg_scs::Constraint's constructor clears m_bodies with
    // `memset(m_bodies, 0, sizeof(int) * MaxBodyCount)` — 8 bytes for what is a
    // 16-byte pointer array on 64-bit targets, so m_bodies[1] is left
    // indeterminate. Null both slots explicitly; calculate() asserts on them.
    m_bodies[0] = nullptr;
    m_bodies[1] = nullptr;

    buildTorqueRatioTable();
}

TorqueConverter::~TorqueConverter() {
    /* void */
}

void TorqueConverter::initialize(const Parameters &params) {
    m_stallTorqueRatio = std::max(params.StallTorqueRatio, 1.0);
    m_capacityFactor = std::max(params.CapacityFactor, 0.0);
    m_lockupRpm = std::max(params.LockupRpm, 0.0);
    m_maxInputTorque = std::max(params.MaxInputTorque, 0.0);
    m_lockupSpeedRatio = std::clamp(params.LockupSpeedRatio, 0.0, 1.0);
    m_lockupHysteresisRpm = std::max(params.LockupHysteresisRpm, 0.0);
    m_lockupBlendTimeS = std::max(params.LockupBlendTimeS, 1e-3);
    m_lockupReleaseTimeS = std::max(params.LockupReleaseTimeS, 1e-3);
    m_lockupEnabled = params.LockupEnabled;

    buildTorqueRatioTable();
}

void TorqueConverter::buildTorqueRatioTable() {
    // Resample the 16-point reference curve onto a uniform grid so the hot path
    // is an index-and-lerp instead of a search.
    for (int i = 0; i < TableResolution; ++i) {
        const double speedRatio =
            static_cast<double>(i) / static_cast<double>(TableResolution - 1);

        double referenceRatio = 1.0;
        for (int j = 0; j < ReferenceTableSize - 1; ++j) {
            const double lo = ReferenceSpeedRatio[j];
            const double hi = ReferenceSpeedRatio[j + 1];
            if (speedRatio >= lo && speedRatio <= hi) {
                const double t = (speedRatio - lo) / (hi - lo);
                referenceRatio =
                    ReferenceTorqueRatio[j] * (1.0 - t)
                    + ReferenceTorqueRatio[j + 1] * t;
                break;
            }
        }

        // Rescale the curve's multiplication band (which spans 1.0 ..
        // ReferenceStallTorqueRatio) onto 1.0 .. m_stallTorqueRatio.
        const double referenceBand = ReferenceStallTorqueRatio - 1.0;
        const double scaled =
            1.0
            + (referenceRatio - 1.0) * (m_stallTorqueRatio - 1.0) / referenceBand;

        m_torqueRatioTable[i] = std::max(scaled, 1.0);
    }
}

double TorqueConverter::lookupTorqueRatio(double speedRatio) const {
    const double clamped = std::clamp(speedRatio, 0.0, 1.0);
    const double index = clamped * static_cast<double>(TableResolution - 1);
    const int i0 = static_cast<int>(index);
    const int i1 = std::min(i0 + 1, TableResolution - 1);
    const double frac = index - static_cast<double>(i0);

    return m_torqueRatioTable[i0] * (1.0 - frac) + m_torqueRatioTable[i1] * frac;
}

void TorqueConverter::updateRpm(double inputRpm, double outputRpm) {
    m_inputRpm = std::max(inputRpm, 0.0);
    m_outputRpm = std::max(outputRpm, 0.0);

    updateLockup();
}

void TorqueConverter::advanceLockupBlend(double dt) {
    // The apply blend is a pure TIME ramp toward the (hysteretic) lockup state:
    // it never feeds engine rpm back into the capacity, so it cannot add a
    // feedback loop — it only bounds how fast the discrete lockup state may
    // change the converter's torque ceiling. ASYMMETRIC by design: applying is
    // progressive (LockupBlendTimeS, a real clutch's apply ramp), releasing is
    // fast (LockupReleaseTimeS — the release fires exactly when the box wants
    // the fluid back, and lingering rigidity through that transition yanks the
    // engine to the new road speed in one frame). dt <= 0 leaves the blend
    // untouched (the owner may call this before the first solve).
    if (dt <= 0.0) return;

    const double rate = m_lockupEngaged ? (1.0 / m_lockupBlendTimeS)
                                        : (1.0 / m_lockupReleaseTimeS);
    const double target = m_lockupEngaged ? 1.0 : 0.0;
    const double maxStep = dt * rate;
    const double error = target - m_lockupBlend;
    m_lockupBlend += std::clamp(error, -maxStep, maxStep);
}

void TorqueConverter::updateLockup() {
    // Hysteretic lockup, keyed on the TURBINE (output/road) speed — NOT the
    // impeller. The turbine is the exogenous side (road-pinned in the twin, or
    // the drivetrain mass otherwise); the impeller is the side whose speed is
    // SET by this constraint's torque balance. An impeller-fed lock state
    // feeds back: at cruise the engine hunts across the engage/release floor
    // (release collapses the capacity to the pump law, the engine flares up,
    // lockup re-engages and yanks it back down) — a self-sustained limit
    // cycle. Keying the state on the turbine breaks that loop by construction
    // and matches the road-side lockup authority the twin's governor already
    // assumes: one owner (the road) for both the pressure ramp and the lock
    // decision. Release on either the turbine RPM floor minus the hysteresis
    // band, or a clear drop in speed ratio. Without the two-sided band the
    // clutch chatters on and off every solver step.
    const double speedRatio = getSpeedRatio();

    bool engaged = false;
    if (!m_lockupEnabled) {
        engaged = false;
    }
    else if (m_lockupEngaged) {
        const bool rpmReleased =
            m_outputRpm < (m_lockupRpm - m_lockupHysteresisRpm);
        const bool slipReleased =
            speedRatio < (m_lockupSpeedRatio - LockupReleaseSpeedRatioBand);
        engaged = !(rpmReleased || slipReleased);
    }
    else {
        engaged =
            m_outputRpm >= m_lockupRpm
            && speedRatio >= m_lockupSpeedRatio;
    }

    m_lockupEngaged = engaged;
}

double TorqueConverter::getTorqueRatio() const {
    // Locked up, the converter is a solid 1:1 shaft — no multiplication.
    return m_lockupEngaged ? 1.0 : lookupTorqueRatio(getSpeedRatio());
}

double TorqueConverter::getSpeedRatio() const {
    return (m_inputRpm > 0.0) ? std::clamp(m_outputRpm / m_inputRpm, 0.0, 1.0) : 0.0;
}

double TorqueConverter::getSlip() const {
    return 1.0 - getSpeedRatio();
}

double TorqueConverter::getMaxInputTorque() const {
    // Rated capacity: the K * N^2 pump law, clamped to the converter's ceiling.
    // This is the RATED figure and deliberately ignores m_capacityScale so the
    // value stays a pure function of the parameters; calculate() applies the
    // scale when it builds the solver limits.
    double capacity = 0.0;
    if (m_inputRpm > 0.0) {
        capacity = std::min(
            m_capacityFactor * m_inputRpm * m_inputRpm,
            m_maxInputTorque);
    }

    return capacity;
}

void TorqueConverter::setCapacityScale(double scale) {
    m_capacityScale = std::clamp(scale, 0.0, 1.0);
}

void TorqueConverter::setImpellerDirection(double direction) {
    // Anything non-negative keeps the historical +1 pairing; only an explicit
    // negative pairs the CW engine with the forward driveline. Clamped to the
    // two valid values so a stray magnitude can never scale the row.
    m_impellerDirection = (direction < 0.0) ? -1.0 : 1.0;
}

void TorqueConverter::setStallTorqueRatio(double ratio) {
    m_stallTorqueRatio = std::max(ratio, 1.0);
    buildTorqueRatioTable();
}

void TorqueConverter::setCapacityFactor(double factor) {
    m_capacityFactor = std::max(factor, 0.0);
}

void TorqueConverter::setLockupRpm(double rpm) {
    m_lockupRpm = std::max(rpm, 0.0);
}

void TorqueConverter::setMaxInputTorque(double torque) {
    m_maxInputTorque = std::max(torque, 0.0);
}

void TorqueConverter::setLockupEnabled(bool enabled) {
    m_lockupEnabled = enabled;
}

void TorqueConverter::calculate(Output *output, atg_scs::SystemState *state) {
    (void)state;

    // Both bodies are an internal contract established by Transmission::
    // addToSystem. A missing body is a wiring bug, not a runtime condition.
    assert(output != nullptr);
    assert(m_bodies[0] != nullptr && "TorqueConverter impeller body not wired");
    assert(m_bodies[1] != nullptr && "TorqueConverter turbine body not wired");

    const double impellerRpm = std::abs(units::toRpm(m_bodies[0]->v_theta));
    const double turbineRpm = std::abs(units::toRpm(m_bodies[1]->v_theta));
    updateRpm(impellerRpm, turbineRpm);

    const double torqueRatio = getTorqueRatio();

    // Row 0 mirrors ClutchConstraint's [-1, +1] pairing with the torque ratio
    // folded into the turbine column, so lambda is the impeller-side torque and
    // the turbine receives D * TR * lambda. The D factor pairs the shafts
    // THROUGH the gearbox's direction: a CW (negative) engine in a forward
    // (positive-driveline) car must satisfy w_imp = -TR * w_turb, not
    // w_imp = +TR * w_turb. Without it the row is sign-blind: lockup keyed on
    // |SR| engages across a direction mismatch and the solver yanks the crank
    // through zero into reverse rotation — a reverse-spinning 4-stroke reads
    // its exhaust port as deep negative flow and goes silent (the family-B
    // reversion lock). See the header for the derivation.
    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = -1.0;

    output->J[0][3] = 0.0;
    output->J[0][4] = 0.0;
    output->J[0][5] = m_impellerDirection * torqueRatio;

    output->J_dot[0][0] = 0.0;
    output->J_dot[0][1] = 0.0;
    output->J_dot[0][2] = 0.0;

    output->J_dot[0][3] = 0.0;
    output->J_dot[0][4] = 0.0;
    output->J_dot[0][5] = 0.0;

    // Written for completeness only — OptimizedNsvRigidBodySystem never reads
    // ks/kd. Matching ClutchConstraint's values keeps the constraint sane if it
    // is ever run under GenericRigidBodySystem, which does read them.
    output->ks[0] = 10.0;
    output->kd[0] = 1.0;

    // Velocity-level constraint with no positional error and no target offset:
    // slip emerges from the solver clamping lambda at the capacity limit, in
    // exactly the way a slipping ClutchConstraint produces slip.
    output->C[0] = 0.0;
    output->v_bias[0] = 0.0;

    // Capacity window. Locked up, the clutch carries the converter's full rated
    // ceiling; open, it carries the K * N^2 pump capacity. The lockup APPLY
    // BLEND interpolates between the two over LockupBlendTimeS: stepping the
    // ceiling in one frame lets the solver close hundreds of rpm of slip
    // instantly (a violent one-frame snap), while the ramp makes engagement a
    // progressive, torque-limited convergence — how a real lockup clutch
    // applies. Either way the owning Transmission scales the result to release
    // the drivetrain during a shift.
    const double ratedTorque =
        getMaxInputTorque()
        + (m_maxInputTorque - getMaxInputTorque()) * m_lockupBlend;
    const double capacity = std::max(ratedTorque * m_capacityScale, 0.0);

    // Symmetric limits. NOTE for review: a real open converter has markedly
    // lower capacity on overrun (turbine driving impeller) and does not
    // multiply torque in that direction. Symmetric is a deliberate first
    // approximation here, not a claim of fidelity.
    output->limits[0][0] = -capacity;
    output->limits[0][1] = capacity;
}
