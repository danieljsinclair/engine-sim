// afterfire_tests.cpp — behaviour-contract tests for the exhaust afterfire
// ("pop on overrun") feature on the live physics-based model.
//
// Intent ported from the backfires-wip snapshot (df4f543:test/afterfire_tests.cpp),
// but rewritten for the current API: the model is now manifold-pressure misfire
// -> Arrhenius auto-ignition -> scavenging reset, NOT the old scheduler
// (intensity/probability/cooldown) design. See combustion_chamber.h.
//
// These are API-level contracts that hold for a default-constructed chamber
// (no wired engine). The physics OUTCOMES (actual pops under overrun) require a
// fully wired engine and are covered by the bridge integration test
// (AfterfireBridgeTest), which drives SimulatorFactory -> BridgeSimulator.
//
// Guarded so the suite still links when ATG_ENGINE_SIM_AFTERFIRE_SPIKE is off.

#include <gtest/gtest.h>

#include "../include/combustion_chamber.h"
#include "../include/units.h"

#ifdef ATG_ENGINE_SIM_AFTERFIRE_SPIKE

TEST(Afterfire, DisabledByDefault) {
    CombustionChamber cc;
    EXPECT_FALSE(cc.isAfterfireEnabled());
    EXPECT_EQ(cc.getAfterfireEventCount(), 0);
    EXPECT_NEAR(cc.getLastAfterfirePeakPressure(), 0.0, 1E-9);
    EXPECT_NEAR(cc.getLastAfterfireEnergyReleased(), 0.0, 1E-9);
    EXPECT_NEAR(cc.getAfterfireIgnitionProgress(), 0.0, 1E-9);

    // With afterfire disabled, updateAfterfire is a no-op and must not touch the
    // (unwired) engine. It returns false and leaves all counters at zero.
    const bool fired = cc.updateAfterfire(0.016, 0.0);
    EXPECT_FALSE(fired);
    EXPECT_EQ(cc.getAfterfireEventCount(), 0);
}

TEST(Afterfire, RuntimeToggleMidDrive) {
    CombustionChamber cc;
    cc.enableAfterfire(true);
    EXPECT_TRUE(cc.isAfterfireEnabled());
    cc.enableAfterfire(false);
    EXPECT_FALSE(cc.isAfterfireEnabled());
}

TEST(Afterfire, NoOpWhenDisabled) {
    CombustionChamber cc;
    cc.enableAfterfire(false);

    // A wide-open-throttle call must not register any event while disabled.
    cc.updateAfterfire(0.016, 1.0);
    cc.updateAfterfire(0.016, 0.0);

    EXPECT_EQ(cc.getAfterfireEventCount(), 0);
    EXPECT_NEAR(cc.getLastAfterfirePeakPressure(), 0.0, 1E-9);
    EXPECT_NEAR(cc.getLastAfterfireEnergyReleased(), 0.0, 1E-9);
}

TEST(Afterfire, DiagnosticsDefaultsZero) {
    CombustionChamber cc;
    CombustionChamber::AfterfireDiagnostics d = cc.getAfterfireDiagnostics();
    EXPECT_EQ(d.eventCount, 0);
    EXPECT_EQ(d.skippedTooCold, 0);
    EXPECT_EQ(d.skippedNoFuel, 0);
    EXPECT_EQ(d.skippedNoOxygen, 0);
    EXPECT_EQ(d.skippedThrottle, 0);
    EXPECT_EQ(d.skippedNotReady, 0);
    EXPECT_EQ(d.misfireCycles, 0);
    EXPECT_NEAR(d.maxIgnitionProgress, 0.0, 1E-9);
    EXPECT_NEAR(d.maxRunnerTempK, 0.0, 1E-9);
    EXPECT_NEAR(d.maxRawFuelFraction, 0.0, 1E-9);
    EXPECT_NEAR(d.minManifoldPressure, 0.0, 1E-9);
    EXPECT_NEAR(d.lastEventRpm, 0.0, 1E-9);
    EXPECT_NEAR(d.lastEventThrottle, 0.0, 1E-9);
    EXPECT_NEAR(d.lastEventPeakPressure, 0.0, 1E-9);
    EXPECT_NEAR(d.lastEventEnergyReleased, 0.0, 1E-9);
    EXPECT_NEAR(d.lastEventRunnerTempK, 0.0, 1E-9);
}

TEST(Afterfire, IgnitionProgressStartsZero) {
    CombustionChamber cc;
    EXPECT_NEAR(cc.getAfterfireIgnitionProgress(), 0.0, 1E-9);
}

TEST(Afterfire, SetGetParametersRoundTrip) {
    CombustionChamber cc;
    CombustionChamber::AfterfireParameters p;
    p.enabled = true;
    p.misfireManifoldPressure = units::pressure(0.5, units::atm);
    p.ignitionDelayRefS = 0.01;
    p.activationTempK = 9000.0;
    p.refTempK = 1100.0;
    p.autoIgnitionTempK = 700.0;
    p.minRawFuelFraction = 0.0003;
    p.minOxygenMoleFraction = 0.005;
    p.energyScale = 2.0;
    p.throttleCutoff = 0.15;
    p.diagnostics = true;
    cc.setAfterfireParameters(p);

    CombustionChamber::AfterfireParameters got = cc.getAfterfireParameters();
    EXPECT_TRUE(got.enabled);
    EXPECT_NEAR(got.misfireManifoldPressure, units::pressure(0.5, units::atm), 1E-6);
    EXPECT_NEAR(got.ignitionDelayRefS, 0.01, 1E-9);
    EXPECT_NEAR(got.activationTempK, 9000.0, 1E-9);
    EXPECT_NEAR(got.refTempK, 1100.0, 1E-9);
    EXPECT_NEAR(got.autoIgnitionTempK, 700.0, 1E-9);
    EXPECT_NEAR(got.minRawFuelFraction, 0.0003, 1E-12);
    EXPECT_NEAR(got.minOxygenMoleFraction, 0.005, 1E-12);
    EXPECT_NEAR(got.energyScale, 2.0, 1E-9);
    EXPECT_NEAR(got.throttleCutoff, 0.15, 1E-9);
    EXPECT_TRUE(got.diagnostics);
}

TEST(Afterfire, ResetAfterfireDiagnosticsClearsState) {
    CombustionChamber cc;
    cc.enableAfterfire(true);
    cc.setAfterfireParameters([] {
        CombustionChamber::AfterfireParameters p;
        p.enabled = true;
        p.diagnostics = true;
        return p;
    }());

    // We cannot drive a real pop without a wired engine here, but reset must
    // restore the diagnostics to a clean, zeroed state regardless of history.
    cc.resetAfterfireDiagnostics();
    CombustionChamber::AfterfireDiagnostics d = cc.getAfterfireDiagnostics();
    EXPECT_EQ(d.eventCount, 0);
    EXPECT_EQ(d.skippedThrottle, 0);
    EXPECT_EQ(d.misfireCycles, 0);
    EXPECT_NEAR(d.maxIgnitionProgress, 0.0, 1E-9);
    EXPECT_NEAR(d.lastEventPeakPressure, 0.0, 1E-9);
    EXPECT_NEAR(d.lastEventEnergyReleased, 0.0, 1E-9);
    EXPECT_NEAR(cc.getAfterfireIgnitionProgress(), 0.0, 1E-9);
}

#endif /* ATG_ENGINE_SIM_AFTERFIRE_SPIKE */
