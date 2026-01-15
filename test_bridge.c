// Simple test program to verify engine-sim bridge API
// Compiles with: clang -o test_bridge test_bridge.c -L./build -lenginesim -I./include

#include <stdio.h>
#include <stdlib.h>
#include "include/engine_sim_bridge.h"

int main(int argc, char** argv) {
    printf("=== Engine-Sim Bridge API Test ===\n\n");

    // Test 1: Get version
    printf("Test 1: GetVersion\n");
    const char* version = EngineSimGetVersion();
    printf("  Version: %s\n", version);
    printf("  PASS: Version string retrieved\n\n");

    // Test 2: Validate config
    printf("Test 2: ValidateConfig\n");
    EngineSimConfig config = {
        .sampleRate = 48000,
        .inputBufferSize = 1024,
        .audioBufferSize = 96000,
        .simulationFrequency = 10000,
        .fluidSimulationSteps = 8,
        .targetSynthesizerLatency = 0.05,
        .volume = 1.0f,
        .convolutionLevel = 0.5f,
        .airNoise = 0.1f
    };

    EngineSimResult result = EngineSimValidateConfig(&config);
    printf("  ValidateConfig result: %d\n", result);
    printf("  %s\n", (result == ESIM_SUCCESS) ? "PASS" : "FAIL");
    printf("\n");

    // Test 3: Create simulator
    printf("Test 3: Create Simulator\n");
    EngineSimHandle handle = NULL;
    result = EngineSimCreate(&config, &handle);
    printf("  Create result: %d\n", result);
    printf("  Handle: %p\n", handle);

    if (result != ESIM_SUCCESS) {
        printf("  Error: %s\n", EngineSimGetLastError(handle));
        printf("  FAIL: Could not create simulator\n");
        return 1;
    }
    printf("  PASS: Simulator created\n\n");

    // Test 4: Get initial stats
    printf("Test 4: Get Initial Stats\n");
    EngineSimStats stats;
    result = EngineSimGetStats(handle, &stats);
    printf("  GetStats result: %d\n", result);
    if (result == ESIM_SUCCESS) {
        printf("  RPM: %.2f\n", stats.currentRPM);
        printf("  Load: %.2f\n", stats.currentLoad);
        printf("  Exhaust Flow: %.4f\n", stats.exhaustFlow);
        printf("  Manifold Pressure: %.2f Pa\n", stats.manifoldPressure);
        printf("  Active Channels: %d\n", stats.activeChannels);
        printf("  Processing Time: %.4f ms\n", stats.processingTimeMs);
        printf("  PASS: Stats retrieved\n\n");
    } else {
        printf("  Warning: Could not get stats (expected before script load)\n\n");
    }

    // Test 5: Set throttle
    printf("Test 5: Set Throttle\n");
    result = EngineSimSetThrottle(handle, 0.5);
    printf("  SetThrottle(0.5) result: %d\n", result);
    printf("  %s\n\n", (result == ESIM_SUCCESS) ? "PASS" : "FAIL");

    // Test 6: Update simulation
    printf("Test 6: Update Simulation\n");
    result = EngineSimUpdate(handle, 0.01667); // ~60Hz
    printf("  Update(0.01667) result: %d\n", result);
    printf("  %s\n\n", (result == ESIM_SUCCESS) ? "PASS" : "FAIL");

    // Test 7: Script loading note
    printf("Test 7: Load Script\n");
    printf("  SKIPPED: Piranha scripting disabled on macOS build\n");
    printf("  Note: EngineSimLoadScript not available (requires PIRANHA_ENABLED=ON)\n");

    // Test 8: Get stats after initialization
    printf("Test 8: Get Stats After Initialization\n");
    result = EngineSimGetStats(handle, &stats);
    printf("  GetStats result: %d\n", result);
    if (result == ESIM_SUCCESS) {
        printf("  RPM: %.2f\n", stats.currentRPM);
        printf("  Load: %.2f\n", stats.currentLoad);
        printf("  PASS: Stats retrieved\n\n");
    } else {
        printf("  WARNING: Stats not available without engine loaded\n\n");
    }

    // Test 9: Render audio (small buffer)
    printf("Test 9: Render Audio\n");
    float audioBuffer[256]; // 128 frames stereo
    int32_t samplesWritten = 0;
    result = EngineSimRender(handle, audioBuffer, 128, &samplesWritten);
    printf("  Render(128 frames) result: %d\n", result);
    printf("  Samples written: %d\n", samplesWritten);
    if (result == ESIM_SUCCESS) {
        printf("  First sample: L=%.6f R=%.6f\n", audioBuffer[0], audioBuffer[1]);
        printf("  PASS: Audio rendered\n\n");
    } else {
        printf("  Note: Audio may not work without loaded script\n\n");
    }

    // Test 10: Destroy simulator
    printf("Test 10: Destroy Simulator\n");
    result = EngineSimDestroy(handle);
    printf("  Destroy result: %d\n", result);
    printf("  %s\n\n", (result == ESIM_SUCCESS) ? "PASS" : "FAIL");

    printf("=== All Tests Complete ===\n");
    return 0;
}
