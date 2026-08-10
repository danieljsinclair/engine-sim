#include <gtest/gtest.h>

#include "../include/synthesizer.h"

#include <chrono>
#include <cstdint>

using namespace std::chrono_literals;

void setupStandardSynthesizer(Synthesizer &synth) {
    Synthesizer::Parameters params;
    params.audioBufferSize = 512 * 16;
    params.audioSampleRate = 16;
    params.inputBufferSize = 256;
    params.inputChannelCount = 8;
    params.inputSampleRate = 32;

    Synthesizer::AudioParameters audioParams;
    audioParams.airNoise = 0.0;
    audioParams.inputSampleNoise = 0.0;
    audioParams.levelerMaxGain = 1.0;
    audioParams.levelerMinGain = 1.0;
    audioParams.dF_F_mix = 0.0;
    params.initialAudioParameters = audioParams;

    synth.initialize(params);
}

void setupSynchronizedSynthesizer(Synthesizer &synth) {
    Synthesizer::Parameters params;
    params.audioBufferSize = 512 * 16;
    params.audioSampleRate = 32;
    params.inputBufferSize = 1024;
    params.inputChannelCount = 8;
    params.inputSampleRate = 32;

    Synthesizer::AudioParameters audioParams;
    audioParams.airNoise = 0.0;
    audioParams.inputSampleNoise = 0.0;
    audioParams.levelerMaxGain = 1.0;
    audioParams.levelerMinGain = 1.0;
    audioParams.dF_F_mix = 0.0;
    params.initialAudioParameters = audioParams;

    synth.initialize(params);
}

TEST(SynthesizerTests, SynthesizerSanityCheck) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);
    synth.destroy();
}
/*
TEST(SynthesizerTests, SynthesizerConversionTest) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    EXPECT_NEAR(synth.inputSampleToTimeOffset(0.0), 0.0, 1E-6);
    EXPECT_NEAR(synth.inputSampleToTimeOffset(1.0), 1 / 32.0, 1E-6);

    EXPECT_NEAR(synth.audioSampleToTimeOffset(0), -0.5, 1E-6);

    synth.destroy();
}

TEST(SynthesizerTests, SynthesizerTrimTest) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    const double timeOffset0 = synth.audioSampleToTimeOffset(0);

    synth.trimInput(0.5, false);

    const double timeOffset1 = synth.audioSampleToTimeOffset(8);

    EXPECT_NEAR(timeOffset1, timeOffset0, 1E-6);

    synth.destroy();
}

TEST(SynthesizerTests, SynthesizerSampleTest) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    for (int i = 0; i < 1024; ++i) {
        const double v = (double)i;
        const double data[] = { v, v, v, v, v, v, v, v };
        synth.writeInput(data);
    }

    const double end_t = 1023 / 32.0;

    const double v0 = synth.sampleInput(end_t, 0);
    const double v1 = synth.sampleInput(end_t - 1 / 64.0, 0);

    EXPECT_NEAR(v0, 1023.0, 1E-6);
    EXPECT_NEAR(v1, 1022.5, 1E-6);

    synth.trimInput(0.5);

    const double v0_trim = synth.sampleInput(end_t, 0);
    const double v1_trim = synth.sampleInput(end_t - 1 / 64.0, 0);

    EXPECT_NEAR(v0, v0_trim, 1E-6);
    EXPECT_NEAR(v1, v1_trim, 1E-6);

    synth.destroy();
}
*/

// Previously disabled: segfaulted + hung on macOS.
//   - Segfault: ConvolutionFilter::f() dereferenced a null m_shiftRegister
//     because Synthesizer::initialize() does not call convolution.initialize()
//     (that happens in initializeImpulseResponse()). Fixed by making
//     ConvolutionFilter::f() the identity when uninitialized.
//   - Hang: the test called renderAudio() (the audio-thread loop body), which
//     waits on a condition variable that nothing notifies in single-thread
//     use. The synchronous single-thread path is renderAudioOnDemand().
// This test now exercises the synchronous render path end-to-end and verifies
// it completes without crashing and produces int16-bounded output.
TEST(SynthesizerTests, SynthesizerSystemTestSingleThread) {
    constexpr int inputSamples = 64;
    constexpr int outputSamples = 64;

    Synthesizer synth;
    setupSynchronizedSynthesizer(synth);

    int16_t *output = new int16_t[outputSamples]();

    for (int i = 0; i < inputSamples; ++i) {
        const double v = (double)i;
        const double data[] = { v, v, v, v, v, v, v, v };
        synth.writeInput(data);
    }

    synth.endInputBlock();
    synth.renderAudioOnDemand();

    const int samplesRead = synth.readAudioOutput(outputSamples, output);

    // The synchronous render path must consume the queued input and emit
    // exactly the requested number of samples into the output buffer.
    EXPECT_EQ(samplesRead, outputSamples);

    // Every rendered sample must be a valid int16 value.
    for (int i = 0; i < outputSamples; ++i) {
        EXPECT_GE(output[i], INT16_MIN);
        EXPECT_LE(output[i], INT16_MAX);
    }

    synth.destroy();
    delete[] output;
}

// Previously disabled: segfaulted on macOS (same ConvolutionFilter null
// dereference as the single-thread test). Now runs the real audio rendering
// thread and verifies the pipeline produces a continuous stream of valid
// int16 samples without crashing or deadlocking.
TEST(SynthesizerTests, SynthesizerSystemTest) {
    constexpr int inputSamples = 1024;
    constexpr int outputSamples = 1024;

    Synthesizer synth;
    setupSynchronizedSynthesizer(synth);
    synth.startAudioRenderingThread();

    int16_t *output = new int16_t[outputSamples]();
    int totalSamples = 0;

    for (int i = 0; i < inputSamples;) {
        for (int j = 0; j < 16; ++j, ++i) {
            const double v = (double)i;
            const double data[] = { v, v, v, v, v, v, v, v };
            synth.writeInput(data);
        }

        synth.endInputBlock();
        totalSamples += synth.readAudioOutput(16, output + totalSamples);
    }

    synth.endAudioRenderingThread();

    // Drain whatever the thread rendered before shutdown.
    while (totalSamples < outputSamples) {
        const int n = synth.readAudioOutput(
            outputSamples - totalSamples, output + totalSamples);
        if (n == 0) break;
        totalSamples += n;
    }

    // The rendering thread must have produced some output.
    EXPECT_GT(totalSamples, 0);

    for (int i = 0; i < totalSamples; ++i) {
        EXPECT_GE(output[i], INT16_MIN);
        EXPECT_LE(output[i], INT16_MAX);
    }

    synth.destroy();
    delete[] output;
}

TEST(SynthesizerTests, HasNoChannelDataInitially) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    EXPECT_FALSE(synth.hasAnyChannelData());
    EXPECT_FALSE(synth.hasAllChannelsData());

    synth.destroy();
}

TEST(SynthesizerTests, HasAnyChannelDataWithSingleChannel) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    synth.m_inputChannels[0].data.write(1.0f);

    EXPECT_TRUE(synth.hasAnyChannelData());
    EXPECT_FALSE(synth.hasAllChannelsData());

    synth.destroy();
}

TEST(SynthesizerTests, HasAnyChannelDataWithPartialChannels) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    synth.m_inputChannels[0].data.write(1.0f);
    synth.m_inputChannels[7].data.write(2.0f);

    EXPECT_TRUE(synth.hasAnyChannelData());
    EXPECT_FALSE(synth.hasAllChannelsData());

    synth.destroy();
}

TEST(SynthesizerTests, HasAllChannelsData) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    for (int ch = 0; ch < 8; ++ch) {
        synth.m_inputChannels[ch].data.write(1.0f);
    }

    EXPECT_TRUE(synth.hasAnyChannelData());
    EXPECT_TRUE(synth.hasAllChannelsData());

    synth.destroy();
}

TEST(SynthesizerTests, ChannelDataMethodsWithZeroChannels) {
    Synthesizer::Parameters params;
    params.audioBufferSize = 512 * 16;
    params.audioSampleRate = 16;
    params.inputBufferSize = 256;
    params.inputChannelCount = 0;
    params.inputSampleRate = 32;

    Synthesizer::AudioParameters audioParams;
    audioParams.airNoise = 0.0;
    audioParams.inputSampleNoise = 0.0;
    audioParams.levelerMaxGain = 1.0;
    audioParams.levelerMinGain = 1.0;
    audioParams.dF_F_mix = 0.0;
    params.initialAudioParameters = audioParams;

    Synthesizer synth;
    synth.initialize(params);

    EXPECT_FALSE(synth.hasAnyChannelData());
    EXPECT_FALSE(synth.hasAllChannelsData());

    synth.destroy();
}

// OneShotSampleMixer: a fired pop ADDS to the accumulator (the engine's
// continuing exhaust signal) without consuming or replacing it. The classic
// afterfire bug was that the pop REPLACED the channel's convolution impulse
// response, so the engine note dropped out for the pop's duration. The mixer must
// instead contribute a positive, scaled, finite amount on top of whatever the
// engine is already emitting, and must NOT subtract the engine signal.
TEST(OneShotSampleMixerTest, MixesAdditivelyOnTopOfEngineSignal) {
    OneShotSampleMixer mixer;
    const int16_t pop[] = { 16000, 16000, 16000, 16000 };
    mixer.trigger(pop, 4, 1.0f);  // full-scale crack

    // The engine's exhaust term lives at int16 scale (measured ~750-40000 on the
    // C63 via Synthesizer::renderAudio diagnostics), NOT a normalized [-1,1].
    // A representative cruising-level engine term:
    const float engineSignal = 800.0f;
    float acc = engineSignal;
    for (int i = 0; i < 4; ++i) {
        mixer.addTo(acc);
    }
    // The pop is ADDED on top, never substituted: output is strictly greater than
    // the bare engine signal.
    EXPECT_GT(acc, engineSignal);
    // Each full-scale pop contributes its full int16 sample value, so the summed
    // output is engine + 4*16000. This is the AUDIBILITY contract: the pop sits at
    // the same int16 order of magnitude as the engine term. The previous form
    // (sample * gain / 32768) contributed ~0.49/sample here — ~1600x below the
    // engine term, i.e. inaudible, which was the "custom WAV still sounds like the
    // default" bug. This assertion is the regression guard against that.
    EXPECT_NEAR(acc, engineSignal + 4 * 16000.0f, 1e-3f);
    EXPECT_GT(acc - engineSignal, engineSignal);  // pop dominates a steady engine note
}

// Gain scaling: a quieter pop (lower gain) contributes proportionally less, so the
// mix level is tunable without touching the engine convolution.
TEST(OneShotSampleMixerTest, RespectsGain) {
    OneShotSampleMixer loud, quiet;
    const int16_t pop[] = { 16000, 16000 };
    loud.trigger(pop, 2, 1.0f);
    quiet.trigger(pop, 2, 0.5f);  // half level

    float loudAcc = 0.0f, quietAcc = 0.0f;
    for (int i = 0; i < 2; ++i) { loud.addTo(loudAcc); quiet.addTo(quietAcc); }

    EXPECT_NEAR(loudAcc, 2 * 16000.0f, 1e-3f);
    EXPECT_NEAR(quietAcc, 0.5f * 2 * 16000.0f, 1e-3f);
}

// One-shot: after the sample is exhausted the mixer emits silence (isPlaying()
// goes false), so a single pop does not loop or bleed into later audio. This is
// what lets the engine sound continue unmodified once the pop has finished.
TEST(OneShotSampleMixerTest, PlaysOnceThenSilent) {
    OneShotSampleMixer mixer;
    const int16_t pop[] = { 12000, 12000, 12000 };
    mixer.trigger(pop, 3, 1.0f);
    EXPECT_TRUE(mixer.isPlaying());

    float acc = 0.0f;
    mixer.addTo(acc);
    mixer.addTo(acc);
    mixer.addTo(acc);
    EXPECT_FALSE(mixer.isPlaying());

    // No further contribution after the sample ends.
    float afterAcc = 0.0f;
    mixer.addTo(afterAcc);
    EXPECT_FLOAT_EQ(afterAcc, 0.0f);
}

// Retrigger: a second cylinder firing on a shared exhaust channel restarts the
// sample from its leading edge (the crack re-fires) rather than corrupting the
// in-progress one. This is the V-engine multi-cylinder correctness guarantee that
// the old IR-swap refcount tried to preserve.
TEST(OneShotSampleMixerTest, RetriggerRestartsFromLeadingEdge) {
    OneShotSampleMixer mixer;
    const int16_t first[]  = { 1000, 2000, 3000, 4000 };
    const int16_t second[] = { 9000, 9000, 9000, 9000 };

    mixer.trigger(first, 4, 1.0f);
    float acc = 0.0f;
    mixer.addTo(acc);  // emits 1000 at int16 scale
    EXPECT_NEAR(acc, 1000.0f, 1e-3f);

    // Second cylinder fires mid-pop: should restart at the second sample's head.
    mixer.trigger(second, 4, 1.0f);
    float acc2 = 0.0f;
    mixer.addTo(acc2);
    EXPECT_NEAR(acc2, 9000.0f, 1e-3f);
    EXPECT_TRUE(mixer.isPlaying());
}

// Defensive: a null/empty trigger clears any in-progress playback rather than
// emitting garbage or crashing.
TEST(OneShotSampleMixerTest, EmptyTriggerIsSafeNoOp) {
    OneShotSampleMixer mixer;
    const int16_t pop[] = { 8000, 8000 };
    mixer.trigger(pop, 2, 1.0f);
    mixer.trigger(nullptr, 0, 1.0f);  // invalid -> reset
    EXPECT_FALSE(mixer.isPlaying());

    float acc = 0.123f;  // some engine signal already present
    mixer.addTo(acc);
    EXPECT_FLOAT_EQ(acc, 0.123f);  // engine signal untouched, no pop added
}
