// span_tame_tests.cpp
//
// Tests for the --span-tame output-stage tamer (see include/span_tame.h for
// the pinned parameterization). Three layers:
//
//   1. Pure-function pins — the exact formulas (ratio, makeup gain, knee,
//      safety soft-clip) and the x == 0 exact-identity off contract.
//   2. Behavior pins on fixtures — windowed-RMS span (p95 - p05 dB) must be
//      non-increasing as x rises, on a synthetic envelope fixture AND on a
//      real embedded V4 40x-overdrive PCM slice; hard rail clipping must be
//      eliminated at x = 1 on the hot fixture.
//   3. Synthesizer integration — renderAudio honors AudioParameters.spanTame:
//      default-off is bit-identical to the legacy render (golden pin), and an
//      active taming value reduces peak output below the clip metric.
//
// The fixtures are COMPILED IN (span_tame_fixture.h / span_tame_render_golden.h)
// — no test reads a transient file.

#include <gtest/gtest.h>

#include "../include/span_tame.h"
#include "../include/synthesizer.h"
#include "span_tame_fixture.h"
#include "span_tame_render_golden.h"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <vector>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

constexpr float kThreshold = 0.25118864f;   // 10^(-12/20)  (kThresholdDb)
constexpr float kKneeFloor = 0.12589254f;   // 10^(-18/20)  (threshold - knee width)
constexpr float kKneeTop = 0.50118723f;     // 10^(-6/20)   (threshold + knee width)

// The int16 conversion renderAudio performs (lround + rail clamp). The clip
// metrics below reuse it so fixture measurements match what reaches the
// output buffer.
int16_t toInt16(float normalizedSample) {
    long r = std::lround(static_cast<double>(normalizedSample) * 32768.0);
    if (r > INT16_MAX) r = INT16_MAX;
    if (r < INT16_MIN) r = INT16_MIN;
    return static_cast<int16_t>(r);
}

// Apply the tamer to a normalized buffer at taming amount x. At x == 0 this
// is the exact identity (the off contract), so the same helper measures the
// baseline.
std::vector<float> applyTame(const std::vector<float>& in, float x) {
    std::vector<float> out;
    out.reserve(in.size());
    for (float v : in) out.push_back(span_tame::shape(v, x));
    return out;
}

// Windowed-RMS envelope span in dB: p95 - p05 of per-window RMS, the metric
// the span-tuning bench targeted (v4scan2 grid). Non-negative.
double windowedRmsSpanDb(const std::vector<float>& normalized,
                         int window = 512, int hop = 256) {
    std::vector<double> rms;
    for (size_t off = 0; off + static_cast<size_t>(window) <= normalized.size();
         off += static_cast<size_t>(hop)) {
        double acc = 0.0;
        for (int i = 0; i < window; ++i) {
            const double s = normalized[off + static_cast<size_t>(i)];
            acc += s * s;
        }
        rms.push_back(std::sqrt(acc / window));
    }
    EXPECT_FALSE(rms.empty());
    std::sort(rms.begin(), rms.end());
    const auto pct = [&rms](double p) {
        const size_t idx = std::min(
            static_cast<size_t>(p * static_cast<double>(rms.size())),
            rms.size() - 1);
        return rms[idx];
    };
    const double hi = 20.0 * std::log10(pct(0.95) + 1e-12);
    const double lo = 20.0 * std::log10(pct(0.05) + 1e-12);
    return hi - lo;
}

// Deterministic synthetic envelope fixture: alternating overdriven (2.5x full
// scale — the legacy path rail-clips these) and quiet (0.15) 110 Hz segments.
// Big p95/p05 envelope spread by construction.
std::vector<float> makeSyntheticEnvelopeFixture() {
    std::vector<float> out;
    const int segment = 2048;
    const double w = 2.0 * 3.14159265358979323846 * 110.0 / 44100.0;
    const double amps[] = { 2.5, 0.15, 2.5, 0.15, 2.5, 0.15, 2.5, 0.15 };
    for (double amp : amps) {
        for (int i = 0; i < segment; ++i) {
            out.push_back(static_cast<float>(amp * std::sin(w * i)));
        }
    }
    return out;
}

// The real embedded V4 40x fixture, normalized to full-scale units.
std::vector<float> normalizedV4Fixture() {
    std::vector<float> out;
    out.reserve(static_cast<size_t>(kV4OverdriveFixtureSamples));
    for (int i = 0; i < kV4OverdriveFixtureSamples; ++i) {
        out.push_back(static_cast<float>(kV4OverdriveFixture[i]) / 32768.0f);
    }
    return out;
}

// Deterministic single-armed render through the REAL Synthesizer: a rising
// ramp (0 .. amplitude) on all channels, unity-pinned leveler, given synth
// volume and spanTame. Mirrors the established single-thread render seam
// (renderAudioOnDemand — see synthesizer_tests.cpp). The ramp is
// low-frequency content at this seam's tiny sample rates, so it survives the
// DC/antialiasing filters; with amplitude*kSamples*volume over full scale the
// UNTAMED arm rail-clamps (the condition under test).
std::vector<int16_t> renderRampInput(double amplitudePerSample, float volume,
                                     float spanTame) {
    constexpr int kSamples = 64;
    Synthesizer synth;
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
    audioParams.volume = volume;
    audioParams.spanTame = spanTame;
    params.initialAudioParameters = audioParams;

    synth.initialize(params);
    for (int i = 0; i < kSamples; ++i) {
        const double v = amplitudePerSample * i;
        const double data[8] = { v, v, v, v, v, v, v, v };
        synth.writeInput(data);
    }
    synth.endInputBlock();
    synth.renderAudioOnDemand();

    std::vector<int16_t> out(kSamples, 0);
    const int n = synth.readAudioOutput(kSamples, out.data());
    out.resize(n > 0 ? static_cast<size_t>(n) : 0);
    synth.destroy();
    return out;
}

int maxAbs(const std::vector<int16_t>& buf) {
    int m = 0;
    for (int16_t s : buf) m = std::max(m, static_cast<int>(std::abs(s)));
    return m;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. Pure-function parameterization pins
// ---------------------------------------------------------------------------

// R(x) = 1 + 5x exactly: 1:1 at x=0 (no compression), 3.5:1 at half, 6:1 at
// full taming. This is THE tuning knob the owner dialled by ear at runtime.
TEST(SpanTameParameterTest, RatioFormula_IsExactOnePlusFiveX) {
    EXPECT_FLOAT_EQ(span_tame::ratioFor(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(span_tame::ratioFor(0.25f), 2.25f);
    EXPECT_FLOAT_EQ(span_tame::ratioFor(0.5f), 3.5f);
    EXPECT_FLOAT_EQ(span_tame::ratioFor(0.75f), 4.75f);
    EXPECT_FLOAT_EQ(span_tame::ratioFor(1.0f), 6.0f);
}

// Makeup gain m(x) = 10^(|T| * (1 - 1/R(x)) / 20): unity (0 dB) at x=0,
// exactly +10 dB at x=1 (R=6). Ceiling-preserving: compression maps a
// full-scale input to -|T|/R dB and the makeup returns it to 0 dBFS, so the
// shaped ceiling stays at full scale at every x — average loudness is kept,
// only the span above the knee is squashed.
TEST(SpanTameParameterTest, MakeupGain_UnityAtOffExactlyTenDbAtFull) {
    EXPECT_FLOAT_EQ(span_tame::makeupGainFor(0.0f), 1.0f);
    EXPECT_NEAR(span_tame::makeupGainFor(1.0f), 3.16227766f, 1e-4f);  // +10 dB
}

// Makeup gain must not decrease as taming rises (more compression needs more
// restoration — a knob that got quieter as it tamed would be unusable).
TEST(SpanTameParameterTest, MakeupGain_NonDecreasingInX) {
    float prev = span_tame::makeupGainFor(0.0f);
    for (int i = 1; i <= 20; ++i) {
        const float x = i / 20.0f;
        const float m = span_tame::makeupGainFor(x);
        EXPECT_GE(m, prev - 1e-6f) << "makeup decreased at x=" << x;
        prev = m;
    }
}

// THE OFF CONTRACT: shape(v, 0) is the EXACT identity for every input,
// including over-scale values the legacy path would rail-clip. Combined with
// the renderAudio gate (never enter the taming path at spanTame == 0) this is
// the bit-identical-default guarantee.
TEST(SpanTameShapeTest, OffValue_IsExactIdentityIncludingOverScale) {
    const float probes[] = {
        0.0f, 0.125f, -0.125f, 0.5f, -0.5f, 0.9f, -0.9f, 1.0f, -1.0f,
        1.5f, -1.5f, 2.5f, -2.5f, 8.0f, -8.0f, 36.0f, -36.0f
    };
    for (float v : probes) {
        EXPECT_FLOAT_EQ(span_tame::shape(v, 0.0f), v) << "identity broken at v=" << v;
    }
}

// Below the knee floor (10^(-18/20) = 0.1259 full scale) the compressor is
// the exact identity at ANY taming amount — sub-floor content passes the
// knee untouched (only the makeup gain then scales it, by design).
TEST(SpanTameKneeTest, BelowKneeFloor_CompressorIsIdentityAtFullTame) {
    EXPECT_FLOAT_EQ(span_tame::kneeCompress(0.05f, 1.0f), 0.05f);
    EXPECT_FLOAT_EQ(span_tame::kneeCompress(0.10f, 1.0f), 0.10f);
    EXPECT_FLOAT_EQ(span_tame::kneeCompress(kKneeFloor, 1.0f), kKneeFloor);
}

// Above the knee top the compressor follows the exact dB-domain ratio curve
// out = T_dB + (in_dB - T_dB)/R. At x=1 (R=6): a 0 dBFS input lands at
// exactly -10 dBFS (0.31623); a +6.02 dBFS input at -8.997 dBFS (0.35495).
TEST(SpanTameKneeTest, AboveKneeTop_ExactDbDomainRatioCurve) {
    EXPECT_NEAR(span_tame::kneeCompress(1.0f, 1.0f), 0.31622777f, 1e-3f);
    EXPECT_NEAR(span_tame::kneeCompress(2.0f, 1.0f), 0.35495393f, 1e-3f);
}

// The knee band must join the two exact regions continuously and monotonically:
// no step, no dip. A discontinuity here would be an audible click generator —
// exactly the class of artifact this feature exists to remove.
TEST(SpanTameKneeTest, KneeBand_IsContinuousAndMonotone) {
    const float xValues[] = { 0.25f, 0.5f, 1.0f };
    for (float x : xValues) {
        float prev = -1.0f;
        for (int i = 0; i <= 3000; ++i) {
            const float v = 0.05f + (3.0f * i) / 3000.0f;
            const float y = span_tame::kneeCompress(v, x);
            EXPECT_GE(y, prev - 1e-6f)
                << "knee curve decreased at v=" << v << " x=" << x;
            if (prev >= 0.0f) {
                EXPECT_LT(y - prev, 0.01f)
                    << "knee curve step at v=" << v << " x=" << x;
            }
            prev = y;
        }
    }
}

// The safety soft-clip is transparent below 0.90 full scale (normal content
// untouched) and saturates smoothly above: exact value at 1.0 in, bounded
// above by the 0.95 ceiling (31130 int16 — below both the rails and the
// 32000 clip-window metric), monotone nondecreasing.
TEST(SpanTameSafetyClipTest, TransparentBelowKnee_TanhSaturationAbove) {
    EXPECT_FLOAT_EQ(span_tame::safetySoftClip(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(span_tame::safetySoftClip(0.25f), 0.25f);
    EXPECT_FLOAT_EQ(span_tame::safetySoftClip(0.89f), 0.89f);
    EXPECT_FLOAT_EQ(span_tame::safetySoftClip(span_tame::kSafetyKnee),
                    span_tame::kSafetyKnee);
    // 0.9 + 0.05 * tanh((1.0 - 0.9)/0.05)
    EXPECT_NEAR(span_tame::safetySoftClip(1.0f), 0.94820138f, 1e-4f);
    EXPECT_NEAR(span_tame::safetySoftClip(0.95f), 0.93807971f, 1e-4f);
}

TEST(SpanTameSafetyClipTest, BoundedByCeilingAndMonotone) {
    float prev = -1.0f;
    for (int i = 0; i <= 2000; ++i) {
        const float a = (50.0f * i) / 2000.0f;  // 0 .. 50 (deep overdrive)
        const float y = span_tame::safetySoftClip(a);
        EXPECT_LT(y, span_tame::kSafetyCeiling)
            << "ceiling exceeded at a=" << a;
        EXPECT_GE(y, prev - 1e-6f) << "not monotone at a=" << a;
        prev = y;
    }
}

// The complete chain is sign-symmetric, sign-preserving, zero-fixing, and
// monotone in the sample value — a waveshaper that folded sign or created
// local dips would add harmonics the engine never produced.
TEST(SpanTameShapeTest, SignSymmetricZeroFixingAndMonotoneInSample) {
    const float xValues[] = { 0.25f, 0.5f, 1.0f };
    for (float x : xValues) {
        EXPECT_FLOAT_EQ(span_tame::shape(0.0f, x), 0.0f);
        float prev = -1e30f;
        for (int i = 0; i <= 4000; ++i) {
            const float v = (-40.0f + 80.0f * i / 4000.0f);
            const float y = span_tame::shape(v, x);
            EXPECT_GE(y, prev - 1e-6f) << "shape not monotone at v=" << v;
            if (v > 0.0f) EXPECT_GE(y, 0.0f) << "positive in, negative out";
            if (v < 0.0f) EXPECT_LE(y, 0.0f) << "negative in, positive out";
            // Sign symmetry within float tolerance.
            EXPECT_NEAR(span_tame::shape(-v, x), -span_tame::shape(v, x), 1e-5f);
            prev = y;
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Behavior pins on fixtures
// ---------------------------------------------------------------------------

// THE MONOTONICITY CONTRACT (synthetic envelope fixture): the windowed-RMS
// span (p95 - p05 dB) must be non-increasing as x rises. Turning the taming
// knob up can never widen the envelope — the bench metric the owner tunes by.
TEST(SpanTameSpanTest, SyntheticEnvelope_SpanNonIncreasingAsTameRises) {
    const auto fixture = makeSyntheticEnvelopeFixture();
    const float xValues[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    double prevSpan = 1e30;
    for (float x : xValues) {
        const double span = windowedRmsSpanDb(applyTame(fixture, x));
        EXPECT_LE(span, prevSpan + 1e-6)
            << "span increased when taming rose to x=" << x;
        prevSpan = span;
    }
}

// Same monotonicity contract on the REAL V4 40x content: the embedded clip
// window from the road-script arm that saturated 99.8% of windows.
TEST(SpanTameSpanTest, RealV4Overdrive_SpanNonIncreasingAsTameRises) {
    const auto fixture = normalizedV4Fixture();
    const float xValues[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    double prevSpan = 1e30;
    for (float x : xValues) {
        const double span = windowedRmsSpanDb(applyTame(fixture, x));
        EXPECT_LE(span, prevSpan + 1e-6)
            << "span increased when taming rose to x=" << x;
        prevSpan = span;
    }
}

// THE HOT-FIXTURE CONTRACT: at x=1 rail clipping is eliminated on the V4 40x
// content. Baseline sanity first (the fixture genuinely clips — guards
// against a silently-truncated fixture), then: zero samples at the 32000
// clip metric, zero samples at the int16 rails, after the exact int16
// conversion renderAudio performs.
TEST(SpanTameClipTest, RealV4Overdrive_RailClippingEliminatedAtFullTame) {
    const auto fixture = normalizedV4Fixture();

    // Baseline: the untamed fixture clips (measured 4.86% >= 32000, 3.89%
    // at the rails when the fixture was captured).
    size_t baselineClip = 0;
    for (float v : fixture) {
        const int16_t r = toInt16(v);
        if (std::abs(r) >= 32000) ++baselineClip;
    }
    ASSERT_GE(baselineClip, fixture.size() / 100)
        << "fixture sanity: expected >= 1% clipped samples at x=0";

    // Full taming: no sample at the clip metric, none at the rails.
    size_t tameClip = 0, tameRail = 0;
    for (float v : fixture) {
        const int16_t r = toInt16(span_tame::shape(v, 1.0f));
        if (std::abs(r) >= 32000) ++tameClip;
        if (std::abs(r) >= 32767) ++tameRail;
    }
    EXPECT_EQ(tameClip, size_t{0})
        << "samples at/above the 32000 clip metric after x=1 taming";
    EXPECT_EQ(tameRail, size_t{0})
        << "samples at the int16 rails after x=1 taming";
}

// Same elimination on the synthetic overdrive fixture (2.5x full-scale
// segments): the tamed output stays under the clip metric entirely.
TEST(SpanTameClipTest, SyntheticOverdrive_NoClipMetricSamplesAtFullTame) {
    const auto fixture = makeSyntheticEnvelopeFixture();
    size_t tameClip = 0;
    for (float v : fixture) {
        const int16_t r = toInt16(span_tame::shape(v, 1.0f));
        if (std::abs(r) >= 32000) ++tameClip;
    }
    EXPECT_EQ(tameClip, size_t{0});
}

// ---------------------------------------------------------------------------
// 3. Synthesizer integration
// ---------------------------------------------------------------------------

// BIT-IDENTITY OFF CONTRACT at the renderAudio level: the DEFAULT
// (spanTame == 0, never configured) render must produce byte-for-byte the
// same PCM as the legacy (pre-feature) renderAudio did. The golden values
// were captured from the unmodified synthesizer on this exact deterministic
// seam (unity leveler, zeroed noise, alternating input) — any wiring that
// lets x == 0 perturb the output by even one sample fails here.
TEST(SpanTameSynthesizerTest, RenderAudio_DefaultOff_IsBitIdenticalToLegacyGolden) {
    const auto rendered = renderRampInput(1000.0, 8.0f, 0.0f);
    ASSERT_EQ(rendered.size(), sizeof(kLegacyRenderGolden) / sizeof(kLegacyRenderGolden[0]));
    for (size_t i = 0; i < rendered.size(); ++i) {
        EXPECT_EQ(rendered[i], kLegacyRenderGolden[i])
            << "bit-identity to legacy broken at sample " << i;
    }
}

// Explicit zero must equal the untouched default (both are "off").
TEST(SpanTameSynthesizerTest, RenderAudio_ExplicitZeroEqualsUntouchedDefault) {
    const auto untouched = renderRampInput(1000.0, 8.0f, 0.0f);
    // Second render with the default-constructed parameters (spanTame left
    // at its in-class default) is a separate Synthesizer instance: any
    // cross-instance nondeterminism also fails here.
    const auto again = renderRampInput(1000.0, 8.0f, 0.0f);
    ASSERT_EQ(untouched.size(), again.size());
    for (size_t i = 0; i < untouched.size(); ++i) {
        EXPECT_EQ(untouched[i], again[i]);
    }
}

// ACTIVE TAMING through the real render path: an overdriven render (volume
// overdrive that rail-clamps without taming) must come back under the clip
// metric with spanTame = 1 — the synthesizer must actually CONSULT the
// parameter in renderAudio, not merely store it.
TEST(SpanTameSynthesizerTest, RenderAudio_ActiveTameReducesPeakBelowClipMetric) {
    const auto untamed = renderRampInput(1000.0, 8.0f, 0.0f);
    const auto tamed = renderRampInput(1000.0, 8.0f, 1.0f);
    ASSERT_FALSE(untamed.empty());
    ASSERT_FALSE(tamed.empty());

    // Sanity: the untamed arm genuinely rail-clamps on this seam.
    ASSERT_GE(maxAbs(untamed), 32000)
        << "test seam too quiet: raise the overdrive so the untamed arm clips";

    // The tamed arm stays below the 32000 clip metric (ceiling 31130) and
    // strictly below the untamed peak.
    EXPECT_LT(maxAbs(tamed), 32000);
    EXPECT_LT(maxAbs(tamed), maxAbs(untamed));
}
