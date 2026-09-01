// span_tame.h
//
// Output-stage span taming (--span-tame <x>, x in [0, 1]).
//
// WHY THIS EXISTS: the script-side audio_volume lever drives the synthesizer
// input PRE-leveler, so the leveler re-normalizes whatever the script does —
// script-side span-taming and rail saturation are the SAME lever and no value
// satisfies both the clip-windows and span gates (v4scan2 grid, 2026-08-31).
// The definitive fix is a waveshaper at the OUTPUT stage: applied to the
// post-leveler, post-volume sample immediately before the int16 conversion in
// Synthesizer::renderAudio, where the p95-p05 windowed-RMS span and the rail
// clipping actually live.
//
// The whole chain is a STATIC per-sample waveshaper (no envelopes, no state)
// so it is deterministic and independently testable.
//
// PINNED PARAMETERIZATION (the contract the tests assert):
//   threshold      T     = -12 dBFS          (kThresholdDb)
//   knee halfwidth W     = 6 dB              (kKneeHalfWidthDb) — quadratic
//                                            soft knee spanning [-18, -6] dBFS
//   ratio          R(x)  = 1 + 5x            (ratioFor)         — 1:1 at x=0,
//                                                            6:1 at x=1
//   makeup gain    m(x)  = 10^(|T| * (1 - 1/R(x)) / 20)         (makeupGainFor)
//                       — ceiling-preserving: compression maps a full-scale
//                         input to -|T|/R dB, the makeup restores it to 0 dB,
//                         so a full-scale input still lands at full scale.
//                         0 dB at x=0, +10 dB at x=1.
//   safety softclip      = identity below 0.90 full scale       (safetySoftClip)
//                         tanh saturation above, asymptote 0.95
//                       — transparent for normal content; guarantees the
//                         shaped output stays below the clip metric's 32000
//                         (0.9766 full scale) so hard rail clipping and the
//                         >=32000 clip-window gate both go to zero.
//
// OFF CONTRACT: x == 0 is feature-off. shape(v, 0) is the EXACT identity for
// every input (including over-scale ones), and renderAudio must not even enter
// the taming path at spanTame == 0 — the audio output is bit-identical to the
// pre-feature synthesizer (pinned by test).

#ifndef ATG_ENGINE_SIM_SPAN_TAME_H
#define ATG_ENGINE_SIM_SPAN_TAME_H

#include <stdexcept>

namespace span_tame {

// ---------------------------------------------------------------------------
// Pinned constants
// ---------------------------------------------------------------------------

// Soft-knee compressor threshold, dBFS.
constexpr float kThresholdDb = -12.0f;

// Quadratic knee half-width in dB: the knee spans [-18, -6] dBFS.
constexpr float kKneeHalfWidthDb = 6.0f;

// Safety soft-clip engagement point, linear full scale. Transparent below.
constexpr float kSafetyKnee = 0.90f;

// Safety soft-clip asymptote (ceiling), linear full scale. 0.95 full scale is
// 31130 in int16 units — below both the int16 rails and the 32000 clip metric.
constexpr float kSafetyCeiling = 0.95f;

// ---------------------------------------------------------------------------
// Skeleton marker
// ---------------------------------------------------------------------------

// Thrown by the SKELETON stubs until the implementer lands the real bodies.
// Tests fail on it (red phase); production code never calls span_tame with
// x > 0 until renderAudio is wired, so default runs are unaffected.
class NotImplementedError : public std::runtime_error {
    public:
        explicit NotImplementedError(const char *what)
            : std::runtime_error(what) {}
};

// ---------------------------------------------------------------------------
// Parameterization functions (pure)
// ---------------------------------------------------------------------------

// Compressor ratio at taming amount x in [0, 1]: R(x) = 1 + 5x.
// 1.0 at x=0 (no compression), 6.0 at x=1.
float ratioFor(float x);

// Ceiling-preserving makeup gain at taming amount x in [0, 1] (linear):
//   m(x) = 10 ^ (|kThresholdDb| * (1 - 1/ratioFor(x)) / 20)
// 1.0 (0 dB) at x=0; 10 ^ (10/20) = 3.16228 (+10 dB) at x=1.
float makeupGainFor(float x);

// Soft-knee magnitude compressor (pure, stateless), input/output in linear
// full-scale units of |sample| (values above 1.0 are over-scale and valid):
//   |v| <= 10^((T - W)/20)   -> identity
//   |v| >= 10^((T + W)/20)   -> T_lin * (|v| / T_lin) ^ (1 / R(x))
//                                (dB domain: T_dB + (in_dB - T_dB) / R)
//   knee band                -> quadratic blend, continuous in value and slope
//                               at both knee edges, monotone nondecreasing.
float kneeCompress(float magnitude, float x);

// Safety soft clip (pure, stateless) on a nonnegative magnitude:
//   a <= kSafetyKnee     -> identity (normal content untouched)
//   a >  kSafetyKnee     -> kSafetyKnee + (kSafetyCeiling - kSafetyKnee)
//                          * tanh((a - kSafetyKnee) / (kSafetyCeiling - kSafetyKnee))
// Monotone nondecreasing, bounded above by kSafetyCeiling, C1-continuous at
// the knee.
float safetySoftClip(float magnitude);

// The complete output-stage tamer for one normalized sample (v is the
// post-leveler, post-volume sample divided by 32768; |v| > 1 means the legacy
// path would clip the int16 rails):
//   shape(v, x) = sign(v) * safetySoftClip(makeupGainFor(x) * kneeCompress(|v|, x))
//
// At x == 0 the result is the EXACT identity (bit-identical off contract).
float shape(float normalizedSample, float x);

}  // namespace span_tame

#endif /* ATG_ENGINE_SIM_SPAN_TAME_H */
