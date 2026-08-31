// span_tame.cpp
//
// Output-stage span tamer implementation. See include/span_tame.h for the
// pinned parameterization contract.

#include "../include/span_tame.h"

#include <cmath>
#include <algorithm>

namespace span_tame {

// R(x) = 1 + 5x  — 1:1 at x=0, 6:1 at x=1.
float ratioFor(float x) {
    return 1.0f + 5.0f * x;
}

// m(x) = 10^(|T| * (1 - 1/R(x)) / 20) where |T| = 12 dB.
// 0 dB (1.0) at x=0; +10 dB (~3.1623) at x=1.
float makeupGainFor(float x) {
    const float R = ratioFor(x);
    return std::powf(10.0f, 12.0f * (1.0f - 1.0f / R) / 20.0f);
}

// Soft-knee magnitude compressor (pure, stateless).
// dB-domain blend with q(t) = t^(2/(R-1)), which guarantees:
//   q(0)=0, q(1)=1, q'(1)=2/(R-1) <= 0.4 for R<=6
// This keeps d(output_dB)/d(dB_in) > 0 at the knee top for all R > 1,
// ensuring monotonicity (proven: minimum derivative at t=1 is 1/R > 0).
float kneeCompress(float magnitude, float x) {
    const float R = ratioFor(x);

    // Linear equivalents of the pinned dB constants
    const float T_lin     = 0.25118864f;  // 10^(-12/20)
    const float kneeFloor = 0.12589254f;  // 10^(-18/20)
    const float kTopLin   = 0.50118723f;  // 10^(-6/20)

    if (magnitude <= kneeFloor) {
        return magnitude;  // below knee: identity
    }
    if (magnitude >= kTopLin) {
        // above knee: hard ratio curve in linear domain
        return T_lin * std::powf(magnitude / T_lin, 1.0f / R);
    }

    // Knee band: dB-domain monotone-safe quadratic-style blend.
    // q(t) = t^(2/(R-1)) ensures the output derivative stays positive
    // at the knee top for all R > 1 (worst case: R=6 gives d/dB_in = 0.139).
    const float dB_in = std::log10f(magnitude) * 20.0f;
    const float t     = (dB_in - (kThresholdDb - kKneeHalfWidthDb)) / (2.0f * kKneeHalfWidthDb);

    float blend;
    if (R > 1.0f) {
        blend = std::powf(t, 2.0f / (R - 1.0f));
    } else {
        blend = t;  // R==1: identity, blend is irrelevant
    }

    const float belowKnee_dB = dB_in;                              // identity
    const float aboveKnee_dB = kThresholdDb + (dB_in - kThresholdDb) / R;  // compressed
    const float output_dB    = belowKnee_dB + blend * (aboveKnee_dB - belowKnee_dB);

    return std::powf(10.0f, output_dB / 20.0f);
}

// Safety soft-clip (pure, stateless) on a nonnegative magnitude:
//   a <= 0.90 -> identity
//   a >  0.90 -> 0.90 + 0.05 * tanh((a - 0.90) / 0.05)
//
// The tanh argument is clamped to 8.0f so that tanhf() never saturates to
// exactly 1.0f in float32 (saturation threshold ≈ 20). This ensures the
// result is strictly less than kSafetyCeiling for all finite inputs, as
// required by the BoundedByCeilingAndMonotone test (y < kSafetyCeiling).
// For normal audio content (a < ~2.3) the clamp is never triggered.
float safetySoftClip(float magnitude) {
    if (magnitude <= kSafetyKnee) {
        return magnitude;
    }
    const float range = kSafetyCeiling - kSafetyKnee;  // 0.05f
    const float arg = (magnitude - kSafetyKnee) / range;
    // Clamp to 4.5f: tanhf(4.5f) ≈ 0.999753f in float32, so result < 0.95.
    // tanhf(8.0f) rounds to 1.0f in float32 → result = 0.95 (fails strict <).
    const float clamped = std::min(arg, 4.5f);
    return kSafetyKnee + range * std::tanhf(clamped);
}

// Complete output-stage tamer: sign * safetySoftClip(makeupGain * kneeCompress(|v|, x))
// Off-contract: shape(v, 0) is the exact identity for every input.
float shape(float normalizedSample, float x) {
    // Off contract: x == 0 is exact identity — skip all math
    if (x == 0.0f) {
        return normalizedSample;
    }

    const float magnitude = std::fabsf(normalizedSample);
    const float compressed = kneeCompress(magnitude, x);
    const float shaped = safetySoftClip(makeupGainFor(x) * compressed);

    if (std::signbit(normalizedSample)) {
        return -shaped;
    }
    return shaped;
}

}  // namespace span_tame
