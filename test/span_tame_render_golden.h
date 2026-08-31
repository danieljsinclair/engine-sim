// span_tame_render_golden.h
//
// BIT-IDENTITY GOLDEN for the span-tame off contract: the 64 int16 samples
// the UNMODIFIED (pre-span-tame) Synthesizer renders on the deterministic
// seam used by SpanTameSynthesizerTest (unity-pinned leveler, zeroed noise,
// volume 8, 0..63k ramp input on all 8 channels — the untamed arm
// rail-clamps at 32767). Captured from the legacy renderAudio at commit
// 7c89f34 via the same seam; renderAlternating-equivalent helper in the test
// must reproduce these EXACTLY when spanTame == 0. Any wiring that lets the
// off value perturb even one sample fails the bit-identity contract.
//
#ifndef SPAN_TAME_RENDER_GOLDEN_H
#define SPAN_TAME_RENDER_GOLDEN_H

#include <cstdint>

static const int16_t kLegacyRenderGolden[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4959,
    20768, 32767, 31182, 30972, 32767, 31750, 32485, 32767, 31940, 32767, 32563, 32409,
    32767, 32434, 32647, 32624, 32537, 32640, 32580, 32585, 32614, 32580, 32600, 32599,
    32586, 32604, 32590, 32596, 32596, 32593, 32596, 32594, 32595, 32595, 32595, 32595,
    32595, 32594, 32596, 32594, 32596, 32594, 32596, 32594, 32596, 32594, 32596, 32594,
    32596, 32593, 32597, 32593,
};

#endif  // SPAN_TAME_RENDER_GOLDEN_H
