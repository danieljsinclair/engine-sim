// span_tame.cpp
//
// SKELETON ONLY — test-architect red phase. Every function body throws until
// the span-tame implementer replaces it with the pinned parameterization
// documented in include/span_tame.h. Nothing in production calls these yet
// (renderAudio is not wired), so default (spanTame == 0) runs are unaffected.

#include "../include/span_tame.h"

namespace span_tame {

float ratioFor(float /*x*/) {
    throw NotImplementedError("span_tame::ratioFor — skeleton, not implemented");
}

float makeupGainFor(float /*x*/) {
    throw NotImplementedError("span_tame::makeupGainFor — skeleton, not implemented");
}

float kneeCompress(float /*magnitude*/, float /*x*/) {
    throw NotImplementedError("span_tame::kneeCompress — skeleton, not implemented");
}

float safetySoftClip(float /*magnitude*/) {
    throw NotImplementedError("span_tame::safetySoftClip — skeleton, not implemented");
}

float shape(float /*normalizedSample*/, float /*x*/) {
    throw NotImplementedError("span_tame::shape — skeleton, not implemented");
}

}  // namespace span_tame
