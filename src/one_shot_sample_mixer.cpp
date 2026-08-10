#include "../include/one_shot_sample_mixer.h"

#include <cmath>

OneShotSampleMixer::OneShotSampleMixer()
    : m_samples(nullptr)
    , m_index(0)
    , m_remaining(0)
    , m_gain(0.0f)
{
}

OneShotSampleMixer::~OneShotSampleMixer() {
}

void OneShotSampleMixer::trigger(const int16_t *samples, size_t count, float gain) {
    // Reject an empty/garbage trigger defensively (external boundary input: a
    // chamber that chose a sample but reported count 0). A null pointer with a
    // non-zero count would be an internal contract violation, not user data, so
    // we do not silently swallow it — but emitting nothing is the safe outcome
    // either way and keeps the channel's engine sound intact.
    if (samples == nullptr || count == 0) {
        reset();
        return;
    }
    m_samples = samples;
    m_index = 0;
    m_remaining = count;
    m_gain = gain;
}

void OneShotSampleMixer::addTo(float &accumulator) {
    if (m_remaining == 0) {
        return;
    }

    // The synthesizer's `signal` accumulator is at int16 scale: it is fed through
    // m_levelingFilter and lround()'d to int16 at the end of renderAudio(), and
    // the engine's own exhaust term (the convolution output) lives in that same
    // scale. Measured on the C63, the per-channel engine term peaks around
    // ~750-3000 steady-state (with transient spikes to ~20000-40000). To mix the
    // pop AUDIBLY alongside it, the int16 sample is therefore used directly,
    // scaled only by the caller's linear gain: gain=0.6 => pop peaks at
    // ~0.6 * 32767 (~19660), comparable to / above the engine note so the crackle
    // is clearly heard without being a single deafening click.
    //
    // The previous form divided by 32768 here, which normalized the pop into
    // [-1,1] — ~1000x-60000x below the engine term, i.e. completely buried and
    // inaudible. That was the root cause of the "custom WAV still sounds like the
    // default" symptom: the pop was genuinely mixed in, just silently.
    const float sample =
        static_cast<float>(m_samples[m_index]) * m_gain;
    accumulator += sample;

    ++m_index;
    --m_remaining;
}

void OneShotSampleMixer::reset() {
    m_samples = nullptr;
    m_index = 0;
    m_remaining = 0;
    m_gain = 0.0f;
}
