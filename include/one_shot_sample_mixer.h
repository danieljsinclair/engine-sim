#ifndef ATG_ENGINE_SIM_ONE_SHOT_SAMPLE_MIXER_H
#define ATG_ENGINE_SIM_ONE_SHOT_SAMPLE_MIXER_H

#include <cstdint>
#include <vector>

// OneShotSampleMixer plays a short pre-loaded audio sample once, scaled, then
// returns silence — and adds that contribution to a caller-supplied accumulator.
//
// Used by the exhaust afterfire "pop on overrun" feature. A fired pop injects a
// custom WAV crack on top of the ongoing engine exhaust sound WITHOUT disturbing
// the exhaust channel's convolution impulse response (which carries the engine's
// real exhaust colour). The pop is therefore MIXED, not swapped: the engine
// convolution runs uninterrupted and the pop is summed in alongside it, so the
// engine note never drops out during a pop.
//
// One instance per exhaust channel. A V-engine shares one exhaust channel across
// several cylinders; per-instance owns its own play-head and gain so a second
// cylinder firing on the same channel simply restarts the sample from its
// leading edge (the crack retriggers) rather than corrupting or cancelling the
// first. This matches real overrun crackle, where each cylinder's pop is its own
// event. The class holds no shared mutable state, so concurrent retriggers are
// safe by construction.
class OneShotSampleMixer {
    public:
        OneShotSampleMixer();
        ~OneShotSampleMixer();

        // Drop-in replacement for the previous convolution-IR swap. Loads the
        // chosen pop sample and starts playing it from its first sample. Any
        // sample already in progress is replaced (retriggered), which is the
        // desired behaviour when a second cylinder fires on a shared channel.
        void trigger(const int16_t *samples, size_t count, float gain);

        // Render one audio frame. Adds the next sample (scaled) to `accumulator`,
        // which the caller feeds into the synthesizer's summed output. Returns
        // nothing; the contribution is applied to `accumulator` by reference.
        void addTo(float &accumulator);

        // Stop playback immediately (no partial fade). Used on teardown.
        void reset();

        bool isPlaying() const { return m_remaining > 0; }

    protected:
        const int16_t *m_samples;   // non-owning view into chamber's stored samples
        size_t m_index;             // next sample to emit
        size_t m_remaining;         // samples left to emit
        float m_gain;               // linear gain applied to each emitted sample
};

#endif /* ATG_ENGINE_SIM_ONE_SHOT_SAMPLE_MIXER_H */
