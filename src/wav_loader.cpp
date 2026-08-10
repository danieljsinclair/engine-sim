// wav_loader.cpp - Single definition of WavLoader and the dr_wav C functions.
//
// DR_WAV_IMPLEMENTATION is defined here and ONLY here. Every other TU that needs
// WAV loading includes wav_loader.h without the macro and links against this
// definition. This is the standard stb-style single-header rule, applied so the
// engine-sim core library is self-contained (the preset-compiler links core
// without the bridge) and no final executable gets duplicate dr_wav symbols.

#define DR_WAV_IMPLEMENTATION
#include "../include/wav_loader.h"

WavLoader::Result WavLoader::load(const std::string& filepath) {
    Result result;
    result.valid = false;

    drwav wav;
    if (!drwav_init_file(&wav, filepath.c_str(), nullptr)) {
        return result;
    }

    result.sampleRate = wav.sampleRate;
    result.channels = wav.channels;
    result.samples.resize(wav.totalPCMFrameCount * wav.channels);

    size_t framesRead = drwav_read_pcm_frames_s16(
        &wav,
        wav.totalPCMFrameCount,
        result.samples.data());

    result.valid = (framesRead == wav.totalPCMFrameCount);
    drwav_uninit(&wav);

    return result;
}

bool WavLoader::isValidWavFile(const std::string& filepath) {
    drwav wav;
    bool valid = drwav_init_file(&wav, filepath.c_str(), nullptr);
    if (valid) {
        drwav_uninit(&wav);
    }
    return valid;
}
