#ifndef ATG_ENGINE_SIM_WAV_LOADER_H
#define ATG_ENGINE_SIM_WAV_LOADER_H

// dr_wav's single-definition C functions are provided by exactly ONE translation
// unit: engine-sim/src/wav_loader.cpp (which #defines DR_WAV_IMPLEMENTATION and
// includes this header). Every other TU includes this header WITHOUT that macro,
// so it only sees the declarations and links against the single definition. This
// keeps the engine-sim core library self-contained: the preset-compiler and any
// final executable link the core lib and get dr_wav with no duplicate symbols
// and no dependency on the bridge.
#include "dr_libs/dr_wav.h"

#include <vector>
#include <inttypes.h>
#include <string>

/**
 * WAV file loader for engine-sim.
 * Uses dr_wav library (https://github.com/mackron/dr_libs).
 *
 * Responsibility: Load WAV files and return audio samples.
 * Single Responsibility Principle - only handles WAV file I/O.
 *
 * The member functions are declared here and defined once in
 * engine-sim/src/wav_loader.cpp (the same TU that owns the dr_wav definition).
 */
class WavLoader {
public:
    struct Result {
        std::vector<int16_t> samples;
        int sampleRate = 0;
        int channels = 1;
        bool valid = false;

        size_t getSampleCount() const { return samples.size(); }
        const int16_t* getData() const { return samples.data(); }
    };

    /**
     * Load a WAV file from disk.
     *
     * @param filepath Path to WAV file (absolute or relative)
     * @return Result struct containing samples and metadata. Check result.valid.
     */
    static Result load(const std::string& filepath);

    /**
     * Check if a file appears to be a valid WAV file by header inspection.
     */
    static bool isValidWavFile(const std::string& filepath);
};

#endif /* ATG_ENGINE_SIM_WAV_LOADER_H */
