using System;

namespace EngineSimBridge
{
    /// <summary>
    /// Configuration parameters for the Engine-Sim simulator.
    /// </summary>
    public sealed class EngineSimConfig
    {
        /// <summary>
        /// Audio output sample rate in Hz.
        /// Default: 48000 Hz (standard for USB DACs)
        /// Valid range: 8000 - 192000
        /// </summary>
        public int SampleRate { get; set; } = 48000;

        /// <summary>
        /// Internal simulation input buffer size.
        /// Default: 1024 samples
        /// Valid range: 64 - 8192
        /// </summary>
        public int InputBufferSize { get; set; } = 1024;

        /// <summary>
        /// Ring buffer size for audio output.
        /// Default: 96000 samples (2 seconds @ 48kHz)
        /// Minimum: SampleRate / 2 (0.5 seconds)
        /// </summary>
        public int AudioBufferSize { get; set; } = 96000;

        /// <summary>
        /// Physics simulation frequency in Hz.
        /// Default: 10000 Hz
        /// Valid range: 1000 - 100000
        /// Higher values = more accurate but more CPU intensive
        /// </summary>
        public int SimulationFrequency { get; set; } = 10000;

        /// <summary>
        /// Number of fluid dynamics substeps per physics step.
        /// Default: 8
        /// Valid range: 1 - 64
        /// Higher values = smoother gas flow simulation
        /// </summary>
        public int FluidSimulationSteps { get; set; } = 8;

        /// <summary>
        /// Target audio synthesizer latency in seconds.
        /// Default: 0.05 (50ms)
        /// Valid range: 0.001 - 1.0
        /// Lower values = more responsive but higher CPU load
        /// </summary>
        public double TargetSynthesizerLatency { get; set; } = 0.05;

        /// <summary>
        /// Master audio volume.
        /// Default: 0.5
        /// Valid range: 0.0 - 10.0
        /// </summary>
        public float Volume { get; set; } = 0.5f;

        /// <summary>
        /// Convolution filter mix level.
        /// Default: 1.0 (full convolution)
        /// Valid range: 0.0 - 1.0
        /// </summary>
        public float ConvolutionLevel { get; set; } = 1.0f;

        /// <summary>
        /// Air intake noise level.
        /// Default: 1.0
        /// Valid range: 0.0 - 2.0
        /// </summary>
        public float AirNoise { get; set; } = 1.0f;

        /// <summary>
        /// Gets a default configuration optimized for low-latency iPhone DAC output.
        /// Target: &lt;40ms end-to-end latency @ 48kHz
        /// </summary>
        public static EngineSimConfig Default => new EngineSimConfig();

        /// <summary>
        /// Gets a low-latency configuration for iPhone 15 USB-C DAC.
        /// 128 frame buffer = 2.67ms @ 48kHz
        /// </summary>
        public static EngineSimConfig LowLatency => new EngineSimConfig
        {
            SampleRate = 48000,
            InputBufferSize = 512,          // Reduced for lower latency
            AudioBufferSize = 48000,        // 1 second buffer
            SimulationFrequency = 10000,
            FluidSimulationSteps = 6,       // Reduced for performance
            TargetSynthesizerLatency = 0.03, // 30ms
            Volume = 0.5f,
            ConvolutionLevel = 0.8f,        // Slightly reduced for performance
            AirNoise = 0.8f
        };

        /// <summary>
        /// Gets a high-quality configuration with more CPU overhead.
        /// Suitable for development/testing on a Mac with good CPU.
        /// </summary>
        public static EngineSimConfig HighQuality => new EngineSimConfig
        {
            SampleRate = 48000,
            InputBufferSize = 2048,
            AudioBufferSize = 144000,       // 3 seconds
            SimulationFrequency = 20000,    // 2x resolution
            FluidSimulationSteps = 12,      // Smoother gas dynamics
            TargetSynthesizerLatency = 0.08,
            Volume = 0.5f,
            ConvolutionLevel = 1.0f,
            AirNoise = 1.0f
        };

        /// <summary>
        /// Validates the configuration.
        /// </summary>
        public void Validate()
        {
            if (SampleRate < 8000 || SampleRate > 192000)
                throw new ArgumentOutOfRangeException(nameof(SampleRate),
                    "Sample rate must be between 8000 and 192000 Hz");

            if (InputBufferSize < 64 || InputBufferSize > 8192)
                throw new ArgumentOutOfRangeException(nameof(InputBufferSize),
                    "Input buffer size must be between 64 and 8192");

            if (AudioBufferSize < SampleRate / 2)
                throw new ArgumentOutOfRangeException(nameof(AudioBufferSize),
                    "Audio buffer size must be at least 0.5 seconds of audio");

            if (SimulationFrequency < 1000 || SimulationFrequency > 100000)
                throw new ArgumentOutOfRangeException(nameof(SimulationFrequency),
                    "Simulation frequency must be between 1000 and 100000 Hz");

            if (FluidSimulationSteps < 1 || FluidSimulationSteps > 64)
                throw new ArgumentOutOfRangeException(nameof(FluidSimulationSteps),
                    "Fluid simulation steps must be between 1 and 64");

            if (TargetSynthesizerLatency < 0.001 || TargetSynthesizerLatency > 1.0)
                throw new ArgumentOutOfRangeException(nameof(TargetSynthesizerLatency),
                    "Target synthesizer latency must be between 0.001 and 1.0 seconds");

            if (Volume < 0.0f || Volume > 10.0f)
                throw new ArgumentOutOfRangeException(nameof(Volume),
                    "Volume must be between 0.0 and 10.0");

            if (ConvolutionLevel < 0.0f || ConvolutionLevel > 1.0f)
                throw new ArgumentOutOfRangeException(nameof(ConvolutionLevel),
                    "Convolution level must be between 0.0 and 1.0");

            if (AirNoise < 0.0f || AirNoise > 2.0f)
                throw new ArgumentOutOfRangeException(nameof(AirNoise),
                    "Air noise must be between 0.0 and 2.0");
        }

        /// <summary>
        /// Calculates the theoretical audio latency for this configuration.
        /// </summary>
        /// <param name="bufferFrames">Hardware buffer size in frames</param>
        /// <returns>Latency in milliseconds</returns>
        public double CalculateLatency(int bufferFrames)
        {
            // Hardware buffer latency
            double hardwareLatency = (double)bufferFrames / SampleRate * 1000.0;

            // Synthesizer latency
            double synthLatency = TargetSynthesizerLatency * 1000.0;

            // Total
            return hardwareLatency + synthLatency;
        }
    }
}
