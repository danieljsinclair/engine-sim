using System;
using System.Runtime.InteropServices;

namespace EngineSimBridge
{
    /// <summary>
    /// Managed wrapper for the Engine-Sim headless simulator.
    /// Provides safe, idiomatic C# API over the native bridge.
    /// </summary>
    public sealed class EngineSimulator : IDisposable
    {
        private IntPtr _handle;
        private bool _disposed;
        private readonly EngineSimConfig _config;

        /// <summary>
        /// Gets the current RPM of the engine.
        /// </summary>
        public double RPM { get; private set; }

        /// <summary>
        /// Gets the current throttle position (0.0 - 1.0).
        /// </summary>
        public double Throttle { get; private set; }

        /// <summary>
        /// Gets whether the simulator is initialized and ready.
        /// </summary>
        public bool IsInitialized => _handle != IntPtr.Zero && !_disposed;

        /// <summary>
        /// Configuration used to create this simulator instance.
        /// </summary>
        public EngineSimConfig Configuration => _config;

        /// <summary>
        /// Creates a new Engine-Sim instance with default configuration.
        /// </summary>
        public EngineSimulator()
            : this(EngineSimConfig.Default)
        {
        }

        /// <summary>
        /// Creates a new Engine-Sim instance with custom configuration.
        /// </summary>
        /// <param name="config">Simulator configuration</param>
        public EngineSimulator(EngineSimConfig config)
        {
            _config = config;

            // Convert to native struct
            var nativeConfig = new EngineSimNative.EngineSimConfig
            {
                SampleRate = config.SampleRate,
                InputBufferSize = config.InputBufferSize,
                AudioBufferSize = config.AudioBufferSize,
                SimulationFrequency = config.SimulationFrequency,
                FluidSimulationSteps = config.FluidSimulationSteps,
                TargetSynthesizerLatency = config.TargetSynthesizerLatency,
                Volume = config.Volume,
                ConvolutionLevel = config.ConvolutionLevel,
                AirNoise = config.AirNoise
            };

            // Create native instance
            var result = EngineSimNative.EngineSimCreate(in nativeConfig, out _handle);

            if (result != EngineSimNative.EngineSimResult.Success)
            {
                throw new EngineSimException($"Failed to create simulator: {result}");
            }
        }

        /// <summary>
        /// Loads an engine configuration from a .mr script file.
        /// </summary>
        /// <param name="scriptPath">Absolute path to the .mr file</param>
        public void LoadScript(string scriptPath)
        {
            ThrowIfDisposed();

            if (string.IsNullOrWhiteSpace(scriptPath))
            {
                throw new ArgumentException("Script path cannot be null or empty", nameof(scriptPath));
            }

            var result = EngineSimNative.EngineSimLoadScript(_handle, scriptPath);

            if (result != EngineSimNative.EngineSimResult.Success)
            {
                string error = GetLastError();
                throw new EngineSimException($"Failed to load script '{scriptPath}': {error}");
            }
        }

        /// <summary>
        /// Starts the internal audio processing thread.
        /// Must be called after LoadScript() and before Render().
        /// </summary>
        public void StartAudioThread()
        {
            ThrowIfDisposed();

            var result = EngineSimNative.EngineSimStartAudioThread(_handle);

            if (result != EngineSimNative.EngineSimResult.Success)
            {
                throw new EngineSimException($"Failed to start audio thread: {result}");
            }
        }

        /// <summary>
        /// Sets the throttle position (0.0 = closed, 1.0 = wide open).
        /// Thread-safe and allocation-free.
        /// </summary>
        /// <param name="position">Throttle position (0.0 - 1.0)</param>
        public void SetThrottle(double position)
        {
            ThrowIfDisposed();

            if (position < 0.0 || position > 1.0)
            {
                throw new ArgumentOutOfRangeException(nameof(position),
                    "Throttle position must be between 0.0 and 1.0");
            }

            var result = EngineSimNative.EngineSimSetThrottle(_handle, position);

            if (result != EngineSimNative.EngineSimResult.Success)
            {
                throw new EngineSimException($"Failed to set throttle: {result}");
            }

            Throttle = position;
        }

        /// <summary>
        /// Updates the simulation by the specified time step.
        /// Should be called from the main thread at ~60-120Hz.
        /// </summary>
        /// <param name="deltaTime">Time step in seconds (e.g., 1/60)</param>
        public void Update(double deltaTime)
        {
            ThrowIfDisposed();

            if (deltaTime <= 0.0)
            {
                throw new ArgumentOutOfRangeException(nameof(deltaTime),
                    "Delta time must be positive");
            }

            var result = EngineSimNative.EngineSimUpdate(_handle, deltaTime);

            if (result != EngineSimNative.EngineSimResult.Success)
            {
                throw new EngineSimException($"Failed to update simulation: {result}");
            }

            // Update cached statistics
            UpdateStats();
        }

        /// <summary>
        /// Renders audio samples to the provided buffer.
        /// CRITICAL: This must be called from the audio callback thread.
        /// The method is allocation-free when used with unsafe pointers.
        /// </summary>
        /// <param name="buffer">Pointer to float buffer (interleaved stereo)</param>
        /// <param name="frames">Number of frames to render</param>
        /// <returns>Number of samples actually written</returns>
        public unsafe int Render(float* buffer, int frames)
        {
            ThrowIfDisposed();

            if (buffer == null)
            {
                throw new ArgumentNullException(nameof(buffer));
            }

            if (frames <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(frames),
                    "Frame count must be positive");
            }

            var result = EngineSimNative.EngineSimRender(
                _handle,
                new IntPtr(buffer),
                frames,
                out int samplesWritten
            );

            if (result != EngineSimNative.EngineSimResult.Success)
            {
                // In audio callback, we can't throw - return 0 to signal error
                return 0;
            }

            return samplesWritten;
        }

        /// <summary>
        /// Renders audio samples to a managed array.
        /// WARNING: This allocates memory and should NOT be used in the audio callback.
        /// Use the unsafe pointer version instead for real-time audio.
        /// </summary>
        /// <param name="buffer">Destination buffer (interleaved stereo)</param>
        /// <param name="offset">Offset in buffer to start writing</param>
        /// <param name="frames">Number of frames to render</param>
        /// <returns>Number of samples actually written</returns>
        public int Render(float[] buffer, int offset, int frames)
        {
            ThrowIfDisposed();

            if (buffer == null)
            {
                throw new ArgumentNullException(nameof(buffer));
            }

            if (offset < 0 || offset >= buffer.Length)
            {
                throw new ArgumentOutOfRangeException(nameof(offset));
            }

            if (frames <= 0 || offset + (frames * 2) > buffer.Length)
            {
                throw new ArgumentOutOfRangeException(nameof(frames));
            }

            // Pin the array and call native function
            unsafe
            {
                fixed (float* ptr = &buffer[offset])
                {
                    return Render(ptr, frames);
                }
            }
        }

        /// <summary>
        /// Gets current engine statistics.
        /// </summary>
        public EngineSimStats GetStats()
        {
            ThrowIfDisposed();

            var result = EngineSimNative.EngineSimGetStats(_handle, out var nativeStats);

            if (result != EngineSimNative.EngineSimResult.Success)
            {
                throw new EngineSimException($"Failed to get stats: {result}");
            }

            return new EngineSimStats
            {
                CurrentRPM = nativeStats.CurrentRPM,
                CurrentLoad = nativeStats.CurrentLoad,
                ExhaustFlow = nativeStats.ExhaustFlow,
                ManifoldPressure = nativeStats.ManifoldPressure,
                ActiveChannels = nativeStats.ActiveChannels,
                ProcessingTimeMs = nativeStats.ProcessingTimeMs
            };
        }

        /// <summary>
        /// Gets the last error message from the native library.
        /// </summary>
        private string GetLastError()
        {
            IntPtr errorPtr = EngineSimNative.EngineSimGetLastError(_handle);
            return errorPtr != IntPtr.Zero
                ? Marshal.PtrToStringAnsi(errorPtr) ?? "Unknown error"
                : "No error";
        }

        /// <summary>
        /// Gets the version of the native bridge library.
        /// </summary>
        public static string GetVersion()
        {
            IntPtr versionPtr = EngineSimNative.EngineSimGetVersion();
            return Marshal.PtrToStringAnsi(versionPtr) ?? "Unknown";
        }

        private void UpdateStats()
        {
            try
            {
                var stats = GetStats();
                RPM = stats.CurrentRPM;
            }
            catch
            {
                // Ignore stats errors
            }
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
            {
                throw new ObjectDisposedException(nameof(EngineSimulator));
            }
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            if (_handle != IntPtr.Zero)
            {
                EngineSimNative.EngineSimDestroy(_handle);
                _handle = IntPtr.Zero;
            }

            _disposed = true;
            GC.SuppressFinalize(this);
        }

        ~EngineSimulator()
        {
            Dispose();
        }
    }
}
