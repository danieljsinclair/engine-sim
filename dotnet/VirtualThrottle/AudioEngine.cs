using System;
using System.Runtime.InteropServices;
using System.Threading;
using EngineSimBridge;

namespace VirtualThrottle
{
    /// <summary>
    /// High-performance audio engine using CoreAudio for low-latency output.
    /// Implements allocation-free audio rendering in the callback.
    /// </summary>
    internal sealed class AudioEngine : IDisposable
    {
        private readonly EngineSimulator _simulator;
        private readonly int _sampleRate;
        private readonly int _channels;
        private readonly int _bufferFrames;
        private readonly int _bufferCount;

        private IntPtr _audioQueue;
        private IntPtr[] _buffers;
        private GCHandle _thisHandle;
        private CoreAudioInterop.AudioQueueOutputCallback? _callback;
        private bool _disposed;
        private bool _running;

        // Statistics (atomic for thread safety)
        private long _callbackCount;
        private long _underrunCount;

        /// <summary>
        /// Gets the number of audio callbacks processed.
        /// </summary>
        public long CallbackCount => Interlocked.Read(ref _callbackCount);

        /// <summary>
        /// Gets the number of audio underruns (buffer starvation).
        /// </summary>
        public long UnderrunCount => Interlocked.Read(ref _underrunCount);

        /// <summary>
        /// Creates a new audio engine.
        /// </summary>
        /// <param name="simulator">Engine simulator instance</param>
        /// <param name="sampleRate">Sample rate (e.g., 48000)</param>
        /// <param name="bufferFrames">Buffer size in frames (128-512 recommended)</param>
        /// <param name="bufferCount">Number of buffers (3 recommended)</param>
        public AudioEngine(
            EngineSimulator simulator,
            int sampleRate = 48000,
            int bufferFrames = 256,
            int bufferCount = 3)
        {
            _simulator = simulator ?? throw new ArgumentNullException(nameof(simulator));
            _sampleRate = sampleRate;
            _channels = 2; // Stereo
            _bufferFrames = bufferFrames;
            _bufferCount = bufferCount;
            _buffers = new IntPtr[bufferCount];

            Initialize();
        }

        private void Initialize()
        {
            // Create audio format descriptor
            var format = CoreAudioInterop.CreateFormat(_sampleRate, _channels);

            // Keep a reference to the callback to prevent GC
            _callback = AudioCallback;

            // Pin this instance so we can pass it to the callback
            _thisHandle = GCHandle.Alloc(this, GCHandleType.Normal);

            // Create audio queue
            int status = CoreAudioInterop.AudioQueueNewOutput(
                ref format,
                _callback,
                GCHandle.ToIntPtr(_thisHandle),
                IntPtr.Zero,  // NULL = use internal thread
                IntPtr.Zero,  // NULL = use internal run loop
                0,            // Reserved flags
                out _audioQueue
            );

            CoreAudioInterop.CheckError(status, "AudioQueueNewOutput");

            // Allocate buffers
            uint bufferSize = (uint)(_bufferFrames * _channels * sizeof(float));

            for (int i = 0; i < _bufferCount; i++)
            {
                status = CoreAudioInterop.AudioQueueAllocateBuffer(
                    _audioQueue,
                    bufferSize,
                    out _buffers[i]
                );

                CoreAudioInterop.CheckError(status, $"AudioQueueAllocateBuffer[{i}]");
            }
        }

        /// <summary>
        /// Starts audio playback.
        /// </summary>
        public void Start()
        {
            if (_running)
                return;

            // Prime the buffers
            for (int i = 0; i < _bufferCount; i++)
            {
                PrimeBuffer(_buffers[i]);
            }

            // Start the queue
            int status = CoreAudioInterop.AudioQueueStart(_audioQueue, IntPtr.Zero);
            CoreAudioInterop.CheckError(status, "AudioQueueStart");

            _running = true;

            Console.WriteLine($"Audio engine started: {_sampleRate}Hz, {_bufferFrames} frames/buffer");
            Console.WriteLine($"Theoretical latency: {(_bufferFrames * 1000.0 / _sampleRate):F2}ms per buffer");
        }

        /// <summary>
        /// Stops audio playback.
        /// </summary>
        public void Stop()
        {
            if (!_running)
                return;

            int status = CoreAudioInterop.AudioQueueStop(_audioQueue, true);
            CoreAudioInterop.CheckError(status, "AudioQueueStop");

            _running = false;
        }

        private void PrimeBuffer(IntPtr buffer)
        {
            unsafe
            {
                // Marshal the buffer structure
                var audioBuffer = Marshal.PtrToStructure<CoreAudioInterop.AudioQueueBuffer>(buffer);

                // Get pointer to audio data
                float* data = (float*)audioBuffer.mAudioData;

                // Fill with silence initially
                int totalSamples = _bufferFrames * _channels;
                for (int i = 0; i < totalSamples; i++)
                {
                    data[i] = 0.0f;
                }

                // Set the actual data size
                audioBuffer.mAudioDataByteSize = (uint)(_bufferFrames * _channels * sizeof(float));
                Marshal.StructureToPtr(audioBuffer, buffer, false);

                // Enqueue the buffer
                int status = CoreAudioInterop.AudioQueueEnqueueBuffer(
                    _audioQueue,
                    buffer,
                    0,
                    IntPtr.Zero
                );

                CoreAudioInterop.CheckError(status, "AudioQueueEnqueueBuffer (prime)");
            }
        }

        /// <summary>
        /// CRITICAL AUDIO CALLBACK
        /// This runs on a high-priority CoreAudio thread.
        /// MUST be allocation-free and complete in < 5ms.
        /// </summary>
        private static void AudioCallback(IntPtr userData, IntPtr queue, IntPtr buffer)
        {
            // Restore the AudioEngine instance
            GCHandle handle = GCHandle.FromIntPtr(userData);
            AudioEngine engine = (AudioEngine)handle.Target!;

            engine.ProcessAudioCallback(buffer);
        }

        [System.Runtime.CompilerServices.MethodImpl(
            System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private unsafe void ProcessAudioCallback(IntPtr buffer)
        {
            try
            {
                // Increment callback counter (atomic)
                Interlocked.Increment(ref _callbackCount);

                // Marshal the buffer structure
                var audioBuffer = Marshal.PtrToStructure<CoreAudioInterop.AudioQueueBuffer>(buffer);

                // Get pointer to audio data
                float* data = (float*)audioBuffer.mAudioData;

                // CRITICAL: Call the simulator render function
                // This is ALLOCATION-FREE by design
                int samplesWritten = _simulator.Render(data, _bufferFrames);

                // Check for underrun
                if (samplesWritten < _bufferFrames)
                {
                    Interlocked.Increment(ref _underrunCount);

                    // Fill remainder with silence (already done by simulator)
                    // No additional action needed
                }

                // Set the actual data size
                audioBuffer.mAudioDataByteSize = (uint)(_bufferFrames * _channels * sizeof(float));
                Marshal.StructureToPtr(audioBuffer, buffer, false);

                // Re-enqueue the buffer for the next callback
                int status = CoreAudioInterop.AudioQueueEnqueueBuffer(
                    _audioQueue,
                    buffer,
                    0,
                    IntPtr.Zero
                );

                // In the callback, we can't throw exceptions
                // Just log errors to console if they occur
                if (status != 0)
                {
                    Console.Error.WriteLine($"AudioQueueEnqueueBuffer failed: {status}");
                }
            }
            catch (Exception ex)
            {
                // NEVER throw from audio callback
                Console.Error.WriteLine($"Audio callback exception: {ex.Message}");
            }
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            Stop();

            if (_audioQueue != IntPtr.Zero)
            {
                CoreAudioInterop.AudioQueueDispose(_audioQueue, true);
                _audioQueue = IntPtr.Zero;
            }

            if (_thisHandle.IsAllocated)
            {
                _thisHandle.Free();
            }

            _disposed = true;
        }
    }
}
