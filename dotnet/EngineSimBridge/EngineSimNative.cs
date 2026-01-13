using System;
using System.Runtime.InteropServices;

namespace EngineSimBridge
{
    /// <summary>
    /// P/Invoke declarations for the native Engine-Sim bridge library.
    /// This class uses [DllImport] to bind to the C API defined in engine_sim_bridge.h
    /// </summary>
    internal static class EngineSimNative
    {
        // Library name varies by platform
        private const string LibraryName = "enginesim";

        // ====================================================================
        // STRUCTS (Must match C layout exactly)
        // ====================================================================

        [StructLayout(LayoutKind.Sequential)]
        internal struct EngineSimConfig
        {
            public int SampleRate;
            public int InputBufferSize;
            public int AudioBufferSize;
            public int SimulationFrequency;
            public int FluidSimulationSteps;
            public double TargetSynthesizerLatency;
            public float Volume;
            public float ConvolutionLevel;
            public float AirNoise;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct EngineSimStats
        {
            public double CurrentRPM;
            public double CurrentLoad;
            public double ExhaustFlow;
            public double ManifoldPressure;
            public int ActiveChannels;
            public double ProcessingTimeMs;
        }

        // ====================================================================
        // ENUMS
        // ====================================================================

        internal enum EngineSimResult : int
        {
            Success = 0,
            ErrorInvalidHandle = -1,
            ErrorNotInitialized = -2,
            ErrorLoadFailed = -3,
            ErrorInvalidParameter = -4,
            ErrorAudioBuffer = -5,
            ErrorScriptCompilation = -6
        }

        // ====================================================================
        // LIFECYCLE FUNCTIONS
        // ====================================================================

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimCreate(
            in EngineSimConfig config,
            out IntPtr outHandle
        );

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern EngineSimResult EngineSimLoadScript(
            IntPtr handle,
            [MarshalAs(UnmanagedType.LPStr)] string scriptPath
        );

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimStartAudioThread(
            IntPtr handle
        );

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimDestroy(
            IntPtr handle
        );

        // ====================================================================
        // CONTROL FUNCTIONS (Hot Path)
        // ====================================================================

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimSetThrottle(
            IntPtr handle,
            double position
        );

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimUpdate(
            IntPtr handle,
            double deltaTime
        );

        // ====================================================================
        // AUDIO RENDERING (Critical Path - Allocation Free)
        // ====================================================================

        /// <summary>
        /// CRITICAL: This must be called from a high-priority audio thread.
        /// The buffer MUST be pinned or allocated in unmanaged memory.
        /// DO NOT allocate managed objects inside the audio callback.
        /// </summary>
        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimRender(
            IntPtr handle,
            IntPtr buffer,          // float* buffer
            int frames,             // Number of frames (NOT samples)
            out int samplesWritten  // Can pass IntPtr.Zero if not needed
        );

        // Overload without output parameter
        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl, EntryPoint = "EngineSimRender")]
        internal static extern EngineSimResult EngineSimRenderNoOutput(
            IntPtr handle,
            IntPtr buffer,
            int frames,
            IntPtr samplesWritten   // Pass IntPtr.Zero
        );

        // ====================================================================
        // DIAGNOSTICS
        // ====================================================================

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimGetStats(
            IntPtr handle,
            out EngineSimStats outStats
        );

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr EngineSimGetLastError(
            IntPtr handle
        );

        // ====================================================================
        // UTILITY
        // ====================================================================

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr EngineSimGetVersion();

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern EngineSimResult EngineSimValidateConfig(
            in EngineSimConfig config
        );
    }
}
