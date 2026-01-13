using System;
using System.Runtime.InteropServices;

namespace VirtualThrottle
{
    /// <summary>
    /// CoreAudio P/Invoke bindings for macOS low-latency audio output.
    /// Provides direct access to the Audio Queue Services API.
    /// </summary>
    internal static class CoreAudioInterop
    {
        private const string AudioToolboxFramework = "/System/Library/Frameworks/AudioToolbox.framework/AudioToolbox";

        // ====================================================================
        // AUDIO QUEUE STRUCTURES
        // ====================================================================

        [StructLayout(LayoutKind.Sequential)]
        internal struct AudioStreamBasicDescription
        {
            public double mSampleRate;
            public uint mFormatID;
            public uint mFormatFlags;
            public uint mBytesPerPacket;
            public uint mFramesPerPacket;
            public uint mBytesPerFrame;
            public uint mChannelsPerFrame;
            public uint mBitsPerChannel;
            public uint mReserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct AudioQueueBuffer
        {
            public uint mAudioDataBytesCapacity;
            public IntPtr mAudioData;
            public uint mAudioDataByteSize;
            public IntPtr mUserData;
            public uint mPacketDescriptionCapacity;
            public IntPtr mPacketDescriptions;
            public uint mPacketDescriptionCount;
        }

        // ====================================================================
        // CONSTANTS
        // ====================================================================

        internal const uint kAudioFormatLinearPCM = 0x6C70636D; // 'lpcm'
        internal const uint kAudioFormatFlagIsFloat = (1 << 0);
        internal const uint kAudioFormatFlagIsPacked = (1 << 3);
        internal const uint kAudioFormatFlagIsNonInterleaved = (1 << 5);

        internal const int kAudioQueueProperty_IsRunning = 0x6171726E; // 'aqrn'

        // ====================================================================
        // CALLBACKS
        // ====================================================================

        internal delegate void AudioQueueOutputCallback(
            IntPtr userData,
            IntPtr queue,
            IntPtr buffer
        );

        // ====================================================================
        // FUNCTIONS
        // ====================================================================

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueNewOutput(
            ref AudioStreamBasicDescription inFormat,
            AudioQueueOutputCallback inCallbackProc,
            IntPtr inUserData,
            IntPtr inCallbackRunLoop,
            IntPtr inCallbackRunLoopMode,
            uint inFlags,
            out IntPtr outAQ
        );

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueAllocateBuffer(
            IntPtr inAQ,
            uint inBufferByteSize,
            out IntPtr outBuffer
        );

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueEnqueueBuffer(
            IntPtr inAQ,
            IntPtr inBuffer,
            uint inNumPacketDescs,
            IntPtr inPacketDescs
        );

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueStart(
            IntPtr inAQ,
            IntPtr inStartTime  // NULL for immediate
        );

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueStop(
            IntPtr inAQ,
            bool inImmediate
        );

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueDispose(
            IntPtr inAQ,
            bool inImmediate
        );

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueSetParameter(
            IntPtr inAQ,
            uint inParamID,
            float inValue
        );

        [DllImport(AudioToolboxFramework)]
        internal static extern int AudioQueueGetProperty(
            IntPtr inAQ,
            uint inID,
            IntPtr outData,
            ref uint ioDataSize
        );

        // ====================================================================
        // UTILITY METHODS
        // ====================================================================

        internal static AudioStreamBasicDescription CreateFormat(int sampleRate, int channels)
        {
            return new AudioStreamBasicDescription
            {
                mSampleRate = sampleRate,
                mFormatID = kAudioFormatLinearPCM,
                mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
                mBytesPerPacket = (uint)(sizeof(float) * channels),
                mFramesPerPacket = 1,
                mBytesPerFrame = (uint)(sizeof(float) * channels),
                mChannelsPerFrame = (uint)channels,
                mBitsPerChannel = 32,
                mReserved = 0
            };
        }

        internal static void CheckError(int status, string operation)
        {
            if (status != 0)
            {
                throw new InvalidOperationException(
                    $"CoreAudio error in {operation}: {status} (0x{status:X})");
            }
        }
    }
}
