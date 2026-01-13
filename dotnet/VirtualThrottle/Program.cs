using System;
using System.IO;
using System.Threading;
using System.Diagnostics;
using EngineSimBridge;

namespace VirtualThrottle
{
    internal class Program
    {
        private static EngineSimulator? _simulator;
        private static AudioEngine? _audioEngine;
        private static ThrottleController? _throttle;
        private static bool _running = true;

        static int Main(string[] args)
        {
            try
            {
                Console.Clear();
                Console.WriteLine("Engine-Sim Virtual Throttle POC");
                Console.WriteLine("================================");
                Console.WriteLine();

                // Parse command line arguments
                string? scriptPath = args.Length > 0 ? args[0] : null;
                if (scriptPath == null || !File.Exists(scriptPath))
                {
                    // Try to find a default engine script
                    scriptPath = FindDefaultEngineScript();

                    if (scriptPath == null)
                    {
                        Console.WriteLine("ERROR: No engine script specified and no default found.");
                        Console.WriteLine();
                        Console.WriteLine("Usage: VirtualThrottle <path-to-engine.mr>");
                        Console.WriteLine();
                        Console.WriteLine("Example:");
                        Console.WriteLine("  VirtualThrottle /path/to/engine-sim/assets/engines/atg-video-2/01_subaru_ej25_eh.mr");
                        return 1;
                    }
                }

                Console.WriteLine($"Loading engine script: {Path.GetFileName(scriptPath)}");
                Console.WriteLine($"Full path: {scriptPath}");
                Console.WriteLine();

                // Print library version
                Console.WriteLine($"Engine-Sim Bridge: {EngineSimulator.GetVersion()}");
                Console.WriteLine();

                // Create simulator with low-latency configuration
                var config = EngineSimConfig.LowLatency;
                Console.WriteLine("Configuration:");
                Console.WriteLine($"  Sample Rate: {config.SampleRate} Hz");
                Console.WriteLine($"  Simulation Frequency: {config.SimulationFrequency} Hz");
                Console.WriteLine($"  Buffer Size: {config.InputBufferSize} samples");
                Console.WriteLine($"  Target Latency: {config.TargetSynthesizerLatency * 1000:F1}ms");
                Console.WriteLine();

                _simulator = new EngineSimulator(config);

                // Load the engine script
                Console.WriteLine("Loading engine configuration...");
                _simulator.LoadScript(scriptPath);
                Console.WriteLine("✓ Engine loaded successfully");
                Console.WriteLine();

                // Start audio thread
                Console.WriteLine("Starting audio processing thread...");
                _simulator.StartAudioThread();
                Console.WriteLine("✓ Audio thread started");
                Console.WriteLine();

                // Create audio engine
                Console.WriteLine("Initializing CoreAudio...");
                const int bufferFrames = 256; // ~5.3ms @ 48kHz
                _audioEngine = new AudioEngine(_simulator, config.SampleRate, bufferFrames);
                _audioEngine.Start();
                Console.WriteLine("✓ Audio engine started");
                Console.WriteLine();

                // Create throttle controller
                _throttle = new ThrottleController(_simulator);
                _throttle.PrintInstructions();

                // Start update loop
                var updateThread = new Thread(UpdateLoop)
                {
                    Name = "Simulation Update",
                    Priority = ThreadPriority.Highest,
                    IsBackground = true
                };
                updateThread.Start();

                // Main input loop
                RunInputLoop();

                // Shutdown
                _running = false;
                updateThread.Join(1000);

                Console.WriteLine();
                Console.WriteLine("Shutting down...");

                // Print statistics
                var stats = _simulator.GetStats();
                Console.WriteLine();
                Console.WriteLine("Statistics:");
                Console.WriteLine($"  Audio Callbacks: {_audioEngine.CallbackCount:N0}");
                Console.WriteLine($"  Underruns: {_audioEngine.UnderrunCount:N0}");
                Console.WriteLine($"  Final RPM: {stats.CurrentRPM:F0}");
                Console.WriteLine($"  Process Time: {stats.ProcessingTimeMs:F2}ms");

                return 0;
            }
            catch (Exception ex)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine();
                Console.WriteLine($"FATAL ERROR: {ex.Message}");
                Console.WriteLine(ex.StackTrace);
                Console.ResetColor();
                return 1;
            }
            finally
            {
                _audioEngine?.Dispose();
                _simulator?.Dispose();
            }
        }

        private static void UpdateLoop()
        {
            const double targetFps = 120.0;
            const double targetDt = 1.0 / targetFps;

            var stopwatch = Stopwatch.StartNew();
            double lastTime = 0.0;
            int frameCount = 0;
            double fpsTimer = 0.0;

            while (_running)
            {
                double currentTime = stopwatch.Elapsed.TotalSeconds;
                double deltaTime = currentTime - lastTime;
                lastTime = currentTime;

                // Clamp delta time to prevent spiral of death
                if (deltaTime > 0.1)
                    deltaTime = targetDt;

                // Update the simulation
                try
                {
                    _simulator?.Update(deltaTime);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Update error: {ex.Message}");
                }

                // Update FPS counter
                frameCount++;
                fpsTimer += deltaTime;

                if (fpsTimer >= 1.0)
                {
                    // Update title bar with FPS and RPM
                    var stats = _simulator?.GetStats();
                    Console.Title = $"Virtual Throttle - {frameCount} FPS - {stats?.CurrentRPM:F0} RPM";
                    frameCount = 0;
                    fpsTimer = 0.0;
                }

                // Sleep to maintain target frame rate
                double sleepTime = targetDt - (stopwatch.Elapsed.TotalSeconds - currentTime);
                if (sleepTime > 0)
                {
                    Thread.Sleep((int)(sleepTime * 1000));
                }
            }
        }

        private static void RunInputLoop()
        {
            while (_running)
            {
                if (Console.KeyAvailable)
                {
                    var keyInfo = Console.ReadKey(intercept: true);

                    // Check for quit
                    if (keyInfo.Key == ConsoleKey.Q || keyInfo.Key == ConsoleKey.Escape)
                    {
                        _running = false;
                        break;
                    }

                    // Handle throttle input
                    _throttle?.HandleKeyPress(keyInfo.Key);
                }

                Thread.Sleep(10); // 100Hz input polling
            }
        }

        private static string? FindDefaultEngineScript()
        {
            // Try to find the engine-sim repository root
            string currentDir = Directory.GetCurrentDirectory();
            string? repoRoot = FindRepoRoot(currentDir);

            if (repoRoot == null)
                return null;

            // Look for a default engine script
            string[] candidates = new[]
            {
                Path.Combine(repoRoot, "assets", "engines", "atg-video-2", "01_subaru_ej25_eh.mr"),
                Path.Combine(repoRoot, "assets", "engines", "atg-video-2", "02_inline_4.mr"),
                Path.Combine(repoRoot, "assets", "main.mr")
            };

            foreach (var path in candidates)
            {
                if (File.Exists(path))
                    return path;
            }

            return null;
        }

        private static string? FindRepoRoot(string startPath)
        {
            string? current = startPath;

            while (current != null)
            {
                // Check if this is the repo root (has assets folder)
                string assetsPath = Path.Combine(current, "assets");
                if (Directory.Exists(assetsPath))
                    return current;

                // Move up one directory
                current = Directory.GetParent(current)?.FullName;
            }

            return null;
        }
    }
}
