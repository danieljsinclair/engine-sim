using System;
using EngineSimBridge;

namespace VirtualThrottle
{
    /// <summary>
    /// Maps keyboard input (1-9 keys) to throttle position.
    /// Provides discrete throttle steps for virtual accelerator pedal.
    /// </summary>
    internal sealed class ThrottleController
    {
        private readonly EngineSimulator _simulator;
        private double _currentThrottle;

        // Throttle map: 1 = 10%, 2 = 20%, ..., 9 = 100%
        private static readonly double[] ThrottleMap = new double[]
        {
            0.0,   // 0 key (if used)
            0.111, // 1 key
            0.222, // 2 key
            0.333, // 3 key
            0.444, // 4 key
            0.556, // 5 key
            0.667, // 6 key
            0.778, // 7 key
            0.889, // 8 key
            1.0    // 9 key
        };

        public double CurrentThrottle => _currentThrottle;

        public ThrottleController(EngineSimulator simulator)
        {
            _simulator = simulator ?? throw new ArgumentNullException(nameof(simulator));
            _currentThrottle = 0.0;
        }

        /// <summary>
        /// Handles keyboard input and updates throttle position.
        /// </summary>
        /// <param name="key">Console key pressed</param>
        /// <returns>True if the key was handled</returns>
        public bool HandleKeyPress(ConsoleKey key)
        {
            double newThrottle = _currentThrottle;

            switch (key)
            {
                case ConsoleKey.D0:
                case ConsoleKey.NumPad0:
                    newThrottle = 0.0;
                    break;

                case ConsoleKey.D1:
                case ConsoleKey.NumPad1:
                    newThrottle = ThrottleMap[1];
                    break;

                case ConsoleKey.D2:
                case ConsoleKey.NumPad2:
                    newThrottle = ThrottleMap[2];
                    break;

                case ConsoleKey.D3:
                case ConsoleKey.NumPad3:
                    newThrottle = ThrottleMap[3];
                    break;

                case ConsoleKey.D4:
                case ConsoleKey.NumPad4:
                    newThrottle = ThrottleMap[4];
                    break;

                case ConsoleKey.D5:
                case ConsoleKey.NumPad5:
                    newThrottle = ThrottleMap[5];
                    break;

                case ConsoleKey.D6:
                case ConsoleKey.NumPad6:
                    newThrottle = ThrottleMap[6];
                    break;

                case ConsoleKey.D7:
                case ConsoleKey.NumPad7:
                    newThrottle = ThrottleMap[7];
                    break;

                case ConsoleKey.D8:
                case ConsoleKey.NumPad8:
                    newThrottle = ThrottleMap[8];
                    break;

                case ConsoleKey.D9:
                case ConsoleKey.NumPad9:
                    newThrottle = ThrottleMap[9];
                    break;

                case ConsoleKey.Spacebar:
                    // Space = release throttle to idle
                    newThrottle = 0.0;
                    break;

                case ConsoleKey.UpArrow:
                    // Increase throttle by 10%
                    newThrottle = Math.Min(1.0, _currentThrottle + 0.1);
                    break;

                case ConsoleKey.DownArrow:
                    // Decrease throttle by 10%
                    newThrottle = Math.Max(0.0, _currentThrottle - 0.1);
                    break;

                default:
                    return false; // Key not handled
            }

            if (Math.Abs(newThrottle - _currentThrottle) > 0.001)
            {
                SetThrottle(newThrottle);
                return true;
            }

            return false;
        }

        private void SetThrottle(double position)
        {
            _currentThrottle = position;
            _simulator.SetThrottle(position);

            // Visual feedback
            PrintThrottleBar(position);
        }

        private void PrintThrottleBar(double position)
        {
            const int barWidth = 40;
            int filled = (int)(position * barWidth);

            Console.Write("\rThrottle: [");

            // Draw filled portion in green
            Console.ForegroundColor = ConsoleColor.Green;
            Console.Write(new string('█', filled));

            // Draw empty portion
            Console.ForegroundColor = ConsoleColor.DarkGray;
            Console.Write(new string('░', barWidth - filled));

            Console.ResetColor();
            Console.Write($"] {position * 100:F0}% ");
        }

        public void PrintInstructions()
        {
            Console.WriteLine();
            Console.WriteLine("╔══════════════════════════════════════════════════╗");
            Console.WriteLine("║       VIRTUAL THROTTLE CONTROL - ENGINE-SIM      ║");
            Console.WriteLine("╚══════════════════════════════════════════════════╝");
            Console.WriteLine();
            Console.WriteLine("CONTROLS:");
            Console.WriteLine("  [1-9]      Set throttle to 10%-100%");
            Console.WriteLine("  [0/Space]  Release throttle (idle)");
            Console.WriteLine("  [↑/↓]      Fine adjust throttle (±10%)");
            Console.WriteLine("  [Q/Esc]    Quit");
            Console.WriteLine();
            Console.WriteLine("FEATURES:");
            Console.WriteLine("  • Inertia-based RPM simulation (no instant jumps)");
            Console.WriteLine("  • Procedural exhaust pops on throttle lift");
            Console.WriteLine("  • Low-latency audio (<40ms target)");
            Console.WriteLine("  • 48kHz output via USB-C DAC");
            Console.WriteLine();
            Console.WriteLine("Listening for input...");
            Console.WriteLine();
        }
    }
}
