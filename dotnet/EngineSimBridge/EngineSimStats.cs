namespace EngineSimBridge
{
    /// <summary>
    /// Runtime statistics from the engine simulator.
    /// </summary>
    public sealed class EngineSimStats
    {
        /// <summary>
        /// Current engine speed in revolutions per minute (RPM).
        /// </summary>
        public double CurrentRPM { get; set; }

        /// <summary>
        /// Current engine load (0.0 = idle, 1.0 = full throttle).
        /// </summary>
        public double CurrentLoad { get; set; }

        /// <summary>
        /// Total exhaust flow rate in cubic meters per second (m³/s).
        /// </summary>
        public double ExhaustFlow { get; set; }

        /// <summary>
        /// Intake manifold pressure in Pascals (Pa).
        /// </summary>
        public double ManifoldPressure { get; set; }

        /// <summary>
        /// Number of active audio channels.
        /// </summary>
        public int ActiveChannels { get; set; }

        /// <summary>
        /// Last frame processing time in milliseconds.
        /// This indicates CPU load. Should be well below frame budget.
        /// </summary>
        public double ProcessingTimeMs { get; set; }

        public override string ToString()
        {
            return $"RPM: {CurrentRPM:F0}, Load: {CurrentLoad:P0}, " +
                   $"Flow: {ExhaustFlow:F2} m³/s, Process: {ProcessingTimeMs:F2}ms";
        }
    }
}
