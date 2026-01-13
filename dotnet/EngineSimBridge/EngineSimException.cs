using System;

namespace EngineSimBridge
{
    /// <summary>
    /// Exception thrown by Engine-Sim operations.
    /// </summary>
    public sealed class EngineSimException : Exception
    {
        public EngineSimException(string message)
            : base(message)
        {
        }

        public EngineSimException(string message, Exception innerException)
            : base(message, innerException)
        {
        }
    }
}
