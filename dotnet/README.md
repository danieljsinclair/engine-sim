# Engine-Sim Virtual Throttle POC

**Drive-by-Wire Accelerator Simulator with Low-Latency Audio Output**

This proof-of-concept implements a virtual throttle control system for the Engine-Sim physics engine, delivering deterministic audio output via CoreAudio with <40ms end-to-end latency.

---

## 🎯 Features

### ✅ Functional Requirements
- **Virtual Throttle Control**: Maps numeric keys (1-9) to target load values (10%-100%)
- **Inertia-Based Revving**: Simulates crankshaft rotational inertia (no instant RPM jumps)
- **Procedural Exhaust Pops**: Natural overrun crackles on throttle lift at high RPM
- **Deterministic Audio**: 48kHz output via USB-C DAC with <40ms latency

### 🔧 Technical Architecture

#### 1. **Native Kernel (C++)**
- Headless Engine-Sim library (`.dylib`)
- Extracted from monolithic application
- Zero OpenGL/GLFW/ImGui dependencies
- Persistent state management across callbacks
- Sample rate: 48,000 Hz (locked to hardware clock)

#### 2. **Managed Wrapper (.NET 10)**
- **P/Invoke Bridge**: Safe, idiomatic C# API
- **Unsafe Audio Rendering**: Allocation-free `float*` pointer manipulation
- **High-Priority Threading**: Simulation runs on `ThreadPriority.Highest`
- **CoreAudio Integration**: Direct hardware-driven callbacks

#### 3. **Audio I/O (CoreAudio)**
- **Pull Model**: Hardware-driven callback architecture
- **Buffer Sizing**: 256 frames @ 48kHz = ~5.3ms per buffer
- **Latency Target**: Buffer (5.3ms) + Synth (30ms) = 35.3ms total

---

## 📁 Project Structure

```
dotnet/
├── EngineSimBridge/          # P/Invoke wrapper library
│   ├── EngineSimNative.cs    # DllImport declarations
│   ├── EngineSimulator.cs    # Managed wrapper
│   ├── EngineSimConfig.cs    # Configuration
│   └── EngineSimStats.cs     # Runtime statistics
│
├── VirtualThrottle/          # Console application
│   ├── CoreAudioInterop.cs   # macOS Audio Queue Services
│   ├── AudioEngine.cs        # Allocation-free audio callback
│   ├── ThrottleController.cs # Keyboard input mapping
│   └── Program.cs            # Main application
│
└── VirtualThrottle.sln       # Solution file
```

---

## 🏗️ Building

### Prerequisites

1. **macOS** (CoreAudio requirement)
2. **.NET 10 SDK** ([Download](https://dotnet.microsoft.com/download/dotnet/10.0))
3. **CMake** 3.10+ (`brew install cmake`)
4. **Xcode Command Line Tools** (`xcode-select --install`)
5. **vcpkg** (for C++ dependencies)

### Build Steps

#### Step 1: Build the Native Library

```bash
# Navigate to engine-sim root
cd /path/to/engine-sim

# Create build directory
mkdir -p build && cd build

# Configure CMake
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_BRIDGE=ON \
  -DPIRANHA_ENABLED=ON \
  -DDISCORD_ENABLED=OFF

# Build
cmake --build . --target engine-sim-bridge -j$(sysctl -n hw.ncpu)

# The library will be at: build/libenginesim.dylib
```

#### Step 2: Copy Library to .NET Runtime Path

```bash
# Copy to a location where .NET can find it
sudo mkdir -p /usr/local/lib
sudo cp libenginesim.dylib /usr/local/lib/

# Or set DYLD_LIBRARY_PATH (development only)
export DYLD_LIBRARY_PATH=/path/to/engine-sim/build:$DYLD_LIBRARY_PATH
```

#### Step 3: Build the .NET Application

```bash
cd dotnet

# Restore dependencies
dotnet restore

# Build
dotnet build -c Release

# Or run directly
dotnet run --project VirtualThrottle/VirtualThrottle.csproj
```

---

## 🚀 Usage

### Running the Application

```bash
# With explicit engine script
dotnet run --project VirtualThrottle/VirtualThrottle.csproj -- \
  /path/to/engine-sim/assets/engines/atg-video-2/01_subaru_ej25_eh.mr

# Auto-detect default engine (searches ../assets/)
cd dotnet/VirtualThrottle
dotnet run
```

### Controls

| Key       | Action                          |
|-----------|---------------------------------|
| `1-9`     | Set throttle to 10%-100%        |
| `0/Space` | Release throttle (idle)         |
| `↑/↓`     | Fine adjust throttle (±10%)     |
| `Q/Esc`   | Quit                            |

### Output

```
Engine-Sim Virtual Throttle POC
================================

Loading engine script: 01_subaru_ej25_eh.mr
Engine-Sim Bridge: engine-sim-bridge/1.0.0

Configuration:
  Sample Rate: 48000 Hz
  Simulation Frequency: 10000 Hz
  Buffer Size: 512 samples
  Target Latency: 30.0ms

✓ Engine loaded successfully
✓ Audio thread started
✓ Audio engine started

Throttle: [████████████░░░░░░░░░░░░░░░░] 40%

Virtual Throttle - 120 FPS - 3245 RPM
```

---

## 🎛️ Configuration Presets

### Low Latency (Default)
- **Target**: iPhone 15 USB-C DAC
- **Latency**: ~35ms end-to-end
- **Buffer**: 256 frames (5.3ms)
- **CPU**: Moderate

```csharp
var config = EngineSimConfig.LowLatency;
```

### High Quality
- **Target**: Development/testing
- **Latency**: ~60ms
- **Buffer**: Larger buffers, smoother audio
- **CPU**: High

```csharp
var config = EngineSimConfig.HighQuality;
```

---

## ⚠️ Architectural Risk: The "GC Gap"

### Problem
.NET's Garbage Collector can pause execution for 1-2ms during Gen 0 collections. With a 5ms audio buffer, this is dangerously close to causing dropouts (xruns).

### Solution
The audio render path is **allocation-free**:

✅ **Do:**
- Use `unsafe` pointers (`float*`)
- Pre-allocate all buffers
- Avoid `new`, `foreach`, LINQ in callbacks

❌ **Don't:**
- Allocate managed objects in audio thread
- Use `string` operations
- Call virtual methods (can trigger JIT)

### Implementation

```csharp
public unsafe int Render(float* buffer, int frames)
{
    // CRITICAL PATH - Allocation-free
    var result = EngineSimNative.EngineSimRender(
        _handle,
        new IntPtr(buffer),
        frames,
        out int samplesWritten
    );

    return samplesWritten; // No exceptions, no allocations
}
```

---

## 📊 Performance Metrics

### Expected Performance (M1 Mac)

| Metric                    | Target   | Typical  |
|---------------------------|----------|----------|
| Audio Latency             | <40ms    | ~35ms    |
| CPU Usage (per core)      | <50%     | 30-40%   |
| Simulation FPS            | 120Hz    | 120Hz    |
| Audio Callbacks/sec       | ~187     | 187      |
| GC Pressure (audio thread)| 0 MB/s   | 0 MB/s   |

### Monitoring

```csharp
var stats = simulator.GetStats();
Console.WriteLine($"RPM: {stats.CurrentRPM:F0}");
Console.WriteLine($"Processing: {stats.ProcessingTimeMs:F2}ms");
Console.WriteLine($"Underruns: {audioEngine.UnderrunCount}");
```

---

## 🐛 Troubleshooting

### Issue: "libenginesim.dylib not found"

**Solution:**
```bash
# Option 1: Install to system path
sudo cp build/libenginesim.dylib /usr/local/lib/

# Option 2: Set library path
export DYLD_LIBRARY_PATH=/path/to/engine-sim/build:$DYLD_LIBRARY_PATH

# Option 3: Copy to output directory
cp build/libenginesim.dylib dotnet/VirtualThrottle/bin/Debug/net10.0/
```

### Issue: Audio dropouts (xruns)

**Solution:**
1. Increase buffer size to 512 frames
2. Reduce `SimulationFrequency` to 8000 Hz
3. Reduce `FluidSimulationSteps` to 4
4. Close other audio applications

### Issue: High CPU usage

**Solution:**
1. Use `EngineSimConfig.LowLatency`
2. Reduce `FluidSimulationSteps` to 6
3. Ensure Release build: `dotnet build -c Release`

### Issue: Script compilation failed

**Solution:**
- Ensure `.mr` file path is absolute
- Check that Piranha dependencies are built
- Verify script syntax (test in main Engine-Sim app first)

---

## 🔬 Technical Deep Dive

### Audio Callback Flow

```
Hardware Timer (48kHz)
    ↓
CoreAudio Callback (5.3ms deadline)
    ↓
AudioEngine.ProcessAudioCallback()
    ↓ (allocation-free)
EngineSimulator.Render(float* buf, 256)
    ↓ (P/Invoke)
EngineSimRender(handle, buf, 256)
    ↓
Synthesizer::readAudioOutput(256, int16_t*)
    ↓ (convert int16 → float32)
Return to hardware
```

### Simulation Update Loop

```
Main Thread (120 Hz)
    ↓
UpdateLoop() [ThreadPriority.Highest]
    ↓
EngineSimUpdate(handle, 1/120s)
    ↓
PistonEngineSimulator::startFrame()
    ↓
while (simulateStep()) {
    • Combustion physics
    • Gas dynamics (8 substeps)
    • Crankshaft dynamics
    • Write to synthesizer
}
    ↓
endFrame()
```

---

## 📝 API Reference

### EngineSimulator

```csharp
// Creation
var sim = new EngineSimulator(EngineSimConfig.LowLatency);

// Loading
sim.LoadScript("/path/to/engine.mr");
sim.StartAudioThread();

// Control (thread-safe)
sim.SetThrottle(0.5); // 50%

// Update (main thread, 60-120Hz)
sim.Update(1.0 / 120.0);

// Render (audio thread, allocation-free)
unsafe {
    int written = sim.Render(buffer, 256);
}

// Diagnostics
var stats = sim.GetStats();
Console.WriteLine($"{stats.CurrentRPM:F0} RPM");
```

---

## 🎓 Learning Resources

- **Engine-Sim Physics**: [YouTube Series](https://www.youtube.com/c/AngeTheGreat)
- **.NET Interop**: [Microsoft Docs](https://docs.microsoft.com/en-us/dotnet/standard/native-interop/)
- **CoreAudio**: [Apple Developer](https://developer.apple.com/documentation/coreaudio)
- **Audio Programming**: [Ross Bencina's Articles](http://www.rossbencina.com/code/real-time-audio-programming-101-time-waits-for-nothing)

---

## 📄 License

This project follows the same license as Engine-Sim (MIT). See parent repository for details.

---

## 🙏 Acknowledgments

- **AngeTheGreat** for the incredible Engine-Sim physics engine
- **Engine-Sim Contributors** for the open-source foundation
- **.NET Team** for P/Invoke and unsafe code support

---

## 📞 Support

For issues related to:
- **Engine-Sim Core**: https://github.com/ange-yaghi/engine-sim/issues
- **This POC**: Open an issue in this repository

---

**Built with 🔥 by the Engine-Sim Virtual Throttle Project**
