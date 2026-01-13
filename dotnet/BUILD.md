# Build Guide: Engine-Sim Virtual Throttle POC

This guide walks through building the complete system from scratch.

---

## 📋 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Building the C++ Bridge](#building-the-c-bridge)
3. [Building the .NET Application](#building-the-net-application)
4. [Deployment](#deployment)
5. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### System Requirements

- **Operating System**: macOS 11.0+ (Big Sur or later)
- **Architecture**: x86_64 (Intel) or ARM64 (Apple Silicon)
- **RAM**: 4GB minimum, 8GB recommended
- **Disk**: 2GB free space for build artifacts

### Software Dependencies

#### 1. Xcode Command Line Tools

```bash
# Check if installed
xcode-select -p

# If not installed
xcode-select --install
```

#### 2. CMake

```bash
# Install via Homebrew
brew install cmake

# Verify version (3.10+ required)
cmake --version
```

#### 3. .NET 10 SDK

```bash
# Install via Homebrew
brew install --cask dotnet-sdk

# Or download from: https://dotnet.microsoft.com/download/dotnet/10.0

# Verify installation
dotnet --version  # Should show 10.0.x
```

#### 4. vcpkg (Optional, for dependencies)

```bash
# Clone vcpkg
git clone https://github.com/Microsoft/vcpkg.git ~/vcpkg
cd ~/vcpkg

# Bootstrap
./bootstrap-vcpkg.sh

# Add to PATH (add to ~/.zshrc or ~/.bash_profile)
export PATH="$HOME/vcpkg:$PATH"
```

---

## Building the C++ Bridge

### Step 1: Verify Engine-Sim Dependencies

The engine-sim project uses Git submodules. Ensure they're initialized:

```bash
cd /path/to/engine-sim

# Initialize submodules
git submodule update --init --recursive

# Verify key dependencies exist
ls -la dependencies/submodules/
# Should see: delta-studio, piranha, simple-2d-constraint-solver, etc.
```

### Step 2: Configure CMake

```bash
# Create and enter build directory
mkdir -p build
cd build

# Configure with bridge enabled
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_BRIDGE=ON \
  -DPIRANHA_ENABLED=ON \
  -DDISCORD_ENABLED=OFF \
  -DDTV=OFF
```

**Configuration Options:**

| Option               | Default | Description                          |
|----------------------|---------|--------------------------------------|
| `BUILD_BRIDGE`       | `ON`    | Build headless bridge library        |
| `PIRANHA_ENABLED`    | `ON`    | Enable .mr script loading (required) |
| `DISCORD_ENABLED`    | `OFF`   | Discord Rich Presence (not needed)   |
| `DTV`                | `OFF`   | Video output (not needed)            |

### Step 3: Build

```bash
# Build just the bridge library
cmake --build . --target engine-sim-bridge -j$(sysctl -n hw.ncpu)

# Or build everything (includes tests, app)
cmake --build . -j$(sysctl -n hw.ncpu)
```

**Build Output:**

```
build/
├── libenginesim.1.dylib       # Versioned library
├── libenginesim.dylib         # Symlink to versioned library
└── src/
    └── engine_sim_bridge.cpp.o
```

### Step 4: Verify Build

```bash
# Check library exists
ls -lh libenginesim.dylib

# Inspect exported symbols
nm -gU libenginesim.dylib | grep EngineSim
# Should see: _EngineSimCreate, _EngineSimRender, etc.

# Check architecture
file libenginesim.dylib
# x86_64: Mach-O 64-bit dynamically linked shared library x86_64
# ARM64:  Mach-O 64-bit dynamically linked shared library arm64
```

### Step 5: Install Library

**Option A: System-wide installation (recommended)**

```bash
sudo mkdir -p /usr/local/lib
sudo cp libenginesim.dylib /usr/local/lib/
sudo cp libenginesim.1.dylib /usr/local/lib/

# Update dylib cache
sudo update_dyld_shared_cache
```

**Option B: Local development (temporary)**

```bash
# Set library path for current session
export DYLD_LIBRARY_PATH=/path/to/engine-sim/build:$DYLD_LIBRARY_PATH

# Or add to ~/.zshrc for persistence
echo 'export DYLD_LIBRARY_PATH=/path/to/engine-sim/build:$DYLD_LIBRARY_PATH' >> ~/.zshrc
```

**Option C: Copy to output directory**

```bash
# This is done automatically by the .NET build if configured
cp libenginesim.dylib ../dotnet/VirtualThrottle/bin/Debug/net10.0/
```

---

## Building the .NET Application

### Step 1: Restore Dependencies

```bash
cd /path/to/engine-sim/dotnet

# Restore NuGet packages
dotnet restore

# Verify project references
dotnet list VirtualThrottle/VirtualThrottle.csproj reference
# Should show: ../EngineSimBridge/EngineSimBridge.csproj
```

### Step 2: Build in Debug Mode

```bash
# Build the solution
dotnet build

# Or build specific project
dotnet build VirtualThrottle/VirtualThrottle.csproj

# Output will be at:
# VirtualThrottle/bin/Debug/net10.0/VirtualThrottle.dll
```

### Step 3: Build in Release Mode (Optimized)

```bash
# Build with optimizations
dotnet build -c Release

# Output will be at:
# VirtualThrottle/bin/Release/net10.0/VirtualThrottle.dll
```

**Release Build Benefits:**
- ~2x faster execution
- No debug symbols
- Inlined methods
- Optimized GC

### Step 4: Run

```bash
# Run with default engine (auto-detects)
dotnet run --project VirtualThrottle/VirtualThrottle.csproj

# Run with specific engine
dotnet run --project VirtualThrottle/VirtualThrottle.csproj -- \
  ../assets/engines/atg-video-2/01_subaru_ej25_eh.mr

# Run release build
dotnet run --project VirtualThrottle/VirtualThrottle.csproj -c Release
```

---

## Deployment

### Creating a Self-Contained Package

For distribution without requiring .NET SDK:

```bash
cd dotnet/VirtualThrottle

# Publish for macOS (Intel)
dotnet publish -c Release -r osx-x64 --self-contained true \
  -p:PublishSingleFile=true \
  -p:IncludeNativeLibrariesForSelfExtract=true

# Publish for macOS (Apple Silicon)
dotnet publish -c Release -r osx-arm64 --self-contained true \
  -p:PublishSingleFile=true \
  -p:IncludeNativeLibrariesForSelfExtract=true
```

**Output:**

```
bin/Release/net10.0/osx-x64/publish/
├── VirtualThrottle          # Single executable (80-90MB)
└── libenginesim.dylib       # Native library (must be present)
```

### Running Published Application

```bash
cd bin/Release/net10.0/osx-x64/publish/

# Ensure native library is present
cp /path/to/libenginesim.dylib .

# Run
./VirtualThrottle /path/to/engine.mr
```

---

## Advanced Build Options

### Cross-Compilation for Apple Silicon (from Intel Mac)

```bash
# Build C++ bridge for ARM64
cd /path/to/engine-sim/build
cmake .. -DCMAKE_OSX_ARCHITECTURES=arm64
cmake --build . --target engine-sim-bridge

# Build .NET app for ARM64
cd /path/to/engine-sim/dotnet
dotnet publish -r osx-arm64 -c Release
```

### Universal Binary (x86_64 + ARM64)

```bash
# Build both architectures
mkdir build-x64 build-arm64

# x86_64 build
cd build-x64
cmake .. -DCMAKE_OSX_ARCHITECTURES=x86_64 -DBUILD_BRIDGE=ON
cmake --build . --target engine-sim-bridge

# ARM64 build
cd ../build-arm64
cmake .. -DCMAKE_OSX_ARCHITECTURES=arm64 -DBUILD_BRIDGE=ON
cmake --build . --target engine-sim-bridge

# Create universal binary
cd ..
lipo -create \
  build-x64/libenginesim.dylib \
  build-arm64/libenginesim.dylib \
  -output libenginesim-universal.dylib

# Verify
lipo -info libenginesim-universal.dylib
# Architectures in the fat file: libenginesim-universal.dylib are: x86_64 arm64
```

---

## Troubleshooting

### Issue: CMake can't find dependencies

**Error:**
```
CMake Error: Could not find piranha
```

**Solution:**
```bash
# Update submodules
git submodule update --init --recursive

# Verify
ls dependencies/submodules/piranha
```

---

### Issue: Linker errors during C++ build

**Error:**
```
Undefined symbols for architecture x86_64:
  "piranha::Compiler::compile()"
```

**Solution:**
```bash
# Clean and rebuild
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_BRIDGE=ON -DPIRANHA_ENABLED=ON
cmake --build . --target engine-sim-bridge
```

---

### Issue: .NET can't find libenginesim.dylib

**Error:**
```
DllNotFoundException: Unable to load shared library 'enginesim'
```

**Solution:**
```bash
# Check library path
otool -L dotnet/VirtualThrottle/bin/Debug/net10.0/VirtualThrottle.dll

# Copy library to output
cp build/libenginesim.dylib \
   dotnet/VirtualThrottle/bin/Debug/net10.0/

# Or set library path
export DYLD_LIBRARY_PATH=/path/to/engine-sim/build:$DYLD_LIBRARY_PATH
```

---

### Issue: Script compilation fails

**Error:**
```
Failed to load script: Script compilation failed
```

**Solution:**
1. **Use absolute path:**
   ```bash
   dotnet run -- $(pwd)/../assets/engines/atg-video-2/01_subaru_ej25_eh.mr
   ```

2. **Verify script syntax:**
   ```bash
   # Test in main Engine-Sim app first
   cd build
   ./engine-sim-app --script ../assets/engines/atg-video-2/01_subaru_ej25_eh.mr
   ```

3. **Check dependencies:**
   - `.mr` files may reference other `.mr` files (imports)
   - Ensure `assets/part-library/` is accessible

---

### Issue: Audio dropouts (xruns)

**Symptoms:**
- Crackling sound
- `UnderrunCount` increases

**Solution:**

1. **Increase buffer size:**
   ```csharp
   const int bufferFrames = 512; // Was 256
   ```

2. **Reduce CPU load:**
   ```csharp
   config.SimulationFrequency = 8000;  // Was 10000
   config.FluidSimulationSteps = 4;    // Was 6
   ```

3. **Use Release build:**
   ```bash
   dotnet run -c Release
   ```

---

### Issue: High latency

**Symptoms:**
- Delay between key press and sound change
- Latency > 60ms

**Solution:**

1. **Use low-latency config:**
   ```csharp
   var config = EngineSimConfig.LowLatency;
   ```

2. **Reduce buffer size:**
   ```csharp
   const int bufferFrames = 128; // Lower latency, higher CPU
   ```

3. **Check system audio settings:**
   ```bash
   # Verify sample rate
   system_profiler SPAudioDataType | grep "Default Output"
   ```

---

## Build Verification Checklist

- [ ] C++ bridge compiles without errors
- [ ] `libenginesim.dylib` exports C symbols
- [ ] .NET solution builds in Debug and Release
- [ ] Application runs and loads engine script
- [ ] Audio plays without xruns
- [ ] Throttle control responds to keys 1-9
- [ ] RPM changes smoothly (inertia simulation)
- [ ] Exhaust pops occur on throttle lift

---

## Performance Testing

```bash
# Monitor CPU usage
top -pid $(pgrep -f VirtualThrottle)

# Monitor audio performance
# (In the application, watch for UnderrunCount)

# Profile with Instruments (macOS)
instruments -t "Time Profiler" \
  dotnet/VirtualThrottle/bin/Release/net10.0/VirtualThrottle
```

---

## Next Steps

After successful build:
1. Read [README.md](README.md) for usage instructions
2. Test with different engine configurations
3. Experiment with latency/quality tradeoffs
4. Measure actual end-to-end latency with audio loopback

---

**Questions?** Check the main [README.md](README.md) or open an issue.
