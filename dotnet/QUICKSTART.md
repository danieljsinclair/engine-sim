# Quick Start Guide

Get up and running with Virtual Throttle in 5 minutes.

---

## Prerequisites

- **macOS** 11.0+ (Big Sur or later)
- **Terminal** access
- **5 minutes** of time

---

## Installation

### Step 1: Install Dependencies

```bash
# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install required tools
brew install cmake dotnet-sdk

# Install Xcode Command Line Tools
xcode-select --install
```

### Step 2: Build the Project

```bash
# Clone the repository (if not already done)
cd /path/to/engine-sim

# Build everything with one command
make all

# This will:
# 1. Build the C++ bridge library
# 2. Build the .NET application
# 3. Copy the library to the output directory
```

**Expected output:**
```
Building C++ bridge library...
✓ Bridge library built: build/libenginesim.dylib
Building .NET application...
✓ .NET application built: dotnet/VirtualThrottle/bin/Release/net10.0
```

### Step 3: Run

```bash
# Run with default engine (auto-detects)
make run

# Or specify an engine
make run ENGINE=assets/engines/atg-video-2/01_subaru_ej25_eh.mr
```

---

## Usage

Once running, you'll see:

```
╔══════════════════════════════════════════════════╗
║       VIRTUAL THROTTLE CONTROL - ENGINE-SIM      ║
╚══════════════════════════════════════════════════╝

CONTROLS:
  [1-9]      Set throttle to 10%-100%
  [0/Space]  Release throttle (idle)
  [↑/↓]      Fine adjust throttle (±10%)
  [Q/Esc]    Quit
```

### Try It Out

1. **Press `1`**: Engine idles at low RPM
2. **Press `5`**: Rev to 50% throttle
3. **Press `9`**: Full throttle (WOT - Wide Open Throttle)
4. **Press `0`**: Release throttle (hear the overrun pops!)
5. **Press `Q`**: Quit

---

## Troubleshooting

### "libenginesim.dylib not found"

```bash
# Install to system path
make install
```

### Audio dropouts (crackling)

```bash
# Use release build for better performance
make clean
make all BUILD_TYPE=Release
make run
```

### Script not found

```bash
# Ensure you're in the engine-sim directory
cd /path/to/engine-sim

# Verify assets exist
ls assets/engines/atg-video-2/
```

---

## What's Next?

- **Read [README.md](README.md)** for detailed information
- **Read [BUILD.md](BUILD.md)** for advanced build options
- **Experiment** with different engines in `assets/engines/`
- **Tweak** configuration in `VirtualThrottle/Program.cs`

---

## Quick Reference

### Build Commands

```bash
make all      # Build everything
make clean    # Clean build artifacts
make run      # Build and run
make install  # Install library system-wide
make help     # Show all commands
```

### File Locations

- **Native Library**: `build/libenginesim.dylib`
- **.NET Binary**: `dotnet/VirtualThrottle/bin/Release/net10.0/VirtualThrottle`
- **Engine Scripts**: `assets/engines/`
- **Documentation**: `dotnet/README.md`

---

## Performance Tips

1. **Use Release build**: `make all BUILD_TYPE=Release`
2. **Close other apps**: Especially audio/music apps
3. **Plug in power**: For laptops (prevents thermal throttling)
4. **Use wired headphones**: Bluetooth adds latency

---

## System Requirements

### Minimum
- macOS 11.0 (Big Sur)
- 4GB RAM
- Dual-core CPU
- USB audio interface

### Recommended
- macOS 12.0+ (Monterey)
- 8GB RAM
- Quad-core CPU (M1 or Intel i5)
- USB-C DAC (e.g., Apple dongle)

---

## Getting Help

If you encounter issues:

1. Check [README.md](README.md) troubleshooting section
2. Review [BUILD.md](BUILD.md) for build details
3. Run `make check-deps` to verify prerequisites
4. Open an issue on GitHub with error details

---

## Behind the Scenes

When you run `make all`, here's what happens:

1. **CMake configures** the C++ build with headless mode enabled
2. **Compiler builds** the Engine-Sim core into a shared library
3. **.NET restores** NuGet packages for P/Invoke wrapper
4. **.NET compiles** the managed application
5. **Library is copied** to the output directory
6. **Ready to run!**

Total build time: ~2-5 minutes (depending on your system)

---

**That's it! You're ready to experience realistic engine simulation with virtual throttle control.**

Press `1` through `9` and listen to the engine respond with realistic inertia and overrun pops. 🏎️🔥
