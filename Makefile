# Makefile for Engine-Sim Virtual Throttle POC
# Builds both C++ bridge and .NET application

.PHONY: all clean bridge dotnet run install help

# Default target
all: bridge dotnet

help:
	@echo "Engine-Sim Virtual Throttle Build System"
	@echo "========================================="
	@echo ""
	@echo "Targets:"
	@echo "  make all      - Build C++ bridge and .NET application"
	@echo "  make bridge   - Build C++ bridge library (.dylib)"
	@echo "  make dotnet   - Build .NET application"
	@echo "  make run      - Build and run the application"
	@echo "  make install  - Install library to system path"
	@echo "  make clean    - Clean build artifacts"
	@echo "  make test     - Run tests"
	@echo ""
	@echo "Configuration:"
	@echo "  BUILD_TYPE    - Debug or Release (default: Release)"
	@echo "  BUFFER_SIZE   - Audio buffer frames (default: 256)"
	@echo ""
	@echo "Example:"
	@echo "  make all BUILD_TYPE=Debug"
	@echo "  make run ENGINE=assets/engines/atg-video-2/01_subaru_ej25_eh.mr"

# Configuration
BUILD_TYPE ?= Release
BUILD_DIR ?= build
BUFFER_SIZE ?= 256
NCPU := $(shell sysctl -n hw.ncpu)

# Detect architecture
ARCH := $(shell uname -m)
ifeq ($(ARCH),arm64)
    DOTNET_ARCH := osx-arm64
else
    DOTNET_ARCH := osx-x64
endif

# Directories
BRIDGE_LIB := $(BUILD_DIR)/libenginesim.dylib
DOTNET_DIR := dotnet
DOTNET_BIN := $(DOTNET_DIR)/VirtualThrottle/bin/$(BUILD_TYPE)/net10.0
ENGINE ?= $(shell pwd)/assets/engines/atg-video-2/01_subaru_ej25_eh.mr

# Build C++ bridge library
bridge:
	@echo "Building C++ bridge library..."
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DBUILD_BRIDGE=ON \
		-DPIRANHA_ENABLED=ON \
		-DDISCORD_ENABLED=OFF \
		-DDTV=OFF
	cd $(BUILD_DIR) && cmake --build . --target engine-sim-bridge -j$(NCPU)
	@echo "✓ Bridge library built: $(BRIDGE_LIB)"

# Build .NET application
dotnet: bridge
	@echo "Building .NET application..."
	cd $(DOTNET_DIR) && dotnet restore
	cd $(DOTNET_DIR) && dotnet build -c $(BUILD_TYPE)
	@# Copy native library to output directory
	@mkdir -p $(DOTNET_BIN)
	cp $(BRIDGE_LIB) $(DOTNET_BIN)/
	@echo "✓ .NET application built: $(DOTNET_BIN)"

# Install library to system path
install: bridge
	@echo "Installing library to /usr/local/lib..."
	sudo mkdir -p /usr/local/lib
	sudo cp $(BRIDGE_LIB) /usr/local/lib/
	sudo cp $(BUILD_DIR)/libenginesim.1.dylib /usr/local/lib/ 2>/dev/null || true
	@echo "✓ Library installed"

# Run the application
run: dotnet
	@echo "Running Virtual Throttle..."
	@if [ -f "$(ENGINE)" ]; then \
		cd $(DOTNET_DIR) && dotnet run --project VirtualThrottle/VirtualThrottle.csproj -c $(BUILD_TYPE) -- "$(ENGINE)"; \
	else \
		echo "Engine script not found: $(ENGINE)"; \
		echo "Trying auto-detect..."; \
		cd $(DOTNET_DIR) && dotnet run --project VirtualThrottle/VirtualThrottle.csproj -c $(BUILD_TYPE); \
	fi

# Run with specific engine
run-engine:
	@if [ -z "$(ENGINE)" ]; then \
		echo "Usage: make run-engine ENGINE=path/to/engine.mr"; \
		exit 1; \
	fi
	$(MAKE) run ENGINE=$(ENGINE)

# Run tests
test: bridge
	@echo "Running C++ tests..."
	cd $(BUILD_DIR) && cmake --build . --target engine-sim-test -j$(NCPU)
	cd $(BUILD_DIR) && ctest --output-on-failure
	@echo "Running .NET tests..."
	cd $(DOTNET_DIR) && dotnet test

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(BUILD_DIR)
	cd $(DOTNET_DIR) && dotnet clean
	rm -rf $(DOTNET_DIR)/*/bin $(DOTNET_DIR)/*/obj
	@echo "✓ Clean complete"

# Full clean (including dependencies)
distclean: clean
	@echo "Cleaning dependencies..."
	git submodule deinit -f .
	git submodule update --init --recursive
	@echo "✓ Full clean complete"

# Check prerequisites
check-deps:
	@echo "Checking prerequisites..."
	@command -v cmake >/dev/null 2>&1 || { echo "❌ CMake not found. Install with: brew install cmake"; exit 1; }
	@command -v dotnet >/dev/null 2>&1 || { echo "❌ .NET SDK not found. Install from: https://dotnet.microsoft.com/download"; exit 1; }
	@command -v git >/dev/null 2>&1 || { echo "❌ Git not found"; exit 1; }
	@echo "✓ CMake: $(shell cmake --version | head -n1)"
	@echo "✓ .NET:  $(shell dotnet --version)"
	@echo "✓ Git:   $(shell git --version)"
	@echo "✓ All prerequisites satisfied"

# Create release package
package: all
	@echo "Creating release package..."
	@mkdir -p release
	cd $(DOTNET_DIR)/VirtualThrottle && dotnet publish -c Release -r $(DOTNET_ARCH) \
		--self-contained true \
		-p:PublishSingleFile=true \
		-p:IncludeNativeLibrariesForSelfExtract=true \
		-o ../../release/
	cp $(BRIDGE_LIB) release/
	@echo "✓ Release package created: release/"
	@echo ""
	@echo "To run:"
	@echo "  cd release"
	@echo "  ./VirtualThrottle /path/to/engine.mr"

# Development: quick rebuild and run
dev:
	@$(MAKE) bridge BUILD_TYPE=Debug
	@$(MAKE) dotnet BUILD_TYPE=Debug
	@$(MAKE) run BUILD_TYPE=Debug

# Print configuration
info:
	@echo "Configuration:"
	@echo "  Build Type:     $(BUILD_TYPE)"
	@echo "  Build Dir:      $(BUILD_DIR)"
	@echo "  Architecture:   $(ARCH)"
	@echo "  .NET Runtime:   $(DOTNET_ARCH)"
	@echo "  CPU Cores:      $(NCPU)"
	@echo "  Buffer Size:    $(BUFFER_SIZE) frames"
	@echo ""
	@echo "Paths:"
	@echo "  Bridge Library: $(BRIDGE_LIB)"
	@echo "  .NET Binary:    $(DOTNET_BIN)"
	@echo "  Default Engine: $(ENGINE)"
