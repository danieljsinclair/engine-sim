# Engine-Sim Bridge Build Configuration for macOS arm64
# Target: M4 macOS (Apple Silicon) native

# Build directory
BUILD_DIR := build

# CMake configuration
CMAKE := cmake
CMAKE_GENERATOR := Unix Makefiles
CMAKE_OSX_ARCHITECTURES := arm64
CMAKE_BUILD_TYPE := Release
CMAKE_POLICY_VERSION_MINIMUM := 3.5
CMAKE_FLAGS := -Werror -Wall -Wextra

# Engine-Sim options
PIRANHA_ENABLED := ON
DISCORD_ENABLED := OFF
DTV := OFF

# Target to build
TARGET := engine-sim-bridge

# Number of parallel jobs
JOBS := 8

.PHONY: all configure build clean rebuild stub test help

all: build

# Check and initialize submodules if needed
SUBMODULE_CHECK := dependencies/submodules/delta-studio/CMakeLists.txt

$(SUBMODULE_CHECK):
	@git submodule update --init --recursive
	@echo "Submodules initialized."

# Configure CMake (run once or after CMakeLists changes)
configure: $(SUBMODULE_CHECK)
	@echo "Configuring CMake for macOS arm64 (Apple Silicon)..."
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(CMAKE) .. \
		-DCMAKE_OSX_ARCHITECTURES=$(CMAKE_OSX_ARCHITECTURES) \
		-DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
		-DCMAKE_POLICY_VERSION_MINIMUM=$(CMAKE_POLICY_VERSION_MINIMUM) \
		-DPIRANHA_ENABLED=$(PIRANHA_ENABLED) \
		-DDISCORD_ENABLED=$(DISCORD_ENABLED) \
		-DDTV=$(DTV) \
		$(CMAKE_FLAGS)
	@echo "Configuration complete. Run 'make build' to compile."

# Build the target
build: configure
	@echo "Building $(TARGET)..."
	cd $(BUILD_DIR) && $(MAKE) -j$(JOBS) $(TARGET)
	@echo "Build complete!"

# Build stub fallback (always works)
stub:
	@echo "Building stub bridge library..."
	clang++ -arch $(CMAKE_OSX_ARCHITECTURES) \
		-fPIC \
		-shared \
		-std=c++17 \
		-o libenginesim.dylib \
		src/engine_sim_bridge_stub.cpp \
		-Iinclude
	@ls -lh libenginesim.dylib
	@echo "Stub build complete!"

# Clean build artifacts
clean:
	@echo "Cleaning build directory..."
	rm -rf $(BUILD_DIR)/*
	@echo "Clean complete."

# Full rebuild
rebuild: clean all

# Test the stub
test: stub
	@echo "Testing stub library..."
	@echo "Library: libenginesim.dylib"
	otool -L libenginesim.dylib
	@echo "If you see libenginesim.dylib above, the stub is ready for .NET wrapper testing"

help:
	@echo "Engine-Sim Bridge Build System"
	@echo ""
	@echo "Targets:"
	@echo "  make          - Build engine-sim-bridge (full)"
	@echo "  make configure - Configure CMake (run first)"
	@echo "  make build     - Build after configure"
	@echo "  make stub      - Build stub fallback (fast, always works)"
	@echo "  make clean     - Remove build artifacts"
	@echo "  make rebuild   - Clean and rebuild"
	@echo "  make test      - Build and test stub"
	@echo "  make help      - Show this help"
	@echo ""
	@echo "Configuration:"
	@echo "  Architecture: $(CMAKE_OSX_ARCHITECTURES)"
	@echo "  Build Type:   $(CMAKE_BUILD_TYPE)"
	@echo "  Jobs:         $(JOBS) parallel"
