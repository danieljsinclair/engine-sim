cmake_minimum_required(VERSION 3.10)

# Prevent in-source builds that overwrite hand-written Makefiles
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR
        "In-source builds not allowed. Run 'make' from the repo root instead. "
        "Or: cmake -B build && cmake --build build")
endif()

option(DTV "Enable video output" OFF)
option(PIRANHA_ENABLED "Enable scripting input" ON)
option(DISCORD_ENABLED "Enable Discord Rich Presence" ON)
option(ATG_ENGINE_SIM_AFTERFIRE_SPIKE "Enable exhaust afterfire spike" OFF)

if (DTV)
    add_compile_definitions(ATG_ENGINE_SIM_VIDEO_CAPTURE)
endif (DTV)

if (PIRANHA_ENABLED)
    add_compile_definitions(ATG_ENGINE_SIM_PIRANHA_ENABLED)
endif (PIRANHA_ENABLED)

if (DISCORD_ENABLED)
    add_compile_definitions(ATG_ENGINE_SIM_DISCORD_ENABLED)
endif (DISCORD_ENABLED)

# Afterfire spike. NOTE (ODR): the macro adds members to CombustionChamber, so it
# changes sizeof(CombustionChamber). Engine::getChamber() is an INLINE function that
# indexes an array of chambers, so any translation unit that sees the headers must
# agree on the macro or it will compute wrong addresses. The definition is therefore
# also applied PUBLIC on the engine-sim target (see below) so it propagates
# transitively to every consumer instead of relying on each one opting in.
if (ATG_ENGINE_SIM_AFTERFIRE_SPIKE)
    add_compile_definitions(ATG_ENGINE_SIM_AFTERFIRE_SPIKE)
endif (ATG_ENGINE_SIM_AFTERFIRE_SPIKE)

# Enable group projects in folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "cmake")

project(engine-sim)

set(CMAKE_CXX_STANDARD 17)

# ========================================================
# GTEST

include(FetchContent)
FetchContent_Declare(
    googletest
    URL
    https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set_property(TARGET gmock PROPERTY FOLDER "gtest")
set_property(TARGET gmock_main PROPERTY FOLDER "gtest")
set_property(TARGET gtest PROPERTY FOLDER "gtest")
set_property(TARGET gtest_main PROPERTY FOLDER "gtest")

# ========================================================

add_library(engine-sim STATIC
    # Source files
    src/audio_buffer.cpp
    src/camshaft.cpp
    src/crankshaft.cpp
    src/combustion_chamber.cpp
    src/connecting_rod.cpp
    src/convolution_filter.cpp
    src/cylinder_bank.cpp
    src/cylinder_head.cpp
    src/delay_filter.cpp
    src/derivative_filter.cpp
    src/direct_throttle_linkage.cpp
    src/dynamometer.cpp
    src/engine.cpp
    src/exhaust_system.cpp
    src/feedback_comb_filter.cpp
    src/filter.cpp
    src/fuel.cpp
    src/function.cpp
    src/gas_system.cpp
    src/gaussian_filter.cpp
    src/governor.cpp
    src/ignition_module.cpp
    src/impulse_response.cpp
    src/intake.cpp
    src/jitter_filter.cpp
    src/leveling_filter.cpp
    src/low_pass_filter.cpp
    src/one_shot_sample_mixer.cpp
    src/part.cpp
    src/piston.cpp
    src/piston_engine_simulator.cpp
    src/simulator.cpp
    src/standard_valvetrain.cpp
    src/starter_motor.cpp
    src/synthesizer.cpp
    src/throttle.cpp
    src/transmission.cpp
    src/torque_converter.cpp
    src/utilities.cpp
    src/wav_loader.cpp
    src/valvetrain.cpp
    src/vehicle.cpp
    src/vehicle_drag_constraint.cpp
    src/vehicle_speed_constraint.cpp
    src/vtec_valvetrain.cpp

    # Include files
    include/audio_buffer.h
    include/application_settings.h
    include/camshaft.h
    include/crankshaft.h
    include/combustion_chamber.h
    include/connecting_rod.h
    include/convolution_filter.h
    include/cylinder_bank.h
    include/cylinder_head.h
    include/delay_filter.h
    include/derivative_filter.h
    include/direct_throttle_linkage.h
    include/dynamometer.h
    include/engine.h
    include/exhaust_system.h
    include/feedback_comb_filter.h
    include/filter.h
    include/fuel.h
    include/function.h
    include/gas_system.h
    include/gaussian_filter.h
    include/governor.h
    include/ignition_module.h
    include/impulse_response.h
    include/intake.h
    include/jitter_filter.h
    include/leveling_filter.h
    include/low_pass_filter.h
    include/one_shot_sample_mixer.h
    include/part.h
    include/piston.h
    include/piston_engine_simulator.h
    include/simulator.h
    include/standard_valvetrain.h
    include/starter_motor.h
    include/synthesizer.h
    include/throttle.h
    include/transmission.h
    include/units.h
    include/utilities.h
    include/valvetrain.h
    include/vehicle.h
    include/vehicle_drag_constraint.h
    include/vehicle_speed_constraint.h
    include/vtec_valvetrain.h
    include/wav_loader.h
)

target_link_libraries(engine-sim
    PUBLIC simple-2d-constraint-solver
    PUBLIC csv-io)

# PUBLIC (not PRIVATE) on purpose: the macro changes sizeof(CombustionChamber) and
# Engine::getChamber() is inline, so every consumer of these headers must compile
# with the same definition or chamber addressing silently diverges (ODR/layout
# mismatch). PUBLIC makes CMake propagate it to all dependents automatically.
if (ATG_ENGINE_SIM_AFTERFIRE_SPIKE)
    target_compile_definitions(engine-sim PUBLIC ATG_ENGINE_SIM_AFTERFIRE_SPIKE)
endif (ATG_ENGINE_SIM_AFTERFIRE_SPIKE)

# Enable compiler warnings as errors
if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
    target_compile_options(engine-sim PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wno-unused-parameter  # Many Piranha warnings
        -Wno-sign-compare    # Platform differences
        -g)
else()
    target_compile_options(engine-sim PRIVATE -g)
endif()

target_include_directories(engine-sim
    PUBLIC dependencies/submodules
    PUBLIC dependencies)

if (PIRANHA_ENABLED)
    add_library(engine-sim-script-interpreter STATIC
        # Source files
        scripting/src/channel_types.cpp
        scripting/src/compiler.cpp
        scripting/src/engine_context.cpp
        scripting/src/language_rules.cpp

        # Include files
        scripting/include/actions.h
        scripting/include/camshaft_node.h
        scripting/include/channel_types.h
        scripting/include/compiler.h
        scripting/include/connecting_rod_node.h
        scripting/include/crankshaft_node.h
        scripting/include/cylinder_bank_node.h
        scripting/include/cylinder_head_node.h
        scripting/include/engine_context.h
        scripting/include/engine_node.h
        scripting/include/engine_sim.h
        scripting/include/exhaust_system_node.h
        scripting/include/intake_node.h
        scripting/include/function_node.h
        scripting/include/ignition_module_node.h
        scripting/include/ignition_wire_node.h
        scripting/include/impulse_response_node.h
        scripting/include/intake_node.h
        scripting/include/language_rules.h
        scripting/include/node.h
        scripting/include/object_reference_node.h
        scripting/include/object_reference_node_output.h
        scripting/include/piranha.h
        scripting/include/piston_node.h
        scripting/include/rod_journal_node.h
        scripting/include/standard_valvetrain_node.h
        scripting/include/transmission_node.h
        scripting/include/valvetrain_node.h
        scripting/include/vtec_valvetrain_node.h
		scripting/include/vehicle_node.h
    )

    target_include_directories(engine-sim-script-interpreter
        PUBLIC dependencies/submodules)

    target_link_libraries(engine-sim-script-interpreter
        PUBLIC csv-io
        PUBLIC piranha)
endif (PIRANHA_ENABLED)

# ========================================================
# ENGINE-SIM HEADLESS BRIDGE (Shared Library for .NET)
# ========================================================
# NOTE: The old C API bridge (engine_sim_bridge.h/cpp) has been removed.
# The new bridge is now at the parent level (../engine-sim-bridge)
# and uses ISimulator interface instead of C functions.

option(BUILD_BRIDGE "Build headless bridge library (.dylib/.so)" OFF)

if (BUILD_BRIDGE)
    add_library(engine-sim-bridge SHARED
        # Bridge source - removed, now using ISimulator at parent level
        # src/engine_sim_bridge.cpp
        # include/engine_sim_bridge.h
    )

    target_link_libraries(engine-sim-bridge
        engine-sim
    )

    if (PIRANHA_ENABLED)
        target_link_libraries(engine-sim-bridge
            engine-sim-script-interpreter
        )
    endif (PIRANHA_ENABLED)

    target_include_directories(engine-sim-bridge
        PUBLIC include
        PUBLIC dependencies/submodules
    )

    # Set library output properties
    set_target_properties(engine-sim-bridge PROPERTIES
        OUTPUT_NAME "enginesim"
        VERSION 1.0.0
        SOVERSION 1
        # PUBLIC_HEADER include/engine_sim_bridge.h  # Removed
    )

    # Platform-specific settings
    if (APPLE)
        set_target_properties(engine-sim-bridge PROPERTIES
            MACOSX_RPATH TRUE
            INSTALL_NAME_DIR "@rpath"
        )
    endif()

    # Install rules
    install(TARGETS engine-sim-bridge
        LIBRARY DESTINATION lib
        # PUBLIC_HEADER DESTINATION include  # Removed
    )
endif()

# ========================================================

if (DISCORD_ENABLED)
    add_library(discord STATIC
        # Source files
        dependencies/discord/Discord.cpp

        # Include files
        dependencies/discord/Discord.h
        dependencies/discord/discord_register.h
        dependencies/discord/discord_rpc.h
    )
endif (DISCORD_ENABLED)

# engine-sim-app (GUI) requires delta-studio (https://github.com/ange-yaghi/delta-studio).
# delta-studio must be built and installed separately — it is NOT a submodule of this repo.
# CI and headless builds target engine-sim + engine-sim-test and skip this target.
# To build the full application locally, install delta-studio so its headers are on your include path.
find_path(DELTA_STUDIO_INCLUDE_DIR
    NAMES delta-studio/include/yds_core.h
    DOC "delta-studio root include directory (https://github.com/ange-yaghi/delta-studio)")
if(NOT DELTA_STUDIO_INCLUDE_DIR AND CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND WIN32)
    message(WARNING
        "delta-studio headers not found -- engine-sim-app (GUI) will not build correctly.\n"
        "Install delta-studio from https://github.com/ange-yaghi/delta-studio and ensure "
        "its parent directory is on your include path.\n"
        "The engine-sim library and engine-sim-test build fine without it.")
endif()

add_executable(engine-sim-app WIN32
    # Source files
    src/main.cpp
    src/engine_sim_application.cpp
    src/geometry_generator.cpp
    src/simulation_object.cpp
    src/piston_object.cpp
    src/connecting_rod_object.cpp
    src/ui_element.cpp
    src/ui_manager.cpp
    src/cylinder_pressure_gauge.cpp
    src/ui_math.cpp
    src/gauge.cpp
    src/crankshaft_object.cpp
    src/cylinder_bank_object.cpp
    src/cylinder_head_object.cpp
    src/ui_button.cpp
    src/ui_utilities.cpp
    src/combustion_chamber_object.cpp
    src/oscilloscope.cpp
    src/shaders.cpp
    src/engine_view.cpp
    src/right_gauge_cluster.cpp
    src/cylinder_temperature_gauge.cpp
    src/labeled_gauge.cpp
    src/throttle_display.cpp
    src/afr_cluster.cpp
    src/fuel_cluster.cpp
    src/oscilloscope_cluster.cpp
    src/performance_cluster.cpp
    src/firing_order_display.cpp
    src/load_simulation_cluster.cpp
    src/mixer_cluster.cpp
    src/info_cluster.cpp

    # Include files
    include/delta.h
    include/dtv.h
    include/engine_sim_application.h
    include/geometry_generator.h
    include/simulation_object.h
    include/piston_object.h
    include/connecting_rod_object.h
    include/ui_element.h
    include/ui_manager.h
    include/cylinder_pressure_gauge.h
    include/ui_math.h
    include/units.h
    include/crankshaft_object.h
    include/cylinder_bank_object.h
    include/cylinder_head_object.h
    include/ui_button.h
    include/ui_utilities.h
    include/combustion_chamber_object.h
    include/oscilloscope.h
    include/shaders.h
    include/engine_view.h
    include/right_gauge_cluster.h
    include/cylinder_temperature_gauge.h
    include/labeled_gauge.h
    include/throttle_display.h
    include/afr_cluster.h
    include/fuel_cluster.h
    include/oscilloscope_cluster.h
    include/performance_cluster.h
    include/firing_order_display.h
    include/load_simulation_cluster.h
    include/mixer_cluster.h
    include/info_cluster.h
)

target_link_libraries(engine-sim-app
    engine-sim
)

if (DTV)
    target_link_libraries(engine-sim-app
        direct-to-video)
endif (DTV)

if (PIRANHA_ENABLED)
    target_link_libraries(engine-sim-app
        engine-sim-script-interpreter)
endif (PIRANHA_ENABLED)

if (DISCORD_ENABLED)
    add_library(discord-rpc STATIC IMPORTED)
    set_property(TARGET discord-rpc PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/dependencies/discord/lib/discord-rpc.lib)
    target_link_libraries(engine-sim-app
    	discord
	discord-rpc)
endif (DISCORD_ENABLED)

target_include_directories(engine-sim-app
    PUBLIC dependencies/submodules)

add_subdirectory(dependencies)

# GTEST
#
# DELEGATION model: engine-sim self-registers its physics safety-net tests
# whether it is built standalone OR pulled in as a subdirectory by a parent
# project (e.g. engine-sim-bridge). The parent project is responsible only
# for ensuring the engine-sim-test binary is built (it un-excludes the target
# from EXCLUDE_FROM_ALL) and for tiering the registered tests by prefix
# (test-core skips them, test-deep runs them). engine-sim owns its own
# registration here — the parent never adds its own add_test() for engine-sim.
enable_testing()

add_executable(engine-sim-test
    # Source files
    test/gas_system_tests.cpp
    test/function_test.cpp
    test/synthesizer_tests.cpp
    test/afterfire_tests.cpp
)

if(NOT MSVC)
    target_compile_options(engine-sim-test PRIVATE
        -Wno-unused-variable
    )
endif()

# Diagnostic output (CSV, .dat) MUST land under the build dir (gitignored),
# never the CWD/repo root. gas_system_tests writes relative-path CSVs; without
# this they land wherever the binary runs (repo root under ctest/coverage).
target_compile_definitions(engine-sim-test PRIVATE
    TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test-output"
)

target_link_libraries(engine-sim-test
    gtest_main
    engine-sim
)

include(GoogleTest)
gtest_discover_tests(engine-sim-test)
