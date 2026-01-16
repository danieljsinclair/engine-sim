add_subdirectory(simple-2d-constraint-solver)

set_property(TARGET simple-2d-constraint-solver PROPERTY FOLDER "scs")
set_property(TARGET simple-2d-constraint-solver-test PROPERTY FOLDER "scs")

add_subdirectory(csv-io)

set_property(TARGET csv-io PROPERTY FOLDER "csv-io")
set_property(TARGET csv-io-main PROPERTY FOLDER "csv-io")
set_property(TARGET csv-io-test PROPERTY FOLDER "csv-io")

if (DTV)
    add_subdirectory(direct-to-video)

    set_property(TARGET direct-to-video PROPERTY FOLDER "dtv")
endif (DTV)

# =========================================================
# Piranha: Detect Homebrew Bison on macOS
# =========================================================
if(APPLE)
    # Find Homebrew Bison 3.2+ (required by Piranha)
    find_program(HOMEBREW_BISON bison
        PATHS
            /opt/homebrew/opt/bison/bin
            /usr/local/opt/bison/bin
        NO_DEFAULT_PATH
    )
    if(HOMEBREW_BISON)
        # Force CMake to use this Bison for Piranha
        set(BISON_EXECUTABLE "${HOMEBREW_BISON}" CACHE FILEPATH "Path to Bison 3.2+" FORCE)
        message(STATUS "Found Homebrew Bison: ${HOMEBREW_BISON}")
    else()
        message(WARNING "Homebrew Bison not found. Piranha may fail to build with system Bison 2.3")
        message(WARNING "Install via: brew install bison")
    endif()
endif()

add_subdirectory(piranha)

set_property(TARGET piranha PROPERTY FOLDER "piranha")
set_property(TARGET piranha_test PROPERTY FOLDER "piranha")
set_property(TARGET piranha_reference_compiler PROPERTY FOLDER "piranha")
