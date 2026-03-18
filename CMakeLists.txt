# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------

find_package(yat CONFIG REQUIRED)

# --------------------------------------------------------------------------
# Collect sources and includes
# --------------------------------------------------------------------------

file(GLOB_RECURSE sources src/*.cpp)
set(includedirs
    include/
    sdk/msvc/include/
)

# --------------------------------------------------------------------------
# Target
# --------------------------------------------------------------------------

add_library(limadhyana ${sources})

target_include_directories(limadhyana
    PUBLIC 
        ${includedirs}
)

target_link_directories(limadhyana
    PRIVATE
        sdk/msvc/lib/x64/
)

target_link_libraries(limadhyana 
    PUBLIC
        limacore
        yat::yat
    PRIVATE
        TUCam
        winmm
)

limatools_set_library_soversion(limadhyana "VERSION")

# --------------------------------------------------------------------------
# Compile definitions
# --------------------------------------------------------------------------

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" DHYANA_VERSION_RAW)
string(STRIP "${DHYANA_VERSION_RAW}" DHYANA_VERSION)

target_compile_definitions(limadhyana
    PUBLIC
        DHYANA_NAME=Dhyana
        DHYANA_VERSION=${DHYANA_VERSION}
        DHYANA_ENABLED
)

message(STATUS "Camera enabled: Dhyana ${DHYANA_VERSION}")

# Windows DLL export
if (WIN32 AND BUILD_SHARED_LIBS)
    target_compile_definitions(limadhyana
        PRIVATE
            LIBDHYANA_EXPORTS
    )
endif()

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------

install(
    TARGETS limadhyana 
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY sdk/msvc/bin/x64/ 
    DESTINATION bin 
    FILES_MATCHING PATTERN "*.dll"
)