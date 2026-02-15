cmake_minimum_required(VERSION 3.14)
project(NthuRoute VERSION 3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Boost (header-only usage)
find_package(Boost REQUIRED)

# Find zlib (required for gzip file handling)
find_package(ZLIB REQUIRED)

# Third-party sources (warnings suppressed)
set(THIRDPARTY_SOURCES
    src/flute/flute.cpp
)

# Project sources
set(PROJECT_SOURCES
    src/grdb/RoutingComponent.cpp
    src/grdb/RoutingRegion.cpp
    src/grdb/parser.cpp
    src/misc/filehandler.cpp
    src/router/Congestion.cpp
    src/router/Construct_2d_tree.cpp
    src/router/DataDef.cpp
    src/router/Layerassignment.cpp
    src/router/MM_mazeroute.cpp
    src/router/Main.cpp
    src/router/MonotonicRouting.cpp
    src/router/OutputGeneration.cpp
    src/router/Post_processing.cpp
    src/router/Range_router.cpp
    src/router/Route.cpp
    src/router/Route_2pinnets.cpp
    src/router/flute4nthuroute.cpp
    src/router/parameter.cpp
)

add_executable(NthuRoute ${PROJECT_SOURCES} ${THIRDPARTY_SOURCES})

target_include_directories(NthuRoute PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${Boost_INCLUDE_DIRS}
)

# Treat spdlog headers as system headers to suppress warnings
target_include_directories(NthuRoute SYSTEM PRIVATE
    ${CMAKE_SOURCE_DIR}/src/spdlog
)

target_link_libraries(NthuRoute PRIVATE
    Boost::boost
    ZLIB::ZLIB
    pthread
)

# Compiler warnings for project code
target_compile_options(NthuRoute PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wnull-dereference
    -Wformat=2
)

# Suppress warnings for third-party source files
set_source_files_properties(${THIRDPARTY_SOURCES} PROPERTIES
    COMPILE_FLAGS "-w"
)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Install target
install(TARGETS NthuRoute DESTINATION bin)

# Copy data files needed at runtime
configure_file(${CMAKE_SOURCE_DIR}/POWV9.dat ${CMAKE_BINARY_DIR}/POWV9.dat COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/POST9.dat ${CMAKE_BINARY_DIR}/POST9.dat COPYONLY)
